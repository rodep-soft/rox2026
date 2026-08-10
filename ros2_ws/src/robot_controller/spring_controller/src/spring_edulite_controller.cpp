#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

SpringEduliteController::SpringEduliteController()
: Node("spring_controller_node")
{
  limit_switch_bit_offset_ = declare_parameter<int>("limit_switch_bit_offset", 0);
  fire_step_rad_ = declare_parameter<double>("fire_increment_rad", -6.283185307);
  homing_vel_rad_s_ = declare_parameter<double>("homing_velocity_rad_s", 0.5);
  homing_timeout_sec_ = declare_parameter<double>("homing_timeout_sec", 30.0);
  stopped_vel_threshold_rad_s_ =
    declare_parameter<double>("zeroing_velocity_threshold_rad_s", 0.05);
  required_stable_feedback_count_ =
    declare_parameter<int>("zeroing_required_stable_feedback_count", 3);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 10);

  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto logical_id = declare_parameter<int>("logical_id", 4);
  const auto target_topic =
    declare_parameter<std::string>("target_topic", "/edulite/target");
  const auto state_topic =
    declare_parameter<std::string>("state_topic", "/edulite/state");
  const auto set_position_service = declare_parameter<std::string>(
    "set_position_service", "/edulite/set_position");

  if (logical_id < 0 || logical_id > 65535) {
    throw std::runtime_error("logical_id must be in [0, 65535]");
  }
  if (target_topic.empty() || state_topic.empty() || set_position_service.empty()) {
    throw std::runtime_error("ROS interface names must not be empty");
  }
  if (limit_switch_bit_offset_ < 0 || limit_switch_bit_offset_ >= 8) {
    throw std::runtime_error("limit_switch_bit_offset must be in [0, 7]");
  }
  if (!std::isfinite(fire_step_rad_) || fire_step_rad_ >= 0.0) {
    throw std::runtime_error("fire_increment_rad must be negative");
  }
  if (!std::isfinite(homing_vel_rad_s_) || homing_vel_rad_s_ <= 0.0) {
    throw std::runtime_error("homing_velocity_rad_s must be positive");
  }
  if (!std::isfinite(homing_timeout_sec_) || homing_timeout_sec_ <= 0.0) {
    throw std::runtime_error("homing_timeout_sec must be positive");
  }
  if (!std::isfinite(stopped_vel_threshold_rad_s_) ||
    stopped_vel_threshold_rad_s_ < 0.0 || required_stable_feedback_count_ <= 0)
  {
    throw std::runtime_error("zeroing stability parameters are invalid");
  }
  if (command_period_ms_ <= 0 || qos_depth <= 0) {
    throw std::runtime_error("command period and QoS depth must be positive");
  }

  logical_id_ = static_cast<uint16_t>(logical_id);
  const auto command_qos = rclcpp::QoS(qos_depth);
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();

  // joy_controller -> spring_controller: trueの立ち上がりで1回発射する。
  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos,
    std::bind(&SpringEduliteController::fire_request_callback, this,
    std::placeholders::_1));

  // joy_controller -> spring_controller: 非常停止中の原点復帰と発射を禁止する。
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", emergency_stop_qos,
    std::bind(&SpringEduliteController::emergency_stop_callback, this,
    std::placeholders::_1));

  // stm32_driver -> spring_controller: 指定bitを原点リミットとして使用する。
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/limit_switchs", command_qos,
    std::bind(&SpringEduliteController::limit_switch_callback, this,
    std::placeholders::_1));

  // hardware_driver -> spring_controller: 接続、原点、速度状態を監視する。
  actuator_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
    state_topic, command_qos,
    std::bind(&SpringEduliteController::actuator_state_callback, this,
    std::placeholders::_1));

  // spring_controller -> hardware_driver: スプリングの累積目標角度[rad]。
  position_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    target_topic, command_qos);

  // spring_controller -> hardware_driver: 原点検出後の現在位置を0 radに設定する。
  set_position_client_ =
    create_client<actuator_msgs::srv::SetPosition>(set_position_service);

  homing_started_at_ = now();
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&SpringEduliteController::control_timer_callback, this));
}

// /spring/fire_requestの立ち上がりを受信したとき、READYかつ非常停止解除中なら
// /edulite/targetへ次の累積目標角度を送信する。それ以外では目標を変更しない。
void SpringEduliteController::fire_request_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool fire_requested = msg->data && !fire_request_active_;
  fire_request_active_ = msg->data;
  if (!fire_requested) {
    return;
  }

  if (emergency_stop_active_ || state_ != ControlState::READY ||
    !position_reference_set_)
  {
    RCLCPP_WARN(get_logger(),
      "Spring fire rejected: homing incomplete or emergency stop active.");
    return;
  }

  position_target_rad_ += fire_step_rad_;
  publish_target();
  RCLCPP_INFO(get_logger(), "Spring target advanced by %.6f rad to %.6f rad.",
    fire_step_rad_, position_target_rad_);
}

// /emergency_stopの状態を保存する。目標角度は変更せず、停止中は原点復帰と発射を止める。
void SpringEduliteController::emergency_stop_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
}

// /limit_switchsから設定bitだけを取り出して、原点リミット状態を更新する。
void SpringEduliteController::limit_switch_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  limit_switch_active_ = ((msg->data >> limit_switch_bit_offset_) & 0x01U) != 0U;
}

// /edulite/stateから対象motorだけを処理する。接続または原点を失った場合は
// HOMINGへ戻し、ZEROING中に停止を確認できた場合は原点設定serviceを要求する。
void SpringEduliteController::actuator_state_callback(const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  if (msg->logical_id != logical_id_) {
    return;
  }

  const bool state_ready = msg->state == actuator_msgs::msg::ActuatorState::STATE_READY;
  if (!state_ready) {
    if (driver_ready_ || position_reference_set_) {
      RCLCPP_WARN(get_logger(), "Spring EduLite disconnected. Clearing target and homing state.");
    }
    driver_ready_ = false;
    position_reference_set_ = false;
    zeroing_request_pending_ = false;
    position_target_rad_ = 0.0;
    state_ = ControlState::HOMING;
    return;
  }

  if (!driver_ready_) {
    driver_ready_ = true;
    if (!msg->position_reference_set) {
      reset_for_homing();
      RCLCPP_WARN(get_logger(),
        "Spring position reference is not set. Starting homing.");
    }
  }

  if (!msg->position_reference_set) {
    if (position_reference_set_) {
      reset_for_homing();
      RCLCPP_WARN(get_logger(),
        "Spring position reference was lost. Target reset and homing restarted.");
    }
    position_reference_set_ = false;

    if (state_ == ControlState::ZEROING && !zeroing_request_pending_) {
      if (std::fabs(msg->velocity) <= stopped_vel_threshold_rad_s_) {
        ++stable_feedback_count_;
      } else {
        stable_feedback_count_ = 0;
      }
      if (stable_feedback_count_ >= required_stable_feedback_count_) {
        request_zero_reference();
      }
    }
    return;
  }

  const bool zeroing_completed = zeroing_request_pending_;
  position_reference_set_ = true;
  zeroing_request_pending_ = false;
  state_ = ControlState::READY;
  if (zeroing_completed) {
    position_target_rad_ = 0.0;
    publish_target();
    RCLCPP_INFO(get_logger(),
      "Spring homing completed after feedback confirmation. Target reset to 0 rad.");
  }
}

// 接続完了後、HOMINGでは一定速度で目標を動かす。リミット検出後はZEROINGで
// 停止を待ち、READYとERRORでは現在目標を保持して /edulite/targetへ送信する。
void SpringEduliteController::control_timer_callback()
{
  if (!driver_ready_ || zeroing_request_pending_) {
    return;
  }

  const bool homing_in_progress = state_ == ControlState::HOMING || state_ == ControlState::ZEROING;
  if (homing_in_progress && (now() - homing_started_at_).seconds() >= homing_timeout_sec_)
  {
    state_ = ControlState::ERROR;
    RCLCPP_ERROR(get_logger(),
      "Spring homing timed out. Target is held at %.6f rad.",
      position_target_rad_);
    return;
  }

  if (state_ == ControlState::HOMING) {
    if (limit_switch_active_) {
      state_ = ControlState::ZEROING;
      stable_feedback_count_ = 0;
      RCLCPP_INFO(get_logger(),
        "Spring limit detected. Holding target until motion settles.");
    } else if (!emergency_stop_active_) {
      const double period_sec = static_cast<double>(command_period_ms_) / 1000.0;
      position_target_rad_ -= homing_vel_rad_s_ * period_sec;
    }
  }
  publish_target();
}

void SpringEduliteController::reset_for_homing()
{
  position_target_rad_ = 0.0;
  position_reference_set_ = false;
  zeroing_request_pending_ = false;
  stable_feedback_count_ = 0;
  state_ = ControlState::HOMING;
  homing_started_at_ = now();
}

void SpringEduliteController::request_zero_reference()
{
  if (zeroing_request_pending_ || !set_position_client_->service_is_ready()) {
    return;
  }

  zeroing_request_pending_ = true;
  auto request = std::make_shared<actuator_msgs::srv::SetPosition::Request>();
  request->logical_id = logical_id_;
  request->position = 0.0F;

  set_position_client_->async_send_request(
    request,
    [this](rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedFuture future) {
      const auto response = future.get();
      if (!response->success) {
        zeroing_request_pending_ = false;
        state_ = ControlState::ERROR;
        RCLCPP_ERROR(get_logger(), "Failed to zero spring position: %s",
          response->message.c_str());
        return;
      }

      RCLCPP_INFO(get_logger(),
        "Spring zero reference updated from the latest EduLite feedback.");
    });
}

void SpringEduliteController::publish_target()
{
  actuator_msgs::msg::ActuatorTarget command;
  command.logical_id = logical_id_;
  command.target = static_cast<float>(position_target_rad_);
  position_command_pub_->publish(command);
}
