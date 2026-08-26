#include "spring_controller/spring_edulite_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

SpringEduliteController::SpringEduliteController()
: Node("spring_controller_node")
{
  auto declare_double_parameter = [this](const std::string & name,
      double default_value) -> double {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.dynamic_typing = true;
      declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
      rclcpp::Parameter param;
      if (get_parameter(name, param)) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          return param.as_double();
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          return static_cast<double>(param.as_int());
        }
      }
      return default_value;
    };

  standby_offset_rad_ = declare_double_parameter("standby_offset_rad", 0.0);
  belt_clearance_ready_travel_rad_ =
    declare_double_parameter("belt_clearance_ready_travel_rad", 3.0);
  pos_tolerance_rad_ =
    declare_double_parameter("position_tolerance_rad", 0.05);
  limit_sw_bit_offset_ =
    declare_parameter<int>("limit_switch_bit_offset", 0);
  fire_increment_rad_ =
    declare_double_parameter("fire_increment_rad", -6.283185307);
  slow_fire_target_pos_rad_ =
    declare_double_parameter("slow_fire_target_position_rad", 13.5);
  slow_fire_base_vel_rad_s_ =
    declare_double_parameter("slow_fire_base_velocity_rad_s", 12.0);
  slow_fire_vel_gain_rad_per_m_ =
    declare_double_parameter("slow_fire_velocity_gain_rad_per_m", 0.0);
  slow_fire_min_vel_rad_s_ =
    declare_double_parameter("slow_fire_min_velocity_rad_s", 1.0);
  slow_fire_max_vel_rad_s_ =
    declare_double_parameter("slow_fire_max_velocity_rad_s", 20.0);
  slow_fire_return_vel_rad_s_ =
    declare_double_parameter("slow_fire_return_velocity_rad_s", 6.0);
  slow_fire_delay_sec_ =
    declare_double_parameter("slow_fire_delay_sec", 0.0);
  slow_fire_settle_timeout_sec_ =
    declare_double_parameter("slow_fire_settle_timeout_sec", 3.0);
  slow_fire_move_spring_ =
    declare_parameter<bool>("slow_fire_move_spring", true);
  slow_fire_arm_only_duration_sec_ =
    declare_double_parameter("slow_fire_arm_only_duration_sec", 0.5);
  homing_vel_rad_s_ =
    declare_double_parameter("homing_velocity_rad_s", 0.5);
  homing_timeout_sec_ = declare_double_parameter("homing_timeout_sec", 30.0);
  motion_timeout_sec_ = declare_double_parameter("motion_timeout_sec", 10.0);
  stopped_vel_threshold_rad_s_ =
    declare_double_parameter("stopped_velocity_threshold_rad_s", 0.05);
  required_stable_fb_count_ =
    declare_parameter<int>("required_stable_feedback_count", 3);
  cmd_vel_timeout_sec_ = declare_double_parameter("cmd_vel_timeout_sec", 0.2);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 10);

  const auto logical_id = declare_parameter<int>("logical_id", 4);
  const auto target_topic =
    declare_parameter<std::string>("target_topic", "/edulite/target");
  const auto state_topic =
    declare_parameter<std::string>("state_topic", "/edulite/state");
  const auto set_position_service = declare_parameter<std::string>(
    "set_position_service", "/edulite/set_position");
  const auto cmd_vel_topic = declare_parameter<std::string>(
    "cmd_vel_topic", "/mecanum/cmd_vel_heading");

  if (belt_clearance_ready_travel_rad_ < 0.0 ||
    slow_fire_delay_sec_ < 0.0 || slow_fire_arm_only_duration_sec_ < 0.0 ||
    logical_id < 0 ||
    logical_id > 65535 || target_topic.empty() || state_topic.empty() ||
    set_position_service.empty() || cmd_vel_topic.empty())
  {
    throw std::runtime_error("Invalid parameter configurations");
  }

  logical_id_ = static_cast<uint16_t>(logical_id);
  const auto command_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();

  fire_req_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos,
    std::bind(
      &SpringEduliteController::fire_request_callback, this,
      std::placeholders::_1));

  slow_fire_req_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/slow_fire_request", command_qos,
    std::bind(
      &SpringEduliteController::slow_fire_request_callback, this,
      std::placeholders::_1));

  e_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", emergency_stop_qos,
    std::bind(
      &SpringEduliteController::emergency_stop_callback, this,
      std::placeholders::_1));

  belt_clearance_req_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/belt_clearance_request",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &SpringEduliteController::belt_clearance_request_callback, this,
      std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, command_qos,
    std::bind(
      &SpringEduliteController::cmd_vel_callback, this,
      std::placeholders::_1));

  limit_sw_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/hardware/limit_switches", command_qos,
    std::bind(
      &SpringEduliteController::limit_switch_callback, this,
      std::placeholders::_1));

  actuator_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
    state_topic, command_qos,
    std::bind(
      &SpringEduliteController::actuator_state_callback, this,
      std::placeholders::_1));

  pos_cmd_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    target_topic, command_qos);

  // 装填・待機完了しているかどうかを見るtopic
  actuator_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/spring/actuator_ready", rclcpp::QoS(1).reliable().transient_local());
  op_state_pub_ =
    create_publisher<robot_msgs::msg::SpringOperationState>(
    "/spring/operation_state",
    rclcpp::QoS(1).reliable().transient_local());

  // spring_controller -> hardware_driver: 原点検出後の現在位置を0
  // radに設定する。
  set_pos_client_ =
    create_client<actuator_msgs::srv::SetPosition>(set_position_service);

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&SpringEduliteController::control_timer_callback, this));

  params_callback_handle_ = add_on_set_parameters_callback(
    std::bind(
      &SpringEduliteController::parameters_callback, this,
      std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "SpringEduliteController initialized. Parameters: "
    "standby_offset_rad=%.3f rad, slow_fire_target_position_rad=%.3f "
    "rad, fire_increment_rad=%.3f rad.",
    standby_offset_rad_, slow_fire_target_pos_rad_,
    fire_increment_rad_);
}

// ----------------------------------------------------------------------------------------
// callback関数一覧
// ----------------------------------------------------------------------------------------
/// @brief ばね発射のリクエスト関数
void SpringEduliteController::fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !fire_req_active_) {
    fire_req_pending_ = true;
  }
  fire_req_active_ = msg->data;
}

/// @brief スロー発射のリクエスト関数
void SpringEduliteController::slow_fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !slow_fire_req_active_) {
    slow_fire_req_pending_ = true;
  }
  slow_fire_req_active_ = msg->data;
}

/// @brief 非常停止を受信する関数
void SpringEduliteController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == e_stop_active_) {
    return;
  }

  e_stop_active_ = msg->data;
  if (!e_stop_active_) {
    resume_after_e_stop_ = true;
    return;
  }

  resume_after_e_stop_ = false;
  // 非常停止だけは制御周期を待たず、受信時に現在位置を保持する
  e_stop_hold_pos_rad_ =
    actuator_pos_received_ ? actuator_pos_rad_ : target_pos_rad_;
  stable_fb_count_ = 0;
  publish_target(e_stop_hold_pos_rad_);

  // 機構の準備ができていないことを送信する
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = false;
  actuator_ready_pub_->publish(ready_msg);
  RCLCPP_WARN(
    get_logger(), "Emergency stop: immediately holding spring at %.3f rad",
    e_stop_hold_pos_rad_);
}

void SpringEduliteController::belt_clearance_request_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  belt_clearance_request_active_ = msg->data;
}

void SpringEduliteController::start_belt_clearance_motion()
{
  is_belt_clearance_active_ = true;
  belt_clearance_return_pos_rad_ = target_pos_rad_;
  belt_clearance_pos_rad_ =
    belt_clearance_return_pos_rad_ - standby_offset_rad_;
  target_pos_rad_ = belt_clearance_pos_rad_;
  state_ = State::MOVING_TO_STANDBY;
  stable_fb_count_ = 0;
  RCLCPP_INFO(
    get_logger(),
    "Retracting spring for belt shot: %.3f -> %.3f rad "
    "(offset: %.3f rad)",
    belt_clearance_return_pos_rad_, belt_clearance_pos_rad_,
    standby_offset_rad_);
}

void SpringEduliteController::finish_belt_clearance_motion()
{
  is_belt_clearance_active_ = false;
  target_pos_rad_ = belt_clearance_return_pos_rad_;
  state_ = State::MOVING_TO_STANDBY;
  stable_fb_count_ = 0;
  RCLCPP_INFO(
    get_logger(), "Returning spring after belt shot: target %.3f rad",
    target_pos_rad_);
}

void SpringEduliteController::cmd_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_forward_vel_m_s_ = std::max(0.0, msg->linear.x);
  last_cmd_vel_time_ = now();
}

void SpringEduliteController::limit_switch_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  limit_sw_active_ =
    ((msg->data >> limit_sw_bit_offset_) & 0x01U) != 0U;
}

void SpringEduliteController::actuator_state_callback(
  const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  if (msg->logical_id != logical_id_) {
    return;
  }

  actuator_state_ = msg->state;
  actuator_ready_ =
    msg->state == actuator_msgs::msg::ActuatorState::STATE_READY;
  position_ref_set_ = msg->position_reference_set;
  actuator_pos_rad_ = msg->position;
  actuator_vel_rad_s_ = msg->velocity;
  actuator_pos_received_ = true;
  new_actuator_fb_ = true;
}

// ------------------------------------------------------------------------------------
// メインタイマーコールバック
// ------------------------------------------------------------------------------------
void SpringEduliteController::control_timer_callback()
{
  // 前のタイマー周期との間でフィードバックが来ているか確認
  const bool has_new_feedback = new_actuator_fb_;
  new_actuator_fb_ = false;

  // eduliteの状態からばね発射機構の状態を更新
  if (actuator_state_ == actuator_msgs::msg::ActuatorState::STATE_ERROR) {
    state_ = State::ERROR;
  } else if (!actuator_pos_received_ || !actuator_ready_) {
    // アクチュエータからデータが来ていない，もしくはREADYが来ていない場合
    if (state_ != State::WAITING_FOR_ACTUATOR_READY) {
      RCLCPP_WARN(
        get_logger(),
        "Spring EduLite is not READY; waiting for driver initialization");
    }
    state_ = State::WAITING_FOR_ACTUATOR_READY;
    homing_required_ = true;
    zero_srv_pending_ = false;
    zero_srv_response_received_ = false;
  } else if (state_ == State::WAITING_FOR_ACTUATOR_READY) {
    // ドライバ初期化完了と、ばね機構のゼロ点取得を別状態として扱う
    target_pos_rad_ = actuator_pos_rad_;
    e_stop_hold_pos_rad_ = actuator_pos_rad_;
    stable_fb_count_ = 0;
    if (position_ref_set_) {
      homing_required_ = false;
      state_ = State::READY;
      RCLCPP_INFO(
        get_logger(), "Spring EduLite READY with a valid zero reference");
    } else {
      state_ = State::WAITING_FOR_HOMING;
      RCLCPP_INFO(
        get_logger(),
        "Spring EduLite READY; waiting to establish the spring zero reference");
    }
  }

  // ゼロ点が取れているかつ，機構のの準備ができているかを送信する
  std_msgs::msg::Bool ready_msg;
  ready_msg.data = actuator_ready_ && position_ref_set_ &&
    state_ == State::READY && !is_belt_clearance_active_ &&
    !e_stop_active_;
  actuator_ready_pub_->publish(ready_msg);

  // driver側からのREADYもしくはERRORが来ている場合は今の状態を送信して終了
  if (state_ == State::WAITING_FOR_ACTUATOR_READY || state_ == State::ERROR) {
    publish_operation_state();
    return;
  }

  // 非常停止中は現在位置を保持し、発射リクエストを拒否する
  if (e_stop_active_) {
    if (fire_req_pending_ || slow_fire_req_pending_) {
      fire_req_pending_ = false;
      slow_fire_req_pending_ = false;
      RCLCPP_WARN(
        get_logger(), "Spring fire request rejected during emergency stop");
    }
    publish_target(e_stop_hold_pos_rad_);
    publish_operation_state();
    return;
  }

  if (resume_after_e_stop_) {
    resume_after_e_stop_ = false;
    homing_start_time_ = now();
    slow_fire_phase_start_time_ = now();
    motion_start_time_ = now();
    stable_fb_count_ = 0;
  }

  // 非常停止解除後にゼロ点設定サービスの結果を反映する
  if (zero_srv_response_received_) {
    zero_srv_response_received_ = false;
    if (!zero_srv_succeeded_) {
      enter_error_with_position_hold(
        actuator_pos_rad_, zero_srv_response_msg_.c_str());
      publish_operation_state();
      return;
    }
    RCLCPP_INFO(get_logger(), "Spring position successfully zeroed to 0.0 rad.");
    homing_required_ = false;
    position_ref_set_ = true;
    target_pos_rad_ = standby_offset_rad_;
    state_ = State::MOVING_TO_STANDBY;
    stable_fb_count_ = 0;
    motion_start_time_ = now();
  }

  // ゼロ点取りの開始
  if (state_ == State::WAITING_FOR_HOMING) {
    state_ = State::HOMING;
    target_pos_rad_ =
      actuator_pos_received_ ? actuator_pos_rad_ : 0.0;
    stable_fb_count_ = 0;
    zero_srv_pending_ = false;
    homing_start_time_ = now();
  }

  // 退避要求が有効な間は、開始可能になるまで毎周期再評価する。
  if (!belt_clearance_request_active_) {
    if (is_belt_clearance_active_) {
      finish_belt_clearance_motion();
      motion_start_time_ = now();
    }
  } else if (!is_belt_clearance_active_ &&
    (state_ == State::READY || state_ == State::MOVING_TO_STANDBY))
  {
    start_belt_clearance_motion();
    motion_start_time_ = now();
  }
  if (state_ == State::READY && !is_belt_clearance_active_ && fire_req_pending_) {
    fire_req_pending_ = false;
    target_pos_rad_ += fire_increment_rad_;
    state_ = State::FIRING;
    stable_fb_count_ = 0;
    motion_start_time_ = now();
    RCLCPP_INFO(
      get_logger(), "Spring firing: target %.3f rad", target_pos_rad_);
  } else if (state_ == State::READY && !is_belt_clearance_active_ &&
    slow_fire_req_pending_)
  {
    slow_fire_req_pending_ = false;
    stable_fb_count_ = 0;
    slow_fire_phase_start_time_ = now();
    state_ = State::SLOW_FIRE_WAITING;
  }

  // 現在の状態に応じて目標値と状態遷移を更新する
  switch (state_) {
    case State::WAITING_FOR_HOMING:
    case State::HOMING:
    case State::WAITING_FOR_STOP:
      if ((now() - homing_start_time_).seconds() >= homing_timeout_sec_) {
        enter_error_with_position_hold(
          actuator_pos_rad_, "Spring homing timed out");
      } else if (state_ == State::HOMING) {
        // リミットスイッチに当たるまでは一定速度で目標値を動かす
        if (limit_sw_active_) {
          state_ = State::WAITING_FOR_STOP;
          stable_fb_count_ = 0;
        } else {
          const double period_sec =
            static_cast<double>(command_period_ms_) / 1000.0;
          target_pos_rad_ -= homing_vel_rad_s_ * period_sec;
        }
      } else if (has_new_feedback && limit_sw_active_ && !zero_srv_pending_) {
        if (std::fabs(actuator_vel_rad_s_) <= stopped_vel_threshold_rad_s_) {
          ++stable_fb_count_;
        } else {
          stable_fb_count_ = 0;
        }
        if (stable_fb_count_ >= required_stable_fb_count_) {
          request_zero_reference();
        }
      }
      break;

    case State::MOVING_TO_STANDBY:
    case State::FIRING:
      if ((now() - motion_start_time_).seconds() >= motion_timeout_sec_) {
        enter_error_with_position_hold(
          actuator_pos_rad_, "Spring motion timed out");
      } else if (has_new_feedback) {
        actuator_msgs::msg::ActuatorState feedback;
        feedback.position = actuator_pos_rad_;
        feedback.velocity = actuator_vel_rad_s_;
        const double clearance_delta_rad =
          belt_clearance_pos_rad_ - belt_clearance_return_pos_rad_;
        const double clearance_direction = clearance_delta_rad < 0.0 ? -1.0 : 1.0;
        const double clearance_travel_rad =
          (actuator_pos_rad_ - belt_clearance_return_pos_rad_) *
          clearance_direction;
        const double required_clearance_travel_rad = std::min(
          belt_clearance_ready_travel_rad_, std::fabs(clearance_delta_rad));
        const bool belt_clearance_reached = is_belt_clearance_active_ &&
          state_ == State::MOVING_TO_STANDBY &&
          clearance_travel_rad >= required_clearance_travel_rad;
        if (belt_clearance_reached || update_settled(feedback)) {
          state_ = State::READY;
          stable_fb_count_ = 0;
        }
      }
      break;

    case State::SLOW_FIRE_WAITING:
      if ((now() - slow_fire_phase_start_time_).seconds() >= slow_fire_delay_sec_) {
        stable_fb_count_ = 0;
        slow_fire_phase_start_time_ = now();
        if (!slow_fire_move_spring_) {
          state_ = State::SLOW_FIRE_ARM_ONLY;
        } else {
          slow_fire_base_rad_ = target_pos_rad_;
          const double stroke_rad = std::max(
            0.0, slow_fire_target_pos_rad_ - standby_offset_rad_);
          slow_fire_peak_rad_ = slow_fire_base_rad_ + stroke_rad;
          state_ = State::SLOW_FIRING_EXTENDING;
        }
      }
      break;
    case State::SLOW_FIRE_ARM_ONLY:
      if ((now() - slow_fire_phase_start_time_).seconds() >=
        slow_fire_arm_only_duration_sec_)
      {
        state_ = State::READY;
      }
      break;

    case State::SLOW_FIRING_EXTENDING:
      {
        const double period_sec = static_cast<double>(command_period_ms_) / 1000.0;
        const bool cmd_vel_fresh = last_cmd_vel_time_.nanoseconds() > 0 &&
          (now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_sec_;
        const double forward_speed =
          cmd_vel_fresh ? cmd_forward_vel_m_s_ : 0.0;
        const double extension_velocity = std::clamp(
          slow_fire_base_vel_rad_s_ -
          forward_speed * slow_fire_vel_gain_rad_per_m_,
          slow_fire_min_vel_rad_s_, slow_fire_max_vel_rad_s_);
        target_pos_rad_ = std::min(
          slow_fire_peak_rad_, target_pos_rad_ + extension_velocity * period_sec);

        const double stroke_rad = std::fabs(slow_fire_peak_rad_ - slow_fire_base_rad_);
        const double expected_duration_sec = slow_fire_min_vel_rad_s_ > 0.0 ?
          stroke_rad / slow_fire_min_vel_rad_s_ : 1.0;
        if ((now() - slow_fire_phase_start_time_).seconds() >=
          expected_duration_sec + slow_fire_settle_timeout_sec_)
        {
          enter_error_with_position_hold(
            actuator_pos_rad_, "Slow fire extension timed out");
        } else if (has_new_feedback &&
          actuator_pos_rad_ >= slow_fire_peak_rad_ - pos_tolerance_rad_ &&
          target_pos_rad_ >= slow_fire_peak_rad_)
        {
          ++stable_fb_count_;
          if (stable_fb_count_ >= required_stable_fb_count_) {
            state_ = State::SLOW_FIRING_RETURNING;
            stable_fb_count_ = 0;
            slow_fire_phase_start_time_ = now();
          }
        } else if (has_new_feedback) {
          stable_fb_count_ = 0;
        }
      }
      break;

    case State::SLOW_FIRING_RETURNING:
      {
        const double period_sec = static_cast<double>(command_period_ms_) / 1000.0;
        target_pos_rad_ = std::max(
          slow_fire_base_rad_,
          target_pos_rad_ - slow_fire_return_vel_rad_s_ * period_sec);
        const double stroke_rad = std::fabs(slow_fire_peak_rad_ - slow_fire_base_rad_);
        const double expected_duration_sec = slow_fire_return_vel_rad_s_ > 0.0 ?
          stroke_rad / slow_fire_return_vel_rad_s_ : 1.0;
        if ((now() - slow_fire_phase_start_time_).seconds() >=
          expected_duration_sec + slow_fire_settle_timeout_sec_)
        {
          enter_error_with_position_hold(
            actuator_pos_rad_, "Slow fire return timed out");
        } else if (has_new_feedback &&
          std::fabs(actuator_pos_rad_ - slow_fire_base_rad_) <=
          pos_tolerance_rad_ &&
          std::fabs(actuator_vel_rad_s_) <= stopped_vel_threshold_rad_s_ &&
          target_pos_rad_ <= slow_fire_base_rad_ + 1e-4)
        {
          ++stable_fb_count_;
          if (stable_fb_count_ >= required_stable_fb_count_) {
            target_pos_rad_ = slow_fire_base_rad_;
            state_ = State::READY;
            stable_fb_count_ = 0;
          }
        } else if (has_new_feedback) {
          stable_fb_count_ = 0;
        }
      }
      break;

    case State::WAITING_FOR_ACTUATOR_READY:
    case State::READY:
    case State::ERROR:
      break;
  }
  publish_target(target_pos_rad_);
  publish_operation_state();
}

// ---------------------------------------------------------------------


/// @brief
void SpringEduliteController::request_zero_reference()
{
  if (zero_srv_pending_ || !set_pos_client_->service_is_ready()) {
    return;
  }

  zero_srv_pending_ = true;
  auto request = std::make_shared<actuator_msgs::srv::SetPosition::Request>();
  request->logical_id = logical_id_;
  request->position = 0.0f;

  set_pos_client_->async_send_request(
    request,
    [this](rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedFuture
    future) {
      const auto response = future.get();
      zero_srv_pending_ = false;
      zero_srv_succeeded_ = response->success;
      zero_srv_response_msg_ = response->success ?
      std::string{} : "Failed to zero spring position: " + response->message;
      zero_srv_response_received_ = true;
    });
}

void SpringEduliteController::enter_error_with_position_hold(
  double current_position_rad, const char * reason)
{
  target_pos_rad_ = current_position_rad;
  stable_fb_count_ = 0;
  state_ = State::ERROR;
  publish_target(target_pos_rad_);
  RCLCPP_ERROR(
    get_logger(), "%s. Holding current position at %.3f rad.",
    reason, target_pos_rad_);
}

bool SpringEduliteController::update_settled(
  const actuator_msgs::msg::ActuatorState & feedback)
{
  const bool settled =
    std::fabs(feedback.position - target_pos_rad_) <=
    pos_tolerance_rad_ &&
    std::fabs(feedback.velocity) <= stopped_vel_threshold_rad_s_;
  stable_fb_count_ = settled ? stable_fb_count_ + 1 : 0;
  return stable_fb_count_ >= required_stable_fb_count_;
}

/// @brief
/// @param target_rad
void SpringEduliteController::publish_target(double target_rad)
{
  actuator_msgs::msg::ActuatorTarget cmd;
  cmd.logical_id = logical_id_;
  cmd.target = static_cast<float>(target_rad);
  pos_cmd_pub_->publish(cmd);
}

/// @brief 現在のばね発射機構の状態を送信する
void SpringEduliteController::publish_operation_state()
{
  uint8_t operation = robot_msgs::msg::SpringOperationState::IDLE;

  if (state_ == State::WAITING_FOR_HOMING || state_ == State::HOMING ||
    state_ == State::WAITING_FOR_STOP)
  {
    operation = robot_msgs::msg::SpringOperationState::HOMING;
  } else if (state_ == State::FIRING) {
    operation = robot_msgs::msg::SpringOperationState::NORMAL_FIRE;
  } else if (state_ == State::SLOW_FIRE_WAITING ||
    state_ == State::SLOW_FIRING_EXTENDING ||
    state_ == State::SLOW_FIRING_RETURNING ||
    state_ == State::SLOW_FIRE_ARM_ONLY)
  {
    operation = robot_msgs::msg::SpringOperationState::SLOW_FIRE;
  } else if (is_belt_clearance_active_ && state_ == State::READY) {
    operation = robot_msgs::msg::SpringOperationState::BELT_CLEARANCE;
  } else if (state_ == State::ERROR) {
    operation = robot_msgs::msg::SpringOperationState::ERROR;
  }
  if (operation == last_pub_op_state_) {
    return;
  }
  robot_msgs::msg::SpringOperationState msg;
  msg.state = operation;
  op_state_pub_->publish(msg);
  last_pub_op_state_ = operation;
}

rcl_interfaces::msg::SetParametersResult
SpringEduliteController::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "standby_offset_rad") {
      double new_standby = 0.0;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        new_standby = param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        new_standby = static_cast<double>(param.as_int());
      } else {
        result.successful = false;
        result.reason = "standby_offset_rad must be a number";
        return result;
      }
      const double standby_delta_rad = new_standby - standby_offset_rad_;
      if (std::fabs(standby_delta_rad) <= 1e-9) {
        continue;
      }

      standby_offset_rad_ = new_standby;
      RCLCPP_INFO(
        get_logger(), "Updated standby_offset_rad to %.3f rad",
        standby_offset_rad_);

      // 累積した発射位置を維持し、待機オフセットの変更量だけを反映する
      if (state_ == State::READY || state_ == State::MOVING_TO_STANDBY) {
        if (is_belt_clearance_active_) {
          belt_clearance_return_pos_rad_ += standby_delta_rad;
          belt_clearance_pos_rad_ =
            belt_clearance_return_pos_rad_ - standby_offset_rad_;
          target_pos_rad_ = belt_clearance_pos_rad_;
        } else {
          target_pos_rad_ += standby_delta_rad;
        }
        stable_fb_count_ = 0;
        state_ = State::MOVING_TO_STANDBY;
        motion_start_time_ = now();
        publish_target(target_pos_rad_);
        RCLCPP_INFO(
          get_logger(), "Applying standby offset delta: target %.3f rad.",
          target_pos_rad_);
      }
    } else if (name == "belt_clearance_ready_travel_rad") {
      double value = 0.0;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(param.as_int());
      } else {
        result.successful = false;
        result.reason = "belt_clearance_ready_travel_rad must be a number";
        return result;
      }
      if (value < 0.0) {
        result.successful = false;
        result.reason = "belt_clearance_ready_travel_rad must be non-negative";
        return result;
      }
      belt_clearance_ready_travel_rad_ = value;
    } else if (name == "position_tolerance_rad") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        pos_tolerance_rad_ = param.as_double();
      }
    } else if (name == "fire_increment_rad") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        fire_increment_rad_ = param.as_double();
      }
    } else if (name == "slow_fire_move_spring") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "slow_fire_move_spring must be a boolean";
        return result;
      }
      slow_fire_move_spring_ = param.as_bool();
    } else if (name == "slow_fire_arm_only_duration_sec") {
      double value = 0.0;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(param.as_int());
      } else {
        result.successful = false;
        result.reason = "slow_fire_arm_only_duration_sec must be a number";
        return result;
      }
      if (value < 0.0) {
        result.successful = false;
        result.reason = "slow_fire_arm_only_duration_sec must be non-negative";
        return result;
      }
      slow_fire_arm_only_duration_sec_ = value;
    } else if (name == "slow_fire_target_position_rad") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_target_pos_rad_ = param.as_double();
      }
    } else if (name == "slow_fire_base_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_base_vel_rad_s_ = param.as_double();
      }
    } else if (name == "slow_fire_velocity_gain_rad_per_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_vel_gain_rad_per_m_ = param.as_double();
      }
    } else if (name == "slow_fire_min_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_min_vel_rad_s_ = param.as_double();
      }
    } else if (name == "slow_fire_max_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_max_vel_rad_s_ = param.as_double();
      }
    } else if (name == "slow_fire_delay_sec") {
      double value = 0.0;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(param.as_int());
      } else {
        result.successful = false;
        result.reason = "slow_fire_delay_sec must be a number";
        return result;
      }
      if (value < 0.0) {
        result.successful = false;
        result.reason = "slow_fire_delay_sec must be non-negative";
        return result;
      }
      slow_fire_delay_sec_ = value;
    } else if (name == "slow_fire_settle_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_settle_timeout_sec_ = param.as_double();
      }
    } else if (name == "slow_fire_return_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_return_vel_rad_s_ = param.as_double();
      }
    } else if (name == "cmd_vel_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        cmd_vel_timeout_sec_ = param.as_double();
      }
    } else if (name == "homing_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        homing_vel_rad_s_ = param.as_double();
      }
    } else if (name == "homing_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        homing_timeout_sec_ = param.as_double();
      }
    } else if (name == "motion_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        motion_timeout_sec_ = param.as_double();
      }
    } else if (name == "stopped_velocity_threshold_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        stopped_vel_threshold_rad_s_ = param.as_double();
      }
    } else if (name == "required_stable_feedback_count") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        required_stable_fb_count_ = static_cast<int>(param.as_int());
      }
    }
  }

  return result;
}
