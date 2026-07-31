#include "dribble_position_controller/dribble_position_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

DribblePositionController::DribblePositionController()
    : Node("dribble_position_controller") {
  declare_parameters();
  get_parameters();
  if (command_period_ms_ <= 0) {
    RCLCPP_WARN(
        get_logger(), "command_period_ms must be greater than zero: %d. Using 20 ms.",
        command_period_ms_);
    command_period_ms_ = 20;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive: %d. Using 1.",
                qos_depth_);
    qos_depth_ = 1;
  }
  if (!std::isfinite(position_tolerance_rad_) ||
      position_tolerance_rad_ <= 0.0) {
    RCLCPP_WARN(
        get_logger(),
        "position_tolerance_rad must be finite and greater than zero: %.6f. Using 0.020000 rad.",
        position_tolerance_rad_);
    position_tolerance_rad_ = 0.02;
  }

  // EduLite 05 driverへ送る目標位置 [rad]。
  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
      dribble_position_command_topic_, rclcpp::QoS(qos_depth_));

  // joy_controllerからの手動位置選択。std_msgs/msg/UInt8。
  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      dribble_position_mode_topic_, rclcpp::QoS(qos_depth_),
      std::bind(&DribblePositionController::position_mode_callback, this,
                std::placeholders::_1));

  // joy_controllerからの運転モード。STOP時の停止とBELT_ONLY時のOPEN移動を判断する。
  // std_msgs/msg/UInt8、状態topicのためtransient local QoSを使う。
  operation_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      operation_mode_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&DribblePositionController::operation_mode_callback, this,
                std::placeholders::_1));

  // belt_dribble_controllerからのshot cycle開始要求。std_msgs/msg/Bool。
  shot_cycle_start_sub_ = create_subscription<std_msgs::msg::Bool>(
      shot_cycle_start_topic_, rclcpp::QoS(qos_depth_),
      std::bind(&DribblePositionController::shot_cycle_start_callback, this,
                std::placeholders::_1));

  // EduLite 05 driverから受ける現在位置 [rad]。std_msgs/msg/Float32。
  position_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
      dribble_position_feedback_topic_, rclcpp::QoS(qos_depth_),
      std::bind(&DribblePositionController::position_feedback_callback, this,
                std::placeholders::_1));

  // joy_controllerからの非常停止。std_msgs/msg/Bool、状態topicのためtransient local QoSを使う。
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&DribblePositionController::emergency_stop_callback, this,
                std::placeholders::_1));

  // joy_controllerへshot cycle実行中・完了を通知する。どちらもstd_msgs/msg/Bool。
  shot_cycle_running_pub_ = create_publisher<std_msgs::msg::Bool>(
      shot_cycle_running_topic_, rclcpp::QoS(qos_depth_));
  shot_cycle_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
      shot_cycle_complete_topic_, rclcpp::QoS(qos_depth_));
  command_timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&DribblePositionController::watchdog_callback, this));
  set_target_position(dribble_position_rad_, State::IDLE);
}

void DribblePositionController::declare_parameters() {
  declare_parameter<std::string>("dribble_position_command_topic",
                                 "/dribble/position_command");
  declare_parameter<std::string>("dribble_position_feedback_topic",
                                 "/dribble/position_feedback");
  declare_parameter<std::string>("dribble_position_mode_topic",
                                 "/dribble/position_mode");
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("shot_cycle_start_topic", "/shot_cycle/start");
  declare_parameter<std::string>("shot_cycle_running_topic",
                                 "/shot_cycle/running");
  declare_parameter<std::string>("shot_cycle_complete_topic",
                                 "/shot_cycle/complete");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  declare_parameter<double>("dribble_position_rad", 0.0);
  declare_parameter<double>("intake_position_rad", 1.5);
  declare_parameter<double>("shoot_position_rad", 2.0);
  declare_parameter<double>("open_position_rad", -1.3);
  declare_parameter<double>("position_tolerance_rad", 0.02);
  declare_parameter<double>("shoot_to_dribble_delay_sec", 1.0);
  declare_parameter<double>("move_timeout_sec", 3.0);
  declare_parameter<double>("feedback_timeout_sec", 0.5);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
}

void DribblePositionController::get_parameters() {
  get_parameter("dribble_position_command_topic",
                dribble_position_command_topic_);
  get_parameter("dribble_position_feedback_topic",
                dribble_position_feedback_topic_);
  get_parameter("dribble_position_mode_topic", dribble_position_mode_topic_);
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("shot_cycle_start_topic", shot_cycle_start_topic_);
  get_parameter("shot_cycle_running_topic", shot_cycle_running_topic_);
  get_parameter("shot_cycle_complete_topic", shot_cycle_complete_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("intake_position_rad", intake_position_rad_);
  get_parameter("shoot_position_rad", shoot_position_rad_);
  get_parameter("open_position_rad", open_position_rad_);
  get_parameter("position_tolerance_rad", position_tolerance_rad_);
  get_parameter("shoot_to_dribble_delay_sec", shoot_to_dribble_delay_sec_);
  get_parameter("move_timeout_sec", move_timeout_sec_);
  get_parameter("feedback_timeout_sec", feedback_timeout_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void DribblePositionController::position_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (!manual_position_allowed()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring dribble position command while moving or stopped");
    return;
  }

  if (msg->data > static_cast<uint8_t>(Position::OPEN)) {
    return;
  }

  const auto position = static_cast<Position>(msg->data);
  switch (position) {
    case Position::DRIBBLE:
      set_target_position(dribble_position_rad_, State::MANUAL_MOVE);
      break;
    case Position::INTAKE:
      set_target_position(intake_position_rad_, State::MANUAL_MOVE);
      break;
    case Position::SHOOT:
      set_target_position(shoot_position_rad_, State::MANUAL_MOVE);
      break;
    case Position::OPEN:
      set_target_position(open_position_rad_, State::MANUAL_MOVE);
      break;
  }
}

void DribblePositionController::shot_cycle_start_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || emergency_stop_active_ ||
      operation_mode_ != OperationMode::SHOT_CYCLE || state_ != State::IDLE ||
      shot_cycle_running_) {
    return;
  }
  shot_cycle_running_ = true;
  publish_shot_cycle_running(true);
  set_target_position(intake_position_rad_, State::INTAKE);
}

void DribblePositionController::operation_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  const auto previous_mode = operation_mode_;
  operation_mode_ = msg->data <= static_cast<uint8_t>(OperationMode::BELT_ONLY)
                        ? static_cast<OperationMode>(msg->data)
                        : OperationMode::STOP;
  if (operation_mode_ == OperationMode::STOP) {
    stop_shot_cycle();
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  } else if (operation_mode_ == OperationMode::BELT_ONLY) {
    stop_shot_cycle();
    set_target_position(open_position_rad_, State::MANUAL_MOVE);
  } else if (previous_mode == OperationMode::BELT_ONLY &&
             operation_mode_ == OperationMode::DRIVE) {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::position_feedback_callback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  if (!std::isfinite(msg->data) || state_ == State::IDLE) {
    return;
  }
  last_feedback_time_ = now();
  if (std::abs(static_cast<double>(msg->data) - target_position_rad_) >
      position_tolerance_rad_) {
    return;
  }
  if (state_ == State::INTAKE) {
    set_target_position(shoot_position_rad_, State::SHOOT);
  } else if (state_ == State::SHOOT) {
    state_ = State::HOLD_SHOOT;
    phase_start_time_ = now();
  } else if (state_ == State::RETURN_TO_DRIBBLE ||
             state_ == State::MANUAL_MOVE) {
    state_ = State::IDLE;
    if (shot_cycle_running_) {
      stop_shot_cycle();
      publish_shot_cycle_complete();
    }
  }
}

void DribblePositionController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    stop_shot_cycle();
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::watchdog_callback() {
  if (state_ == State::IDLE) {
    return;
  }

  // 位置指令を送るたびにEduLite
  // 05からfeedbackが返るため、到達するまで同じ目標を送る。
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(target_position_rad_);
  position_command_pub_->publish(command);

  const auto current_time = now();
  if ((current_time - last_feedback_time_).seconds() > feedback_timeout_sec_ ||
      (state_ != State::HOLD_SHOOT &&
       (current_time - phase_start_time_).seconds() > move_timeout_sec_)) {
    stop_shot_cycle();
    if (state_ == State::RETURN_TO_DRIBBLE) {
      state_ = State::IDLE;
      return;
    }
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
    return;
  }
  if (state_ == State::HOLD_SHOOT &&
      (current_time - phase_start_time_).seconds() >=
          shoot_to_dribble_delay_sec_) {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

bool DribblePositionController::manual_position_allowed() const {
  return !emergency_stop_active_ && state_ == State::IDLE &&
         !shot_cycle_running_ &&
         (operation_mode_ == OperationMode::DRIVE ||
          operation_mode_ == OperationMode::SHOT_CYCLE);
}

void DribblePositionController::stop_shot_cycle() {
  if (!shot_cycle_running_) {
    return;
  }
  shot_cycle_running_ = false;
  publish_shot_cycle_running(false);
}

void DribblePositionController::publish_shot_cycle_running(bool running) {
  std_msgs::msg::Bool message;
  message.data = running;
  shot_cycle_running_pub_->publish(message);
}

void DribblePositionController::publish_shot_cycle_complete() {
  std_msgs::msg::Bool message;
  message.data = true;
  shot_cycle_complete_pub_->publish(message);
}

void DribblePositionController::set_target_position(double position_rad,
                                                    State state) {
  target_position_rad_ = position_rad;
  state_ = state;
  phase_start_time_ = now();
  last_feedback_time_ = phase_start_time_;
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_pub_->publish(command);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblePositionController>());
  rclcpp::shutdown();
  return 0;
}
