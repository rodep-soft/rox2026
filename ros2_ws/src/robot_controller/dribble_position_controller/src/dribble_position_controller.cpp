#include "dribble_position_controller/dribble_position_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

DribblePositionController::DribblePositionController()
: Node("dribble_position_controller")
{
  declare_parameters();
  get_parameters();
  if (command_period_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "command_period_ms must be greater than zero: %d. Using 20 ms.",
      command_period_ms_);
    command_period_ms_ = 20;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "qos_depth must be positive: %d. Using 1.",
      qos_depth_);
    qos_depth_ = 1;
  }
  if (!std::isfinite(position_tolerance_rad_) || position_tolerance_rad_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "position_tolerance_rad must be finite and greater than zero: "
      "%.6f. Using 0.020000 rad.",
      position_tolerance_rad_);
    position_tolerance_rad_ = 0.02;
  }
  validate_parameters();

  // EduLite 05 driverへ送る目標位置 [rad]。
  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    "/dribble/position_command", rclcpp::QoS(qos_depth_));

  // joy_controllerからの手動位置選択。std_msgs/msg/UInt8。
  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/dribble/position_mode", rclcpp::QoS(qos_depth_),
    std::bind(
      &DribblePositionController::position_mode_callback, this,
      std::placeholders::_1));

  // joy_controllerからの運転モード。STOP時の停止とBELT_ONLY時のOPEN移動を判断する。
  // std_msgs/msg/UInt8、状態topicのためtransient local QoSを使う。
  operation_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &DribblePositionController::operation_mode_callback, this,
      std::placeholders::_1));

  // belt_dribble_controllerからのshot cycle開始要求。std_msgs/msg/Bool。
  shot_cycle_start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/shot_cycle/start", rclcpp::QoS(qos_depth_),
    std::bind(
      &DribblePositionController::shot_cycle_start_callback, this,
      std::placeholders::_1));

  // EduLite 05 driverから受ける現在位置 [rad]。std_msgs/msg/Float32。
  position_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/dribble/position_feedback", rclcpp::QoS(qos_depth_),
    std::bind(
      &DribblePositionController::position_feedback_callback, this,
      std::placeholders::_1));

  // joy_controllerからの非常停止。std_msgs/msg/Bool、状態topicのためtransient
  // local QoSを使う。
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &DribblePositionController::emergency_stop_callback, this,
      std::placeholders::_1));

  // joy_controllerへshot
  // cycle実行中・完了を通知する。どちらもstd_msgs/msg/Bool。
  shot_cycle_running_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/shot_cycle/running", rclcpp::QoS(qos_depth_));
  shot_cycle_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/shot_cycle/complete", rclcpp::QoS(qos_depth_));
  command_timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribblePositionController::watchdog_callback, this));
  if (configuration_valid_) {
    set_target_position(dribble_position_rad_, State::IDLE);
  }
}

void DribblePositionController::declare_parameters()
{
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

void DribblePositionController::get_parameters()
{
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

void DribblePositionController::validate_parameters()
{
  constexpr double edulite_position_limit_rad = 3.14159265358979323846;
  if (!std::isfinite(dribble_position_rad_) ||
    !std::isfinite(intake_position_rad_) ||
    !std::isfinite(shoot_position_rad_) || !std::isfinite(open_position_rad_))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Position parameters must be finite: dribble=%.6f, "
      "intake=%.6f, shoot=%.6f, open=%.6f",
      dribble_position_rad_, intake_position_rad_,
      shoot_position_rad_, open_position_rad_);
    configuration_valid_ = false;
  }
  if (std::abs(dribble_position_rad_) > edulite_position_limit_rad ||
    std::abs(intake_position_rad_) > edulite_position_limit_rad ||
    std::abs(shoot_position_rad_) > edulite_position_limit_rad ||
    std::abs(open_position_rad_) > edulite_position_limit_rad)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Position parameters must be in [-pi, pi] rad: dribble=%.6f, "
      "intake=%.6f, shoot=%.6f, open=%.6f",
      dribble_position_rad_, intake_position_rad_,
      shoot_position_rad_, open_position_rad_);
    configuration_valid_ = false;
  }
  if (!std::isfinite(shoot_to_dribble_delay_sec_) ||
    shoot_to_dribble_delay_sec_ < 0.0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "shoot_to_dribble_delay_sec must be finite and zero or greater: %.3f",
      shoot_to_dribble_delay_sec_);
    configuration_valid_ = false;
  }
  if (!std::isfinite(move_timeout_sec_) || move_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "move_timeout_sec must be finite and greater than zero: %.3f",
      move_timeout_sec_);
    configuration_valid_ = false;
  }
  if (!std::isfinite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "feedback_timeout_sec must be finite and greater than zero: %.3f",
      feedback_timeout_sec_);
    configuration_valid_ = false;
  }
}

void DribblePositionController::position_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  // /dribble/position_mode受信時に、許可された手動位置へ移動指令をpublishする。
  if (!configuration_valid_) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring dribble position command: controller configuration "
      "is invalid.");
    return;
  }
  if (!manual_position_allowed()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring dribble position command: emergency_stop=%s, state=%s, "
      "shot_cycle_running=%s, operation_mode=%u",
      emergency_stop_active_ ? "ON" : "OFF", state_name(state_),
      shot_cycle_running_ ? "true" : "false",
      static_cast<uint8_t>(operation_mode_));
    return;
  }

  if (msg->data > static_cast<uint8_t>(Position::OPEN)) {
    RCLCPP_WARN(
      get_logger(), "Ignoring invalid dribble position mode: %u.",
      msg->data);
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

void DribblePositionController::operation_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  // /operation_mode受信時に停止・BELT_ONLYに応じて位置遷移し、必要ならshot
  // cycle状態をpublishする。
  const auto previous_mode = operation_mode_;
  if (msg->data <= static_cast<uint8_t>(OperationMode::BELT_ONLY)) {
    operation_mode_ = static_cast<OperationMode>(msg->data);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Invalid operation mode received: %u. Treating as STOP.",
      msg->data);
    operation_mode_ = OperationMode::STOP;
  }
  if (operation_mode_ == OperationMode::STOP) {
    stop_shot_cycle();
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  } else if (operation_mode_ == OperationMode::BELT_ONLY) {
    stop_shot_cycle();
    set_target_position(open_position_rad_, State::MANUAL_MOVE);
  } else if (previous_mode == OperationMode::BELT_ONLY &&
    operation_mode_ == OperationMode::DRIVE)
  {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::shot_cycle_start_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  // /shot_cycle/startのtrue受信時に、条件を満たせばINTAKEへ遷移し実行中状態をpublishする。
  if (!msg->data) {
    return;
  }
  if (!configuration_valid_ || emergency_stop_active_ ||
    operation_mode_ != OperationMode::SHOT_CYCLE || state_ != State::IDLE ||
    shot_cycle_running_)
  {
    log_shot_cycle_start_rejection();
    return;
  }
  shot_cycle_running_ = true;
  publish_shot_cycle_running(true);
  set_target_position(intake_position_rad_, State::INTAKE);
}

void DribblePositionController::position_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  // /dribble/position_feedback受信時に到達を判定し、shot
  // cycleの次位置または完了通知へ進める。
  if (!std::isfinite(msg->data) || state_ == State::IDLE) {
    return;
  }
  last_feedback_time_ = now();
  last_feedback_position_rad_ = msg->data;
  has_position_feedback_ = true;
  if (std::abs(static_cast<double>(msg->data) - target_position_rad_) >
    position_tolerance_rad_)
  {
    return;
  }
  if (state_ == State::INTAKE) {
    set_target_position(shoot_position_rad_, State::SHOOT);
  } else if (state_ == State::SHOOT) {
    state_ = State::HOLD_SHOOT;
    phase_start_time_ = now();
  } else if (state_ == State::RETURN_TO_DRIBBLE || state_ == State::MANUAL_MOVE) {
    state_ = State::IDLE;
    if (shot_cycle_running_) {
      stop_shot_cycle();
      publish_shot_cycle_complete();
    }
  }
}

void DribblePositionController::emergency_stop_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  // /emergency_stop受信時にshot
  // cycleを中断し、dribble位置へ戻す指令をpublishする。
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    RCLCPP_WARN(
      get_logger(),
      "Emergency stop: interrupting state=%s and returning to "
      "dribble position.",
      state_name(state_));
    stop_shot_cycle();
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::watchdog_callback()
{
  // 設定周期で/dribble/position_commandを再送し、feedbackとタイムアウトを監視する。
  if (state_ == State::IDLE) {
    return;
  }

  // 位置指令を送るたびにEduLite
  // 05からfeedbackが返るため、到達するまで同じ目標を送る。
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(target_position_rad_);
  position_command_pub_->publish(command);

  const auto current_time = now();
  const double feedback_elapsed_sec =
    (current_time - last_feedback_time_).seconds();
  const double move_elapsed_sec = (current_time - phase_start_time_).seconds();
  const bool feedback_timed_out = feedback_elapsed_sec > feedback_timeout_sec_;
  const bool move_timed_out =
    state_ != State::HOLD_SHOOT && move_elapsed_sec > move_timeout_sec_;
  if (feedback_timed_out || move_timed_out) {
    stop_shot_cycle();
    if (state_ == State::RETURN_TO_DRIBBLE) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to return to dribble position: state=%s, target=%.6f rad, "
        "feedback_timeout=%s, move_timeout=%s. Shot cycle completion is not "
        "published.",
        state_name(state_), target_position_rad_,
        feedback_timed_out ? "true" : "false",
        move_timed_out ? "true" : "false");
      state_ = State::IDLE;
      return;
    }
    if (feedback_timed_out) {
      RCLCPP_WARN(
        get_logger(),
        "Position feedback timed out after %.3f s: state=%s, "
        "target=%.6f rad. "
        "Returning to dribble position.",
        feedback_elapsed_sec, state_name(state_),
        target_position_rad_);
    }
    if (move_timed_out) {
      RCLCPP_WARN(
        get_logger(),
        "Position move timed out after %.3f s: state=%s, target=%.6f rad, "
        "last_feedback=%.6f rad. Returning to dribble position.",
        move_elapsed_sec, state_name(state_), target_position_rad_,
        last_feedback_position_rad_);
    }
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
    return;
  }
  if (state_ != State::HOLD_SHOOT && has_position_feedback_ &&
    move_elapsed_sec >= move_timeout_sec_ * 0.5 &&
    std::abs(last_feedback_position_rad_ - target_position_rad_) >
    position_tolerance_rad_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Position is not converging: state=%s, target=%.6f "
      "rad, current=%.6f rad, "
      "error=%.6f rad, tolerance=%.6f rad, elapsed=%.3f s",
      state_name(state_), target_position_rad_,
      last_feedback_position_rad_,
      last_feedback_position_rad_ - target_position_rad_,
      position_tolerance_rad_, move_elapsed_sec);
  }
  if (state_ == State::HOLD_SHOOT &&
    (current_time - phase_start_time_).seconds() >=
    shoot_to_dribble_delay_sec_)
  {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::set_target_position(
  double position_rad,
  State state)
{
  if (!std::isfinite(position_rad)) {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to publish a non-finite position command: %.6f rad.",
      position_rad);
    return;
  }
  if (!configuration_valid_) {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to publish position command: controller "
      "configuration is invalid.");
    return;
  }
  target_position_rad_ = position_rad;
  state_ = state;
  phase_start_time_ = now();
  last_feedback_time_ = phase_start_time_;
  has_position_feedback_ = false;
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_pub_->publish(command);
  RCLCPP_INFO(
    get_logger(),
    "Position command published: state=%s, target=%.6f rad",
    state_name(state_), target_position_rad_);
}

void DribblePositionController::stop_shot_cycle()
{
  if (!shot_cycle_running_) {
    return;
  }
  shot_cycle_running_ = false;
  publish_shot_cycle_running(false);
}

void DribblePositionController::publish_shot_cycle_running(bool running)
{
  std_msgs::msg::Bool message;
  message.data = running;
  shot_cycle_running_pub_->publish(message);
}

void DribblePositionController::publish_shot_cycle_complete()
{
  std_msgs::msg::Bool message;
  message.data = true;
  shot_cycle_complete_pub_->publish(message);
}

bool DribblePositionController::manual_position_allowed() const
{
  if (emergency_stop_active_) {
    return false;
  }
  if (state_ != State::IDLE) {
    return false;
  }
  if (shot_cycle_running_) {
    return false;
  }
  if (operation_mode_ != OperationMode::DRIVE &&
    operation_mode_ != OperationMode::SHOT_CYCLE)
  {
    return false;
  }
  return true;
}

const char * DribblePositionController::state_name(State state) const
{
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::MANUAL_MOVE:
      return "MANUAL_MOVE";
    case State::INTAKE:
      return "INTAKE";
    case State::SHOOT:
      return "SHOOT";
    case State::HOLD_SHOOT:
      return "HOLD_SHOOT";
    case State::RETURN_TO_DRIBBLE:
      return "RETURN_TO_DRIBBLE";
  }
  return "UNKNOWN";
}

void DribblePositionController::log_shot_cycle_start_rejection() const
{
  if (!configuration_valid_) {
    RCLCPP_WARN(
      get_logger(),
      "Shot cycle rejected: controller configuration is invalid.");
    return;
  }
  if (emergency_stop_active_) {
    RCLCPP_WARN(get_logger(), "Shot cycle rejected: emergency stop is active.");
    return;
  }
  if (operation_mode_ != OperationMode::SHOT_CYCLE) {
    RCLCPP_WARN(
      get_logger(),
      "Shot cycle rejected: operation mode is not SHOT_CYCLE.");
    return;
  }
  if (state_ != State::IDLE) {
    RCLCPP_WARN(
      get_logger(), "Shot cycle rejected: state is %s, not IDLE.",
      state_name(state_));
    return;
  }
  if (shot_cycle_running_) {
    RCLCPP_WARN(get_logger(), "Shot cycle rejected: it is already running.");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblePositionController>());
  rclcpp::shutdown();
  return 0;
}
