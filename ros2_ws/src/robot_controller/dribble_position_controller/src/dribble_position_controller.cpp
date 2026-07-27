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
  if (command_period_ms_ <= 0) {command_period_ms_ = 20;}
  if (qos_depth_ <= 0) {qos_depth_ = 1;}
  if (!std::isfinite(position_tolerance_rad_) || position_tolerance_rad_ <= 0.0) {
    position_tolerance_rad_ = 0.02;
  }

  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    dribble_position_command_topic_, rclcpp::QoS(qos_depth_));
  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    dribble_position_mode_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribblePositionController::position_mode_callback, this, std::placeholders::_1));
  intake_shoot_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    intake_shoot_request_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &DribblePositionController::intake_shoot_request_callback, this, std::placeholders::_1));
  position_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    dribble_position_feedback_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribblePositionController::position_feedback_callback, this, std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&DribblePositionController::emergency_stop_callback, this, std::placeholders::_1));
  command_timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribblePositionController::watchdog_callback, this));
  set_target_position(dribble_position_rad_, State::IDLE);
}

void DribblePositionController::declare_parameters()
{
  declare_parameter<std::string>("dribble_position_command_topic", "/dribble/position_command");
  declare_parameter<std::string>("dribble_position_feedback_topic", "/dribble/position_feedback");
  declare_parameter<std::string>("dribble_position_mode_topic", "/dribble/position_mode");
  declare_parameter<std::string>("intake_shoot_request_topic", "/dribble/intake_shoot_request");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  declare_parameter<double>("dribble_position_rad", 0.0);
  declare_parameter<double>("intake_position_rad", 1.5);
  declare_parameter<double>("shoot_position_rad", 2.0);
  declare_parameter<double>("max_open_position_rad", -1.3);
  declare_parameter<double>("position_tolerance_rad", 0.02);
  declare_parameter<double>("shoot_to_dribble_delay_sec", 1.0);
  declare_parameter<double>("move_timeout_sec", 3.0);
  declare_parameter<double>("feedback_timeout_sec", 0.5);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
}

void DribblePositionController::get_parameters()
{
  get_parameter("dribble_position_command_topic", dribble_position_command_topic_);
  get_parameter("dribble_position_feedback_topic", dribble_position_feedback_topic_);
  get_parameter("dribble_position_mode_topic", dribble_position_mode_topic_);
  get_parameter("intake_shoot_request_topic", intake_shoot_request_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("intake_position_rad", intake_position_rad_);
  get_parameter("shoot_position_rad", shoot_position_rad_);
  get_parameter("max_open_position_rad", max_open_position_rad_);
  get_parameter("position_tolerance_rad", position_tolerance_rad_);
  get_parameter("shoot_to_dribble_delay_sec", shoot_to_dribble_delay_sec_);
  get_parameter("move_timeout_sec", move_timeout_sec_);
  get_parameter("feedback_timeout_sec", feedback_timeout_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void DribblePositionController::position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (emergency_stop_active_ || state_ != State::IDLE) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(), 1000, "Ignoring dribble position command while moving or stopped");
    return;
  }
  if (msg->data == 0) {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  } else if (msg->data == 1) {
    set_target_position(intake_position_rad_, State::INTAKE);
  } else if (msg->data == 2) {
    set_target_position(max_open_position_rad_, State::MAX_OPEN);
  }
}

void DribblePositionController::intake_shoot_request_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || emergency_stop_active_ || state_ != State::IDLE) {
    return;
  }
  set_target_position(intake_position_rad_, State::INTAKE);
}

void DribblePositionController::position_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  if (!std::isfinite(msg->data) || state_ == State::IDLE) {return;}
  last_feedback_time_ = now();
  if (std::abs(static_cast<double>(msg->data) - target_position_rad_) > position_tolerance_rad_) {
    return;
  }
  if (state_ == State::INTAKE) {
    set_target_position(shoot_position_rad_, State::SHOOT);
  } else if (state_ == State::SHOOT) {
    state_ = State::HOLD_SHOOT;
    phase_start_time_ = now();
  } else if (state_ == State::RETURN_TO_DRIBBLE) {
    state_ = State::IDLE;
  } else if (state_ == State::MAX_OPEN) {
    state_ = State::IDLE;
  }
}

void DribblePositionController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::watchdog_callback()
{
  if (state_ == State::IDLE) {return;}

  // 位置指令を送るたびにEduLite 05からfeedbackが返るため、到達するまで同じ目標を送る。
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(target_position_rad_);
  position_command_pub_->publish(command);

  const auto current_time = now();
  if ((current_time - last_feedback_time_).seconds() > feedback_timeout_sec_ ||
    (state_ != State::HOLD_SHOOT &&
    (current_time - phase_start_time_).seconds() > move_timeout_sec_))
  {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
    return;
  }
  if (state_ == State::HOLD_SHOOT &&
    (current_time - phase_start_time_).seconds() >= shoot_to_dribble_delay_sec_)
  {
    set_target_position(dribble_position_rad_, State::RETURN_TO_DRIBBLE);
  }
}

void DribblePositionController::set_target_position(double position_rad, State state)
{
  target_position_rad_ = position_rad;
  state_ = state;
  phase_start_time_ = now();
  last_feedback_time_ = phase_start_time_;
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_pub_->publish(command);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblePositionController>());
  rclcpp::shutdown();
  return 0;
}
