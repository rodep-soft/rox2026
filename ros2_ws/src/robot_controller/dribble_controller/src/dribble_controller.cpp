#include "dribble_controller/dribble_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

DribbleController::DribbleController()
: Node("dribble_controller_node")
{
  const auto robot_command_topic = declare_parameter<std::string>(
    "robot_command_topic", "/robot_command");
  const auto dribble_velocity_command_topic = declare_parameter<std::string>(
    "dribble_velocity_command_topic", "/dribble_velocity_command");
  const auto dribble_stop_request_topic = declare_parameter<std::string>(
    "dribble_stop_request_topic", "/dribble_stop_request");
  const auto dribble_is_stopped_topic = declare_parameter<std::string>(
    "dribble_is_stopped_topic", "/dribble_is_stopped");

  low_velocity_rad_s_ = declare_parameter<double>("low_velocity_rad_s", 30.0);
  high_velocity_rad_s_ = declare_parameter<double>("high_velocity_rad_s", 60.0);
  stop_deceleration_rad_s2_ = declare_parameter<double>("stop_deceleration_rad_s2", 20.0);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 10);

  if (stop_deceleration_rad_s2_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "stop_deceleration_rad_s2 must be greater than zero");
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    is_configuration_valid_ = false;
  }

  robot_command_sub_ = create_subscription<robot_controller::msg::RobotCommand>(
    robot_command_topic, 10,
    std::bind(&DribbleController::robot_command_callback, this, std::placeholders::_1));
  stop_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    dribble_stop_request_topic, 10,
    std::bind(&DribbleController::stop_request_callback, this, std::placeholders::_1));
  velocity_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    dribble_velocity_command_topic, 10);
  is_stopped_pub_ = create_publisher<std_msgs::msg::Bool>(dribble_is_stopped_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribbleController::timer_callback, this));
}

void DribbleController::robot_command_callback(
  const robot_controller::msg::RobotCommand::SharedPtr msg)
{
  dribble_mode_ = msg->dribble_mode;
}

void DribbleController::stop_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  stop_requested_ = msg->data;
}

double DribbleController::target_velocity_from_mode(uint8_t mode)
{
  switch (mode) {
    case stop_mode_:
      return 0.0;
    case low_mode_:
      return low_velocity_rad_s_;
    case high_mode_:
      return high_velocity_rad_s_;
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported dribble_mode %u. Stopping dribble.", mode);
      return 0.0;
  }
}

void DribbleController::timer_callback()
{
  const double target_velocity_rad_s =
    stop_requested_ ? 0.0 : target_velocity_from_mode(dribble_mode_);

  if (stop_requested_) {
    const double maximum_decrease =
      stop_deceleration_rad_s2_ * static_cast<double>(command_period_ms_) / 1000.0;
    current_velocity_rad_s_ = std::max(
      0.0, current_velocity_rad_s_ - maximum_decrease);
  } else {
    current_velocity_rad_s_ = target_velocity_rad_s;
  }

  if (!is_configuration_valid_) {
    current_velocity_rad_s_ = 0.0;
  }

  std_msgs::msg::Float32 velocity_command;
  velocity_command.data = static_cast<float>(current_velocity_rad_s_);
  velocity_command_pub_->publish(velocity_command);

  std_msgs::msg::Bool is_stopped;
  is_stopped.data = stop_requested_ && current_velocity_rad_s_ == 0.0;
  is_stopped_pub_->publish(is_stopped);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribbleController>());
  rclcpp::shutdown();
  return 0;
}
