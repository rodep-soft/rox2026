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
  const auto dribble_mode_topic = declare_parameter<std::string>("dribble_mode_topic", "/dribble/mode");
  const auto dribble_rpm_topic = declare_parameter<std::string>(
    "dribble_rpm_topic", "/dribble/rpm");
  const auto dribble_stop_request_topic = declare_parameter<std::string>(
    "dribble_stop_request_topic", "/dribble_stop_request");
  const auto dribble_is_stopped_topic = declare_parameter<std::string>(
    "dribble_is_stopped_topic", "/dribble_is_stopped");

  low_rpm_ = declare_parameter<double>("low_rpm", 300.0);
  high_rpm_ = declare_parameter<double>("high_rpm", 600.0);
  stop_deceleration_rpm_s_ = declare_parameter<double>("stop_deceleration_rpm_s", 200.0);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 10);

  if (stop_deceleration_rpm_s_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "stop_deceleration_rpm_s must be greater than zero");
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    is_configuration_valid_ = false;
  }

  dribble_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    dribble_mode_topic, 10,
    std::bind(&DribbleController::dribble_mode_callback, this, std::placeholders::_1));
  stop_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    dribble_stop_request_topic, 10,
    std::bind(&DribbleController::stop_request_callback, this, std::placeholders::_1));
  rpm_pub_ = create_publisher<std_msgs::msg::Int16>(dribble_rpm_topic, 10);
  is_stopped_pub_ = create_publisher<std_msgs::msg::Bool>(dribble_is_stopped_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribbleController::timer_callback, this));
}

void DribbleController::dribble_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  dribble_mode_ = msg->data;
}

void DribbleController::stop_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  stop_requested_ = msg->data;
}

double DribbleController::target_rpm_from_mode(uint8_t mode)
{
  switch (mode) {
    case stop_mode_:
      return 0.0;
    case high_mode_:
      return high_rpm_;
    case low_mode_:
      return low_rpm_;
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported dribble_mode %u. Stopping dribble.", mode);
      return 0.0;
  }
}

void DribbleController::timer_callback()
{
  const double target_rpm = stop_requested_ ? 0.0 : target_rpm_from_mode(dribble_mode_);

  if (stop_requested_) {
    const double maximum_decrease =
      stop_deceleration_rpm_s_ * static_cast<double>(command_period_ms_) / 1000.0;
    current_rpm_ = std::max(0.0, current_rpm_ - maximum_decrease);
  } else {
    current_rpm_ = target_rpm;
  }

  if (!is_configuration_valid_) {
    current_rpm_ = 0.0;
  }

  std_msgs::msg::Int16 rpm_command;
  rpm_command.data = static_cast<int16_t>(std::round(current_rpm_));
  rpm_pub_->publish(rpm_command);

  std_msgs::msg::Bool is_stopped;
  is_stopped.data = stop_requested_ && current_rpm_ == 0.0;
  is_stopped_pub_->publish(is_stopped);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribbleController>());
  rclcpp::shutdown();
  return 0;
}
