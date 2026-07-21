#include "belt_controller.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  const auto robot_command_topic = declare_parameter<std::string>(
    "robot_command_topic", "/robot_command");
  const auto belt_rpm_topic = declare_parameter<std::string>("belt_rpm_topic", "/belt/rpm");

  stop_rpm_ = declare_parameter<int>("stop_rpm", 0);
  level_1_rpm_ = declare_parameter<int>("level_1_rpm", 3000);
  level_2_rpm_ = declare_parameter<int>("level_2_rpm", 4000);
  level_3_rpm_ = declare_parameter<int>("level_3_rpm", 5000);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 10);

  if (stop_rpm_ != 0) {
    RCLCPP_ERROR(get_logger(), "stop_rpm must be zero");
    is_configuration_valid_ = false;
  }
  if (!is_rpm_valid(level_1_rpm_) || !is_rpm_valid(level_2_rpm_) || !is_rpm_valid(level_3_rpm_)) {
    RCLCPP_ERROR(get_logger(), "Each level RPM must fit in std_msgs/msg/Int16");
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    is_configuration_valid_ = false;
    command_period_ms_ = 10;
  }

  robot_command_sub_ = create_subscription<robot_controller::msg::RobotCommand>(
    robot_command_topic, 10,
    std::bind(&BeltControllerNode::robot_command_callback, this, std::placeholders::_1));
  rpm_pub_ = create_publisher<std_msgs::msg::Int16>(belt_rpm_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&BeltControllerNode::timer_callback, this));
}

void BeltControllerNode::robot_command_callback(
  const robot_controller::msg::RobotCommand::SharedPtr msg)
{
  belt_is_fire_ = msg->belt_is_fire;
  belt_mode_ = msg->belt_mode;
}

void BeltControllerNode::timer_callback()
{
  std_msgs::msg::Int16 rpm_command;
  rpm_command.data = static_cast<int16_t>(
    is_configuration_valid_ && belt_is_fire_ ? target_rpm_from_mode(belt_mode_) : 0);
  rpm_pub_->publish(rpm_command);
}

int BeltControllerNode::target_rpm_from_mode(uint8_t mode)
{
  switch (mode) {
    case stop_mode_:
      return stop_rpm_;
    case level_1_mode_:
      return level_1_rpm_;
    case level_2_mode_:
      return level_2_rpm_;
    case level_3_mode_:
      return level_3_rpm_;
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unsupported belt_mode %u. Stopping belt.", mode);
      return stop_rpm_;
  }
}

bool BeltControllerNode::is_rpm_valid(int rpm) const
{
  return rpm >= std::numeric_limits<int16_t>::min() &&
         rpm <= std::numeric_limits<int16_t>::max();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeltControllerNode>());
  rclcpp::shutdown();
  return 0;
}
