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
  declare_parameters();
  get_parameters();

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
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }

  belt_fire_sub_ = create_subscription<std_msgs::msg::Bool>(
    belt_fire_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&BeltControllerNode::belt_fire_callback, this, std::placeholders::_1));

  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    belt_mode_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&BeltControllerNode::belt_mode_callback, this, std::placeholders::_1));

  rpm_pub_ = create_publisher<std_msgs::msg::Int16>(belt_rpm_topic_, rclcpp::QoS(qos_depth_));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&BeltControllerNode::timer_callback, this));
}

void BeltControllerNode::declare_parameters()
{
  declare_parameter<std::string>("belt_fire_topic", "/belt/fire_enabled");
  declare_parameter<std::string>("belt_mode_topic", "/belt/mode");
  declare_parameter<std::string>("belt_rpm_topic", "/belt/rpm_command");
  declare_parameter<int>("stop_rpm", 0);
  declare_parameter<int>("level_1_rpm", 3000);
  declare_parameter<int>("level_2_rpm", 4000);
  declare_parameter<int>("level_3_rpm", 5000);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void BeltControllerNode::get_parameters()
{
  get_parameter("belt_fire_topic", belt_fire_topic_);
  get_parameter("belt_mode_topic", belt_mode_topic_);
  get_parameter("belt_rpm_topic", belt_rpm_topic_);
  get_parameter("stop_rpm", stop_rpm_);
  get_parameter("level_1_rpm", level_1_rpm_);
  get_parameter("level_2_rpm", level_2_rpm_);
  get_parameter("level_3_rpm", level_3_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void BeltControllerNode::belt_fire_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  belt_is_fire_ = msg->data;
}

void BeltControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  belt_mode_ = msg->data;
}

void BeltControllerNode::timer_callback()
{
  std_msgs::msg::Int16 rpm_command;
  // 設定が不正、または発射無効中なら安全側として0 RPMを送る。
  rpm_command.data = static_cast<int16_t>(
    is_configuration_valid_ && belt_is_fire_ ? target_rpm_from_mode(belt_mode_) : 0);
  rpm_pub_->publish(rpm_command);
}

int BeltControllerNode::target_rpm_from_mode(uint8_t mode)
{
  // joy_controllerから受けた速度段階を、configで設定した実RPMへ変換する。
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
