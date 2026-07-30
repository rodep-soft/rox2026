#include "belt_controller.hpp"

#include <chrono>
#include <cmath>
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
  if (!is_rpm_valid(level_1_rpm_) || !is_rpm_valid(level_2_rpm_) ||
    !is_rpm_valid(level_3_rpm_) || !is_rpm_valid(level_4_rpm_) ||
    !is_rpm_valid(level_5_rpm_) || !is_rpm_valid(level_6_rpm_))
  {
    RCLCPP_ERROR(get_logger(), "Each level RPM must fit in std_msgs/msg/Int16");
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    is_configuration_valid_ = false;
    command_period_ms_ = 10;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }
  if (ready_tolerance_rpm_ < 0) {
    ready_tolerance_rpm_ = 100;
  }
  if (ready_hold_sec_ < 0.0) {
    ready_hold_sec_ = 0.1;
  }

  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    belt_mode_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &BeltControllerNode::belt_mode_callback, this,
      std::placeholders::_1));
  underbelt_feedback_sub_ = create_subscription<std_msgs::msg::Int16>(
    underbelt_current_rpm_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &BeltControllerNode::underbelt_feedback_callback, this,
      std::placeholders::_1));
  upperbelt_feedback_sub_ = create_subscription<std_msgs::msg::Int16>(
    upperbelt_current_rpm_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &BeltControllerNode::upperbelt_feedback_callback, this,
      std::placeholders::_1));

  underbelt_rpm_pub_ = create_publisher<std_msgs::msg::Int16>(
    underbelt_rpm_topic_, rclcpp::QoS(qos_depth_));
  upperbelt_rpm_pub_ = create_publisher<std_msgs::msg::Int16>(
    upperbelt_rpm_topic_, rclcpp::QoS(qos_depth_));
  belt_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    belt_ready_topic_, rclcpp::QoS(1).reliable().transient_local());

  timer_ =
    create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&BeltControllerNode::timer_callback, this));
}

void BeltControllerNode::declare_parameters()
{
  declare_parameter<std::string>("belt_mode_topic", "/belt/mode");
  declare_parameter<std::string>(
    "underbelt_rpm_topic",
    "/underbelt/target/rpm");
  declare_parameter<std::string>(
    "upperbelt_rpm_topic",
    "/upperbelt/target/rpm");
  declare_parameter<std::string>(
    "underbelt_current_rpm_topic",
    "/underbelt/current/rpm");
  declare_parameter<std::string>(
    "upperbelt_current_rpm_topic",
    "/upperbelt/current/rpm");
  declare_parameter<std::string>("belt_ready_topic", "/belt/ready");
  declare_parameter<int>("stop_rpm", 0);
  declare_parameter<int>("level_1_rpm", 3000);
  declare_parameter<int>("level_2_rpm", 3500);
  declare_parameter<int>("level_3_rpm", 4000);
  declare_parameter<int>("level_4_rpm", 4500);
  declare_parameter<int>("level_5_rpm", 5000);
  declare_parameter<int>("level_6_rpm", 5500);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("ready_tolerance_rpm", 100);
  declare_parameter<double>("ready_hold_sec", 0.1);
  declare_parameter<int>("qos_depth", 1);
}

void BeltControllerNode::get_parameters()
{
  get_parameter("belt_mode_topic", belt_mode_topic_);
  get_parameter("underbelt_rpm_topic", underbelt_rpm_topic_);
  get_parameter("upperbelt_rpm_topic", upperbelt_rpm_topic_);
  get_parameter("underbelt_current_rpm_topic", underbelt_current_rpm_topic_);
  get_parameter("upperbelt_current_rpm_topic", upperbelt_current_rpm_topic_);
  get_parameter("belt_ready_topic", belt_ready_topic_);
  get_parameter("stop_rpm", stop_rpm_);
  get_parameter("level_1_rpm", level_1_rpm_);
  get_parameter("level_2_rpm", level_2_rpm_);
  get_parameter("level_3_rpm", level_3_rpm_);
  get_parameter("level_4_rpm", level_4_rpm_);
  get_parameter("level_5_rpm", level_5_rpm_);
  get_parameter("level_6_rpm", level_6_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("ready_tolerance_rpm", ready_tolerance_rpm_);
  get_parameter("ready_hold_sec", ready_hold_sec_);
  get_parameter("qos_depth", qos_depth_);
}

void BeltControllerNode::belt_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > static_cast<uint8_t>(BeltMode::LEVEL_6)) {
    belt_mode_ = BeltMode::STOP;
    return;
  }
  belt_mode_ = static_cast<BeltMode>(msg->data);
}

void BeltControllerNode::underbelt_feedback_callback(
  const std_msgs::msg::Int16::SharedPtr msg)
{
  underbelt_current_rpm_ = msg->data;
  underbelt_feedback_received_ = true;
}

void BeltControllerNode::upperbelt_feedback_callback(
  const std_msgs::msg::Int16::SharedPtr msg)
{
  upperbelt_current_rpm_ = msg->data;
  upperbelt_feedback_received_ = true;
}

void BeltControllerNode::timer_callback()
{
  std_msgs::msg::Int16 rpm_command;
  int target_rpm = stop_rpm_;
  if (is_configuration_valid_) {
    target_rpm = target_rpm_from_mode(belt_mode_);
  }
  rpm_command.data = static_cast<int16_t>(target_rpm);
  // under/upperの2モータへ同一RPMを送る。
  underbelt_rpm_pub_->publish(rpm_command);
  upperbelt_rpm_pub_->publish(rpm_command);
  std_msgs::msg::Bool ready;
  ready.data = is_belt_ready(rpm_command.data, now());
  belt_ready_pub_->publish(ready);
}

bool BeltControllerNode::is_belt_ready(
  int target_rpm,
  const rclcpp::Time & current_time)
{
  const bool within_tolerance =
    target_rpm != stop_rpm_ && underbelt_feedback_received_ &&
    upperbelt_feedback_received_ &&
    std::abs(underbelt_current_rpm_ - target_rpm) <= ready_tolerance_rpm_ &&
    std::abs(upperbelt_current_rpm_ - target_rpm) <= ready_tolerance_rpm_;
  if (!within_tolerance) {
    ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return false;
  }
  if (ready_since_.nanoseconds() == 0) {
    ready_since_ = current_time;
  }
  return (current_time - ready_since_).seconds() >= ready_hold_sec_;
}

int BeltControllerNode::target_rpm_from_mode(BeltMode mode)
{
  // joy_controllerから受けた速度段階を、configで設定した実RPMへ変換する。
  switch (mode) {
    case BeltMode::STOP:
      return stop_rpm_;

    case BeltMode::LEVEL_1:
      return level_1_rpm_;

    case BeltMode::LEVEL_2:
      return level_2_rpm_;

    case BeltMode::LEVEL_3:
      return level_3_rpm_;

    case BeltMode::LEVEL_4:
      return level_4_rpm_;

    case BeltMode::LEVEL_5:
      return level_5_rpm_;

    case BeltMode::LEVEL_6:
      return level_6_rpm_;
  }
  return stop_rpm_;
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
