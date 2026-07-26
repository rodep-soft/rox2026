#include "belt_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  declare_parameters();
  get_parameters();

  if (stop_rpm_ != 0.0) {
    RCLCPP_ERROR(get_logger(), "stop_rpm must be zero");
    is_configuration_valid_ = false;
  }
  if (
    !is_rpm_valid(level_1_rpm_) ||
    !is_rpm_valid(level_2_rpm_) ||
    !is_rpm_valid(level_3_rpm_))
  {
    RCLCPP_ERROR(
      get_logger(), "Each level RPM must be finite and non-negative");
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
  if (ready_tolerance_rpm_ < 0.0) {
    ready_tolerance_rpm_ = 100.0;
  }
  if (ready_hold_sec_ < 0.0) {
    ready_hold_sec_ = 0.1;
  }

  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    belt_mode_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &BeltControllerNode::belt_mode_callback, this,
      std::placeholders::_1));
  underbelt_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    underbelt_current_rpm_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &BeltControllerNode::underbelt_feedback_callback, this,
      std::placeholders::_1));
  upperbelt_feedback_sub_ = create_subscription<std_msgs::msg::Float32>(
    upperbelt_current_rpm_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &BeltControllerNode::upperbelt_feedback_callback, this,
      std::placeholders::_1));

  underbelt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>(
    underbelt_rpm_topic_, rclcpp::QoS(qos_depth_));
  upperbelt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>(
    upperbelt_rpm_topic_, rclcpp::QoS(qos_depth_));
  belt_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    belt_ready_topic_, rclcpp::QoS(1).reliable().transient_local());

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&BeltControllerNode::timer_callback, this));
}

void BeltControllerNode::declare_parameters()
{
  declare_parameter<std::string>("belt_mode_topic", "/belt/mode");
  declare_parameter<std::string>(
    "underbelt_rpm_topic", "/underbelt/target/rpm");
  declare_parameter<std::string>(
    "upperbelt_rpm_topic", "/upperbelt/target/rpm");
  declare_parameter<std::string>(
    "underbelt_current_rpm_topic", "/underbelt/current/rpm");
  declare_parameter<std::string>(
    "upperbelt_current_rpm_topic", "/upperbelt/current/rpm");
  declare_parameter<std::string>("belt_ready_topic", "/belt/ready");
  declare_parameter<double>("stop_rpm", 0.0);
  declare_parameter<double>("level_1_rpm", 1000.0);
  declare_parameter<double>("level_2_rpm", 2000.0);
  declare_parameter<double>("level_3_rpm", 3000.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<double>("ready_tolerance_rpm", 100.0);
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
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("ready_tolerance_rpm", ready_tolerance_rpm_);
  get_parameter("ready_hold_sec", ready_hold_sec_);
  get_parameter("qos_depth", qos_depth_);
}

void BeltControllerNode::belt_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  belt_mode_ = msg->data;
}

void BeltControllerNode::underbelt_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  underbelt_current_rpm_ = msg->data;
  underbelt_feedback_received_ = true;
}

void BeltControllerNode::upperbelt_feedback_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
{
  upperbelt_current_rpm_ = msg->data;
  upperbelt_feedback_received_ = true;
}

void BeltControllerNode::timer_callback()
{
  std_msgs::msg::Float32 rpm_command;
  rpm_command.data = static_cast<float>(
    is_configuration_valid_ ? target_rpm_from_mode(belt_mode_) : 0.0);
  underbelt_rpm_pub_->publish(rpm_command);
  upperbelt_rpm_pub_->publish(rpm_command);

  std_msgs::msg::Bool ready;
  ready.data = is_belt_ready(rpm_command.data, now());
  belt_ready_pub_->publish(ready);
}

bool BeltControllerNode::is_belt_ready(
  double target_rpm, const rclcpp::Time & current_time)
{
  const bool within_tolerance =
    target_rpm != stop_rpm_ &&
    underbelt_feedback_received_ &&
    upperbelt_feedback_received_ &&
    std::abs(underbelt_current_rpm_ - target_rpm) <=
    ready_tolerance_rpm_ &&
    std::abs(upperbelt_current_rpm_ - target_rpm) <=
    ready_tolerance_rpm_;

  if (!within_tolerance) {
    ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return false;
  }
  if (ready_since_.nanoseconds() == 0) {
    ready_since_ = current_time;
  }
  return (current_time - ready_since_).seconds() >= ready_hold_sec_;
}

double BeltControllerNode::target_rpm_from_mode(uint8_t mode)
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
        get_logger(), *get_clock(), 1000,
        "Unsupported belt_mode %u. Stopping belt.", mode);
      return stop_rpm_;
  }
}

bool BeltControllerNode::is_rpm_valid(double rpm) const
{
  return std::isfinite(rpm) && rpm >= 0.0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeltControllerNode>());
  rclcpp::shutdown();
  return 0;
}
