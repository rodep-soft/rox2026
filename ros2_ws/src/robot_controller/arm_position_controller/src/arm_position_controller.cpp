#include "arm_position_controller/arm_position_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>

ArmPositionControllerNode::ArmPositionControllerNode()
: Node("arm_position_controller_node")
{
  declare_parameters();
  get_parameters();

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos = rclcpp::QoS(qos_depth_);

  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    "/dribble/position_command", command_qos);

  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/dribble/position_mode", command_qos,
    std::bind(&ArmPositionControllerNode::position_mode_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&ArmPositionControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&ArmPositionControllerNode::timer_callback, this));
}

void ArmPositionControllerNode::declare_parameters()
{
  declare_parameter<double>("dribble_position_rad", 0.35);
  declare_parameter<double>("open_position_rad", -1.0);
  declare_parameter<double>("feed_position_rad", 1.3);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
}

void ArmPositionControllerNode::get_parameters()
{
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("open_position_rad", open_position_rad_);
  get_parameter("feed_position_rad", feed_position_rad_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void ArmPositionControllerNode::position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data <= static_cast<uint8_t>(PositionMode::FEED)) {
    const auto new_mode = static_cast<PositionMode>(msg->data);
    if (new_mode != current_position_mode_) {
      RCLCPP_INFO(
        get_logger(), "Arm Mode Changed: %s -> %s",
        mode_name(current_position_mode_), mode_name(new_mode));
      current_position_mode_ = new_mode;
    }
  }
}

void ArmPositionControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data != emergency_stop_active_) {
    if (msg->data) {
      RCLCPP_WARN(get_logger(), "Emergency Stop Activated in ArmPositionController!");
    } else {
      RCLCPP_INFO(get_logger(), "Emergency Stop Released in ArmPositionController.");
    }
  }
  emergency_stop_active_ = msg->data;
}

void ArmPositionControllerNode::timer_callback()
{
  std_msgs::msg::Float32 pos_msg;
  if (emergency_stop_active_) {
    pos_msg.data = static_cast<float>(dribble_position_rad_);
  } else {
    switch (current_position_mode_) {
      case PositionMode::DRIBBLE: pos_msg.data = static_cast<float>(dribble_position_rad_); break;
      case PositionMode::OPEN: pos_msg.data = static_cast<float>(open_position_rad_); break;
      case PositionMode::FEED: pos_msg.data = static_cast<float>(feed_position_rad_); break;
    }
  }
  position_command_pub_->publish(pos_msg);
}

const char * ArmPositionControllerNode::mode_name(PositionMode mode) const
{
  switch (mode) {
    case PositionMode::DRIBBLE: return "DRIBBLE";
    case PositionMode::OPEN:    return "OPEN";
    case PositionMode::FEED:    return "FEED";
  }
  return "UNKNOWN";
}
