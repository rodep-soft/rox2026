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
    current_position_mode_ = static_cast<PositionMode>(msg->data);
  }
}

void ArmPositionControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
