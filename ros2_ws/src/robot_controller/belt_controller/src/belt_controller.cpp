#include "belt_controller/belt_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  declare_parameters();
  get_parameters();
  if (command_period_ms_ <= 0) command_period_ms_ = 10;

  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&BeltControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos,
    std::bind(&BeltControllerNode::belt_mode_callback, this, std::placeholders::_1));

  underbelt_command_pub_ = create_publisher<std_msgs::msg::Int16>(
    "/underbelt/target/rpm", command_qos);
  upperbelt_command_pub_ = create_publisher<std_msgs::msg::Int16>(
    "/upperbelt/target/rpm", command_qos);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&BeltControllerNode::timer_callback, this));
}

void BeltControllerNode::declare_parameters()
{
  declare_parameter<int>("level_1_rpm", 3000);
  declare_parameter<int>("level_2_rpm", 3500);
  declare_parameter<int>("level_3_rpm", 4000);
  declare_parameter<int>("level_4_rpm", 4500);
  declare_parameter<int>("level_5_rpm", 5000);
  declare_parameter<int>("level_6_rpm", 5500);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void BeltControllerNode::get_parameters()
{
  get_parameter("level_1_rpm", level_1_rpm_);
  get_parameter("level_2_rpm", level_2_rpm_);
  get_parameter("level_3_rpm", level_3_rpm_);
  get_parameter("level_4_rpm", level_4_rpm_);
  get_parameter("level_5_rpm", level_5_rpm_);
  get_parameter("level_6_rpm", level_6_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void BeltControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  belt_mode_ = (msg->data <= static_cast<uint8_t>(BeltMode::LEVEL_6))
                 ? static_cast<BeltMode>(msg->data)
                 : BeltMode::STOP;
}

void BeltControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
}

void BeltControllerNode::timer_callback()
{
  std_msgs::msg::Int16 cmd;
  cmd.data = static_cast<int16_t>(emergency_stop_active_ ? 0 : belt_target_rpm());
  underbelt_command_pub_->publish(cmd);
  upperbelt_command_pub_->publish(cmd);
}

int BeltControllerNode::belt_target_rpm() const
{
  switch (belt_mode_) {
    case BeltMode::LEVEL_1: return level_1_rpm_;
    case BeltMode::LEVEL_2: return level_2_rpm_;
    case BeltMode::LEVEL_3: return level_3_rpm_;
    case BeltMode::LEVEL_4: return level_4_rpm_;
    case BeltMode::LEVEL_5: return level_5_rpm_;
    case BeltMode::LEVEL_6: return level_6_rpm_;
    default: return 0;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeltControllerNode>());
  rclcpp::shutdown();
  return 0;
}
