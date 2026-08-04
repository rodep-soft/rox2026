#include "belt_controller/belt_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  declare_parameters();
  get_parameters();
  if (command_period_ms_ <= 0) {command_period_ms_ = 10;}

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
  // 既存のyamlキー名との後方互換性を保つため個別に読み込んで配列に詰める
  int rpm = 0;
  get_parameter("level_1_rpm", rpm); level_rpms_[0] = rpm;
  get_parameter("level_2_rpm", rpm); level_rpms_[1] = rpm;
  get_parameter("level_3_rpm", rpm); level_rpms_[2] = rpm;
  get_parameter("level_4_rpm", rpm); level_rpms_[3] = rpm;
  get_parameter("level_5_rpm", rpm); level_rpms_[4] = rpm;
  get_parameter("level_6_rpm", rpm); level_rpms_[5] = rpm;
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void BeltControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  belt_mode_ = (msg->data <= static_cast<uint8_t>(BeltMode::LEVEL_6)) ?
    static_cast<BeltMode>(msg->data) :
    BeltMode::STOP;
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
  const auto level = static_cast<uint8_t>(belt_mode_);
  if (level == 0 || level > kNumLevels) {
    return 0;
  }
  return level_rpms_[level - 1];  // LEVEL_1=インデックス0, ..., LEVEL_6=インデックス5
}
