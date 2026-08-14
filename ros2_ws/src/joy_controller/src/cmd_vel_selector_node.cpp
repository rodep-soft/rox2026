#include "joy_controller/cmd_vel_selector_node.hpp"

namespace joy_controller
{

CmdVelSelectorNode::CmdVelSelectorNode(const rclcpp::NodeOptions & options)
: Node("cmd_vel_selector_node", options)
{
  declare_parameters();
  get_parameters();

  // サブスクリプション・パブリッシャーの作成
  operation_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    operation_mode_topic_, rclcpp::QoS(10),
    std::bind(&CmdVelSelectorNode::operation_mode_callback, this, std::placeholders::_1));

  manual_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    manual_cmd_vel_topic_, rclcpp::QoS(10),
    std::bind(&CmdVelSelectorNode::manual_cmd_vel_callback, this, std::placeholders::_1));

  auto_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    auto_cmd_vel_topic_, rclcpp::QoS(10),
    std::bind(&CmdVelSelectorNode::auto_cmd_vel_callback, this, std::placeholders::_1));

  output_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    output_cmd_vel_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "CmdVelSelectorNode initialized. Output topic: %s",
    output_cmd_vel_topic_.c_str());
}

void CmdVelSelectorNode::declare_parameters()
{
  declare_parameter<std::string>("manual_cmd_vel_topic", "/joy_controller/cmd_vel");
  declare_parameter<std::string>("auto_cmd_vel_topic", "/auto_game1/cmd_vel");
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("output_cmd_vel_topic", "/mecanum/cmd_vel");
}

void CmdVelSelectorNode::get_parameters()
{
  get_parameter("manual_cmd_vel_topic", manual_cmd_vel_topic_);
  get_parameter("auto_cmd_vel_topic", auto_cmd_vel_topic_);
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
}

void CmdVelSelectorNode::operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (current_operation_mode_ != msg->data) {
    current_operation_mode_ = msg->data;
    RCLCPP_INFO(
      get_logger(),
      "Operation mode updated in CmdVelSelector: %d (%s)",
      current_operation_mode_,
      (current_operation_mode_ == mode_drive) ? "MANUAL / DRIVE" :
      (current_operation_mode_ == mode_shot_cycle) ? "AUTO / SHOT_CYCLE" : "STOP");

    // モード切替時に一瞬停止コマンドを出力して急変を防止
    geometry_msgs::msg::Twist stop_twist;
    output_cmd_vel_pub_->publish(stop_twist);
  }
}

void CmdVelSelectorNode::manual_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 手動モード (DRIVE = 1) の場合のみ転送
  if (current_operation_mode_ == mode_drive) {
    output_cmd_vel_pub_->publish(*msg);
  }
}

void CmdVelSelectorNode::auto_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 自動モード (SHOT_CYCLE = 2) の場合のみ転送
  if (current_operation_mode_ == mode_shot_cycle) {
    output_cmd_vel_pub_->publish(*msg);
  }
}

}  // namespace joy_controller
