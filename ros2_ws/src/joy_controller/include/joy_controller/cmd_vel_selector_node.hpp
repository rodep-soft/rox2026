#ifndef JOY_CONTROLLER__CMD_VEL_SELECTOR_NODE_HPP_
#define JOY_CONTROLLER__CMD_VEL_SELECTOR_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace joy_controller
{

class CmdVelSelectorNode : public rclcpp::Node
{
public:
  explicit CmdVelSelectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CmdVelSelectorNode() = default;

private:
  void declare_parameters();
  void get_parameters();

  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void manual_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void auto_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // パラメータ
  std::string manual_cmd_vel_topic_;
  std::string auto_cmd_vel_topic_;
  std::string operation_mode_topic_;
  std::string output_cmd_vel_topic_;

  // モード定数 (joy_controller/README.md 準拠: C++ルールに従い k プレフィックスなし)
  static constexpr uint8_t mode_stop = 0;
  static constexpr uint8_t mode_drive = 1;       // 手動モード
  static constexpr uint8_t mode_shot_cycle = 2;  // 自動制御モード
  static constexpr uint8_t mode_belt_only = 3;

  // 内部状態
  uint8_t current_operation_mode_{mode_stop};

  // 通信
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr operation_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_cmd_vel_pub_;
};

}  // namespace joy_controller

#endif  // JOY_CONTROLLER__CMD_VEL_SELECTOR_NODE_HPP_
