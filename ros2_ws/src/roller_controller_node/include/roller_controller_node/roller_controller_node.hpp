#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

// Modeを管理するためのenum
enum class RotationMode {Stop, PositiveRotate, NegativeRotate};

// 送るコマンドを格納
struct RollerCommand
{
  RotationMode mode;
  int16_t pwm_value;
};

class RollerControllerNode : public rclcpp::Node
{
public:
  RollerControllerNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pwm_publisher_;

  // Callback
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  // Parameters
  void DeclareParameters();
  void GetParameters();

  bool IsButtonPressed(const sensor_msgs::msg::Joy::SharedPtr joy_msg, int button_index) const;
  int16_t GetPwmValueFromMode(RotationMode mode) const;

  // Parameters
  // "enable_button_" + "それぞれのbutton"で
  // Positive, Negative, StopのModeを判断
  int enable_button_;
  int positive_button_;
  int negative_button_;
  int stop_button_;

  // pwm値を送るためのparam
  int positive_pwm_;
  int negative_pwm_;
  int stop_pwm_;

  // 現在のRollerのmodeを保存
  RollerCommand current_command_;
};
