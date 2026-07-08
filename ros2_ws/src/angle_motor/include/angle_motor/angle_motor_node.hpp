#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

// 回転状態を管理する列挙型 (正しく Positive に修正)
enum class RotationMode
{
  Stop,
  PositiveRotate,
  NegativeRotate
};

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
  // メンバ変数のスペルを修正
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pwm_publisher_;

  // 関数名のスペルを修正 (JoycallBack -> joyCallback, Parametes -> Parameters)
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  void declareParameters();
  void getParameters();

  RotationMode determineRotationMode(const sensor_msgs::msg::Joy & joy_msg);
  bool isButtonPressed(const sensor_msgs::msg::Joy & joy_msg, int button_index);
  int16_t getPwmValueFromMode(RotationMode mode) const;

  // ボタン・PWM関連変数のタイポをすべて修正
  int enable_button_;
  int positive_button_;
  int negative_button_;
  int stop_button_;

  int positive_pwm_;
  int negative_pwm_;
  int stop_pwm_;

  RollerCommand current_command_;
};
