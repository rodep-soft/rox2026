#pragma once

#include <cstdint>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

class MotorCanCommandNode : public rclcpp::Node
{
public:
  MotorCanCommandNode();

private:
  void DeclareParameters();
  void GetParameters();
  void PwmCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void TimerCallback();

  int16_t ClampPwm(int value) const;
  int16_t GetPwmOrZeroOnTimeout(const rclcpp::Time & now) const;
  can_msgs::msg::Frame CreateCanFrame(int16_t pwm, const rclcpp::Time & stamp) const;

  std::string pwm_topic_;
  std::string can_tx_topic_;
  uint32_t can_id_;
  uint8_t motor_id_;
  bool is_extended_;
  int min_pwm_;
  int max_pwm_;
  int send_period_ms_;
  int timeout_ms_;

  int16_t latest_pwm_;
  rclcpp::Time last_pwm_time_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pwm_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
