#pragma once

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  static constexpr uint8_t stop_mode_{0};
  static constexpr uint8_t level_1_mode_{1};
  static constexpr uint8_t level_2_mode_{2};
  static constexpr uint8_t level_3_mode_{3};

  void belt_fire_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void timer_callback();
  int target_rpm_from_mode(uint8_t mode);
  bool is_rpm_valid(int rpm) const;

  bool is_configuration_valid_{true};
  bool belt_is_fire_{false};
  uint8_t belt_mode_{stop_mode_};
  int stop_rpm_{0};
  int level_1_rpm_{3000};
  int level_2_rpm_{4000};
  int level_3_rpm_{5000};
  int command_period_ms_{10};
  int qos_depth_{1};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr belt_fire_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
