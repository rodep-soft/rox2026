#pragma once

#include <cstdint>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  static constexpr uint8_t stop_mode_{1};
  static constexpr uint8_t level_1_mode_{2};
  static constexpr uint8_t level_2_mode_{3};
  static constexpr uint8_t level_3_mode_{4};

  void declare_parameters();
  void get_parameters();
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void underbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void upperbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void timer_callback();
  int target_rpm_from_mode(uint8_t mode);
  bool is_rpm_valid(int rpm) const;
  bool is_belt_ready(
    int underbelt_target_rpm, int upperbelt_target_rpm,
    const rclcpp::Time & current_time);

  bool is_configuration_valid_{true};
  uint8_t belt_mode_{stop_mode_};
  int stop_rpm_{0};
  int level_1_rpm_{1000};
  int level_2_rpm_{2000};
  int level_3_rpm_{3000};
  int command_period_ms_{10};
  int ready_tolerance_rpm_{100};
  double ready_hold_sec_{0.1};
  int qos_depth_{1};
  std::string belt_mode_topic_;
  std::string underbelt_rpm_topic_;
  std::string upperbelt_rpm_topic_;
  std::string underbelt_current_rpm_topic_;
  std::string upperbelt_current_rpm_topic_;
  std::string belt_ready_topic_;
  int underbelt_current_rpm_{0};
  int upperbelt_current_rpm_{0};
  bool underbelt_feedback_received_{false};
  bool upperbelt_feedback_received_{false};
  rclcpp::Time ready_since_{};

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr underbelt_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr upperbelt_feedback_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr underbelt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr upperbelt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr belt_ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
