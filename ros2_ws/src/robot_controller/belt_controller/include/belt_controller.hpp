#pragma once

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
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
  void underbelt_feedback_callback(
    const std_msgs::msg::Float32::SharedPtr msg);
  void upperbelt_feedback_callback(
    const std_msgs::msg::Float32::SharedPtr msg);
  void timer_callback();
  double target_rpm_from_mode(uint8_t mode);
  bool is_rpm_valid(double rpm) const;
  bool is_belt_ready(
    double target_rpm, const rclcpp::Time & current_time);

  bool is_configuration_valid_{true};
  uint8_t belt_mode_{stop_mode_};
  double stop_rpm_{0.0};
  double level_1_rpm_{1000.0};
  double level_2_rpm_{2000.0};
  double level_3_rpm_{3000.0};
  int command_period_ms_{10};
  double ready_tolerance_rpm_{100.0};
  double ready_hold_sec_{0.1};
  int qos_depth_{1};
  std::string belt_mode_topic_;
  std::string underbelt_rpm_topic_;
  std::string upperbelt_rpm_topic_;
  std::string underbelt_current_rpm_topic_;
  std::string upperbelt_current_rpm_topic_;
  std::string belt_ready_topic_;
  double underbelt_current_rpm_{0.0};
  double upperbelt_current_rpm_{0.0};
  bool underbelt_feedback_received_{false};
  bool upperbelt_feedback_received_{false};
  rclcpp::Time ready_since_{};

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    underbelt_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    upperbelt_feedback_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr underbelt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr upperbelt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr belt_ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
