#pragma once

#include <cstdint>
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
  enum class BeltMode : uint8_t
  {
    STOP,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
    LEVEL_5,
    LEVEL_6,
  };

  void declare_parameters();
  void get_parameters();
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void underbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void upperbelt_feedback_callback(const std_msgs::msg::Int16::SharedPtr msg);
  void timer_callback();
  int target_rpm_from_mode(BeltMode mode);
  bool is_rpm_valid(int rpm) const;
  bool is_belt_ready(int target_rpm, const rclcpp::Time & current_time);

  bool is_configuration_valid_{true};
  BeltMode belt_mode_{BeltMode::STOP};
  int stop_rpm_{0};
  int level_1_rpm_{3000};
  int level_2_rpm_{3500};
  int level_3_rpm_{4000};
  int level_4_rpm_{4500};
  int level_5_rpm_{5000};
  int level_6_rpm_{5500};
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
#include <chrono>
