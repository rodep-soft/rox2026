#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

class RollerBeltCanPackerNode : public rclcpp::Node
{
public:
  RollerBeltCanPackerNode();

private:
  void DeclareParameters();
  void GetParameters();

  void RollerCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void BeltCallback(const std_msgs::msg::Int16::SharedPtr msg);

  void TimerCallback();
  int16_t ClampRpm(int value) const;

  int16_t GetRpmOrZeroOnTimeout(
    int16_t rpm,
    const rclcpp::Time & last_received_time,
    const rclcpp::Time & now) const;
  can_msgs::msg::Frame CreateStmCanFrame(const rclcpp::Time & stamp) const;

  std::string can_transmit_topic_;
  uint32_t stm_can_id_;
  bool is_extended_;
  std::string roller_rpm_topic_;
  std::string belt_rpm_topic_;
  int min_rpm_;
  int max_rpm_;
  int publish_period_ms_;
  int timeout_ms_;

  int16_t roller_rpm_;
  int16_t belt_rpm_;
  rclcpp::Time roller_last_received_time_;
  rclcpp::Time belt_last_received_time_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr roller_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr belt_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
