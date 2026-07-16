#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

class RollerControllerNode : public rclcpp::Node
{
public:
  RollerControllerNode();

private:
  void DeclareParameters();
  void GetParameters();
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  int16_t SelectRpm(const sensor_msgs::msg::Joy & msg);

  std::string rpm_topic_;
  int enable_axis_;
  double enable_axis_threshold_;
  int stop_button_;
  int high_button_;
  int low_button_;
  int16_t current_rpm_;
  int16_t stop_rpm_;
  int16_t high_rpm_;
  int16_t low_rpm_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_publisher_;
};
