#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  void DeclareParameters();
  void GetParameters();
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  int16_t SelectRpm(const sensor_msgs::msg::Joy & msg);

  std::string rpm_topic_;
  int enable_axis_;
  double enable_axis_threshold_;
  std::vector<int> command_buttons_;
  std::vector<int16_t> command_rpms_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_publisher_;
};
