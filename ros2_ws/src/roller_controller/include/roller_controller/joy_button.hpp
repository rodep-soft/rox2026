#pragma once

#include <cstddef>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace roller_controller
{
inline bool IsButtonPressed(
  rclcpp::Node & node,
  const sensor_msgs::msg::Joy & msg,
  int button_index)
{
  if (button_index < 0 || static_cast<size_t>(button_index) >= msg.buttons.size()) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 1000,
      "Button index %d is outside the received Joy message.", button_index);
    return false;
  }
  return msg.buttons[button_index] == 1;
}

}  // namespace roller_controller
