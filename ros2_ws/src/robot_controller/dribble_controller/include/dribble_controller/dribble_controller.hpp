#ifndef DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
#define DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_controller/msg/robot_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"

class DribbleController : public rclcpp::Node
{
public:
  DribbleController();

private:
  static constexpr uint8_t stop_mode_{0};
  static constexpr uint8_t high_mode_{1};
  static constexpr uint8_t low_mode_{2};

  void robot_command_callback(const robot_controller::msg::RobotCommand::SharedPtr msg);
  void stop_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();
  double target_rpm_from_mode(uint8_t mode);

  bool is_configuration_valid_{true};
  bool stop_requested_{false};
  uint8_t dribble_mode_{stop_mode_};
  double current_rpm_{0.0};
  double low_rpm_{300.0};
  double high_rpm_{600.0};
  double stop_deceleration_rpm_s_{200.0};
  int command_period_ms_{10};

  rclcpp::Subscription<robot_controller::msg::RobotCommand>::SharedPtr robot_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_request_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_stopped_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
