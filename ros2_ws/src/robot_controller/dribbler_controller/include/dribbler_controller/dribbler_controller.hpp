#ifndef DRIBBLER_CONTROLLER__DRIBBLER_CONTROLLER_HPP_
#define DRIBBLER_CONTROLLER__DRIBBLER_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"

class DribblerControllerNode : public rclcpp::Node
{
public:
  DribblerControllerNode();

private:
  void declare_parameters();
  void get_parameters();

  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();

  int dribble_on_rpm_{2000};
  int command_period_ms_{20};
  int qos_depth_{1};

  bool dribble_enabled_{false};
  bool emergency_stop_active_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr dribble_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DRIBBLER_CONTROLLER__DRIBBLER_CONTROLLER_HPP_
