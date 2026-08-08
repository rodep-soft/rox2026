#ifndef DRIBBLER_CONTROLLER__DRIBBLER_CONTROLLER_HPP_

#include <cstdint>
#include <string>
#define DRIBBLER_CONTROLLER__DRIBBLER_CONTROLLER_HPP_

#include "actuator_msgs/msg/actuator_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

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
  uint16_t logical_id_{12};
  std::string target_topic_;

  bool dribble_enabled_{false};
  bool emergency_stop_active_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr dribble_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DRIBBLER_CONTROLLER__DRIBBLER_CONTROLLER_HPP_
