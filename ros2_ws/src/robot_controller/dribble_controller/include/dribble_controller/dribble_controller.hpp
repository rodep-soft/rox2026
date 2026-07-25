#ifndef DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
#define DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"

class DribbleController : public rclcpp::Node
{
public:
  DribbleController();

private:
  void declare_parameters();
  void get_parameters();
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();

  bool is_configuration_valid_{true};
  bool dribble_enabled_{false};
  double on_rpm_{600.0};
  int command_period_ms_{10};
  int qos_depth_{1};
  std::string dribble_enabled_topic_;
  std::string dribble_rpm_topic_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
