#ifndef DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
#define DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

class DribbleController : public rclcpp::Node
{
public:
  DribbleController();

private:
  static constexpr uint8_t stop_mode_{1};
  static constexpr uint8_t high_mode_{2};
  static constexpr uint8_t low_mode_{3};

  void declare_parameters();
  void get_parameters();
  void dribble_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void timer_callback();
  double target_rpm_from_mode(uint8_t mode);

  bool is_configuration_valid_{true};
  uint8_t dribble_mode_{stop_mode_};
  double current_rpm_{0.0};
  double low_rpm_{300.0};
  double high_rpm_{600.0};
  int command_period_ms_{10};
  int qos_depth_{1};
  std::string dribble_mode_topic_;
  std::string dribble_rpm_topic_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr dribble_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
