#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robot_controller/msg/robot_command.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class SpringEduliteController : public rclcpp::Node
{
public:
  SpringEduliteController();

private:
  enum class State
  {
    READY,
    LOAD,
    FIRE,
  };

  State now_state_{State::LOAD};
  int limit_switch_index_{0};
  bool is_configuration_valid_{true};
  bool is_loaded_{false};
  bool previous_fire_request_{false};
  bool fire_pending_{false};
  double loading_velocity_rad_s_{0.0};
  double fire_velocity_rad_s_{0.0};
  double fire_duration_sec_{0.0};
  rclcpp::Time fire_start_time_;

  rclcpp::Subscription<robot_controller::msg::RobotCommand>::SharedPtr robot_command_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_switch_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spring_velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void robot_command_callback(const robot_controller::msg::RobotCommand::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void start_fire();
  void timer_callback();
};

#endif  // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
