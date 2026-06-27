#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <array>

class MecanumControllerNode : public rclcpp::Node
{
public:
  MecanumControllerNode();

private:
  void declare_parameters();
  void get_parameters();

  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void motor_init();
  void motor_enable();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_vel_pub_;

  enum WheelIndex
  {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
  };
  std::array<double, 4> wheel_vels = {0.0, 0.0, 0.0, 0.0};

  double vx, vy, wz;

  double wheel_radius;
  double robot_length;
  double robot_width;
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
