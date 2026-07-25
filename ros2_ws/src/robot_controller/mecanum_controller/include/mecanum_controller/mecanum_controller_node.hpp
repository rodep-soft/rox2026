#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

class MecanumControllerNode : public rclcpp::Node
{
public:
  MecanumControllerNode();

private:
  enum WheelIndex
  {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
  };

  void declare_parameters();
  void get_parameters();
  void create_interfaces();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4> wheel_velocity_pubs_;

  std::array<double, 4> wheel_vels_{0.0, 0.0, 0.0, 0.0};
  double vx_{0.0};
  double vy_{0.0};
  double wz_{0.0};
  double wheel_radius_{0.0};
  double robot_length_{0.0};
  double robot_width_{0.0};
  std::vector<double> velocity_corrections_;
  double vx_sign_{1.0};
  double vy_sign_{1.0};
  double angular_z_sign_{1.0};
  std::string cmd_vel_topic_;
  std::array<std::string, 4> wheel_velocity_topics_;
  int qos_depth_{1};
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
