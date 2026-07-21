#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_controller/msg/robot_command.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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
  void robot_command_callback(const robot_controller::msg::RobotCommand::SharedPtr msg);

  rclcpp::Subscription<robot_controller::msg::RobotCommand>::SharedPtr robot_command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_velocity_pub_;

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
  std::string robot_command_topic_;
  std::string wheel_velocity_command_topic_;
  int qos_depth_{10};
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
