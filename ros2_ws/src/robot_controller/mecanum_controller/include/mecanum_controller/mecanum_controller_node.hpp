#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MecanumControllerNode : public rclcpp::Node {
 public:
  MecanumControllerNode();

 private:
  enum class OperationMode : uint8_t {
    STOP = 0,
    DRIVE,
    INTAKE_AND_SHOOT,
    GAME2_MODE,
  };

  enum WheelIndex {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3,
  };

  void declare_parameters();
  void get_parameters();
  void create_interfaces();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_wheel_commands();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr operation_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4>
      wheel_velocity_pubs_;

  std::array<double, 4> wheel_vels_{0.0, 0.0, 0.0, 0.0};
  geometry_msgs::msg::Twist last_cmd_vel_;
  OperationMode operation_mode_{OperationMode::STOP};
  bool emergency_stop_active_{false};
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
  std::string operation_mode_topic_;
  std::string emergency_stop_topic_;
  std::array<std::string, 4> wheel_velocity_topics_;
  int qos_depth_{1};
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
