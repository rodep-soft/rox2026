#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class MecanumControllerNode : public rclcpp::Node
{
 public:
  MecanumControllerNode();

 private:
  enum class OperationMode : uint8_t
  {
    STOP,
    DRIVE,
    SHOT_CYCLE,
    BELT_ONLY,
  };

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

  // /mecanum/cmd_vel受信時に呼ばれる。非有限値ならゼロ速度へ置換し、どちらの場合も4輪指令をpublishする。
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // /operation_mode受信時に呼ばれる。不正値はSTOPにして4輪指令を再publishする。
  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  // /emergency_stop受信時に呼ばれる。trueなら全輪ゼロ、falseなら最新cmd_velに基づく指令をpublishする。
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // 最新cmd_velを符号補正後に逆運動学で4輪速度へ変換する。STOP/非常停止は全軸ゼロ、
  // SHOT_CYCLE/BELT_ONLYは並進だけゼロにする。上限超過時は全輪を同率で縮小してpublishする。
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
  double max_wheel_velocity_rad_s_{50.0};
  std::vector<double> velocity_corrections_;
  double vx_sign_{1.0};
  double vy_sign_{1.0};
  double angular_z_sign_{1.0};
  int qos_depth_{1};
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
