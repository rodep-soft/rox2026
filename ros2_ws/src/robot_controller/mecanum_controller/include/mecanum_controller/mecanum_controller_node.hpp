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

  /// @brief /mecanum/cmd_vel 受信時に呼ばれる。非有限値はゼロ速度へ置換して publish する。
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /// @brief /emergency_stop 受信時に呼ばれる。全輪ゼロ or 最新 cmd_vel に基づく指令を publish する。
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  /// @brief 最新 cmd_vel を逆運動学で4輪速度へ変換して publish する。
  /// 上限超過時は全輪を同率で縮小する。非常停止中は全輪ゼロ。
  void publish_wheel_commands();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 4> wheel_velocity_pubs_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  bool emergency_stop_active_{false};

  // ロボット機構パラメータ
  double wheel_radius_{0.05};
  double robot_length_{0.47};
  double robot_width_{0.41};
  double max_wheel_velocity_rad_s_{50.0};
  std::vector<double> velocity_corrections_;
  double vx_sign_{1.0};
  double vy_sign_{1.0};
  double angular_z_sign_{1.0};
  int command_period_ms_{20};
  int qos_depth_{1};

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
