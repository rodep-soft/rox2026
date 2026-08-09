#ifndef MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
#define MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_

#include <array>
#include <cstdint>

#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class MecanumControllerNode : public rclcpp::Node
{
public:
  MecanumControllerNode();

private:
  enum WheelIndex
  {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
  };

  void configure_parameters();

  /// @brief /mecanum/cmd_vel 受信時に呼ばれる。非有限値はゼロ速度へ置換して publish する。
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /// @brief /emergency_stop 受信時に呼ばれる。全輪ゼロ or 最新 cmd_vel に基づく指令を publish する。
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  /// @brief 最新 cmd_vel を逆運動学で4輪速度へ変換して publish する。
  /// 上限超過時は全輪を同率で縮小する。非常停止中は全輪ゼロ。
  void publish_wheel_commands();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr target_array_pub_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  bool emergency_stop_active_{false};

  // ロボット機構パラメータ
  double wheel_radius_m_{0.075};
  double robot_length_m_{0.47};
  double robot_width_m_{0.41};
  double max_wheel_velocity_rad_s_{50.0};
  std::array<uint16_t, 4> wheel_logical_ids_{0, 1, 2, 3};

  rclcpp::TimerBase::SharedPtr emergency_stop_timer_;
};

#endif  // MECANUM_CONTROLLER__MECANUM_CONTROLLER_NODE_HPP_
