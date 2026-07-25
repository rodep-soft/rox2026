#pragma once

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class DribblePositionController : public rclcpp::Node
{
public:
  DribblePositionController();

private:
  // /dribble/position_modeで受け付ける指令。joy_controllerのボタンに対応する。
  static constexpr uint8_t dribble_mode_{0};
  static constexpr uint8_t shoot_mode_{1};

  void declare_parameters();
  void get_parameters();
  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void timer_callback();
  void publish_target_position(double position_rad);

  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  // SHOOT指令でINTAKE位置へ動かしてから、SHOOT位置へ進めるまでの待ち時間[s]。
  double intake_to_shoot_delay_sec_{1.0};
  int command_period_ms_{20};
  int qos_depth_{1};

  bool shoot_pending_{false};  // SHOOT指令後、SHOOT位置への遷移待ちかどうか。
  rclcpp::Time intake_start_time_;  // INTAKE位置指令を出した時刻。

  std::string dribble_position_command_topic_;
  std::string dribble_position_mode_topic_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
