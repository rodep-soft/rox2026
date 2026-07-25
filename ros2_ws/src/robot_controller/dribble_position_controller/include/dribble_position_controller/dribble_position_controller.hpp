#pragma once

#include <chrono>
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

  // SHOOTシーケンスの進行状態。
  enum class State : uint8_t
  {
    IDLE,          // 待機(位置を保持しているだけ)。
    WAIT_SHOOT,    // INTAKE位置へ移動済み、SHOOT位置への遷移待ち。
    WAIT_RETURN,   // SHOOT位置へ移動済み、DRIBBLE位置への復帰待ち。
  };

  void declare_parameters();
  void get_parameters();
  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void timer_callback();
  void set_target_position(double position_rad);

  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  // SHOOT指令でINTAKE位置へ動かしてから、SHOOT位置へ進めるまでの待ち時間[s]。
  double intake_to_shoot_delay_sec_{1.0};
  // SHOOT位置へ動かしてから、DRIBBLE位置へ戻すまでの待ち時間[s]。
  double shoot_to_dribble_delay_sec_{1.0};
  int command_period_ms_{20};
  int qos_depth_{1};

  double target_position_rad_{0.0};  // タイマーで常時publishする現在の目標位置。
  State state_{State::IDLE};
  std::chrono::steady_clock::time_point phase_start_time_;  // 現在のフェーズを開始した時刻。

  std::string dribble_position_command_topic_;
  std::string dribble_position_mode_topic_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
