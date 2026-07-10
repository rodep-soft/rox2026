#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class RobstrideCanNode : public rclcpp::Node
{
public:
  RobstrideCanNode();

  // ノード終了時の安全停止。contextが有効なうち(shutdownの前)にmainから呼ぶ想定。
  // on_shutdownからはcontext無効化済みでpublishが失敗するため呼ばない。
  void SendStop();

private:
  void DeclareParameters();
  void GetParameters();

  void CommandCallback(const std_msgs::msg::Float32::SharedPtr msg);
  // send_period_ms_ごとに呼ばれ、最後に受け取ったloc_refを送り続ける。
  void TimerCallback();

  // Communication Type 3 (Motor enabled to run) を送信する。
  void SendEnable();
  // Communication Type 4 (Motor stop) を送信し、モーターを無効化(トルク出力オフ)する。
  void SendDisable();
  // Communication Type 18でrun_mode(index 0x7005)を書き込む。
  void SendRunMode(uint8_t run_mode);
  // position mode用の起動時速度/加速度パラメータを書き込む。
  void SendPositionStartupParameters();
  // Communication Type 18で任意のfloatパラメータ(index指定)を書き込む共通ヘルパー。
  void SendFloatParam(uint16_t index, float value);
  // 29bit拡張CAN IDを組み立ててcan_publisher_へpublishする最終出力経路。
  void PublishFrame(uint8_t comm_type, uint16_t data_area2, const std::array<uint8_t, 8> & data);

  static double Clamp(double value, double min_value, double max_value);

  std::string can_tx_topic_;

  uint8_t motor_can_id_;
  uint8_t host_can_id_;

  std::string command_topic_;

  int send_period_ms_;

  double position_min_rad_;
  double position_max_rad_;

  bool enable_on_startup_;

  double position_speed_;
  double position_acceleration_;

  bool startup_completed_;
  double command_target_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};
