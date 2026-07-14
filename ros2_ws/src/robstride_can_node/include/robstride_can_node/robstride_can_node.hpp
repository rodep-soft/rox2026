#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

class RobstrideCanNode : public rclcpp::Node
{
public:
  RobstrideCanNode();

  // home_position_rad_へ復帰させた後に停止する
  void SendStop();

private:
  void DeclareParameters();
  void GetParameters();

  void CommandCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // roller_angle_controller等から「起動シーケンスを送っていいか」の指示を受け取る。
  void EnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // send_period_ms_ごとに呼ばれ、最後に受け取ったloc_refを送り続ける。
  void TimerCallback();

  // Type 3 Motor enabled to run
  void SendEnable();
  // Type 4 Motor stop
  void SendDisable();
  // Type 18 run_mode (index 0x7005)
  void SendRunMode(uint8_t run_mode);

  // position mode用の起動時速度/加速度パラメータを書き込む
  void SendPositionStartupParameters();
  // PP/速度モード用の電流上限を書き込み
  void SendPositionCurrentLimit();
  // Type 18で任意のfloatパラメータ
  void SendFloatParam(uint16_t index, float value);
  // 29bit拡張CAN IDを組み立ててcan_publisher_へpublishする
  void PublishFrame(uint8_t comm_type, uint16_t data_area2, const std::array<uint8_t, 8> & data);

  static double Clamp(double value, double min_value, double max_value);

  std::string can_tx_topic_;

  uint8_t motor_can_id_;
  uint8_t host_can_id_;

  std::string command_topic_;
  std::string enable_topic_;

  int send_period_ms_;
  int startup_inter_frame_ms_;

  double position_min_rad_;
  double position_max_rad_;
  double home_position_rad_;

  bool enable_on_startup_;

  double position_speed_;
  double position_acceleration_;
  double position_current_limit_;
  int shutdown_return_wait_ms_;

  bool startup_completed_;
  bool enable_requested_;
  double command_target_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};
