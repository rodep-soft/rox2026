#pragma once

#include <cstdint>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
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

  // send_period_ms_ごとに呼ばれ、最後に受け取ったloc_refを送り続ける。
  void TimerCallback();

  // Type 3 Motor enabled to run
  void SendEnable();
  // Type 4 Motor stop
  void SendDisable();
  // Type 6 Set motor mechanical zero: 01 00 00 00 00 00 00 00
  void SendSetMechanicalZero();
  // Type 18 run_mode (index 0x7005): 05 70 00 00 <mode> 00 00 00
  void SendRunMode(uint8_t run_mode);

  void SendPositionCurrentLimit();
  void SendPositionCurrentLimit(float current_limit);
  void SendPositionMaxVelocity();
  void SendPositionMaxVelocity(float velocity);
  void SendPositionAcceleration();
  void SendPositionAcceleration(float acceleration);
  void SendZeroPositionFlag();
  void SendFloatParameter(uint16_t index, float value);
  void SendPositionReference(float position);

  static double Clamp(double value, double min_value, double max_value);

  std::string can_tx_topic_;

  uint8_t motor_can_id_;
  uint8_t host_can_id_;

  std::string command_topic_;
  int send_period_ms_;
  int startup_inter_frame_ms_;

  double position_min_rad_;
  double position_max_rad_;
  double home_position_rad_;

  bool enable_on_startup_;

  double position_current_limit_;
  double position_velocity_rad_s_;
  double position_acceleration_rad_s2_;
  int shutdown_return_wait_ms_;

  bool startup_completed_;
  double command_target_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};
