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

private:
  // control_mode_はconfig/launchで固定する起動時パラメータ。実行中に切り替えることは想定せず、
  // position用/velocity用それぞれの専用launchファイルでノードを起動し分ける設計。
  enum class ControlMode
  {
    kPosition,
    kVelocity,
  };

  void DeclareParameters();
  void GetParameters();
  void SetupRosInterfaces();

  void CommandCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void TimerCallback();

  void SendEnable();
  void SendRunMode(uint8_t run_mode);
  void SendStartupLimits();
  void SendFloatParam(uint16_t index, float value);
  void PublishFrame(uint8_t comm_type, uint16_t data_area2, const std::array<uint8_t, 8> & data);

  static double Clamp(double value, double min_value, double max_value);
  ControlMode ParseControlMode(const std::string & value) const;

  std::string can_tx_topic_;
  bool is_extended_;

  uint8_t motor_can_id_;
  uint8_t host_can_id_;

  ControlMode control_mode_;
  std::string command_topic_;

  int send_period_ms_;
  int timeout_ms_;

  double position_min_rad_;
  double position_max_rad_;
  double velocity_min_rad_s_;
  double velocity_max_rad_s_;

  bool enable_on_startup_;

  // 負値は「未指定＝モーター側のデフォルト/現在値のまま変更しない」を意味する。
  double limit_torque_;
  double limit_cur_;
  double limit_spd_;

  double command_target_;
  rclcpp::Time last_command_time_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};
