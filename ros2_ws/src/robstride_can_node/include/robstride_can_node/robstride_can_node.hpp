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

  // ノード終了時の安全停止。velocity modeならspd_ref=0を送ってから無効化(Type 4)する。
  // contextが有効なうち(shutdownの前)にmainから呼ぶ想定。on_shutdownからは
  // context無効化済みでpublishが失敗するため呼ばない。
  void SendStop();

private:
  // control_mode_はconfig/launchで固定する起動時パラメータ。実行中に切り替えることは想定せず、
  // position用/velocity用それぞれの専用launchファイルでノードを起動し分ける設計。
  enum class ControlMode
  {
    Position,
    Velocity,
  };

  void DeclareParameters();
  void GetParameters();
  void SetupRosInterfaces();

  void CommandCallback(const std_msgs::msg::Float32::SharedPtr msg);
  // send_period_ms_ごとに呼ばれ、modeに応じてloc_ref/spd_refを送り続ける
  // (velocity modeではtimeout_ms_超過時にspd_ref=0へ差し替える安全停止も担う)。
  void TimerCallback();

  // Communication Type 3 (Motor enabled to run) を送信する。
  void SendEnable();
  // Communication Type 4 (Motor stop) を送信し、モーターを無効化(トルク出力オフ)する。
  void SendDisable();
  // Communication Type 18でrun_mode(index 0x7005)を書き込む。
  void SendRunMode(uint8_t run_mode);
  // limit_torque_/limit_cur_/limit_spd_のうち値が設定されている(0.0以上の)ものだけ送信する。
  void SendStartupLimits();
  // Communication Type 18で任意のfloatパラメータ(index指定)を書き込む共通ヘルパー。
  void SendFloatParam(uint16_t index, float value);
  // 29bit拡張CAN IDを組み立ててcan_publisher_へpublishする最終出力経路。
  void PublishFrame(uint8_t comm_type, uint16_t data_area2, const std::array<uint8_t, 8> & data);

  static double Clamp(double value, double min_value, double max_value);
  // 不明な文字列はvelocity modeにフォールバックする(ParseControlModeの実装コメント参照)。
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
