#pragma once

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class DribblePositionController : public rclcpp::Node
{
public:
  DribblePositionController();

private:
  enum class Position : uint8_t
  {
    DRIBBLE,
    INTAKE,
    SHOOT,
    OPEN,
  };

  enum class State : uint8_t
  {
    IDLE,
    MANUAL_MOVE,
    INTAKE,
    SHOOT,
    HOLD_SHOOT,
    RETURN_TO_DRIBBLE
  };

  enum class OperationMode : uint8_t
  {
    STOP,
    DRIVE,
    SHOT_CYCLE,
    BELT_ONLY,
  };

  void declare_parameters();
  void get_parameters();
  void validate_parameters();

  // /dribble/position_mode受信時に呼ばれる。非常停止中、cycle実行中、IDLE以外、
  // DRIVE/SHOT_CYCLE以外では無視する。許可時は選択位置を/position_commandへpublishする。
  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  // /operation_mode受信時に呼ばれる。STOPはcycleを中断してDRIBBLEへ、BELT_ONLYはOPENへ移動する。
  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  // /shot_cycle/startのtrue受信時に呼ばれる。設定正常・非常停止なし・SHOT_CYCLE・IDLE・未実行の
  // 全条件を満たす場合だけINTAKEへ進み、/shot_cycle/runningへtrueをpublishする。
  void shot_cycle_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // /dribble/position_feedback受信時に目標到達を判定する。INTAKE→SHOOT→HOLD→DRIBBLEを遷移し、
  // DRIBBLE復帰完了時だけ/running=falseと/complete=trueをpublishする。
  void position_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // 設定周期で呼ばれる。IDLE以外で/position_commandを再送し、feedbackまたは移動のtimeoutなら
  // cycleを中断してDRIBBLEへ戻す。HOLD_SHOOTの保持時間後にもDRIBBLE復帰を開始する。
  void watchdog_callback();

  // target/state/時刻を更新して/position_commandを直ちにpublishする。非有限値または設定不正時は拒否する。
  void set_target_position(double position_rad, State state);
  void stop_shot_cycle();
  void publish_shot_cycle_running(bool running);
  void publish_shot_cycle_complete();
  // 非常停止でなく、IDLE、cycle未実行、かつDRIVEまたはSHOT_CYCLEのときだけtrueを返す。
  bool manual_position_allowed() const;
  const char * state_name(State state) const;
  void log_shot_cycle_start_rejection() const;

  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  double open_position_rad_{0.0};
  double position_tolerance_rad_{0.02};
  double shoot_to_dribble_delay_sec_{1.0};
  double move_timeout_sec_{3.0};
  double feedback_timeout_sec_{0.5};
  int command_period_ms_{20};
  int qos_depth_{1};
  double target_position_rad_{0.0};
  double last_feedback_position_rad_{0.0};
  State state_{State::IDLE};
  OperationMode operation_mode_{OperationMode::STOP};
  bool configuration_valid_{true};
  bool emergency_stop_active_{false};
  bool shot_cycle_running_{false};
  bool has_position_feedback_{false};
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_feedback_time_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_command_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr operation_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    position_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shot_cycle_running_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shot_cycle_complete_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};
