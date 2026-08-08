#ifndef ARM_POSITION_CONTROLLER__ARM_POSITION_CONTROLLER_HPP_
#define ARM_POSITION_CONTROLLER__ARM_POSITION_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "actuator_msgs/msg/actuator_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class ArmPositionControllerNode : public rclcpp::Node
{
public:
  ArmPositionControllerNode();

private:
  enum class PositionMode : uint8_t
  {
    DRIBBLE = 0,
    OPEN = 1,
    FEED = 2,
  };

  /// @brief 自動シュートサイクルの進行フェーズ
  enum class ShotCyclePhase : uint8_t
  {
    OPENING = 0,  ///< OPEN 位置へ移動中
    FEEDING = 1,  ///< FEED 位置へ押し込み中
  };

  void declare_parameters();
  void get_parameters();

  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();

  /// @brief PositionMode に対応する文字列表現を返す
  const char * mode_name(PositionMode mode) const;
  /// @brief 現在モードに対応する目標位置 [rad] を返す
  float target_position_rad() const;

  // ── パラメータ ──────────────────────────────────────
  double dribble_position_rad_{0.35};
  double open_position_rad_{-1.0};
  double feed_position_rad_{1.3};
  double open_duration_sec_{0.3};  ///< OPEN 姿勢を保持する時間 [s]
  double feed_duration_sec_{0.6};  ///< FEED 押し込みを保持する時間 [s]
  int command_period_ms_{20};
  int qos_depth_{1};
  int logical_id_{5};
  std::string target_topic_{"/edulite/target"};

  // ── 状態変数 ────────────────────────────────────────
  PositionMode current_position_mode_{PositionMode::DRIBBLE};
  bool emergency_stop_active_{false};

  bool in_shot_cycle_{false};
  ShotCyclePhase shot_cycle_phase_{ShotCyclePhase::OPENING};
  rclcpp::Time shot_cycle_start_time_;

  // ── ROS インタフェース ──────────────────────────────
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr position_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // ARM_POSITION_CONTROLLER__ARM_POSITION_CONTROLLER_HPP_
