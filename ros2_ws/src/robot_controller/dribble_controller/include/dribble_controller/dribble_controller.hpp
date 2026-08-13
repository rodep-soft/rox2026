#ifndef DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
#define DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <vector>

#include "actuator_msgs/msg/actuator_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class DribbleControllerNode : public rclcpp::Node
{
public:
  DribbleControllerNode();

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
    BELT_SPINUP = 0, ///< belt spin-up 待機中（belt が STOP から自動 ON した場合のみ）
    OPENING = 1,     ///< OPEN 位置へ移動中
    FEEDING = 2,     ///< FEED 位置へ押し込み中
    RETURNING = 3,   ///< DRIBBLE 位置へ復帰中
  };

  void load_parameters();

  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void control_timer_callback();
  void publish_shot_cycle_state();
  int roller_target_rpm() const;
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// @brief 現在モードに対応する目標位置 [rad] を返す
  double target_position_rad() const;
  double interpolated_position_rad(
    double start_rad, double target_rad, double elapsed_sec, double max_vel_rad_s) const;
  double transition_duration_sec(
    double start_rad, double target_rad, double max_vel_rad_s) const;

  // ── パラメータ ──────────────────────────────────────
  double dribble_position_rad_{0.35};
  double open_position_rad_{-1.0};
  double feed_position_rad_{1.3};
  double open_duration_sec_{0.3}; ///< OPEN 姿勢を保持する時間 [s]
  double feed_duration_sec_{0.6}; ///< FEED 押し込みを保持する時間 [s]
  double opening_max_velocity_rad_s_{4.0};
  double feeding_max_velocity_rad_s_{6.0};
  double returning_max_velocity_rad_s_{4.0};
  int dribble_on_rpm_{800};
  int shot_cycle_opening_rpm_{800};
  int shot_cycle_feeding_rpm_{500};
  int shot_cycle_returning_rpm_{800};
  uint8_t shot_cycle_belt_spinup_level_{1}; ///< belt 自動 ON 時のレベル (1–4)
  double belt_spinup_delay_sec_{0.5};       ///< belt STOP→ON 後の spin-up 待機時間 [s]
  uint16_t position_logical_id_{5};
  uint16_t roller_logical_id_{12};

  // ── 状態変数 ────────────────────────────────────────
  PositionMode position_mode_{PositionMode::DRIBBLE};
  bool dribble_enabled_{false};
  bool emergency_stop_active_{false};

  bool manual_transition_active_{false};
  rclcpp::Time manual_transition_start_time_;
  double manual_transition_start_position_rad_{0.35};

  bool shot_cycle_active_{false};
  ShotCyclePhase shot_cycle_phase_{ShotCyclePhase::OPENING};
  rclcpp::Time shot_cycle_start_time_;
  double shot_cycle_start_position_rad_{0.35};
  double last_position_command_rad_{0.35};

  uint8_t current_belt_mode_{0}; ///< /belt/mode の最新値（0=STOP）
  bool belt_auto_started_{false}; ///< shot cycle で belt を自動 ON したか

  // ── ROS インタフェース ──────────────────────────────
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr position_command_pub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr roller_command_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr belt_mode_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr shot_cycle_state_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif // DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
