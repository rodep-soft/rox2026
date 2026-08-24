#ifndef DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
#define DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/shot_cycle_state.hpp"
#include "robot_msgs/msg/spring_operation_state.hpp"
#include "std_msgs/msg/bool.hpp"

class DribbleControllerNode : public rclcpp::Node {
public:
  DribbleControllerNode();

private:
  enum class ParameterConstraint {
    NONE,
    NONNEGATIVE,
    POSITIVE,
    UNIT_INTERVAL,
    BELT_LEVEL,
  };
  using ParameterValue = std::variant<bool *, int *, double *>;
  struct ParameterBinding {
    const char *name;
    ParameterValue value;
    ParameterConstraint constraint;
    bool affects_trajectory;
  };
  std::vector<ParameterBinding> parameter_bindings();
  static bool parameter_value_is_valid(ParameterConstraint constraint,
                                       double value);
  void declare_binding(ParameterBinding &binding);
  static bool apply_binding(const ParameterBinding &binding,
                            const rclcpp::Parameter &parameter);
  void restart_active_trajectory();
  void load_parameters();

  void
  position_mode_callback(const robot_msgs::msg::ArmPosition::SharedPtr msg);
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void start_shot_cycle();
  void publish_belt_clearance_request(bool requested);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void edulite_state_callback(
      const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void
  vesc_state_callback(const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void spring_operation_state_callback(
      const robot_msgs::msg::SpringOperationState::SharedPtr msg);
  void control_timer_callback();
  void update_motion_compensation();
  void update_and_publish_roller_command();
  double update_manual_position_command(double position_command_rad);
  void publish_position_command(double position_rad);
  void publish_shot_cycle_state();
  int roller_target_rpm() const;
  rcl_interfaces::msg::SetParametersResult
  parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

  double target_position_rad() const;
  double manual_transition_max_velocity_rad_s() const;
  double manual_transition_max_acceleration_rad_s2() const;
  struct TrajectorySample {
    double position_rad;
    double duration_sec;
  };
  TrajectorySample sample_trajectory(double start_rad, double target_rad,
                                     double elapsed_sec,
                                     double max_velocity_rad_s,
                                     double max_acceleration_rad_s2) const;

  // ── パラメータ ──────────────────────────────────────
  double dribble_position_rad_{-0.86};
  double open_position_rad_{-1.27};
  double bottom_position_rad_{0.0};
  double feed_position_rad_{1.3};
  double feed_duration_sec_{0.6};
  double opening_max_velocity_rad_s_{4.0};
  double feeding_max_velocity_rad_s_{6.0};
  double returning_max_velocity_rad_s_{4.0};
  double dribbling_max_velocity_rad_s_{3.0};
  double opening_max_acceleration_rad_s2_{15.0};
  double feeding_max_acceleration_rad_s2_{15.0};
  double returning_max_acceleration_rad_s2_{18.0};
  double dribbling_max_acceleration_rad_s2_{12.0};
  double ball_detection_threshold_a_{4.5};
  double ball_lost_threshold_a_{2.2};
  double current_lpf_alpha_{0.07};
  int dribble_on_rpm_{400};
  int shot_cycle_opening_rpm_{800};
  int shot_cycle_feeding_rpm_{500};
  int shot_cycle_returning_rpm_{800};
  int shot_cycle_belt_spinup_level_{1};
  double belt_shot_delay_sec_{0.0};
  double prepare_from_open_delay_sec_{0.1};
  double slow_fire_dribble_position_rad_{-0.8};
  int slow_fire_dribble_rpm_{-500};
  uint16_t position_logical_id_{5};
  uint16_t roller_logical_id_{12};
  uint16_t upper_belt_logical_id_{10};
  uint16_t under_belt_logical_id_{11};

  // ── 運動補正パラメータ (後退・急減速時のボール安定化) ──
  bool enable_motion_compensation_{true};
  double backward_velocity_boost_rpm_per_mps_{500.0};
  double backward_acceleration_rpm_per_mps2_{200.0};
  double cmd_vel_acceleration_lpf_alpha_{0.2};
  double cmd_vel_timeout_sec_{0.2};
  int max_boost_rpm_{1200};
  std::string cmd_vel_topic_{"/mecanum/cmd_vel_heading"};

  // ── 状態変数 ────────────────────────────────────────
  uint8_t position_mode_{robot_msgs::msg::ArmPosition::DRIBBLE};
  bool dribble_enabled_{false};
  bool emergency_stop_active_{false};
  bool edulite_state_received_{false};
  bool edulite_ready_{false};
  bool startup_waiting_for_emergency_release_{true};
  bool startup_emergency_seen_active_{false};
  double emergency_hold_position_rad_{0.0};

  // 運動補正用の一時変数
  double commanded_vx_m_s_{0.0};
  double commanded_ax_m_s2_{0.0};
  double last_commanded_vx_m_s_{0.0};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  int current_motion_boost_rpm_{0};

  bool manual_transition_active_{false};
  rclcpp::Time manual_transition_start_time_;
  double manual_transition_start_position_rad_{0.0};
  int manual_transition_start_rpm_{0};

  bool shot_cycle_active_{false};
  bool shot_prepare_from_open_{false};
  bool shot_prepare_delay_started_{false};
  rclcpp::Time shot_prepare_start_time_;
  uint8_t spring_operation_state_{robot_msgs::msg::SpringOperationState::IDLE};
  uint8_t shot_cycle_phase_{robot_msgs::msg::ShotCycleState::FEEDING};
  uint8_t last_published_shot_cycle_state_{0xFF};
  rclcpp::Time shot_cycle_start_time_;
  double shot_cycle_start_position_rad_{0.0};
  double last_position_command_rad_{-0.86};
  double current_arm_position_rad_{0.0};
  float upper_belt_measured_rpm_{0.0f};
  float under_belt_measured_rpm_{0.0f};

  int current_filtered_roller_rpm_{0};
  bool belt_auto_started_{false};
  bool has_ball_{false};
  double filtered_roller_current_a_{0.0};
  bool roller_current_initialized_{false};
  std::optional<bool> last_published_ball_state_;
  std::optional<double> last_published_position_rad_;
  std::optional<int> last_published_roller_rpm_;
  int ball_detected_counter_{0};
  int ball_lost_counter_{0};
  int ball_detection_debounce_count_{
      12}; // 連続12回(約240ms)の判定で発進・停止時のスパイクを除外
  int ball_lost_debounce_count_{
      12}; // 連続12回(約240ms)の判定で停止時のバウンド誤解除を防止

  // ── ROS インタフェース ──────────────────────────────
  rclcpp::Subscription<robot_msgs::msg::ArmPosition>::SharedPtr
      position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr
      edulite_state_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr
      vesc_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<robot_msgs::msg::SpringOperationState>::SharedPtr
      spring_operation_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr
      position_command_pub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr
      roller_command_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<robot_msgs::msg::ShotCycleState>::SharedPtr
      shot_cycle_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr belt_clearance_request_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
};

#endif // DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
