#ifndef DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
#define DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_

#include <cstdint>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/shot_cycle_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

class DribbleControllerNode : public rclcpp::Node
{
public:
  DribbleControllerNode();

private:
  void load_parameters();

  void position_mode_callback(const robot_msgs::msg::ArmPosition::SharedPtr msg);
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void dribble_reverse_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void spring_decel_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void opening_rpm_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void actuator_state_callback(const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void vesc_state_callback(const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void control_timer_callback();
  void publish_shot_cycle_state();
  int roller_target_rpm() const;
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  double target_position_rad() const;
  double interpolated_position_rad(
    double start_rad, double target_rad, double elapsed_sec, double max_vel_rad_s,
    double accel_factor = 1.0) const;
  double transition_duration_sec(
    double start_rad, double target_rad, double max_vel_rad_s, double accel_factor = 1.0) const;

  // ── パラメータ ──────────────────────────────────────
  bool enable_receive_state_{true};
  double receive_position_rad_{0.0};
  double dribble_position_rad_{0.35};
  double open_position_rad_{-1.0};
  double bottom_position_rad_{0.0};
  double feed_position_rad_{1.3};
  double open_duration_sec_{0.3};
  double feed_duration_sec_{0.6};
  double opening_max_velocity_rad_s_{4.0};
  double feeding_max_velocity_rad_s_{6.0};
  double returning_max_velocity_rad_s_{4.0};
  double dribbling_max_velocity_rad_s_{1.0};
  double receiving_max_velocity_rad_s_{1.0};
  double opening_accel_factor_{1.8};
  double dribbling_accel_factor_{1.5};
  double receiving_accel_factor_{1.5};
  double ball_detection_threshold_a_{1.7};
  double ball_lost_threshold_a_{1.0};
  double current_lpf_alpha_{0.3};
  int dribble_on_rpm_{800};
  int dribble_receive_rpm_{500};
  int dribble_reverse_rpm_{800};
  double dribble_reverse_ramp_sec_{2.0};
  int spring_fire_dribble_rpm_{600};
  int shot_cycle_opening_rpm_{800};
  int shot_cycle_feeding_rpm_{500};
  int shot_cycle_returning_rpm_{800};
  uint8_t shot_cycle_belt_spinup_level_{1};
  double belt_spinup_delay_sec_{0.5};
  uint16_t position_logical_id_{5};
  uint16_t roller_logical_id_{12};
  uint16_t upper_belt_logical_id_{10};
  uint16_t under_belt_logical_id_{11};

  // ── 状態変数 ────────────────────────────────────────
  uint8_t position_mode_{robot_msgs::msg::ArmPosition::RECEIVE};
  bool dribble_enabled_{false};
  bool dribble_reverse_enabled_{false};
  bool reverse_transition_active_{false};
  rclcpp::Time reverse_transition_start_time_;
  int reverse_transition_start_rpm_{0};
  bool spring_decel_active_{false};
  bool emergency_stop_active_{false};

  bool manual_transition_active_{false};
  rclcpp::Time manual_transition_start_time_;
  double manual_transition_start_position_rad_{0.0};
  int manual_transition_start_rpm_{0};

  bool shot_cycle_active_{false};
  uint8_t shot_cycle_phase_{robot_msgs::msg::ShotCycleState::OPENING};
  rclcpp::Time shot_cycle_start_time_;
  double shot_cycle_start_position_rad_{0.0};
  double last_position_command_rad_{0.0};
  double current_arm_position_rad_{0.0};
  float upper_belt_measured_rpm_{0.0f};
  float under_belt_measured_rpm_{0.0f};
  float roller_measured_rpm_{0.0f};
  float upper_belt_min_shot_rpm_{99999.0f};
  float under_belt_min_shot_rpm_{99999.0f};

  uint8_t current_belt_mode_{robot_msgs::msg::BeltMode::STOP};
  int current_filtered_roller_rpm_{0};
  bool belt_auto_started_{false};
  bool has_ball_{false};
  double filtered_roller_current_a_{0.0};
  bool roller_current_initialized_{false};
  int ball_detected_counter_{0};
  int ball_lost_counter_{0};
  int ball_detection_debounce_count_{3};  // 連続3回(約600ms)の判定で確実化

  // ── ROS インタフェース ──────────────────────────────
  rclcpp::Subscription<robot_msgs::msg::ArmPosition>::SharedPtr position_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_reverse_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spring_decel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr opening_rpm_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr actuator_state_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr vesc_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr position_command_pub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr roller_command_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<robot_msgs::msg::ShotCycleState>::SharedPtr shot_cycle_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_detected_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif // DRIBBLE_CONTROLLER__DRIBBLE_CONTROLLER_HPP_
