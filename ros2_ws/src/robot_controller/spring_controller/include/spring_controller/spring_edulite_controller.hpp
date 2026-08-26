#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <cstdint>
#include <string>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/srv/set_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/spring_operation_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringEduliteController : public rclcpp::Node
{
public:
  SpringEduliteController();

private:
  enum class State : uint8_t
  {
    WAITING_FOR_ACTUATOR_READY, //driverからのREADYを待つ状態
    WAITING_FOR_HOMING,         // HOMINGの開始を待つ状態
    HOMING,                     // HOMING中
    WAITING_FOR_STOP,           // 停止待ち　
    MOVING_TO_STANDBY,          // 機構の動作準備中
    READY,                      //機構の動作準備完了
    FIRING,                     //ばね発射中
    SLOW_FIRING_EXTENDING,      // スロー発射進行中
    SLOW_FIRING_RETURNING,      // スロー発射からもとに位置に帰還中
    SLOW_FIRE_ARM_ONLY,         // スロー発射のアームだけの動作中
    ERROR,                      // driver側のエラー
  };

  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void slow_fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void
  belt_clearance_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void start_belt_clearance_motion();
  void finish_belt_clearance_motion();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void limit_switch_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void actuator_state_callback(
    const actuator_msgs::msg::ActuatorState::SharedPtr msg);
  void control_timer_callback();

  void start_homing();
  void request_zero_reference();
  void enter_error_with_position_hold(
    double current_position_rad,
    const char * reason);
  void publish_target(double target_rad);
  void publish_operation_state();
  bool update_settled(const actuator_msgs::msg::ActuatorState & feedback);

  State state_{State::WAITING_FOR_ACTUATOR_READY};
  bool e_stop_active_{true};
  bool resume_after_e_stop_{false};
  bool fire_req_active_{false};
  bool fire_req_pending_{false};
  bool slow_fire_req_active_{false};
  bool slow_fire_req_pending_{false};
  bool slow_fire_move_spring_{true};
  bool limit_sw_active_{false};
  bool actuator_ready_{false};
  bool position_ref_set_{false};
  bool zero_srv_pending_{false};
  bool actuator_pos_received_{false};
  bool homing_required_{true};
  bool belt_clearance_request_active_{false};
  bool is_belt_clearance_active_{false};
  bool new_actuator_fb_{false};
  bool zero_srv_response_received_{false};
  bool zero_srv_succeeded_{false};

  int limit_sw_bit_offset_{0};
  int command_period_ms_{10};
  int stable_fb_count_{0};
  int required_stable_fb_count_{3};

  double standby_offset_rad_{0.0};
  double belt_clearance_ready_travel_rad_{3.0};
  double pos_tolerance_rad_{0.05};
  double fire_increment_rad_{-6.283185307};
  double slow_fire_target_pos_rad_{13.5};
  double slow_fire_base_vel_rad_s_{12.0};
  double slow_fire_vel_gain_rad_per_m_{0.0};
  double slow_fire_min_vel_rad_s_{1.0};
  double slow_fire_max_vel_rad_s_{20.0};
  double slow_fire_settle_timeout_sec_{3.0};
  double slow_fire_arm_only_duration_sec_{0.5};
  double slow_fire_return_vel_rad_s_{6.0};
  double homing_vel_rad_s_{0.5};
  double homing_timeout_sec_{30.0};
  double motion_timeout_sec_{10.0};
  double stopped_vel_threshold_rad_s_{0.05};
  double target_pos_rad_{0.0};
  double slow_fire_base_rad_{0.0};
  double slow_fire_peak_rad_{0.0};
  double actuator_pos_rad_{0.0};
  double actuator_vel_rad_s_{0.0};
  double e_stop_hold_pos_rad_{0.0};
  double belt_clearance_pos_rad_{0.0};
  double belt_clearance_return_pos_rad_{0.0};
  double cmd_forward_vel_m_s_{0.0};
  double cmd_vel_timeout_sec_{0.2};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  uint8_t last_pub_op_state_{255};
  uint8_t actuator_state_{actuator_msgs::msg::ActuatorState::STATE_OFFLINE};
  uint16_t logical_id_{4};

  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Time homing_start_time_;
  rclcpp::Time slow_fire_phase_start_time_;
  rclcpp::Time motion_start_time_;
  std::string zero_srv_response_msg_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    params_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actuator_ready_pub_;
  rclcpp::Publisher<robot_msgs::msg::SpringOperationState>::SharedPtr
    op_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_req_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slow_fire_req_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    belt_clearance_req_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limit_sw_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr
    actuator_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr
    pos_cmd_pub_;
  rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedPtr
    set_pos_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
