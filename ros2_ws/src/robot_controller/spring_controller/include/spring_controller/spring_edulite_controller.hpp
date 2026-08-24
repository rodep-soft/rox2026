#ifndef SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
#define SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/srv/set_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/spring_operation_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringEduliteController : public rclcpp::Node {
public:
  SpringEduliteController();

private:
  enum class State : uint8_t {
    UNINITIALIZED,
    HOMING,
    WAITING_FOR_STOP,
    MOVING_TO_STANDBY,
    READY,
    FIRING,
    SLOW_FIRING_EXTENDING,
    SLOW_FIRING_RETURNING,
    ERROR,
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
  void enter_error_with_position_hold(double current_position_rad,
                                      const char *reason);
  void publish_target(double target_rad, bool force = false);
  void publish_operation_state();
  bool update_settled(const actuator_msgs::msg::ActuatorState &feedback);

  State state_{State::UNINITIALIZED};
  bool emergency_stop_active_{true};
  bool fire_request_active_{false};
  bool slow_fire_request_active_{false};
  bool limit_switch_active_{false};
  bool actuator_ready_{false};
  bool position_reference_set_{false};
  bool zero_service_pending_{false};
  bool actuator_position_received_{false};
  bool homing_required_{true};
  bool belt_clearance_requested_{false};
  bool belt_clearance_request_pending_{false};

  int limit_switch_bit_offset_{0};
  int limit_switch_debounce_count_{0};
  int command_period_ms_{10};
  int stable_feedback_count_{0};
  int required_stable_feedback_count_{3};

  double standby_offset_rad_{0.0};
  double position_tolerance_rad_{0.05};
  double fire_increment_rad_{-6.283185307};
  double slow_fire_target_position_rad_{13.5};
  double slow_fire_base_velocity_rad_s_{12.0};
  double slow_fire_velocity_gain_rad_per_m_{0.0};
  double slow_fire_min_velocity_rad_s_{1.0};
  double slow_fire_max_velocity_rad_s_{20.0};
  double slow_fire_settle_timeout_sec_{3.0};
  double slow_fire_return_velocity_rad_s_{6.0};
  double homing_velocity_rad_s_{0.5};
  double homing_timeout_sec_{30.0};
  double stopped_velocity_threshold_rad_s_{0.05};
  double target_position_rad_{0.0};
  double slow_fire_base_rad_{0.0};
  double slow_fire_peak_rad_{0.0};
  double actuator_position_rad_{0.0};
  double belt_clearance_position_rad_{0.0};
  double belt_clearance_return_position_rad_{0.0};
  double commanded_forward_speed_m_s_{0.0};
  double cmd_vel_timeout_sec_{0.2};
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  std::optional<double> last_published_target_rad_;
  uint8_t last_published_operation_state_{255};
  uint16_t logical_id_{4};

  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Time homing_start_time_;
  rclcpp::Time slow_fire_phase_start_time_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      params_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actuator_ready_pub_;
  rclcpp::Publisher<robot_msgs::msg::SpringOperationState>::SharedPtr
      operation_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slow_fire_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      belt_clearance_request_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limit_switch_sub_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorState>::SharedPtr
      actuator_state_sub_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorTarget>::SharedPtr
      position_command_pub_;
  rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedPtr
      set_position_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif // SPRING_CONTROLLER__SPRING_EDULITE_CONTROLLER_HPP_
