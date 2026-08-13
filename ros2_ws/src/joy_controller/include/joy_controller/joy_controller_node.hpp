#ifndef JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
#define JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "joy_controller/slew_rate_limiter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"

class JoyControllerNode : public rclcpp::Node
{
public:
  JoyControllerNode();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void loop_callback();
  void joy_timeout_timer_callback();
  void state_publish_timer_callback();
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  void spring_actuator_ready_callback(
    const std_msgs::msg::Bool::SharedPtr msg);
  void publish_emergency_stop(bool active);
  void publish_belt_mode(uint8_t mode);
  void publish_dribble_enabled(bool enabled);
  void publish_opening_rpm(int rpm);
  void publish_drive_reversed(bool reversed);
  void publish_stop_commands();
  void publish_limited_velocity(double target_x_m_s, double target_y_m_s, double target_yaw_rad_s);
  void update_acceleration_limits();

  bool is_button_down(const sensor_msgs::msg::Joy & msg, int index) const;
  bool is_button_just_pressed(const sensor_msgs::msg::Joy & msg, int index) const;
  double get_axis_value(const sensor_msgs::msg::Joy & msg, int index) const;
  bool is_axis_just_triggered(
    const sensor_msgs::msg::Joy & msg, int index, bool positive) const;

  double apply_axis_deadzone(double value) const;
  static uint8_t increment_mode(uint8_t mode, uint8_t maximum_mode);
  static uint8_t decrement_mode(uint8_t mode);

  int command_qos_depth_{1};
  int joy_timeout_ms_{200};
  int state_publish_period_ms_{20};

  double max_vel_x_m_s_{2.0};
  double max_vel_y_m_s_{2.0};
  double max_vel_z_rad_s_{2.0};
  double acceleration_x_m_s2_{2.0};
  double acceleration_y_m_s2_{2.0};
  double acceleration_yaw_rad_s2_{4.0};
  double deceleration_x_m_s2_{3.0};
  double deceleration_y_m_s2_{3.0};
  double deceleration_yaw_rad_s2_{6.0};
  double axis_deadzone_{0.05};
  double axis_on_threshold_{0.7};

  int ps_button_{12};
  int home_button_{13};
  int circle_button_{2};
  int dribble_enable_button_{5};
  int game2_start_button_{9};
  int left_trigger_axis_{3};
  int right_trigger_axis_{4};
  int left_stick_x_axis_{0};
  int left_stick_y_axis_{1};
  int right_stick_x_axis_{2};
  int dpad_horizontal_axis_{6};
  int dpad_vertical_axis_{7};

  geometry_msgs::msg::Twist cmd_vel_;
  joy_controller::SlewRateLimiter velocity_limiter_x_{2.0, 3.0};
  joy_controller::SlewRateLimiter velocity_limiter_y_{2.0, 3.0};
  joy_controller::SlewRateLimiter velocity_limiter_yaw_{4.0, 6.0};
  std::chrono::steady_clock::time_point last_velocity_update_time_{};
  bool velocity_limiter_initialized_{false};

  bool is_emergency_stop_{true};
  uint8_t belt_rpm_mode_{robot_msgs::msg::BeltMode::STOP};
  int shot_cycle_opening_rpm_{1500};
  bool dribble_enabled_{false};
  bool dribble_enabled_before_spring_{false};
  bool was_spring_ready_{false};
  bool game2_active_{false};
  bool is_drive_reversed_{false};
  bool joy_received_{false};
  bool joy_timeout_active_{false};
  bool spring_actuator_ready_{false};
  std::chrono::steady_clock::time_point last_joy_received_time_{};

  std::optional<sensor_msgs::msg::Joy> last_joy_msg_{};
  sensor_msgs::msg::Joy joy_msg_{};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mecanum_cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shot_cycle_request_pub_;
  rclcpp::Publisher<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr opening_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr game2_start_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spring_actuator_ready_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_reversed_pub_;
  rclcpp::TimerBase::SharedPtr joy_timeout_timer_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

#endif  // JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
