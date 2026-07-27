#ifndef JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
#define JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class JoyControllerNode : public rclcpp::Node
{
public:
  JoyControllerNode();

private:
  enum class BeltRpmMode : uint8_t
  {
    STOP = 1,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
  };

  enum class DribblePositionMode : uint8_t
  {
    DRIBBLE = 0,
    SHOOT = 1,
    MAX_OPEN = 2,
  };

  void declare_parameters();
  void get_parameters();
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void belt_ready_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void control_timer_callback();
  void process_joy_message(const sensor_msgs::msg::Joy & msg);

  static bool button_pressed(const sensor_msgs::msg::Joy & msg, int index);
  static double axis_value(const sensor_msgs::msg::Joy & msg, int index);
  double apply_axis_deadzone(double value) const;
  static double apply_axis_limit(double value, double limit);
  static uint8_t increment_mode(uint8_t mode, uint8_t maximum_mode);
  static uint8_t decrement_mode(uint8_t mode);

  void publish_dribble_position_mode(DribblePositionMode mode);
  void publish_intake_shoot_request();
  void publish_emergency_stop();
  void publish_state_commands();
  void publish_stop_commands();
  void update_chord_inputs(const sensor_msgs::msg::Joy & msg);
  bool handle_emergency_stop();
  void update_previous_chord_inputs();

  std::string joy_topic_;
  std::string mecanum_cmd_vel_topic_;
  std::string spring_fire_request_topic_;
  std::string belt_mode_topic_;
  std::string dribble_enabled_topic_;
  std::string emergency_stop_topic_;
  std::string dribble_position_mode_topic_;
  std::string intake_shoot_request_topic_;
  std::string belt_ready_topic_;
  int joy_qos_depth_{1};
  int command_qos_depth_{1};
  int joy_timeout_ms_{200};
  int control_period_ms_{10};
  double linear_x_scale_{1.0};
  double linear_y_scale_{1.0};
  double angular_z_scale_{1.0};
  double linear_x_limit_{2.0};
  double linear_y_limit_{2.0};
  double angular_z_limit_{2.0};
  double axis_deadzone_{0.05};
  double axis_on_threshold_{0.7};

  int spring_fire_enable_button_{4};
  int spring_fire_button_{2};
  int create_button_{8};
  int ps_button_{12};
  int options_button_{9};
  int home_button_{13};
  int circle_button_{2};
  int dribble_enable_button_{5};
  int left_trigger_axis_{3};
  int left_stick_x_axis_{0};
  int left_stick_y_axis_{1};
  int right_stick_x_axis_{2};
  int mode_change_axis_{7};

  geometry_msgs::msg::Twist cmd_vel_;
  sensor_msgs::msg::Joy latest_joy_msg_;
  bool spring_fire_enabled_{false};
  uint8_t belt_rpm_mode_{static_cast<uint8_t>(BeltRpmMode::STOP)};
  bool dribble_enabled_{false};
  bool belt_ready_{false};
  bool intake_shoot_pending_{false};
  bool forward_reverse_{false};
  bool position_max_open_{false};
  bool joy_received_{false};
  bool joy_timeout_active_{false};
  std::chrono::steady_clock::time_point last_joy_received_time_{};

  bool emergency_stop_latched_{false};
  bool spring_fire_chord_on_{false};
  bool belt_mode_up_chord_on_{false};
  bool belt_mode_down_chord_on_{false};
  bool dribble_enable_button_on_{false};
  bool dribble_disable_chord_on_{false};
  bool emergency_stop_chord_on_{false};
  bool max_open_chord_on_{false};
  bool intake_shoot_chord_on_{false};
  bool forward_reverse_button_on_{false};
  bool pre_spring_fire_chord_on_{false};
  bool pre_belt_mode_up_chord_on_{false};
  bool pre_belt_mode_down_chord_on_{false};
  bool pre_dribble_enable_button_on_{false};
  bool pre_dribble_disable_chord_on_{false};
  bool pre_emergency_stop_chord_on_{false};
  bool pre_max_open_chord_on_{false};
  bool pre_intake_shoot_chord_on_{false};
  bool pre_forward_reverse_button_on_{false};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    belt_ready_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    mecanum_cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr belt_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    dribble_enabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
    dribble_position_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    intake_shoot_request_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

#endif  // JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
