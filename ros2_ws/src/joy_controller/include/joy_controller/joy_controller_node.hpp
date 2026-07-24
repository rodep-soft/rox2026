#ifndef JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
#define JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_controller/action/dribble_position.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

  enum class DribbleRpmMode : uint8_t
  {
    STOP = 1,
    HIGH,
    LOW,
  };

  void declare_parameters();
  void get_parameters();

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  static bool button_pressed(const sensor_msgs::msg::Joy & msg, int index);
  static double axis_value(const sensor_msgs::msg::Joy & msg, int index);
  double apply_axis_deadzone(double value) const;
  static double apply_axis_limits(double value, double minimum, double maximum);

  static uint8_t increment_mode(uint8_t mode, uint8_t maximum_mode);
  static uint8_t decrement_mode(uint8_t mode);

  void send_dribble_position_goal(uint8_t command);
  void publish_emergency_stop();
  void publish_state_commands();
  void publish_stop_commands();
  void joy_timeout_callback();
  void state_publish_timer_callback();
  void update_chord_inputs(const sensor_msgs::msg::Joy & msg);
  bool handle_emergency_stop();
  void update_previous_chord_inputs();

  std::string joy_topic_;
  std::string mecanum_cmd_vel_topic_, spring_fire_request_topic_, belt_fire_topic_,
    belt_mode_topic_, dribble_mode_topic_;
  std::string emergency_stop_topic_;
  std::string dribble_position_action_;
  int joy_qos_depth_;
  int command_qos_depth_;
  int joy_timeout_ms_;
  int state_publish_period_ms_;

  double linear_x_scale_;
  double linear_y_scale_;
  double angular_z_scale_;

  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;
  double min_linear_x_;
  double min_linear_y_;
  double min_angular_z_;

  double axis_deadzone_;
  double axis_on_threshold_;

  int intake_is_enable_button_;
  int intake_button_on_;

  int spring_fire_is_enable_button_;
  int spring_fire_button_on_;

  int belt_fire_is_enable_button_;
  int belt_fire_button_on_;

  int belt_mode_is_enable_button_;
  int dribble_mode_is_enable_button_;

  int emergency_stop_is_enable_button_;
  int emergency_stop_button_on_;
  int dribble_position_is_enable_button_;
  int dribble_position_dribble_button_;
  int dribble_position_fire_button_;

  int left_stick_x_axis_;
  int left_stick_y_axis_;
  int right_stick_x_axis_;

  int mode_change_axis_;

  geometry_msgs::msg::Twist cmd_vel_;
  bool intake_enabled_{false};
  bool spring_fire_enabled_{false};
  bool belt_fire_enabled_{false};
  uint8_t belt_rpm_mode_{static_cast<uint8_t>(BeltRpmMode::STOP)};
  uint8_t dribble_rpm_mode_{static_cast<uint8_t>(DribbleRpmMode::STOP)};
  bool joy_received_{false};
  bool joy_timeout_active_{false};
  std::chrono::steady_clock::time_point last_joy_received_time_{};

  bool emergency_stop_latched_{false};
  bool intake_chord_on_{false};
  bool spring_fire_chord_on_{false};
  bool belt_fire_chord_on_{false};
  bool belt_mode_up_chord_on_{false};
  bool belt_mode_down_chord_on_{false};
  bool dribble_mode_up_chord_on_{false};
  bool dribble_mode_down_chord_on_{false};
  bool emergency_stop_chord_on_{false};
  bool dribble_position_dribble_chord_on_{false};
  bool dribble_position_fire_chord_on_{false};
  bool pre_intake_chord_on_;
  bool pre_spring_fire_chord_on_;
  bool pre_belt_fire_chord_on_;
  bool pre_belt_mode_up_chord_on_;
  bool pre_belt_mode_down_chord_on_;
  bool pre_dribble_mode_up_chord_on_;
  bool pre_dribble_mode_down_chord_on_;
  bool pre_emergency_stop_chord_on_;
  bool pre_dribble_position_dribble_chord_on_;
  bool pre_dribble_position_fire_chord_on_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mecanum_cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_publisher_, belt_fire_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr belt_mode_publisher_, dribble_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  rclcpp_action::Client<robot_controller::action::DribblePosition>::SharedPtr
    dribble_position_action_client_;
  rclcpp::TimerBase::SharedPtr joy_timeout_timer_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

#endif  // JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
