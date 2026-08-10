#ifndef JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
#define JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cstdint>

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
    STOP = 0,
    LEVEL_1 = 1,
    LEVEL_2 = 2,
    LEVEL_3 = 3,
    LEVEL_4 = 4,
  };

  enum class ArmPositionMode : uint8_t
  {
    DRIBBLE = 0,
    OPEN = 1,
    FEED = 2,
  };

  void declare_parameters();
  void get_parameters();

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void loop_callback();
  void joy_timeout_timer_callback();
  void state_publish_timer_callback();

  void publish_emergency_stop();
  void publish_belt_mode();
  void publish_dribble_enabled();
  void publish_stop_commands();

  // 直感的なエッジ検出・入力判定ヘルパー
  bool is_button_down(const sensor_msgs::msg::Joy & msg, int index) const;
  bool is_button_just_pressed(const sensor_msgs::msg::Joy & msg, int index) const;
  double get_axis_value(const sensor_msgs::msg::Joy & msg, int index) const;
  bool is_axis_just_triggered(const sensor_msgs::msg::Joy & msg, int index, bool positive) const;

  double apply_axis_deadzone(double value) const;
  static double apply_axis_limit(double value, double limit);
  static uint8_t increment_mode(uint8_t mode, uint8_t maximum_mode);
  static uint8_t decrement_mode(uint8_t mode);

  int joy_qos_depth_{1};
  int command_qos_depth_{1};
  int joy_timeout_ms_{200};
  int state_publish_period_ms_{20};

  double linear_x_limit_{2.0};
  double linear_y_limit_{2.0};
  double angular_z_limit_{2.0};
  double axis_deadzone_{0.05};
  double axis_on_threshold_{0.7};

  int spring_fire_enable_button_{4};
  int spring_fire_button_{2};
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
  bool is_emergency_stop_{true};
  uint8_t belt_rpm_mode_{static_cast<uint8_t>(BeltRpmMode::STOP)};
  bool dribble_enabled_{false};
  bool game2_active_{false};
  bool is_drive_reversed_{false};
  bool joy_received_{false};
  bool joy_timeout_active_{false};
  std::chrono::steady_clock::time_point last_joy_received_time_{};

  std::optional<sensor_msgs::msg::Joy> last_joy_msg_{};
  sensor_msgs::msg::Joy joy_msg_{};


  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mecanum_cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shot_cycle_request_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_position_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr game2_start_pub_;

  rclcpp::TimerBase::SharedPtr joy_timeout_timer_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

#endif  // JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
