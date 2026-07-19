#ifndef JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
#define JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_controller/msg/robot_command.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace joy_controller
{

class JoyControllerNode : public rclcpp::Node
{
public:
  JoyControllerNode();

private:
  enum class BeltMode : uint8_t
  {
    STOP = 0,
    LEVEL_1 = 1,
    LEVEL_2 = 2,
    LEVEL_3 = 3,
    LEVEL_4 = 4,
  };

  enum class DribbleMode : uint8_t
  {
    STOP = 0,
    LOW = 1,
    HIGH = 2,
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

  void call_emergency_stop();

  std::string joy_topic_;
  std::string command_topic_;
  std::string emergency_stop_service_;
  int qos_depth_;

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

  int left_stick_x_axis_;
  int left_stick_y_axis_;
  int right_stick_x_axis_;

  int mode_change_axis_;

  robot_controller::msg::RobotCommand command_;

  bool pre_intake_button_on_;
  bool pre_spring_fire_button_on_;
  bool pre_belt_fire_button_on_;
  bool pre_mode_up_;
  bool pre_mode_down_;
  bool pre_emergency_stop_button_on_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<robot_controller::msg::RobotCommand>::SharedPtr command_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;
};

}  // namespace joy_controller

#endif  // JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
