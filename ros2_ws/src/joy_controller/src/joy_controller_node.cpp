#include "joy_controller/joy_controller_node.hpp"

#include <cmath>
#include <functional>
#include <memory>
namespace joy_controller
{

JoyControllerNode::JoyControllerNode()
: Node("joy_controller"),
  pre_intake_button_on_(false),
  pre_spring_fire_button_on_(false),
  pre_belt_fire_button_on_(false),
  pre_dpad_is_up_(false),
  pre_dpad_is_down_(false),
  pre_emergency_stop_button_on_(false)
{
  declare_parameters();
  get_parameters();

  if (qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "qos_depth must be positive. Using the default value of 10.");
    qos_depth_ = 10;
  }

  command_.belt_mode = static_cast<uint8_t>(BeltMode::STOP);
  command_.dribble_mode = static_cast<uint8_t>(DribbleMode::STOP);

  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));
  command_publisher_ = create_publisher<robot_controller::msg::RobotCommand>(
    command_topic_, rclcpp::QoS(qos_depth_));
  emergency_stop_client_ = create_client<std_srvs::srv::Trigger>(emergency_stop_service_);

  RCLCPP_INFO(
    get_logger(), "Subscribing to %s and publishing commands on %s",
    joy_topic_.c_str(), command_topic_.c_str());
}

void JoyControllerNode::declare_parameters()
{
  declare_parameter<std::string>("joy_topic", "/joy");
  declare_parameter<std::string>("command_topic", "/robot_command");
  declare_parameter<std::string>("emergency_stop_service", "/emergency_stop");
  declare_parameter<int>("qos_depth", 10);
  declare_parameter<double>("linear_x_scale", 1.0);
  declare_parameter<double>("linear_y_scale", 1.0);
  declare_parameter<double>("angular_z_scale", 1.0);
  declare_parameter<double>("max_linear_x", 2.0);
  declare_parameter<double>("min_linear_x", -2.0);
  declare_parameter<double>("max_linear_y", 2.0);
  declare_parameter<double>("min_linear_y", -2.0);
  declare_parameter<double>("max_angular_z", 2.0);
  declare_parameter<double>("min_angular_z", -2.0);
  declare_parameter<double>("axis_deadzone", 0.05);
  declare_parameter<double>("axis_on_threshold", 0.7);
  declare_parameter<int>("intake_is_enable_button", 6);
  declare_parameter<int>("intake_button_on", 3);
  declare_parameter<int>("spring_fire_is_enable_button", 7);
  declare_parameter<int>("spring_fire_button_on", 2);
  declare_parameter<int>("belt_fire_is_enable_button", 6);
  declare_parameter<int>("belt_fire_button_on", 2);
  declare_parameter<int>("belt_mode_is_enable_button", 6);
  declare_parameter<int>("dribble_mode_is_enable_button", 7);
  declare_parameter<int>("emergency_stop_is_enable_button", 8);
  declare_parameter<int>("emergency_stop_button_on", 13);
  declare_parameter<int>("left_stick_x_axis", 0);
  declare_parameter<int>("left_stick_y_axis", 1);
  declare_parameter<int>("right_stick_x_axis", 2);
  declare_parameter<int>("dpad_y_axis", 7);
}

void JoyControllerNode::get_parameters()
{
  get_parameter("joy_topic", joy_topic_);
  get_parameter("command_topic", command_topic_);
  get_parameter("emergency_stop_service", emergency_stop_service_);
  get_parameter("qos_depth", qos_depth_);
  get_parameter("linear_x_scale", linear_x_scale_);
  get_parameter("linear_y_scale", linear_y_scale_);
  get_parameter("angular_z_scale", angular_z_scale_);
  get_parameter("max_linear_x", max_linear_x_);
  get_parameter("min_linear_x", min_linear_x_);
  get_parameter("max_linear_y", max_linear_y_);
  get_parameter("min_linear_y", min_linear_y_);
  get_parameter("max_angular_z", max_angular_z_);
  get_parameter("min_angular_z", min_angular_z_);
  get_parameter("axis_deadzone", axis_deadzone_);
  get_parameter("axis_on_threshold", axis_on_threshold_);
  get_parameter("intake_is_enable_button", intake_is_enable_button_);
  get_parameter("intake_button_on", intake_button_on_);
  get_parameter("spring_fire_is_enable_button", spring_fire_is_enable_button_);
  get_parameter("spring_fire_button_on", spring_fire_button_on_);
  get_parameter("belt_fire_is_enable_button", belt_fire_is_enable_button_);
  get_parameter("belt_fire_button_on", belt_fire_button_on_);
  get_parameter("belt_mode_is_enable_button", belt_mode_is_enable_button_);
  get_parameter("dribble_mode_is_enable_button", dribble_mode_is_enable_button_);
  get_parameter("emergency_stop_is_enable_button", emergency_stop_is_enable_button_);
  get_parameter("emergency_stop_button_on", emergency_stop_button_on_);
  get_parameter("left_stick_x_axis", left_stick_x_axis_);
  get_parameter("left_stick_y_axis", left_stick_y_axis_);
  get_parameter("right_stick_x_axis", right_stick_x_axis_);
  get_parameter("dpad_y_axis", dpad_y_axis_);
}

bool JoyControllerNode::button_pressed(const sensor_msgs::msg::Joy & msg, int index)
{
  return index >= 0 && static_cast<std::size_t>(index) < msg.buttons.size() &&
         msg.buttons[static_cast<std::size_t>(index)] != 0;
}

double JoyControllerNode::axis_value(const sensor_msgs::msg::Joy & msg, int index)
{
  return index >= 0 && static_cast<std::size_t>(index) < msg.axes.size() ?
         msg.axes[static_cast<std::size_t>(index)] : 0.0;
}

double JoyControllerNode::apply_axis_deadzone(double value) const
{
  return std::abs(value) < axis_deadzone_ ? 0.0 : value;
}

double JoyControllerNode::apply_axis_limits(double value, double minimum, double maximum)
{
  return value >= 0.0 ? value * maximum : -value * minimum;
}

uint8_t JoyControllerNode::increment_mode(uint8_t mode, uint8_t maximum_mode)
{
  return mode < maximum_mode ? static_cast<uint8_t>(mode + 1) : maximum_mode;
}

uint8_t JoyControllerNode::decrement_mode(uint8_t mode)
{
  return mode > 1 ? static_cast<uint8_t>(mode - 1) : 1;
}

void JoyControllerNode::call_emergency_stop()
{
  if (!emergency_stop_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "Emergency-stop service %s is not available",
      emergency_stop_service_.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  emergency_stop_client_->async_send_request(request);
  RCLCPP_WARN(get_logger(), "Emergency-stop service request sent");
}

void JoyControllerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  const auto & joy_msg = *msg;
  const bool intake_is_enable_button = button_pressed(joy_msg, intake_is_enable_button_);
  const bool intake_button_on = button_pressed(joy_msg, intake_button_on_);
  const bool spring_fire_is_enable_button = button_pressed(joy_msg, spring_fire_is_enable_button_);
  const bool spring_fire_button_on = button_pressed(joy_msg, spring_fire_button_on_);
  const bool belt_fire_is_enable_button = button_pressed(joy_msg, belt_fire_is_enable_button_);
  const bool belt_fire_button_on = button_pressed(joy_msg, belt_fire_button_on_);
  const bool belt_mode_is_enable_button = button_pressed(joy_msg, belt_mode_is_enable_button_);
  const bool dribble_mode_is_enable_button = button_pressed(joy_msg, dribble_mode_is_enable_button_);
  const bool emergency_stop_is_enable_button =
    button_pressed(joy_msg, emergency_stop_is_enable_button_);
  const bool emergency_stop_button_on = button_pressed(joy_msg, emergency_stop_button_on_);
  const double dpad_y = axis_value(joy_msg, dpad_y_axis_);
  const bool is_belt_mode_up = dpad_y >= axis_on_threshold_;
  const bool is_belt_mode_down = dpad_y <= -axis_on_threshold_;
  const bool is_dribble_mode_up = dpad_y >= axis_on_threshold_;
  const bool is_dribble_mode_down = dpad_y <= -axis_on_threshold_;

  if (intake_is_enable_button && intake_button_on && !pre_intake_button_on_) {
    command_.is_intake = !command_.is_intake;
  }
  if (spring_fire_is_enable_button && spring_fire_button_on && !pre_spring_fire_button_on_)
  {
    command_.spring_is_fire = !command_.spring_is_fire;
  }
  if (belt_fire_is_enable_button && belt_fire_button_on && !pre_belt_fire_button_on_) {
    command_.belt_is_fire = !command_.belt_is_fire;
  }
  if (belt_mode_is_enable_button && is_belt_mode_up && !pre_dpad_is_up_) {
    command_.belt_mode = increment_mode(
      command_.belt_mode, static_cast<uint8_t>(BeltMode::HIGH));
  }
  if (belt_mode_is_enable_button && is_belt_mode_down && !pre_dpad_is_down_) {
    command_.belt_mode = decrement_mode(command_.belt_mode);
  }
  if (dribble_mode_is_enable_button && is_dribble_mode_up && !pre_dpad_is_up_) {
    command_.dribble_mode = increment_mode(
      command_.dribble_mode, static_cast<uint8_t>(DribbleMode::MAX));
  }
  if (dribble_mode_is_enable_button && is_dribble_mode_down && !pre_dpad_is_down_) {
    command_.dribble_mode = decrement_mode(command_.dribble_mode);
  }

  const double linear_x = apply_axis_deadzone(axis_value(joy_msg, left_stick_y_axis_));
  const double linear_y = apply_axis_deadzone(axis_value(joy_msg, left_stick_x_axis_));
  const double angular_z = apply_axis_deadzone(axis_value(joy_msg, right_stick_x_axis_));
  command_.cmd_vel.linear.x =
    apply_axis_limits(linear_x, min_linear_x_, max_linear_x_) * linear_x_scale_;
  command_.cmd_vel.linear.y =
    apply_axis_limits(linear_y, min_linear_y_, max_linear_y_) * linear_y_scale_;
  command_.cmd_vel.angular.z =
    apply_axis_limits(angular_z, min_angular_z_, max_angular_z_) * angular_z_scale_;
  command_publisher_->publish(command_);

  if (emergency_stop_is_enable_button && emergency_stop_button_on &&
    !pre_emergency_stop_button_on_)
  {
    call_emergency_stop();
  }

  pre_intake_button_on_ = intake_button_on;
  pre_spring_fire_button_on_ = spring_fire_button_on;
  pre_belt_fire_button_on_ = belt_fire_button_on;
  pre_dpad_is_up_ = is_belt_mode_up;
  pre_dpad_is_down_ = is_belt_mode_down;
  pre_emergency_stop_button_on_ = emergency_stop_button_on;
}

}  // namespace joy_controller
