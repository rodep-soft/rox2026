#include "joy_controller/joy_controller_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

JoyControllerNode::JoyControllerNode()
: Node("joy_controller")
{
  declare_parameters();
  get_parameters();

  if (joy_qos_depth_ <= 0) {
    joy_qos_depth_ = 1;
  }
  if (command_qos_depth_ <= 0) {
    command_qos_depth_ = 1;
  }
  if (joy_timeout_ms_ <= 0) {
    joy_timeout_ms_ = 200;
  }
  if (state_publish_period_ms_ <= 0) {
    state_publish_period_ms_ = 20;
  }

  auto joy_qos = rclcpp::SensorDataQoS();
  joy_qos.keep_last(joy_qos_depth_);

  // Joy入力を受け取る。
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, joy_qos,
    std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));
  // shot cycleが動作中かを受け取る。
  shot_cycle_running_subscription_ = create_subscription<std_msgs::msg::Bool>(
    shot_cycle_running_topic_, rclcpp::QoS(command_qos_depth_),
    std::bind(
      &JoyControllerNode::shot_cycle_running_callback, this,
      std::placeholders::_1));
  // shot cycleの完了通知を受け取る。
  shot_cycle_complete_subscription_ = create_subscription<std_msgs::msg::Bool>(
    shot_cycle_complete_topic_, rclcpp::QoS(command_qos_depth_),
    std::bind(
      &JoyControllerNode::shot_cycle_complete_callback, this,
      std::placeholders::_1));

  // 各controllerへ現在の操作モードを送る。
  operation_mode_publisher_ = create_publisher<std_msgs::msg::UInt8>(
    operation_mode_topic_, rclcpp::QoS(1).reliable().transient_local());
  // 各controllerへ停止状態を送る。
  emergency_stop_publisher_ = create_publisher<std_msgs::msg::Bool>(
    emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local());

  // mecanum_controllerへ走行速度指令を送る。
  mecanum_cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    mecanum_cmd_vel_topic_, rclcpp::QoS(command_qos_depth_));

  // spring_position_controllerへ発射要求を送る。
  spring_fire_publisher_ = create_publisher<std_msgs::msg::Bool>(
    spring_fire_request_topic_, rclcpp::QoS(command_qos_depth_));
  // belt_dribble_controllerへベルト回転モードを送る。
  belt_mode_publisher_ = create_publisher<std_msgs::msg::UInt8>(
    belt_mode_topic_, rclcpp::QoS(command_qos_depth_));
  // belt_dribble_controllerへドリブル有効状態を送る。
  dribble_enabled_publisher_ = create_publisher<std_msgs::msg::Bool>(
    dribble_enabled_topic_, rclcpp::QoS(command_qos_depth_));
  // belt_dribble_controllerへshot cycle開始要求を送る。
  shot_cycle_request_publisher_ = create_publisher<std_msgs::msg::Bool>(
    shot_cycle_request_topic_, rclcpp::QoS(command_qos_depth_));
  // spring_position_controllerへ手動位置指令を送る。
  dribble_position_mode_publisher_ = create_publisher<std_msgs::msg::UInt8>(
    dribble_position_mode_topic_, rclcpp::QoS(command_qos_depth_));

  publish_stop_commands();
  joy_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&JoyControllerNode::joy_timeout_callback, this));
  state_publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(state_publish_period_ms_),
    std::bind(&JoyControllerNode::state_publish_timer_callback, this));
}

void JoyControllerNode::declare_parameters()
{
  // Topic名
  declare_parameter<std::string>("joy_topic", "/joy");
  declare_parameter<std::string>("mecanum_cmd_vel_topic", "/mecanum/cmd_vel");
  declare_parameter<std::string>("spring_fire_request_topic",
    "/spring/fire_request");
  declare_parameter<std::string>("belt_mode_topic", "/belt/mode");
  declare_parameter<std::string>("dribble_enabled_topic", "/dribble/enabled");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("shot_cycle_complete_topic",
    "/shot_cycle/complete");
  declare_parameter<std::string>("shot_cycle_request_topic",
    "/shot_cycle/request");
  declare_parameter<std::string>("shot_cycle_running_topic",
    "/shot_cycle/running");
  declare_parameter<std::string>("dribble_position_mode_topic",
    "/dribble/position_mode");

  // QoSと周期
  declare_parameter<int>("joy_qos_depth", 1);
  declare_parameter<int>("command_qos_depth", 1);
  declare_parameter<int>("joy_timeout_ms", 200);
  declare_parameter<int>("state_publish_period_ms", 20);
  declare_parameter<bool>("auto_drive_on_shot_cycle_complete", true);

  // 走行指令
  declare_parameter<double>("linear_x_scale", 1.0);
  declare_parameter<double>("linear_y_scale", 1.0);
  declare_parameter<double>("angular_z_scale", 1.0);
  declare_parameter<double>("linear_x_limit", 2.0);
  declare_parameter<double>("linear_y_limit", 2.0);
  declare_parameter<double>("angular_z_limit", 2.0);

  // 入力判定
  declare_parameter<double>("axis_deadzone", 0.05);
  declare_parameter<double>("axis_on_threshold", 0.7);

  // ボタン番号
  declare_parameter<int>("spring_fire_enable_button", 4);
  declare_parameter<int>("spring_fire_button", 2);
  declare_parameter<int>("create_button", 8);
  declare_parameter<int>("ps_button", 12);
  declare_parameter<int>("options_button", 9);
  declare_parameter<int>("home_button", 13);
  declare_parameter<int>("circle_button", 2);
  declare_parameter<int>("dribble_enable_button", 5);

  // 軸番号
  declare_parameter<int>("left_trigger_axis", 3);
  declare_parameter<int>("right_trigger_axis", 4);
  declare_parameter<int>("left_stick_x_axis", 0);
  declare_parameter<int>("left_stick_y_axis", 1);
  declare_parameter<int>("right_stick_x_axis", 2);
  declare_parameter<int>("dpad_horizontal_axis", 6);
  declare_parameter<int>("dpad_vertical_axis", 7);
}

void JoyControllerNode::get_parameters()
{
  // Topic名
  get_parameter("joy_topic", joy_topic_);
  get_parameter("mecanum_cmd_vel_topic", mecanum_cmd_vel_topic_);
  get_parameter("spring_fire_request_topic", spring_fire_request_topic_);
  get_parameter("belt_mode_topic", belt_mode_topic_);
  get_parameter("dribble_enabled_topic", dribble_enabled_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("shot_cycle_complete_topic", shot_cycle_complete_topic_);
  get_parameter("shot_cycle_request_topic", shot_cycle_request_topic_);
  get_parameter("shot_cycle_running_topic", shot_cycle_running_topic_);
  get_parameter("dribble_position_mode_topic", dribble_position_mode_topic_);

  // QoSと周期
  get_parameter("joy_qos_depth", joy_qos_depth_);
  get_parameter("command_qos_depth", command_qos_depth_);
  get_parameter("joy_timeout_ms", joy_timeout_ms_);
  get_parameter("state_publish_period_ms", state_publish_period_ms_);
  get_parameter(
    "auto_drive_on_shot_cycle_complete",
    auto_drive_on_shot_cycle_complete_);

  // 走行指令
  get_parameter("linear_x_scale", linear_x_scale_);
  get_parameter("linear_y_scale", linear_y_scale_);
  get_parameter("angular_z_scale", angular_z_scale_);
  get_parameter("linear_x_limit", linear_x_limit_);
  get_parameter("linear_y_limit", linear_y_limit_);
  get_parameter("angular_z_limit", angular_z_limit_);

  // 入力判定
  get_parameter("axis_deadzone", axis_deadzone_);
  get_parameter("axis_on_threshold", axis_on_threshold_);

  // ボタン番号
  get_parameter("spring_fire_enable_button", spring_fire_enable_button_);
  get_parameter("spring_fire_button", spring_fire_button_);
  get_parameter("create_button", create_button_);
  get_parameter("ps_button", ps_button_);
  get_parameter("options_button", options_button_);
  get_parameter("home_button", home_button_);
  get_parameter("circle_button", circle_button_);
  get_parameter("dribble_enable_button", dribble_enable_button_);

  // 軸番号
  get_parameter("left_trigger_axis", left_trigger_axis_);
  get_parameter("right_trigger_axis", right_trigger_axis_);
  get_parameter("left_stick_x_axis", left_stick_x_axis_);
  get_parameter("left_stick_y_axis", left_stick_y_axis_);
  get_parameter("right_stick_x_axis", right_stick_x_axis_);
  get_parameter("dpad_horizontal_axis", dpad_horizontal_axis_);
  get_parameter("dpad_vertical_axis", dpad_vertical_axis_);
}

bool JoyControllerNode::button_pressed(
  const sensor_msgs::msg::Joy & msg,
  int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.buttons.size()) {
    return false;
  }
  return msg.buttons[static_cast<std::size_t>(index)] != 0;
}

double JoyControllerNode::axis_value(
  const sensor_msgs::msg::Joy & msg,
  int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.axes.size()) {
    return 0.0;
  }
  return msg.axes[static_cast<std::size_t>(index)];
}

double JoyControllerNode::apply_axis_deadzone(double value) const
{
  if (std::abs(value) < axis_deadzone_) {
    return 0.0;
  }
  return value;
}

double JoyControllerNode::apply_axis_limit(double value, double limit)
{
  return value * std::abs(limit);
}

uint8_t JoyControllerNode::increment_mode(uint8_t mode, uint8_t maximum_mode)
{
  if (mode < maximum_mode) {
    return static_cast<uint8_t>(mode + 1);
  }
  return maximum_mode;
}

uint8_t JoyControllerNode::decrement_mode(uint8_t mode)
{
  if (mode > static_cast<uint8_t>(BeltRpmMode::STOP)) {
    return static_cast<uint8_t>(mode - 1);
  }
  return static_cast<uint8_t>(BeltRpmMode::STOP);
}

void JoyControllerNode::publish_position(Position position)
{
  std_msgs::msg::UInt8 message;
  message.data = static_cast<uint8_t>(position);
  dribble_position_mode_publisher_->publish(message);
}

void JoyControllerNode::publish_shot_cycle_request()
{
  std_msgs::msg::Bool message;
  message.data = true;
  shot_cycle_request_publisher_->publish(message);
}

void JoyControllerNode::publish_spring_fire_request(bool requested)
{
  std_msgs::msg::Bool message;
  message.data = requested;
  spring_fire_publisher_->publish(message);
}

void JoyControllerNode::publish_emergency_stop()
{
  std_msgs::msg::Bool message;
  message.data = operation_mode_ == OperationMode::STOP;
  emergency_stop_publisher_->publish(message);
}

void JoyControllerNode::publish_operation_mode()
{
  std_msgs::msg::UInt8 message;
  message.data = static_cast<uint8_t>(operation_mode_);
  operation_mode_publisher_->publish(message);
}

void JoyControllerNode::publish_belt_mode()
{
  std_msgs::msg::UInt8 belt;
  belt.data = belt_rpm_mode_;
  belt_mode_publisher_->publish(belt);
}

void JoyControllerNode::publish_dribble_enabled()
{
  std_msgs::msg::Bool dribble;
  dribble.data = dribble_enabled_;
  dribble_enabled_publisher_->publish(dribble);
}

void JoyControllerNode::publish_stop_commands()
{
  cmd_vel_ = geometry_msgs::msg::Twist{};
  operation_mode_ = OperationMode::STOP;
  shot_cycle_running_ = false;
  belt_rpm_mode_ = static_cast<uint8_t>(BeltRpmMode::STOP);
  dribble_enabled_ = false;
  mecanum_cmd_vel_publisher_->publish(cmd_vel_);
  publish_spring_fire_request(false);
  publish_belt_mode();
  publish_dribble_enabled();
  publish_operation_mode();
  publish_emergency_stop();
}

void JoyControllerNode::set_operation_mode(OperationMode mode)
{
  operation_mode_ = mode;
  publish_operation_mode();
  publish_emergency_stop();
}

void JoyControllerNode::joy_timeout_callback()
{
  if (!joy_received_) {
    return;
  }
  if (joy_timeout_active_) {
    return;
  }
  if (std::chrono::steady_clock::now() - last_joy_received_time_ >
    std::chrono::milliseconds(joy_timeout_ms_))
  {
    joy_timeout_active_ = true;
    publish_stop_commands();
  }
}

void JoyControllerNode::state_publish_timer_callback()
{
  if (!joy_timeout_active_) {
    publish_belt_mode();
    publish_dribble_enabled();
    publish_operation_mode();
  }
}

void JoyControllerNode::shot_cycle_running_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  if (operation_mode_ == OperationMode::SHOT_CYCLE) {
    shot_cycle_running_ = msg->data;
  }
}

void JoyControllerNode::shot_cycle_complete_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && operation_mode_ == OperationMode::SHOT_CYCLE) {
    shot_cycle_running_ = false;
    if (auto_drive_on_shot_cycle_complete_) {
      set_operation_mode(OperationMode::DRIVE);
    }
  }
}

void JoyControllerNode::update_chord_inputs(const sensor_msgs::msg::Joy & msg)
{
  const bool l2 = axis_value(msg, left_trigger_axis_) <= -axis_on_threshold_;
  const bool r2 = axis_value(msg, right_trigger_axis_) <= -axis_on_threshold_;
  const double dpad_horizontal = axis_value(msg, dpad_horizontal_axis_);
  const double dpad_vertical = axis_value(msg, dpad_vertical_axis_);

  spring_fire_chord_on_ = button_pressed(msg, spring_fire_enable_button_) &&
    button_pressed(msg, spring_fire_button_);
  belt_mode_up_chord_on_ = dpad_vertical >= axis_on_threshold_;
  belt_mode_down_chord_on_ = dpad_vertical <= -axis_on_threshold_;
  dribble_enable_button_on_ = button_pressed(msg, dribble_enable_button_);
  home_button_on_ = button_pressed(msg, home_button_);
  create_button_on_ = button_pressed(msg, create_button_);
  options_button_on_ = button_pressed(msg, options_button_);
  shot_cycle_chord_on_ = l2 && button_pressed(msg, circle_button_);
  manual_shoot_chord_on_ = r2 && dpad_horizontal >= axis_on_threshold_;
  manual_open_chord_on_ = r2 && dpad_horizontal <= -axis_on_threshold_;
  forward_reverse_button_on_ = button_pressed(msg, ps_button_);
}

void JoyControllerNode::handle_operation_mode()
{
  if (home_button_on_ && !pre_home_button_on_) {
    if (operation_mode_ == OperationMode::STOP) {
      set_operation_mode(OperationMode::DRIVE);
    } else {
      publish_stop_commands();
      publish_position(Position::DRIBBLE);
    }
    return;
  }

  if (shot_cycle_running_) {
    return;
  }

  if (create_button_on_ && !pre_create_button_on_) {
    if (operation_mode_ == OperationMode::STOP ||
      operation_mode_ == OperationMode::DRIVE)
    {
      set_operation_mode(OperationMode::SHOT_CYCLE);
    } else if (operation_mode_ == OperationMode::SHOT_CYCLE) {
      set_operation_mode(OperationMode::DRIVE);
    }
    return;
  }

  if (options_button_on_ && !pre_options_button_on_) {
    if (operation_mode_ == OperationMode::STOP ||
      operation_mode_ == OperationMode::DRIVE)
    {
      set_operation_mode(OperationMode::BELT_ONLY);
    } else if (operation_mode_ == OperationMode::BELT_ONLY) {
      set_operation_mode(OperationMode::DRIVE);
    }
  }
}

bool JoyControllerNode::is_manual_position_allowed() const
{
  return !shot_cycle_running_ && (operation_mode_ == OperationMode::DRIVE ||
         operation_mode_ == OperationMode::SHOT_CYCLE);
}

void JoyControllerNode::update_previous_chord_inputs()
{
  pre_belt_mode_up_chord_on_ = belt_mode_up_chord_on_;
  pre_belt_mode_down_chord_on_ = belt_mode_down_chord_on_;
  pre_dribble_enable_button_on_ = dribble_enable_button_on_;
  pre_home_button_on_ = home_button_on_;
  pre_create_button_on_ = create_button_on_;
  pre_options_button_on_ = options_button_on_;
  pre_shot_cycle_chord_on_ = shot_cycle_chord_on_;
  pre_manual_shoot_chord_on_ = manual_shoot_chord_on_;
  pre_manual_open_chord_on_ = manual_open_chord_on_;
  pre_forward_reverse_button_on_ = forward_reverse_button_on_;
}

void JoyControllerNode::joy_callback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_timeout_active_ = false;
  joy_received_ = true;
  last_joy_received_time_ = std::chrono::steady_clock::now();
  update_chord_inputs(*msg);
  handle_operation_mode();

  const bool spring_fire_requested =
    spring_fire_chord_on_ && operation_mode_ == OperationMode::DRIVE;
  publish_spring_fire_request(spring_fire_requested);

  if (belt_mode_up_chord_on_ && !pre_belt_mode_up_chord_on_ &&
    operation_mode_ != OperationMode::STOP)
  {
    belt_rpm_mode_ = increment_mode(
      belt_rpm_mode_,
      static_cast<uint8_t>(BeltRpmMode::LEVEL_6));
  }
  if (belt_mode_down_chord_on_ && !pre_belt_mode_down_chord_on_ &&
    operation_mode_ != OperationMode::STOP)
  {
    belt_rpm_mode_ = decrement_mode(belt_rpm_mode_);
  }

  const bool dribble_mode = operation_mode_ == OperationMode::DRIVE ||
    operation_mode_ == OperationMode::SHOT_CYCLE;
  if (dribble_enable_button_on_ && !pre_dribble_enable_button_on_ &&
    dribble_mode)
  {
    dribble_enabled_ = !dribble_enabled_;
  }
  if (forward_reverse_button_on_ && !pre_forward_reverse_button_on_) {
    forward_reverse_ = !forward_reverse_;
  }
  if (shot_cycle_chord_on_ && !pre_shot_cycle_chord_on_ &&
    operation_mode_ == OperationMode::SHOT_CYCLE && !shot_cycle_running_)
  {
    publish_shot_cycle_request();
  }
  if (manual_shoot_chord_on_ && !pre_manual_shoot_chord_on_ &&
    is_manual_position_allowed())
  {
    publish_position(Position::SHOOT);
  }
  if (manual_open_chord_on_ && !pre_manual_open_chord_on_ &&
    is_manual_position_allowed())
  {
    publish_position(Position::OPEN);
  }

  const double linear_x =
    apply_axis_deadzone(axis_value(*msg, left_stick_y_axis_));
  const double linear_y =
    apply_axis_deadzone(axis_value(*msg, left_stick_x_axis_));
  const double angular_z =
    apply_axis_deadzone(axis_value(*msg, right_stick_x_axis_));

  cmd_vel_.linear.x =
    apply_axis_limit(linear_x, linear_x_limit_) * linear_x_scale_;
  cmd_vel_.linear.y =
    apply_axis_limit(linear_y, linear_y_limit_) * linear_y_scale_;
  if (forward_reverse_) {
    cmd_vel_.linear.x *= -1.0;
    cmd_vel_.linear.y *= -1.0;
  }
  cmd_vel_.angular.z =
    apply_axis_limit(angular_z, angular_z_limit_) * angular_z_scale_;
  mecanum_cmd_vel_publisher_->publish(cmd_vel_);
  update_previous_chord_inputs();
}
