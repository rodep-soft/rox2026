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

  if (joy_qos_depth_ <= 0) {joy_qos_depth_ = 1;}
  if (command_qos_depth_ <= 0) {command_qos_depth_ = 1;}
  if (joy_timeout_ms_ <= 0) {joy_timeout_ms_ = 200;}
  if (state_publish_period_ms_ <= 0) {state_publish_period_ms_ = 20;}

  auto joy_qos = rclcpp::SensorDataQoS();
  joy_qos.keep_last(joy_qos_depth_);

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", joy_qos,
    std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  mecanum_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/mecanum/cmd_vel", rclcpp::QoS(command_qos_depth_));

  spring_fire_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/spring/fire_request", rclcpp::QoS(command_qos_depth_));

  belt_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/belt/mode", rclcpp::QoS(command_qos_depth_));

  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/enabled", rclcpp::QoS(command_qos_depth_));

  shot_cycle_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/shot_cycle/request", rclcpp::QoS(command_qos_depth_));

  arm_position_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/dribble/position_mode", rclcpp::QoS(command_qos_depth_));

  publish_stop_commands();

  joy_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&JoyControllerNode::joy_timeout_timer_callback, this));

  state_publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(state_publish_period_ms_),
    std::bind(&JoyControllerNode::state_publish_timer_callback, this));
}

void JoyControllerNode::declare_parameters()
{
  declare_parameter<int>("joy_qos_depth", 1);
  declare_parameter<int>("command_qos_depth", 1);
  declare_parameter<int>("joy_timeout_ms", 200);
  declare_parameter<int>("state_publish_period_ms", 20);

  declare_parameter<double>("linear_x_scale", 1.0);
  declare_parameter<double>("linear_y_scale", 1.0);
  declare_parameter<double>("angular_z_scale", 1.0);
  declare_parameter<double>("linear_x_limit", 2.0);
  declare_parameter<double>("linear_y_limit", 2.0);
  declare_parameter<double>("angular_z_limit", 2.0);

  declare_parameter<double>("axis_deadzone", 0.05);
  declare_parameter<double>("axis_on_threshold", 0.7);

  declare_parameter<int>("spring_fire_enable_button", 4);
  declare_parameter<int>("spring_fire_button", 2);
  declare_parameter<int>("ps_button", 12);
  declare_parameter<int>("home_button", 13);
  declare_parameter<int>("circle_button", 2);
  declare_parameter<int>("dribble_enable_button", 5);

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
  get_parameter("joy_qos_depth", joy_qos_depth_);
  get_parameter("command_qos_depth", command_qos_depth_);
  get_parameter("joy_timeout_ms", joy_timeout_ms_);
  get_parameter("state_publish_period_ms", state_publish_period_ms_);

  get_parameter("linear_x_scale", linear_x_scale_);
  get_parameter("linear_y_scale", linear_y_scale_);
  get_parameter("angular_z_scale", angular_z_scale_);
  get_parameter("linear_x_limit", linear_x_limit_);
  get_parameter("linear_y_limit", linear_y_limit_);
  get_parameter("angular_z_limit", angular_z_limit_);

  get_parameter("axis_deadzone", axis_deadzone_);
  get_parameter("axis_on_threshold", axis_on_threshold_);

  get_parameter("spring_fire_enable_button", spring_fire_enable_button_);
  get_parameter("spring_fire_button", spring_fire_button_);
  get_parameter("ps_button", ps_button_);
  get_parameter("home_button", home_button_);
  get_parameter("circle_button", circle_button_);
  get_parameter("dribble_enable_button", dribble_enable_button_);

  get_parameter("left_trigger_axis", left_trigger_axis_);
  get_parameter("right_trigger_axis", right_trigger_axis_);
  get_parameter("left_stick_x_axis", left_stick_x_axis_);
  get_parameter("left_stick_y_axis", left_stick_y_axis_);
  get_parameter("right_stick_x_axis", right_stick_x_axis_);
  get_parameter("dpad_horizontal_axis", dpad_horizontal_axis_);
  get_parameter("dpad_vertical_axis", dpad_vertical_axis_);
}

void JoyControllerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_timeout_active_ = false;
  joy_received_ = true;
  last_joy_received_time_ = std::chrono::steady_clock::now();

  // 1. HOMEボタンで非常停止切替 (ACTIVE ↔ STOP)
  if (is_button_just_pressed(*msg, home_button_)) {
    is_emergency_stop_ = !is_emergency_stop_;
    publish_emergency_stop();
    if (is_emergency_stop_) {
      publish_stop_commands();
    } else {
      RCLCPP_INFO(get_logger(), "Emergency stop released. System is ACTIVE.");
    }
  }

  // 非常停止中でない場合のみ、各種操作を受け付ける
  if (!is_emergency_stop_) {
    // 2. R2 が押されていない時は、DPAD 上/下でベルトレベル昇降 (Level 0〜6)
    const bool is_r2_active = get_axis_value(*msg, right_trigger_axis_) <= -axis_on_threshold_;
    if (!is_r2_active) {
      if (is_axis_just_triggered(*msg, dpad_vertical_axis_, true)) {
        belt_rpm_mode_ = increment_mode(belt_rpm_mode_, static_cast<uint8_t>(BeltRpmMode::LEVEL_6));
        RCLCPP_INFO(get_logger(), "Belt level changed to: %u", belt_rpm_mode_);
        publish_belt_mode();
      }
      if (is_axis_just_triggered(*msg, dpad_vertical_axis_, false)) {
        belt_rpm_mode_ = decrement_mode(belt_rpm_mode_);
        RCLCPP_INFO(get_logger(), "Belt level changed to: %u", belt_rpm_mode_);
        publish_belt_mode();
      }
    }

    // 3. R1ボタンでドリブル回転ON/OFF
    if (is_button_just_pressed(*msg, dribble_enable_button_)) {
      dribble_enabled_ = !dribble_enabled_;
      RCLCPP_INFO(
        get_logger(), "Dribble toggled: %s",
        dribble_enabled_ ? "ON (2000 RPM)" : "OFF (0 RPM)");
      publish_dribble_enabled();
    }

    // 4. PSボタンで前後反転
    if (is_button_just_pressed(*msg, ps_button_)) {
      is_drive_reversed_ = !is_drive_reversed_;
      RCLCPP_INFO(
        get_logger(), "Drive direction toggled: %s",
        is_drive_reversed_ ? "REVERSED" : "FORWARD");
    }

    // 5. L2 + ○ ボタンで自動シュートサイクル要求
    if (get_axis_value(*msg, left_trigger_axis_) <= -axis_on_threshold_ &&
      is_button_just_pressed(*msg, circle_button_))
    {
      RCLCPP_INFO(get_logger(), "Shot cycle requested!");
      std_msgs::msg::Bool req; req.data = true;
      shot_cycle_request_pub_->publish(req);
    }

    // 6. R2 + DPAD で手動アーム位置切替 (R2+DPAD右: DRIBBLE / R2+DPAD左: OPEN / R2+DPAD上: FEED)
    if (is_r2_active) {
      std_msgs::msg::UInt8 arm_msg;
      if (is_axis_just_triggered(*msg, dpad_horizontal_axis_, true)) {
        arm_msg.data = static_cast<uint8_t>(ArmPositionMode::DRIBBLE);
        RCLCPP_INFO(get_logger(), "Manual Arm Position: DRIBBLE (0.35 rad)");
        arm_position_mode_pub_->publish(arm_msg);
      }
      if (is_axis_just_triggered(*msg, dpad_horizontal_axis_, false)) {
        arm_msg.data = static_cast<uint8_t>(ArmPositionMode::OPEN);
        RCLCPP_INFO(get_logger(), "Manual Arm Position: OPEN (-1.0 rad)");
        arm_position_mode_pub_->publish(arm_msg);
      }
      if (is_axis_just_triggered(*msg, dpad_vertical_axis_, true)) {
        arm_msg.data = static_cast<uint8_t>(ArmPositionMode::FEED);
        RCLCPP_INFO(get_logger(), "Manual Arm Position: FEED (1.3 rad)");
        arm_position_mode_pub_->publish(arm_msg);
      }
    }

    // 7. スティックでの足回り走行制御
    double linear_x = apply_axis_deadzone(get_axis_value(*msg, left_stick_y_axis_));
    double linear_y = -apply_axis_deadzone(get_axis_value(*msg, left_stick_x_axis_));
    double angular_z = apply_axis_deadzone(get_axis_value(*msg, right_stick_x_axis_));

    cmd_vel_.linear.x = apply_axis_limit(
      linear_x,
      linear_x_limit_) * linear_x_scale_ * (is_drive_reversed_ ? -1.0 : 1.0);
    cmd_vel_.linear.y = apply_axis_limit(
      linear_y,
      linear_y_limit_) * linear_y_scale_ * (is_drive_reversed_ ? -1.0 : 1.0);
    cmd_vel_.angular.z = apply_axis_limit(angular_z, angular_z_limit_) * angular_z_scale_;
  } else {
    cmd_vel_ = geometry_msgs::msg::Twist{};
  }

  mecanum_cmd_vel_pub_->publish(cmd_vel_);
  last_joy_msg_ = msg;
}

void JoyControllerNode::joy_timeout_timer_callback()
{
  if (!joy_received_) {return;}
  if (std::chrono::steady_clock::now() - last_joy_received_time_ >
    std::chrono::milliseconds(joy_timeout_ms_))
  {
    joy_timeout_active_ = true;
    publish_stop_commands();
  } else if (joy_timeout_active_) {
    joy_timeout_active_ = false;
  }
}

void JoyControllerNode::state_publish_timer_callback()
{
  if (!joy_timeout_active_ && last_joy_msg_) {
    publish_belt_mode();
    publish_dribble_enabled();
    publish_emergency_stop();

    // L1 + ○ ボタンでばね発射要求
    const bool spring_fire_requested = is_button_down(*last_joy_msg_, spring_fire_enable_button_) &&
      is_button_down(*last_joy_msg_, spring_fire_button_) &&
      !is_emergency_stop_;
    std_msgs::msg::Bool spring_msg; spring_msg.data = spring_fire_requested;
    spring_fire_pub_->publish(spring_msg);
  }
}

void JoyControllerNode::publish_emergency_stop()
{
  std_msgs::msg::Bool message;
  message.data = is_emergency_stop_;
  emergency_stop_pub_->publish(message);
}

void JoyControllerNode::publish_belt_mode()
{
  std_msgs::msg::UInt8 belt;
  belt.data = belt_rpm_mode_;
  belt_mode_pub_->publish(belt);
}

void JoyControllerNode::publish_dribble_enabled()
{
  std_msgs::msg::Bool dribble;
  dribble.data = dribble_enabled_;
  dribble_enabled_pub_->publish(dribble);
}

void JoyControllerNode::publish_stop_commands()
{
  cmd_vel_ = geometry_msgs::msg::Twist{};
  belt_rpm_mode_ = static_cast<uint8_t>(BeltRpmMode::STOP);
  dribble_enabled_ = false;
  is_emergency_stop_ = true;
  mecanum_cmd_vel_pub_->publish(cmd_vel_);

  std_msgs::msg::Bool spring_msg; spring_msg.data = false;
  spring_fire_pub_->publish(spring_msg);
  publish_belt_mode();
  publish_dribble_enabled();
  publish_emergency_stop();
}

bool JoyControllerNode::is_button_down(const sensor_msgs::msg::Joy & msg, int index) const
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.buttons.size()) {return false;}
  return msg.buttons[static_cast<std::size_t>(index)] != 0;
}

bool JoyControllerNode::is_button_just_pressed(const sensor_msgs::msg::Joy & msg, int index) const
{
  const bool curr = is_button_down(msg, index);
  const bool prev = last_joy_msg_ ? is_button_down(*last_joy_msg_, index) : false;
  return curr && !prev;
}

double JoyControllerNode::get_axis_value(const sensor_msgs::msg::Joy & msg, int index) const
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.axes.size()) {return 0.0;}
  const double value = msg.axes[static_cast<std::size_t>(index)];
  return std::isfinite(value) ? value : 0.0;
}

bool JoyControllerNode::is_axis_just_triggered(
  const sensor_msgs::msg::Joy & msg, int index,
  bool positive) const
{
  const double val = get_axis_value(msg, index);
  const double prev_val = last_joy_msg_ ? get_axis_value(*last_joy_msg_, index) : 0.0;
  return positive ? (val >= axis_on_threshold_ && prev_val < axis_on_threshold_) :
         (val <= -axis_on_threshold_ && prev_val > -axis_on_threshold_);
}

double JoyControllerNode::apply_axis_deadzone(double value) const
{
  const double abs_val = std::abs(value);
  if (abs_val < axis_deadzone_ || axis_deadzone_ >= 1.0) {return 0.0;}
  const double scaled = (abs_val - axis_deadzone_) / (1.0 - axis_deadzone_);
  return (value > 0.0) ? scaled : -scaled;
}

double JoyControllerNode::apply_axis_limit(double value, double limit)
{
  return value * std::abs(limit);
}

uint8_t JoyControllerNode::increment_mode(uint8_t mode, uint8_t maximum_mode)
{
  return (mode < maximum_mode) ? static_cast<uint8_t>(mode + 1) : maximum_mode;
}

uint8_t JoyControllerNode::decrement_mode(uint8_t mode)
{
  return (mode >
         static_cast<uint8_t>(BeltRpmMode::STOP)) ? static_cast<uint8_t>(mode -
         1) : static_cast<uint8_t>(BeltRpmMode::STOP);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyControllerNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
