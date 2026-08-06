#include "joy_controller/joy_controller_node.hpp"

#include <algorithm>

JoyControllerNode::JoyControllerNode()
: Node("joy_controller_node")
{
  declare_parameters();
  get_parameters();

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::QoS(joy_qos_depth_),
    std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(command_qos_depth_));

  mecanum_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/mecanum/cmd_vel", rclcpp::QoS(command_qos_depth_));

  spring_fire_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/spring/fire", rclcpp::QoS(command_qos_depth_));

  belt_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/belt/mode", rclcpp::QoS(command_qos_depth_));

  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/enabled", rclcpp::QoS(command_qos_depth_));

  shot_cycle_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/shot_cycle/request", rclcpp::QoS(command_qos_depth_));

  arm_position_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/dribble/position_mode", rclcpp::QoS(command_qos_depth_));

  game2_start_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/game2/start", rclcpp::QoS(command_qos_depth_));

  publish_stop_commands();

  joy_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&JoyControllerNode::joy_timeout_timer_callback, this));

  state_publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(state_publish_period_ms_),
    std::bind(&JoyControllerNode::state_publish_timer_callback, this));

  loop_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&JoyControllerNode::loop_callback, this));
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
  declare_parameter<int>("game2_start_button", 9);

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
  get_parameter("game2_start_button", game2_start_button_);

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
  joy_msg_ = *msg;
}

void JoyControllerNode::loop_callback()
{

  // 1. HOMEボタンで非常停止切替 (ACTIVE ↔ STOP)
  if (is_button_just_pressed(joy_msg_, home_button_)) {
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
    const bool is_r2_active = get_axis_value(joy_msg_, right_trigger_axis_) <= -axis_on_threshold_;
    if (!is_r2_active) {
      if (is_axis_just_triggered(joy_msg_, dpad_vertical_axis_, true)) {
        belt_rpm_mode_ = increment_mode(belt_rpm_mode_, static_cast<uint8_t>(BeltRpmMode::LEVEL_6));
        RCLCPP_INFO(get_logger(), "Belt level changed to: %u", belt_rpm_mode_);
        publish_belt_mode();
      }
      if (is_axis_just_triggered(joy_msg_, dpad_vertical_axis_, false)) {
        belt_rpm_mode_ = decrement_mode(belt_rpm_mode_);
        RCLCPP_INFO(get_logger(), "Belt level changed to: %u", belt_rpm_mode_);
        publish_belt_mode();
      }
    }

    // 3. R1ボタンでドリブル回転ON/OFF
    if (is_button_just_pressed(joy_msg_, dribble_enable_button_)) {
      dribble_enabled_ = !dribble_enabled_;
      RCLCPP_INFO(
        get_logger(), "Dribble toggled: %s",
        dribble_enabled_ ? "ON (2000 RPM)" : "OFF (0 RPM)");
      publish_dribble_enabled();
    }

    // 4. PSボタンで前後反転
    if (is_button_just_pressed(joy_msg_, ps_button_)) {
      is_drive_reversed_ = !is_drive_reversed_;
      RCLCPP_INFO(
        get_logger(), "Drive direction toggled: %s",
        is_drive_reversed_ ? "REVERSED" : "FORWARD");
    }

    // 5. L2 + ○ ボタンで自動シュートサイクル要求
    if (get_axis_value(joy_msg_, left_trigger_axis_) <= -axis_on_threshold_ &&
      is_button_just_pressed(joy_msg_, circle_button_))
    {
      RCLCPP_INFO(get_logger(), "Shot cycle requested!");
      std_msgs::msg::Bool req; req.data = true;
      shot_cycle_request_pub_->publish(req);
    }

    // 6. OPTIONSボタン (ボタン9) で Game 2 自動戦術モード開始/停止切替
    if (is_button_just_pressed(joy_msg_, game2_start_button_)) {
      game2_active_ = !game2_active_;
      RCLCPP_INFO(
        get_logger(), "🎮 Game 2 mode toggled: %s",
        game2_active_ ? "START (AUTO ON)" : "STOP (STANDBY)");
      std_msgs::msg::Bool game2_msg;
      game2_msg.data = game2_active_;
      game2_start_pub_->publish(game2_msg);
    }

    // 7. 手動オーバーライド: Game 2 自動モード中に人間がスティックを動かしたら即座に自動解除！
    if (game2_active_) {
      double raw_vx = apply_axis_deadzone(get_axis_value(joy_msg_, left_stick_y_axis_));
      double raw_vy = apply_axis_deadzone(get_axis_value(joy_msg_, left_stick_x_axis_));
      double raw_wz = apply_axis_deadzone(get_axis_value(joy_msg_, right_stick_x_axis_));

      if (raw_vx != 0.0 || raw_vy != 0.0 || raw_wz != 0.0) {
        game2_active_ = false;
        RCLCPP_WARN(get_logger(), "⚠️ Manual stick input detected! Game 2 AUTO mode DISENGAGED.");
        std_msgs::msg::Bool game2_msg;
        game2_msg.data = false;
        game2_start_pub_->publish(game2_msg);
      }
    }

    // 8. L1 + R1 + △ボタンでスプリング射出 (セーフティ解除付き)
    const bool is_l1_down = is_button_down(joy_msg_, spring_fire_enable_button_);
    const bool is_r1_down = is_button_down(joy_msg_, dribble_enable_button_);
    if (is_l1_down && is_r1_down && is_button_just_pressed(joy_msg_, spring_fire_button_)) {
      RCLCPP_INFO(get_logger(), "Spring firing triggered!");
      std_msgs::msg::Bool msg; msg.data = true;
      spring_fire_pub_->publish(msg);
    }

    // 9. DPAD 左右でアームポジション切替 (DRIBBLE / OPEN / FEED)
    if (is_axis_just_triggered(joy_msg_, dpad_horizontal_axis_, true)) {
      std_msgs::msg::UInt8 mode_msg;
      mode_msg.data = static_cast<uint8_t>(ArmPositionMode::OPEN);
      arm_position_mode_pub_->publish(mode_msg);
      RCLCPP_INFO(get_logger(), "Arm position set to: OPEN");
    } else if (is_axis_just_triggered(joy_msg_, dpad_horizontal_axis_, false)) {
      std_msgs::msg::UInt8 mode_msg;
      mode_msg.data = static_cast<uint8_t>(ArmPositionMode::FEED);
      arm_position_mode_pub_->publish(mode_msg);
      RCLCPP_INFO(get_logger(), "Arm position set to: FEED");
    }

    // 10. アナログスティックによるメカナム走行速度の算出 (Game 2 モード非アクティブ時)
    if (!game2_active_) {
      double raw_vx = get_axis_value(joy_msg_, left_stick_y_axis_);
      double raw_vy = -get_axis_value(joy_msg_, left_stick_x_axis_);
      double raw_wz = -get_axis_value(joy_msg_, right_stick_x_axis_);

      if (is_drive_reversed_) {
        raw_vx = -raw_vx;
        raw_vy = -raw_vy;
      }

      cmd_vel_.linear.x = apply_axis_limit(
        apply_axis_deadzone(raw_vx) * linear_x_scale_,
        linear_x_limit_);

      cmd_vel_.linear.y = apply_axis_limit(
        apply_axis_deadzone(raw_vy) * linear_y_scale_,
        linear_y_limit_);

      cmd_vel_.angular.z = apply_axis_limit(
        apply_axis_deadzone(raw_wz) * angular_z_scale_,
        angular_z_limit_);

      mecanum_cmd_vel_pub_->publish(cmd_vel_);
    }

    last_joy_msg_ = joy_msg_;
  }
}

void JoyControllerNode::joy_timeout_timer_callback()
{
  if (!joy_received_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_joy_received_time_).count();

  if (elapsed > joy_timeout_ms_ && !joy_timeout_active_) {
    joy_timeout_active_ = true;
    RCLCPP_WARN(get_logger(), "Joy message timeout (%ld ms). Stopping robot.", elapsed);
    publish_stop_commands();
  }
}

void JoyControllerNode::state_publish_timer_callback()
{
  publish_emergency_stop();
  publish_belt_mode();
  publish_dribble_enabled();
}

void JoyControllerNode::publish_emergency_stop()
{
  std_msgs::msg::Bool msg;
  msg.data = is_emergency_stop_;
  emergency_stop_pub_->publish(msg);
}

void JoyControllerNode::publish_belt_mode()
{
  std_msgs::msg::UInt8 msg;
  msg.data = belt_rpm_mode_;
  belt_mode_pub_->publish(msg);
}

void JoyControllerNode::publish_dribble_enabled()
{
  std_msgs::msg::Bool msg;
  msg.data = dribble_enabled_;
  dribble_enabled_pub_->publish(msg);
}

void JoyControllerNode::publish_stop_commands()
{
  cmd_vel_ = geometry_msgs::msg::Twist{};
  mecanum_cmd_vel_pub_->publish(cmd_vel_);

  belt_rpm_mode_ = static_cast<uint8_t>(BeltRpmMode::STOP);
  publish_belt_mode();

  dribble_enabled_ = false;
  publish_dribble_enabled();
}

bool JoyControllerNode::is_button_down(const sensor_msgs::msg::Joy & msg, int index) const
{
  if (index < 0 || static_cast<size_t>(index) >= msg.buttons.size()) {
    return false;
  }
  return msg.buttons[index] != 0;
}

bool JoyControllerNode::is_button_just_pressed(const sensor_msgs::msg::Joy & msg, int index) const
{
  if (!last_joy_msg_.has_value()) {
    return is_button_down(msg, index);
  }
  return is_button_down(msg, index) && !is_button_down(last_joy_msg_.value(), index);
}

double JoyControllerNode::get_axis_value(const sensor_msgs::msg::Joy & msg, int index) const
{
  if (index < 0 || static_cast<size_t>(index) >= msg.axes.size()) {
    return 0.0;
  }
  return msg.axes[index];
}

bool JoyControllerNode::is_axis_just_triggered(
  const sensor_msgs::msg::Joy & msg, int index, bool positive) const
{
  const double current_val = get_axis_value(msg, index);
  const double last_val = last_joy_msg_.has_value() ? get_axis_value(last_joy_msg_.value(), index) : 0.0;

  if (positive) {
    return current_val >= axis_on_threshold_ && last_val < axis_on_threshold_;
  } else {
    return current_val <= -axis_on_threshold_ && last_val > -axis_on_threshold_;
  }
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
  return std::clamp(value, -limit, limit);
}

uint8_t JoyControllerNode::increment_mode(uint8_t mode, uint8_t maximum_mode)
{
  if (mode < maximum_mode) {
    return mode + 1;
  }
  return maximum_mode;
}

uint8_t JoyControllerNode::decrement_mode(uint8_t mode)
{
  if (mode > 0) {
    return mode - 1;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
