#include "joy_controller/joy_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

JoyControllerNode::JoyControllerNode()
: Node("joy_controller_node")
{
  command_qos_depth_ = declare_parameter<int>("command_qos_depth", 1);
  joy_timeout_ms_ = declare_parameter<int>("joy_timeout_ms", 200);
  state_publish_period_ms_ = declare_parameter<int>("state_publish_period_ms", 20);

  max_vel_x_m_s_ = declare_parameter<double>("linear_x_limit", 2.0);
  max_vel_y_m_s_ = declare_parameter<double>("linear_y_limit", 2.0);
  max_vel_z_rad_s_ = declare_parameter<double>("angular_z_limit", 2.0);
  acceleration_x_m_s2_ = declare_parameter<double>("linear_x_acceleration_limit", 2.0);
  acceleration_y_m_s2_ = declare_parameter<double>("linear_y_acceleration_limit", 2.0);
  acceleration_yaw_rad_s2_ = declare_parameter<double>("angular_z_acceleration_limit", 4.0);
  deceleration_x_m_s2_ = declare_parameter<double>("linear_x_deceleration_limit", 3.0);
  deceleration_y_m_s2_ = declare_parameter<double>("linear_y_deceleration_limit", 3.0);
  deceleration_yaw_rad_s2_ = declare_parameter<double>("angular_z_deceleration_limit", 6.0);
  axis_deadzone_ = declare_parameter<double>("axis_deadzone", 0.05);
  axis_on_threshold_ = declare_parameter<double>("axis_on_threshold", 0.7);

  ps_button_ = declare_parameter<int>("ps_button", 12);
  home_button_ = declare_parameter<int>("home_button", 13);
  circle_button_ = declare_parameter<int>("circle_button", 2);
  dribble_enable_button_ = declare_parameter<int>("dribble_enable_button", 5);
  game2_start_button_ = declare_parameter<int>("game2_start_button", 9);

  left_trigger_axis_ = declare_parameter<int>("left_trigger_axis", 3);
  right_trigger_axis_ = declare_parameter<int>("right_trigger_axis", 4);
  left_stick_x_axis_ = declare_parameter<int>("left_stick_x_axis", 0);
  left_stick_y_axis_ = declare_parameter<int>("left_stick_y_axis", 1);
  right_stick_x_axis_ = declare_parameter<int>("right_stick_x_axis", 2);
  dpad_horizontal_axis_ = declare_parameter<int>("dpad_horizontal_axis", 6);
  dpad_vertical_axis_ = declare_parameter<int>("dpad_vertical_axis", 7);

  if (command_qos_depth_ <= 0 || joy_timeout_ms_ <= 0 ||
    state_publish_period_ms_ <= 0)
  {
    throw std::runtime_error("QoS depth and timer periods must be positive");
  }
  if (!std::isfinite(max_vel_x_m_s_) || max_vel_x_m_s_ < 0.0 ||
    !std::isfinite(max_vel_y_m_s_) || max_vel_y_m_s_ < 0.0 ||
    !std::isfinite(max_vel_z_rad_s_) || max_vel_z_rad_s_ < 0.0 ||
    !std::isfinite(acceleration_x_m_s2_) || acceleration_x_m_s2_ <= 0.0 ||
    !std::isfinite(acceleration_y_m_s2_) || acceleration_y_m_s2_ <= 0.0 ||
    !std::isfinite(acceleration_yaw_rad_s2_) || acceleration_yaw_rad_s2_ <= 0.0 ||
    !std::isfinite(deceleration_x_m_s2_) || deceleration_x_m_s2_ <= 0.0 ||
    !std::isfinite(deceleration_y_m_s2_) || deceleration_y_m_s2_ <= 0.0 ||
    !std::isfinite(deceleration_yaw_rad_s2_) || deceleration_yaw_rad_s2_ <= 0.0)
  {
    throw std::runtime_error("velocity limits must be nonnegative and rate limits positive");
  }
  update_acceleration_limits();

  // joy_node -> joy_controller: 最新性を優先する操作入力。
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SensorDataQoS(),
    std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

  const auto command_qos = rclcpp::QoS(command_qos_depth_);
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();
  emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", emergency_stop_qos);

  mecanum_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/mecanum/cmd_vel", command_qos);

  spring_fire_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos);

  belt_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos);

  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/enabled", command_qos);

  shot_cycle_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/shot_cycle/request", command_qos);

  arm_position_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/dribble/position_mode", command_qos);

  game2_start_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/game2/start", command_qos);
  // joy_controller -> led_controller: drive direction selected with the PS button.
  drive_reversed_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/drive/reversed", rclcpp::QoS(1).reliable().transient_local());


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

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(
      &JoyControllerNode::parameter_callback, this,
      std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult JoyControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  int joy_timeout_ms = joy_timeout_ms_;
  double max_vel_x_m_s = max_vel_x_m_s_;
  double max_vel_y_m_s = max_vel_y_m_s_;
  double max_vel_z_rad_s = max_vel_z_rad_s_;
  double acceleration_x_m_s2 = acceleration_x_m_s2_;
  double acceleration_y_m_s2 = acceleration_y_m_s2_;
  double acceleration_yaw_rad_s2 = acceleration_yaw_rad_s2_;
  double deceleration_x_m_s2 = deceleration_x_m_s2_;
  double deceleration_y_m_s2 = deceleration_y_m_s2_;
  double deceleration_yaw_rad_s2 = deceleration_yaw_rad_s2_;
  double axis_deadzone = axis_deadzone_;
  double axis_on_threshold = axis_on_threshold_;
  int ps_button = ps_button_;
  int home_button = home_button_;
  int circle_button = circle_button_;
  int dribble_enable_button = dribble_enable_button_;
  int game2_start_button = game2_start_button_;
  int left_trigger_axis = left_trigger_axis_;
  int right_trigger_axis = right_trigger_axis_;
  int left_stick_x_axis = left_stick_x_axis_;
  int left_stick_y_axis = left_stick_y_axis_;
  int right_stick_x_axis = right_stick_x_axis_;
  int dpad_horizontal_axis = dpad_horizontal_axis_;
  int dpad_vertical_axis = dpad_vertical_axis_;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "command_qos_depth") {
      if (parameter.as_int() != command_qos_depth_) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "command_qos_depth requires a node restart";
        return result;
      }
    } else if (name == "state_publish_period_ms") {
      if (parameter.as_int() != state_publish_period_ms_) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "state_publish_period_ms requires a node restart";
        return result;
      }
    } else if (name == "joy_timeout_ms") {
      joy_timeout_ms = static_cast<int>(parameter.as_int());
    } else if (name == "linear_x_limit") {
      max_vel_x_m_s = parameter.as_double();
    } else if (name == "linear_y_limit") {
      max_vel_y_m_s = parameter.as_double();
    } else if (name == "angular_z_limit") {
      max_vel_z_rad_s = parameter.as_double();
    } else if (name == "linear_x_acceleration_limit") {
      acceleration_x_m_s2 = parameter.as_double();
    } else if (name == "linear_y_acceleration_limit") {
      acceleration_y_m_s2 = parameter.as_double();
    } else if (name == "angular_z_acceleration_limit") {
      acceleration_yaw_rad_s2 = parameter.as_double();
    } else if (name == "linear_x_deceleration_limit") {
      deceleration_x_m_s2 = parameter.as_double();
    } else if (name == "linear_y_deceleration_limit") {
      deceleration_y_m_s2 = parameter.as_double();
    } else if (name == "angular_z_deceleration_limit") {
      deceleration_yaw_rad_s2 = parameter.as_double();
    } else if (name == "axis_deadzone") {
      axis_deadzone = parameter.as_double();
    } else if (name == "axis_on_threshold") {
      axis_on_threshold = parameter.as_double();
    } else if (name == "ps_button") {
      ps_button = static_cast<int>(parameter.as_int());
    } else if (name == "home_button") {
      home_button = static_cast<int>(parameter.as_int());
    } else if (name == "circle_button") {
      circle_button = static_cast<int>(parameter.as_int());
    } else if (name == "dribble_enable_button") {
      dribble_enable_button = static_cast<int>(parameter.as_int());
    } else if (name == "game2_start_button") {
      game2_start_button = static_cast<int>(parameter.as_int());
    } else if (name == "left_trigger_axis") {
      left_trigger_axis = static_cast<int>(parameter.as_int());
    } else if (name == "right_trigger_axis") {
      right_trigger_axis = static_cast<int>(parameter.as_int());
    } else if (name == "left_stick_x_axis") {
      left_stick_x_axis = static_cast<int>(parameter.as_int());
    } else if (name == "left_stick_y_axis") {
      left_stick_y_axis = static_cast<int>(parameter.as_int());
    } else if (name == "right_stick_x_axis") {
      right_stick_x_axis = static_cast<int>(parameter.as_int());
    } else if (name == "dpad_horizontal_axis") {
      dpad_horizontal_axis = static_cast<int>(parameter.as_int());
    } else if (name == "dpad_vertical_axis") {
      dpad_vertical_axis = static_cast<int>(parameter.as_int());
    } else {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = name + " cannot be changed while the node is running";
      return result;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful =
    joy_timeout_ms > 0 &&
    std::isfinite(max_vel_x_m_s) && max_vel_x_m_s >= 0.0 &&
    std::isfinite(max_vel_y_m_s) && max_vel_y_m_s >= 0.0 &&
    std::isfinite(max_vel_z_rad_s) && max_vel_z_rad_s >= 0.0 &&
    std::isfinite(acceleration_x_m_s2) && acceleration_x_m_s2 > 0.0 &&
    std::isfinite(acceleration_y_m_s2) && acceleration_y_m_s2 > 0.0 &&
    std::isfinite(acceleration_yaw_rad_s2) && acceleration_yaw_rad_s2 > 0.0 &&
    std::isfinite(deceleration_x_m_s2) && deceleration_x_m_s2 > 0.0 &&
    std::isfinite(deceleration_y_m_s2) && deceleration_y_m_s2 > 0.0 &&
    std::isfinite(deceleration_yaw_rad_s2) && deceleration_yaw_rad_s2 > 0.0 &&
    std::isfinite(axis_deadzone) && axis_deadzone >= 0.0 &&
    axis_deadzone<1.0 && std::isfinite(axis_on_threshold) &&
      axis_on_threshold>0.0 && axis_on_threshold <= 1.0 &&
    ps_button >= 0 && home_button >= 0 && circle_button >= 0 &&
    dribble_enable_button >= 0 && game2_start_button >= 0 &&
    left_trigger_axis >= 0 && right_trigger_axis >= 0 &&
    left_stick_x_axis >= 0 && left_stick_y_axis >= 0 &&
    right_stick_x_axis >= 0 && dpad_horizontal_axis >= 0 &&
    dpad_vertical_axis >= 0;
  if (!result.successful) {
    result.reason = "Joy parameters contain an invalid value";
    return result;
  }

  joy_timeout_ms_ = joy_timeout_ms;
  max_vel_x_m_s_ = max_vel_x_m_s;
  max_vel_y_m_s_ = max_vel_y_m_s;
  max_vel_z_rad_s_ = max_vel_z_rad_s;
  acceleration_x_m_s2_ = acceleration_x_m_s2;
  acceleration_y_m_s2_ = acceleration_y_m_s2;
  acceleration_yaw_rad_s2_ = acceleration_yaw_rad_s2;
  deceleration_x_m_s2_ = deceleration_x_m_s2;
  deceleration_y_m_s2_ = deceleration_y_m_s2;
  deceleration_yaw_rad_s2_ = deceleration_yaw_rad_s2;
  update_acceleration_limits();
  axis_deadzone_ = axis_deadzone;
  axis_on_threshold_ = axis_on_threshold;
  ps_button_ = ps_button;
  home_button_ = home_button;
  circle_button_ = circle_button;
  dribble_enable_button_ = dribble_enable_button;
  game2_start_button_ = game2_start_button;
  left_trigger_axis_ = left_trigger_axis;
  right_trigger_axis_ = right_trigger_axis;
  left_stick_x_axis_ = left_stick_x_axis;
  left_stick_y_axis_ = left_stick_y_axis;
  right_stick_x_axis_ = right_stick_x_axis;
  dpad_horizontal_axis_ = dpad_horizontal_axis;
  dpad_vertical_axis_ = dpad_vertical_axis;

  if (joy_received_) {
    last_joy_msg_ = joy_msg_;
  }
  return result;
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
  if (!joy_received_ || joy_timeout_active_) {
    return;
  }

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
    // 2. R2 が押されていない時は、DPAD 上/下でベルトレベル昇降 (Level 0〜4)
    const bool is_l2_active = get_axis_value(joy_msg_, left_trigger_axis_) <= -axis_on_threshold_;
    const bool is_r2_active = get_axis_value(joy_msg_, right_trigger_axis_) <= -axis_on_threshold_;
    if (!is_r2_active) {
      if (is_axis_just_triggered(joy_msg_, dpad_vertical_axis_, true)) {
        belt_rpm_mode_ = increment_mode(
          belt_rpm_mode_, static_cast<uint8_t>(BeltRpmMode::LEVEL_4));
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
      publish_drive_reversed();
      RCLCPP_INFO(
        get_logger(), "Drive direction toggled: %s",
        is_drive_reversed_ ? "REVERSED" : "FORWARD");
    }

    // 5. L2 + ○ ボタンで自動シュートサイクル要求
    if (is_l2_active && is_button_just_pressed(joy_msg_, circle_button_)) {
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
      double raw_wz = apply_axis_deadzone(-get_axis_value(joy_msg_, right_stick_x_axis_));

      if (raw_vx != 0.0 || raw_vy != 0.0 || raw_wz != 0.0) {
        game2_active_ = false;
        RCLCPP_WARN(get_logger(), "⚠️ Manual stick input detected! Game 2 AUTO mode DISENGAGED.");
        std_msgs::msg::Bool game2_msg;
        game2_msg.data = false;
        game2_start_pub_->publish(game2_msg);
      }
    }

    // 8. L2とR2を同時に押した瞬間にスプリングを1回発射
    const bool was_l2_active = last_joy_msg_.has_value() && get_axis_value(
      last_joy_msg_.value(), left_trigger_axis_) <= -axis_on_threshold_;
    const bool was_r2_active = last_joy_msg_.has_value() && get_axis_value(
      last_joy_msg_.value(), right_trigger_axis_) <= -axis_on_threshold_;
    const bool spring_fire_triggered = is_l2_active && is_r2_active &&
      !(was_l2_active && was_r2_active);
    if (spring_fire_triggered) {
      RCLCPP_INFO(get_logger(), "Spring firing triggered!");
    }
    std_msgs::msg::Bool spring_fire_msg;
    spring_fire_msg.data = spring_fire_triggered;
    spring_fire_pub_->publish(spring_fire_msg);

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
      double raw_vy = get_axis_value(joy_msg_, left_stick_x_axis_);
      double raw_wz = -get_axis_value(joy_msg_, right_stick_x_axis_);

      if (is_drive_reversed_) {
        raw_vx = -raw_vx;
        raw_vy = -raw_vy;
      }

      publish_limited_velocity(
        apply_axis_deadzone(raw_vx) * max_vel_x_m_s_,
        apply_axis_deadzone(raw_vy) * max_vel_y_m_s_,
        apply_axis_deadzone(raw_wz) * max_vel_z_rad_s_);
    }

  }

  last_joy_msg_ = joy_msg_;
}

void JoyControllerNode::joy_timeout_timer_callback()
{
  if (!joy_received_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_joy_received_time_).count();

  if (elapsed > joy_timeout_ms_ && !joy_timeout_active_) {
    joy_timeout_active_ = true;
    last_joy_msg_.reset();
    RCLCPP_WARN(get_logger(), "Joy message timeout (%ld ms). Stopping robot.", elapsed);
    publish_stop_commands();
  }
}

void JoyControllerNode::state_publish_timer_callback()
{
  publish_emergency_stop();
  publish_belt_mode();
  publish_dribble_enabled();
  publish_drive_reversed();
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
void JoyControllerNode::publish_drive_reversed()
{
  std_msgs::msg::Bool msg;
  msg.data = is_drive_reversed_;
  drive_reversed_pub_->publish(msg);
}


void JoyControllerNode::publish_stop_commands()
{
  velocity_limiter_x_.reset();
  velocity_limiter_y_.reset();
  velocity_limiter_yaw_.reset();
  velocity_limiter_initialized_ = false;
  cmd_vel_ = geometry_msgs::msg::Twist{};
  mecanum_cmd_vel_pub_->publish(cmd_vel_);

  std_msgs::msg::Bool spring_fire_msg;
  spring_fire_msg.data = false;
  spring_fire_pub_->publish(spring_fire_msg);

  belt_rpm_mode_ = static_cast<uint8_t>(BeltRpmMode::STOP);
  publish_belt_mode();

  dribble_enabled_ = false;
  publish_dribble_enabled();
}

void JoyControllerNode::publish_limited_velocity(
  const double target_x_m_s, const double target_y_m_s, const double target_yaw_rad_s)
{
  const auto current_time = std::chrono::steady_clock::now();
  double dt_sec = 0.01;
  if (velocity_limiter_initialized_) {
    dt_sec = std::chrono::duration<double>(current_time - last_velocity_update_time_).count();
    dt_sec = std::clamp(dt_sec, 0.001, 0.1);
  } else {
    velocity_limiter_initialized_ = true;
  }
  last_velocity_update_time_ = current_time;

  cmd_vel_.linear.x = velocity_limiter_x_.update(target_x_m_s, dt_sec);
  cmd_vel_.linear.y = velocity_limiter_y_.update(target_y_m_s, dt_sec);
  cmd_vel_.angular.z = velocity_limiter_yaw_.update(target_yaw_rad_s, dt_sec);
  mecanum_cmd_vel_pub_->publish(cmd_vel_);
}

void JoyControllerNode::update_acceleration_limits()
{
  velocity_limiter_x_.set_limits(acceleration_x_m_s2_, deceleration_x_m_s2_);
  velocity_limiter_y_.set_limits(acceleration_y_m_s2_, deceleration_y_m_s2_);
  velocity_limiter_yaw_.set_limits(acceleration_yaw_rad_s2_, deceleration_yaw_rad_s2_);
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
  const double value = msg.axes[index];
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, -1.0, 1.0);
}

bool JoyControllerNode::is_axis_just_triggered(
  const sensor_msgs::msg::Joy & msg, int index, bool positive) const
{
  const double current_val = get_axis_value(msg, index);
  const double last_val =
    last_joy_msg_.has_value() ? get_axis_value(last_joy_msg_.value(), index) : 0.0;

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
