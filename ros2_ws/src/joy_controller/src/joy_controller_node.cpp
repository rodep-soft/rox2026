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

  is_emergency_stop_ = true;

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
  dribble_reverse_button_ = declare_parameter<int>("dribble_reverse_button", -1);
  game2_start_button_ = declare_parameter<int>("game2_start_button", 9);
  heading_hold_toggle_button_ = declare_parameter<int>("heading_hold_toggle_button", 8);
  slow_turn_button_ = declare_parameter<int>("slow_turn_button", 7);
  slow_turn_scale_ = declare_parameter<double>("slow_turn_scale", 0.5);
  slow_linear_scale_ = declare_parameter<double>("slow_linear_scale", 0.5);

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
    !std::isfinite(slow_turn_scale_) || slow_turn_scale_ < 0.0 ||
    !std::isfinite(slow_linear_scale_) || slow_linear_scale_ < 0.0 ||
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
    "/system/emergency_stop", emergency_stop_qos);

  spring_actuator_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/actuator_ready", command_qos,
    std::bind(
      &JoyControllerNode::spring_actuator_ready_callback, this,
      std::placeholders::_1));

  mecanum_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/drive/cmd_vel", command_qos);

  spring_fire_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos);

  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", command_qos);

  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/command_enabled", command_qos);

  dribble_reverse_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/command_reverse", command_qos);

  spring_decel_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/spring_decel", command_qos);

  shot_cycle_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", command_qos);

  arm_position_mode_pub_ = create_publisher<robot_msgs::msg::ArmPosition>(
    "/dribble/command_position", command_qos);

  opening_rpm_pub_ = create_publisher<std_msgs::msg::Int32>(
    "/dribble/command_opening_rpm", command_qos);

  game2_start_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/game2/command_start", command_qos);

  heading_control_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/heading_control/enable", rclcpp::QoS(1).reliable().transient_local());

  drive_reversed_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/drive/reversed", rclcpp::QoS(1).reliable().transient_local());

  // 初期状態で Heading Hold を ON でパブリッシュ
  std_msgs::msg::Bool initial_heading_enable_msg;
  initial_heading_enable_msg.data = is_heading_hold_enabled_;
  heading_control_enable_pub_->publish(initial_heading_enable_msg);

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
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    // 再起動が必要なパラメータの変更を拒否
    if (name == "command_qos_depth" || name == "state_publish_period_ms") {
      if (param.as_int() !=
        (name == "command_qos_depth" ? command_qos_depth_ : state_publish_period_ms_))
      {
        result.successful = false;
        result.reason = name + " requires a node restart";
        return result;
      }
      continue;
    }

    // パラメータ値の適用 (int / double 自動分岐)
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      const int val = static_cast<int>(param.as_int());
      if (val < 0 && name != "joy_timeout_ms" && name != "dribble_reverse_button" &&
        name != "slow_turn_button")
      {
        result.successful = false;
        result.reason = name + " must be non-negative";
        return result;
      }
      if (val < -1 && (name == "dribble_reverse_button" || name == "slow_turn_button")) {
        result.successful = false;
        result.reason = name + " must be >= -1";
        return result;
      }
      if (name == "joy_timeout_ms") {joy_timeout_ms_ = val;} else if (name == "ps_button") {
        ps_button_ = val;
      } else if (name == "home_button") {home_button_ = val;} else if (name == "circle_button") {
        circle_button_ = val;
      } else if (name == "dribble_enable_button") {
        dribble_enable_button_ = val;
      } else if (name == "dribble_reverse_button") {
        dribble_reverse_button_ = val;
      } else if (name == "game2_start_button") {
        game2_start_button_ = val;
      } else if (name == "heading_hold_toggle_button") {
        heading_hold_toggle_button_ = val;
      } else if (name == "slow_turn_button") {
        slow_turn_button_ = val;
      } else if (name == "left_trigger_axis") {
        left_trigger_axis_ = val;
      } else if (name == "right_trigger_axis") {
        right_trigger_axis_ = val;
      } else if (name == "left_stick_x_axis") {
        left_stick_x_axis_ = val;
      } else if (name == "left_stick_y_axis") {
        left_stick_y_axis_ = val;
      } else if (name == "right_stick_x_axis") {
        right_stick_x_axis_ = val;
      } else if (name == "dpad_horizontal_axis") {
        dpad_horizontal_axis_ = val;
      } else if (name == "dpad_vertical_axis") {dpad_vertical_axis_ = val;}
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double val = param.as_double();
      if (!std::isfinite(val) || val < 0.0) {
        result.successful = false;
        result.reason = name + " must be valid positive number";
        return result;
      }
      if (name == "linear_x_limit") {max_vel_x_m_s_ = val;} else if (name == "linear_y_limit") {
        max_vel_y_m_s_ = val;
      } else if (name == "angular_z_limit") {
        max_vel_z_rad_s_ = val;
      } else if (name == "linear_x_acceleration_limit") {
        acceleration_x_m_s2_ = val;
      } else if (name == "linear_y_acceleration_limit") {
        acceleration_y_m_s2_ = val;
      } else if (name == "angular_z_acceleration_limit") {
        acceleration_yaw_rad_s2_ = val;
      } else if (name == "linear_x_deceleration_limit") {
        deceleration_x_m_s2_ = val;
      } else if (name == "linear_y_deceleration_limit") {
        deceleration_y_m_s2_ = val;
      } else if (name == "angular_z_deceleration_limit") {
        deceleration_yaw_rad_s2_ = val;
      } else if (name == "axis_deadzone") {
        axis_deadzone_ = val;
      } else if (name == "axis_on_threshold") {
        axis_on_threshold_ = val;
      } else if (name == "slow_turn_scale" || name == "slow_turn_ratio") {
        slow_turn_scale_ = val;
      } else if (name == "slow_linear_scale" || name == "slow_linear_ratio") {
        slow_linear_scale_ = val;
      }
    }
  }

  update_acceleration_limits();
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
    publish_emergency_stop(is_emergency_stop_);
    if (is_emergency_stop_) {
      publish_stop_commands();
      last_joy_msg_ = joy_msg_;
      return;
    }
    RCLCPP_INFO(get_logger(), "Emergency stop released. System is ACTIVE.");
  }

  // 非常停止中は各種操作を受け付けない
  if (is_emergency_stop_) {
    last_joy_msg_ = joy_msg_;
    return;
  }

  const bool is_l2_active = get_axis_value(joy_msg_, left_trigger_axis_) <= -axis_on_threshold_;
  const bool is_r2_active = get_axis_value(joy_msg_, right_trigger_axis_) <= -axis_on_threshold_;

  // 2. DPAD 入力処理
  if (is_r2_active) {
    // R2 + DPAD 左右で手動アーム位置変更 (左: DRIBBLE, 右: OPEN)
    if (is_axis_just_triggered(joy_msg_, dpad_horizontal_axis_, false)) { // DPAD 右 (-1.0)
      robot_msgs::msg::ArmPosition arm_msg;
      arm_msg.position = robot_msgs::msg::ArmPosition::OPEN;
      arm_position_mode_pub_->publish(arm_msg);
      RCLCPP_INFO(get_logger(), "Manual arm position -> OPEN");
    } else if (is_axis_just_triggered(joy_msg_, dpad_horizontal_axis_, true)) { // DPAD 左 (+1.0)
      robot_msgs::msg::ArmPosition arm_msg;
      arm_msg.position = robot_msgs::msg::ArmPosition::DRIBBLE;
      arm_position_mode_pub_->publish(arm_msg);
      RCLCPP_INFO(get_logger(), "Manual arm position -> DRIBBLE");
    }
  } else {
    // R2非押下時: DPAD 上/下でベルトレベル昇降
    if (is_axis_just_triggered(joy_msg_, dpad_vertical_axis_, true)) {
      belt_rpm_mode_ = increment_mode(belt_rpm_mode_, robot_msgs::msg::BeltMode::LEVEL_4);
      RCLCPP_INFO(get_logger(), "Belt level changed to: %u", belt_rpm_mode_);
      publish_belt_mode(belt_rpm_mode_);
    } else if (is_axis_just_triggered(joy_msg_, dpad_vertical_axis_, false)) {
      belt_rpm_mode_ = decrement_mode(belt_rpm_mode_);
      RCLCPP_INFO(get_logger(), "Belt level changed to: %u", belt_rpm_mode_);
      publish_belt_mode(belt_rpm_mode_);
    }
  }

  // 3. ドリブル回転ON/OFF (R1)
  if (is_button_just_pressed(joy_msg_, dribble_enable_button_)) {
    dribble_enabled_ = !dribble_enabled_;
    if (dribble_enabled_) {
      dribble_reversed_ = false;
      publish_dribble_reverse(false);
    }
    RCLCPP_INFO(get_logger(), "Dribble toggled: %s", dribble_enabled_ ? "ON" : "OFF");
    publish_dribble_enabled(dribble_enabled_);
  }

  // 3b. ドリブル逆回転ON/OFF (未割り当てでない場合)
  if (dribble_reverse_button_ >= 0 && is_button_just_pressed(joy_msg_, dribble_reverse_button_)) {
    dribble_reversed_ = !dribble_reversed_;
    if (dribble_reversed_) {
      dribble_enabled_ = false;
      publish_dribble_enabled(false);
    }
    RCLCPP_INFO(get_logger(), "Dribble reverse toggled: %s", dribble_reversed_ ? "ON" : "OFF");
    publish_dribble_reverse(dribble_reversed_);
  }

  // 4. 前後反転 (PS)
  if (is_button_just_pressed(joy_msg_, ps_button_)) {
    is_drive_reversed_ = !is_drive_reversed_;
    publish_drive_reversed(is_drive_reversed_);
    RCLCPP_INFO(
      get_logger(), "Drive direction toggled: %s", is_drive_reversed_ ? "REVERSED" : "FORWARD");
  }

  // 4b. Heading Hold ON/OFF 切替 (SHARE / BACK)
  if (is_button_just_pressed(joy_msg_, heading_hold_toggle_button_)) {
    is_heading_hold_enabled_ = !is_heading_hold_enabled_;
    std_msgs::msg::Bool hh_msg;
    hh_msg.data = is_heading_hold_enabled_;
    heading_control_enable_pub_->publish(hh_msg);
    if (is_heading_hold_enabled_) {
      RCLCPP_INFO(
        get_logger(), "=== [JoyController] Heading Hold: ENABLED (IMU Stabilization ON) ===");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "=== [JoyController] Heading Hold: DISABLED (Raw Manual Passthrough Mode) ===");
    }
  }

  // 5. 自動シュートサイクル要求 (L2 + ○)
  if (is_l2_active && is_button_just_pressed(joy_msg_, circle_button_)) {
    RCLCPP_INFO(get_logger(), "Shot cycle requested!");
    std_msgs::msg::Bool req;
    req.data = true;
    shot_cycle_request_pub_->publish(req);
  }

  // 6. Game 2 自動戦術モード切替 (OPTIONS)
  if (is_button_just_pressed(joy_msg_, game2_start_button_)) {
    game2_active_ = !game2_active_;
    RCLCPP_INFO(get_logger(), "Game 2 mode toggled: %s", game2_active_ ? "START" : "STOP");
    std_msgs::msg::Bool game2_msg;
    game2_msg.data = game2_active_;
    game2_start_pub_->publish(game2_msg);
  }

  // 7. 手動オーバーライド: Game 2 モード中にスティック操作を検出したら自動解除
  if (game2_active_) {
    const double raw_vx = apply_axis_deadzone(get_axis_value(joy_msg_, left_stick_y_axis_));
    const double raw_vy = apply_axis_deadzone(get_axis_value(joy_msg_, left_stick_x_axis_));
    const double raw_wz = apply_axis_deadzone(-get_axis_value(joy_msg_, right_stick_x_axis_));

    if (raw_vx != 0.0 || raw_vy != 0.0 || raw_wz != 0.0) {
      game2_active_ = false;
      RCLCPP_WARN(get_logger(), "Manual stick input detected! Game 2 AUTO mode disengaged.");
      std_msgs::msg::Bool game2_msg;
      game2_msg.data = false;
      game2_start_pub_->publish(game2_msg);
    }
  }


  // 8. L2とR2を同時に押した瞬間にスプリング発射シーケンスを開始
  const bool was_l2_active = last_joy_msg_.has_value() && get_axis_value(
    last_joy_msg_.value(), left_trigger_axis_) <= -axis_on_threshold_;
  const bool was_r2_active = last_joy_msg_.has_value() && get_axis_value(
    last_joy_msg_.value(), right_trigger_axis_) <= -axis_on_threshold_;
  const bool spring_fire_input_triggered = is_l2_active && is_r2_active &&
    !(was_l2_active && was_r2_active);
  const bool is_ready_rising = spring_actuator_ready_ && !was_spring_ready_;

  bool should_publish_spring_fire = false;

  if (spring_fire_input_triggered && !spring_fire_pending_) {
    // 1) ドリブル減速通知 (300 RPM 案内回転へ滑らか減速)
    publish_spring_decel(true);
    RCLCPP_INFO(
      get_logger(),
      "Spring fire sequence started: smoothly decelerating dribble roller to 300 RPM...");

    // 2) 減速完了待機モードにセット
    spring_fire_pending_ = true;
    spring_fire_pending_start_time_ = std::chrono::steady_clock::now();
  }

  // ドリブルの滑らか減速時間 (150ms) が経過したら実際にキッカーばねを解放 (FIRE!)
  if (spring_fire_pending_) {
    const auto now_tp = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now_tp - spring_fire_pending_start_time_).count();
    if (elapsed_ms >= spring_fire_decel_delay_ms_) {
      should_publish_spring_fire = true;
      spring_fire_pending_ = false;
      RCLCPP_INFO(get_logger(), "Dribble decel complete -> Spring FIRE released!");
    }
  }

  std_msgs::msg::Bool spring_fire_msg;
  spring_fire_msg.data = should_publish_spring_fire;
  spring_fire_pub_->publish(spring_fire_msg);

  // ばねの再充填・準備完了(is_ready_rising)で減速モードを解除
  if (is_ready_rising || should_publish_spring_fire) {
    publish_spring_decel(false);
  }

  was_spring_ready_ = spring_actuator_ready_;

  // 9. DPAD 左右で自動シュート OPEN 動作時のドリブル回転数を変更 (+200 / -200 RPM, R2非押下時)
  if (!is_r2_active) {
    if (is_axis_just_triggered(joy_msg_, dpad_horizontal_axis_, true)) {
      shot_cycle_opening_rpm_ = std::min(2500, shot_cycle_opening_rpm_ + 200);
      publish_opening_rpm(shot_cycle_opening_rpm_);
      RCLCPP_INFO(get_logger(), "Shot cycle opening RPM set to: %d RPM", shot_cycle_opening_rpm_);
    } else if (is_axis_just_triggered(joy_msg_, dpad_horizontal_axis_, false)) {
      shot_cycle_opening_rpm_ = std::max(0, shot_cycle_opening_rpm_ - 200);
      publish_opening_rpm(shot_cycle_opening_rpm_);
      RCLCPP_INFO(get_logger(), "Shot cycle opening RPM set to: %d RPM", shot_cycle_opening_rpm_);
    }
  }

  // 10. アナログスティック走行コマンド算出 (Game 2 非アクティブ時)
  if (!game2_active_) {
    double raw_vx = get_axis_value(joy_msg_, left_stick_y_axis_);
    double raw_vy = get_axis_value(joy_msg_, left_stick_x_axis_);
    const double raw_wz = -get_axis_value(joy_msg_, right_stick_x_axis_);

    if (is_drive_reversed_) {
      raw_vx = -raw_vx;
      raw_vy = -raw_vy;
    }

    bool is_slow_turn_active = false;
    if (slow_turn_button_ >= 0) {
      if (is_button_down(joy_msg_, slow_turn_button_)) {
        is_slow_turn_active = true;
      } else if (slow_turn_button_ == 7 && is_r2_active) {
        is_slow_turn_active = true;
      } else if (slow_turn_button_ == 6 && is_l2_active) {
        is_slow_turn_active = true;
      }
    }

    double target_vx = apply_axis_deadzone(raw_vx) * max_vel_x_m_s_;
    double target_vy = apply_axis_deadzone(raw_vy) * max_vel_y_m_s_;
    double target_yaw = apply_axis_deadzone(raw_wz) * max_vel_z_rad_s_;
    if (is_slow_turn_active) {
      target_vx *= slow_linear_scale_;
      target_vy *= slow_linear_scale_;
      target_yaw *= slow_turn_scale_;
    }

    publish_limited_velocity(target_vx, target_vy, target_yaw);
  }

  last_joy_msg_ = joy_msg_;
}

void JoyControllerNode::spring_actuator_ready_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  spring_actuator_ready_ = msg->data;
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
  publish_emergency_stop(is_emergency_stop_);
  publish_belt_mode(belt_rpm_mode_);
  publish_dribble_enabled(dribble_enabled_);
  publish_dribble_reverse(dribble_reversed_);
  publish_drive_reversed(is_drive_reversed_);
}

void JoyControllerNode::publish_emergency_stop(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  emergency_stop_pub_->publish(msg);
}

void JoyControllerNode::publish_belt_mode(uint8_t mode)
{
  robot_msgs::msg::BeltMode msg;
  msg.mode = mode;
  belt_mode_pub_->publish(msg);
}

void JoyControllerNode::publish_dribble_enabled(bool enabled)
{
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  dribble_enabled_pub_->publish(msg);
}

void JoyControllerNode::publish_dribble_reverse(bool reversed)
{
  std_msgs::msg::Bool msg;
  msg.data = reversed;
  dribble_reverse_pub_->publish(msg);
}

void JoyControllerNode::publish_spring_decel(bool active)
{
  std_msgs::msg::Bool msg;
  msg.data = active;
  spring_decel_pub_->publish(msg);
}

void JoyControllerNode::publish_opening_rpm(int rpm)
{
  std_msgs::msg::Int32 msg;
  msg.data = rpm;
  opening_rpm_pub_->publish(msg);
}

void JoyControllerNode::publish_drive_reversed(bool reversed)
{
  std_msgs::msg::Bool msg;
  msg.data = reversed;
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

  belt_rpm_mode_ = robot_msgs::msg::BeltMode::STOP;
  publish_belt_mode(belt_rpm_mode_);

  dribble_enabled_ = false;
  publish_dribble_enabled(dribble_enabled_);

  dribble_reversed_ = false;
  publish_dribble_reverse(dribble_reversed_);
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
