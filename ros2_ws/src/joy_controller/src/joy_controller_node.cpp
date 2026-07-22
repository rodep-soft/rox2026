#include "joy_controller/joy_controller_node.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

// コンストラクタ
JoyControllerNode::JoyControllerNode()
: Node("joy_controller"),
  pre_intake_chord_on_(false),
  pre_spring_fire_chord_on_(false),
  pre_belt_fire_chord_on_(false),
  pre_belt_mode_up_chord_on_(false),
  pre_belt_mode_down_chord_on_(false),
  pre_dribble_mode_up_chord_on_(false),
  pre_dribble_mode_down_chord_on_(false),
  pre_emergency_stop_chord_on_(false),
  pre_dribble_position_dribble_chord_on_(false),
  pre_dribble_position_shoot_chord_on_(false)
{
  declare_parameters();
  get_parameters();

  // qosが有効値かどうか
  if (joy_qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "joy_qos_depth must be positive. Using the default value of 1.");
    joy_qos_depth_ = 1;
  }
  if (command_qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "command_qos_depth must be positive. Using the default value of 1.");
    command_qos_depth_ = 1;
  }
  if (joy_timeout_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "joy_timeout_ms must be positive. Using the default value of 200 ms.");
    joy_timeout_ms_ = 200;
  }

  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::QoS(rclcpp::KeepLast(joy_qos_depth_)).best_effort(),
    std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));

  mecanum_cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    mecanum_cmd_vel_topic_, rclcpp::QoS(
      command_qos_depth_));

  spring_fire_publisher_ = create_publisher<std_msgs::msg::Bool>(
    spring_fire_request_topic_, rclcpp::QoS(command_qos_depth_));

  belt_fire_publisher_ =
    create_publisher<std_msgs::msg::Bool>(belt_fire_topic_, rclcpp::QoS(command_qos_depth_));

  belt_mode_publisher_ =
    create_publisher<std_msgs::msg::UInt8>(belt_mode_topic_, rclcpp::QoS(command_qos_depth_));

  dribble_mode_publisher_ =
    create_publisher<std_msgs::msg::UInt8>(dribble_mode_topic_, rclcpp::QoS(command_qos_depth_));

  emergency_stop_client_ = create_client<std_srvs::srv::Trigger>(emergency_stop_service_);

  dribble_position_action_client_ =
    rclcpp_action::create_client<robot_controller::action::DribblePosition>(
    this,
    dribble_position_action_);

  // 起動直後は操作入力が来るまで、各機構へ停止指令を出しておく。
  publish_stop_commands();
  joy_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&JoyControllerNode::joy_timeout_callback, this));

  RCLCPP_INFO(
    get_logger(), "Subscribing to %s and publishing mechanism commands", joy_topic_.c_str());
}

void JoyControllerNode::declare_parameters()
{
  // Topic名の宣言
  declare_parameter<std::string>("joy_topic", "/joy");
  declare_parameter<std::string>("mecanum_cmd_vel_topic", "/mecanum/cmd_vel");
  declare_parameter<std::string>("spring_fire_request_topic", "/spring/fire_request");
  declare_parameter<std::string>("belt_fire_topic", "/belt/fire_enabled");
  declare_parameter<std::string>("belt_mode_topic", "/belt/mode");
  declare_parameter<std::string>("dribble_mode_topic", "/dribble/mode");
  declare_parameter<std::string>("emergency_stop_service", "/emergency_stop");
  declare_parameter<std::string>("dribble_position_action", "/dribble/position");

  // Qos設定
  declare_parameter<int>("joy_qos_depth", 1);
  declare_parameter<int>("command_qos_depth", 1);
  declare_parameter<int>("joy_timeout_ms", 200);

  // Mecanum制御のスケールと制限値
  declare_parameter<double>("linear_x_scale", 1.0);
  declare_parameter<double>("linear_y_scale", 1.0);
  declare_parameter<double>("angular_z_scale", 1.0);
  declare_parameter<double>("max_linear_x", 2.0);
  declare_parameter<double>("min_linear_x", -2.0);
  declare_parameter<double>("max_linear_y", 2.0);
  declare_parameter<double>("min_linear_y", -2.0);
  declare_parameter<double>("max_angular_z", 2.0);
  declare_parameter<double>("min_angular_z", -2.0);

  // ジョイスティックのデッドゾーンと有効閾値
  declare_parameter<double>("axis_deadzone", 0.05);
  declare_parameter<double>("axis_on_threshold", 0.7);

  // joyのボタン
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
  declare_parameter<int>("dribble_position_is_enable_button", 4);
  declare_parameter<int>("dribble_position_dribble_button", 1);
  declare_parameter<int>("dribble_position_shoot_button", 2);
  declare_parameter<int>("left_stick_x_axis", 0);
  declare_parameter<int>("left_stick_y_axis", 1);
  declare_parameter<int>("right_stick_x_axis", 2);
  declare_parameter<int>("mode_change_axis", 7);
}

void JoyControllerNode::get_parameters()
{
  get_parameter("joy_topic", joy_topic_);
  get_parameter("mecanum_cmd_vel_topic", mecanum_cmd_vel_topic_);
  get_parameter("spring_fire_request_topic", spring_fire_request_topic_);
  get_parameter("belt_fire_topic", belt_fire_topic_);
  get_parameter("belt_mode_topic", belt_mode_topic_);
  get_parameter("dribble_mode_topic", dribble_mode_topic_);
  get_parameter("emergency_stop_service", emergency_stop_service_);
  get_parameter("dribble_position_action", dribble_position_action_);
  get_parameter("joy_qos_depth", joy_qos_depth_);
  get_parameter("command_qos_depth", command_qos_depth_);
  get_parameter("joy_timeout_ms", joy_timeout_ms_);
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
  get_parameter("dribble_position_is_enable_button", dribble_position_is_enable_button_);
  get_parameter("dribble_position_dribble_button", dribble_position_dribble_button_);
  get_parameter("dribble_position_shoot_button", dribble_position_shoot_button_);
  get_parameter("left_stick_x_axis", left_stick_x_axis_);
  get_parameter("left_stick_y_axis", left_stick_y_axis_);
  get_parameter("right_stick_x_axis", right_stick_x_axis_);
  get_parameter("mode_change_axis", mode_change_axis_);
}

bool JoyControllerNode::button_pressed(const sensor_msgs::msg::Joy & msg, int index)
{
  // 設定されたボタン番号が範囲外なら、未押下として扱う。
  return index >= 0 && static_cast<std::size_t>(index) < msg.buttons.size() &&
         msg.buttons[static_cast<std::size_t>(index)] != 0;
}

double JoyControllerNode::axis_value(const sensor_msgs::msg::Joy & msg, int index)
{
  // 設定された軸番号が範囲外なら、入力なしとして0.0を返す。
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
  return mode > 0 ? static_cast<uint8_t>(mode - 1) : 0;
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

void JoyControllerNode::send_dribble_position_goal(uint8_t command)
{
  if (!dribble_position_action_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN(get_logger(), "Dribble position action server is not available");
    return;
  }
  robot_controller::action::DribblePosition::Goal goal;
  goal.command = command;
  dribble_position_action_client_->async_send_goal(goal);
}

void JoyControllerNode::publish_stop_commands()
{
  cmd_vel_ = geometry_msgs::msg::Twist{};
  intake_enabled_ = false;
  spring_fire_enabled_ = false;
  belt_fire_enabled_ = false;
  belt_rpm_mode_ = static_cast<uint8_t>(BeltRpmMode::STOP);
  dribble_rpm_mode_ = static_cast<uint8_t>(DribbleRpmMode::STOP);

  mecanum_cmd_vel_publisher_->publish(cmd_vel_);
  std_msgs::msg::Bool spring;
  spring.data = spring_fire_enabled_;
  spring_fire_publisher_->publish(spring);
  std_msgs::msg::Bool belt;
  belt.data = belt_fire_enabled_;
  belt_fire_publisher_->publish(belt);
  std_msgs::msg::UInt8 belt_mode_msg;
  belt_mode_msg.data = belt_rpm_mode_;
  belt_mode_publisher_->publish(belt_mode_msg);
  std_msgs::msg::UInt8 dribble_mode_msg;
  dribble_mode_msg.data = dribble_rpm_mode_;
  dribble_mode_publisher_->publish(dribble_mode_msg);
}

void JoyControllerNode::joy_timeout_callback()
{
  if (!joy_received_) {
    return;
  }

  const auto elapsed = std::chrono::steady_clock::now() - last_joy_received_time_;
  if (elapsed > std::chrono::milliseconds(joy_timeout_ms_)) {
    if (!joy_timeout_active_) {
      // 非常停止操作ではなくJoy通信断による停止だと切り分けるため、最初の1回だけ記録する。
      RCLCPP_ERROR(get_logger(), "Joy input timed out. Sending stop commands.");
      joy_timeout_active_ = true;
    }
    // Joy入力が途絶えた場合は、最後の操作指令を残さず停止状態を維持する。
    publish_stop_commands();
  }
}

void JoyControllerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (joy_timeout_active_) {
    // 停止原因となったJoy通信断が解消したことを、実機確認時に追跡できるよう記録する。
    RCLCPP_INFO(get_logger(), "Joy input recovered.");
    joy_timeout_active_ = false;
  }
  joy_received_ = true;
  last_joy_received_time_ = std::chrono::steady_clock::now();
  const auto & joy_msg = *msg;
  const bool intake_is_enable_button = button_pressed(joy_msg, intake_is_enable_button_);
  const bool intake_button_on = button_pressed(joy_msg, intake_button_on_);
  const bool spring_fire_is_enable_button = button_pressed(joy_msg, spring_fire_is_enable_button_);
  const bool spring_fire_button_on = button_pressed(joy_msg, spring_fire_button_on_);
  const bool belt_fire_is_enable_button = button_pressed(joy_msg, belt_fire_is_enable_button_);
  const bool belt_fire_button_on = button_pressed(joy_msg, belt_fire_button_on_);
  const bool belt_mode_is_enable_button = button_pressed(joy_msg, belt_mode_is_enable_button_);
  const bool dribble_mode_is_enable_button =
    button_pressed(joy_msg, dribble_mode_is_enable_button_);
  const bool emergency_stop_is_enable_button =
    button_pressed(joy_msg, emergency_stop_is_enable_button_);
  const bool emergency_stop_button_on = button_pressed(joy_msg, emergency_stop_button_on_);
  const bool dribble_position_is_enable_button =
    button_pressed(joy_msg, dribble_position_is_enable_button_);
  const bool dribble_position_dribble_button_on =
    button_pressed(joy_msg, dribble_position_dribble_button_);
  const bool dribble_position_shoot_button_on =
    button_pressed(joy_msg, dribble_position_shoot_button_);
  const double mode_change_value = axis_value(joy_msg, mode_change_axis_);
  const bool is_mode_up = mode_change_value >= axis_on_threshold_;
  const bool is_mode_down = mode_change_value <= -axis_on_threshold_;

  const bool intake_chord_on = intake_is_enable_button && intake_button_on;
  const bool spring_fire_chord_on = spring_fire_is_enable_button && spring_fire_button_on;
  const bool belt_fire_chord_on = belt_fire_is_enable_button && belt_fire_button_on;
  const bool belt_mode_up_chord_on = belt_mode_is_enable_button && is_mode_up;
  const bool belt_mode_down_chord_on = belt_mode_is_enable_button && is_mode_down;
  const bool dribble_mode_up_chord_on = dribble_mode_is_enable_button && is_mode_up;
  const bool dribble_mode_down_chord_on = dribble_mode_is_enable_button && is_mode_down;
  const bool emergency_stop_chord_on =
    emergency_stop_is_enable_button && emergency_stop_button_on;
  const bool dribble_position_dribble_chord_on =
    dribble_position_is_enable_button && dribble_position_dribble_button_on;
  const bool dribble_position_shoot_chord_on =
    dribble_position_is_enable_button && dribble_position_shoot_button_on;

  const auto update_previous_chord_states = [&]() {
      pre_intake_chord_on_ = intake_chord_on;
      pre_spring_fire_chord_on_ = spring_fire_chord_on;
      pre_belt_fire_chord_on_ = belt_fire_chord_on;
      pre_belt_mode_up_chord_on_ = belt_mode_up_chord_on;
      pre_belt_mode_down_chord_on_ = belt_mode_down_chord_on;
      pre_dribble_mode_up_chord_on_ = dribble_mode_up_chord_on;
      pre_dribble_mode_down_chord_on_ = dribble_mode_down_chord_on;
      pre_emergency_stop_chord_on_ = emergency_stop_chord_on;
      pre_dribble_position_dribble_chord_on_ = dribble_position_dribble_chord_on;
      pre_dribble_position_shoot_chord_on_ = dribble_position_shoot_chord_on;
    };

  if (emergency_stop_chord_on && !pre_emergency_stop_chord_on_) {
    emergency_stop_latched_ = !emergency_stop_latched_;
    if (emergency_stop_latched_) {
      RCLCPP_WARN(get_logger(), "Emergency stop mode enabled");
      call_emergency_stop();
      // ドリブルする位置にDribble Positionを移動
      send_dribble_position_goal(robot_controller::action::DribblePosition::Goal::DRIBBLE);
    } else {
      RCLCPP_WARN(get_logger(), "Emergency stop mode disabled");
    }
    publish_stop_commands();
    update_previous_chord_states();
    return;
  }

  if (emergency_stop_latched_) {
    // 非常停止モード中であることをログでも追跡できるようにしつつ、停止指令を維持する。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Emergency stop mode is active. Sending stop commands.");
    publish_stop_commands();
    update_previous_chord_states();
    return;
  }

  // ボタンの押下開始を検出し、各機構の有効状態や動作モードを更新する。
  if (intake_chord_on && !pre_intake_chord_on_) {
    intake_enabled_ = !intake_enabled_;
  }
  if (spring_fire_chord_on && !pre_spring_fire_chord_on_) {
    spring_fire_enabled_ = !spring_fire_enabled_;
  }
  if (belt_fire_chord_on && !pre_belt_fire_chord_on_) {
    belt_fire_enabled_ = !belt_fire_enabled_;
  }
  if (belt_mode_up_chord_on && !pre_belt_mode_up_chord_on_) {
    belt_rpm_mode_ = increment_mode(belt_rpm_mode_, static_cast<uint8_t>(BeltRpmMode::LEVEL_3));
  }
  if (belt_mode_down_chord_on && !pre_belt_mode_down_chord_on_) {
    belt_rpm_mode_ = decrement_mode(belt_rpm_mode_);
  }
  if (dribble_mode_up_chord_on && !pre_dribble_mode_up_chord_on_) {
    dribble_rpm_mode_ =
      increment_mode(dribble_rpm_mode_, static_cast<uint8_t>(DribbleRpmMode::LOW));
  }
  if (dribble_mode_down_chord_on && !pre_dribble_mode_down_chord_on_) {
    dribble_rpm_mode_ = decrement_mode(dribble_rpm_mode_);
  }
  if (dribble_position_dribble_chord_on && !pre_dribble_position_dribble_chord_on_) {
    send_dribble_position_goal(robot_controller::action::DribblePosition::Goal::DRIBBLE);
  }
  if (dribble_position_shoot_chord_on && !pre_dribble_position_shoot_chord_on_) {
    send_dribble_position_goal(robot_controller::action::DribblePosition::Goal::SHOOT);
  }

  const double linear_x = apply_axis_deadzone(axis_value(joy_msg, left_stick_y_axis_));
  const double linear_y = apply_axis_deadzone(axis_value(joy_msg, left_stick_x_axis_));
  const double angular_z = apply_axis_deadzone(axis_value(joy_msg, right_stick_x_axis_));

  // スティック入力に制限値とスケールをかけて、mecanum_controller向けの速度指令にする。
  cmd_vel_.linear.x =
    apply_axis_limits(linear_x, min_linear_x_, max_linear_x_) * linear_x_scale_;
  cmd_vel_.linear.y =
    apply_axis_limits(linear_y, min_linear_y_, max_linear_y_) * linear_y_scale_;
  cmd_vel_.angular.z =
    apply_axis_limits(angular_z, min_angular_z_, max_angular_z_) * angular_z_scale_;

  mecanum_cmd_vel_publisher_->publish(cmd_vel_);
  std_msgs::msg::Bool spring; spring.data = spring_fire_enabled_;
  spring_fire_publisher_->publish(spring);
  std_msgs::msg::Bool belt; belt.data = belt_fire_enabled_; belt_fire_publisher_->publish(belt);
  std_msgs::msg::UInt8 belt_mode_msg; belt_mode_msg.data = belt_rpm_mode_;
  belt_mode_publisher_->publish(belt_mode_msg);
  std_msgs::msg::UInt8 dribble_mode_msg; dribble_mode_msg.data = dribble_rpm_mode_;
  dribble_mode_publisher_->publish(dribble_mode_msg);

  update_previous_chord_states();
}
