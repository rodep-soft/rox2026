#include "mecanum_controller/mecanum_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

MecanumControllerNode::MecanumControllerNode()
: Node("mecanum_controller_node")
{
  get_parameters();

  create_interfaces();
}

void MecanumControllerNode::get_parameters()
{
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);
  robot_length_ = declare_parameter<double>("robot_length", 0.47);
  robot_width_ = declare_parameter<double>("robot_width", 0.41);
  max_wheel_velocity_rad_s_ = declare_parameter<double>("max_wheel_velocity_rad_s", 50.0);
  velocity_corrections_ = declare_parameter<std::vector<double>>("velocity_corrections", {1.0, 1.0, 1.0, 1.0});
  vx_sign_ = declare_parameter<double>("vx_sign", 1.0);
  vy_sign_ = declare_parameter<double>("vy_sign", 1.0);
  angular_z_sign_ = declare_parameter<double>("angular_z_sign", 1.0);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 20);
  qos_depth_ = declare_parameter<int>("qos_depth", 1);

  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel");
  emergency_stop_topic_ = declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  target_array_topic_ = declare_parameter<std::string>("target_array_topic", "/edulite/target_array");
  const auto wheel_logical_ids = declare_parameter<std::vector<int64_t>>("wheel_logical_ids", {0, 1, 2, 3});


  if (wheel_logical_ids.size() != wheel_logical_ids_.size()) {
    throw std::runtime_error("wheel_logical_ids must contain four elements");
  }
  for (std::size_t index = 0; index < wheel_logical_ids.size(); ++index) {
    if (wheel_logical_ids[index] < 0 || wheel_logical_ids[index] > 65535) {
      throw std::runtime_error("wheel_logical_ids values must be in [0, 65535]");
    }
    const auto logical_id = static_cast<uint16_t>(wheel_logical_ids[index]);
    if (std::find(wheel_logical_ids_.cbegin(), wheel_logical_ids_.cbegin() + index, logical_id) !=
      wheel_logical_ids_.cbegin() + index)
    {
      throw std::runtime_error("wheel_logical_ids values must be unique");
    }
    wheel_logical_ids_[index] = logical_id;
  }
  if (cmd_vel_topic_.empty() || emergency_stop_topic_.empty() || target_array_topic_.empty()) {
    throw std::runtime_error("topic parameters must not be empty");
  }

  if (command_period_ms_ <= 0) {
    command_period_ms_ = 20;
  }

  constexpr std::size_t kNumWheels = 4;
  if (velocity_corrections_.size() != kNumWheels) {
    RCLCPP_ERROR(
      get_logger(),
      "velocity_corrections must contain %zu elements, but %zu were provided",
      kNumWheels, velocity_corrections_.size());
    velocity_corrections_.assign(kNumWheels, 1.0);
  }
  for (std::size_t index = 0; index < velocity_corrections_.size(); ++index) {
    if (!std::isfinite(velocity_corrections_[index])) {
      RCLCPP_WARN(
        get_logger(),
        "velocity_corrections[%zu] must be finite: %.6f. Using 1.0.",
        index, velocity_corrections_[index]);
      velocity_corrections_[index] = 1.0;
    }
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }
  if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "wheel_radius must be finite and greater than zero: %.6f. "
      "Using 0.05 m.",
      wheel_radius_);
    wheel_radius_ = 0.05;
  }
  if (!std::isfinite(robot_length_) || robot_length_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "robot_length must be finite and zero or greater: %.6f. Using 0.47 m.",
      robot_length_);
    robot_length_ = 0.47;
  }
  if (!std::isfinite(robot_width_) || robot_width_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "robot_width must be finite and zero or greater: %.6f. Using 0.41 m.",
      robot_width_);
    robot_width_ = 0.41;
  }
  constexpr double edulite_velocity_limit_rad_s = 50.0;
  if (!std::isfinite(max_wheel_velocity_rad_s_) ||
    max_wheel_velocity_rad_s_ <= 0.0 ||
    max_wheel_velocity_rad_s_ > edulite_velocity_limit_rad_s)
  {
    RCLCPP_WARN(
      get_logger(),
      "max_wheel_velocity_rad_s must be in (0.0, 50.0]: %.6f. "
      "Using 50.0 rad/s.",
      max_wheel_velocity_rad_s_);
    max_wheel_velocity_rad_s_ = edulite_velocity_limit_rad_s;
  }
  const auto validate_sign = [this](const char * name, double & value)
    {
      if (std::isfinite(value) && value != 0.0) {
        return;
      }
      RCLCPP_WARN(
        get_logger(), "%s must be finite and nonzero: %.6f. Using 1.0.",
        name, value);
      value = 1.0;
    };
  validate_sign("vx_sign", vx_sign_);
  validate_sign("vy_sign", vy_sign_);
  validate_sign("angular_z_sign", angular_z_sign_);
}

void MecanumControllerNode::create_interfaces()
{
  // /mecanum/cmd_vel: joy_controllerから受ける機体座標系の走行速度指令。
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &MecanumControllerNode::cmd_vel_callback, this,
      std::placeholders::_1));
      
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic_, state_qos,
    std::bind(
      &MecanumControllerNode::emergency_stop_callback, this,
      std::placeholders::_1));
  // 各/vel_command: hardware_driverへ送る4輪それぞれの目標角速度 [rad/s]。
  target_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorTargetArray>(
    target_array_topic_, rclcpp::QoS(qos_depth_));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&MecanumControllerNode::publish_wheel_commands, this));
}

void MecanumControllerNode::cmd_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
    !std::isfinite(msg->angular.z))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Non-finite cmd_vel received. Publishing zero wheel velocity.");
    last_cmd_vel_ = geometry_msgs::msg::Twist{};
    publish_wheel_commands();
    return;
  }
  last_cmd_vel_ = *msg;
  publish_wheel_commands();
}

void MecanumControllerNode::emergency_stop_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  publish_wheel_commands();
}

void MecanumControllerNode::publish_wheel_commands()
{
  double vx = last_cmd_vel_.linear.x * vx_sign_;
  double vy = last_cmd_vel_.linear.y * vy_sign_;
  double wz = last_cmd_vel_.angular.z * angular_z_sign_;

  if (emergency_stop_active_) {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  const double half_lw = (robot_length_ + robot_width_) / 2.0;
  std::array<double, 4> wheel_vels;
  wheel_vels[FL] = -(vx + vy - half_lw * wz) / wheel_radius_;
  wheel_vels[FR] = (vx - vy + half_lw * wz) / wheel_radius_;
  wheel_vels[RL] = -(vx - vy - half_lw * wz) / wheel_radius_;
  wheel_vels[RR] = (vx + vy + half_lw * wz) / wheel_radius_;

  double maximum_wheel_velocity = 0.0;
  std::array<double, 4> corrected_wheel_vels;
  for (std::size_t index = 0; index < wheel_vels.size(); ++index) {
    corrected_wheel_vels[index] = wheel_vels[index] * velocity_corrections_[index];
    maximum_wheel_velocity =
      std::max(maximum_wheel_velocity, std::abs(corrected_wheel_vels[index]));
  }

  const double velocity_scale = (maximum_wheel_velocity > max_wheel_velocity_rad_s_) ?
    (max_wheel_velocity_rad_s_ / maximum_wheel_velocity) : 1.0;

  actuator_msgs::msg::ActuatorTargetArray command_message;
  command_message.header.stamp = now();
  command_message.actuators.reserve(wheel_vels.size());
  for (std::size_t index = 0; index < wheel_vels.size(); ++index) {
    actuator_msgs::msg::ActuatorTarget target;
    target.logical_id = wheel_logical_ids_[index];
    target.target = static_cast<float>(corrected_wheel_vels[index] * velocity_scale);
    command_message.actuators.push_back(target);
  }
  target_array_pub_->publish(command_message);
}
