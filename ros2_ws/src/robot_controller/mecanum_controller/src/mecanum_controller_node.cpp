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
  configure_parameters();

  const auto cmd_vel_topic =
    declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel");
  const auto emergency_stop_topic =
    declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  const auto target_array_topic =
    declare_parameter<std::string>("target_array_topic", "/edulite/target_array");
  auto emergency_stop_period_ms = declare_parameter<int>("emergency_stop_period_ms", 20);
  auto qos_depth = declare_parameter<int>("qos_depth", 1);

  if (cmd_vel_topic.empty() || emergency_stop_topic.empty() || target_array_topic.empty()) {
    throw std::runtime_error("topic parameters must not be empty");
  }
  if (emergency_stop_period_ms <= 0) {
    RCLCPP_WARN(get_logger(), "emergency_stop_period_ms must be positive. Using 20 ms.");
    emergency_stop_period_ms = 20;
  }
  if (qos_depth <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using 1.");
    qos_depth = 1;
  }

  // /mecanum/cmd_vel: joy_controllerから機体座標系の速度指令を受け取る。
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(qos_depth),
    std::bind(&MecanumControllerNode::cmd_vel_callback, this, std::placeholders::_1));

  // /emergency_stop: joy_controllerから非常停止状態を受け取る。
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic, emergency_stop_qos,
    std::bind(&MecanumControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  // /edulite/target_array: hardware_driverへ4輪の目標角速度[rad/s]を送る。
  target_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorTargetArray>(
    target_array_topic, rclcpp::QoS(qos_depth));

  // 非常停止中だけ全輪ゼロを周期再送し、単発メッセージの欠落や遅延に備える。
  emergency_stop_timer_ = create_wall_timer(
    std::chrono::milliseconds(emergency_stop_period_ms), [this]() {
      if (emergency_stop_active_) {
        publish_wheel_commands();
      }
    });
}

void MecanumControllerNode::configure_parameters()
{
  wheel_radius_m_ = declare_parameter<double>("wheel_radius", 0.075);
  robot_length_m_ = declare_parameter<double>("robot_length", 0.47);
  robot_width_m_ = declare_parameter<double>("robot_width", 0.41);
  max_wheel_velocity_rad_s_ = declare_parameter<double>("max_wheel_velocity_rad_s", 50.0);
  const auto wheel_logical_ids =
    declare_parameter<std::vector<int64_t>>("wheel_logical_ids", {0, 1, 2, 3});

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
  if (!std::isfinite(wheel_radius_m_) || wheel_radius_m_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "wheel_radius must be finite and greater than zero: %.6f. "
      "Using 0.075 m.",
      wheel_radius_m_);
    wheel_radius_m_ = 0.075;
  }
  if (!std::isfinite(robot_length_m_) || robot_length_m_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "robot_length must be finite and zero or greater: %.6f. Using 0.47 m.",
      robot_length_m_);
    robot_length_m_ = 0.47;
  }
  if (!std::isfinite(robot_width_m_) || robot_width_m_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "robot_width must be finite and zero or greater: %.6f. Using 0.41 m.",
      robot_width_m_);
    robot_width_m_ = 0.41;
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
  double velocity_x_m_s = last_cmd_vel_.linear.x;
  double velocity_y_m_s = last_cmd_vel_.linear.y;
  double angular_velocity_rad_s = last_cmd_vel_.angular.z;

  if (emergency_stop_active_) {
    velocity_x_m_s = 0.0;
    velocity_y_m_s = 0.0;
    angular_velocity_rad_s = 0.0;
  }

  const double rotation_radius_m = (robot_length_m_ + robot_width_m_) / 2.0;
  std::array<double, 4> wheel_velocities_rad_s;
  wheel_velocities_rad_s[FRONT_LEFT] =
    -(velocity_x_m_s + velocity_y_m_s - rotation_radius_m * angular_velocity_rad_s) /
    wheel_radius_m_;
  wheel_velocities_rad_s[FRONT_RIGHT] =
    (velocity_x_m_s - velocity_y_m_s + rotation_radius_m * angular_velocity_rad_s) /
    wheel_radius_m_;
  wheel_velocities_rad_s[REAR_LEFT] =
    -(velocity_x_m_s - velocity_y_m_s - rotation_radius_m * angular_velocity_rad_s) /
    wheel_radius_m_;
  wheel_velocities_rad_s[REAR_RIGHT] =
    (velocity_x_m_s + velocity_y_m_s + rotation_radius_m * angular_velocity_rad_s) /
    wheel_radius_m_;

  double maximum_absolute_velocity_rad_s = 0.0;
  for (const double wheel_velocity_rad_s : wheel_velocities_rad_s) {
    maximum_absolute_velocity_rad_s =
      std::max(maximum_absolute_velocity_rad_s, std::abs(wheel_velocity_rad_s));
  }

  const double wheel_velocity_scale =
    maximum_absolute_velocity_rad_s > max_wheel_velocity_rad_s_ ?
    max_wheel_velocity_rad_s_ / maximum_absolute_velocity_rad_s : 1.0;

  actuator_msgs::msg::ActuatorTargetArray command_message;
  command_message.header.stamp = now();
  command_message.actuators.reserve(wheel_velocities_rad_s.size());
  for (std::size_t index = 0; index < wheel_velocities_rad_s.size(); ++index) {
    actuator_msgs::msg::ActuatorTarget target;
    target.logical_id = wheel_logical_ids_[index];
    target.target =
      static_cast<float>(wheel_velocities_rad_s[index] * wheel_velocity_scale);
    command_message.actuators.push_back(target);
  }
  target_array_pub_->publish(command_message);
}
