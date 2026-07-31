#include "mecanum_controller/mecanum_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

MecanumControllerNode::MecanumControllerNode()
: Node("mecanum_controller_node")
{
  declare_parameters();
  get_parameters();

  create_interfaces();
}

void MecanumControllerNode::create_interfaces()
{
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/mecanum/cmd_vel", rclcpp::QoS(qos_depth_),
    std::bind(
      &MecanumControllerNode::cmd_vel_callback, this,
      std::placeholders::_1));
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  operation_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/operation_mode", state_qos,
    std::bind(
      &MecanumControllerNode::operation_mode_callback, this,
      std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(
      &MecanumControllerNode::emergency_stop_callback, this,
      std::placeholders::_1));
  constexpr std::array<const char *, 4> wheel_velocity_topics = {
    "/mecanum/fl/vel_command",
    "/mecanum/fr/vel_command",
    "/mecanum/rl/vel_command",
    "/mecanum/rr/vel_command",
  };
  for (std::size_t index = 0; index < wheel_velocity_pubs_.size(); ++index) {
    wheel_velocity_pubs_[index] = create_publisher<std_msgs::msg::Float32>(
      wheel_velocity_topics[index], rclcpp::QoS(qos_depth_));
  }
}

void MecanumControllerNode::declare_parameters()
{
  declare_parameter<double>("wheel_radius", 0.05);
  declare_parameter<double>("robot_length", 0.47);
  declare_parameter<double>("robot_width", 0.41);
  declare_parameter<double>("max_wheel_velocity_rad_s", 50.0);
  declare_parameter<std::vector<double>>(
    "velocity_corrections",
    {1.0, 1.0, 1.0, 1.0});
  declare_parameter<double>("vx_sign", 1.0);
  declare_parameter<double>("vy_sign", 1.0);
  declare_parameter<double>("angular_z_sign", 1.0);
  declare_parameter<int>("qos_depth", 1);
}

void MecanumControllerNode::get_parameters()
{
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("robot_length", robot_length_);
  get_parameter("robot_width", robot_width_);
  get_parameter("max_wheel_velocity_rad_s", max_wheel_velocity_rad_s_);
  get_parameter("velocity_corrections", velocity_corrections_);
  get_parameter("vx_sign", vx_sign_);
  get_parameter("vy_sign", vy_sign_);
  get_parameter("angular_z_sign", angular_z_sign_);
  get_parameter("qos_depth", qos_depth_);

  if (velocity_corrections_.size() != wheel_vels_.size()) {
    RCLCPP_ERROR(
      get_logger(),
      "velocity_corrections must contain %zu elements, but %zu were provided",
      wheel_vels_.size(), velocity_corrections_.size());
    velocity_corrections_.assign(wheel_vels_.size(), 1.0);
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }
  if (max_wheel_velocity_rad_s_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "max_wheel_velocity_rad_s must be greater than zero. Using 50.0 rad/s.");
    max_wheel_velocity_rad_s_ = 50.0;
  }
}

void MecanumControllerNode::cmd_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_ = *msg;
  publish_wheel_commands();
}

void MecanumControllerNode::operation_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > static_cast<uint8_t>(OperationMode::BELT_ONLY)) {
    operation_mode_ = OperationMode::STOP;
  } else {
    operation_mode_ = static_cast<OperationMode>(msg->data);
  }
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
  // 機体座標系の速度指令に、配線や機構に合わせた符号補正をかける。
  vx_ = last_cmd_vel_.linear.x * vx_sign_;
  vy_ = last_cmd_vel_.linear.y * vy_sign_;
  wz_ = last_cmd_vel_.angular.z * angular_z_sign_;

  if (emergency_stop_active_ || operation_mode_ == OperationMode::STOP) {
    vx_ = 0.0;
    vy_ = 0.0;
    wz_ = 0.0;
  } else if (operation_mode_ == OperationMode::SHOT_CYCLE ||
    operation_mode_ == OperationMode::BELT_ONLY)
  {
    vx_ = 0.0;
    vy_ = 0.0;
  }

  // mecanumの逆運動学で、機体速度から各車輪の目標角速度を計算する。
  wheel_vels_[FL] =
    -(vx_ + vy_ - (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[FR] =
    (vx_ - vy_ + (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[RL] =
    -(vx_ - vy_ - (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[RR] =
    (vx_ + vy_ + (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;

  // 車輪ごとの補正係数をかける。
  std::array<double, 4> corrected_wheel_vels;
  double maximum_wheel_velocity = 0.0;
  for (std::size_t index = 0; index < wheel_vels_.size(); ++index) {
    corrected_wheel_vels[index] =
      wheel_vels_[index] * velocity_corrections_[index];
    maximum_wheel_velocity = std::max(
      maximum_wheel_velocity, std::abs(corrected_wheel_vels[index]));
  }

  // いずれかの車輪が上限を超える場合は、全輪へ同じ比率をかけて
  // 速度ベクトルと車輪間の比率を保ったまま上限内へ収める。
  double velocity_scale = 1.0;
  if (maximum_wheel_velocity > max_wheel_velocity_rad_s_) {
    velocity_scale = max_wheel_velocity_rad_s_ / maximum_wheel_velocity;
  }

  // 上限適用後の車輪速度をhardware_driverへpublishする。
  for (std::size_t index = 0; index < wheel_vels_.size(); ++index) {
    std_msgs::msg::Float32 cmd_msg;
    cmd_msg.data =
      static_cast<float>(corrected_wheel_vels[index] * velocity_scale);
    wheel_velocity_pubs_[index]->publish(cmd_msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumControllerNode>());
  rclcpp::shutdown();
  return 0;
}
