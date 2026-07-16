#include "mecanum_controller/mecanum_controller_node.hpp"

#include <functional>
#include <memory>

MecanumControllerNode::MecanumControllerNode()
: Node("mecanum_controller_node")
{
  declare_parameters();
  get_parameters();

  robot_command_sub_ = create_subscription<robot_interfaces::msg::RobotCommand>(
    "/robot_command", 10,
    std::bind(&MecanumControllerNode::robot_command_callback, this, std::placeholders::_1));
  motor_vel_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motor_vel", 10);
}

void MecanumControllerNode::declare_parameters()
{
  declare_parameter<double>("wheel_radius", 0.05);
  declare_parameter<double>("robot_length", 0.4);
  declare_parameter<double>("robot_width", 0.4);
  declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4});
  declare_parameter<double>("vx_sign", 1.0);
  declare_parameter<double>("vy_sign", 1.0);
  declare_parameter<double>("angular_z_sign", 1.0);
}

void MecanumControllerNode::get_parameters()
{
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("robot_length", robot_length_);
  get_parameter("robot_width", robot_width_);
  get_parameter("motor_ids", motor_ids_);
  get_parameter("vx_sign", vx_sign_);
  get_parameter("vy_sign", vy_sign_);
  get_parameter("angular_z_sign", angular_z_sign_);

  if (motor_ids_.size() != wheel_vels_.size()) {
    RCLCPP_ERROR(
      get_logger(), "motor_ids must contain %zu elements, but %zu were provided",
      wheel_vels_.size(), motor_ids_.size());
  }
}

void MecanumControllerNode::robot_command_callback(
  const robot_interfaces::msg::RobotCommand::SharedPtr msg)
{
  vx_ = msg->cmd_vel.linear.x * vx_sign_;
  vy_ = msg->cmd_vel.linear.y * vy_sign_;
  wz_ = msg->cmd_vel.angular.z * angular_z_sign_;

  wheel_vels_[FL] = -(vx_ + vy_ - (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[FR] = (vx_ - vy_ + (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[RL] = -(vx_ - vy_ - (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[RR] = (vx_ + vy_ + (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;

  std_msgs::msg::Float32MultiArray cmd_msg;
  for (const auto & vel : wheel_vels_) {
    cmd_msg.data.push_back(static_cast<float>(vel));
  }
  motor_vel_pub_->publish(cmd_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumControllerNode>());
  rclcpp::shutdown();
  return 0;
}
