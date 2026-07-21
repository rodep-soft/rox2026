#include "mecanum_controller/mecanum_controller_node.hpp"

#include <functional>
#include <memory>

MecanumControllerNode::MecanumControllerNode()
: Node("mecanum_controller_node")
{
  declare_parameters();
  get_parameters();

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&MecanumControllerNode::cmd_vel_callback, this, std::placeholders::_1));
  for (std::size_t index = 0; index < wheel_velocity_pubs_.size(); ++index) {
    wheel_velocity_pubs_[index] = create_publisher<std_msgs::msg::Float32>(
      wheel_velocity_topics_[index], rclcpp::QoS(qos_depth_));
  }
}

void MecanumControllerNode::declare_parameters()
{
  declare_parameter<double>("wheel_radius", 0.05);
  declare_parameter<double>("robot_length", 0.47);
  declare_parameter<double>("robot_width", 0.41);
  declare_parameter<std::vector<double>>("velocity_corrections", {1.0, 1.0, 1.0, 1.0});
  declare_parameter<double>("vx_sign", 1.0);
  declare_parameter<double>("vy_sign", 1.0);
  declare_parameter<double>("angular_z_sign", 1.0);
  declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel");
  declare_parameter<std::string>("front_left_velocity_topic", "/mecanum/front_left/velocity");
  declare_parameter<std::string>("front_right_velocity_topic", "/mecanum/front_right/velocity");
  declare_parameter<std::string>("rear_left_velocity_topic", "/mecanum/rear_left/velocity");
  declare_parameter<std::string>("rear_right_velocity_topic", "/mecanum/rear_right/velocity");
  declare_parameter<int>("qos_depth", 10);
}

void MecanumControllerNode::get_parameters()
{
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("robot_length", robot_length_);
  get_parameter("robot_width", robot_width_);
  get_parameter("velocity_corrections", velocity_corrections_);
  get_parameter("vx_sign", vx_sign_);
  get_parameter("vy_sign", vy_sign_);
  get_parameter("angular_z_sign", angular_z_sign_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("front_left_velocity_topic", wheel_velocity_topics_[FL]);
  get_parameter("front_right_velocity_topic", wheel_velocity_topics_[FR]);
  get_parameter("rear_left_velocity_topic", wheel_velocity_topics_[RL]);
  get_parameter("rear_right_velocity_topic", wheel_velocity_topics_[RR]);
  get_parameter("qos_depth", qos_depth_);

  if (velocity_corrections_.size() != wheel_vels_.size()) {
    RCLCPP_ERROR(
      get_logger(), "velocity_corrections must contain %zu elements, but %zu were provided",
      wheel_vels_.size(), velocity_corrections_.size());
    velocity_corrections_.assign(wheel_vels_.size(), 1.0);
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using the default value of 10.");
    qos_depth_ = 10;
  }
}

void MecanumControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vx_ = msg->linear.x * vx_sign_;
  vy_ = msg->linear.y * vy_sign_;
  wz_ = msg->angular.z * angular_z_sign_;

  wheel_vels_[FL] = -(vx_ + vy_ - (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[FR] = (vx_ - vy_ + (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[RL] = -(vx_ - vy_ - (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;
  wheel_vels_[RR] = (vx_ + vy_ + (robot_length_ + robot_width_) / 2.0 * wz_) / wheel_radius_;

  for (std::size_t index = 0; index < wheel_vels_.size(); ++index) {
    std_msgs::msg::Float32 cmd_msg;
    cmd_msg.data = static_cast<float>(wheel_vels_[index] * velocity_corrections_[index]);
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
