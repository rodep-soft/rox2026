#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class MecanumOdometryNode : public rclcpp::Node
{
public:
  MecanumOdometryNode()
  : Node("mecanum_odometry_node")
  {
    wheel_radius_ = declare_parameter("wheel_radius", 0.05);
    robot_length_ = declare_parameter("robot_length", 0.47);
    robot_width_ = declare_parameter("robot_width", 0.41);
    publish_rate_ = declare_parameter("publish_rate", 50.0);
    feedback_timeout_ = declare_parameter("feedback_timeout", 0.25);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    const auto topics = declare_parameter<std::vector<std::string>>(
      "wheel_feedback_topics", {"/mecanum/fl/feedback", "/mecanum/fr/feedback",
        "/mecanum/rl/feedback", "/mecanum/rr/feedback"});

    for (std::size_t i = 0; i < 4; ++i) {
      subscriptions_[i] = create_subscription<std_msgs::msg::Float32>(
        topics[i], 10, [this, i](std_msgs::msg::Float32::SharedPtr msg) {
          wheel_velocity_[i] = msg->data;
          wheel_stamp_[i] = now();
          wheel_received_[i] = true;
        });
    }
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 20);
    last_update_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&MecanumOdometryNode::update, this));
  }

private:
  void update()
  {
    const auto stamp = now();
    const double dt = (stamp - last_update_).seconds();
    last_update_ = stamp;
    if (dt <= 0.0 || dt > 1.0) {return;}

    bool valid = true;
    for (std::size_t i = 0; i < 4; ++i) {
      valid &= wheel_received_[i] && (stamp - wheel_stamp_[i]).seconds() <= feedback_timeout_;
    }
    const double r = wheel_radius_;
    const double k = (robot_length_ + robot_width_) / 2.0;
    const double a = -wheel_velocity_[0] * r;
    const double b = wheel_velocity_[1] * r;
    const double c = -wheel_velocity_[2] * r;
    const double d = wheel_velocity_[3] * r;
    const double vx = valid ? (a + b + c + d) / 4.0 : 0.0;
    const double vy = valid ? (a - b - c + d) / 4.0 : 0.0;
    const double wz = valid ? (-a + b - c + d) / (4.0 * k) : 0.0;

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    x_ += (vx * cos_yaw - vy * sin_yaw) * dt;
    y_ += (vx * sin_yaw + vy * cos_yaw) * dt;
    yaw_ += wz * dt;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = wz;
    msg.pose.covariance[0] = msg.pose.covariance[7] = valid ? 0.02 : 1.0;
    msg.pose.covariance[35] = valid ? 0.05 : 1.0;
    msg.twist.covariance[0] = msg.twist.covariance[7] = valid ? 0.03 : 1.0;
    msg.twist.covariance[35] = valid ? 0.06 : 1.0;
    odom_pub_->publish(msg);
  }

  std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> subscriptions_;
  std::array<double, 4> wheel_velocity_{};
  std::array<rclcpp::Time, 4> wheel_stamp_{
    rclcpp::Time(0), rclcpp::Time(0), rclcpp::Time(0), rclcpp::Time(0)};
  std::array<bool, 4> wheel_received_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_update_;
  double wheel_radius_, robot_length_, robot_width_, publish_rate_, feedback_timeout_;
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  std::string odom_frame_, base_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
