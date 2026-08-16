#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>
#include <memory>
#include <algorithm>

namespace robot_controller
{

class TestGame1WpMoveNode : public rclcpp::Node
{
public:
  explicit TestGame1WpMoveNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_game1_wp_move_node", options)
  {
    target_x_ = declare_parameter<double>("target_x", 1.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);
    target_yaw_ = declare_parameter<double>("target_yaw", 0.0);
    kp_linear_ = declare_parameter<double>("kp_linear", 1.0);
    kp_angular_ = declare_parameter<double>("kp_angular", 1.5);
    max_linear_vel_ = declare_parameter<double>("max_linear_vel", 1.0);
    max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.0);
    pos_tolerance_ = declare_parameter<double>("pos_tolerance", 0.05);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.05);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 15.0);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
    completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game1/wp_test/completed", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&TestGame1WpMoveNode::odom_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TestGame1WpMoveNode::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "TestGame1WpMoveNode initialized. Target Relative WP: (%.2f, %.2f, %.2f rad)",
      target_x_, target_y_, target_yaw_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double raw_yaw = std::atan2(siny_cosp, cosy_cosp);

    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;

    if (!first_odom_received_) {
      start_x_ = raw_x;
      start_y_ = raw_y;
      start_yaw_ = raw_yaw;
      start_time_ = now();
      first_odom_received_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Start pose zero-reset at X: %.3f, Y: %.3f, Yaw: %.3f rad",
        start_x_, start_y_, start_yaw_);
    }

    const double dx_raw = raw_x - start_x_;
    const double dy_raw = raw_y - start_y_;

    const double cos_yaw = std::cos(-start_yaw_);
    const double sin_yaw = std::sin(-start_yaw_);
    current_x_ = dx_raw * cos_yaw - dy_raw * sin_yaw;
    current_y_ = dx_raw * sin_yaw + dy_raw * cos_yaw;
    current_yaw_ = std::remainder(raw_yaw - start_yaw_, 2.0 * M_PI);

    odom_received_ = true;
  }

  void control_loop()
  {
    if (!odom_received_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for /odometry/filtered...");
      return;
    }

    const double elapsed = (now() - start_time_).seconds();

    const double dx = target_x_ - current_x_;
    const double dy = target_y_ - current_y_;
    const double dist_err = std::hypot(dx, dy);
    const double yaw_err = std::remainder(target_yaw_ - current_yaw_, 2.0 * M_PI);

    if ((dist_err <= pos_tolerance_ && std::abs(yaw_err) <= yaw_tolerance_) || elapsed > timeout_sec_) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());

      std_msgs::msg::Bool comp_msg;
      comp_msg.data = true;
      completed_pub_->publish(comp_msg);

      if (elapsed > timeout_sec_) {
        RCLCPP_WARN(
          get_logger(),
          "Target WP TIMED OUT after %.1fs. Final Pos: (%.3f, %.3f, %.3f)",
          elapsed, current_x_, current_y_, current_yaw_);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Reached Target WP successfully! Pos: (%.3f, %.3f, %.3f)",
          current_x_, current_y_, current_yaw_);
      }

      timer_->cancel();
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(kp_linear_ * dx, -max_linear_vel_, max_linear_vel_);
    cmd.linear.y = std::clamp(kp_linear_ * dy, -max_linear_vel_, max_linear_vel_);
    cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);

    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[Moving] Current: (%.2f, %.2f, %.2f) -> Err: dist=%.3fm, yaw=%.3frad",
      current_x_, current_y_, current_yaw_, dist_err, yaw_err);
  }

  // Parameters
  double target_x_{1.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
  double kp_linear_{1.0};
  double kp_angular_{1.5};
  double max_linear_vel_{1.0};
  double max_angular_vel_{1.0};
  double pos_tolerance_{0.05};
  double yaw_tolerance_{0.05};
  double timeout_sec_{15.0};

  // State
  bool odom_received_{false};
  bool first_odom_received_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  rclcpp::Time start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_controller

int main(int argc, char ** argv)
{
  rclcpp.init(argc, argv);
  auto node = std::make_shared<robot_controller::TestGame1WpMoveNode>();
  rclcpp::spin(node);
  rclcpp.shutdown();
  return 0;
}
