#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace
{
constexpr int TIMER_PERIOD_MS = 20;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}
}

class HeadingHoldNode : public rclcpp::Node
{
public:
  HeadingHoldNode()
  : Node("heading_hold_node")
  {
    last_time_ = this->now();

    this->declare_parameter<double>("kp", 0.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("integral_limit", 0.5);

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    integral_limit_ = this->get_parameter("integral_limit").as_double();

    raw_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 1,
      std::bind(&HeadingHoldNode::rawVelocityCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 1,
      std::bind(&HeadingHoldNode::imuCallback, this, std::placeholders::_1));

    corrected_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(TIMER_PERIOD_MS),
      std::bind(&HeadingHoldNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Heading Hold Node started.");
  }

private:
  double current_yaw_ = 0.0;
  double target_yaw_ = 0.0;
  bool has_imu_ = false;
  geometry_msgs::msg::Twist latest_raw_vel_;
  double kp_;
  double ki_;
  double kd_;
  double integral_ = 0.0;
  double integral_limit_;
  double prev_error_ = 0.0;
  rclcpp::Time last_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  void rawVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_raw_vel_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    current_yaw_ = tf2::getYaw(msg->orientation);

    if (!has_imu_) {
      target_yaw_ = current_yaw_;
      has_imu_ = true;
    }
  }

  void controlLoop()
{
  if (!has_imu_) {
      last_time_ = this->now();
      corrected_vel_pub_->publish(latest_raw_vel_);
      return;
  }
  rclcpp::Time now = this->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  if (std::abs(latest_raw_vel_.angular.z) > 0.01) {
      target_yaw_ = current_yaw_;
      integral_ = 0.0;
      prev_error_ = 0.0;

      corrected_vel_pub_->publish(latest_raw_vel_);
      return;
  }

  double error = normalizeAngle(target_yaw_ - current_yaw_);

  integral_ += error * dt;

  if (integral_ > integral_limit_) {
    integral_ = integral_limit_;
  } else if (integral_ < -integral_limit_) {
    integral_ = -integral_limit_;
  }

  double derivative = (dt > 0.0) ? (error - prev_error_) / dt : 0.0;

  prev_error_ = error;

  double correction = kp_ * error + ki_ * integral_ + kd_ * derivative;

  geometry_msgs::msg::Twist out = latest_raw_vel_;
  out.angular.z += correction;

  corrected_vel_pub_->publish(out);
}

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingHoldNode>());
  rclcpp::shutdown();
  return 0;
}
