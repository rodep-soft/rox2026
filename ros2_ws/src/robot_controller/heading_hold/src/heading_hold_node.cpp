#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>

namespace
{
constexpr int TIMER_PERIOD_MS = 20;
constexpr double CMD_VEL_TIMEOUT_SEC = 0.5;
constexpr double IMU_TIMEOUT_SEC = 1;

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
  : Node("heading_hold_node"),
    last_cmd_vel_time_(0, 0, RCL_ROS_TIME),
    last_imu_time_(0, 0, RCL_ROS_TIME)
  {
    last_update_time_ = this->now();

    this->declare_parameter<double>("kp", 0.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("integral_limit", 0.5);
    this->declare_parameter<std::string>("raw_cmd_vel_topic", "/mecanum/cmd_vel_raw");
    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("corrected_cmd_vel_topic", "/mecanum/cmd_vel");

    kp_ = this->get_parameter("kp").as_double();
    ki_ = this->get_parameter("ki").as_double();
    kd_ = this->get_parameter("kd").as_double();
    integral_limit_ = this->get_parameter("integral_limit").as_double();
    raw_cmd_vel_topic_ = this->get_parameter("raw_cmd_vel_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    corrected_cmd_vel_topic_ = this->get_parameter("corrected_cmd_vel_topic").as_string();

    raw_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      raw_cmd_vel_topic_, 1,
      std::bind(&HeadingHoldNode::rawVelocityCallback, this, std::placeholders::_1));

    const auto imu_sub_qos = rclcpp::SensorDataQoS();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_sub_qos,
      std::bind(&HeadingHoldNode::imuCallback, this, std::placeholders::_1));

    corrected_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(corrected_cmd_vel_topic_, 10);

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HeadingHoldNode::on_set_parameters, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(TIMER_PERIOD_MS),
      std::bind(&HeadingHoldNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Heading Hold Node started.");
  }

private:
  double current_yaw_ = 0.0;
  double target_yaw_ = 0.0;
  bool has_target_yaw_ = false;
  geometry_msgs::msg::Twist latest_raw_vel_;
  double kp_;
  double ki_;
  double kd_;
  double integral_ = 0.0;
  double integral_limit_;
  double prev_error_ = 0.0;
  std::string raw_cmd_vel_topic_;
  std::string imu_topic_;
  std::string corrected_cmd_vel_topic_;

  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_vel_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & param : parameters) {
      if (param.get_name() == "kp") {
        kp_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated kp: %f", kp_);
      } else if (param.get_name() == "ki") {
        ki_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated ki: %f", ki_);
      } else if (param.get_name() == "kd") {
        kd_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated kd: %f", kd_);
      } else if (param.get_name() == "integral_limit") {
        integral_limit_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated integral_limit: %f", integral_limit_);
      }
    }
    return result;
  }

  void rawVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_raw_vel_ = *msg;
    last_cmd_vel_time_ = this->now();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
    //RCLCPP_INFO(this->get_logger(), "Current roll, pitch, yaw: %lf, %lf, %lf", roll, pitch, yaw);
    last_imu_time_ = this->now();
  }

  void controlLoop()
  {
    rclcpp::Time now = this->now();

    if (last_cmd_vel_time_.nanoseconds() == 0 || (now - last_cmd_vel_time_).seconds() > CMD_VEL_TIMEOUT_SEC) {
      geometry_msgs::msg::Twist stop_vel;
      corrected_vel_pub_->publish(stop_vel);
      integral_ = 0.0;
      prev_error_ = 0.0;
      has_target_yaw_ = false;
      last_update_time_ = now;
      RCLCPP_INFO(this->get_logger(), "did not recieve Twist command");
      return;
    }

    if (last_imu_time_.nanoseconds() == 0 || (now - last_imu_time_).seconds() > IMU_TIMEOUT_SEC) {
      corrected_vel_pub_->publish(latest_raw_vel_);
      integral_ = 0.0;
      prev_error_ = 0.0;
      has_target_yaw_ = false;
      last_update_time_ = now;
      RCLCPP_INFO(this->get_logger(), "did not recieve imu data");
      return;
    }

    double dt = (now - last_update_time_).seconds();
    if (dt < TIMER_PERIOD_MS / 1000.0 / 2 || dt > TIMER_PERIOD_MS / 1000.0 * 1.5) {
      dt = TIMER_PERIOD_MS / 1000.0;
    }
    last_update_time_ = now;

    if (!has_target_yaw_) {
      target_yaw_ = current_yaw_;
      has_target_yaw_ = true;
    }

    if (std::abs(latest_raw_vel_.angular.z) > 0) {
      target_yaw_ = current_yaw_;
      integral_ = 0.0;
      prev_error_ = 0.0;

      corrected_vel_pub_->publish(latest_raw_vel_);
      return;
    }

    // 4. 姿勢維持（PID計算）と不感帯（デッドバンド）の追加
    double error = normalizeAngle(target_yaw_ - current_yaw_);
    if (std::abs(error) < 0.06) {
      error = 0.0;
    }

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

    RCLCPP_INFO(this->get_logger(), "target yaw: %lf, current_yaw: %lf", target_yaw_, current_yaw_);
    corrected_vel_pub_->publish(out);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingHoldNode>());
  rclcpp::shutdown();
  return 0;
}
