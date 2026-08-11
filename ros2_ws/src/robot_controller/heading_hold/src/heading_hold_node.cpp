#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
double normalize_angle(const double angle_rad)
{
  return std::remainder(angle_rad, 2.0 * M_PI);
}

bool is_finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}
}  // namespace

class HeadingHoldNode : public rclcpp::Node
{
public:
  HeadingHoldNode()
  : Node("heading_hold_node")
  {
    configure_parameters();

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_cmd_vel_topic_, rclcpp::QoS(command_qos_depth_),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        receive_command(*message);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {receive_imu(*message);});
    corrected_command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      corrected_cmd_vel_topic_, rclcpp::QoS(command_qos_depth_));

    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return update_parameters(parameters);
      });
    last_control_time_ = now();
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_), [this]() {control();});

    RCLCPP_INFO(
      get_logger(), "Heading hold: input=%s output=%s imu=%s",
      raw_cmd_vel_topic_.c_str(), corrected_cmd_vel_topic_.c_str(), imu_topic_.c_str());
  }

private:
  void configure_parameters()
  {
    kp_ = declare_parameter("kp", 4.0);
    ki_ = declare_parameter("ki", 0.0);
    kd_ = declare_parameter("kd", 0.0);
    integral_limit_rad_s_ = declare_parameter("integral_limit", 0.5);
    heading_deadband_rad_ = declare_parameter("heading_deadband_rad", 0.02);
    rotation_input_deadband_rad_s_ =
      declare_parameter("rotation_input_deadband_rad_s", 0.02);
    max_correction_rad_s_ = declare_parameter("max_correction_rad_s", 1.5);
    control_period_ms_ = declare_parameter("control_period_ms", 20);
    command_timeout_ms_ = declare_parameter("command_timeout_ms", 500);
    imu_timeout_ms_ = declare_parameter("imu_timeout_ms", 250);
    command_qos_depth_ = declare_parameter("command_qos_depth", 10);
    raw_cmd_vel_topic_ =
      declare_parameter<std::string>("raw_cmd_vel_topic", "/mecanum/cmd_vel");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    corrected_cmd_vel_topic_ =
      declare_parameter<std::string>("corrected_cmd_vel_topic", "/mecanum/cmd_vel_heading");

    validate_non_negative("kp", kp_);
    validate_non_negative("ki", ki_);
    validate_non_negative("kd", kd_);
    validate_non_negative("integral_limit", integral_limit_rad_s_);
    validate_non_negative("heading_deadband_rad", heading_deadband_rad_);
    validate_non_negative("rotation_input_deadband_rad_s", rotation_input_deadband_rad_s_);
    validate_positive("max_correction_rad_s", max_correction_rad_s_);
    if (control_period_ms_ <= 0 || command_timeout_ms_ <= 0 || imu_timeout_ms_ <= 0 ||
      command_qos_depth_ <= 0)
    {
      throw std::invalid_argument("period, timeout, and QoS parameters must be positive");
    }
  }

  static void validate_non_negative(const std::string & name, const double value)
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be finite and non-negative");
    }
  }

  static void validate_positive(const std::string & name, const double value)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
  }

  rcl_interfaces::msg::SetParametersResult update_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = false;
    result.reason = "Only PID and heading-hold limits can be changed while running";

    double next_kp = kp_;
    double next_ki = ki_;
    double next_kd = kd_;
    double next_integral_limit = integral_limit_rad_s_;
    double next_heading_deadband = heading_deadband_rad_;
    double next_rotation_deadband = rotation_input_deadband_rad_s_;
    double next_max_correction = max_correction_rad_s_;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "kp") {
        next_kp = parameter.as_double();
      } else if (name == "ki") {
        next_ki = parameter.as_double();
      } else if (name == "kd") {
        next_kd = parameter.as_double();
      } else if (name == "integral_limit") {
        next_integral_limit = parameter.as_double();
      } else if (name == "heading_deadband_rad") {
        next_heading_deadband = parameter.as_double();
      } else if (name == "rotation_input_deadband_rad_s") {
        next_rotation_deadband = parameter.as_double();
      } else if (name == "max_correction_rad_s") {
        next_max_correction = parameter.as_double();
      } else {
        return result;
      }
    }

    if (!std::isfinite(next_kp) || !std::isfinite(next_ki) || !std::isfinite(next_kd) ||
      !std::isfinite(next_integral_limit) || !std::isfinite(next_heading_deadband) ||
      !std::isfinite(next_rotation_deadband) || !std::isfinite(next_max_correction) ||
      next_kp < 0.0 || next_ki < 0.0 || next_kd < 0.0 || next_integral_limit < 0.0 ||
      next_heading_deadband < 0.0 || next_rotation_deadband < 0.0 ||
      next_max_correction <= 0.0)
    {
      result.reason = "PID gains and limits must be finite and non-negative";
      return result;
    }

    kp_ = next_kp;
    ki_ = next_ki;
    kd_ = next_kd;
    integral_limit_rad_s_ = next_integral_limit;
    heading_deadband_rad_ = next_heading_deadband;
    rotation_input_deadband_rad_s_ = next_rotation_deadband;
    max_correction_rad_s_ = next_max_correction;
    result.successful = true;
    result.reason = "success";
    RCLCPP_INFO(get_logger(), "Heading-hold parameters updated");
    return result;
  }

  void receive_command(const geometry_msgs::msg::Twist & message)
  {
    if (!is_finite_twist(message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignored a non-finite cmd_vel message");
      return;
    }
    latest_command_ = message;
    last_command_time_ = now();
  }

  void receive_imu(const sensor_msgs::msg::Imu & message)
  {
    const auto & orientation = message.orientation;
    if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
      !std::isfinite(message.angular_velocity.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignored IMU data containing non-finite values");
      return;
    }

    tf2::Quaternion quaternion;
    tf2::fromMsg(orientation, quaternion);
    const double norm_squared = quaternion.length2();
    if (norm_squared < 1e-12) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignored an invalid IMU quaternion");
      return;
    }
    quaternion.normalize();

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll_rad, pitch_rad, current_yaw_rad_);
    current_yaw_rad_ = -current_yaw_rad_;
    current_angular_velocity_z_rad_s_ = message.angular_velocity.z;
    last_imu_time_ = now();
  }

  void reset_heading_hold()
  {
    target_yaw_initialized_ = false;
    integral_error_rad_s_ = 0.0;
  }

  void control()
  {
    const auto current_time = now();
    const double dt_s = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;

    if (last_command_time_.nanoseconds() == 0 ||
      (current_time - last_command_time_).nanoseconds() > command_timeout_ms_ * 1000000LL)
    {
      corrected_command_pub_->publish(geometry_msgs::msg::Twist());
      reset_heading_hold();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "cmd_vel timed out; publishing zero velocity");
      return;
    }

    const bool imu_is_fresh = last_imu_time_.nanoseconds() != 0 &&
      (current_time - last_imu_time_).nanoseconds() <= imu_timeout_ms_ * 1000000LL;
    if (!imu_is_fresh) {
      corrected_command_pub_->publish(latest_command_);
      reset_heading_hold();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU data is unavailable; passing through the upstream cmd_vel without correction");
      return;
    }

    if (std::abs(latest_command_.angular.z) > rotation_input_deadband_rad_s_) {
      target_yaw_rad_ = current_yaw_rad_;
      target_yaw_initialized_ = true;
      integral_error_rad_s_ = 0.0;
      corrected_command_pub_->publish(latest_command_);
      return;
    }

    if (!target_yaw_initialized_) {
      target_yaw_rad_ = current_yaw_rad_;
      target_yaw_initialized_ = true;
      integral_error_rad_s_ = 0.0;
    }

    const double safe_dt_s =
      dt_s > 0.0 && dt_s < 0.5 ? dt_s : static_cast<double>(control_period_ms_) / 1000.0;
    double heading_error_rad = normalize_angle(target_yaw_rad_ - current_yaw_rad_);
    if (std::abs(heading_error_rad) < heading_deadband_rad_) {
      heading_error_rad = 0.0;
    }

    integral_error_rad_s_ = std::clamp(
      integral_error_rad_s_ + heading_error_rad * safe_dt_s,
      -integral_limit_rad_s_, integral_limit_rad_s_);

    const double correction_rad_s = std::clamp(
      kp_ * heading_error_rad + ki_ * integral_error_rad_s_ -
      kd_ * current_angular_velocity_z_rad_s_,
      -max_correction_rad_s_, max_correction_rad_s_);

    auto corrected_command = latest_command_;
    corrected_command.angular.z = correction_rad_s;
    corrected_command_pub_->publish(corrected_command);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  geometry_msgs::msg::Twist latest_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  double current_yaw_rad_{0.0};
  double current_angular_velocity_z_rad_s_{0.0};
  double target_yaw_rad_{0.0};
  double integral_error_rad_s_{0.0};
  bool target_yaw_initialized_{false};

  double kp_{4.0};
  double ki_{0.0};
  double kd_{0.0};
  double integral_limit_rad_s_{0.5};
  double heading_deadband_rad_{0.02};
  double rotation_input_deadband_rad_s_{0.02};
  double max_correction_rad_s_{1.5};
  int control_period_ms_{20};
  int command_timeout_ms_{500};
  int imu_timeout_ms_{250};
  int command_qos_depth_{10};
  std::string raw_cmd_vel_topic_;
  std::string imu_topic_;
  std::string corrected_cmd_vel_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingHoldNode>());
  rclcpp::shutdown();
  return 0;
}
