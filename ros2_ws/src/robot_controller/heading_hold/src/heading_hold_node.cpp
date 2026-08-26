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
#include "std_msgs/msg/bool.hpp"
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

    // raw cmd_velを受け取り、IMU補正後のcmd_vel_headingとして出力する。
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_cmd_vel_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        receive_command(*message);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {receive_imu(*message);});
    heading_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      heading_enable_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        set_heading_hold_enabled(message->data);
      });
    corrected_command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      corrected_cmd_vel_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

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
    rotation_settle_velocity_rad_s_ =
      declare_parameter("rotation_settle_velocity_rad_s", 0.08);
    rotation_settle_duration_ms_ = declare_parameter("rotation_settle_duration_ms", 100);
    max_correction_rad_s_ = declare_parameter("max_correction_rad_s", 1.5);
    control_period_ms_ = declare_parameter("control_period_ms", 20);
    command_timeout_ms_ = declare_parameter("command_timeout_ms", 500);
    imu_timeout_ms_ = declare_parameter("imu_timeout_ms", 250);
    raw_cmd_vel_topic_ =
      declare_parameter<std::string>("raw_cmd_vel_topic", "/mecanum/cmd_vel");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    corrected_cmd_vel_topic_ =
      declare_parameter<std::string>("corrected_cmd_vel_topic", "/mecanum/cmd_vel_heading");
    heading_enable_topic_ =
      declare_parameter<std::string>("heading_enable_topic", "/heading_control/enable");

    validate_non_negative("kp", kp_);
    validate_non_negative("ki", ki_);
    validate_non_negative("kd", kd_);
    validate_non_negative("integral_limit", integral_limit_rad_s_);
    validate_non_negative("heading_deadband_rad", heading_deadband_rad_);
    validate_non_negative("rotation_input_deadband_rad_s", rotation_input_deadband_rad_s_);
    validate_non_negative("rotation_settle_velocity_rad_s", rotation_settle_velocity_rad_s_);
    validate_positive("max_correction_rad_s", max_correction_rad_s_);
    if (rotation_settle_duration_ms_ <= 0 || control_period_ms_ <= 0 ||
      command_timeout_ms_ <= 0 || imu_timeout_ms_ <= 0)
    {
      throw std::invalid_argument("period and timeout parameters must be positive");
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

    double next_settle_velocity = rotation_settle_velocity_rad_s_;
    int next_settle_duration_ms = rotation_settle_duration_ms_;
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
      } else if (name == "rotation_settle_velocity_rad_s") {
        next_settle_velocity = parameter.as_double();
      } else if (name == "rotation_settle_duration_ms") {
        next_settle_duration_ms = parameter.as_int();
      } else {
        return result;
      }
    }

    if (!std::isfinite(next_kp) || !std::isfinite(next_ki) || !std::isfinite(next_kd) ||
      !std::isfinite(next_integral_limit) || !std::isfinite(next_heading_deadband) ||
      !std::isfinite(next_rotation_deadband) || !std::isfinite(next_settle_velocity) ||
      !std::isfinite(next_max_correction) ||
      next_kp < 0.0 || next_ki < 0.0 || next_kd < 0.0 || next_integral_limit < 0.0 ||
      next_heading_deadband < 0.0 || next_rotation_deadband < 0.0 ||
      next_settle_velocity < 0.0 || next_settle_duration_ms <= 0 ||
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
    rotation_settle_velocity_rad_s_ = next_settle_velocity;
    rotation_settle_duration_ms_ = next_settle_duration_ms;
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
    cmd_vel_timeout_logged_ = false;
  }

  void receive_imu(const sensor_msgs::msg::Imu & message)
  {
    const auto reception_time = now();
    const bool recovered_after_timeout = last_imu_time_.nanoseconds() != 0 &&
      (reception_time - last_imu_time_).nanoseconds() > imu_timeout_ms_ * 1000000LL;

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

    // The IMU callback can update last_imu_time_ before the control timer observes
    // the timeout. Rebase here as well so a restarted IMU cannot apply its new
    // reference frame to the heading target retained from before the outage.
    if (recovered_after_timeout) {
      reset_heading_hold_state();
      RCLCPP_WARN(
        get_logger(),
        "IMU data has recovered after a timeout; rebasing the heading-hold target");
    }

    last_imu_time_ = reception_time;
  }

  void reset_heading_hold_state()
  {
    target_yaw_initialized_ = false;
    integral_error_rad_s_ = 0.0;
    rotation_state_ = RotationState::HOLDING;
    settle_velocity_is_stable_ = false;
  }

  void publish_output_command(const geometry_msgs::msg::Twist & command)
  {
    corrected_command_pub_->publish(command);
  }

  void set_heading_hold_enabled(const bool enabled)
  {
    if (heading_hold_enabled_ == enabled) {
      return;
    }

    // OFF時は補正状態を破棄し、入力cmd_velをそのまま通す。
    heading_hold_enabled_ = enabled;
    reset_heading_hold_state();
    if (!heading_hold_enabled_) {
      publish_output_command(latest_command_);
    }
    RCLCPP_INFO(
      get_logger(), "Heading hold %s; mecanum command is %s",
      heading_hold_enabled_ ? "enabled" : "disabled",
      heading_hold_enabled_ ? "IMU-corrected" : "passed through");
  }

  bool command_is_fresh(const rclcpp::Time & current_time) const
  {
    return last_command_time_.nanoseconds() != 0 &&
           (current_time - last_command_time_).nanoseconds() <=
           command_timeout_ms_ * 1000000LL;
  }

  bool imu_is_fresh(const rclcpp::Time & current_time) const
  {
    return last_imu_time_.nanoseconds() != 0 &&
           (current_time - last_imu_time_).nanoseconds() <= imu_timeout_ms_ * 1000000LL;
  }

  geometry_msgs::msg::Twist select_output_command(
    const rclcpp::Time & current_time, const double dt_s)
  {
    // cmd_vel途絶時は、安全のため補正状態をリセットする。
    if (!command_is_fresh(current_time)) {
      reset_heading_hold_state();
      if (!cmd_vel_timeout_logged_) {
        RCLCPP_INFO(get_logger(), "cmd_vel idle / timed out; publishing zero velocity");
        cmd_vel_timeout_logged_ = true;
      }
      return geometry_msgs::msg::Twist();
    }

    if (!heading_hold_enabled_) {
      return latest_command_;
    }

    // IMU途絶時は補正せず、入力cmd_velをそのまま出力する。
    if (!imu_is_fresh(current_time)) {
      reset_heading_hold_state();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU data is unavailable; passing through the upstream cmd_vel without correction");
      return latest_command_;
    }

    // 旋回入力中はHeading Holdを適用せず、操作者の旋回指令をそのまま出力する。
    if (std::abs(latest_command_.angular.z) > rotation_input_deadband_rad_s_) {
      target_yaw_rad_ = current_yaw_rad_;
      target_yaw_initialized_ = true;
      integral_error_rad_s_ = 0.0;
      rotation_state_ = RotationState::MANUAL_ROTATION;
      return latest_command_;
    }

    if (rotation_state_ == RotationState::MANUAL_ROTATION) {
      // 旋回停止後、機体が静止するまで待ってから新しい保持角度を設定する。
      rotation_state_ = RotationState::SETTLING;
      settle_velocity_is_stable_ = false;
    }

    if (rotation_state_ == RotationState::SETTLING) {
      target_yaw_rad_ = current_yaw_rad_;
      target_yaw_initialized_ = true;
      integral_error_rad_s_ = 0.0;

      auto settling_command = latest_command_;
      settling_command.angular.z = 0.0;

      if (std::abs(current_angular_velocity_z_rad_s_) >
        rotation_settle_velocity_rad_s_)
      {
        settle_velocity_is_stable_ = false;
        return settling_command;
      }

      if (!settle_velocity_is_stable_) {
        settle_velocity_is_stable_ = true;
        settle_stable_since_ = current_time;
        return settling_command;
      }

      if ((current_time - settle_stable_since_).nanoseconds() <
        rotation_settle_duration_ms_ * 1000000LL)
      {
        return settling_command;
      }

      rotation_state_ = RotationState::HOLDING;
      settle_velocity_is_stable_ = false;
      target_yaw_rad_ = current_yaw_rad_;
    }

    if (!target_yaw_initialized_) {
      target_yaw_rad_ = current_yaw_rad_;
      target_yaw_initialized_ = true;
      integral_error_rad_s_ = 0.0;
      return latest_command_;
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
    return corrected_command;
  }

  void control()
  {
    const auto current_time = now();
    const double dt_s = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;

    publish_output_command(select_output_command(current_time, dt_s));
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  geometry_msgs::msg::Twist latest_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};

  enum class RotationState {HOLDING, MANUAL_ROTATION, SETTLING};
  double current_yaw_rad_{0.0};
  double current_angular_velocity_z_rad_s_{0.0};
  double target_yaw_rad_{0.0};
  double integral_error_rad_s_{0.0};
  bool target_yaw_initialized_{false};
  bool heading_hold_enabled_{true};
  bool cmd_vel_timeout_logged_{false};

  double kp_{4.0};
  double ki_{0.0};
  double kd_{0.0};
  double integral_limit_rad_s_{0.5};
  double heading_deadband_rad_{0.02};
  double rotation_input_deadband_rad_s_{0.02};
  double rotation_settle_velocity_rad_s_{0.08};
  int rotation_settle_duration_ms_{100};
  RotationState rotation_state_{RotationState::HOLDING};
  bool settle_velocity_is_stable_{false};
  rclcpp::Time settle_stable_since_{0, 0, RCL_ROS_TIME};
  double max_correction_rad_s_{1.5};
  int control_period_ms_{20};
  int command_timeout_ms_{500};
  int imu_timeout_ms_{250};
  std::string raw_cmd_vel_topic_;
  std::string imu_topic_;
  std::string corrected_cmd_vel_topic_;
  std::string heading_enable_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingHoldNode>());
  rclcpp::shutdown();
  return 0;
}
