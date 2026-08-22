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

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      raw_cmd_vel_topic_, rclcpp::QoS(command_qos_depth_),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        receive_command(*message);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {receive_imu(*message);});

    if (!enable_topic_.empty()) {
      enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        enable_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr message) {
          heading_hold_enabled_ = message->data;
          if (!heading_hold_enabled_) {
            reset_heading_hold();
          }
          RCLCPP_INFO(
            get_logger(), "Heading hold enable state changed: %s",
            heading_hold_enabled_ ? "ENABLED" : "DISABLED");
        });
    }

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
      get_logger(), "Heading hold: input=%s output=%s imu=%s (FF vel=%.2f, acc=%.3f)",
      raw_cmd_vel_topic_.c_str(), corrected_cmd_vel_topic_.c_str(), imu_topic_.c_str(),
      k_ff_vel_, k_ff_acc_);
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
    command_qos_depth_ = declare_parameter("command_qos_depth", 10);
    raw_cmd_vel_topic_ =
      declare_parameter<std::string>("raw_cmd_vel_topic", "/mecanum/cmd_vel");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    corrected_cmd_vel_topic_ =
      declare_parameter<std::string>("corrected_cmd_vel_topic", "/mecanum/cmd_vel_heading");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/heading_control/enable");
    heading_hold_enabled_ = declare_parameter<bool>("enable_heading_hold", true);

    // フィードフォワードパラメータ
    enable_feedforward_ = declare_parameter<bool>("enable_feedforward", true);
    k_ff_vel_ = declare_parameter<double>("k_ff_vel", 1.0);
    k_ff_acc_ = declare_parameter<double>("k_ff_acc", 0.0);
    k_ff_drift_vx_ = declare_parameter<double>("k_ff_drift_vx", 0.0);
    k_ff_drift_vy_ = declare_parameter<double>("k_ff_drift_vy", 0.0);
    k_ff_drift_ax_ = declare_parameter<double>("k_ff_drift_ax", 0.0);
    k_ff_drift_ay_ = declare_parameter<double>("k_ff_drift_ay", 0.0);
    accel_filter_alpha_ = declare_parameter<double>("accel_filter_alpha", 0.3);

    validate_non_negative("kp", kp_);
    validate_non_negative("ki", ki_);
    validate_non_negative("kd", kd_);
    validate_non_negative("integral_limit", integral_limit_rad_s_);
    validate_non_negative("heading_deadband_rad", heading_deadband_rad_);
    validate_non_negative("rotation_input_deadband_rad_s", rotation_input_deadband_rad_s_);
    validate_non_negative("rotation_settle_velocity_rad_s", rotation_settle_velocity_rad_s_);
    validate_positive("max_correction_rad_s", max_correction_rad_s_);
    if (rotation_settle_duration_ms_ <= 0 || control_period_ms_ <= 0 ||
      command_timeout_ms_ <= 0 || imu_timeout_ms_ <= 0 || command_qos_depth_ <= 0)
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
    result.reason = "Only PID, FF, and heading-hold limits can be changed while running";

    double next_kp = kp_;
    double next_ki = ki_;
    double next_kd = kd_;
    double next_integral_limit = integral_limit_rad_s_;
    double next_heading_deadband = heading_deadband_rad_;
    double next_rotation_deadband = rotation_input_deadband_rad_s_;
    double next_max_correction = max_correction_rad_s_;
    double next_settle_velocity = rotation_settle_velocity_rad_s_;
    int next_settle_duration_ms = rotation_settle_duration_ms_;

    bool next_enable_ff = enable_feedforward_;
    double next_k_ff_vel = k_ff_vel_;
    double next_k_ff_acc = k_ff_acc_;
    double next_k_ff_drift_vx = k_ff_drift_vx_;
    double next_k_ff_drift_vy = k_ff_drift_vy_;
    double next_k_ff_drift_ax = k_ff_drift_ax_;
    double next_k_ff_drift_ay = k_ff_drift_ay_;
    double next_accel_filter_alpha = accel_filter_alpha_;
    bool next_enable_heading_hold = heading_hold_enabled_;

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
      } else if (name == "enable_feedforward") {
        next_enable_ff = parameter.as_bool();
      } else if (name == "k_ff_vel") {
        next_k_ff_vel = parameter.as_double();
      } else if (name == "k_ff_acc") {
        next_k_ff_acc = parameter.as_double();
      } else if (name == "k_ff_drift_vx") {
        next_k_ff_drift_vx = parameter.as_double();
      } else if (name == "k_ff_drift_vy") {
        next_k_ff_drift_vy = parameter.as_double();
      } else if (name == "k_ff_drift_ax") {
        next_k_ff_drift_ax = parameter.as_double();
      } else if (name == "k_ff_drift_ay") {
        next_k_ff_drift_ay = parameter.as_double();
      } else if (name == "accel_filter_alpha") {
        next_accel_filter_alpha = parameter.as_double();
      } else if (name == "enable_heading_hold") {
        next_enable_heading_hold = parameter.as_bool();
      } else {
        return result;
      }
    }

    if (!std::isfinite(next_kp) || !std::isfinite(next_ki) || !std::isfinite(next_kd) ||
      !std::isfinite(next_integral_limit) || !std::isfinite(next_heading_deadband) ||
      !std::isfinite(next_rotation_deadband) || !std::isfinite(next_settle_velocity) ||
      !std::isfinite(next_max_correction) ||
      !std::isfinite(next_k_ff_vel) || !std::isfinite(next_k_ff_acc) ||
      !std::isfinite(next_k_ff_drift_vx) || !std::isfinite(next_k_ff_drift_vy) ||
      !std::isfinite(next_k_ff_drift_ax) || !std::isfinite(next_k_ff_drift_ay) ||
      !std::isfinite(next_accel_filter_alpha) ||
      next_kp < 0.0 || next_ki < 0.0 || next_kd < 0.0 || next_integral_limit < 0.0 ||
      next_heading_deadband < 0.0 || next_rotation_deadband < 0.0 ||
      next_settle_velocity < 0.0 || next_settle_duration_ms <= 0 ||
      next_max_correction <= 0.0 || next_accel_filter_alpha < 0.0 || next_accel_filter_alpha > 1.0)
    {
      result.reason = "Gains and limits must be finite and within valid ranges";
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
    enable_feedforward_ = next_enable_ff;
    k_ff_vel_ = next_k_ff_vel;
    k_ff_acc_ = next_k_ff_acc;
    k_ff_drift_vx_ = next_k_ff_drift_vx;
    k_ff_drift_vy_ = next_k_ff_drift_vy;
    k_ff_drift_ax_ = next_k_ff_drift_ax;
    k_ff_drift_ay_ = next_k_ff_drift_ay;
    accel_filter_alpha_ = next_accel_filter_alpha;
    heading_hold_enabled_ = next_enable_heading_hold;

    result.successful = true;
    result.reason = "success";
    RCLCPP_INFO(get_logger(), "Heading-hold and feedforward parameters updated");
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
    rotation_state_ = RotationState::HOLDING;
    settle_velocity_is_stable_ = false;
    prev_cmd_vx_ = 0.0;
    prev_cmd_vy_ = 0.0;
    prev_cmd_wz_ = 0.0;
    filtered_ax_ = 0.0;
    filtered_ay_ = 0.0;
    filtered_ang_accel_ = 0.0;
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
      if (!cmd_vel_timeout_logged_) {
        RCLCPP_INFO(get_logger(), "cmd_vel idle / timed out; publishing zero velocity");
        cmd_vel_timeout_logged_ = true;
      }
      return;
    }

    // Heading Hold 自体が無効化されている場合はスルー
    if (!heading_hold_enabled_) {
      corrected_command_pub_->publish(latest_command_);
      reset_heading_hold();
      return;
    }

    const bool imu_is_fresh = last_imu_time_.nanoseconds() != 0 &&
      (current_time - last_imu_time_).nanoseconds() <= imu_timeout_ms_ * 1000000LL;
    if (!imu_is_fresh) {
      corrected_command_pub_->publish(latest_command_);
      reset_heading_hold();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU data is unavailable; passing through upstream cmd_vel without correction");
      return;
    }

    const double safe_dt_s =
      dt_s > 0.0 && dt_s < 0.5 ? dt_s : static_cast<double>(control_period_ms_) / 1000.0;

    // --- 加速度・加減速の推定 (ローパスフィルタ付き数値微分) ---
    const double raw_ax = (latest_command_.linear.x - prev_cmd_vx_) / safe_dt_s;
    const double raw_ay = (latest_command_.linear.y - prev_cmd_vy_) / safe_dt_s;
    const double raw_ang_accel = (latest_command_.angular.z - prev_cmd_wz_) / safe_dt_s;

    filtered_ax_ = (1.0 - accel_filter_alpha_) * filtered_ax_ + accel_filter_alpha_ * raw_ax;
    filtered_ay_ = (1.0 - accel_filter_alpha_) * filtered_ay_ + accel_filter_alpha_ * raw_ay;
    filtered_ang_accel_ =
      (1.0 - accel_filter_alpha_) * filtered_ang_accel_ + accel_filter_alpha_ * raw_ang_accel;

    prev_cmd_vx_ = latest_command_.linear.x;
    prev_cmd_vy_ = latest_command_.linear.y;
    prev_cmd_wz_ = latest_command_.angular.z;

    // --- フィードフォワード項の計算 ---
    double ff_rad_s = 0.0;
    if (enable_feedforward_) {
      // 1. 旋回速度・角加速度 FF
      ff_rad_s += k_ff_vel_ * latest_command_.angular.z;
      ff_rad_s += k_ff_acc_ * filtered_ang_accel_;

      // 2. 並進速度・加速度起因の回転偏向相殺 FF (Cross-coupling Drift Cancellation)
      ff_rad_s += k_ff_drift_vx_ * latest_command_.linear.x;
      ff_rad_s += k_ff_drift_vy_ * latest_command_.linear.y;
      ff_rad_s += k_ff_drift_ax_ * filtered_ax_;
      ff_rad_s += k_ff_drift_ay_ * filtered_ay_;
    } else {
      ff_rad_s = latest_command_.angular.z;
    }

    // --- 目標方位のトラッキングとフィードバック制御 (2-DOF Control) ---
    const bool is_manual_turning =
      std::abs(latest_command_.angular.z) > rotation_input_deadband_rad_s_;

    if (!target_yaw_initialized_) {
      target_yaw_rad_ = current_yaw_rad_;
      target_yaw_initialized_ = true;
      integral_error_rad_s_ = 0.0;
    }

    if (is_manual_turning) {
      // 旋回中: 指令角速度分だけ目標方位を積分更新し、リアルタイム追従
      target_yaw_rad_ = normalize_angle(target_yaw_rad_ + latest_command_.angular.z * safe_dt_s);
      rotation_state_ = RotationState::MANUAL_ROTATION;
    } else if (rotation_state_ == RotationState::MANUAL_ROTATION) {
      // 旋回直後: 目標方位を現在方位に再同期して過度な戻りを防止
      target_yaw_rad_ = current_yaw_rad_;
      integral_error_rad_s_ = 0.0;
      rotation_state_ = RotationState::HOLDING;
    }

    // 姿勢誤差の計算
    double heading_error_rad = normalize_angle(target_yaw_rad_ - current_yaw_rad_);
    if (std::abs(heading_error_rad) < heading_deadband_rad_) {
      heading_error_rad = 0.0;
    }

    // 積分項 (アンチワインドアップ)
    if (!is_manual_turning) {
      integral_error_rad_s_ = std::clamp(
        integral_error_rad_s_ + heading_error_rad * safe_dt_s,
        -integral_limit_rad_s_, integral_limit_rad_s_);
    } else {
      integral_error_rad_s_ = 0.0;
    }

    // IMU フィードバック補正量
    const double feedback_rad_s = std::clamp(
      kp_ * heading_error_rad + ki_ * integral_error_rad_s_ -
      kd_ * current_angular_velocity_z_rad_s_,
      -max_correction_rad_s_, max_correction_rad_s_);

    // 最終出力 = フィードフォワード + フィードバック
    auto corrected_command = latest_command_;
    corrected_command.angular.z = ff_rad_s + feedback_rad_s;
    corrected_command_pub_->publish(corrected_command);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
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
  bool cmd_vel_timeout_logged_{false};

  // 前周期の指令値（微分用）
  double prev_cmd_vx_{0.0};
  double prev_cmd_vy_{0.0};
  double prev_cmd_wz_{0.0};
  double filtered_ax_{0.0};
  double filtered_ay_{0.0};
  double filtered_ang_accel_{0.0};

  // PID ゲイン・制限
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
  int command_qos_depth_{10};
  std::string raw_cmd_vel_topic_;
  std::string imu_topic_;
  std::string corrected_cmd_vel_topic_;
  std::string enable_topic_;
  bool heading_hold_enabled_{true};

  // フィードフォワード設定
  bool enable_feedforward_{true};
  double k_ff_vel_{1.0};
  double k_ff_acc_{0.0};
  double k_ff_drift_vx_{0.0};
  double k_ff_drift_vy_{0.0};
  double k_ff_drift_ax_{0.0};
  double k_ff_drift_ay_{0.0};
  double accel_filter_alpha_{0.3};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingHoldNode>());
  rclcpp::shutdown();
  return 0;
}
