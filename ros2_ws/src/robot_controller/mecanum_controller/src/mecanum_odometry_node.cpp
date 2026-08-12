#include "mecanum_controller/mecanum_odometry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

class MecanumOdometryNode : public rclcpp::Node
{
public:
  MecanumOdometryNode()
  : Node("mecanum_odometry_node")
  {
    configure_parameters();
    state_sub_ = create_subscription<actuator_msgs::msg::ActuatorStateArray>(
      state_topic_, 10,
      [this](const actuator_msgs::msg::ActuatorStateArray::SharedPtr msg) {receive_state(*msg);});
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    last_update_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double, std::milli>(publish_period_ms_),
      [this]() {update();});
  }

private:
  static void require_positive(const std::string & name, const double value)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and greater than zero");
    }
  }

  void configure_parameters()
  {
    wheel_radius_m_ = declare_parameter("wheel_radius", 0.075);
    robot_length_m_ = declare_parameter("robot_length", 0.355);
    robot_width_m_ = declare_parameter("robot_width", 0.353);
    publish_period_ms_ = declare_parameter("publish_period_ms", 20.0);
    feedback_timeout_ms_ = declare_parameter("feedback_timeout_ms", 250.0);
    scale_x_ = declare_parameter("velocity_scale_x", 1.0);
    scale_y_ = declare_parameter("velocity_scale_y", 1.0);
    scale_yaw_ = declare_parameter("velocity_scale_yaw", 1.0);
    filter_alpha_ = declare_parameter("velocity_filter_alpha", 0.35);
    slip_enabled_ = declare_parameter("slip_compensation.enabled", true);
    accel_threshold_x_ =
      declare_parameter("slip_compensation.acceleration_threshold_x_m_s2", 1.0);
    accel_threshold_y_ =
      declare_parameter("slip_compensation.acceleration_threshold_y_m_s2", 0.7);
    accel_threshold_yaw_ =
      declare_parameter("slip_compensation.angular_acceleration_threshold_rad_s2", 2.0);
    max_covariance_multiplier_ =
      declare_parameter("slip_compensation.maximum_covariance_multiplier", 10.0);
    pose_covariance_xy_ = declare_parameter("pose_covariance_xy", 0.02);
    pose_covariance_yaw_ = declare_parameter("pose_covariance_yaw", 0.05);
    twist_covariance_xy_ = declare_parameter("twist_covariance_xy", 0.03);
    twist_covariance_yaw_ = declare_parameter("twist_covariance_yaw", 0.06);
    state_topic_ = declare_parameter<std::string>("state_array_topic", "/edulite/state_array");
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/wheel/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    const auto ids = declare_parameter<std::vector<int64_t>>("wheel_logical_ids", {0, 1, 2, 3});
    if (ids.size() != wheel_ids_.size()) {
      throw std::invalid_argument("wheel_logical_ids must contain exactly four values");
    }
    for (std::size_t index = 0; index < ids.size(); ++index) {
      if (ids[index] < 0 || ids[index] > 65535) {
        throw std::invalid_argument("wheel_logical_ids values must be in [0, 65535]");
      }
      wheel_ids_[index] = static_cast<uint16_t>(ids[index]);
      if (std::count(wheel_ids_.cbegin(), wheel_ids_.cbegin() + index + 1, wheel_ids_[index]) > 1) {
        throw std::invalid_argument("wheel_logical_ids values must be unique");
      }
    }

    for (const auto & parameter : std::vector<std::pair<std::string, double>>{
      {"wheel_radius", wheel_radius_m_}, {"publish_period_ms", publish_period_ms_},
      {"feedback_timeout_ms", feedback_timeout_ms_}, {"velocity_scale_x", scale_x_},
      {"velocity_scale_y", scale_y_}, {"velocity_scale_yaw", scale_yaw_},
      {"acceleration_threshold_x", accel_threshold_x_},
      {"acceleration_threshold_y", accel_threshold_y_},
      {"acceleration_threshold_yaw", accel_threshold_yaw_},
      {"maximum_covariance_multiplier", max_covariance_multiplier_}})
    {
      require_positive(parameter.first, parameter.second);
    }
    if (!std::isfinite(robot_length_m_) || !std::isfinite(robot_width_m_) ||
      robot_length_m_ < 0.0 || robot_width_m_ < 0.0 || robot_length_m_ + robot_width_m_ <= 0.0)
    {
      throw std::invalid_argument("robot dimensions must be non-negative with a positive sum");
    }
    if (!std::isfinite(filter_alpha_) || filter_alpha_ <= 0.0 || filter_alpha_ > 1.0) {
      throw std::invalid_argument("velocity_filter_alpha must be in (0, 1]");
    }
  }

  void receive_state(const actuator_msgs::msg::ActuatorStateArray & message)
  {
    std::array<double, mecanum_odometry::WHEEL_COUNT> received{};
    std::array<bool, mecanum_odometry::WHEEL_COUNT> found{};
    for (const auto & actuator : message.actuators) {
      const auto iterator = std::find(wheel_ids_.cbegin(), wheel_ids_.cend(), actuator.logical_id);
      if (iterator == wheel_ids_.cend()) {
        continue;
      }
      const auto index = static_cast<std::size_t>(std::distance(wheel_ids_.cbegin(), iterator));
      if (actuator.state == actuator_msgs::msg::ActuatorState::STATE_READY &&
        std::isfinite(actuator.velocity))
      {
        received[index] = actuator.velocity;
        found[index] = true;
      }
    }
    feedback_valid_ = std::all_of(found.cbegin(), found.cend(), [](bool value) {return value;});
    feedback_stamp_ = now();
    if (feedback_valid_) {
      wheel_velocity_ = received;
    }
  }

  mecanum_odometry::BodyVelocity measured_velocity() const
  {
    const double rotation_radius_m = (robot_length_m_ + robot_width_m_) / 2.0;
    auto velocity = mecanum_odometry::calculate_body_velocity(
      wheel_velocity_, wheel_radius_m_, rotation_radius_m);
    velocity.x_m_s *= scale_x_;
    velocity.y_m_s *= scale_y_;
    velocity.yaw_rad_s *= scale_yaw_;
    return velocity;
  }

  mecanum_odometry::BodyVelocity filtered_velocity(
    const mecanum_odometry::BodyVelocity & measured)
  {
    if (!filter_initialized_) {
      filtered_ = measured;
      filter_initialized_ = true;
      return filtered_;
    }
    const double old_weight = 1.0 - filter_alpha_;
    filtered_.x_m_s = filter_alpha_ * measured.x_m_s + old_weight * filtered_.x_m_s;
    filtered_.y_m_s = filter_alpha_ * measured.y_m_s + old_weight * filtered_.y_m_s;
    filtered_.yaw_rad_s = filter_alpha_ * measured.yaw_rad_s + old_weight * filtered_.yaw_rad_s;
    return filtered_;
  }

  double covariance_multiplier(
    const mecanum_odometry::BodyVelocity & velocity, const double dt_s)
  {
    if (!slip_enabled_ || !previous_velocity_initialized_) {
      previous_velocity_ = velocity;
      previous_velocity_initialized_ = true;
      return 1.0;
    }
    const mecanum_odometry::BodyVelocity acceleration{
      (velocity.x_m_s - previous_velocity_.x_m_s) / dt_s,
      (velocity.y_m_s - previous_velocity_.y_m_s) / dt_s,
      (velocity.yaw_rad_s - previous_velocity_.yaw_rad_s) / dt_s};
    previous_velocity_ = velocity;
    return mecanum_odometry::calculate_covariance_multiplier(
      acceleration, accel_threshold_x_, accel_threshold_y_, accel_threshold_yaw_,
      max_covariance_multiplier_);
  }

  void update()
  {
    const auto stamp = now();
    const double dt_s = (stamp - last_update_).seconds();
    last_update_ = stamp;
    if (dt_s <= 0.0 || dt_s > 1.0) {
      return;
    }

    const double feedback_age_ms = (stamp - feedback_stamp_).seconds() * 1000.0;
    const bool usable = feedback_valid_ && feedback_age_ms >= 0.0 &&
      feedback_age_ms <= feedback_timeout_ms_;
    mecanum_odometry::BodyVelocity velocity;
    double covariance_scale = 1000.0;
    if (usable) {
      velocity = filtered_velocity(measured_velocity());
      covariance_scale = covariance_multiplier(velocity, dt_s);
      const double cos_yaw = std::cos(yaw_rad_);
      const double sin_yaw = std::sin(yaw_rad_);
      position_x_m_ += (velocity.x_m_s * cos_yaw - velocity.y_m_s * sin_yaw) * dt_s;
      position_y_m_ += (velocity.x_m_s * sin_yaw + velocity.y_m_s * cos_yaw) * dt_s;
      yaw_rad_ = std::remainder(yaw_rad_ + velocity.yaw_rad_s * dt_s, 2.0 * M_PI);
    } else {
      filter_initialized_ = false;
      previous_velocity_initialized_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel feedback is incomplete, not ready, or stale; integration is paused");
    }
    publish(stamp, velocity, covariance_scale);
  }

  void publish(
    const rclcpp::Time & stamp, const mecanum_odometry::BodyVelocity & velocity,
    const double covariance_scale)
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = position_x_m_;
    message.pose.pose.position.y = position_y_m_;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_rad_);
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    message.twist.twist.linear.x = velocity.x_m_s;
    message.twist.twist.linear.y = velocity.y_m_s;
    message.twist.twist.angular.z = velocity.yaw_rad_s;
    message.pose.covariance[0] = message.pose.covariance[7] = pose_covariance_xy_ *
      covariance_scale;
    message.pose.covariance[35] = pose_covariance_yaw_ * covariance_scale;
    message.twist.covariance[0] = message.twist.covariance[7] = twist_covariance_xy_ *
      covariance_scale;
    message.twist.covariance[35] = twist_covariance_yaw_ * covariance_scale;
    odom_pub_->publish(message);
  }

  rclcpp::Subscription<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<uint16_t, mecanum_odometry::WHEEL_COUNT> wheel_ids_{};
  std::array<double, mecanum_odometry::WHEEL_COUNT> wheel_velocity_{};
  rclcpp::Time feedback_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  mecanum_odometry::BodyVelocity filtered_;
  mecanum_odometry::BodyVelocity previous_velocity_;
  bool feedback_valid_{false};
  bool filter_initialized_{false};
  bool previous_velocity_initialized_{false};
  double wheel_radius_m_, robot_length_m_, robot_width_m_;
  double publish_period_ms_, feedback_timeout_ms_;
  double scale_x_, scale_y_, scale_yaw_, filter_alpha_;
  bool slip_enabled_;
  double accel_threshold_x_, accel_threshold_y_, accel_threshold_yaw_;
  double max_covariance_multiplier_;
  double pose_covariance_xy_, pose_covariance_yaw_, twist_covariance_xy_, twist_covariance_yaw_;
  double position_x_m_{0.0}, position_y_m_{0.0}, yaw_rad_{0.0};
  std::string state_topic_, odom_topic_, odom_frame_, base_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
