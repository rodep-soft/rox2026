#include "game1_boundary_guard/game1_boundary_guard_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"

namespace robot_controller {

Game1BoundaryGuardNode::Game1BoundaryGuardNode(
    const rclcpp::NodeOptions &options)
    : Node("game1_boundary_guard", options) {
  tag_id_ = declare_parameter<int>("tag_id", 10);
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  tag_frame_prefix_ =
      declare_parameter<std::string>("tag_frame_prefix", "tag16h5:");
  distance_limit_m_ = declare_parameter<double>("distance_limit_m", 1.0);
  slowdown_distance_m_ = declare_parameter<double>("slowdown_distance_m", 0.35);
  max_view_angle_rad_ =
      declare_parameter<double>("max_view_angle_deg", 45.0) * M_PI / 180.0;
  detection_timeout_s_ = declare_parameter<double>("detection_timeout_s", 0.5);
  vertical_half_width_m_ =
      declare_parameter<double>("vertical_half_width_m", 1.0);
  toggle_button_ = declare_parameter<int>("toggle_button", 11);
  enabled_ = declare_parameter<bool>("enabled_at_startup", true);

  const auto input_topic = declare_parameter<std::string>("input_cmd_vel_topic",
                                                          "/game1/cmd_vel_raw");
  const auto output_topic =
      declare_parameter<std::string>("output_cmd_vel_topic", "/drive/cmd_vel");
  const auto detections_topic =
      declare_parameter<std::string>("detections_topic", "/detections");
  const auto odometry_topic =
      declare_parameter<std::string>("odometry_topic", "/odometry/filtered");

  if (distance_limit_m_ <= 0.0 || slowdown_distance_m_ <= 0.0 ||
      vertical_half_width_m_ <= 0.0 || max_view_angle_rad_ <= 0.0 ||
      detection_timeout_s_ <= 0.0 || toggle_button_ < 0) {
    throw std::invalid_argument(
        "Boundary guard distance, angle, and timeout must be positive");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  detections_sub_ =
      create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
          detections_topic, rclcpp::SensorDataQoS(),
          std::bind(&Game1BoundaryGuardNode::detections_callback, this,
                    std::placeholders::_1));
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(20),
      std::bind(&Game1BoundaryGuardNode::odometry_callback, this,
                std::placeholders::_1));
  command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, rclcpp::QoS(10),
      std::bind(&Game1BoundaryGuardNode::command_callback, this,
                std::placeholders::_1));
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SensorDataQoS(),
      std::bind(&Game1BoundaryGuardNode::joy_callback, this,
                std::placeholders::_1));

  command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic,
                                                             rclcpp::QoS(10));
  active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/game1/boundary_guard/active",
      rclcpp::QoS(1).reliable().transient_local());
  enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/game1/boundary_guard/enabled",
      rclcpp::QoS(1).reliable().transient_local());
  publish_active(false);
  publish_enabled();

  RCLCPP_INFO(get_logger(),
              "Game1 boundary guard: tag=%d, horizontal limit=%.2f m, view "
              "angle=%.1f deg, vertical range=+/-%.2f m, timeout=%.2f s, "
              "toggle=button %d",
              tag_id_, distance_limit_m_, max_view_angle_rad_ * 180.0 / M_PI,
              vertical_half_width_m_, detection_timeout_s_, toggle_button_);
}

void Game1BoundaryGuardNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  odometry_received_ = true;
  odom_x_ = message->pose.pose.position.x;
  odom_y_ = message->pose.pose.position.y;

  const auto &q_msg = message->pose.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, odom_yaw_);
}

void Game1BoundaryGuardNode::detections_callback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message) {
  if (!odometry_received_) {
    return;
  }

  const auto detection =
      std::find_if(message->detections.begin(), message->detections.end(),
                   [this](const auto &item) { return item.id == tag_id_; });
  if (detection == message->detections.end()) {
    return;
  }

  std::string tag_frame = tag_frame_prefix_ + std::to_string(tag_id_);
  if (!detection->family.empty()) {
    tag_frame = detection->family + ":" + std::to_string(tag_id_);
  }

  try {
    const auto transform =
        tf_buffer_->lookupTransform(base_frame_, tag_frame, tf2::TimePointZero);
    const double tag_base_x = transform.transform.translation.x;
    const double tag_base_y = transform.transform.translation.y;
    const double planar_distance = std::hypot(tag_base_x, tag_base_y);
    if (!std::isfinite(tag_base_x) || !std::isfinite(tag_base_y) ||
        planar_distance < 0.05) {
      return;
    }

    // AprilTag's local +Z axis is normal to its printed plane. Project it onto
    // the floor, then choose the sign that points from the tag toward the
    // robot.
    const auto &q_msg = transform.transform.rotation;
    const tf2::Quaternion tag_rotation(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    const tf2::Vector3 normal_base_3d =
        tf2::Matrix3x3(tag_rotation) * tf2::Vector3(0.0, 0.0, 1.0);
    double normal_base_x = normal_base_3d.x();
    double normal_base_y = normal_base_3d.y();
    const double normal_norm = std::hypot(normal_base_x, normal_base_y);
    if (!std::isfinite(normal_norm) || normal_norm < 0.5) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Tag %d normal is not sufficiently horizontal; ignoring detection",
          tag_id_);
      return;
    }
    normal_base_x /= normal_norm;
    normal_base_y /= normal_norm;
    if (normal_base_x * -tag_base_x + normal_base_y * -tag_base_y < 0.0) {
      normal_base_x = -normal_base_x;
      normal_base_y = -normal_base_y;
    }

    const double normal_distance =
        normal_base_x * -tag_base_x + normal_base_y * -tag_base_y;
    const double tangent_distance =
        -normal_base_y * -tag_base_x + normal_base_x * -tag_base_y;
    const double view_angle =
        std::abs(std::atan2(tangent_distance, normal_distance));
    if (normal_distance <= 0.0 || view_angle >= max_view_angle_rad_) {
      return;
    }

    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    tag_odom_x_ = odom_x_ + c * tag_base_x - s * tag_base_y;
    tag_odom_y_ = odom_y_ + s * tag_base_x + c * tag_base_y;
    outward_normal_odom_x_ = c * normal_base_x - s * normal_base_y;
    outward_normal_odom_y_ = s * normal_base_x + c * normal_base_y;
    tag_anchor_valid_ = true;
    last_detection_time_ = now();
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Tag %d was detected but TF is unavailable: %s",
                         tag_id_, error.what());
  }
}

void Game1BoundaryGuardNode::command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message) {
  geometry_msgs::msg::Twist limited = *message;
  bool active = false;

  if (!enabled_) {
    publish_active(false);
    command_pub_->publish(limited);
    return;
  }

  const bool detection_is_fresh =
      tag_anchor_valid_ && odometry_received_ &&
      (now() - last_detection_time_).seconds() <= detection_timeout_s_;
  if (detection_is_fresh) {
    const double robot_from_tag_x = odom_x_ - tag_odom_x_;
    const double robot_from_tag_y = odom_y_ - tag_odom_y_;
    const double normal_distance = robot_from_tag_x * outward_normal_odom_x_ +
                                   robot_from_tag_y * outward_normal_odom_y_;
    const double tangent_distance = robot_from_tag_x * -outward_normal_odom_y_ +
                                    robot_from_tag_y * outward_normal_odom_x_;
    const double view_angle =
        std::abs(std::atan2(tangent_distance, normal_distance));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    const double normal_body_x =
        c * outward_normal_odom_x_ + s * outward_normal_odom_y_;
    const double normal_body_y =
        -s * outward_normal_odom_x_ + c * outward_normal_odom_y_;

    if (normal_distance > 0.0 &&
        std::abs(tangent_distance) <= vertical_half_width_m_ &&
        view_angle < max_view_angle_rad_) {
      // Only the component moving away from the tag plane is reduced. Motion
      // parallel to the boundary and motion back toward the tag pass unchanged.
      const double outward_speed =
          limited.linear.x * normal_body_x + limited.linear.y * normal_body_y;
      const double remaining =
          std::max(0.0, distance_limit_m_ - normal_distance);
      const double outward_scale =
          std::clamp(remaining / slowdown_distance_m_, 0.0, 1.0);
      const double requested_outward_speed = std::max(0.0, outward_speed);
      const double allowed_outward_speed =
          requested_outward_speed * outward_scale;

      if (outward_speed > allowed_outward_speed) {
        const double correction = allowed_outward_speed - outward_speed;
        limited.linear.x += correction * normal_body_x;
        limited.linear.y += correction * normal_body_y;
        active = true;
      }
    }
  }

  publish_active(active);
  command_pub_->publish(limited);
}

void Game1BoundaryGuardNode::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr message) {
  const bool pressed =
      static_cast<size_t>(toggle_button_) < message->buttons.size() &&
      message->buttons[toggle_button_] != 0;
  if (pressed && !toggle_button_was_pressed_) {
    enabled_ = !enabled_;
    publish_enabled();
    if (!enabled_) {
      publish_active(false);
    }
    RCLCPP_WARN(get_logger(), "Game1 boundary guard %s by button %d",
                enabled_ ? "ENABLED" : "DISABLED", toggle_button_);
  }
  toggle_button_was_pressed_ = pressed;
}

void Game1BoundaryGuardNode::publish_enabled() {
  std_msgs::msg::Bool message;
  message.data = enabled_;
  enabled_pub_->publish(message);
}

void Game1BoundaryGuardNode::publish_active(bool active) {
  std_msgs::msg::Bool message;
  message.data = active;
  active_pub_->publish(message);
  if (active != last_active_) {
    RCLCPP_INFO(get_logger(), "Boundary speed limiting %s",
                active ? "ACTIVE" : "inactive");
  }
  last_active_ = active;
}

} // namespace robot_controller
