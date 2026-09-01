#include "game1_boundary_guard/game1_boundary_guard_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
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
      declare_parameter<std::string>("tag_frame_prefix", "16h5:");
  distance_limit_m_ = declare_parameter<double>("distance_limit_m", 1.0);
  slowdown_distance_m_ = declare_parameter<double>("slowdown_distance_m", 0.35);
  max_view_angle_rad_ =
      declare_parameter<double>("max_view_angle_deg", 45.0) * M_PI / 180.0;
  detection_timeout_s_ = declare_parameter<double>("detection_timeout_s", 0.5);
  vertical_half_width_m_ =
      declare_parameter<double>("vertical_half_width_m", 1.0);
  const double debug_publish_rate_hz =
      declare_parameter<double>("debug_publish_rate_hz", 10.0);
  debug_publish_period_s_ = 1.0 / debug_publish_rate_hz;
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
      detection_timeout_s_ <= 0.0 || debug_publish_rate_hz <= 0.0 ||
      toggle_button_ < 0) {
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
  detection_fresh_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/game1/boundary_guard/detection_fresh",
      rclcpp::QoS(1).reliable().transient_local());
  measurement_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/game1/boundary_guard/measurement", rclcpp::QoS(10));
  velocity_debug_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/game1/boundary_guard/velocity_debug", rclcpp::QoS(10));
  limits_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/game1/boundary_guard/limits",
      rclcpp::QoS(1).reliable().transient_local());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/game1/boundary_guard/markers", rclcpp::QoS(10));
  publish_active(false);
  publish_enabled();
  publish_debug(false, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

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
  if (!message->header.frame_id.empty()) {
    odom_frame_ = message->header.frame_id;
  }
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

  std::vector<std::string> tag_frames = {tag_frame_prefix_ +
                                         std::to_string(tag_id_)};
  if (!detection->family.empty()) {
    const std::string family_frame =
        detection->family + ":" + std::to_string(tag_id_);
    tag_frames.push_back(family_frame);
    if (detection->family.rfind("tag", 0) == 0) {
      tag_frames.push_back(detection->family.substr(3) + ":" +
                           std::to_string(tag_id_));
    } else {
      tag_frames.push_back("tag" + detection->family + ":" +
                           std::to_string(tag_id_));
    }
  }
  tag_frames.push_back("16h5:" + std::to_string(tag_id_));
  tag_frames.push_back("tag16h5:" + std::to_string(tag_id_));

  geometry_msgs::msg::TransformStamped transform;
  bool transform_found = false;
  for (const auto &tag_frame : tag_frames) {
    try {
      transform = tf_buffer_->lookupTransform(base_frame_, tag_frame,
                                              tf2::TimePointZero);
      transform_found = true;
      break;
    } catch (const tf2::TransformException &) {
      // Try the next family naming convention.
    }
  }
  if (!transform_found) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Tag %d detected, but no TF exists (tried %s, %s, %s)",
                         tag_id_, tag_frames.front().c_str(),
                         tag_frames[tag_frames.size() - 2].c_str(),
                         tag_frames.back().c_str());
    return;
  }

  try {
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
  } catch (const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Tag %d TF was found but could not be processed: %s",
                         tag_id_, error.what());
  }
}

void Game1BoundaryGuardNode::command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message) {
  geometry_msgs::msg::Twist limited = *message;
  bool active = false;
  double normal_distance = 0.0;
  double tangent_distance = 0.0;
  double view_angle_deg = 0.0;
  double outward_speed_input = 0.0;
  double outward_speed_output = 0.0;
  double outward_scale = 1.0;

  const bool detection_is_fresh =
      tag_anchor_valid_ && odometry_received_ &&
      (now() - last_detection_time_).seconds() <= detection_timeout_s_;
  if (detection_is_fresh) {
    const double robot_from_tag_x = odom_x_ - tag_odom_x_;
    const double robot_from_tag_y = odom_y_ - tag_odom_y_;
    normal_distance = robot_from_tag_x * outward_normal_odom_x_ +
                      robot_from_tag_y * outward_normal_odom_y_;
    tangent_distance = robot_from_tag_x * -outward_normal_odom_y_ +
                       robot_from_tag_y * outward_normal_odom_x_;
    const double view_angle =
        std::abs(std::atan2(tangent_distance, normal_distance));
    view_angle_deg = view_angle * 180.0 / M_PI;
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    const double normal_body_x =
        c * outward_normal_odom_x_ + s * outward_normal_odom_y_;
    const double normal_body_y =
        -s * outward_normal_odom_x_ + c * outward_normal_odom_y_;

    // The safe-side normal points from the tag toward the robot. Therefore the
    // restricted direction (toward the tag) is the negative normal direction.
    outward_speed_input =
        -(limited.linear.x * normal_body_x + limited.linear.y * normal_body_y);
    outward_speed_output = outward_speed_input;

    if (enabled_ && normal_distance > 0.0 &&
        std::abs(tangent_distance) <= vertical_half_width_m_ &&
        view_angle < max_view_angle_rad_) {
      // Only the component approaching the tag/boundary is reduced. Motion
      // parallel to the boundary and motion away from the tag pass unchanged.
      const double outward_speed = outward_speed_input;
      const double remaining =
          std::max(0.0, normal_distance - distance_limit_m_);
      outward_scale = std::clamp(remaining / slowdown_distance_m_, 0.0, 1.0);
      const double requested_outward_speed = std::max(0.0, outward_speed);
      const double allowed_outward_speed =
          requested_outward_speed * outward_scale;

      if (outward_speed > allowed_outward_speed) {
        const double correction = outward_speed - allowed_outward_speed;
        limited.linear.x += correction * normal_body_x;
        limited.linear.y += correction * normal_body_y;
        outward_speed_output = allowed_outward_speed;
        active = true;
      }
    }
  }

  publish_debug(detection_is_fresh, normal_distance, tangent_distance,
                view_angle_deg, outward_speed_input, outward_speed_output,
                outward_scale);
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

void Game1BoundaryGuardNode::publish_debug(
    bool detection_fresh, double normal_distance, double tangent_distance,
    double view_angle_deg, double outward_speed_input,
    double outward_speed_output, double outward_scale) {
  const auto stamp = now();
  if (last_debug_publish_time_.nanoseconds() != 0 &&
      (stamp - last_debug_publish_time_).seconds() < debug_publish_period_s_) {
    return;
  }
  last_debug_publish_time_ = stamp;

  std_msgs::msg::Bool fresh_message;
  fresh_message.data = detection_fresh;
  detection_fresh_pub_->publish(fresh_message);

  geometry_msgs::msg::Vector3Stamped measurement;
  measurement.header.stamp = stamp;
  measurement.header.frame_id = odom_frame_;
  measurement.vector.x = normal_distance;
  measurement.vector.y = tangent_distance;
  measurement.vector.z = view_angle_deg;
  measurement_pub_->publish(measurement);

  geometry_msgs::msg::Vector3Stamped velocity;
  velocity.header = measurement.header;
  velocity.vector.x = outward_speed_input;
  velocity.vector.y = outward_speed_output;
  velocity.vector.z = outward_scale;
  velocity_debug_pub_->publish(velocity);

  geometry_msgs::msg::Vector3Stamped limits;
  limits.header = measurement.header;
  limits.vector.x = distance_limit_m_;
  limits.vector.y = vertical_half_width_m_;
  limits.vector.z = distance_limit_m_ + slowdown_distance_m_;
  limits_pub_->publish(limits);

  if (!tag_anchor_valid_ || !odometry_received_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  auto make_marker = [&](int id, int type, const std::string &name) {
    visualization_msgs::msg::Marker marker;
    marker.header = measurement.header;
    marker.ns = "game1_boundary_guard";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.text = name;
    return marker;
  };
  auto point_at = [&](double normal, double tangent) {
    geometry_msgs::msg::Point point;
    point.x = tag_odom_x_ + normal * outward_normal_odom_x_ -
              tangent * outward_normal_odom_y_;
    point.y = tag_odom_y_ + normal * outward_normal_odom_y_ +
              tangent * outward_normal_odom_x_;
    point.z = 0.05;
    return point;
  };

  auto tag_marker =
      make_marker(0, visualization_msgs::msg::Marker::SPHERE, "ID 10");
  tag_marker.pose.position = point_at(0.0, 0.0);
  tag_marker.scale.x = 0.20;
  tag_marker.scale.y = 0.20;
  tag_marker.scale.z = 0.20;
  tag_marker.color.r = 0.1F;
  tag_marker.color.g = detection_fresh ? 1.0F : 0.3F;
  tag_marker.color.b = 1.0F;
  tag_marker.color.a = 1.0F;
  marker_array.markers.push_back(tag_marker);

  auto limit_marker = make_marker(
      1, visualization_msgs::msg::Marker::LINE_STRIP, "minimum distance");
  limit_marker.points = {point_at(distance_limit_m_, -vertical_half_width_m_),
                         point_at(distance_limit_m_, vertical_half_width_m_)};
  limit_marker.scale.x = 0.06;
  limit_marker.color.r = 1.0F;
  limit_marker.color.a = 1.0F;
  marker_array.markers.push_back(limit_marker);

  auto slowdown_marker = make_marker(
      2, visualization_msgs::msg::Marker::LINE_STRIP, "slowdown start");
  const double slowdown_start = distance_limit_m_ + slowdown_distance_m_;
  slowdown_marker.points = {point_at(slowdown_start, -vertical_half_width_m_),
                            point_at(slowdown_start, vertical_half_width_m_)};
  slowdown_marker.scale.x = 0.035;
  slowdown_marker.color.r = 1.0F;
  slowdown_marker.color.g = 0.8F;
  slowdown_marker.color.a = 1.0F;
  marker_array.markers.push_back(slowdown_marker);

  auto normal_marker = make_marker(3, visualization_msgs::msg::Marker::ARROW,
                                   "restricted +X direction");
  geometry_msgs::msg::Point direction_start;
  direction_start.x = odom_x_;
  direction_start.y = odom_y_;
  direction_start.z = 0.25;
  geometry_msgs::msg::Point direction_end = direction_start;
  direction_end.x -= outward_normal_odom_x_ * 0.8;
  direction_end.y -= outward_normal_odom_y_ * 0.8;
  normal_marker.points = {direction_start, direction_end};
  direction_end.x = direction_start.x - outward_normal_odom_x_ * 1.5;
  direction_end.y = direction_start.y - outward_normal_odom_y_ * 1.5;
  normal_marker.points = {direction_start, direction_end};
  normal_marker.scale.x = 0.08;
  normal_marker.scale.y = 0.16;
  normal_marker.scale.z = 0.20;
  normal_marker.color.r = 1.0F;
  normal_marker.color.b = 1.0F;
  normal_marker.color.a = 1.0F;
  marker_array.markers.push_back(normal_marker);

  auto robot_marker =
      make_marker(4, visualization_msgs::msg::Marker::CYLINDER, "robot");
  robot_marker.pose.position.x = odom_x_;
  robot_marker.pose.position.y = odom_y_;
  robot_marker.pose.position.z = 0.08;
  robot_marker.scale.x = 0.30;
  robot_marker.scale.y = 0.30;
  robot_marker.scale.z = 0.16;
  robot_marker.color.g = enabled_ ? 1.0F : 0.3F;
  robot_marker.color.r = enabled_ ? 0.0F : 0.7F;
  robot_marker.color.a = 0.9F;
  marker_array.markers.push_back(robot_marker);

  auto text_marker = make_marker(
      5, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "status");
  text_marker.pose.position.x = odom_x_;
  text_marker.pose.position.y = odom_y_;
  text_marker.pose.position.z = 0.65;
  text_marker.scale.z = 0.22;
  text_marker.color.r = 1.0F;
  text_marker.color.g = detection_fresh ? 1.0F : 0.3F;
  text_marker.color.b = 1.0F;
  text_marker.color.a = 1.0F;
  std::ostringstream status;
  status << "ID10 " << (detection_fresh ? "FRESH" : "STALE") << " | guard "
         << (enabled_ ? "ON" : "OFF") << "\n"
         << std::fixed << std::setprecision(2) << "distance " << normal_distance
         << " / " << distance_limit_m_ << " m | vertical " << tangent_distance
         << " m\n"
         << "angle " << view_angle_deg << " deg | speed scale "
         << outward_scale;
  text_marker.text = status.str();
  marker_array.markers.push_back(text_marker);

  const auto add_filled_zone = [&](int id, double near_normal,
                                   double far_normal, float red, float green,
                                   const std::string &name) {
    auto zone =
        make_marker(id, visualization_msgs::msg::Marker::TRIANGLE_LIST, name);
    const double angle_tangent = std::tan(max_view_angle_rad_);
    const double near_half_width =
        std::min(vertical_half_width_m_, near_normal * angle_tangent);
    const double far_half_width =
        std::min(vertical_half_width_m_, far_normal * angle_tangent);
    const auto near_left = point_at(near_normal, -near_half_width);
    const auto near_right = point_at(near_normal, near_half_width);
    const auto far_left = point_at(far_normal, -far_half_width);
    const auto far_right = point_at(far_normal, far_half_width);
    zone.points = {near_left, far_left,  far_right,
                   near_left, far_right, near_right};
    zone.color.r = red;
    zone.color.g = green;
    zone.color.b = 0.0F;
    zone.color.a = 0.28F;
    marker_array.markers.push_back(zone);
  };

  // These polygons match all spatial conditions used by the limiter:
  // tag-facing angle, vertical half-width, minimum distance and slowdown band.
  add_filled_zone(6, 0.0, distance_limit_m_, 1.0F, 0.0F, "STOP ZONE");
  add_filled_zone(7, distance_limit_m_, slowdown_start, 1.0F, 0.8F,
                  "SLOWDOWN ZONE");

  const auto add_zone_label = [&](int id, double normal,
                                  const std::string &label, float red,
                                  float green) {
    auto marker = make_marker(
        id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, label);
    marker.pose.position = point_at(normal, 0.0);
    marker.pose.position.z = 0.18;
    marker.scale.z = 0.24;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.a = 1.0F;
    marker.text = label;
    marker_array.markers.push_back(marker);
  };
  add_zone_label(8, distance_limit_m_ * 0.55, "STOP (+X = 0)", 1.0F, 0.1F);
  add_zone_label(9, (distance_limit_m_ + slowdown_start) * 0.5, "SLOWDOWN",
                 1.0F, 0.85F);

  markers_pub_->publish(marker_array);
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
