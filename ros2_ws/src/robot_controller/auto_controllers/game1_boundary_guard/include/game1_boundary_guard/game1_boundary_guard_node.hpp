#ifndef GAME1_BOUNDARY_GUARD__GAME1_BOUNDARY_GUARD_NODE_HPP_
#define GAME1_BOUNDARY_GUARD__GAME1_BOUNDARY_GUARD_NODE_HPP_

#include <memory>
#include <string>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace robot_controller {

class Game1BoundaryGuardNode : public rclcpp::Node {
public:
  explicit Game1BoundaryGuardNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void detections_callback(
      const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr message);
  void publish_active(bool active);
  void publish_enabled();
  void publish_debug(bool detection_fresh, double normal_distance,
                     double tangent_distance, double view_angle_deg,
                     double outward_speed_input, double outward_speed_output,
                     double outward_scale);

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      detections_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_fresh_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      measurement_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      velocity_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr limits_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int tag_id_{10};
  std::string base_frame_{"base_link"};
  std::string tag_frame_prefix_{"tag16h5:"};
  double distance_limit_m_{1.0};
  double slowdown_distance_m_{0.35};
  double max_view_angle_rad_{0.78539816339};
  double detection_timeout_s_{0.5};
  double vertical_half_width_m_{1.0};
  double debug_publish_period_s_{0.1};
  int toggle_button_{11};
  bool enabled_{true};
  bool toggle_button_was_pressed_{false};

  bool odometry_received_{false};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  std::string odom_frame_{"odom"};

  bool tag_anchor_valid_{false};
  double tag_odom_x_{0.0};
  double tag_odom_y_{0.0};
  double outward_normal_odom_x_{1.0};
  double outward_normal_odom_y_{0.0};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};
  bool last_active_{false};
};

} // namespace robot_controller

#endif // GAME1_BOUNDARY_GUARD__GAME1_BOUNDARY_GUARD_NODE_HPP_
