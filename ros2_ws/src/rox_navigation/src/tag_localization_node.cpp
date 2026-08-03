#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TagLocalizationNode : public rclcpp::Node
{
public:
  TagLocalizationNode()
  : Node("tag_localization_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    detection_topic_ = declare_parameter("detection_topic", "/detection");
    output_topic_ = declare_parameter("output_topic", "/tag/pose");
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    camera_frame_ = declare_parameter("camera_frame", "camera_link");
    tag_position_ = declare_parameter<std::vector<double>>("tag_position", {0.0, 0.0, 1.0});
    tag_rpy_ = declare_parameter<std::vector<double>>("tag_rpy", {0.0, 0.0, 0.0});
    covariance_xy_ = declare_parameter("covariance_xy", 0.04);
    covariance_yaw_ = declare_parameter("covariance_yaw", 0.08);
    if (tag_position_.size() != 3 || tag_rpy_.size() != 3) {
      throw std::invalid_argument("tag_position and tag_rpy must each contain 3 values");
    }
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(output_topic_, 10);
    detection_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      detection_topic_, 10,
      std::bind(&TagLocalizationNode::on_detection, this, std::placeholders::_1));
  }

private:
  void on_detection(const geometry_msgs::msg::PoseStamped::SharedPtr detection)
  {
    const std::string camera_frame = detection->header.frame_id.empty() ?
      camera_frame_ : detection->header.frame_id;
    geometry_msgs::msg::TransformStamped base_to_camera_msg;
    try {
      base_to_camera_msg = tf_buffer_.lookupTransform(
        base_frame_, camera_frame, detection->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Camera TF unavailable: %s",
        ex.what());
      return;
    }

    tf2::Transform map_to_tag;
    map_to_tag.setOrigin(tf2::Vector3(tag_position_[0], tag_position_[1], tag_position_[2]));
    tf2::Quaternion tag_q;
    tag_q.setRPY(tag_rpy_[0], tag_rpy_[1], tag_rpy_[2]);
    map_to_tag.setRotation(tag_q);

    tf2::Transform camera_to_tag;
    tf2::fromMsg(detection->pose, camera_to_tag);
    tf2::Transform base_to_camera;
    tf2::fromMsg(base_to_camera_msg.transform, base_to_camera);
    const tf2::Transform map_to_base =
      map_to_tag * camera_to_tag.inverse() * base_to_camera.inverse();

    geometry_msgs::msg::PoseWithCovarianceStamped output;
    output.header.stamp = detection->header.stamp;
    output.header.frame_id = map_frame_;
    output.pose.pose.position.x = map_to_base.getOrigin().x();
    output.pose.pose.position.y = map_to_base.getOrigin().y();
    output.pose.pose.position.z = map_to_base.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(map_to_base.getRotation());
    output.pose.covariance[0] = covariance_xy_;
    output.pose.covariance[7] = covariance_xy_;
    output.pose.covariance[14] = 1.0;
    output.pose.covariance[21] = 1.0;
    output.pose.covariance[28] = 1.0;
    output.pose.covariance[35] = covariance_yaw_;
    pose_pub_->publish(output);
  }

  std::string detection_topic_, output_topic_, map_frame_, base_frame_, camera_frame_;
  std::vector<double> tag_position_, tag_rpy_;
  double covariance_xy_, covariance_yaw_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
