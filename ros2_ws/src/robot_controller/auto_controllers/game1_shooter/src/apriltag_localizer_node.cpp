#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace robot_controller
{

struct TagMapEntry
{
  double x;
  double y;
  double yaw;
};

class ApriltagLocalizerNode : public rclcpp::Node
{
public:
  explicit ApriltagLocalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("apriltag_localizer_node", options)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");
    position_covariance_ = declare_parameter<double>("position_covariance", 0.01);
    yaw_covariance_ = declare_parameter<double>("yaw_covariance", 0.005);

    // 使用するタグIDリスト読み込み
    const auto active_ids = declare_parameter<std::vector<int64_t>>("active_tag_ids", {});

    // タグIDごとの絶対座標マップを構築
    for (const auto id : active_ids) {
      const std::string prefix = "tag_" + std::to_string(id) + "_";
      TagMapEntry entry;
      entry.x   = declare_parameter<double>(prefix + "x",   0.0);
      entry.y   = declare_parameter<double>(prefix + "y",   0.0);
      entry.yaw = declare_parameter<double>(prefix + "yaw", 0.0);
      tag_map_[static_cast<int>(id)] = entry;
      RCLCPP_INFO(
        get_logger(), "Registered Tag ID=%ld at field (%.3f, %.3f, %.3frad)",
        id, entry.x, entry.y, entry.yaw);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/apriltag/pose", 10);

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&ApriltagLocalizerNode::detections_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ApriltagLocalizerNode initialized. %zu tags registered.",
      tag_map_.size());
  }

private:
  void detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    for (const auto & det : msg->detections) {
      const int tag_id = det.id;
      if (tag_map_.find(tag_id) == tag_map_.end()) {
        continue;  // 使用リストに無いタグはスキップ
      }

      const auto & field_pose = tag_map_.at(tag_id);
      const std::string tag_frame = tag_prefix_ + std::to_string(tag_id);

      // TF2でbase_link → tag_frame の変換を取得
      geometry_msgs::msg::TransformStamped t_base_tag;
      try {
        t_base_tag = tf_buffer_->lookupTransform(
          base_frame_, tag_frame,
          tf2::TimePointZero, tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF lookup failed for %s: %s", tag_frame.c_str(), ex.what());
        continue;
      }

      // base_link→タグ の相対位置・向き
      const double bx = t_base_tag.transform.translation.x;
      const double by = t_base_tag.transform.translation.y;
      const double & qx = t_base_tag.transform.rotation.x;
      const double & qy = t_base_tag.transform.rotation.y;
      const double & qz = t_base_tag.transform.rotation.z;
      const double & qw = t_base_tag.transform.rotation.w;
      const double tag_yaw_in_base = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));

      // タグの絶対yawとbase→タグのyawから、ロボットの絶対yawを計算
      // robot_yaw_field = field_tag_yaw - tag_yaw_in_base
      const double robot_yaw = std::remainder(
        field_pose.yaw - tag_yaw_in_base, 2.0 * M_PI);

      // タグの絶対位置からロボットの絶対位置を逆算
      // robot = tag_field - R(robot_yaw) * base_tag_vec
      const double cos_yaw = std::cos(robot_yaw);
      const double sin_yaw = std::sin(robot_yaw);
      const double robot_x = field_pose.x - (cos_yaw * bx - sin_yaw * by);
      const double robot_y = field_pose.y - (sin_yaw * bx + cos_yaw * by);

      // PoseWithCovarianceStamped を組み立てて publish
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.stamp = msg->header.stamp;
      pose_msg.header.frame_id = "odom";

      pose_msg.pose.pose.position.x = robot_x;
      pose_msg.pose.pose.position.y = robot_y;
      pose_msg.pose.pose.position.z = 0.0;

      // yaw → quaternion
      pose_msg.pose.pose.orientation.x = 0.0;
      pose_msg.pose.pose.orientation.y = 0.0;
      pose_msg.pose.pose.orientation.z = std::sin(robot_yaw / 2.0);
      pose_msg.pose.pose.orientation.w = std::cos(robot_yaw / 2.0);

      // 共分散 (EKFが参照するx,y,yaw成分のみ設定)
      // 6x6行列 [x,y,z,roll,pitch,yaw]
      pose_msg.pose.covariance[0]  = position_covariance_;  // x
      pose_msg.pose.covariance[7]  = position_covariance_;  // y
      pose_msg.pose.covariance[14] = 1e6;                   // z (使わない)
      pose_msg.pose.covariance[21] = 1e6;                   // roll
      pose_msg.pose.covariance[28] = 1e6;                   // pitch
      pose_msg.pose.covariance[35] = yaw_covariance_;       // yaw

      pose_pub_->publish(pose_msg);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[Tag ID=%d] Robot absolute pose: (%.3f, %.3f, %.3frad)",
        tag_id, robot_x, robot_y, robot_yaw);
    }
  }

  std::string base_frame_;
  std::string tag_prefix_;
  double position_covariance_;
  double yaw_covariance_;

  std::map<int, TagMapEntry> tag_map_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
};

}  // namespace robot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::ApriltagLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
