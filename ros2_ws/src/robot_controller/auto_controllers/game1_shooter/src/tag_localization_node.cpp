#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_controller
{

struct TagMapPose
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0}; // [rad] Field map yaw orientation of the tag
};

class TagLocalizationNode : public rclcpp::Node
{
public:
  TagLocalizationNode()
  : Node("tag_localization_node")
  {
    load_parameters();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, 10);

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      detection_topic_, 10,
      std::bind(&TagLocalizationNode::detections_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&TagLocalizationNode::camera_info_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "TagLocalizationNode initialized. Listening: %s, Publishing: %s, Map frame: %s, Registered tags: %zu",
      detection_topic_.c_str(), output_topic_.c_str(), map_frame_.c_str(), tag_map_.size());
  }

private:
  void load_parameters()
  {
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/detections");
    output_topic_ = declare_parameter<std::string>("output_topic", "/apriltag/pose");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/image_combine_raw/left/camera_info");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.265);
    camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.035);
    camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.193);

    camera_fx_ = declare_parameter<double>("camera_fx", 800.0);
    camera_fy_ = declare_parameter<double>("camera_fy", 800.0);
    camera_cx_ = declare_parameter<double>("camera_cx", 960.0);
    camera_cy_ = declare_parameter<double>("camera_cy", 540.0);

    covariance_xy_ = declare_parameter<double>("covariance_xy", 0.04);
    covariance_yaw_ = declare_parameter<double>("covariance_yaw", 0.08);

    // Multi-tag map parameters: ids, pos_x, pos_y, pos_z, yaw
    const std::vector<int64_t> default_ids = {0, 1, 2};
    const auto tag_ids = declare_parameter<std::vector<int64_t>>("tag_ids", default_ids);

    const std::vector<double> default_xs = {1.5, 3.5, 0.0};
    const std::vector<double> default_ys = {0.0, 0.0, 0.0};
    const std::vector<double> default_zs = {0.5, 0.5, 0.5};
    const std::vector<double> default_yaws = {M_PI, M_PI, 0.0};

    const auto xs = declare_parameter<std::vector<double>>("tag_positions_x", default_xs);
    const auto ys = declare_parameter<std::vector<double>>("tag_positions_y", default_ys);
    const auto zs = declare_parameter<std::vector<double>>("tag_positions_z", default_zs);
    const auto yaws = declare_parameter<std::vector<double>>("tag_yaws", default_yaws);

    tag_map_.clear();
    for (size_t i = 0; i < tag_ids.size(); ++i) {
      const int id = static_cast<int>(tag_ids[i]);
      TagMapPose p;
      p.id = id;
      p.x = (i < xs.size()) ? xs[i] : 0.0;
      p.y = (i < ys.size()) ? ys[i] : 0.0;
      p.z = (i < zs.size()) ? zs[i] : 0.0;
      p.yaw = (i < yaws.size()) ? yaws[i] : 0.0;
      tag_map_[id] = p;
      RCLCPP_INFO(
        get_logger(), "📍 Registered Tag #%d in Map: pos=[%.2f, %.2f, %.2f], yaw=%.2f rad",
        id, p.x, p.y, p.z, p.yaw);
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->k[0] > 10.0 && msg->k[4] > 10.0) {
      camera_fx_ = msg->k[0];
      camera_cx_ = msg->k[2];
      camera_fy_ = msg->k[4];
      camera_cy_ = msg->k[5];
    }
  }

  void detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (msg->detections.empty()) {
      return;
    }

    const auto now_stamp = msg->header.stamp;

    for (const auto & det : msg->detections) {
      const int id = det.id;
      const auto it = tag_map_.find(id);
      if (it == tag_map_.end()) {
        continue; // 未登録のタグは無視
      }

      const auto & tag_world = it->second;

      // 1. カメラ光学系における相対位置 (X_cam: 深度, Y_cam: 左)
      double tag_pixel_width = 0.0;
      if (det.corners.size() >= 4) {
        const double dx1 = det.corners[0].x - det.corners[1].x;
        const double dy1 = det.corners[0].y - det.corners[1].y;
        const double dx2 = det.corners[3].x - det.corners[2].x;
        const double dy2 = det.corners[3].y - det.corners[2].y;
        tag_pixel_width = (std::hypot(dx1, dy1) + std::hypot(dx2, dy2)) * 0.5;
      }

      constexpr double real_tag_size = 0.18; // 18cm
      if (tag_pixel_width < 10.0) {
        continue;
      }

      const double z_cam = (camera_fx_ * real_tag_size) / tag_pixel_width;
      if (z_cam < 0.3 || z_cam > 6.0) {
        continue; // 外れ値除外
      }

      const double y_cam_left = -(det.centre.x - camera_cx_) * z_cam / camera_fx_;

      // 2. ロボット中心 (base_link) から見た Tag 相対位置 (x_rel, y_rel)
      const double x_rel = z_cam + camera_offset_x_;
      const double y_rel = y_cam_left + camera_offset_y_;
      const double heading_to_tag = std::atan2(y_rel, x_rel);

      // 3. マップ上のロボット自己位置 (robot_x, robot_y, robot_yaw) を逆算
      const double robot_yaw = std::remainder(tag_world.yaw + M_PI - heading_to_tag, 2.0 * M_PI);

      const double cos_yaw = std::cos(robot_yaw);
      const double sin_yaw = std::sin(robot_yaw);
      const double robot_x = tag_world.x - (x_rel * cos_yaw - y_rel * sin_yaw);
      const double robot_y = tag_world.y - (x_rel * sin_yaw + y_rel * cos_yaw);

      // 4. PoseWithCovarianceStamped を作成して EKF へ配信
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.stamp = now_stamp;
      pose_msg.header.frame_id = map_frame_;

      pose_msg.pose.pose.position.x = robot_x;
      pose_msg.pose.pose.position.y = robot_y;
      pose_msg.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, robot_yaw);
      pose_msg.pose.pose.orientation = tf2::toMsg(q);

      pose_msg.pose.covariance.fill(0.0);
      pose_msg.pose.covariance[0] = covariance_xy_;     // Var(x)
      pose_msg.pose.covariance[7] = covariance_xy_;     // Var(y)
      pose_msg.pose.covariance[14] = 1e6;               // Var(z)
      pose_msg.pose.covariance[21] = 1e6;               // Var(roll)
      pose_msg.pose.covariance[28] = 1e6;               // Var(pitch)
      pose_msg.pose.covariance[35] = covariance_yaw_;   // Var(yaw)

      pose_pub_->publish(pose_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "📍 [TagLocalization] Fixed Pose by Tag #%d -> Map Pos: (%.3f, %.3f), Yaw: %+.2f deg (Dist: %.2fm)",
        id, robot_x, robot_y, robot_yaw * 180.0 / M_PI, z_cam);
    }
  }

  // Publishers & Subscriptions
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Parameters
  std::string detection_topic_{"/detections"};
  std::string output_topic_{"/apriltag/pose"};
  std::string camera_info_topic_{"/image_combine_raw/left/camera_info"};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};

  double camera_offset_x_{0.265};
  double camera_offset_y_{0.035};
  double camera_offset_z_{0.193};
  double camera_fx_{800.0};
  double camera_fy_{800.0};
  double camera_cx_{960.0};
  double camera_cy_{540.0};

  double covariance_xy_{0.04};
  double covariance_yaw_{0.08};

  std::unordered_map<int, TagMapPose> tag_map_;
};

}  // namespace robot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::TagLocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
