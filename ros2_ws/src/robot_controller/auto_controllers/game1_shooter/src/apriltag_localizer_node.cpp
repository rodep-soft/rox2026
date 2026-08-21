#include <cmath>
#include <map>
#include <memory>
#include <string>
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

struct TagMapEntry
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class ApriltagLocalizerNode : public rclcpp::Node
{
public:
  explicit ApriltagLocalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("apriltag_localizer_node", options)
  {
    load_parameters();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/apriltag/pose", 10);

    detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections", 10,
      std::bind(&ApriltagLocalizerNode::detections_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/image_combine_raw/left/camera_info", 10,
      std::bind(&ApriltagLocalizerNode::camera_info_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "ApriltagLocalizerNode initialized. %zu active field tags registered.",
      tag_map_.size());
  }

private:
  void load_parameters()
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tag_size_ = declare_parameter<double>("tag_size", 0.18);
    camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.265);
    camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.035);
    camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.193);

    camera_fx_ = declare_parameter<double>("camera_fx", 800.0);
    camera_fy_ = declare_parameter<double>("camera_fy", 800.0);
    camera_cx_ = declare_parameter<double>("camera_cx", 960.0);
    camera_cy_ = declare_parameter<double>("camera_cy", 540.0);

    base_pos_cov_ = declare_parameter<double>("base_position_covariance", 0.01);
    base_yaw_cov_ = declare_parameter<double>("base_yaw_covariance", 0.005);
    max_valid_distance_ = declare_parameter<double>("max_valid_distance", 5.5);

    const auto active_ids = declare_parameter<std::vector<int64_t>>("active_tag_ids", {0, 1, 2, 3});

    tag_map_.clear();
    for (const auto id : active_ids) {
      const std::string prefix = "tag_" + std::to_string(id) + "_";
      TagMapEntry entry;
      entry.id = static_cast<int>(id);
      entry.x = declare_parameter<double>(prefix + "x", 0.0);
      entry.y = declare_parameter<double>(prefix + "y", 0.0);
      entry.yaw = declare_parameter<double>(prefix + "yaw", 0.0);
      tag_map_[static_cast<int>(id)] = entry;
      RCLCPP_INFO(
        get_logger(), "📍 [Field Map Tag #%d] Pos: (%.3f, %.3f), Heading: %+.2f rad (%.1f deg)",
        entry.id, entry.x, entry.y, entry.yaw, entry.yaw * 180.0 / M_PI);
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
      const int tag_id = det.id;
      auto it = tag_map_.find(tag_id);
      if (it == tag_map_.end()) {
        continue; // マップ未登録タグは無視
      }

      const auto & field_tag = it->second;

      // ── 📐 1. 検出Tagのピクセル寸法からカメラ距離 Z_cam を高精度逆算 ──
      double tag_pixel_width = 0.0;
      if (det.corners.size() >= 4) {
        const double dx1 = det.corners[0].x - det.corners[1].x;
        const double dy1 = det.corners[0].y - det.corners[1].y;
        const double dx2 = det.corners[3].x - det.corners[2].x;
        const double dy2 = det.corners[3].y - det.corners[2].y;
        tag_pixel_width = (std::hypot(dx1, dy1) + std::hypot(dx2, dy2)) * 0.5;
      }

      if (tag_pixel_width < 12.0) {
        continue; // 微小・ノイズ検出を除外
      }

      const double z_cam = (camera_fx_ * tag_size_) / tag_pixel_width;
      if (z_cam < 0.2 || z_cam > max_valid_distance_) {
        continue; // 距離リミット外除外
      }

      const double y_cam_left = -(det.centre.x - camera_cx_) * z_cam / camera_fx_;

      // ── 📐 2. ロボット中心 (base_link) から見た Tag 相対位置 ──
      const double x_rel = z_cam + camera_offset_x_;
      const double y_rel = y_cam_left + camera_offset_y_;
      const double heading_to_tag = std::atan2(y_rel, x_rel);

      // ── 📐 3. マップ上のロボット絶対姿勢 & 位置逆算 ──
      // ロボットがTag正面を向いているとき robot_yaw = field_tag.yaw - π
      const double robot_yaw = std::remainder(field_tag.yaw + M_PI - heading_to_tag, 2.0 * M_PI);

      const double cos_yaw = std::cos(robot_yaw);
      const double sin_yaw = std::sin(robot_yaw);
      const double robot_x = field_tag.x - (x_rel * cos_yaw - y_rel * sin_yaw);
      const double robot_y = field_tag.y - (x_rel * sin_yaw + y_rel * cos_yaw);

      // ── 📐 4. 距離に応じた適応的共分散スケーリング (遠いほど不確かさを上げる) ──
      // 距離比 (z / 2.0m)^2 に比例して共分散を拡大
      const double dist_factor = std::max(1.0, (z_cam / 2.0) * (z_cam / 2.0));
      const double current_pos_cov = base_pos_cov_ * dist_factor;
      const double current_yaw_cov = base_yaw_cov_ * dist_factor;

      // ── 🚀 5. EKF (/apriltag/pose) への高信頼度配信 ──
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
      pose_msg.pose.covariance[0] = current_pos_cov;   // Var(x)
      pose_msg.pose.covariance[7] = current_pos_cov;   // Var(y)
      pose_msg.pose.covariance[14] = 1e6;              // Var(z)
      pose_msg.pose.covariance[21] = 1e6;              // Var(roll)
      pose_msg.pose.covariance[28] = 1e6;              // Var(pitch)
      pose_msg.pose.covariance[35] = current_yaw_cov;  // Var(yaw)

      pose_pub_->publish(pose_msg);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "📍 [EKF Reset by Tag #%d] Map Pos: (%.3f, %.3f), Yaw: %+.2f deg (Dist: %.2fm, Cov: %.4f)",
        tag_id, robot_x, robot_y, robot_yaw * 180.0 / M_PI, z_cam, current_pos_cov);
    }
  }

  // Publishers & Subscriptions
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Parameters
  std::string base_frame_{"base_link"};
  std::string map_frame_{"map"};
  double tag_size_{0.18};
  double camera_offset_x_{0.265};
  double camera_offset_y_{0.035};
  double camera_offset_z_{0.193};
  double camera_fx_{800.0};
  double camera_fy_{800.0};
  double camera_cx_{960.0};
  double camera_cy_{540.0};

  double base_pos_cov_{0.01};
  double base_yaw_cov_{0.005};
  double max_valid_distance_{5.5};

  std::map<int, TagMapEntry> tag_map_;
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
