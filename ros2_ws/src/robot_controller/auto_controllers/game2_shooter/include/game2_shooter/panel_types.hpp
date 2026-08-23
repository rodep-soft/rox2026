#pragma once

#include <rclcpp/rclcpp.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_controller
{

struct PanelTagInfo
{
  int tag_id{0};
  int row{0}; // 0: Bottom, 1: Middle, 2: Top
  int col{0}; // 0: Left, 1: Center, 2: Right
  bool shot_completed{false}; // ノックダウン（消失）確認済みフラグ
  int shot_count{0};          // 射出試行回数
  double pixel_x{0.0};
  double pixel_y{0.0};
  double x{0.0};              // ロボットbase_link座標系 (m)
  double y{0.0};              // ロボットbase_link座標系 (m)
  double z{0.0};              // ロボットbase_link座標系 (m)
  bool detected{false};
  rclcpp::Time last_seen;
};

struct TargetCandidate
{
  int row{0};
  std::vector<int> tag_ids;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double heading_err{0.0};
  bool is_pair{false};
  std::string desc;
};

class VisionTracker
{
public:
  VisionTracker() = default;

  void initialize_grid(
    const std::vector<int64_t> & bottom_tags,
    const std::vector<int64_t> & middle_tags,
    const std::vector<int64_t> & top_tags,
    const rclcpp::Time & now);

  void update_detections(
    const apriltag_msgs::msg::AprilTagDetectionArray & msg,
    const rclcpp::Time & now,
    double target_distance,
    double tag_lost_timeout,
    double tag_pitch_x,
    double camera_fx,
    double camera_cx,
    double camera_offset_x,
    double camera_offset_y,
    double camera_offset_z);

  void timeout_unseen_tags(const rclcpp::Time & now, double tag_lost_timeout);

  std::unordered_map<int, PanelTagInfo> & get_panel_grid() { return panel_grid_; }
  const std::unordered_map<int, PanelTagInfo> & get_panel_grid() const { return panel_grid_; }

  double get_estimated_z() const { return estimated_z_; }

private:
  std::unordered_map<int, PanelTagInfo> panel_grid_;
  double estimated_z_{4.0};
};

}  // namespace robot_controller
