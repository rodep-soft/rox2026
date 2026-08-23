#include "game2_shooter/panel_types.hpp"
#include <algorithm>

namespace robot_controller
{

void VisionTracker::initialize_grid(
  const std::vector<int64_t> & bottom_tags,
  const std::vector<int64_t> & middle_tags,
  const std::vector<int64_t> & top_tags,
  const rclcpp::Time & now)
{
  panel_grid_.clear();
  auto register_row = [this, &now](const std::vector<int64_t> & tags, int row) {
    for (size_t col = 0; col < tags.size(); ++col) {
      const int id = static_cast<int>(tags[col]);
      PanelTagInfo info;
      info.tag_id = id;
      info.row = row;
      info.col = static_cast<int>(col);
      info.shot_completed = false;
      info.shot_count = 0;
      info.detected = false;
      info.last_seen = now;
      panel_grid_[id] = info;
    }
  };
  register_row(bottom_tags, 0);
  register_row(middle_tags, 1);
  register_row(top_tags, 2);
}

void VisionTracker::update_detections(
  const apriltag_msgs::msg::AprilTagDetectionArray & msg,
  const rclcpp::Time & now,
  double target_distance,
  double tag_lost_timeout,
  double tag_pitch_x,
  double camera_fx,
  double camera_cx,
  double camera_offset_x,
  double camera_offset_y,
  double camera_offset_z)
{
  // 1. 各検出タグのピクセル座標を記録
  for (const auto & detection : msg.detections) {
    const int id = detection.id;
    auto it = panel_grid_.find(id);
    if (it != panel_grid_.end()) {
      it->second.last_seen = now;
      it->second.detected = true;
      it->second.pixel_x = static_cast<double>(detection.centre.x);
      it->second.pixel_y = static_cast<double>(detection.centre.y);
    }
  }

  // 2. 既知のTagピッチを用いたリアルタイム実距離(Z)三角測量推定
  double estimated_z = target_distance;
  double z_sum = 0.0;
  int z_count = 0;

  for (int r = 0; r <= 2; ++r) {
    std::vector<const PanelTagInfo *> row_detected;
    for (const auto & [id, panel] : panel_grid_) {
      if (panel.row == r && panel.detected &&
        (now - panel.last_seen).seconds() <= tag_lost_timeout)
      {
        row_detected.push_back(&panel);
      }
    }

    if (row_detected.size() >= 2) {
      for (size_t i = 0; i < row_detected.size(); ++i) {
        for (size_t j = i + 1; j < row_detected.size(); ++j) {
          const double delta_col = std::abs(row_detected[i]->col - row_detected[j]->col);
          const double delta_pixel_x =
            std::abs(row_detected[i]->pixel_x - row_detected[j]->pixel_x);
          if (delta_col > 0 && delta_pixel_x > 10.0) {
            const double real_dx = delta_col * tag_pitch_x;
            const double z_est = (camera_fx * real_dx) / delta_pixel_x;
            if (z_est >= 2.0 && z_est <= 6.0) {
              z_sum += z_est;
              z_count++;
            }
          }
        }
      }
    }
  }

  if (z_count > 0) {
    estimated_z = z_sum / static_cast<double>(z_count);
  }
  estimated_z_ = estimated_z;

  // 3. 各タグの base_link 3D座標 (x, y, z) を精密計算 (後方 -X 空間)
  for (auto & [id, panel] : panel_grid_) {
    if (panel.detected) {
      const double z_cam = estimated_z;
      const double y_offset_from_cam = -(panel.pixel_x - camera_cx) * z_cam / camera_fx;
      panel.x = camera_offset_x - z_cam;
      panel.y = camera_offset_y + y_offset_from_cam;
      panel.z = camera_offset_z;
    }
  }
}

void VisionTracker::timeout_unseen_tags(const rclcpp::Time & now, double tag_lost_timeout)
{
  for (auto & [id, panel] : panel_grid_) {
    if (panel.detected && (now - panel.last_seen).seconds() > tag_lost_timeout) {
      panel.detected = false;
    }
  }
}

}  // namespace robot_controller
