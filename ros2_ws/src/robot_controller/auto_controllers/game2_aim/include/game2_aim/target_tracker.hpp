#ifndef GAME2_AIM__TARGET_TRACKER_HPP_
#define GAME2_AIM__TARGET_TRACKER_HPP_

#include <cmath>
#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace robot_controller
{

struct PanelTagInfo
{
  int tag_id{0};
  int row{0};     // 0: Bottom, 1: Middle, 2: Top
  int col{0};     // 0: Left, 1: Center, 2: Right
  bool detected{false};
  bool is_standing{true};
  double aspect_ratio{1.0};
  double tilt_deg{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double pixel_x{0.0};
  double pixel_y{0.0};
  double yaw_at_detection{0.0};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  rclcpp::Time shot_cooldown_until{0, 0, RCL_ROS_TIME};
  double initial_standing_z{0.0};
  bool has_initial_z{false};
};

class TargetTracker
{
public:
  struct Config
  {
    std::string tag_prefix{"tag16h5:"};
    std::string base_frame{"base_link"};
    double target_distance{4.0};
    double camera_offset_x{0.265};
    double camera_offset_y{0.035};
    double camera_offset_z{0.193};
    double camera_image_width{1920.0};
    double camera_image_height{1080.0};
    double camera_fx{800.0};
    double camera_fy{800.0};
    double camera_cx{960.0};
    double camera_cy{540.0};
    double tag_lost_timeout{1.5};
    bool enable_double_panel_midpoint_targeting{true};
    bool test_alignment_only{false};
    int min_detection_frames{2};
    double aim_yaw_offset_rad{0.0};
    double min_standing_aspect_ratio{0.80};
    double max_standing_tilt_deg{15.0};    // [deg] 倒れ判定最大傾き角 (15度以上傾いたら即座に倒れ)
    double max_standing_height_drop{0.07}; // [m] 倒れ判定の高さ落下量 (7cm以上落下で倒れ)
    double shot_target_cooldown_sec{2.0};
    double midpoint_blend_ratio{0.65}; // 端列(Col 0/2)へのシフト比率 (0.5: 真ん中, 0.65: 端寄り)
  };

  TargetTracker() = default;

  void set_config(const Config & config) {config_ = config;}
  const Config & config() const {return config_;}

  void init_panel_grid(
    const std::vector<int64_t> & bottom_tags,
    const std::vector<int64_t> & middle_tags,
    const std::vector<int64_t> & top_tags)
  {
    panel_grid_.clear();
    for (size_t col = 0; col < bottom_tags.size(); ++col) {
      panel_grid_[static_cast<int>(bottom_tags[col])] =
      {static_cast<int>(bottom_tags[col]), 0, static_cast<int>(col)};
    }
    for (size_t col = 0; col < middle_tags.size(); ++col) {
      panel_grid_[static_cast<int>(middle_tags[col])] =
      {static_cast<int>(middle_tags[col]), 1, static_cast<int>(col)};
    }
    for (size_t col = 0; col < top_tags.size(); ++col) {
      panel_grid_[static_cast<int>(top_tags[col])] =
      {static_cast<int>(top_tags[col]), 2, static_cast<int>(col)};
    }
  }

  void update_camera_info(const sensor_msgs::msg::CameraInfo & msg)
  {
    if (msg.k[0] > 1.0) {
      config_.camera_fx = msg.k[0];
      config_.camera_fy = msg.k[4];
      config_.camera_cx = msg.k[2];
      config_.camera_cy = msg.k[5];
    }
    if (msg.width > 0 && msg.height > 0) {
      config_.camera_image_width = static_cast<double>(msg.width);
      config_.camera_image_height = static_cast<double>(msg.height);
    }
  }

  void update_detections(
    const apriltag_msgs::msg::AprilTagDetectionArray & msg,
    const tf2_ros::Buffer & tf_buffer,
    double current_yaw,
    const rclcpp::Time & now)
  {
    geometry_msgs::msg::TransformStamped tf_optical_to_base;
    bool has_tf = false;
    if (!msg.header.frame_id.empty()) {
      try {
        tf_optical_to_base = tf_buffer.lookupTransform(
          config_.base_frame, msg.header.frame_id, tf2::TimePointZero);
        has_tf = true;
      } catch (const tf2::TransformException &) {
        has_tf = false;
      }
    }

    for (const auto & det : msg.detections) {
      int tag_id = det.id;
      if (tag_id < 0 && !det.family.empty()) {
        try {
          if (det.family.rfind(config_.tag_prefix, 0) == 0) {
            tag_id = std::stoi(det.family.substr(config_.tag_prefix.length()));
          }
        } catch (...) {
          continue;
        }
      }

      auto it = panel_grid_.find(tag_id);
      if (it == panel_grid_.end()) {
        continue;
      }

      auto & panel = it->second;
      panel.last_seen = now;
      panel.pixel_x = det.centre.x;
      panel.pixel_y = det.centre.y;
      panel.yaw_at_detection = current_yaw;

      // ── 📐 アスペクト比 (縦幅 / 横幅) の算出 ──
      double aspect_ratio = 1.0;
      if (det.corners.size() >= 4) {
        const double w1 = std::hypot(det.corners[1].x - det.corners[0].x,
            det.corners[1].y - det.corners[0].y);
        const double w2 = std::hypot(det.corners[2].x - det.corners[3].x,
            det.corners[2].y - det.corners[3].y);
        const double h1 = std::hypot(det.corners[3].x - det.corners[0].x,
            det.corners[3].y - det.corners[0].y);
        const double h2 = std::hypot(det.corners[2].x - det.corners[1].x,
            det.corners[2].y - det.corners[1].y);
        const double w_avg = (w1 + w2) * 0.5;
        const double h_avg = (h1 + h2) * 0.5;
        if (w_avg > 1.0) {
          aspect_ratio = h_avg / w_avg;
        }
      }
      panel.aspect_ratio = aspect_ratio;

      // ── 📐 3D Tilt 角 (法線の傾き) の算出 (Homography 行列またはアスペクト比) ──
      double tilt_deg = 0.0;
      if (det.homography.size() == 9 && config_.camera_fx > 1.0 && config_.camera_fy > 1.0) {
        // H = K * [r1 r2 t] => r1 = K^-1 * H[:,0], r2 = K^-1 * H[:,1]
        const double r1_x = (det.homography[0] - config_.camera_cx * det.homography[6]) /
          config_.camera_fx;
        const double r1_y = (det.homography[3] - config_.camera_cy * det.homography[6]) /
          config_.camera_fy;
        const double r1_z = det.homography[6];

        const double r2_x = (det.homography[1] - config_.camera_cx * det.homography[7]) /
          config_.camera_fx;
        const double r2_y = (det.homography[4] - config_.camera_cy * det.homography[7]) /
          config_.camera_fy;
        const double r2_z = det.homography[7];

        // 法線ベクトル n = r1 x r2
        const double nx = r1_y * r2_z - r1_z * r2_y;
        const double ny = r1_z * r2_x - r1_x * r2_z;
        const double nz = r1_x * r2_y - r1_y * r2_x;
        const double norm = std::hypot(nx, ny, nz);
        if (norm > 1e-6) {
          const double cos_tilt = std::clamp(std::abs(nz) / norm, 0.0, 1.0);
          tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;
        }
      } else {
        // フォールバック: アスペクト比からの傾き推定
        const double cos_tilt = std::clamp(aspect_ratio, 0.0, 1.0);
        tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;
      }
      panel.tilt_deg = tilt_deg;

      const double x_c = (det.centre.x - config_.camera_cx) * config_.target_distance /
        config_.camera_fx;
      const double y_c = (det.centre.y - config_.camera_cy) * config_.target_distance /
        config_.camera_fy;
      const double z_c = config_.target_distance;

      if (has_tf) {
        geometry_msgs::msg::PoseStamped tag_cam_pose;
        tag_cam_pose.header = msg.header;
        tag_cam_pose.pose.position.x = x_c;
        tag_cam_pose.pose.position.y = y_c;
        tag_cam_pose.pose.position.z = z_c;
        tag_cam_pose.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped tag_base_pose;
        tf2::doTransform(tag_cam_pose, tag_base_pose, tf_optical_to_base);
        panel.x = tag_base_pose.pose.position.x;
        panel.y = tag_base_pose.pose.position.y;
        panel.z = tag_base_pose.pose.position.z;
      } else {
        // Optical to Base Link transform fallback
        panel.x = z_c + config_.camera_offset_x;
        panel.y = -x_c + config_.camera_offset_y;
        panel.z = -y_c + config_.camera_offset_z;
      }

      // 初回直立時の基準高さを記憶
      if (!panel.has_initial_z && tilt_deg < config_.max_standing_tilt_deg &&
        aspect_ratio >= config_.min_standing_aspect_ratio) {
        panel.initial_standing_z = panel.z;
        panel.has_initial_z = true;
      }

      panel.is_standing = true;
      panel.detected = true;
    }
  }

  void update_panel_states(const rclcpp::Time & now)
  {
    for (auto & [id, panel] : panel_grid_) {
      if ((now - panel.last_seen).seconds() > config_.tag_lost_timeout) {
        panel.detected = false;
      }
    }
  }

  bool find_and_lock_target(const rclcpp::Time & now, const rclcpp::Logger & logger)
  {
    update_panel_states(now);

    if (config_.test_alignment_only) {
      int best_id = -1;
      double min_heading_err_abs = 1e9;
      for (const auto & [id, panel] : panel_grid_) {
        if (panel.detected) {
          const double heading_err = std::atan2(panel.y, panel.x);
          if (std::abs(heading_err) < min_heading_err_abs) {
            min_heading_err_abs = std::abs(heading_err);
            best_id = id;
          }
        }
      }

      if (best_id != -1) {
        if (candidate_pattern_key_ == best_id) {
          consecutive_detection_count_++;
        } else {
          candidate_pattern_key_ = best_id;
          consecutive_detection_count_ = 1;
        }

        if (consecutive_detection_count_ >= config_.min_detection_frames) {
          target_locked_ = true;
          is_midpoint_target_ = false;
          current_target_tag_ids_ = {best_id};
          active_target_id_ = best_id;
          active_row_ = panel_grid_[best_id].row;
          target_belt_mode_ = get_target_belt_mode(active_row_);
          target_x_ = panel_grid_[best_id].x;
          target_y_ = panel_grid_[best_id].y;
          target_z_ = panel_grid_[best_id].z;
          target_yaw_at_detection_ = panel_grid_[best_id].yaw_at_detection;
          target_tag_offset_x_ = 0.0;
          target_tag_offset_y_ = 0.0;
          target_heading_err_ = std::remainder(std::atan2(target_y_,
              target_x_) + config_.aim_yaw_offset_rad,
              2.0 * M_PI);
          last_visually_confirmed_time_ = now;

          RCLCPP_INFO(
            logger,
            "🎯 [Game2 TEST Target Confirmed] Tag #%d (Row %d) | Err: %+.2f deg",
            best_id, active_row_, target_heading_err_ * 180.0 / M_PI);
          return true;
        }
      } else {
        consecutive_detection_count_ = 0;
        candidate_pattern_key_ = -1;
      }
      return false;
    }

    // ── 🎯 Column優先ターゲットパターン定義 (5段階) ──
    enum class TargetPattern
    {
      MIDPOINT_0_1 = 0,
      MIDPOINT_1_2 = 1,
      SINGLE_COL_1 = 2,
      SINGLE_COL_0 = 3,
      SINGLE_COL_2 = 4
    };

    const std::vector<TargetPattern> patterns = {
      TargetPattern::MIDPOINT_0_1,
      TargetPattern::MIDPOINT_1_2,
      TargetPattern::SINGLE_COL_1,
      TargetPattern::SINGLE_COL_0,
      TargetPattern::SINGLE_COL_2
    };

    for (const auto pattern : patterns) {
      if ((pattern == TargetPattern::MIDPOINT_0_1 || pattern == TargetPattern::MIDPOINT_1_2) &&
        !config_.enable_double_panel_midpoint_targeting)
      {
        continue;
      }

      for (int row = 2; row >= 0; --row) {
        const PanelTagInfo * p0 = nullptr;
        const PanelTagInfo * p1 = nullptr;
        const PanelTagInfo * p2 = nullptr;

        for (const auto & [id, panel] : panel_grid_) {
          if (panel.row == row) {
            if (panel.col == 0) {p0 = &panel;} else if (panel.col == 1) {p1 = &panel;
            } else if (panel.col == 2) {p2 = &panel;}
          }
        }

        auto is_tag_active = [&](const PanelTagInfo * p) -> bool {
            if (!p) {return false;}
            if (!p->detected || !p->is_standing) {return false;}
            if (now < p->shot_cooldown_until) {return false;}
            return true;
          };

        bool match = false;
        int pattern_key = static_cast<int>(pattern) * 10 + row;

        if (pattern == TargetPattern::MIDPOINT_0_1 && is_tag_active(p0) && is_tag_active(p1)) {
          match = true;
        } else if (pattern == TargetPattern::MIDPOINT_1_2 && is_tag_active(p1) &&
          is_tag_active(p2)) {
          match = true;
        } else if (pattern == TargetPattern::SINGLE_COL_0 && is_tag_active(p0)) {
          match = true;
        } else if (pattern == TargetPattern::SINGLE_COL_1 && is_tag_active(p1)) {
          match = true;
        } else if (pattern == TargetPattern::SINGLE_COL_2 && is_tag_active(p2)) {
          match = true;
        }

        if (match) {
          if (candidate_pattern_key_ == pattern_key) {
            consecutive_detection_count_++;
          } else {
            candidate_pattern_key_ = pattern_key;
            consecutive_detection_count_ = 1;
          }

          if (consecutive_detection_count_ >= config_.min_detection_frames) {
            target_locked_ = true;
            active_row_ = row;
            target_belt_mode_ = get_target_belt_mode(row);
            last_visually_confirmed_time_ = now;

            if (pattern == TargetPattern::MIDPOINT_0_1) {
              is_midpoint_target_ = true;
              current_target_tag_ids_ = {p0->tag_id, p1->tag_id};
              active_target_id_ = p0->tag_id;
              const double r = config_.midpoint_blend_ratio;
              target_x_ = p0->x * r + p1->x * (1.0 - r);
              target_y_ = p0->y * r + p1->y * (1.0 - r);
              target_z_ = p0->z * r + p1->z * (1.0 - r);
              target_yaw_at_detection_ = p0->yaw_at_detection * r + p1->yaw_at_detection *
                (1.0 - r);
              target_tag_offset_x_ = 0.0;
              target_tag_offset_y_ = 0.0;
              target_heading_err_ = std::remainder(std::atan2(target_y_,
                  target_x_) + config_.aim_yaw_offset_rad,
                  2.0 * M_PI);
              RCLCPP_INFO(
                logger,
                "🔒 [Target Confirmed: Col 0-1 Midpoint | %s] Tags #%d & #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
                get_row_name(
                  row).c_str(), p0->tag_id, p1->tag_id, target_heading_err_ * 180.0 / M_PI,
                target_belt_mode_);
            } else if (pattern == TargetPattern::MIDPOINT_1_2) {
              is_midpoint_target_ = true;
              current_target_tag_ids_ = {p2->tag_id, p1->tag_id};
              active_target_id_ = p2->tag_id;
              const double r = config_.midpoint_blend_ratio;
              target_x_ = p2->x * r + p1->x * (1.0 - r);
              target_y_ = p2->y * r + p1->y * (1.0 - r);
              target_z_ = p2->z * r + p1->z * (1.0 - r);
              target_yaw_at_detection_ = p2->yaw_at_detection * r + p1->yaw_at_detection *
                (1.0 - r);
              target_tag_offset_x_ = 0.0;
              target_tag_offset_y_ = 0.0;
              target_heading_err_ = std::remainder(std::atan2(target_y_,
                  target_x_) + config_.aim_yaw_offset_rad,
                  2.0 * M_PI);
              RCLCPP_INFO(
                logger,
                "🔒 [Target Confirmed: Col 1-2 Midpoint | %s] Tags #%d & #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
                get_row_name(
                  row).c_str(), p2->tag_id, p1->tag_id, target_heading_err_ * 180.0 / M_PI,
                target_belt_mode_);
            } else {
              const PanelTagInfo * p = (pattern == TargetPattern::SINGLE_COL_0) ? p0 :
                ((pattern == TargetPattern::SINGLE_COL_1) ? p1 : p2);
              is_midpoint_target_ = false;
              current_target_tag_ids_ = {p->tag_id};
              active_target_id_ = p->tag_id;
              target_x_ = p->x;
              target_y_ = p->y;
              target_z_ = p->z;
              target_yaw_at_detection_ = p->yaw_at_detection;
              target_tag_offset_x_ = 0.0;
              target_tag_offset_y_ = 0.0;
              target_heading_err_ = std::remainder(std::atan2(target_y_,
                  target_x_) + config_.aim_yaw_offset_rad,
                  2.0 * M_PI);
            }

            // 全グリッド認識状況のサマリー文字列を構築
            std::stringstream status_ss;
            for (int r = 2; r >= 0; --r) {
              const char * rname = (r == 2) ? "上" : ((r == 1) ? "中" : "下");
              status_ss << "[" << rname << ":";
              for (int c = 0; c < 3; ++c) {
                const PanelTagInfo * pt = nullptr;
                for (const auto & [id, p_info] : panel_grid_) {
                  if (p_info.row == r && p_info.col == c) {
                    pt = &p_info;
                    break;
                  }
                }
                if (!pt) {
                  status_ss << " ?";
                } else if (!pt->detected) {
                  status_ss << " #" << pt->tag_id << "(未)";
                } else if (pt->is_standing) {
                  status_ss << " #" << pt->tag_id << "(立)";
                } else {
                  status_ss << " #" << pt->tag_id << "(倒" << std::fixed << std::setprecision(0) <<
                      pt->tilt_deg << "°)";
                }
              }
              status_ss << "] ";
            }

            RCLCPP_INFO(
              logger,
              "🔒 [Game2 TARGET LOCK 決定] %s\n"
              "   ▶ 9枚認識状況: %s\n"
              "   ▶ 狙い角度: %+.2f deg | ベルト: LEVEL_%d",
              target_description().c_str(),
              status_ss.str().c_str(),
              target_heading_err_ * 180.0 / M_PI,
              target_belt_mode_);

            return true;
          }
          return false;
        }
      }
    }

    consecutive_detection_count_ = 0;
    candidate_pattern_key_ = -1;
    return false;
  }

  void update_tracking(double current_yaw, const rclcpp::Time & now)
  {
    if (!target_locked_ || current_target_tag_ids_.empty()) {
      return;
    }

    update_panel_states(now);
    bool visual_found = false;

    if (is_midpoint_target_ && current_target_tag_ids_.size() >= 2) {
      const int id_a = current_target_tag_ids_[0];
      const int id_b = current_target_tag_ids_[1];
      const auto it_a = panel_grid_.find(id_a);
      const auto it_b = panel_grid_.find(id_b);

      const bool a_detected =
        (it_a != panel_grid_.end() && it_a->second.detected && it_a->second.is_standing);
      const bool b_detected =
        (it_b != panel_grid_.end() && it_b->second.detected && it_b->second.is_standing);

      if (a_detected && b_detected) {
        const double r = config_.midpoint_blend_ratio;
        target_x_ = it_a->second.x * r + it_b->second.x * (1.0 - r);
        target_y_ = it_a->second.y * r + it_b->second.y * (1.0 - r);
        target_z_ = it_a->second.z * r + it_b->second.z * (1.0 - r);
        target_yaw_at_detection_ = it_a->second.yaw_at_detection * r +
          it_b->second.yaw_at_detection * (1.0 - r);
        visual_found = true;
      } else if (a_detected || b_detected) {
        // 片方でも見えていれば視覚ロストとは判定せず、前回の target_x_, target_y_ を維持して
        // IMU姿勢補間で滑らかに追従する（目標角度のジャンプを完全防止）
        visual_found = true;
      }
    } else {
      const int id = current_target_tag_ids_[0];
      const auto it = panel_grid_.find(id);
      if (it != panel_grid_.end() && it->second.detected && it->second.is_standing) {
        target_x_ = it->second.x;
        target_y_ = it->second.y;
        target_z_ = it->second.z;
        target_yaw_at_detection_ = it->second.yaw_at_detection;
        visual_found = true;
      }
    }

    if (visual_found) {
      last_visually_confirmed_time_ = now;
    }

    // IMUオドメトリ姿勢補間（デッドレコニング）
    const double raw_heading_err = std::atan2(target_y_, target_x_);
    const double rotated = std::remainder(current_yaw - target_yaw_at_detection_, 2.0 * M_PI);
    target_heading_err_ = std::remainder(raw_heading_err - rotated + config_.aim_yaw_offset_rad,
        2.0 * M_PI);
  }

  bool has_locked_target() const {return target_locked_;}

  bool is_currently_visible(const rclcpp::Time & now, double timeout_sec = 0.3) const
  {
    if (!target_locked_) {return false;}
    return (now - last_visually_confirmed_time_).seconds() <= timeout_sec;
  }

  bool is_lost_timeout(const rclcpp::Time & now, double timeout_sec = 1.0) const
  {
    if (!target_locked_) {return false;}
    return (now - last_visually_confirmed_time_).seconds() > timeout_sec;
  }

  double heading_error() const {return target_heading_err_;}
  uint8_t target_belt_mode() const {return target_belt_mode_;}
  int active_target_id() const {return active_target_id_;}
  int active_row() const {return active_row_;}
  bool is_midpoint_target() const {return is_midpoint_target_;}

  std::string target_description() const
  {
    if (!target_locked_) {return "No Target";}
    std::stringstream ss;
    if (is_midpoint_target_ && current_target_tag_ids_.size() >= 2) {
      ss << "2枚抜き中点: Tags #" << current_target_tag_ids_[0] << " & #" << current_target_tag_ids_[1]
         << " (" << get_row_name(active_row_) << ")";
    } else if (!current_target_tag_ids_.empty()) {
      ss << "単体真ん中: Tag #" << current_target_tag_ids_[0]
         << " (" << get_row_name(active_row_) << ")";
    }
    return ss.str();
  }

  std::string get_grid_visual_summary(const rclcpp::Time & now) const
  {
    std::stringstream ss;
    ss << "\n═══════════════ 🎯 9マス起立検出モニター ═══════════════\n";
    for (int r = 2; r >= 0; --r) {
      const char * rname = (r == 2) ? "[上段]" : ((r == 1) ? "[中段]" : "[下段]");
      ss << " " << rname << " │";
      for (int c = 0; c < 3; ++c) {
        const PanelTagInfo * pt = nullptr;
        for (const auto & [id, p_info] : panel_grid_) {
          if (p_info.row == r && p_info.col == c) {
            pt = &p_info;
            break;
          }
        }
        if (!pt) {
          ss << "  ??? 🔴 ( --)  │";
          continue;
        }

        const double dt = (now - pt->last_seen).seconds();
        const bool is_recent_rx =
          (pt->last_seen.nanoseconds() > 0 && dt <= config_.tag_lost_timeout);

        char buf[32];
        if (is_recent_rx) {
          std::snprintf(buf, sizeof(buf), "  #%2d 🟢 (%2.0f°)  │", pt->tag_id, pt->tilt_deg);
        } else {
          std::snprintf(buf, sizeof(buf), "  #%2d 🔴 ( --)  │", pt->tag_id);
        }
        ss << buf;
      }
      ss << "\n";
    }
    ss << " 凡例: 🟢 狙える (立/検出中)    🔴 狙わない (倒れ/未検出--)\n"
       << "════════════════════════════════════════════════════════";
    return ss.str();
  }

  void mark_active_target_shot(const rclcpp::Time & now)
  {
    for (int tag_id : current_target_tag_ids_) {
      auto it = panel_grid_.find(tag_id);
      if (it != panel_grid_.end()) {
        it->second.shot_cooldown_until = now + rclcpp::Duration::from_seconds(
          config_.shot_target_cooldown_sec);
        it->second.detected = false;
      }
    }
  }

  void clear_target()
  {
    target_locked_ = false;
    current_target_tag_ids_.clear();
    active_target_id_ = -1;
    target_belt_mode_ = robot_msgs::msg::BeltMode::STOP;
    target_heading_err_ = 0.0;
    candidate_pattern_key_ = -1;
    consecutive_detection_count_ = 0;
  }

  void reset_fallen_states()
  {
    for (auto & [id, panel] : panel_grid_) {
      panel.is_standing = true;
      panel.has_initial_z = false;
      panel.initial_standing_z = 0.0;
    }
  }

  static uint8_t get_target_belt_mode(int row)
  {
    switch (row) {
      case 0: return robot_msgs::msg::BeltMode::LEVEL_1;
      case 1: return robot_msgs::msg::BeltMode::LEVEL_2;
      case 2: return robot_msgs::msg::BeltMode::LEVEL_3;
      default: return robot_msgs::msg::BeltMode::LEVEL_1;
    }
  }

  static std::string get_row_name(int row)
  {
    switch (row) {
      case 0: return "Bottom (Row 0)";
      case 1: return "Middle (Row 1)";
      case 2: return "Top (Row 2)";
      default: return "Unknown Row";
    }
  }

  std::string get_detection_summary(const rclcpp::Time & now) const
  {
    std::stringstream ss;
    bool found_any = false;
    for (const auto & [id, panel] : panel_grid_) {
      if ((now - panel.last_seen).seconds() <= config_.tag_lost_timeout) {
        if (found_any) {ss << " | ";}
        ss << "#" << id << "(R" << panel.row << "C" << panel.col << ":"
           << (panel.is_standing ? "STAND" : "FALLEN")
           << " Y=" << std::showpos << std::fixed << std::setprecision(2) << panel.y
           << " Z=" << std::noshowpos << std::setprecision(2) << panel.z
           << " Tilt=" << std::setprecision(0) << panel.tilt_deg << "°)";
        found_any = true;
      }
    }
    if (!found_any) {return "No tags in view";}
    return ss.str();
  }

private:
  Config config_;
  std::unordered_map<int, PanelTagInfo> panel_grid_;

  bool target_locked_{false};
  std::vector<int> current_target_tag_ids_;
  bool is_midpoint_target_{false};
  int active_target_id_{-1};
  int active_row_{2};
  uint8_t target_belt_mode_{robot_msgs::msg::BeltMode::STOP};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_yaw_at_detection_{0.0};
  double target_tag_offset_x_{0.0};
  double target_tag_offset_y_{0.0};
  double target_heading_err_{0.0};

  rclcpp::Time last_visually_confirmed_time_{0, 0, RCL_ROS_TIME};
  int candidate_pattern_key_{-1};
  int consecutive_detection_count_{0};
};

}  // namespace robot_controller

#endif  // GAME2_AIM__TARGET_TRACKER_HPP_
