#ifndef GAME2_AIM__PK_TARGET_TRACKER_HPP_
#define GAME2_AIM__PK_TARGET_TRACKER_HPP_

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
#include "robot_msgs/msg/target_grid_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace robot_controller
{

struct PKPanelInfo
{
  int index{0};   // 0 to 8
  int tag_id{0};  // e.g. 14 to 22
  int row{0};     // 0: Bottom, 1: Middle, 2: Top
  int col{0};     // 0: Left, 1: Center, 2: Right
  std::string name{""};
  uint8_t belt_mode{robot_msgs::msg::BeltMode::STOP};

  bool detected{false};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double pixel_x{0.0};
  double pixel_y{0.0};
  double yaw_at_detection{0.0};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

class PKTargetTracker
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
    double aim_yaw_offset_rad{0.0};
  };

  PKTargetTracker()
  {
    init_default_grid();
  }

  void set_config(const Config & config) {config_ = config;}
  const Config & config() const {return config_;}

  void init_default_grid()
  {
    // 9つの的 (3x3 グリッド)
    // Row 2: 上段 (Tags 14, 15, 16) -> Belt: LEVEL_3
    // Row 1: 中段 (Tags 17, 18, 19) -> Belt: LEVEL_2
    // Row 0: 下段 (Tags 20, 21, 22) -> Belt: LEVEL_1
    panel_grid_.clear();
    index_to_tag_id_.resize(9);

    auto add_panel =
      [this](int idx, int tag_id, int row, int col, const std::string & name, uint8_t mode) {
        PKPanelInfo p;
        p.index = idx;
        p.tag_id = tag_id;
        p.row = row;
        p.col = col;
        p.name = name;
        p.belt_mode = mode;
        panel_grid_[tag_id] = p;
        index_to_tag_id_[idx] = tag_id;
      };

    // 上段 (Row 2, LEVEL_3)
    add_panel(0, 14, 2, 0, "上段左 [Top-Left]", robot_msgs::msg::BeltMode::LEVEL_3);
    add_panel(1, 15, 2, 1, "上段中 [Top-Center]", robot_msgs::msg::BeltMode::LEVEL_3);
    add_panel(2, 16, 2, 2, "上段右 [Top-Right]", robot_msgs::msg::BeltMode::LEVEL_3);

    // 中段 (Row 1, LEVEL_2)
    add_panel(3, 17, 1, 0, "中段左 [Mid-Left]", robot_msgs::msg::BeltMode::LEVEL_2);
    add_panel(4, 18, 1, 1, "中段中 [Mid-Center]", robot_msgs::msg::BeltMode::LEVEL_2);
    add_panel(5, 19, 1, 2, "中段右 [Mid-Right]", robot_msgs::msg::BeltMode::LEVEL_2);

    // 下段 (Row 0, LEVEL_1)
    add_panel(6, 20, 0, 0, "下段左 [Bot-Left]", robot_msgs::msg::BeltMode::LEVEL_1);
    add_panel(7, 21, 0, 1, "下段中 [Bot-Center]", robot_msgs::msg::BeltMode::LEVEL_1);
    add_panel(8, 22, 0, 2, "下段右 [Bot-Right]", robot_msgs::msg::BeltMode::LEVEL_1);

    selected_index_ = 4;
  }

  void init_custom_tags(
    const std::vector<int64_t> & bottom_tags,
    const std::vector<int64_t> & middle_tags,
    const std::vector<int64_t> & top_tags)
  {
    if (top_tags.size() == 3 && middle_tags.size() == 3 && bottom_tags.size() == 3) {
      panel_grid_.clear();
      index_to_tag_id_.resize(9);

      auto add_panel =
        [this](int idx, int tag_id, int row, int col, const std::string & name, uint8_t mode) {
          PKPanelInfo p;
          p.index = idx;
          p.tag_id = tag_id;
          p.row = row;
          p.col = col;
          p.name = name;
          p.belt_mode = mode;
          panel_grid_[tag_id] = p;
          index_to_tag_id_[idx] = tag_id;
        };

      add_panel(0, static_cast<int>(top_tags[0]), 2, 0, "上段左", robot_msgs::msg::BeltMode::LEVEL_3);
      add_panel(1, static_cast<int>(top_tags[1]), 2, 1, "上段中", robot_msgs::msg::BeltMode::LEVEL_3);
      add_panel(2, static_cast<int>(top_tags[2]), 2, 2, "上段右", robot_msgs::msg::BeltMode::LEVEL_3);

      add_panel(
        3, static_cast<int>(middle_tags[0]), 1, 0, "中段左",
        robot_msgs::msg::BeltMode::LEVEL_2);
      add_panel(
        4, static_cast<int>(middle_tags[1]), 1, 1, "中段中",
        robot_msgs::msg::BeltMode::LEVEL_2);
      add_panel(
        5, static_cast<int>(middle_tags[2]), 1, 2, "中段右",
        robot_msgs::msg::BeltMode::LEVEL_2);

      add_panel(
        6, static_cast<int>(bottom_tags[0]), 0, 0, "下段左",
        robot_msgs::msg::BeltMode::LEVEL_1);
      add_panel(
        7, static_cast<int>(bottom_tags[1]), 0, 1, "下段中",
        robot_msgs::msg::BeltMode::LEVEL_1);
      add_panel(
        8, static_cast<int>(bottom_tags[2]), 0, 2, "下段右",
        robot_msgs::msg::BeltMode::LEVEL_1);
    }
  }

  // ── インデックス選択操作 ──
  int selected_index() const {return selected_index_;}

  void set_selected_index(int idx)
  {
    selected_index_ = std::clamp(idx, 0, 8);
  }

  int select_next()
  {
    selected_index_ = (selected_index_ + 1) % 9;
    return selected_index_;
  }

  int select_prev()
  {
    selected_index_ = (selected_index_ - 1 + 9) % 9;
    return selected_index_;
  }

  const PKPanelInfo & get_selected_panel() const
  {
    int tag_id = index_to_tag_id_[selected_index_];
    return panel_grid_.at(tag_id);
  }

  const std::unordered_map<int, PKPanelInfo> & panel_grid() const {return panel_grid_;}

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
      panel.detected = true;
      panel.yaw_at_detection = current_yaw;

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
        panel.x = z_c + config_.camera_offset_x;
        panel.y = -x_c + config_.camera_offset_y;
        panel.z = -y_c + config_.camera_offset_z;
      }
    }
  }

  // ── 照準ターゲットの確定・ロック ──
  bool lock_selected_target(
    const rclcpp::Time & now, const rclcpp::Logger & logger, double current_yaw = 0.0)
  {
    int target_tag_id = index_to_tag_id_[selected_index_];
    auto it = panel_grid_.find(target_tag_id);
    if (it == panel_grid_.end()) {
      return false;
    }

    const auto & panel = it->second;
    active_target_id_ = panel.tag_id;
    active_row_ = panel.row;
    target_belt_mode_ = panel.belt_mode;
    target_locked_ = true;

    if (panel.detected && (now - panel.last_seen).seconds() <= config_.tag_lost_timeout) {
      target_x_ = panel.x;
      target_y_ = panel.y;
      target_z_ = panel.z;
      target_yaw_at_detection_ = panel.yaw_at_detection;
      last_visually_confirmed_time_ = now;
    } else {
      // 未検出時は幾何学的グリッド配置から推定
      // Col 0: +0.35m, Col 1: 0.0m, Col 2: -0.35m
      const double col_offsets[3] = {0.35, 0.0, -0.35};
      target_x_ = config_.target_distance + config_.camera_offset_x;
      target_y_ = col_offsets[panel.col] + config_.camera_offset_y;
      target_z_ = 0.5;
      target_yaw_at_detection_ = current_yaw;
    }

    // 目標の絶対角度 (ワールド座標系) を確定固定
    const double raw_aim_angle = std::atan2(target_y_, target_x_) + config_.aim_yaw_offset_rad;
    locked_target_yaw_ = std::remainder(current_yaw + raw_aim_angle, 2.0 * M_PI);
    target_heading_err_ = std::remainder(locked_target_yaw_ - current_yaw, 2.0 * M_PI);

    RCLCPP_INFO(
      logger,
      "🔒 [PK TARGET LOCKED] [Idx %d] %s (Tag #%d | Row %d Col %d) | Target Angle: %+.2f deg (World: %+.2f deg) | Belt: LEVEL_%d",
      selected_index_, panel.name.c_str(), panel.tag_id, panel.row, panel.col,
      target_heading_err_ * 180.0 / M_PI, locked_target_yaw_ * 180.0 / M_PI, target_belt_mode_);

    return true;
  }

  void update_tracking(double current_yaw, const rclcpp::Time & now)
  {
    if (!target_locked_) {
      return;
    }

    int target_tag_id = index_to_tag_id_[selected_index_];
    auto it = panel_grid_.find(target_tag_id);
    bool visual_found = false;

    if (it != panel_grid_.end()) {
      const auto & panel = it->second;
      if (panel.detected && (now - panel.last_seen).seconds() <= config_.tag_lost_timeout) {
        target_x_ = panel.x;
        target_y_ = panel.y;
        target_z_ = panel.z;
        target_yaw_at_detection_ = panel.yaw_at_detection;
        visual_found = true;
      }
    }

    if (visual_found) {
      last_visually_confirmed_time_ = now;
    }

    // 旋回中・静止中ともに固定された絶対目標角度 locked_target_yaw_ と現在のIMU角度の引き算で完全安定追従
    target_heading_err_ = std::remainder(locked_target_yaw_ - current_yaw, 2.0 * M_PI);
  }

  bool is_currently_visible(const rclcpp::Time & now, double valid_timeout = 0.5) const
  {
    return (now - last_visually_confirmed_time_).seconds() <= valid_timeout;
  }

  bool is_lost_timeout(const rclcpp::Time & now, double timeout_sec = 3.0) const
  {
    if (!target_locked_) {return false;}
    return (now - last_visually_confirmed_time_).seconds() > timeout_sec;
  }

  void reset()
  {
    target_locked_ = false;
    active_target_id_ = -1;
  }

  // Getters
  bool is_target_locked() const {return target_locked_;}
  int active_target_id() const {return active_target_id_;}
  int active_row() const {return active_row_;}
  uint8_t target_belt_mode() const {return target_belt_mode_;}
  std::vector<int> get_target_indices() const
  {
    if (!target_locked_) {
      return {};
    }
    return {selected_index_};
  }

  std::vector<int> get_fallen_indices(const rclcpp::Time & now) const
  {
    std::vector<int> fallen;
    for (int idx = 0; idx < 9; ++idx) {
      int tag_id = index_to_tag_id_[idx];
      auto it = panel_grid_.find(tag_id);
      if (it == panel_grid_.end()) {
        fallen.push_back(idx);
        continue;
      }
      const auto & p = it->second;
      const double dt = (now - p.last_seen).seconds();
      const bool is_active =
        (p.detected && p.last_seen.nanoseconds() > 0 && dt <= config_.tag_lost_timeout);
      if (!is_active) {
        fallen.push_back(idx);
      }
    }
    return fallen;
  }

  std::vector<int> get_standing_indices(const rclcpp::Time & now) const
  {
    std::vector<int> standing;
    for (int idx = 0; idx < 9; ++idx) {
      int tag_id = index_to_tag_id_[idx];
      auto it = panel_grid_.find(tag_id);
      if (it == panel_grid_.end()) {continue;}
      const auto & p = it->second;
      const double dt = (now - p.last_seen).seconds();
      const bool is_active =
        (p.detected && p.last_seen.nanoseconds() > 0 && dt <= config_.tag_lost_timeout);
      if (is_active) {
        standing.push_back(idx);
      }
    }
    return standing;
  }

  robot_msgs::msg::TargetGridState get_target_grid_state(const rclcpp::Time & now) const
  {
    robot_msgs::msg::TargetGridState msg;
    for (int r = 2; r >= 0; --r) {
      for (int c = 0; c < 3; ++c) {
        int idx = (2 - r) * 3 + c;
        const PKPanelInfo * pt = nullptr;
        for (const auto & [id, p_info] : panel_grid_) {
          if (p_info.row == r && p_info.col == c) {
            pt = &p_info;
            break;
          }
        }
        if (!pt) {
          msg.states[idx] = robot_msgs::msg::TargetGridState::FALLEN;
          continue;
        }

        const double dt = (now - pt->last_seen).seconds();
        const bool is_active =
          (pt->detected && (pt->last_seen.nanoseconds() > 0 && dt <= config_.tag_lost_timeout));

        if (!is_active) {
          msg.states[idx] = robot_msgs::msg::TargetGridState::FALLEN;
        } else if (target_locked_ && idx == selected_index_) {
          msg.states[idx] = robot_msgs::msg::TargetGridState::TARGET;
        } else {
          msg.states[idx] = robot_msgs::msg::TargetGridState::STANDING;
        }
      }
    }
    return msg;
  }

  double heading_error() const {return target_heading_err_;}
  double locked_target_yaw() const {return locked_target_yaw_;}
  double target_x() const {return target_x_;}
  double target_y() const {return target_y_;}
  double target_z() const {return target_z_;}

  std::string target_description() const
  {
    if (selected_index_ >= 0 && selected_index_ < 9) {
      int tag_id = index_to_tag_id_[selected_index_];
      auto it = panel_grid_.find(tag_id);
      if (it != panel_grid_.end()) {
        return it->second.name + " (#" + std::to_string(tag_id) + ")";
      }
    }
    return "Unknown Target";
  }

private:
  Config config_;
  std::unordered_map<int, PKPanelInfo> panel_grid_;
  std::vector<int> index_to_tag_id_;
  int selected_index_{4};

  bool target_locked_{false};
  int active_target_id_{-1};
  int active_row_{0};
  uint8_t target_belt_mode_{robot_msgs::msg::BeltMode::STOP};

  double target_x_{4.0};
  double target_y_{0.0};
  double target_z_{0.5};
  double target_heading_err_{0.0};
  double locked_target_yaw_{0.0};
  double target_yaw_at_detection_{0.0};
  rclcpp::Time last_visually_confirmed_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace robot_controller

#endif  // GAME2_AIM__PK_TARGET_TRACKER_HPP_
