#include "game2_aim/game2_aim_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game2AimNode::Game2AimNode(const rclcpp::NodeOptions & options)
: Node("game2_aim", options)
{
  load_parameters();

  const auto init_now = this->now();
  state_start_time_ = init_now;
  last_imu_time_ = init_now;
  last_loop_time_ = init_now;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto cmd_qos = rclcpp::QoS(10);
  const auto estop_qos = rclcpp::QoS(1).reliable().transient_local();

  // Subscriptions
  detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    detections_topic_, cmd_qos,
    std::bind(&Game2AimNode::tag_detections_callback, this, std::placeholders::_1));

  // /camera_info から実際のカメラ内部パラメータ行列 K を自動取得 (未受信時はYAML値で動作)
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/image_combine_raw/left/camera_info", cmd_qos,
    std::bind(&Game2AimNode::camera_info_callback, this, std::placeholders::_1));

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game2/command_start", cmd_qos,
    std::bind(&Game2AimNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2AimNode::imu_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", estop_qos,
    std::bind(&Game2AimNode::emergency_stop_callback, this, std::placeholders::_1));

  shot_cycle_state_sub_ = create_subscription<robot_msgs::msg::ShotCycleState>(
    "/dribble/shot_cycle_state", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Game2AimNode::shot_cycle_state_callback, this, std::placeholders::_1));

  shot_cycle_req_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", cmd_qos,
    std::bind(&Game2AimNode::shot_cycle_req_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, cmd_qos);
  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>("/belt/command_mode", cmd_qos);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", cmd_qos);
  arm_position_pub_ =
    create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", cmd_qos);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game2/completed", cmd_qos);
  state_pub_ = create_publisher<robot_msgs::msg::Game2State>(
    "/game2/state", rclcpp::QoS(1).reliable().transient_local());

  // Dynamic Parameter Callback
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&Game2AimNode::parameter_callback, this, std::placeholders::_1));

  // 20 Hz Control Loop Timer (50 ms)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2AimNode::control_loop, this));

  RCLCPP_INFO(
    get_logger(),
    "Game2AimNode initialized. Output CmdVel: %s, TestAlignmentOnly: %s",
    cmd_vel_topic_.c_str(), test_alignment_only_ ? "true" : "false");
}

void Game2AimNode::load_parameters()
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel_heading");
  detections_topic_ = declare_parameter<std::string>("detections_topic", "/detections");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");

  // Control gains & Limits
  kp_yaw_ = declare_parameter<double>("kp_yaw", 1.8);
  kd_yaw_ = declare_parameter<double>("kd_yaw", 0.12);
  min_angular_z_ = declare_parameter<double>("min_angular_z", 0.12);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.40);
  max_angular_accel_ = declare_parameter<double>("max_angular_accel", 2.5);
  target_distance_ = declare_parameter<double>("target_distance", 4.0);

  // Camera Physical & Optical Parameters
  camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.265);
  camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.035);
  camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.193);
  camera_image_width_ = declare_parameter<double>("camera_image_width", 1920.0);
  camera_image_height_ = declare_parameter<double>("camera_image_height", 1080.0);
  camera_fx_ = declare_parameter<double>("camera_fx", 800.0);
  camera_fy_ = declare_parameter<double>("camera_fy", 800.0);
  camera_cx_ = declare_parameter<double>("camera_cx", camera_image_width_ / 2.0);
  camera_cy_ = declare_parameter<double>("camera_cy", camera_image_height_ / 2.0);

  // Tolerances & Timings
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.030);
  dist_tolerance_ = declare_parameter<double>("dist_tolerance", 0.05);

  tag_lost_timeout_ = declare_parameter<double>("tag_lost_timeout", 1.5);
  search_angular_z_ = declare_parameter<double>("search_angular_z", 0.15);
  test_alignment_only_ = declare_parameter<bool>("test_alignment_only", false);
  enable_double_panel_midpoint_targeting_ =
    declare_parameter<bool>("enable_double_panel_midpoint_targeting", true);
  shot_fallback_timeout_ = declare_parameter<double>("shot_fallback_timeout", 2.0);

  // AprilTag Panel Configuration (Row 0: Bottom, 1: Middle, 2: Top)
  const std::vector<int64_t> default_bottom = {20, 21, 22};
  const std::vector<int64_t> default_middle = {17, 18, 19};
  const std::vector<int64_t> default_top = {14, 15, 16};
  const auto bottom_tags = declare_parameter<std::vector<int64_t>>("bottom_tags", default_bottom);
  const auto middle_tags = declare_parameter<std::vector<int64_t>>("middle_tags", default_middle);
  const auto top_tags = declare_parameter<std::vector<int64_t>>("top_tags", default_top);

  panel_grid_.clear();
  auto register_row = [this](const std::vector<int64_t> & tags, int row) {
      for (size_t col = 0; col < tags.size(); ++col) {
        const int id = static_cast<int>(tags[col]);
        PanelTagInfo info;
        info.tag_id = id;
        info.row = row;
        info.col = static_cast<int>(col);
        info.last_seen = this->now();
        panel_grid_[id] = info;
      }
    };
  register_row(bottom_tags, 0);
  register_row(middle_tags, 1);
  register_row(top_tags, 2);
}

rcl_interfaces::msg::SetParametersResult Game2AimNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "kp_yaw") {
      kp_yaw_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Param updated: kp_yaw = %.3f", kp_yaw_);
    } else if (name == "kd_yaw") {
      kd_yaw_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Param updated: kd_yaw = %.3f", kd_yaw_);
    } else if (name == "min_angular_z") {
      min_angular_z_ = param.as_double();
    } else if (name == "max_angular_z") {
      max_angular_z_ = param.as_double();
    } else if (name == "max_angular_accel") {
      max_angular_accel_ = param.as_double();
    } else if (name == "yaw_tolerance") {
      yaw_tolerance_ = param.as_double();
    } else if (name == "target_distance") {
      target_distance_ = param.as_double();
    } else if (name == "camera_fx") {
      camera_fx_ = param.as_double();
    } else if (name == "camera_offset_y") {
      camera_offset_y_ = param.as_double();
    } else if (name == "shot_fallback_timeout") {
      shot_fallback_timeout_ = param.as_double();
    } else if (name == "test_alignment_only") {
      test_alignment_only_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: test_alignment_only = %s",
        test_alignment_only_ ? "true" : "false");
    } else if (name == "enable_double_panel_midpoint_targeting") {
      enable_double_panel_midpoint_targeting_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: enable_double_panel_midpoint_targeting = %s",
        enable_double_panel_midpoint_targeting_ ? "true" : "false");
    }
  }

  return result;
}

void Game2AimNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (msg->k[0] > 10.0 && msg->k[4] > 10.0) {
    camera_fx_ = msg->k[0];
    camera_cx_ = msg->k[2];
    camera_fy_ = msg->k[4];
    camera_cy_ = msg->k[5];
    camera_image_width_ = static_cast<double>(msg->width);
    camera_image_height_ = static_cast<double>(msg->height);

    RCLCPP_INFO_ONCE(
      get_logger(),
      "📷 CameraInfo received! Calibrated intrinsics loaded: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f (Image: %dx%d)",
      camera_fx_, camera_fy_, camera_cx_, camera_cy_, msg->width, msg->height);
  }
}

void Game2AimNode::reset_sequence()
{
  active_row_ = 2;
  active_target_id_ = -1;
  target_locked_ = false;
  locked_is_midpoint_ = false;
  locked_target_tag_ids_.clear();
  current_target_tag_ids_.clear();
  target_belt_mode_ = robot_msgs::msg::BeltMode::STOP;
  target_valid_ = false;
  last_cmd_wz_ = 0.0;
  locked_target_x_ = 0.0;
  locked_target_y_ = 0.0;
  locked_target_z_ = 0.0;
  locked_yaw_at_detection_ = 0.0;
  locked_tag_offset_x_ = 0.0;
  locked_tag_offset_y_ = 0.0;
  shot_requested_ = false;
  prev_shot_cycle_state_ = robot_msgs::msg::ShotCycleState::IDLE;

  for (auto & [id, panel] : panel_grid_) {
    panel.detected = false;
  }
}

void Game2AimNode::unlock_target(const std::string & reason)
{
  if (target_locked_) {
    RCLCPP_INFO(
      get_logger(),
      "🔓 [Game2 Target UNLOCKED] %s. Resetting target lock -> SEARCHING",
      reason.c_str());
    target_locked_ = false;
    target_valid_ = false;
    locked_is_midpoint_ = false;
    locked_target_tag_ids_.clear();
    current_target_tag_ids_.clear();
    active_target_id_ = -1;
    target_belt_mode_ = robot_msgs::msg::BeltMode::STOP;
    shot_requested_ = false;
    if (state_ != robot_msgs::msg::Game2State::STANDBY) {
      state_ = robot_msgs::msg::Game2State::SEARCHING;
      state_start_time_ = now();
    }
  }
}

void Game2AimNode::shot_cycle_state_callback(
  const robot_msgs::msg::ShotCycleState::SharedPtr msg)
{
  const uint8_t current_state = msg->state;
  // FEEDING（射出押し込み開始）または RETURNING（射出完了・アーム復帰開始）遷移時にターゲット固定を解除
  if ((current_state == robot_msgs::msg::ShotCycleState::FEEDING &&
       prev_shot_cycle_state_ != robot_msgs::msg::ShotCycleState::FEEDING) ||
      (current_state == robot_msgs::msg::ShotCycleState::RETURNING &&
       prev_shot_cycle_state_ == robot_msgs::msg::ShotCycleState::FEEDING))
  {
    unlock_target("Shot cycle firing/feeding detected via /dribble/shot_cycle_state");
  }
  prev_shot_cycle_state_ = current_state;
}

void Game2AimNode::shot_cycle_req_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && is_enabled_) {
    shot_requested_ = true;
    shot_requested_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "🎯 [Game2] Shot cycle requested (L2+Circle). Insurance timer started (%.1fs timeout)",
      shot_fallback_timeout_);
  }
}

uint8_t Game2AimNode::get_target_belt_mode(int row) const
{
  switch (row) {
    case 0: return robot_msgs::msg::BeltMode::LEVEL_1; // 下段 (Row 0) -> Level 1
    case 1: return robot_msgs::msg::BeltMode::LEVEL_2; // 中段 (Row 1) -> Level 2
    case 2: return robot_msgs::msg::BeltMode::LEVEL_3; // 上段 (Row 2) -> Level 3
    default: return robot_msgs::msg::BeltMode::LEVEL_1;
  }
}

void Game2AimNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    is_enabled_ = true;
    reset_sequence();
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    const auto start_time = this->now();
    state_start_time_ = start_time;
    last_imu_time_ = start_time;
    last_loop_time_ = start_time;
    RCLCPP_INFO(
      get_logger(),
      "=== [Game2 START] Sequence Activated -> SEARCHING (IMU Yaw Zero-Reset Offset: %.3f rad, TestMode: %s) ===",
      yaw_offset_, test_alignment_only_ ? "ON" : "OFF");
  } else {
    if (is_enabled_) {
      is_enabled_ = false;
      state_ = robot_msgs::msg::Game2State::STANDBY;
      reset_sequence();
      RCLCPP_INFO(get_logger(), "=== [Game2 STOP] Sequence Disengaged -> STANDBY ===");
      // 安全のため停止した瞬間に1度だけアクチュエータ停止指令を送信
      publish_all(
        geometry_msgs::msg::Twist{}, robot_msgs::msg::BeltMode::STOP,
        false, robot_msgs::msg::ArmPosition::DRIBBLE, false);
      robot_msgs::msg::Game2State state_msg;
      state_msg.state = state_;
      state_pub_->publish(state_msg);
    }
  }
}

void Game2AimNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_ && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    reset_sequence();
    RCLCPP_WARN(get_logger(), "Emergency Stop Triggered! Game 2 Auto Sequence ABORTED.");
    publish_all(
      geometry_msgs::msg::Twist{}, robot_msgs::msg::BeltMode::STOP, false,
      robot_msgs::msg::ArmPosition::DRIBBLE, false);
  }
}

void Game2AimNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  last_imu_time_ = now();

  // Angular velocities (rad/s)
  gyro_x_ = msg->angular_velocity.x;
  gyro_y_ = msg->angular_velocity.y;
  gyro_z_ = msg->angular_velocity.z;

  // Linear accelerations (m/s^2)
  accel_x_ = msg->linear_acceleration.x;
  accel_y_ = msg->linear_acceleration.y;
  accel_z_ = msg->linear_acceleration.z;

  // Orientation Quaternion -> Euler Roll, Pitch, Yaw
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll_ = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0) {
    pitch_ = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch_ = std::asin(sinp);
  }

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
}

void Game2AimNode::tag_detections_callback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  const auto current_time = now();

  for (const auto & detection : msg->detections) {
    const int id = detection.id;
    auto it = panel_grid_.find(id);
    if (it != panel_grid_.end()) {
      it->second.last_seen = current_time;
      it->second.detected = true;
      it->second.pixel_x = static_cast<double>(detection.centre.x);
      it->second.pixel_y = static_cast<double>(detection.centre.y);
      it->second.yaw_at_detection = yaw_;

      // ── 📐 厳密な3D幾何学座標変換 (カメラ光学座標系 -> ロボット旋回/射出口座標系) ──
      // 1. カメラ光学系におけるターゲット相対位置 (X_cam: 前方深度, Y_cam: 左方距離)
      const double z_cam = target_distance_;
      const double y_cam_left = -(it->second.pixel_x - camera_cx_) * z_cam / camera_fx_;

      // 2. カメラオフセット (X: +265mm前方, Y: +35mm左方) を加算してロボット座標系へ変換
      //    ロボット中心/射出口から見たターゲット位置 (x_robot, y_robot)
      it->second.x = z_cam + camera_offset_x_;
      it->second.y = y_cam_left + camera_offset_y_;
      it->second.z = camera_offset_z_;
    }
  }
}

void Game2AimNode::update_panel_states()
{
  const auto current_time = now();

  for (auto & [id, panel] : panel_grid_) {
    if ((current_time - panel.last_seen).seconds() > tag_lost_timeout_) {
      panel.detected = false;
    }
  }
}

void Game2AimNode::select_target_and_aim()
{
  if (test_alignment_only_) {
    if (!target_locked_) {
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
        target_locked_ = true;
        locked_is_midpoint_ = false;
        locked_target_tag_ids_ = {best_id};
        locked_row_ = panel_grid_[best_id].row;
        locked_belt_mode_ = get_target_belt_mode(locked_row_);
        locked_target_x_ = panel_grid_[best_id].x;
        locked_target_y_ = panel_grid_[best_id].y;
        locked_target_z_ = panel_grid_[best_id].z;
        locked_yaw_at_detection_ = panel_grid_[best_id].yaw_at_detection;
        RCLCPP_INFO(
          get_logger(),
          "🔒 [Game2 TEST Target LOCKED] Tag #%d (Row %d) Locked!",
          best_id, locked_row_);
      }
    }

    if (target_locked_ && !locked_target_tag_ids_.empty()) {
      const int target_id = locked_target_tag_ids_[0];
      const auto it = panel_grid_.find(target_id);
      if (it != panel_grid_.end() && it->second.detected) {
        locked_target_x_ = it->second.x;
        locked_target_y_ = it->second.y;
        locked_target_z_ = it->second.z;
        locked_yaw_at_detection_ = it->second.yaw_at_detection;
      }
      target_x_ = locked_target_x_;
      target_y_ = locked_target_y_;
      target_z_ = locked_target_z_;
      active_target_id_ = target_id;
      active_row_ = locked_row_;
      target_belt_mode_ = locked_belt_mode_;
      current_target_tag_ids_ = locked_target_tag_ids_;

      const double raw_heading_err = std::atan2(target_y_, target_x_);
      const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
      target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
      target_valid_ = true;
    } else {
      target_valid_ = false;
    }
    return;
  }

  // ── 🎯 ターゲットロック中: 毎ループの的変更を行わず、ロックされたターゲットのみを追従 ──
  if (target_locked_) {
    if (locked_is_midpoint_ && locked_target_tag_ids_.size() >= 2) {
      const int id_a = locked_target_tag_ids_[0];
      const int id_b = locked_target_tag_ids_[1];
      const auto it_a = panel_grid_.find(id_a);
      const auto it_b = panel_grid_.find(id_b);

      const bool a_detected = (it_a != panel_grid_.end() && it_a->second.detected);
      const bool b_detected = (it_b != panel_grid_.end() && it_b->second.detected);

      if (a_detected && b_detected) {
        locked_target_x_ = (it_a->second.x + it_b->second.x) * 0.5;
        locked_target_y_ = (it_a->second.y + it_b->second.y) * 0.5;
        locked_target_z_ = (it_a->second.z + it_b->second.z) * 0.5;
        locked_yaw_at_detection_ = (it_a->second.yaw_at_detection + it_b->second.yaw_at_detection) * 0.5;
        locked_tag_offset_x_ = locked_target_x_ - it_a->second.x;
        locked_tag_offset_y_ = locked_target_y_ - it_a->second.y;
      } else if (a_detected) {
        locked_target_x_ = it_a->second.x + locked_tag_offset_x_;
        locked_target_y_ = it_a->second.y + locked_tag_offset_y_;
        locked_target_z_ = it_a->second.z;
        locked_yaw_at_detection_ = it_a->second.yaw_at_detection;
      } else if (b_detected) {
        locked_target_x_ = it_b->second.x - locked_tag_offset_x_;
        locked_target_y_ = it_b->second.y - locked_tag_offset_y_;
        locked_target_z_ = it_b->second.z;
        locked_yaw_at_detection_ = it_b->second.yaw_at_detection;
      }
      // 両方未検出の場合は最後に取得した座標・yawを保持（IMU姿勢補間で追従）
    } else if (!locked_target_tag_ids_.empty()) {
      const int id = locked_target_tag_ids_[0];
      const auto it = panel_grid_.find(id);
      if (it != panel_grid_.end() && it->second.detected) {
        locked_target_x_ = it->second.x;
        locked_target_y_ = it->second.y;
        locked_target_z_ = it->second.z;
        locked_yaw_at_detection_ = it->second.yaw_at_detection;
      }
    }

    target_x_ = locked_target_x_;
    target_y_ = locked_target_y_;
    target_z_ = locked_target_z_;
    active_row_ = locked_row_;
    target_belt_mode_ = locked_belt_mode_;
    current_target_tag_ids_ = locked_target_tag_ids_;
    active_target_id_ = locked_target_tag_ids_[0];

    const double raw_heading_err = std::atan2(target_y_, target_x_);
    const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
    target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
    target_valid_ = true;
    return;
  }

  // ── 🔍 未ロック時: 優先パターンに従ってターゲットを探索し、決定した瞬間にロック ──
  target_valid_ = false;
  current_target_tag_ids_.clear();

  enum class TargetPattern
  {
    MIDPOINT_0_1 = 0,
    MIDPOINT_1_2 = 1,
    SINGLE_COL_0 = 2,
    SINGLE_COL_1 = 3,
    SINGLE_COL_2 = 4
  };

  const std::vector<TargetPattern> patterns = {
    TargetPattern::MIDPOINT_0_1,
    TargetPattern::MIDPOINT_1_2,
    TargetPattern::SINGLE_COL_0,
    TargetPattern::SINGLE_COL_1,
    TargetPattern::SINGLE_COL_2
  };

  const std::vector<int> rows_order = {2, 1, 0};

  auto get_row_name = [](int r) -> const char * {
    switch (r) {
      case 0: return "Bottom (Row 0)";
      case 1: return "Middle (Row 1)";
      case 2: return "Top (Row 2)";
      default: return "Unknown";
    }
  };

  for (const auto pattern : patterns) {
    if ((pattern == TargetPattern::MIDPOINT_0_1 || pattern == TargetPattern::MIDPOINT_1_2) &&
      !enable_double_panel_midpoint_targeting_)
    {
      continue;
    }

    for (const int row : rows_order) {
      const PanelTagInfo * p0 = nullptr;
      const PanelTagInfo * p1 = nullptr;
      const PanelTagInfo * p2 = nullptr;

      for (const auto & [id, panel] : panel_grid_) {
        if (panel.row == row) {
          if (panel.col == 0) {
            p0 = &panel;
          } else if (panel.col == 1) {
            p1 = &panel;
          } else if (panel.col == 2) {
            p2 = &panel;
          }
        }
      }

      if (pattern == TargetPattern::MIDPOINT_0_1) {
        if (p0 && p1 && p0->detected && p1->detected) {
          target_locked_ = true;
          locked_is_midpoint_ = true;
          locked_target_tag_ids_ = {p0->tag_id, p1->tag_id};
          locked_row_ = row;
          locked_belt_mode_ = get_target_belt_mode(row);
          locked_target_x_ = (p0->x + p1->x) * 0.5;
          locked_target_y_ = (p0->y + p1->y) * 0.5;
          locked_target_z_ = (p0->z + p1->z) * 0.5;
          locked_yaw_at_detection_ = (p0->yaw_at_detection + p1->yaw_at_detection) * 0.5;
          locked_tag_offset_x_ = locked_target_x_ - p0->x;
          locked_tag_offset_y_ = locked_target_y_ - p0->y;

          current_target_tag_ids_ = locked_target_tag_ids_;
          target_x_ = locked_target_x_;
          target_y_ = locked_target_y_;
          target_z_ = locked_target_z_;
          active_row_ = row;
          active_target_id_ = p0->tag_id;
          target_belt_mode_ = locked_belt_mode_;

          const double raw_heading_err = std::atan2(target_y_, target_x_);
          const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
          target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
          target_valid_ = true;

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target LOCKED: Col 0-1 Midpoint | %s] Tags #%d & #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p0->tag_id, p1->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return;
        }
      } else if (pattern == TargetPattern::MIDPOINT_1_2) {
        if (p1 && p2 && p1->detected && p2->detected) {
          target_locked_ = true;
          locked_is_midpoint_ = true;
          locked_target_tag_ids_ = {p1->tag_id, p2->tag_id};
          locked_row_ = row;
          locked_belt_mode_ = get_target_belt_mode(row);
          locked_target_x_ = (p1->x + p2->x) * 0.5;
          locked_target_y_ = (p1->y + p2->y) * 0.5;
          locked_target_z_ = (p1->z + p2->z) * 0.5;
          locked_yaw_at_detection_ = (p1->yaw_at_detection + p2->yaw_at_detection) * 0.5;
          locked_tag_offset_x_ = locked_target_x_ - p1->x;
          locked_tag_offset_y_ = locked_target_y_ - p1->y;

          current_target_tag_ids_ = locked_target_tag_ids_;
          target_x_ = locked_target_x_;
          target_y_ = locked_target_y_;
          target_z_ = locked_target_z_;
          active_row_ = row;
          active_target_id_ = p1->tag_id;
          target_belt_mode_ = locked_belt_mode_;

          const double raw_heading_err = std::atan2(target_y_, target_x_);
          const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
          target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
          target_valid_ = true;

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target LOCKED: Col 1-2 Midpoint | %s] Tags #%d & #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p1->tag_id, p2->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return;
        }
      } else if (pattern == TargetPattern::SINGLE_COL_0) {
        if (p0 && p0->detected) {
          target_locked_ = true;
          locked_is_midpoint_ = false;
          locked_target_tag_ids_ = {p0->tag_id};
          locked_row_ = row;
          locked_belt_mode_ = get_target_belt_mode(row);
          locked_target_x_ = p0->x;
          locked_target_y_ = p0->y;
          locked_target_z_ = p0->z;
          locked_yaw_at_detection_ = p0->yaw_at_detection;

          current_target_tag_ids_ = locked_target_tag_ids_;
          target_x_ = locked_target_x_;
          target_y_ = locked_target_y_;
          target_z_ = locked_target_z_;
          active_row_ = row;
          active_target_id_ = p0->tag_id;
          target_belt_mode_ = locked_belt_mode_;

          const double raw_heading_err = std::atan2(target_y_, target_x_);
          const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
          target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
          target_valid_ = true;

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target LOCKED: Single Left (Col 0) | %s] Tag #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p0->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return;
        }
      } else if (pattern == TargetPattern::SINGLE_COL_1) {
        if (p1 && p1->detected) {
          target_locked_ = true;
          locked_is_midpoint_ = false;
          locked_target_tag_ids_ = {p1->tag_id};
          locked_row_ = row;
          locked_belt_mode_ = get_target_belt_mode(row);
          locked_target_x_ = p1->x;
          locked_target_y_ = p1->y;
          locked_target_z_ = p1->z;
          locked_yaw_at_detection_ = p1->yaw_at_detection;

          current_target_tag_ids_ = locked_target_tag_ids_;
          target_x_ = locked_target_x_;
          target_y_ = locked_target_y_;
          target_z_ = locked_target_z_;
          active_row_ = row;
          active_target_id_ = p1->tag_id;
          target_belt_mode_ = locked_belt_mode_;

          const double raw_heading_err = std::atan2(target_y_, target_x_);
          const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
          target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
          target_valid_ = true;

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target LOCKED: Single Center (Col 1) | %s] Tag #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p1->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return;
        }
      } else if (pattern == TargetPattern::SINGLE_COL_2) {
        if (p2 && p2->detected) {
          target_locked_ = true;
          locked_is_midpoint_ = false;
          locked_target_tag_ids_ = {p2->tag_id};
          locked_row_ = row;
          locked_belt_mode_ = get_target_belt_mode(row);
          locked_target_x_ = p2->x;
          locked_target_y_ = p2->y;
          locked_target_z_ = p2->z;
          locked_yaw_at_detection_ = p2->yaw_at_detection;

          current_target_tag_ids_ = locked_target_tag_ids_;
          target_x_ = locked_target_x_;
          target_y_ = locked_target_y_;
          target_z_ = locked_target_z_;
          active_row_ = row;
          active_target_id_ = p2->tag_id;
          target_belt_mode_ = locked_belt_mode_;

          const double raw_heading_err = std::atan2(target_y_, target_x_);
          const double rotated = std::remainder(yaw_ - locked_yaw_at_detection_, 2.0 * M_PI);
          target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
          target_valid_ = true;

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target LOCKED: Single Right (Col 2) | %s] Tag #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p2->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return;
        }
      }
    }
  }
}

void Game2AimNode::control_loop()
{
  const auto current_time = now();
  double dt = (current_time - last_loop_time_).seconds();
  if (dt <= 0.001 || dt > 0.2) {
    dt = 0.05;
  }
  last_loop_time_ = current_time;

  if (!is_enabled_ || emergency_stop_active_ || state_ == robot_msgs::msg::Game2State::STANDBY) {
    state_ = robot_msgs::msg::Game2State::STANDBY;
    last_cmd_wz_ = 0.0;
    robot_msgs::msg::Game2State state_msg;
    state_msg.state = state_;
    state_pub_->publish(state_msg);
    // STANDBY時は手動操作 (joy_controller / heading_hold) にトピックを譲るため
    // 周期的なゼロ速度・停止コマンドのパブリッシュは行わない
    return;
  }

  // ── 保険タイマー: 射出要求から一定秒経過しても完了通知が来ない場合のフェイルセーフ ──
  if (shot_requested_ && (current_time - shot_requested_time_).seconds() >= shot_fallback_timeout_) {
    unlock_target("Shot fallback timeout elapsed (insurance timer fired)");
  }

  update_panel_states();
  select_target_and_aim();

  // Target not detected: Search mode
  if (!target_valid_) {
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    robot_msgs::msg::Game2State state_msg;
    state_msg.state = state_;
    state_pub_->publish(state_msg);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = search_angular_z_;
    last_cmd_wz_ = search_angular_z_;

    if (std::abs(search_angular_z_) > 0.001) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(), 1000,
        "🔍 [Game2 Search] Scanning for target AprilTags... Rotating %+.2f rad/s",
        search_angular_z_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(), 1000,
        "🔍 [Game2 Search] Waiting for target AprilTags... Standing still");
    }

    publish_all(
      cmd, robot_msgs::msg::BeltMode::STOP,
      !test_alignment_only_, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  // Target is valid: check alignment
  geometry_msgs::msg::Twist cmd;
  const double heading_err = target_heading_err_;
  const bool is_aligned = (std::abs(heading_err) < yaw_tolerance_);
  uint8_t current_belt_mode = robot_msgs::msg::BeltMode::STOP;

  if (is_aligned) {
    // 照準完了 -> PREPARING_SHOOT (ベルト回転開始)
    state_ = robot_msgs::msg::Game2State::PREPARING_SHOOT;
    cmd.angular.z = 0.0;
    last_cmd_wz_ = 0.0;
    current_belt_mode = test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : target_belt_mode_;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(), 500,
      "🚀 [Game2 PREPARING_SHOOT] Tag #%d (Row %d) Aligned! Spinning Belt (Mode: %u)",
      active_target_id_, active_row_, current_belt_mode);
  } else {
    // 照準旋回中 -> ALIGNING (ベルト停止)
    state_ = robot_msgs::msg::Game2State::ALIGNING;
    current_belt_mode = robot_msgs::msg::BeltMode::STOP;

    // Proportional visual error + IMU Gyro damping (PD control)
    // 機体ハードウェアの旋回方向（+wz = CW右旋回）に合わせて符号を設定
    double desired_wz = -kp_yaw_ * heading_err;
    if (imu_received_ && (current_time - last_imu_time_).seconds() < 0.5) {
      desired_wz -= kd_yaw_ * gyro_z_;
    }

    // Stiction overcoming minimum angular velocity
    if (std::abs(desired_wz) < min_angular_z_) {
      desired_wz = std::copysign(min_angular_z_, desired_wz);
    }
    desired_wz = std::clamp(desired_wz, -max_angular_z_, max_angular_z_);

    // Slew rate limiter on angular acceleration
    const double max_wz_step = max_angular_accel_ * dt;
    if (desired_wz > last_cmd_wz_ + max_wz_step) {
      desired_wz = last_cmd_wz_ + max_wz_step;
    } else if (desired_wz < last_cmd_wz_ - max_wz_step) {
      desired_wz = last_cmd_wz_ - max_wz_step;
    }

    cmd.angular.z = desired_wz;
    last_cmd_wz_ = cmd.angular.z;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(), 200,
      "🎯 [Game2 ALIGNING Tag #%d (Row %d)] Err: %+.2f deg | Cmd wz: %+.3f rad/s | 🔄 TURNING",
      active_target_id_, active_row_, heading_err * 180.0 / M_PI, cmd.angular.z);
  }

  robot_msgs::msg::Game2State state_msg;
  state_msg.state = state_;
  state_pub_->publish(state_msg);

  publish_all(
    cmd, current_belt_mode,
    !test_alignment_only_,
    robot_msgs::msg::ArmPosition::DRIBBLE,
    false);
}

void Game2AimNode::publish_all(
  const geometry_msgs::msg::Twist & cmd_vel,
  uint8_t belt_mode,
  bool dribble_enabled,
  uint8_t arm_mode,
  bool completed)
{
  cmd_vel_pub_->publish(cmd_vel);

  robot_msgs::msg::BeltMode mode_msg;
  mode_msg.mode = belt_mode;
  belt_mode_pub_->publish(mode_msg);

  std_msgs::msg::Bool dribble_msg;
  dribble_msg.data = dribble_enabled;
  dribble_enabled_pub_->publish(dribble_msg);

  robot_msgs::msg::ArmPosition arm_msg;
  arm_msg.position = arm_mode;
  arm_position_pub_->publish(arm_msg);

  std_msgs::msg::Bool completed_msg;
  completed_msg.data = completed;
  completed_pub_->publish(completed_msg);
}

}  // namespace robot_controller
