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
    camera_info_topic_, cmd_qos,
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
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/drive/cmd_vel");
  detections_topic_ = declare_parameter<std::string>("detections_topic", "/detections");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");
  camera_info_topic_ =
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");

  // Control gains & Limits
  kp_yaw_ = declare_parameter<double>("kp_yaw", 1.8);
  yaw_command_sign_ = declare_parameter<double>("yaw_command_sign", -1.0);
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
    } else if (name == "yaw_command_sign") {
      yaw_command_sign_ = param.as_double();
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

void Game2AimNode::clear_target()
{
  current_target_tag_ids_.clear();
  is_midpoint_target_ = false;
  active_target_id_ = -1;
  target_belt_mode_ = robot_msgs::msg::BeltMode::STOP;
  target_x_ = 0.0;
  target_y_ = 0.0;
  target_z_ = 0.0;
  target_yaw_at_detection_ = 0.0;
  target_tag_offset_x_ = 0.0;
  target_tag_offset_y_ = 0.0;
  target_heading_err_ = 0.0;
  shot_requested_ = false;
}

void Game2AimNode::transition_to(uint8_t new_state, const std::string & reason)
{
  if (state_ == new_state) {
    return;
  }
  const auto get_state_name = [](uint8_t s) -> const char * {
    switch (s) {
      case robot_msgs::msg::Game2State::STANDBY: return "STANDBY";
      case robot_msgs::msg::Game2State::SEARCHING: return "SEARCHING";
      case robot_msgs::msg::Game2State::ALIGNING: return "ALIGNING";
      case robot_msgs::msg::Game2State::PREPARING_SHOOT: return "PREPARING_SHOOT";
      case robot_msgs::msg::Game2State::SHOOTING: return "SHOOTING";
      case robot_msgs::msg::Game2State::WAITING_RESULT: return "WAITING_RESULT";
      case robot_msgs::msg::Game2State::COMPLETED: return "COMPLETED";
      default: return "UNKNOWN";
    }
  };

  RCLCPP_INFO(
    get_logger(),
    "🔄 [Game2 State Transition] %s -> %s (Reason: %s)",
    get_state_name(state_), get_state_name(new_state), reason.c_str());

  state_ = new_state;
  state_start_time_ = now();

  if (state_ == robot_msgs::msg::Game2State::STANDBY || state_ == robot_msgs::msg::Game2State::SEARCHING) {
    clear_target();
  }

  robot_msgs::msg::Game2State state_msg;
  state_msg.state = state_;
  state_pub_->publish(state_msg);
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
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    const auto start_time = this->now();
    last_imu_time_ = start_time;
    last_loop_time_ = start_time;
    transition_to(robot_msgs::msg::Game2State::SEARCHING, "Start command received (true)");
  } else {
    if (state_ != robot_msgs::msg::Game2State::STANDBY) {
      transition_to(robot_msgs::msg::Game2State::STANDBY, "Start command received (false)");
      publish_all(
        geometry_msgs::msg::Twist{}, robot_msgs::msg::BeltMode::STOP,
        false, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    }
  }
}

void Game2AimNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_ && state_ != robot_msgs::msg::Game2State::STANDBY) {
    RCLCPP_WARN(get_logger(), "Emergency Stop Triggered! Game 2 Auto Sequence ABORTED.");
    transition_to(robot_msgs::msg::Game2State::STANDBY, "Emergency stop active");
    publish_all(
      geometry_msgs::msg::Twist{}, robot_msgs::msg::BeltMode::STOP, false,
      robot_msgs::msg::ArmPosition::DRIBBLE, false);
  }
}

void Game2AimNode::shot_cycle_state_callback(
  const robot_msgs::msg::ShotCycleState::SharedPtr msg)
{
  const uint8_t current_state = msg->state;
  if (state_ == robot_msgs::msg::Game2State::PREPARING_SHOOT || state_ == robot_msgs::msg::Game2State::ALIGNING) {
    // FEEDING（射出押し込み開始）または RETURNING（射出完了・アーム復帰開始）遷移時に次の探索へ移行
    if ((current_state == robot_msgs::msg::ShotCycleState::FEEDING &&
         prev_shot_cycle_state_ != robot_msgs::msg::ShotCycleState::FEEDING) ||
        (current_state == robot_msgs::msg::ShotCycleState::RETURNING &&
         prev_shot_cycle_state_ == robot_msgs::msg::ShotCycleState::FEEDING))
    {
      transition_to(robot_msgs::msg::Game2State::SEARCHING, "Shot cycle firing/feeding detected via /dribble/shot_cycle_state");
    }
  }
  prev_shot_cycle_state_ = current_state;
}

void Game2AimNode::shot_cycle_req_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && state_ != robot_msgs::msg::Game2State::STANDBY) {
    shot_requested_ = true;
    shot_requested_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "🎯 [Game2] Shot cycle requested (L2+Circle). Insurance timer started (%.1fs timeout)",
      shot_fallback_timeout_);
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
      const double z_cam = target_distance_;
      const double y_cam_left = -(it->second.pixel_x - camera_cx_) * z_cam / camera_fx_;

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

bool Game2AimNode::find_best_target()
{
  if (test_alignment_only_) {
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
      target_heading_err_ = std::atan2(target_y_, target_x_);
      RCLCPP_INFO(
        get_logger(),
        "🎯 [Game2 TEST Target Selected] Tag #%d (Row %d) | Err: %+.2f deg",
        best_id, active_row_, target_heading_err_ * 180.0 / M_PI);
      return true;
    }
    return false;
  }

  // ── 🎯 Column優先ターゲットパターン定義 (5段階) ──
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
          is_midpoint_target_ = true;
          current_target_tag_ids_ = {p0->tag_id, p1->tag_id};
          active_target_id_ = p0->tag_id;
          active_row_ = row;
          target_belt_mode_ = get_target_belt_mode(row);
          target_x_ = (p0->x + p1->x) * 0.5;
          target_y_ = (p0->y + p1->y) * 0.5;
          target_z_ = (p0->z + p1->z) * 0.5;
          target_yaw_at_detection_ = (p0->yaw_at_detection + p1->yaw_at_detection) * 0.5;
          target_tag_offset_x_ = target_x_ - p0->x;
          target_tag_offset_y_ = target_y_ - p0->y;
          target_heading_err_ = std::atan2(target_y_, target_x_);

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target Locked: Col 0-1 Midpoint | %s] Tags #%d & #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p0->tag_id, p1->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return true;
        }
      } else if (pattern == TargetPattern::MIDPOINT_1_2) {
        if (p1 && p2 && p1->detected && p2->detected) {
          is_midpoint_target_ = true;
          current_target_tag_ids_ = {p1->tag_id, p2->tag_id};
          active_target_id_ = p1->tag_id;
          active_row_ = row;
          target_belt_mode_ = get_target_belt_mode(row);
          target_x_ = (p1->x + p2->x) * 0.5;
          target_y_ = (p1->y + p2->y) * 0.5;
          target_z_ = (p1->z + p2->z) * 0.5;
          target_yaw_at_detection_ = (p1->yaw_at_detection + p2->yaw_at_detection) * 0.5;
          target_tag_offset_x_ = target_x_ - p1->x;
          target_tag_offset_y_ = target_y_ - p1->y;
          target_heading_err_ = std::atan2(target_y_, target_x_);

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target Locked: Col 1-2 Midpoint | %s] Tags #%d & #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p1->tag_id, p2->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return true;
        }
      } else if (pattern == TargetPattern::SINGLE_COL_0) {
        if (p0 && p0->detected) {
          is_midpoint_target_ = false;
          current_target_tag_ids_ = {p0->tag_id};
          active_target_id_ = p0->tag_id;
          active_row_ = row;
          target_belt_mode_ = get_target_belt_mode(row);
          target_x_ = p0->x;
          target_y_ = p0->y;
          target_z_ = p0->z;
          target_yaw_at_detection_ = p0->yaw_at_detection;
          target_tag_offset_x_ = 0.0;
          target_tag_offset_y_ = 0.0;
          target_heading_err_ = std::atan2(target_y_, target_x_);

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target Locked: Single Left (Col 0) | %s] Tag #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p0->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return true;
        }
      } else if (pattern == TargetPattern::SINGLE_COL_1) {
        if (p1 && p1->detected) {
          is_midpoint_target_ = false;
          current_target_tag_ids_ = {p1->tag_id};
          active_target_id_ = p1->tag_id;
          active_row_ = row;
          target_belt_mode_ = get_target_belt_mode(row);
          target_x_ = p1->x;
          target_y_ = p1->y;
          target_z_ = p1->z;
          target_yaw_at_detection_ = p1->yaw_at_detection;
          target_tag_offset_x_ = 0.0;
          target_tag_offset_y_ = 0.0;
          target_heading_err_ = std::atan2(target_y_, target_x_);

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target Locked: Single Center (Col 1) | %s] Tag #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p1->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return true;
        }
      } else if (pattern == TargetPattern::SINGLE_COL_2) {
        if (p2 && p2->detected) {
          is_midpoint_target_ = false;
          current_target_tag_ids_ = {p2->tag_id};
          active_target_id_ = p2->tag_id;
          active_row_ = row;
          target_belt_mode_ = get_target_belt_mode(row);
          target_x_ = p2->x;
          target_y_ = p2->y;
          target_z_ = p2->z;
          target_yaw_at_detection_ = p2->yaw_at_detection;
          target_tag_offset_x_ = 0.0;
          target_tag_offset_y_ = 0.0;
          target_heading_err_ = std::atan2(target_y_, target_x_);

          RCLCPP_INFO(
            get_logger(),
            "🔒 [Target Locked: Single Right (Col 2) | %s] Tag #%d (Err: %+.2f deg | BeltMode: LEVEL_%d)",
            get_row_name(row), p2->tag_id, target_heading_err_ * 180.0 / M_PI,
            target_belt_mode_);
          return true;
        }
      }
    }
  }

  return false;
}

void Game2AimNode::update_active_target_tracking()
{
  if (current_target_tag_ids_.empty()) {
    return;
  }

  if (is_midpoint_target_ && current_target_tag_ids_.size() >= 2) {
    const int id_a = current_target_tag_ids_[0];
    const int id_b = current_target_tag_ids_[1];
    const auto it_a = panel_grid_.find(id_a);
    const auto it_b = panel_grid_.find(id_b);

    const bool a_detected = (it_a != panel_grid_.end() && it_a->second.detected);
    const bool b_detected = (it_b != panel_grid_.end() && it_b->second.detected);

    if (a_detected && b_detected) {
      target_x_ = (it_a->second.x + it_b->second.x) * 0.5;
      target_y_ = (it_a->second.y + it_b->second.y) * 0.5;
      target_z_ = (it_a->second.z + it_b->second.z) * 0.5;
      target_yaw_at_detection_ = (it_a->second.yaw_at_detection + it_b->second.yaw_at_detection) * 0.5;
      target_tag_offset_x_ = target_x_ - it_a->second.x;
      target_tag_offset_y_ = target_y_ - it_a->second.y;
    } else if (a_detected) {
      target_x_ = it_a->second.x + target_tag_offset_x_;
      target_y_ = it_a->second.y + target_tag_offset_y_;
      target_z_ = it_a->second.z;
      target_yaw_at_detection_ = it_a->second.yaw_at_detection;
    } else if (b_detected) {
      target_x_ = it_b->second.x - target_tag_offset_x_;
      target_y_ = it_b->second.y - target_tag_offset_y_;
      target_z_ = it_b->second.z;
      target_yaw_at_detection_ = it_b->second.yaw_at_detection;
    }
  } else {
    const int id = current_target_tag_ids_[0];
    const auto it = panel_grid_.find(id);
    if (it != panel_grid_.end() && it->second.detected) {
      target_x_ = it->second.x;
      target_y_ = it->second.y;
      target_z_ = it->second.z;
      target_yaw_at_detection_ = it->second.yaw_at_detection;
    }
  }

  // IMUオドメトリ姿勢補間（デッドレコニング）
  const double raw_heading_err = std::atan2(target_y_, target_x_);
  const double rotated = std::remainder(yaw_ - target_yaw_at_detection_, 2.0 * M_PI);
  target_heading_err_ = std::remainder(raw_heading_err - rotated, 2.0 * M_PI);
}

void Game2AimNode::control_loop()
{
  const auto current_time = now();
  double dt = (current_time - last_loop_time_).seconds();
  if (dt <= 0.001 || dt > 0.2) {
    dt = 0.05;
  }
  last_loop_time_ = current_time;

  // 非常停止チェック
  if (emergency_stop_active_ && state_ != robot_msgs::msg::Game2State::STANDBY) {
    transition_to(robot_msgs::msg::Game2State::STANDBY, "Emergency stop active");
  }

  // STANDBY時は手動操作 (joy_controller / heading_hold) にトピックを譲るため何も出力せずreturn
  if (state_ == robot_msgs::msg::Game2State::STANDBY) {
    last_cmd_wz_ = 0.0;
    robot_msgs::msg::Game2State state_msg;
    state_msg.state = state_;
    state_pub_->publish(state_msg);
    return;
  }

  // AprilTag 見失いタイムアウト更新
  update_panel_states();

  // 保険タイマー: 射出ボタン押下から一定時間経過しても完了ステートが来ない場合のフォールバック
  if (shot_requested_ && (current_time - shot_requested_time_).seconds() >= shot_fallback_timeout_) {
    transition_to(robot_msgs::msg::Game2State::SEARCHING, "Shot fallback timeout elapsed (insurance timer fired)");
  }

  geometry_msgs::msg::Twist cmd;
  uint8_t current_belt_mode = robot_msgs::msg::BeltMode::STOP;

  switch (state_) {
    case robot_msgs::msg::Game2State::SEARCHING: {
      current_belt_mode = robot_msgs::msg::BeltMode::STOP;
      cmd.angular.z = search_angular_z_;
      last_cmd_wz_ = search_angular_z_;

      if (find_best_target()) {
        transition_to(robot_msgs::msg::Game2State::ALIGNING, "Target panel found and locked");
      } else {
        if (std::abs(search_angular_z_) > 0.001) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "🔍 [Game2 SEARCHING] Scanning for target AprilTags... Rotating %+.2f rad/s",
            search_angular_z_);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "🔍 [Game2 SEARCHING] Waiting for target AprilTags... Standing still");
        }
      }
      break;
    }

    case robot_msgs::msg::Game2State::ALIGNING: {
      current_belt_mode = robot_msgs::msg::BeltMode::STOP;
      update_active_target_tracking();

      const double heading_err = target_heading_err_;
      const bool is_aligned = (std::abs(heading_err) < yaw_tolerance_);

      if (is_aligned) {
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;
        transition_to(robot_msgs::msg::Game2State::PREPARING_SHOOT, "Target aligned within tolerance");
        current_belt_mode = test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : target_belt_mode_;
      } else {
        // PD Control (IMU Gyro damping)
        double desired_wz = yaw_command_sign_ * kp_yaw_ * heading_err;
        if (imu_received_ && (current_time - last_imu_time_).seconds() < 0.5) {
          desired_wz -= kd_yaw_ * gyro_z_;
        }

        if (std::abs(desired_wz) < min_angular_z_) {
          desired_wz = std::copysign(min_angular_z_, desired_wz);
        }
        desired_wz = std::clamp(desired_wz, -max_angular_z_, max_angular_z_);

        const double max_wz_step = max_angular_accel_ * dt;
        if (desired_wz > last_cmd_wz_ + max_wz_step) {
          desired_wz = last_cmd_wz_ + max_wz_step;
        } else if (desired_wz < last_cmd_wz_ - max_wz_step) {
          desired_wz = last_cmd_wz_ - max_wz_step;
        }

        cmd.angular.z = desired_wz;
        last_cmd_wz_ = cmd.angular.z;

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 200,
          "🎯 [Game2 ALIGNING Tag #%d (Row %d)] Err: %+.2f deg | Cmd wz: %+.3f rad/s | 🔄 TURNING",
          active_target_id_, active_row_, heading_err * 180.0 / M_PI, cmd.angular.z);
      }
      break;
    }

    case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
      cmd.angular.z = 0.0;
      last_cmd_wz_ = 0.0;
      current_belt_mode = test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : target_belt_mode_;
      update_active_target_tracking();

      // 外力等で誤差が許容値を超えてズレた場合は ALIGNING へ戻って再照準
      if (std::abs(target_heading_err_) > yaw_tolerance_ * 1.5) {
        transition_to(robot_msgs::msg::Game2State::ALIGNING, "Heading error exceeded tolerance in PREPARING_SHOOT");
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "🚀 [Game2 PREPARING_SHOOT] Tag #%d (Row %d) Aligned! Spinning Belt (Mode: %u) | Ready for Shot",
          active_target_id_, active_row_, current_belt_mode);
      }
      break;
    }

    default:
      break;
  }

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
