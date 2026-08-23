#include "game2_shooter/game2_auto_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game2AutoNode::Game2AutoNode(const rclcpp::NodeOptions & options)
: Node("game2_auto_node", options)
{
  load_parameters();

  const auto init_now = this->now();
  state_start_time_ = init_now;
  shoot_start_time_ = init_now;
  ball_detected_time_ = init_now;
  last_imu_time_ = init_now;
  last_loop_time_ = init_now;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto cmd_qos = rclcpp::QoS(10);
  const auto estop_qos = rclcpp::QoS(1).reliable().transient_local();

  // Subscriptions
  detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "/detections", cmd_qos,
    std::bind(&Game2AutoNode::tag_detections_callback, this, std::placeholders::_1));

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game2/command_start", cmd_qos,
    std::bind(&Game2AutoNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2AutoNode::imu_callback, this, std::placeholders::_1));

  ball_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/ball_detected", cmd_qos,
    std::bind(&Game2AutoNode::ball_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", estop_qos,
    std::bind(&Game2AutoNode::emergency_stop_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, cmd_qos);
  underbelt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/belt/command_underbelt_rpm", cmd_qos);
  upperbelt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/belt/command_upperbelt_rpm", cmd_qos);
  shoot_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", cmd_qos);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", cmd_qos);
  arm_position_pub_ =
    create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", cmd_qos);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game2/completed", cmd_qos);
  state_pub_ = create_publisher<robot_msgs::msg::Game2State>(
    "/game2/state", rclcpp::QoS(1).reliable().transient_local());

  // Dynamic Parameter Callback
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&Game2AutoNode::parameter_callback, this, std::placeholders::_1));

  // 20 Hz Control Loop Timer (50 ms)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2AutoNode::control_loop, this));

  RCLCPP_INFO(
    get_logger(),
    "Game2AutoNode initialized. Output CmdVel: %s, TestAlignmentOnly: %s",
    cmd_vel_topic_.c_str(), test_alignment_only_ ? "true" : "false");
}

void Game2AutoNode::load_parameters()
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel_heading");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");

  // Control gains & Limits
  kp_yaw_ = declare_parameter<double>("kp_yaw", 2.2);
  kd_yaw_ = declare_parameter<double>("kd_yaw", 0.10);
  min_angular_z_ = declare_parameter<double>("min_angular_z", 0.12);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.80);
  max_angular_accel_ = declare_parameter<double>("max_angular_accel", 4.0);
  target_distance_ = declare_parameter<double>("target_distance", 4.0);

  // Camera Physical & Optical Parameters (base_link frame: kicker is +X, left is +Y)
  camera_offset_x_ = declare_parameter<double>("camera_offset_x", -0.265);
  camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.035);
  camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.193);
  shooter_offset_x_ = declare_parameter<double>("shooter_offset_x", -0.265);
  shooter_offset_y_ = declare_parameter<double>("shooter_offset_y", 0.0);
  camera_image_width_ = declare_parameter<double>("camera_image_width", 1920.0);
  camera_image_height_ = declare_parameter<double>("camera_image_height", 1080.0);
  camera_fx_ = declare_parameter<double>("camera_fx", 723.47);
  camera_fy_ = declare_parameter<double>("camera_fy", 723.47);
  camera_cx_ = declare_parameter<double>("camera_cx", 746.35);
  camera_cy_ = declare_parameter<double>("camera_cy", 578.43);

  // Tolerances & Timings
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.015);
  dist_tolerance_ = declare_parameter<double>("dist_tolerance", 0.05);
  underbelt_rpm_bottom_ = declare_parameter<double>("underbelt_rpm_bottom", 2050.0);
  upperbelt_rpm_bottom_ = declare_parameter<double>("upperbelt_rpm_bottom", 2050.0);
  underbelt_rpm_middle_ = declare_parameter<double>("underbelt_rpm_middle", 2300.0);
  upperbelt_rpm_middle_ = declare_parameter<double>("upperbelt_rpm_middle", 2300.0);
  underbelt_rpm_top_ = declare_parameter<double>("underbelt_rpm_top", 2700.0);
  upperbelt_rpm_top_ = declare_parameter<double>("upperbelt_rpm_top", 2700.0);
  open_duration_ = declare_parameter<double>("open_duration", 0.3);
  shoot_hold_duration_ = declare_parameter<double>("shoot_hold_duration", 0.8);
  ball_settle_duration_ = declare_parameter<double>("ball_settle_duration", 0.3);
  belt_spinup_duration_ = declare_parameter<double>("belt_spinup_duration", 0.5);
  tag_pitch_x_ = declare_parameter<double>("tag_pitch_x", 0.40);
  tag_pitch_y_ = declare_parameter<double>("tag_pitch_y", 0.43);
  tag_lost_timeout_ = declare_parameter<double>("tag_lost_timeout", 0.5);
  aligning_timeout_ = declare_parameter<double>("aligning_timeout", 10.0);
  shooting_timeout_ = declare_parameter<double>("shooting_timeout", 3.0);
  result_wait_duration_ = declare_parameter<double>("result_wait_duration", 1.0);
  max_shots_per_panel_ = declare_parameter<int>("max_shots_per_panel", 0);
  max_total_balls_ = declare_parameter<int>("max_total_balls", 15);
  enable_ball_limit_ = declare_parameter<bool>("enable_ball_limit", false);
  require_ball_detected_ = declare_parameter<bool>("require_ball_detected", true);
  test_alignment_only_ = declare_parameter<bool>("test_alignment_only", false);
  auto_advance_rows_ = declare_parameter<bool>("auto_advance_rows", true);
  enable_double_panel_midpoint_targeting_ =
    declare_parameter<bool>("enable_double_panel_midpoint_targeting", true);
  prefer_same_row_first_ = declare_parameter<bool>("prefer_same_row_first", false);
  enable_vertical_sweep_ = declare_parameter<bool>("enable_vertical_sweep", true);
  enable_nearest_angle_search_ = declare_parameter<bool>("enable_nearest_angle_search", true);

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
        info.shot_completed = false;
        info.shot_count = 0;
        info.last_seen = this->now();
        panel_grid_[id] = info;
      }
    };
  register_row(bottom_tags, 0);
  register_row(middle_tags, 1);
  register_row(top_tags, 2);
}

rcl_interfaces::msg::SetParametersResult Game2AutoNode::parameter_callback(
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
    } else if (name == "shooting_timeout") {
      shooting_timeout_ = param.as_double();
    } else if (name == "belt_spinup_duration") {
      belt_spinup_duration_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Param updated: belt_spinup_duration = %.3f", belt_spinup_duration_);
    } else if (name == "result_wait_duration") {
      result_wait_duration_ = param.as_double();
    } else if (name == "max_total_balls") {
      max_total_balls_ = param.as_int();
    } else if (name == "enable_ball_limit") {
      enable_ball_limit_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: enable_ball_limit = %s",
        enable_ball_limit_ ? "true" : "false");
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
    } else if (name == "tag_pitch_x") {
      tag_pitch_x_ = param.as_double();
    } else if (name == "tag_pitch_y") {
      tag_pitch_y_ = param.as_double();
    } else if (name == "camera_fx") {
      camera_fx_ = param.as_double();
    } else if (name == "camera_offset_x") {
      camera_offset_x_ = param.as_double();
    } else if (name == "camera_offset_y") {
      camera_offset_y_ = param.as_double();
    } else if (name == "camera_offset_z") {
      camera_offset_z_ = param.as_double();
    } else if (name == "shooter_offset_x") {
      shooter_offset_x_ = param.as_double();
    } else if (name == "shooter_offset_y") {
      shooter_offset_y_ = param.as_double();
    } else if (name == "underbelt_rpm_bottom") {
      underbelt_rpm_bottom_ = param.as_double();
    } else if (name == "upperbelt_rpm_bottom") {
      upperbelt_rpm_bottom_ = param.as_double();
    } else if (name == "underbelt_rpm_middle") {
      underbelt_rpm_middle_ = param.as_double();
    } else if (name == "upperbelt_rpm_middle") {
      upperbelt_rpm_middle_ = param.as_double();
    } else if (name == "underbelt_rpm_top") {
      underbelt_rpm_top_ = param.as_double();
    } else if (name == "upperbelt_rpm_top") {
      upperbelt_rpm_top_ = param.as_double();
    } else if (name == "test_alignment_only") {
      test_alignment_only_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: test_alignment_only = %s",
        test_alignment_only_ ? "true" : "false");
    } else if (name == "max_shots_per_panel") {
      max_shots_per_panel_ = param.as_int();
      RCLCPP_INFO(get_logger(), "Param updated: max_shots_per_panel = %d", max_shots_per_panel_);
    } else if (name == "require_ball_detected") {
      require_ball_detected_ = param.as_bool();
    } else if (name == "enable_double_panel_midpoint_targeting") {
      enable_double_panel_midpoint_targeting_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: enable_double_panel_midpoint_targeting = %s",
        enable_double_panel_midpoint_targeting_ ? "true" : "false");
    } else if (name == "prefer_same_row_first") {
      prefer_same_row_first_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: prefer_same_row_first = %s",
        prefer_same_row_first_ ? "true" : "false");
    } else if (name == "enable_vertical_sweep") {
      enable_vertical_sweep_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: enable_vertical_sweep = %s",
        enable_vertical_sweep_ ? "true" : "false");
    } else if (name == "enable_nearest_angle_search") {
      enable_nearest_angle_search_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(), "Param updated: enable_nearest_angle_search = %s",
        enable_nearest_angle_search_ ? "true" : "false");
    }
  }

  return result;
}

void Game2AutoNode::reset_sequence()
{
  active_row_ = 0;
  active_target_id_ = -1;
  locked_target_id_ = -1;
  current_target_tag_ids_.clear();
  target_valid_ = false;
  last_cmd_wz_ = 0.0;
  total_shots_fired_ = 0;
  last_target_underbelt_rpm_ = 0.0;
  last_target_upperbelt_rpm_ = 0.0;
  rpm_changed_time_ = this->now();

  for (auto & [id, panel] : panel_grid_) {
    panel.shot_completed = false;
    panel.shot_count = 0;
    panel.detected = false;
  }
}

void Game2AutoNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    reset_sequence();
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    const auto start_time = this->now();
    state_start_time_ = start_time;
    ball_detected_time_ = start_time;
    shoot_start_time_ = start_time;
    last_imu_time_ = start_time;
    last_loop_time_ = start_time;
    RCLCPP_INFO(
      get_logger(),
      "=== [Game2 START] Sequence Activated (IMU Yaw Zero-Reset Offset: %.3f rad, TestMode: %s) ===",
      yaw_offset_, test_alignment_only_ ? "ON" : "OFF");
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    reset_sequence();
    RCLCPP_INFO(get_logger(), "=== [Game2 STOP] Sequence Disengaged ===");
  }
}

void Game2AutoNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_ && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    reset_sequence();
    RCLCPP_WARN(get_logger(), "Emergency Stop Triggered! Game 2 Auto Sequence ABORTED.");
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f, false, false,
      robot_msgs::msg::ArmPosition::DRIBBLE, false);
  }
}

void Game2AutoNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

void Game2AutoNode::tag_detections_callback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  const auto current_time = now();

  // 1. 各検出タグのピクセル座標を記録
  for (const auto & detection : msg->detections) {
    const int id = detection.id;
    auto it = panel_grid_.find(id);
    if (it != panel_grid_.end()) {
      it->second.last_seen = current_time;
      it->second.detected = true;
      it->second.pixel_x = static_cast<double>(detection.centre.x);
      it->second.pixel_y = static_cast<double>(detection.centre.y);
    }
  }

  // 2. 📐 既知のTag中心間距離（横0.40m, 縦0.43m）を用いたリアルタイム高精度実距離(Z)推定
  double estimated_z = target_distance_; // デフォルト値 (4.0m)
  double z_sum = 0.0;
  int z_count = 0;

  // 同一同一段内で2つ以上のTagが検出されている場合、横ピッチ(0.40m)からZ距離を三角測量
  for (int r = 0; r <= 2; ++r) {
    std::vector<const PanelTagInfo *> row_detected;
    for (const auto & [id, panel] : panel_grid_) {
      if (panel.row == r && panel.detected &&
        (current_time - panel.last_seen).seconds() <= tag_lost_timeout_)
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
            const double real_dx = delta_col * tag_pitch_x_; // [m]
            const double z_est = (camera_fx_ * real_dx) / delta_pixel_x;
            // 妥当な距離範囲 (2.0m 〜 6.0m) の場合のみ採用
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

  // 3. 各タグのロボット座標系 (base_link: x_robot, y_robot, z_robot) を精密計算
  // カメラは後方 -X 向きに搭載されているため:
  // - 画像の奥行き Z_cam は base_link の後方方向 (-X) に展開
  // - 画像の水平オフセット (u - cx) は、後方を向いたカメラの画像左側 (u < cx) が base_link の左側 (+Y) に対応
  for (auto & [id, panel] : panel_grid_) {
    if (panel.detected) {
      const double z_cam = estimated_z;
      const double y_offset_from_cam = -(panel.pixel_x - camera_cx_) * z_cam / camera_fx_;
      panel.x = camera_offset_x_ - z_cam; // 後方 -X 空間 (約 -0.265 - 4.0 = -4.265m)
      panel.y = camera_offset_y_ + y_offset_from_cam; // base_link の Y 空間
      panel.z = camera_offset_z_;
    }
  }

  // テストモード時: 最も射出口正面に近いタグを選択
  if (test_alignment_only_) {
    int best_id = -1;
    double min_heading_err_abs = 1e9;
    double best_heading_err = 0.0;
    double best_pixel_x = 0.0;

    for (const auto & detection : msg->detections) {
      const int id = detection.id;
      if (panel_grid_.find(id) != panel_grid_.end()) {
        const double u = static_cast<double>(detection.centre.x);
        const double z_cam = target_distance_;
        const double y_offset_from_cam = -(u - camera_cx_) * z_cam / camera_fx_;
        const double x_target = camera_offset_x_ - z_cam;
        const double y_target = camera_offset_y_ + y_offset_from_cam;

        // 射出口(shooter_offset_x, shooter_offset_y)から後方(-X)ターゲットへの方位角誤差
        // 射出口の射出方向ベクトルは (-1, 0) すなわち yaw = M_PI (または -M_PI)
        const double dx = x_target - shooter_offset_x_;
        const double dy = y_target - shooter_offset_y_;
        const double heading_err = std::remainder(std::atan2(dy, dx) - M_PI, 2.0 * M_PI);

        if (std::abs(heading_err) < min_heading_err_abs) {
          min_heading_err_abs = std::abs(heading_err);
          best_id = id;
          best_heading_err = heading_err;
          best_pixel_x = u;
        }
      }
    }

    if (best_id != -1) {
      active_target_id_ = best_id;
      const auto & target = panel_grid_[best_id];
      target_x_ = target.x;
      target_y_ = target.y;
      target_z_ = target.z;
      target_heading_err_ = best_heading_err;
      target_valid_ = true;

      // 射出口と完全に一致する理論目標ピクセル (カメラが射出口より左(+35mm)にあるため、ターゲットは画像右側に映る)
      const double aligned_target_pixel =
        camera_cx_ + ((camera_offset_y_ - shooter_offset_y_) / target_distance_) * camera_fx_;

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(), 200,
        "📷 [VisualServo Track Tag #%d] HeadingErr: %+.2f deg | Pixel: %.1f px (TargetAligned: %.1f px)",
        best_id, best_heading_err * 180.0 / M_PI, best_pixel_x, aligned_target_pixel);
    }
  }
}

void Game2AutoNode::update_panel_states()
{
  const auto current_time = now();

  for (auto & [id, panel] : panel_grid_) {
    if (panel.detected && (current_time - panel.last_seen).seconds() > tag_lost_timeout_) {
      panel.detected = false;
    }
  }
}

void Game2AutoNode::select_target_and_aim()
{
  if (test_alignment_only_) {
    // In test mode, target is updated in tag_detections_callback
    if (active_target_id_ != -1) {
      const auto it = panel_grid_.find(active_target_id_);
      if (it == panel_grid_.end() || !it->second.detected) {
        target_valid_ = false;
      }
    }
    return;
  }

  target_valid_ = false;

  // 1. 各段(Row 0, 1, 2)の未射出タグを走査して候補リスト (TargetCandidate) を構築
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

  std::vector<TargetCandidate> candidates;

  auto compute_candidate_heading_err = [this](double x, double y) {
      const double dx = x - shooter_offset_x_;
      const double dy = y - shooter_offset_y_;
      return std::remainder(std::atan2(dy, dx) - M_PI, 2.0 * M_PI);
    };

  for (int row = 0; row <= 2; ++row) {
    const PanelTagInfo * p0 = nullptr;
    const PanelTagInfo * p1 = nullptr;
    const PanelTagInfo * p2 = nullptr;

    for (const auto & [id, panel] : panel_grid_) {
      if (panel.row == row && !panel.shot_completed && panel.detected) {
        if (panel.col == 0) {
          p0 = &panel;
        } else if (panel.col == 1) {
          p1 = &panel;
        } else if (panel.col == 2) {
          p2 = &panel;
        }
      }
    }

    if (!p0 && !p1 && !p2) {
      continue;
    }

    if (enable_double_panel_midpoint_targeting_) {
      // 3枚残存時: 左側2枚(0&1)の中点
      if (p0 && p1 && p2) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p0->tag_id, p1->tag_id};
        c.x = (p0->x + p1->x) * 0.5;
        c.y = (p0->y + p1->y) * 0.5;
        c.z = (p0->z + p1->z) * 0.5;
        c.heading_err = compute_candidate_heading_err(c.x, c.y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Midpoint #" + std::to_string(p0->tag_id) +
          " & #" + std::to_string(p1->tag_id);
        candidates.push_back(c);
      } else if (p0 && p1) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p0->tag_id, p1->tag_id};
        c.x = (p0->x + p1->x) * 0.5;
        c.y = (p0->y + p1->y) * 0.5;
        c.z = (p0->z + p1->z) * 0.5;
        c.heading_err = compute_candidate_heading_err(c.x, c.y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Midpoint #" + std::to_string(p0->tag_id) +
          " & #" + std::to_string(p1->tag_id);
        candidates.push_back(c);
      } else if (p1 && p2) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {p1->tag_id, p2->tag_id};
        c.x = (p1->x + p2->x) * 0.5;
        c.y = (p1->y + p2->y) * 0.5;
        c.z = (p1->z + p2->z) * 0.5;
        c.heading_err = compute_candidate_heading_err(c.x, c.y);
        c.is_pair = true;
        c.desc = "Row " + std::to_string(row) + " Midpoint #" + std::to_string(p1->tag_id) +
          " & #" + std::to_string(p2->tag_id);
        candidates.push_back(c);
      }
    }

    // 単独残存候補 (ペアが作れなかった、または単独のパネル)
    if (candidates.empty() || candidates.back().row != row) {
      const PanelTagInfo * single = p1 ? p1 : (p0 ? p0 : p2);
      if (single) {
        TargetCandidate c;
        c.row = row;
        c.tag_ids = {single->tag_id};
        c.x = single->x;
        c.y = single->y;
        c.z = single->z;
        c.heading_err = compute_candidate_heading_err(c.x, c.y);
        c.is_pair = false;
        c.desc = "Row " + std::to_string(row) + " Single Tag #" + std::to_string(single->tag_id);
        candidates.push_back(c);
      }
    }
  }

  if (candidates.empty()) {
    return;
  }

  // 2. 最適ターゲット選定 (旋回量・旋回回数の最小化を最優先)
  const TargetCandidate * chosen = nullptr;

  // 戦略A: 垂直スイープ (直前の照準方向と同方位に未射出パネルがあれば、旋回0度・段のみ移行で即撃ち)
  if (enable_vertical_sweep_ && !current_target_tag_ids_.empty()) {
    for (const auto & cand : candidates) {
      if (std::abs(cand.heading_err - target_heading_err_) < (yaw_tolerance_ * 1.5)) {
        chosen = &cand;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "⚡ [Vertical Sweep: ZERO TURNING] Selected vertical target: %s (Heading Diff: %.2f deg)",
          cand.desc.c_str(), std::abs(cand.heading_err - target_heading_err_) * 180.0 / M_PI);
        break;
      }
    }
  }

  // 戦略B: TSP最小角度移動 (現在の方位から最も近い方位のターゲットを選択して旋回量を最小化)
  if (!chosen && enable_nearest_angle_search_) {
    double min_delta_angle = 1e9;
    for (const auto & cand : candidates) {
      const double delta = std::abs(cand.heading_err - target_heading_err_);
      if (delta < min_delta_angle) {
        min_delta_angle = delta;
        chosen = &cand;
      }
    }
  }

  // 戦略C: 同一段・同RPM優先
  if (!chosen && prefer_same_row_first_ && !current_target_tag_ids_.empty()) {
    std::vector<const TargetCandidate *> same_row_candidates;
    for (const auto & cand : candidates) {
      if (cand.row == active_row_) {
        same_row_candidates.push_back(&cand);
      }
    }

    if (!same_row_candidates.empty()) {
      double min_delta_angle = 1e9;
      for (const auto * cand : same_row_candidates) {
        const double delta = std::abs(cand->heading_err - target_heading_err_);
        if (delta < min_delta_angle) {
          min_delta_angle = delta;
          chosen = cand;
        }
      }
    }
  }

  // 戦略D: フォールバック（リスト先頭）
  if (!chosen) {
    chosen = &candidates.front();
  }

  // 3. 確定ターゲットの諸元を反映
  active_row_ = chosen->row;
  current_target_tag_ids_ = chosen->tag_ids;
  target_x_ = chosen->x;
  target_y_ = chosen->y;
  target_z_ = chosen->z;
  target_heading_err_ = chosen->heading_err;
  active_target_id_ = current_target_tag_ids_.empty() ? -1 : current_target_tag_ids_[0];
  target_valid_ = true;

  double next_under_rpm = underbelt_rpm_bottom_;
  double next_upper_rpm = upperbelt_rpm_bottom_;

  switch (active_row_) {
    case 0:
      next_under_rpm = underbelt_rpm_bottom_;
      next_upper_rpm = upperbelt_rpm_bottom_;
      break;
    case 1:
      next_under_rpm = underbelt_rpm_middle_;
      next_upper_rpm = upperbelt_rpm_middle_;
      break;
    case 2:
      next_under_rpm = underbelt_rpm_top_;
      next_upper_rpm = upperbelt_rpm_top_;
      break;
    default:
      next_under_rpm = underbelt_rpm_bottom_;
      next_upper_rpm = upperbelt_rpm_bottom_;
      break;
  }

  if (std::abs(next_under_rpm - last_target_underbelt_rpm_) > 10.0 ||
    std::abs(next_upper_rpm - last_target_upperbelt_rpm_) > 10.0)
  {
    rpm_changed_time_ = now();
    last_target_underbelt_rpm_ = next_under_rpm;
    last_target_upperbelt_rpm_ = next_upper_rpm;
  }

  target_underbelt_rpm_ = next_under_rpm;
  target_upperbelt_rpm_ = next_upper_rpm;
}

void Game2AutoNode::ball_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool prev = ball_detected_;
  ball_detected_ = msg->data;
  if (!prev && ball_detected_) {
    ball_detected_time_ = now();
    RCLCPP_INFO(get_logger(), "⚽ Game2: Ball DETECTED in dribble intake!");
  } else if (prev && !ball_detected_) {
    RCLCPP_INFO(get_logger(), "💨 Game2: Ball LEFT dribble intake (Shot or Lost)!");
  }
}

void Game2AutoNode::control_loop()
{
  const auto current_time = now();
  double dt = (current_time - last_loop_time_).seconds();
  if (dt <= 0.001 || dt > 0.2) {
    dt = 0.05;
  }
  last_loop_time_ = current_time;

  // Publish current state
  robot_msgs::msg::Game2State state_msg;
  state_msg.state = state_;
  state_pub_->publish(state_msg);

  if (!is_enabled_ || emergency_stop_active_ || state_ == robot_msgs::msg::Game2State::STANDBY) {
    last_cmd_wz_ = 0.0;
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f, 0.0f,
      false, false, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  update_panel_states();
  select_target_and_aim();

  // Check if all panels are completed or all balls fired (if limit is enabled)
  bool all_panels_completed = true;
  for (const auto & [id, panel] : panel_grid_) {
    if (!panel.shot_completed) {
      all_panels_completed = false;
      break;
    }
  }

  const bool balls_exhausted = enable_ball_limit_ && (total_shots_fired_ >= max_total_balls_);
  if (all_panels_completed || balls_exhausted) {
    state_ = robot_msgs::msg::Game2State::COMPLETED;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(), 3000,
      "🏆 Game 2: Sequence FINISHED! (Total shots: %d%s, All Panels Cleared: %s)",
      total_shots_fired_,
      enable_ball_limit_ ? ("/" + std::to_string(max_total_balls_)).c_str() : " (No limit)",
      all_panels_completed ? "YES" : "NO");
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f, 0.0f,
      false, false, robot_msgs::msg::ArmPosition::DRIBBLE, true);
    return;
  }

  // Target not detected: Search mode
  if (!target_valid_) {
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;
    last_cmd_wz_ = 0.0;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(), 1000,
      "🔍 [Game2 Search] Waiting for target AprilTags (Active Row: %d)... Standing still",
      active_row_);

    publish_all(
      cmd,
      test_alignment_only_ ? 0.0f : static_cast<float>(target_underbelt_rpm_),
      test_alignment_only_ ? 0.0f : static_cast<float>(target_upperbelt_rpm_),
      false, !test_alignment_only_, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  geometry_msgs::msg::Twist cmd;
  bool shoot_trigger = false;
  uint8_t arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
  const double state_elapsed = (current_time - state_start_time_).seconds();

  switch (state_) {
    case robot_msgs::msg::Game2State::SEARCHING:
    case robot_msgs::msg::Game2State::ALIGNING: {
        state_ = robot_msgs::msg::Game2State::ALIGNING;

        const double heading_err = target_heading_err_;
        const bool is_aligned = (std::abs(heading_err) < yaw_tolerance_);

        if (is_aligned) {
          cmd.angular.z = 0.0;
        } else {
          // Proportional visual error + IMU Gyro damping (PD control)
          double desired_wz = kp_yaw_ * heading_err;
          if (imu_received_ && (current_time - last_imu_time_).seconds() < 0.5) {
            desired_wz -= kd_yaw_ * gyro_z_;
          }

          // Stiction overcoming minimum angular velocity with smooth attenuation near zero
          // 誤差が tolerance の近傍になったら急に min_angular_z で突き抜けないよう滑らかに減速
          const double err_abs = std::abs(heading_err);
          const double slow_zone = yaw_tolerance_ * 3.0;
          double eff_min_wz = min_angular_z_;
          if (err_abs < slow_zone) {
            eff_min_wz = min_angular_z_ * (err_abs / slow_zone);
          }

          if (std::abs(desired_wz) < eff_min_wz) {
            desired_wz = std::copysign(eff_min_wz, desired_wz);
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
        }
        last_cmd_wz_ = cmd.angular.z;

        const bool is_ball_settled = !require_ball_detected_ || (
          ball_detected_ &&
          ((current_time - ball_detected_time_).seconds() >= ball_settle_duration_));

        const bool is_belt_ready =
          ((current_time - rpm_changed_time_).seconds() >= belt_spinup_duration_);

        // Diagnostics output (200ms)
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(), 200,
          "🎯 [Game2 Track Tag #%d (Row %d)] Err: %+.2f deg | Cmd wz: %+.3f rad/s | %s | Ball: %s | Belt: %s",
          active_target_id_, active_row_, heading_err * 180.0 / M_PI, cmd.angular.z,
          is_aligned ? "✨ ALIGNED" : "🔄 TURNING",
          ball_detected_ ? "YES" : "NO",
          is_belt_ready ? "READY" : "SPINUP");

        if (test_alignment_only_) {
          if (is_aligned) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "✨ [Game2 TEST] Target Perfect Aligned! Holding heading.");
          }
        } else {
          if (is_aligned && is_ball_settled && is_belt_ready) {
            RCLCPP_INFO(
              get_logger(),
              "🚀 [Game2] Target Aligned & Belt Stable & Ball Settled! Moving arm to OPEN for shooting.");
            state_ = robot_msgs::msg::Game2State::PREPARING_SHOOT;
            state_start_time_ = current_time;
            shoot_start_time_ = current_time;
          } else if (is_aligned && !is_belt_ready) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "⏳ [Game2] Aligned to Target Tag #%d, waiting for flywheel belt to stabilize (%.0f / %.0f RPM)...",
              active_target_id_, target_underbelt_rpm_, target_upperbelt_rpm_);
          } else if (is_aligned && !is_ball_settled && require_ball_detected_) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1500,
              "⏳ [Game2] Aligned to Target Tag #%d, waiting for ball to settle in dribble...",
              active_target_id_);
          }
        }
        arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
        break;
      }

    case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
        arm_mode = robot_msgs::msg::ArmPosition::OPEN;
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;

        if (state_elapsed >= open_duration_) {
          RCLCPP_INFO(
            get_logger(),
            "🔥 [Game2] Arm OPEN complete. Feeding ball to dual flywheel belt (Under RPM: %.0f, Upper RPM: %.0f)!",
            target_underbelt_rpm_, target_upperbelt_rpm_);
          state_ = robot_msgs::msg::Game2State::SHOOTING;
          state_start_time_ = current_time;
          shoot_start_time_ = current_time;
        }
        break;
      }

    case robot_msgs::msg::Game2State::SHOOTING: {
        arm_mode = robot_msgs::msg::ArmPosition::FEED;
        shoot_trigger = true;
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;

        if (state_elapsed >= shoot_hold_duration_) {
          total_shots_fired_++;
          if (current_target_tag_ids_.size() >= 2) {
            RCLCPP_INFO(
              get_logger(),
              "🚀 [Game2 SPLIT SHOT!] Shot #%d/%d triggered for Tags #%d & #%d. Waiting for impact...",
              total_shots_fired_, max_total_balls_,
              current_target_tag_ids_[0], current_target_tag_ids_[1]);
          } else if (!current_target_tag_ids_.empty()) {
            RCLCPP_INFO(
              get_logger(),
              "🚀 [Game2] Shot #%d/%d triggered for Tag #%d. Waiting for impact...",
              total_shots_fired_, max_total_balls_,
              current_target_tag_ids_[0]);
          }

          // Increment shot attempts for targeted panels
          for (const int tid : current_target_tag_ids_) {
            auto it = panel_grid_.find(tid);
            if (it != panel_grid_.end()) {
              it->second.shot_count++;
            }
          }

          state_ = robot_msgs::msg::Game2State::WAITING_RESULT;
          state_start_time_ = current_time;
          shoot_start_time_ = current_time;
        }
        break;
      }

    case robot_msgs::msg::Game2State::WAITING_RESULT: {
        arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;

        // ── 👁️ 着弾・ノックダウン（AprilTag消失）視覚判定 ──
        if (state_elapsed >= result_wait_duration_) {
          for (const int tid : current_target_tag_ids_) {
            auto it = panel_grid_.find(tid);
            if (it != panel_grid_.end()) {
              // タグが見えなくなっていれば（消失 = 倒れた）ノックダウン成功！
              if (!it->second.detected) {
                it->second.shot_completed = true;
                RCLCPP_INFO(
                  get_logger(),
                  "💥 [Game2 KNOCKDOWN CONFIRMED!] Tag #%d vanished (panel fell down). Marked as completed! (Shot attempts: %d)",
                  tid, it->second.shot_count);
              } else {
                // まだタグが見えている（倒れなかった or ミス）
                if (max_shots_per_panel_ > 0 && it->second.shot_count >= max_shots_per_panel_) {
                  it->second.shot_completed = true;
                  RCLCPP_WARN(
                    get_logger(),
                    "⚠️ [Game2 MAX SHOTS REACHED] Tag #%d reached max attempts (%d/%d). Advancing target.",
                    tid, it->second.shot_count, max_shots_per_panel_);
                } else {
                  RCLCPP_WARN(
                    get_logger(),
                    "⚠️ [Game2 SHOT MISSED / STANDING!] Tag #%d still visible. Will retry until knockdown! (Shot attempts: %d%s)",
                    tid, it->second.shot_count,
                    max_shots_per_panel_ > 0 ? ("/" + std::to_string(max_shots_per_panel_)).c_str() : "");
                }
              }
            }
          }

          // 状態をリセットして次の照準・射出へ
          state_ = robot_msgs::msg::Game2State::ALIGNING;
          state_start_time_ = current_time;
        }
        break;
      }

    case robot_msgs::msg::Game2State::COMPLETED: {
        arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;
        break;
      }

    default:
      break;
  }

  publish_all(
    cmd,
    test_alignment_only_ ? 0.0f : static_cast<float>(target_underbelt_rpm_),
    test_alignment_only_ ? 0.0f : static_cast<float>(target_upperbelt_rpm_),
    test_alignment_only_ ? false : shoot_trigger,
    test_alignment_only_ ? false : true,
    arm_mode, state_ == robot_msgs::msg::Game2State::COMPLETED);
}

void Game2AutoNode::publish_all(
  const geometry_msgs::msg::Twist & cmd_vel,
  float underbelt_rpm,
  float upperbelt_rpm,
  bool shoot_trigger,
  bool dribble_enabled,
  uint8_t arm_mode,
  bool completed)
{
  cmd_vel_pub_->publish(cmd_vel);

  std_msgs::msg::Float32 under_rpm_msg;
  under_rpm_msg.data = underbelt_rpm;
  underbelt_rpm_pub_->publish(under_rpm_msg);

  std_msgs::msg::Float32 upper_rpm_msg;
  upper_rpm_msg.data = upperbelt_rpm;
  upperbelt_rpm_pub_->publish(upper_rpm_msg);

  std_msgs::msg::Bool shoot_msg;
  shoot_msg.data = shoot_trigger;
  shoot_trigger_pub_->publish(shoot_msg);

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
