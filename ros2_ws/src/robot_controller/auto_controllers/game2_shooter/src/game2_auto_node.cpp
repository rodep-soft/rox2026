#include "game2_shooter/game2_auto_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game2AutoNode::Game2AutoNode(const rclcpp::NodeOptions & options)
: Node("game2_auto_node", options)
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");
  kp_yaw_ = declare_parameter<double>("kp_yaw", 0.5);
  kd_yaw_ = declare_parameter<double>("kd_yaw", 0.05);
  kp_dist_ = declare_parameter<double>("kp_dist", 0.8);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.35);
  target_distance_ = declare_parameter<double>("target_distance", 4.0);
  camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.265);
  camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.035);
  camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.193);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.02);
  dist_tolerance_ = declare_parameter<double>("dist_tolerance", 0.03);
  rpm_bottom_ = declare_parameter<double>("rpm_bottom", 3000.0);
  rpm_middle_ = declare_parameter<double>("rpm_middle", 4500.0);
  rpm_top_ = declare_parameter<double>("rpm_top", 6000.0);
  shoot_hold_duration_ = declare_parameter<double>("shoot_hold_duration", 0.8);
  test_alignment_only_ = declare_parameter<bool>("test_alignment_only", false);

  // シュートパネルのTag IDを段ごとに登録 (row: 0=下段, 1=中段, 2=上段)
  const std::vector<int64_t> default_bottom = {20, 21, 22};
  const std::vector<int64_t> default_middle = {17, 18, 19};
  const std::vector<int64_t> default_top = {14, 15, 16};
  const auto bottom_tags = declare_parameter<std::vector<int64_t>>("bottom_tags", default_bottom);
  const auto middle_tags = declare_parameter<std::vector<int64_t>>("middle_tags", default_middle);
  const auto top_tags = declare_parameter<std::vector<int64_t>>("top_tags", default_top);

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

  const auto init_now = this->now();
  ball_detected_time_ = init_now;
  shoot_start_time_ = init_now;
  last_imu_time_ = init_now;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "/detections", 10,
    std::bind(&Game2AutoNode::tag_detections_callback, this, std::placeholders::_1));
  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game2/command_start", 10,
    std::bind(&Game2AutoNode::start_callback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2AutoNode::imu_callback, this, std::placeholders::_1));
  ball_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/ball_detected", 10,
    std::bind(&Game2AutoNode::ball_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
  belt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/belt/command_rpm", 10);
  shoot_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", 10);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", 10);
  arm_position_pub_ =
    create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", 10);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game2/completed", 10);
  state_pub_ = create_publisher<robot_msgs::msg::Game2State>(
    "/game2/state", rclcpp::QoS(1).reliable().transient_local());

  // 制御ループ 20 Hz
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2AutoNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Game2AutoNode initialized.");
}

void Game2AutoNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    active_row_ = 0;
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    const auto start_time = this->now();
    ball_detected_time_ = start_time;
    shoot_start_time_ = start_time;
    last_imu_time_ = start_time;
    RCLCPP_INFO(get_logger(), "Game 2 START. IMU Yaw Zero-Reset (Offset: %.3f rad).", yaw_offset_);
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    RCLCPP_INFO(get_logger(), "Game 2 STOP.");
  }
}

void Game2AutoNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  last_imu_time_ = now();

  // 1. 角速度 (rad/s)
  gyro_x_ = msg->angular_velocity.x;
  gyro_y_ = msg->angular_velocity.y;
  gyro_z_ = msg->angular_velocity.z;

  // 2. 加速度 (m/s^2) - 射出反動の衝撃検知用
  accel_x_ = msg->linear_acceleration.x;
  accel_y_ = msg->linear_acceleration.y;
  accel_z_ = msg->linear_acceleration.z;

  // 3. 姿勢 クォータニオン -> オイラー角 (Roll, Pitch, Yaw) 変換
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;

  // Roll (x-axis rotation)
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll_ = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0) {
    pitch_ = std::copysign(M_PI / 2.0, sinp);
  } else {
    pitch_ = std::asin(sinp);
  }

  // Yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
}

void Game2AutoNode::tag_detections_callback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  const auto current_time = now();

  for (const auto & detection : msg->detections) {
    const int id = detection.id;
    auto it = panel_grid_.find(id);
    if (it != panel_grid_.end()) {
      it->second.last_seen = current_time;
      it->second.detected = true;

      // 1080p 画像中心 (cx=960) からのピクセルズレ
      const double pixel_x_err = static_cast<double>(detection.centre.x) - 960.0;
      
      // カメラ視野角から見たタグの光軸角度 (rad) (水平FOV ≈ 105° = 1.83 rad, 焦点距離 fx ≈ 740px)
      const double tag_angle_cam = - std::atan2(pixel_x_err, 740.0);
      const double estimated_dist = 2.5; // [m] 推定距離

      // カメラ座標系でのタグ位置 (前方 X_cam, 左 Y_cam)
      const double x_cam = estimated_dist * std::cos(tag_angle_cam);
      const double y_cam = estimated_dist * std::sin(tag_angle_cam);

      // 📐 base_link (ロボット旋回中心) 基準に変換 (カメラ位置 offset_x, offset_y, offset_z を加算)
      it->second.x = x_cam + camera_offset_x_;
      it->second.y = y_cam + camera_offset_y_;
      it->second.z = camera_offset_z_;

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "📷 [AprilTag Track] Tag ID %d: PixelErr=%.1f px -> RobotBase (X=%.2fm, Y=%.3fm, HeadingErr=%.2f deg)",
        id, pixel_x_err, it->second.x, it->second.y, std::atan2(it->second.y, it->second.x) * 180.0 / M_PI);
    }
  }
}

void Game2AutoNode::update_panel_states()
{
  const auto current_time = now();

  for (auto & [id, panel] : panel_grid_) {
    // 1. TF が利用可能な場合は TF で精密更新
    const std::string frame1 = tag_prefix_ + std::to_string(id);
    const std::string frame2 = "tag_" + std::to_string(id);

    for (const auto & frame : {frame1, frame2}) {
      try {
        const auto tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
        panel.x = tf.transform.translation.x;
        panel.y = tf.transform.translation.y;
        panel.z = tf.transform.translation.z;
        panel.detected = true;
        panel.last_seen = current_time;
        break;
      } catch (const tf2::TransformException &) {
        // TF が無ければ tag_detections_callback の direct 検出結果をそのまま使用
      }
    }

    // 2. 1.5秒以上どちらからも見えなくなったらロスト判定
    if ((current_time - panel.last_seen).seconds() > 1.5) {
      panel.detected = false;
    }
  }
}

void Game2AutoNode::select_target_and_aim()
{
  target_valid_ = false;

  // 1. テストモード時は直近 0.5秒以内に検出されたタグの中から最新のものをターゲットに選定
  if (test_alignment_only_) {
    int best_id = -1;
    rclcpp::Time newest_time = rclcpp::Time(0, 0, RCL_ROS_TIME);

    for (const auto & [id, panel] : panel_grid_) {
      if (panel.detected && (now() - panel.last_seen).seconds() < 0.5) {
        if (panel.last_seen > newest_time) {
          newest_time = panel.last_seen;
          best_id = id;
        }
      }
    }

    if (best_id != -1) {
      const auto & target = panel_grid_[best_id];
      target_x_ = target.x;
      target_y_ = target.y;
      target_z_ = target.z;
      target_valid_ = true;
      active_row_ = target.row;
      return;
    }
  }

  // 2. 本番モード: 下段(0) -> 中段(1) -> 上段(2) の順にクリア
  for (int row = active_row_; row <= 2; ++row) {
    int best_id = -1;
    double min_dist_sq = 1e9;

    for (const auto & [id, panel] : panel_grid_) {
      if (panel.row == row && panel.detected) {
        const double dist_sq = panel.x * panel.x + panel.y * panel.y;
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          best_id = id;
        }
      }
    }

    if (best_id != -1) {
      active_row_ = row;
      const auto & target = panel_grid_[best_id];
      target_x_ = target.x;
      target_y_ = target.y;
      target_z_ = target.z;
      target_valid_ = true;

      switch (row) {
        case 0: target_rpm_ = rpm_bottom_; break;
        case 1: target_rpm_ = rpm_middle_; break;
        case 2: target_rpm_ = rpm_top_; break;
        default: target_rpm_ = rpm_bottom_; break;
      }
      return;
    }
  }
}

void Game2AutoNode::ball_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool prev = ball_detected_;
  ball_detected_ = msg->data;
  if (!prev && ball_detected_) {
    ball_detected_time_ = now();
    RCLCPP_INFO(get_logger(), "Game2: Ball detected in dribble!");
  }
}

void Game2AutoNode::control_loop()
{
  robot_msgs::msg::Game2State state_msg;
  state_msg.state = state_;
  state_pub_->publish(state_msg);

  if (!is_enabled_ || state_ == robot_msgs::msg::Game2State::STANDBY) {
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f,
      false, false, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  update_panel_states();
  select_target_and_aim();

  if (active_row_ > 2) {
    state_ = robot_msgs::msg::Game2State::COMPLETED;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Game 2: all panels cleared.");
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f,
      false, false, robot_msgs::msg::ArmPosition::DRIBBLE, true);
    return;
  }

  if (!target_valid_) {
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;  // タグ未検出時は勝手に回転せず静止待機
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "🔍 [Game2 Search] Waiting for target AprilTags (Row: %d)... Standing still (0.0 rad/s)", active_row_);
    publish_all(
      cmd, test_alignment_only_ ? 0.0f : static_cast<float>(target_rpm_),
      false, !test_alignment_only_, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  const double dist_err = target_x_ - target_distance_;
  const double y_err = target_y_;
  geometry_msgs::msg::Twist cmd;
  bool shoot_trigger = false;
  uint8_t arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;

  switch (state_) {
    case robot_msgs::msg::Game2State::SEARCHING:
    case robot_msgs::msg::Game2State::ALIGNING: {
        state_ = robot_msgs::msg::Game2State::ALIGNING;

        // 📐 ロボット旋回中心から見た真の目標角度誤差 (rad)
        const double heading_err = std::atan2(y_err, target_x_);
        const bool is_aligned = (std::abs(heading_err) < yaw_tolerance_);

        if (is_aligned) {
          cmd.angular.z = 0.0;  // 中心にピタッと一致したら完全制動
        } else {
          // 📐 正しい回転方向: 左にあるタグ(heading_err > 0)へは反時計回り(+wz)で向く
          double wz = kp_yaw_ * heading_err;
          if (imu_received_ && (now() - last_imu_time_).seconds() < 1.0) {
            wz -= kd_yaw_ * gyro_z_;
          }
          // 静止摩擦を突破する最小角速度 (0.12 rad/s) を保証
          const double min_angular_z = 0.12;
          if (std::abs(wz) < min_angular_z) {
            wz = std::copysign(min_angular_z, wz);
          }
          cmd.angular.z = std::clamp(wz, -max_angular_z_, max_angular_z_);
        }

        const bool is_ball_settled = ball_detected_ &&
          ((now() - ball_detected_time_).seconds() >= ball_settle_duration_);

        // 📊 リアルタイム詳細デバッグ出力 (500ms周期)
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "🎯 [Game2 Track] Target: x=%.2fm, y=%.3fm (AngleErr: %.2f deg) | Angular Cmd: %.3f rad/s | Aligned: %s",
          target_x_, y_err, heading_err * 180.0 / M_PI, cmd.angular.z, is_aligned ? "YES (MATCH)" : "NO (TURNING)");

        if (test_alignment_only_) {
          if (is_aligned) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "✨ [Game2 TEST] Target Perfect Aligned! Holding heading.");
          }
        } else {
          if (is_aligned && is_ball_settled) {
            RCLCPP_INFO(get_logger(), "Game2: Aligned & Ball Settled! Moving arm to OPEN.");
            state_ = robot_msgs::msg::Game2State::PREPARING_SHOOT;
            shoot_start_time_ = now();
          } else if (is_aligned && !ball_detected_) {
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Game2: Aligned to Target, waiting for ball intake...");
          }
        }
        arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
        break;
      }

    case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
        arm_mode = robot_msgs::msg::ArmPosition::OPEN;
        if ((now() - shoot_start_time_).seconds() > 0.3) {
          RCLCPP_INFO(get_logger(), "Game2: arm open. Moving to FEED.");
          state_ = robot_msgs::msg::Game2State::SHOOTING;
          shoot_start_time_ = now();
        }
        break;
      }

    case robot_msgs::msg::Game2State::SHOOTING: {
        arm_mode = robot_msgs::msg::ArmPosition::FEED;
        shoot_trigger = true;
        if ((now() - shoot_start_time_).seconds() > shoot_hold_duration_) {
          RCLCPP_INFO(get_logger(), "Game2: shot complete. Returning arm to DRIBBLE.");
          state_ = robot_msgs::msg::Game2State::WAITING_RESULT;
          shoot_start_time_ = now();
        }
        break;
      }

    case robot_msgs::msg::Game2State::WAITING_RESULT: {
        arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
        if ((now() - shoot_start_time_).seconds() > 1.2) {
          state_ = robot_msgs::msg::Game2State::ALIGNING;
        }
        break;
      }

    default:
      break;
  }

  publish_all(
    cmd, test_alignment_only_ ? 0.0f : static_cast<float>(target_rpm_),
    test_alignment_only_ ? false : shoot_trigger,
    test_alignment_only_ ? false : true,
    arm_mode, false);
}

void Game2AutoNode::publish_all(
  const geometry_msgs::msg::Twist & cmd_vel,
  float belt_rpm,
  bool shoot_trigger,
  bool dribble_enabled,
  uint8_t arm_mode,
  bool completed)
{
  cmd_vel_pub_->publish(cmd_vel);

  std_msgs::msg::Float32 rpm_msg;
  rpm_msg.data = belt_rpm;
  belt_rpm_pub_->publish(rpm_msg);

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
