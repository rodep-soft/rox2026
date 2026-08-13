#include "game2_shooter/game2_tactical_shooter_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game2TacticalShooterNode::Game2TacticalShooterNode(const rclcpp::NodeOptions & options)
: Node("game2_tactical_shooter_node", options)
{
  base_frame_        = declare_parameter<std::string>("base_frame", "base_link");
  tag_prefix_        = declare_parameter<std::string>("tag_prefix", "tag16h5:");
  kp_yaw_            = declare_parameter<double>("kp_yaw", 0.5);
  kd_yaw_            = declare_parameter<double>("kd_yaw", 0.05);
  kp_dist_           = declare_parameter<double>("kp_dist", 0.8);
  max_angular_z_     = declare_parameter<double>("max_angular_z", 0.35);
  target_distance_   = declare_parameter<double>("target_distance", 1.5);
  yaw_tolerance_     = declare_parameter<double>("yaw_tolerance", 0.04);
  dist_tolerance_    = declare_parameter<double>("dist_tolerance", 0.03);
  rpm_bottom_        = declare_parameter<double>("rpm_bottom", 3000.0);
  rpm_middle_        = declare_parameter<double>("rpm_middle", 4500.0);
  rpm_top_           = declare_parameter<double>("rpm_top", 6000.0);
  shoot_hold_duration_ = declare_parameter<double>("shoot_hold_duration", 0.8);

  // シュートパネルのTag IDを段ごとに登録 (row: 0=下段, 1=中段, 2=上段)
  const std::vector<int64_t> default_bottom = {20, 21, 22};
  const std::vector<int64_t> default_middle = {17, 18, 19};
  const std::vector<int64_t> default_top    = {14, 15, 16};
  const auto bottom_tags = declare_parameter<std::vector<int64_t>>("bottom_tags", default_bottom);
  const auto middle_tags = declare_parameter<std::vector<int64_t>>("middle_tags", default_middle);
  const auto top_tags    = declare_parameter<std::vector<int64_t>>("top_tags", default_top);

  auto register_row = [this](const std::vector<int64_t> & tags, int row) {
    for (size_t col = 0; col < tags.size(); ++col) {
      const int id = static_cast<int>(tags[col]);
      panel_grid_[id] = {id, row, static_cast<int>(col)};
    }
  };
  register_row(bottom_tags, 0);
  register_row(middle_tags, 1);
  register_row(top_tags, 2);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game2/command_start", 10,
    std::bind(&Game2TacticalShooterNode::start_callback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2TacticalShooterNode::imu_callback, this, std::placeholders::_1));

  cmd_vel_pub_       = create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
  belt_rpm_pub_      = create_publisher<std_msgs::msg::Float32>("/belt/command_rpm", 10);
  shoot_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", 10);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", 10);
  arm_position_pub_  = create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", 10);
  completed_pub_     = create_publisher<std_msgs::msg::Bool>("/game2/completed", 10);
  state_pub_         = create_publisher<robot_msgs::msg::Game2State>(
    "/game2/state", rclcpp::QoS(1).reliable().transient_local());

  // 制御ループ 20 Hz
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2TacticalShooterNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Game2TacticalShooterNode initialized.");
}

void Game2TacticalShooterNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    active_row_ = 0;
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    RCLCPP_INFO(get_logger(), "Game 2 START. IMU Yaw Zero-Reset (Offset: %.3f rad).", yaw_offset_);
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    RCLCPP_INFO(get_logger(), "Game 2 STOP.");
  }
}

void Game2TacticalShooterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_  = true;
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

void Game2TacticalShooterNode::update_panel_states()
{
  const auto current_time = now();

  // tag_prefix_ を使った1種類のフレーム名で lookup する
  for (auto & [tag_id, info] : panel_grid_) {
    const std::string frame = tag_prefix_ + std::to_string(tag_id);
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      info.detected  = true;
      info.x         = tf.transform.translation.x;
      info.y         = tf.transform.translation.y;
      info.z         = tf.transform.translation.z;
      info.last_seen = current_time;
    } catch (const tf2::TransformException &) {
      if ((current_time - info.last_seen).seconds() > 2.0) {
        info.detected = false;
      }
    }
  }
}

void Game2TacticalShooterNode::select_target_and_aim()
{
  target_valid_ = false;

  // 下段(0) → 中段(1) → 上段(2) の順に検索
  while (active_row_ <= 2) {
    // 現在段のパネルをcol順に並べる
    std::vector<PanelTagInfo *> row_panels;
    for (auto & [tag_id, info] : panel_grid_) {
      if (info.row == active_row_) {
        row_panels.push_back(&info);
      }
    }
    std::sort(row_panels.begin(), row_panels.end(), [](const PanelTagInfo * a,
      const PanelTagInfo * b) {
      return a->col < b->col;
    });

    if (row_panels.empty()) {
      ++active_row_;
      continue;
    }

    PanelTagInfo * left   = row_panels.size() > 0 ? row_panels[0] : nullptr;
    PanelTagInfo * center = row_panels.size() > 1 ? row_panels[1] : nullptr;
    PanelTagInfo * right  = row_panels.size() > 2 ? row_panels[2] : nullptr;

    // 隣接2枚狙い: 左・中央 → 境界の中点
    if (left && center && left->detected && center->detected) {
      target_x_ = (left->x + center->x) / 2.0;
      target_y_ = (left->y + center->y) / 2.0;
      target_z_ = (left->z + center->z) / 2.0;
      target_valid_ = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Row %d: Left & Center detected -> aiming at boundary (Tag %d & %d)",
        active_row_, left->tag_id, center->tag_id);
      break;
    }

    // 隣接2枚狙い: 中央・右 → 境界の中点
    if (center && right && center->detected && right->detected) {
      target_x_ = (center->x + right->x) / 2.0;
      target_y_ = (center->y + right->y) / 2.0;
      target_z_ = (center->z + right->z) / 2.0;
      target_valid_ = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Row %d: Center & Right detected -> aiming at boundary (Tag %d & %d)",
        active_row_, center->tag_id, right->tag_id);
      break;
    }

    // 孤立1枚狙い: 残っている任意のパネルの中心
    for (auto * panel : row_panels) {
      if (panel && panel->detected) {
        target_x_ = panel->x;
        target_y_ = panel->y;
        target_z_ = panel->z;
        target_valid_ = true;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Row %d: Single panel Tag %d -> aiming at center", active_row_, panel->tag_id);
        break;
      }
    }

    if (target_valid_) {
      break;
    }

    // この段のパネルが全滅 → 次の段へ
    ++active_row_;
  }

  // 段に応じたベルト RPM を設定
  if (active_row_ == 0) {
    target_rpm_ = rpm_bottom_;
  } else if (active_row_ == 1) {
    target_rpm_ = rpm_middle_;
  } else {
    target_rpm_ = rpm_top_;
  }
}

void Game2TacticalShooterNode::control_loop()
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
    cmd.angular.z = 0.2;
    publish_all(cmd, static_cast<float>(target_rpm_),
      false, true, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  const double dist_err = target_x_ - target_distance_;
  const double y_err    = target_y_;
  geometry_msgs::msg::Twist cmd;
  bool shoot_trigger = false;
  uint8_t arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;

  switch (state_) {
    case robot_msgs::msg::Game2State::SEARCHING:
    case robot_msgs::msg::Game2State::ALIGNING: {
      state_ = robot_msgs::msg::Game2State::ALIGNING;
      cmd.linear.x = kp_dist_ * dist_err;
      cmd.linear.y = 0.0;

      double wz = -kp_yaw_ * y_err;
      if (imu_received_ && (now() - last_imu_time_).seconds() < 1.0) {
        wz -= kd_yaw_ * gyro_z_;
      }
      cmd.angular.z = std::clamp(wz, -max_angular_z_, max_angular_z_);

      if (std::abs(y_err) < yaw_tolerance_ && std::abs(dist_err) < dist_tolerance_) {
        RCLCPP_INFO(get_logger(), "Game2: aligned. Moving arm to OPEN.");
        state_ = robot_msgs::msg::Game2State::PREPARING_SHOOT;
        shoot_start_time_ = now();
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

  publish_all(cmd, static_cast<float>(target_rpm_),
    shoot_trigger, true, arm_mode, false);
}

void Game2TacticalShooterNode::publish_all(
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
