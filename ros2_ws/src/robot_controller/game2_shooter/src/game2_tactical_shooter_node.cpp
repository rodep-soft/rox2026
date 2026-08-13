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
  kp_y_              = declare_parameter<double>("kp_y", 0.0);
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
    "/game2/start", 10,
    std::bind(&Game2TacticalShooterNode::start_callback, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2TacticalShooterNode::imu_callback, this, std::placeholders::_1));

  cmd_vel_pub_       = create_publisher<geometry_msgs::msg::Twist>("/mecanum/cmd_vel", 10);
  belt_rpm_pub_      = create_publisher<std_msgs::msg::Float32>("/belt/target_rpm", 10);
  shoot_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", 10);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/enabled", 10);
  arm_position_pub_  = create_publisher<std_msgs::msg::UInt8>("/dribble/position_mode", 10);
  completed_pub_     = create_publisher<std_msgs::msg::Bool>("/game2/completed", 10);
  // game2_shooter -> led_controller: シーケンス状態
  state_pub_ = create_publisher<std_msgs::msg::UInt8>(
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
    state_ = State::SEARCHING;
    active_row_ = 0;
    RCLCPP_INFO(get_logger(), "Game 2 START.");
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = State::STANDBY;
    RCLCPP_INFO(get_logger(), "Game 2 STOP.");
  }
}

void Game2TacticalShooterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_   = true;
  last_imu_time_  = now();
  current_gyro_z_ = msg->angular_velocity.z;
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
  // 状態をpublish
  std_msgs::msg::UInt8 state_msg;
  state_msg.data = static_cast<uint8_t>(state_);
  state_pub_->publish(state_msg);

  // STANDBY 中は全停止
  if (!is_enabled_ || state_ == State::STANDBY) {
    publish_all(
      geometry_msgs::msg::Twist{}, /*belt_rpm=*/0.0f,
      /*shoot=*/false, /*dribble=*/false, /*arm=*/0, /*completed=*/false);
    return;
  }

  update_panel_states();
  select_target_and_aim();

  // 全9枚クリア
  if (active_row_ > 2) {
    state_ = State::COMPLETED;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Game 2: all panels cleared.");
    publish_all(
      geometry_msgs::msg::Twist{}, /*belt_rpm=*/0.0f,
      /*shoot=*/false, /*dribble=*/false, /*arm=*/0, /*completed=*/true);
    return;
  }

  // ターゲット未検出 → ゆっくり旋回しながら探索
  if (!target_valid_) {
    state_ = State::SEARCHING;
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.2;
    publish_all(cmd, static_cast<float>(target_rpm_),
      /*shoot=*/false, /*dribble=*/true, /*arm=*/0, /*completed=*/false);
    return;
  }

  const double dist_err = target_x_ - target_distance_;
  const double y_err    = target_y_;
  geometry_msgs::msg::Twist cmd;
  bool shoot_trigger = false;
  uint8_t arm_mode = 0;  // 0=DRIBBLE, 1=OPEN, 2=FEED

  switch (state_) {
    case State::SEARCHING:
    case State::ALIGNING: {
      state_ = State::ALIGNING;
      cmd.linear.x = kp_dist_ * dist_err;
      cmd.linear.y = 0.0;

      // P(カメラ誤差) + D(ジャイロ) による旋回制御
      double wz = -kp_yaw_ * y_err;
      if (imu_received_ && (now() - last_imu_time_).seconds() < 1.0) {
        wz -= kd_yaw_ * current_gyro_z_;
      }
      cmd.angular.z = std::clamp(wz, -max_angular_z_, max_angular_z_);

      if (std::abs(y_err) < yaw_tolerance_ && std::abs(dist_err) < dist_tolerance_) {
        RCLCPP_INFO(get_logger(), "Game2: aligned. Moving arm to OPEN.");
        state_ = State::PREPARING_SHOOT;
        shoot_start_time_ = now();
      }
      arm_mode = 0;
      break;
    }

    case State::PREPARING_SHOOT: {
      // アームをOPENに開いてボールの拘束を解放
      arm_mode = 1;
      if ((now() - shoot_start_time_).seconds() > 0.3) {
        RCLCPP_INFO(get_logger(), "Game2: arm open. Moving to FEED.");
        state_ = State::SHOOTING;
        shoot_start_time_ = now();
      }
      break;
    }

    case State::SHOOTING: {
      // FEEDに倒し込んでベルトへ押し込み
      arm_mode = 2;
      shoot_trigger = true;
      if ((now() - shoot_start_time_).seconds() > shoot_hold_duration_) {
        RCLCPP_INFO(get_logger(), "Game2: shot complete. Returning arm to DRIBBLE.");
        state_ = State::WAITING_RESULT;
        shoot_start_time_ = now();
      }
      break;
    }

    case State::WAITING_RESULT: {
      // 次のボールを受け取る位置にアームを戻す
      arm_mode = 0;
      if ((now() - shoot_start_time_).seconds() > 1.2) {
        state_ = State::ALIGNING;
      }
      break;
    }

    default:
      break;
  }

  publish_all(cmd, static_cast<float>(target_rpm_),
    shoot_trigger, /*dribble=*/true, arm_mode, /*completed=*/false);
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

  std_msgs::msg::UInt8 arm_msg;
  arm_msg.data = arm_mode;
  arm_position_pub_->publish(arm_msg);

  std_msgs::msg::Bool completed_msg;
  completed_msg.data = completed;
  completed_pub_->publish(completed_msg);
}

}  // namespace robot_controller
