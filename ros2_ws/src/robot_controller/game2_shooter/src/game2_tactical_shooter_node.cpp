#include "game2_shooter/game2_tactical_shooter_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game2TacticalShooterNode::Game2TacticalShooterNode(const rclcpp::NodeOptions & options)
: Node("game2_tactical_shooter_node", options)
{
  // Declare Parameters
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  tag_prefix_ = this->declare_parameter<std::string>("tag_prefix", "tag16h5:");
  kp_yaw_ = this->declare_parameter<double>("kp_yaw", 0.5);          // Game2用 低感度旋回Pゲイン
  kd_yaw_ = this->declare_parameter<double>("kd_yaw", 0.05);         // IMUジャイロDゲイン (アクティブブレーキ)
  kp_y_ = this->declare_parameter<double>("kp_y", 0.0);            // 横スライドはオフ (旋回アライメント優先)
  kp_dist_ = this->declare_parameter<double>("kp_dist", 0.8);        // 前後距離ゲイン
  max_angular_z_ = this->declare_parameter<double>("max_angular_z", 0.35); // 最大旋回速度制限 (rad/s)
  target_distance_ = this->declare_parameter<double>("target_distance", 1.5); // 1.5m射程
  yaw_tolerance_ = this->declare_parameter<double>("yaw_tolerance", 0.04);   // rad (約2.3度)
  dist_tolerance_ = this->declare_parameter<double>("dist_tolerance", 0.03); // 3cm
  rpm_bottom_ = this->declare_parameter<double>("rpm_bottom", 3000.0);
  rpm_middle_ = this->declare_parameter<double>("rpm_middle", 4500.0);
  rpm_top_ = this->declare_parameter<double>("rpm_top", 6000.0);
  shoot_hold_duration_ = this->declare_parameter<double>("shoot_hold_duration", 0.8);

  // 大会公式シュートパネル Tag ID 設定
  // 上段: [14, 15, 16], 中段: [17, 18, 19], 下段: [20, 21, 22]
  std::vector<int64_t> default_bottom = {20, 21, 22};
  std::vector<int64_t> default_middle = {17, 18, 19};
  std::vector<int64_t> default_top = {14, 15, 16};

  auto bottom_tags = this->declare_parameter<std::vector<int64_t>>("bottom_tags", default_bottom);
  auto middle_tags = this->declare_parameter<std::vector<int64_t>>("middle_tags", default_middle);
  auto top_tags = this->declare_parameter<std::vector<int64_t>>("top_tags", default_top);

  // パネルマップへの登録 (0:Bottom, 1:Middle, 2:Top)
  for (size_t col = 0; col < bottom_tags.size(); ++col) {
    int id = static_cast<int>(bottom_tags[col]);
    panel_grid_[id] = {id, 0, static_cast<int>(col)};
  }
  for (size_t col = 0; col < middle_tags.size(); ++col) {
    int id = static_cast<int>(middle_tags[col]);
    panel_grid_[id] = {id, 1, static_cast<int>(col)};
  }
  for (size_t col = 0; col < top_tags.size(); ++col) {
    int id = static_cast<int>(top_tags[col]);
    panel_grid_[id] = {id, 2, static_cast<int>(col)};
  }

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions & Publishers
  start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/game2/start", 10,
    std::bind(&Game2TacticalShooterNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2TacticalShooterNode::imu_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mecanum/cmd_vel", 10);
  belt_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/belt/target_rpm", 10);
  shoot_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", 10);
  dribble_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/dribble/enabled", 10);
  arm_position_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/dribble/position_mode", 10);
  completed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/game2/completed", 10);
  // game2_shooter -> led_controller: current Game2 sequence state.
  state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
    "/game2/state", rclcpp::QoS(1).reliable().transient_local());

  // Control Loop Timer (20Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2TacticalShooterNode::control_loop, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Game2TacticalShooterNode initialized (DRIBBLE -> OPEN -> FEED Sequential Loading Mode).");
}

void Game2TacticalShooterNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    state_ = State::SEARCHING;
    active_row_ = 0; // 下段からスタート
    RCLCPP_INFO(
      this->get_logger(),
      "▶️ Game 2 START! DRIBBLE -> OPEN -> FEED Auto-Loading Sequence Active.");
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = State::STANDBY;
    RCLCPP_INFO(this->get_logger(), "⏹️ Game 2 STOP! Entering STANDBY mode.");
  }
}

void Game2TacticalShooterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  last_imu_time_ = this->now();
  current_gyro_z_ = msg->angular_velocity.z;
}

void Game2TacticalShooterNode::update_panel_states()
{
  const auto now = this->now();

  for (auto & [tag_id, info] : panel_grid_) {
    std::vector<std::string> candidate_frames = {
      "16h5:" + std::to_string(tag_id),
      "tag16h5:" + std::to_string(tag_id),
      tag_prefix_ + std::to_string(tag_id),
      "tag36h11:" + std::to_string(tag_id)
    };

    bool found = false;
    for (const auto & frame_name : candidate_frames) {
      try {
        geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(base_frame_, frame_name, tf2::TimePointZero);

        info.detected = true;
        info.x = tf.transform.translation.x;
        info.y = tf.transform.translation.y;
        info.z = tf.transform.translation.z;
        info.last_seen = now;
        found = true;
        break;
      } catch (const tf2::TransformException & ex) {
        // Try next candidate frame
      }
    }

    if (!found && (now - info.last_seen).seconds() > 2.0) {
      info.detected = false;
    }
  }
}

void Game2TacticalShooterNode::select_target_and_aim()
{
  target_valid_ = false;

  // 下段(0) -> 中段(1) -> 上段(2) の順に盤面状況を動的評価
  while (active_row_ <= 2) {
    std::vector<PanelTagInfo *> row_panels;
    for (auto & [tag_id, info] : panel_grid_) {
      if (info.row == active_row_) {
        row_panels.push_back(&info);
      }
    }
    std::sort(
      row_panels.begin(), row_panels.end(), [](PanelTagInfo * a, PanelTagInfo * b) {
        return a->col < b->col;
      });

    if (row_panels.empty()) {
      active_row_++;
      continue;
    }

    PanelTagInfo * left = row_panels.size() > 0 ? row_panels[0] : nullptr;
    PanelTagInfo * center = row_panels.size() > 1 ? row_panels[1] : nullptr;
    PanelTagInfo * right = row_panels.size() > 2 ? row_panels[2] : nullptr;

    // 🎯 ルール1 (隣接2枚狙い): 左と中央が隣り合って残っている -> L&Cの境目を狙う！
    if (left && center && left->detected && center->detected) {
      target_x_ = (left->x + center->x) / 2.0;
      target_y_ = (left->y + center->y) / 2.0;
      target_z_ = (left->z + center->z) / 2.0;
      target_valid_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Row %d: Adjacent Left & Center found (Tag %d & %d) -> Rotation Aiming at LC Boundary!",
        active_row_, left->tag_id, center->tag_id);
      break;
    }

    // 🎯 ルール2 (隣接2枚狙い): 中央と右が隣り合って残っている -> C&Rの境目を狙う！
    if (center && right && center->detected && right->detected) {
      target_x_ = (center->x + right->x) / 2.0;
      target_y_ = (center->y + right->y) / 2.0;
      target_z_ = (center->z + right->z) / 2.0;
      target_valid_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Row %d: Adjacent Center & Right found (Tag %d & %d) -> Rotation Aiming at CR Boundary!",
        active_row_, center->tag_id, right->tag_id);
      break;
    }

    // 🎯 ルール3 (1枚孤立狙い): 倒れ残りなどで1枚だけ残っている -> その1枚の「真芯」を狙う！
    for (auto * panel : row_panels) {
      if (panel && panel->detected) {
        target_x_ = panel->x;
        target_y_ = panel->y;
        target_z_ = panel->z;
        target_valid_ = true;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(), 2000,
          "Row %d: Single isolated panel Tag %d -> Rotation Pinpoint Aiming at Center!", active_row_,
          panel->tag_id);
        break;
      }
    }

    if (target_valid_) {
      break;
    }

    // この段のパネルが全滅していれば次の段へ！
    active_row_++;
  }

  // 段に応じた RPM 設定
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
  geometry_msgs::msg::Twist cmd_vel;
  std_msgs::msg::Float32 rpm_msg;
  std_msgs::msg::Bool trigger_msg;
  std_msgs::msg::Bool dribble_msg;
  std_msgs::msg::UInt8 arm_pos_msg;
  std_msgs::msg::Bool completed_msg;

  trigger_msg.data = false;
  std_msgs::msg::UInt8 state_msg;
  state_msg.data = static_cast<uint8_t>(state_);
  state_pub_->publish(state_msg);

  dribble_msg.data = false;
  arm_pos_msg.data = 0; // 0: DRIBBLE
  completed_msg.data = false;

  // ボタンが押されていない (STANDBY) の時は制御を停止
  if (!is_enabled_ || state_ == State::STANDBY) {
    rpm_msg.data = 0.0f;
    cmd_vel_pub_->publish(cmd_vel);
    belt_rpm_pub_->publish(rpm_msg);
    shoot_trigger_pub_->publish(trigger_msg);
    dribble_enabled_pub_->publish(dribble_msg);
    arm_position_pub_->publish(arm_pos_msg);
    completed_pub_->publish(completed_msg);
    return;
  }

  // Game 2 実行中はドリブラー常時ON (手動で置かれたボールを自動保持・引き込み)
  dribble_msg.data = true;

  update_panel_states();
  select_target_and_aim();

  rpm_msg.data = static_cast<float>(target_rpm_);

  // 全9枚のパネルをすべて倒した場合 (Game 2 完全クリア！)
  if (active_row_ > 2) {
    state_ = State::COMPLETED;
    completed_msg.data = true;
    rpm_msg.data = 0.0f;
    dribble_msg.data = false;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "🏆 GAME 2 ALL PANELS CLEARED! PERFECT VICTORY! 🏆");

    cmd_vel_pub_->publish(cmd_vel);
    belt_rpm_pub_->publish(rpm_msg);
    shoot_trigger_pub_->publish(trigger_msg);
    dribble_enabled_pub_->publish(dribble_msg);
    arm_position_pub_->publish(arm_pos_msg);
    completed_pub_->publish(completed_msg);
    return;
  }

  if (!target_valid_) {
    state_ = State::SEARCHING;
    // ターゲット探索中：マイルドな低速旋回 (0.2 rad/s) ＆ DRIBBLE位置でボール保持
    cmd_vel.angular.z = 0.2;
    arm_pos_msg.data = 0; // 0: DRIBBLE
    cmd_vel_pub_->publish(cmd_vel);
    belt_rpm_pub_->publish(rpm_msg);
    shoot_trigger_pub_->publish(trigger_msg);
    dribble_enabled_pub_->publish(dribble_msg);
    arm_position_pub_->publish(arm_pos_msg);
    completed_pub_->publish(completed_msg);
    return;
  }

  const double dist_err = target_x_ - target_distance_;
  const double y_err = target_y_;

  switch (state_) {
    case State::SEARCHING:
    case State::ALIGNING: {
        state_ = State::ALIGNING;

        // 横スライド補正はオフ (100% 旋回のみでエイム)
        cmd_vel.linear.y = 0.0;
        // 前後距離補正
        cmd_vel.linear.x = kp_dist_ * dist_err;
        arm_pos_msg.data = 0; // STEP 1: DRIBBLE位置でボールを回転保持しながらアライメント！

        // カメラ角度誤差 (P項) ＋ IMUジャイロアクティブブレーキ (D項) によるハイブリッド旋回制御
        double camera_p_term = -kp_yaw_ * y_err;
        double imu_d_term = 0.0;

        // IMU受信中（過去1秒以内に受信あり）なら、ジャイロ反力をダンパーブレーキとして利用！
        if (imu_received_ && (this->now() - last_imu_time_).seconds() < 1.0) {
          imu_d_term = -kd_yaw_ * current_gyro_z_;
        }

        double raw_wz = camera_p_term + imu_d_term;
        cmd_vel.angular.z = std::clamp(raw_wz, -max_angular_z_, max_angular_z_);

        // 照準完了チェック (誤差判定)
        if (std::abs(y_err) < yaw_tolerance_ && std::abs(dist_err) < dist_tolerance_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Game2 Target Alignment ACQUIRED! STEP 2: Transitioning to OPEN position...");
          state_ = State::PREPARING_SHOOT;
          shoot_start_time_ = this->now();
        }
        break;
      }

    case State::PREPARING_SHOOT: {
        // STEP 2: アームを一度パカッと開く (1: OPEN) でボールの拘束を解放
        cmd_vel = geometry_msgs::msg::Twist{};
        arm_pos_msg.data = 1; // 1: OPEN

        if ((this->now() - shoot_start_time_).seconds() > 0.3) {
          RCLCPP_INFO(
            this->get_logger(), "STEP 3: Transitioning to FEED position for belt loading...");
          state_ = State::SHOOTING;
          shoot_start_time_ = this->now();
        }
        break;
      }

    case State::SHOOTING: {
        // STEP 3: アームを FEED (2: FEED) へ倒し込み、高速ベルトへボールを自動押し込み装填＆射出！
        cmd_vel = geometry_msgs::msg::Twist{};
        trigger_msg.data = true; // ベルト射出トリガーオン！
        arm_pos_msg.data = 2;  // 2: FEED

        if ((this->now() - shoot_start_time_).seconds() > shoot_hold_duration_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Game2 Ball Fired! Resetting arm to DRIBBLE position for next ball...");
          state_ = State::WAITING_RESULT;
          shoot_start_time_ = this->now();
        }
        break;
      }

    case State::WAITING_RESULT: {
        cmd_vel = geometry_msgs::msg::Twist{};
        arm_pos_msg.data = 0; // 次のボール受け取り用にアームを 0: DRIBBLE 位置へリセット
        if ((this->now() - shoot_start_time_).seconds() > 1.2) {
          state_ = State::ALIGNING;
        }
        break;
      }

    default:
      break;
  }

  cmd_vel_pub_->publish(cmd_vel);
  belt_rpm_pub_->publish(rpm_msg);
  shoot_trigger_pub_->publish(trigger_msg);
  dribble_enabled_pub_->publish(dribble_msg);
  arm_position_pub_->publish(arm_pos_msg);
  completed_pub_->publish(completed_msg);
}

}  // namespace robot_controller
