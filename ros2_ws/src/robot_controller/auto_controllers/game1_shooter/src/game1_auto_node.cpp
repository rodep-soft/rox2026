#include "game1_shooter/game1_auto_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game1AutoNode::Game1AutoNode(const rclcpp::NodeOptions & options)
: Node("game1_auto_node", options)
{
  kp_linear_ = declare_parameter<double>("kp_linear", 1.0);
  kp_angular_ = declare_parameter<double>("kp_angular", 2.0);
  max_linear_vel_ = declare_parameter<double>("max_linear_vel", 3.5);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 3.5);
  pos_tolerance_ = declare_parameter<double>("pos_tolerance", 0.05);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.05);

  // テストモード設定
  test_mode_ = declare_parameter<bool>("test_mode", false);
  test_dist_x_ = declare_parameter<double>("test_dist_x", 1.0);
  test_dist_y_ = declare_parameter<double>("test_dist_y", 0.0);
  test_max_vel_ = declare_parameter<double>("test_max_vel", 3.5);

  // フィールドサイド設定 ("left" or "right" / 左右反転フィールド対応)
  field_side_ = declare_parameter<std::string>("field_side", "left");
  const double mirror_x = (field_side_ == "right" || field_side_ == "blue") ? -1.0 : 1.0;
  if (mirror_x < 0.0) {
    RCLCPP_INFO(
      get_logger(),
      "[Game 1] RIGHT/BLUE Field Side selected! Auto-mirroring X coordinates and flipping Yaw angles.");
  } else {
    RCLCPP_INFO(get_logger(), "[Game 1] LEFT/RED Field Side selected (Standard orientation).");
  }

  // YAML からの Waypoint 読み込み (左右陣営のX軸反転と進行方向Yaw反転を自動適用)
  wp_gate_.x = declare_parameter<double>("wp_gate_x", -5.925) * mirror_x;
  wp_gate_.y = declare_parameter<double>("wp_gate_y", 1.500);
  wp_gate_.yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_gate_yaw", 0.0);

  wp_around_gate_.x = declare_parameter<double>("wp_around_gate_x", -4.500) * mirror_x;
  wp_around_gate_.y = declare_parameter<double>("wp_around_gate_y", 0.500);
  wp_around_gate_.yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_around_gate_yaw", 0.0);

  wp_ball_.x = declare_parameter<double>("wp_ball_x", -2.700) * mirror_x;
  wp_ball_.y = declare_parameter<double>("wp_ball_y", 1.500);
  wp_ball_.yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_ball_yaw", 0.0);

  wp_pass_area_.x = declare_parameter<double>("wp_pass_area_x", -2.050) * mirror_x;
  wp_pass_area_.y = declare_parameter<double>("wp_pass_area_y", 1.500);
  wp_pass_area_.yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_pass_area_yaw", 0.0);

  wp_start_.x = declare_parameter<double>("wp_start_x", -5.925) * mirror_x;
  wp_start_.y = declare_parameter<double>("wp_start_y", 4.950);
  wp_start_.yaw = declare_parameter<double>("wp_start_yaw", 0.0);

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game1/command_start", 10,
    std::bind(&Game1AutoNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game1AutoNode::imu_callback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&Game1AutoNode::odom_callback, this, std::placeholders::_1));

  ball_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ball_pose", 10,
    std::bind(&Game1AutoNode::ball_detection_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", 10);
  arm_position_pub_ =
    create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", 10);
  spring_fire_pub_ = create_publisher<std_msgs::msg::Bool>("/spring/fire_request", 10);
  spring_slow_fire_pub_ = create_publisher<std_msgs::msg::Bool>("/spring/slow_fire_request", 10);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game1/completed", 10);

  // 20 Hz 制御ループ
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game1AutoNode::control_loop, this));

  RCLCPP_INFO(
    get_logger(), "Game1AutoNode initialized with %s feedback (test_mode=%s).",
    odom_topic.c_str(), test_mode_ ? "true" : "false");
}

void Game1AutoNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_received_ = true;
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // EKF 融合後の Orientation クォータニオンから Yaw を取得
  const double qx = msg->pose.pose.orientation.x;
  const double qy = msg->pose.pose.orientation.y;
  const double qz = msg->pose.pose.orientation.z;
  const double qw = msg->pose.pose.orientation.w;
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

void Game1AutoNode::ball_detection_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ball_detected_ = true;
  last_ball_detection_time_ = now();
  detected_ball_x_ = msg->pose.position.x;
  detected_ball_y_ = msg->pose.position.y;
}

void Game1AutoNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    state_ = test_mode_ ? Game1State::TEST_SINGLE_WP : Game1State::NAV_TO_GATE;
    state_start_time_ = now();

    if (test_mode_) {
      test_start_x_ = current_x_;
      test_start_y_ = current_y_;
      test_start_yaw_ = raw_yaw_;
      RCLCPP_INFO(
        get_logger(),
        "=== [Test Mode STARTED] Target Relative: (dx=%.2fm, dy=%.2fm) | Start Pose: (%.2f, %.2f, Yaw: %.2f rad) ===",
        test_dist_x_, test_dist_y_, test_start_x_, test_start_y_, test_start_yaw_);
    } else {
      RCLCPP_INFO(
        get_logger(), "Game 1 Auto Sequence STARTED (Field side: %s, Start Pos: [%.2f, %.2f], Yaw: %.2f rad).",
        field_side_.c_str(), current_x_, current_y_, raw_yaw_);
    }
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = Game1State::STANDBY;
    RCLCPP_INFO(get_logger(), "Game 1 Auto Sequence STOPPED.");
  }
}

void Game1AutoNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  imu_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  if (!odom_received_) {
    // EKF 未受信時のみバックアップとして直読み IMU を使用
    raw_yaw_ = imu_yaw_;
  }
}

geometry_msgs::msg::Twist Game1AutoNode::compute_holonomic_pursuit(const Waypoint & target, double speed_limit)
{
  geometry_msgs::msg::Twist cmd;

  const double max_speed = (speed_limit > 0.0) ? speed_limit : max_linear_vel_;

  // 1. ワールド座標系（フィールド基準）での位置誤差と距離
  const double dx_world = target.x - current_x_;
  const double dy_world = target.y - current_y_;
  const double dist = std::hypot(dx_world, dy_world);

  if (dist > 1e-4) {
    // 2. ベクトル比例減速プロファイル（目標に近づくほど滑らかに減速しオーバーシュートを防止）
    const double target_speed = std::min(max_speed, kp_linear_ * dist);

    // ワールド座標系での速度ベクトル
    const double vx_world = target_speed * (dx_world / dist);
    const double vy_world = target_speed * (dy_world / dist);

    // 3. フィールド座標系 ➔ ロボット車体座標系への回転変換 (Field-Oriented -> Body-Centric)
    const double cos_yaw = std::cos(raw_yaw_);
    const double sin_yaw = std::sin(raw_yaw_);

    cmd.linear.x = cos_yaw * vx_world + sin_yaw * vy_world;
    cmd.linear.y = -sin_yaw * vx_world + cos_yaw * vy_world;
  }

  // 4. 独立した姿勢角（Heading）制御
  const double yaw_err = std::remainder(target.yaw - raw_yaw_, 2.0 * M_PI);
  cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);

  return cmd;
}

bool Game1AutoNode::is_aligned_to_target(const Waypoint & target)
{
  const double dist_err = std::hypot(target.x - current_x_, target.y - current_y_);
  const double yaw_err = std::abs(std::remainder(target.yaw - raw_yaw_, 2.0 * M_PI));

  return (dist_err <= pos_tolerance_) && (yaw_err <= yaw_tolerance_);
}

void Game1AutoNode::control_loop()
{
  if (!is_enabled_ || state_ == Game1State::STANDBY) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  bool dribble_enabled = false;
  uint8_t arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;
  bool spring_fire = false;

  const double elapsed = (now() - state_start_time_).seconds();

  switch (state_) {
    case Game1State::NAV_TO_GATE: {
        // 1. ゲート射出位置へ全方位追従移動 (走りながら通過＆射出)
        cmd = compute_holonomic_pursuit(wp_gate_);
        const double dist_to_gate = std::hypot(wp_gate_.x - current_x_, wp_gate_.y - current_y_);
        // ゲート前方に近接（距離35cm以内）した瞬間に停止せず走りながら射出リクエストを発行
        if (dist_to_gate <= 0.35 || elapsed > 3.0) {
          RCLCPP_INFO(
            get_logger(), "Passing Gate shooting position (shoot on the move!). Firing 1st Spring!");
          state_ = Game1State::FIRE_GATE_SPRING;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::FIRE_GATE_SPRING: {
        // 2. 走りながらゲート横回り込みへ滑らかに移行
        cmd = compute_holonomic_pursuit(wp_around_gate_);
        spring_fire = true;
        if ((now() - state_start_time_).seconds() > 0.3) {
          RCLCPP_INFO(get_logger(), "Gate Shot fired on the move. Continuing around gate.");
          state_ = Game1State::NAV_AROUND_GATE;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::NAV_AROUND_GATE: {
        // 3. ロボットはゲートの横を通ってボール背後 (wp_ball_: X=-3.20m, Y=2.165m) へノンストップで回り込む
        cmd = compute_holonomic_pursuit(wp_around_gate_);
        const double dist_around = std::hypot(wp_around_gate_.x - current_x_, wp_around_gate_.y - current_y_);
        // 回り込み中間点を高速フライスルー (35cm以内通過で即座にボール背後追従へ)
        if (dist_around <= 0.35 || (now() - state_start_time_).seconds() > 2.0) {
          RCLCPP_INFO(
            get_logger(),
            "Bypassed gate. Sweeping into position behind ball facing forward with DRIBBLE ON.");
          state_ = Game1State::SEARCH_AND_CATCH_BALL;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::SEARCH_AND_CATCH_BALL: {
        // 4. 正面 (yaw=0.0) を向けたままボール背後から前進してボールをキャッチ＋ドリブルON
        dribble_enabled = true; // バックスピンでボールをしっかり吸い寄せる
        arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;

        const bool is_recent_detection = ball_detected_ &&
          (now() - last_ball_detection_time_).seconds() < 1.0;
        if (is_recent_detection) {
          // カメラ座標系（ロボット基準）：detected_ball_x_ が前方距離、detected_ball_y_ が横オフセット
          const double ball_dist = std::hypot(detected_ball_x_, detected_ball_y_);
          if (ball_dist > 1e-3) {
            const double ball_speed = std::min(max_linear_vel_, kp_linear_ * ball_dist);
            cmd.linear.x = ball_speed * (detected_ball_x_ / ball_dist);
            cmd.linear.y = ball_speed * (detected_ball_y_ / ball_dist);
          }
          // 正面 (yaw=0.0) を維持
          cmd.angular.z = std::clamp(kp_angular_ * std::remainder(wp_ball_.yaw - raw_yaw_, 2.0 * M_PI), -max_angular_vel_, max_angular_vel_);
        } else {
          // ボール未検出：予想ターゲット位置へ向かってホロノミック追従走行
          cmd = compute_holonomic_pursuit(wp_ball_);
        }

        const double dist_to_ball_wp = std::hypot(wp_ball_.x - current_x_, wp_ball_.y - current_y_);
        // ボール位置に到達またはタイムアウトでキャッチ成立 -> 止まらずそのままパスエリアへ
        if (dist_to_ball_wp <= 0.35 || (now() - state_start_time_).seconds() > 2.5) {
          RCLCPP_INFO(get_logger(), "Ball caught! Sweeping directly to Pass Area keeping forward orientation.");
          state_ = Game1State::NAV_TO_PASS_AREA;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::NAV_TO_PASS_AREA: {
        // 5. ボール保持のままパスエリア射出位置へ前向き平行移動
        cmd = compute_holonomic_pursuit(wp_pass_area_);
        dribble_enabled = true;
        arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;

        const double dist_to_pass = std::hypot(wp_pass_area_.x - current_x_, wp_pass_area_.y - current_y_);
        if (dist_to_pass <= 0.15 || elapsed > 3.0) {
          RCLCPP_INFO(get_logger(), "Arrived at Pass Area boundary flush. Opening arm for Slow Fire (L1 behavior)...");
          state_ = Game1State::FIRE_PASS_SPRING;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::FIRE_PASS_SPRING: {
        // 5. パスエリアへスプリングゆっくり射出 (L1方式: ドリブルOFF, アームOPEN, slow_fire_request)
        cmd = geometry_msgs::msg::Twist{};
        dribble_enabled = false;
        arm_pos = robot_msgs::msg::ArmPosition::OPEN;

        // アームが展開する時間 (0.3秒後) にゆっくり射出リクエストを発行
        bool spring_slow_fire = (elapsed >= 0.3);

        if (elapsed > 0.8) {
          RCLCPP_INFO(get_logger(), "Pass Area Slow Fire Complete. Returning straight to Start position (zero spin).");
          state_ = Game1State::NAV_TO_START;
          state_start_time_ = now();
        }
        publish_commands(cmd, dribble_enabled, arm_pos, false, spring_slow_fire);
        return;
      }

    case Game1State::NAV_TO_START: {
        // 6. スタート位置へ正面(yaw=0.0)をキープしたまま直線斜め自動復帰
        cmd = compute_holonomic_pursuit(wp_start_);
        arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;
        const double dist_to_start = std::hypot(wp_start_.x - current_x_, wp_start_.y - current_y_);
        if (dist_to_start <= pos_tolerance_ || (now() - state_start_time_).seconds() > 4.0) {
          RCLCPP_INFO(get_logger(), "Game 1 Auto Sequence COMPLETED! Ready for reload.");
          state_ = Game1State::COMPLETED;
        }
        break;
      }

    case Game1State::TEST_SINGLE_WP: {
        const double rel_yaw = std::remainder(raw_yaw_ - test_start_yaw_, 2.0 * M_PI);

        // 現在の生位置をスタート地点基準の相対座標（スタート時のロボット座標系）へ変換
        const double dx_raw = current_x_ - test_start_x_;
        const double dy_raw = current_y_ - test_start_y_;
        const double cos_start_yaw = std::cos(-test_start_yaw_);
        const double sin_start_yaw = std::sin(-test_start_yaw_);
        const double rel_x = dx_raw * cos_start_yaw - dy_raw * sin_start_yaw;
        const double rel_y = dx_raw * sin_start_yaw + dy_raw * cos_start_yaw;

        // 目標との差分 (相対座標系上)
        const double err_x = test_dist_x_ - rel_x;
        const double err_y = test_dist_y_ - rel_y;
        const double dist_err = std::hypot(err_x, err_y);
        const double yaw_err = std::remainder(0.0 - rel_yaw, 2.0 * M_PI);

        if (dist_err > 1e-4) {
          const double target_speed = std::min(test_max_vel_, kp_linear_ * dist_err);
          const double vx_rel = target_speed * (err_x / dist_err);
          const double vy_rel = target_speed * (err_y / dist_err);

          // 相対座標系 ➔ 現在のロボット車体座標系への回転変換
          const double cos_cur_yaw = std::cos(rel_yaw);
          const double sin_cur_yaw = std::sin(rel_yaw);
          cmd.linear.x = cos_cur_yaw * vx_rel + sin_cur_yaw * vy_rel;
          cmd.linear.y = -sin_cur_yaw * vx_rel + cos_cur_yaw * vy_rel;
        }
        cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);

        if (dist_err <= pos_tolerance_ || elapsed > 10.0) {
          RCLCPP_INFO(
            get_logger(),
            "=== [Test Mode COMPLETED] Target reached! Moved: (dx=%.3fm, dy=%.3fm), DistErr: %.3fm ===",
            rel_x, rel_y, dist_err);
          cmd = geometry_msgs::msg::Twist{};
          publish_commands(cmd, false, robot_msgs::msg::ArmPosition::DRIBBLE, false, false);
          state_ = Game1State::COMPLETED;
        }
        break;
      }

    case Game1State::COMPLETED: {
        is_enabled_ = false;
        state_ = Game1State::STANDBY;
        cmd = geometry_msgs::msg::Twist{};
        publish_commands(cmd, false, robot_msgs::msg::ArmPosition::DRIBBLE, false, false);
        std_msgs::msg::Bool comp;
        comp.data = true;
        completed_pub_->publish(comp);
        break;
      }

    default:
      break;
  }

  publish_commands(cmd, dribble_enabled, arm_pos, spring_fire, false);
}

void Game1AutoNode::publish_commands(
  const geometry_msgs::msg::Twist & cmd_vel,
  bool dribble_enabled,
  uint8_t arm_position,
  bool spring_fire,
  bool spring_slow_fire)
{
  cmd_vel_pub_->publish(cmd_vel);

  std_msgs::msg::Bool dribble_msg;
  dribble_msg.data = dribble_enabled;
  dribble_enabled_pub_->publish(dribble_msg);

  robot_msgs::msg::ArmPosition arm_msg;
  arm_msg.position = arm_position;
  arm_position_pub_->publish(arm_msg);

  std_msgs::msg::Bool spring_msg;
  spring_msg.data = spring_fire;
  spring_fire_pub_->publish(spring_msg);

  std_msgs::msg::Bool slow_spring_msg;
  slow_spring_msg.data = spring_slow_fire;
  spring_slow_fire_pub_->publish(slow_spring_msg);
}

}  // namespace robot_controller
