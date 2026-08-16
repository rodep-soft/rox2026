#include "game1_shooter/game1_auto_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game1AutoNode::Game1AutoNode(const rclcpp::NodeOptions & options)
: Node("game1_auto_node", options)
{
  kp_linear_ = declare_parameter<double>("kp_linear", 1.0);
  kp_angular_ = declare_parameter<double>("kp_angular", 1.5);
  max_linear_vel_ = declare_parameter<double>("max_linear_vel", 1.5);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.0);
  pos_tolerance_ = declare_parameter<double>("pos_tolerance", 0.08);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.05);
  align_timeout_ = declare_parameter<double>("align_timeout", 3.0);

  // ゲートに貼ってあるAprilTagのID
  gate_tag_id_ = declare_parameter<int>("gate_tag_id", 0);
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");

  // テストモード設定
  test_mode_ = declare_parameter<bool>("test_mode", false);
  wp_test_.x = declare_parameter<double>("wp_test_x", 1.0);
  wp_test_.y = declare_parameter<double>("wp_test_y", 0.0);
  wp_test_.yaw = declare_parameter<double>("wp_test_yaw", 0.0);

  // YAML からの Waypoint 読み込み
  wp_gate_.x = declare_parameter<double>("wp_gate_x", 1.5);
  wp_gate_.y = declare_parameter<double>("wp_gate_y", 0.0);
  wp_gate_.yaw = declare_parameter<double>("wp_gate_yaw", 0.0);

  wp_around_gate_.x = declare_parameter<double>("wp_around_gate_x", 2.5);
  wp_around_gate_.y = declare_parameter<double>("wp_around_gate_y", 1.0);
  wp_around_gate_.yaw = declare_parameter<double>("wp_around_gate_yaw", 0.0);

  wp_ball_.x = declare_parameter<double>("wp_ball_x", 3.5);
  wp_ball_.y = declare_parameter<double>("wp_ball_y", 0.0);
  wp_ball_.yaw = declare_parameter<double>("wp_ball_yaw", 0.0);

  wp_pass_area_.x = declare_parameter<double>("wp_pass_area_x", 2.0);
  wp_pass_area_.y = declare_parameter<double>("wp_pass_area_y", -1.0);
  wp_pass_area_.yaw = declare_parameter<double>("wp_pass_area_yaw", -1.5708);

  wp_start_.x = declare_parameter<double>("wp_start_x", 0.0);
  wp_start_.y = declare_parameter<double>("wp_start_y", 0.0);
  wp_start_.yaw = declare_parameter<double>("wp_start_yaw", 0.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    "/detection", 10,
    std::bind(&Game1AutoNode::ball_detection_callback, this, std::placeholders::_1));

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SensorDataQoS(),
    std::bind(&Game1AutoNode::joy_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", 10);
  arm_position_pub_ =
    create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", 10);
  spring_fire_pub_ = create_publisher<std_msgs::msg::Bool>("/spring/fire_request", 10);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game1/completed", 10);

  // 20 Hz 制御ループ
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game1AutoNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Game1AutoNode initialized with EKF /odometry/filtered feedback.");
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
  current_yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
}

void Game1AutoNode::ball_detection_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ball_detected_ = true;
  last_ball_detection_time_ = now();
  detected_ball_x_ = msg->pose.position.x;
  detected_ball_y_ = msg->pose.position.y;
}

void Game1AutoNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!is_enabled_) {
    return;
  }

  // ジョイスティックの各軸（スティック）の操作量を判定
  constexpr double deadzone = 0.15;
  bool joystick_moved = false;
  for (const float axis_val : msg->axes) {
    if (std::abs(axis_val) > deadzone) {
      joystick_moved = true;
      break;
    }
  }

  if (joystick_moved) {
    is_enabled_ = false;
    state_ = Game1State::STANDBY;
    RCLCPP_WARN(
      get_logger(),
      "Joystick input detected! Emergency Manual Override: Stopping Game 1 Auto Sequence.");
  }
}

void Game1AutoNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    state_ = test_mode_ ? Game1State::TEST_SINGLE_WP : Game1State::NAV_TO_GATE;
    state_start_time_ = now();
    // スタート時のIMU/EKF生角度をオフセットとして記録し、スタート位置の向きを 0.0 rad にゼロリセット
    yaw_offset_ = raw_yaw_;
    current_yaw_ = 0.0;
    RCLCPP_INFO(
      get_logger(), "Game 1 Auto Sequence STARTED (TestMode: %s). EKF/IMU Zero-Reset (Offset: %.3f rad).",
      test_mode_ ? "ON" : "OFF", yaw_offset_);
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = Game1State::STANDBY;
    RCLCPP_INFO(get_logger(), "Game 1 Auto Sequence STOPPED.");
  }
}

void Game1AutoNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  if (!odom_received_) {
    // EKF 未受信時のみバックアップとして直読み IMU をバックアップ受信用に使用
    const double qx = msg->orientation.x;
    const double qy = msg->orientation.y;
    const double qz = msg->orientation.z;
    const double qw = msg->orientation.w;
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    current_yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
  }
}

geometry_msgs::msg::Twist Game1AutoNode::compute_pure_pursuit(const Waypoint & target)
{
  geometry_msgs::msg::Twist cmd;
  // EKF 自己位置 (current_x, current_y, current_yaw) から見た目標位置への差分
  const double dx = target.x - current_x_;
  const double dy = target.y - current_y_;
  const double yaw_err = std::remainder(target.yaw - current_yaw_, 2.0 * M_PI);

  cmd.linear.x = std::clamp(kp_linear_ * dx, -max_linear_vel_, max_linear_vel_);
  cmd.linear.y = std::clamp(kp_linear_ * dy, -max_linear_vel_, max_linear_vel_);
  cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);

  return cmd;
}

bool Game1AutoNode::is_aligned_to_target(const Waypoint & target)
{
  const double dist_err = std::hypot(target.x - current_x_, target.y - current_y_);
  const double yaw_err = std::abs(std::remainder(target.yaw - current_yaw_, 2.0 * M_PI));

  return (dist_err <= pos_tolerance_) && (yaw_err <= yaw_tolerance_);
}

void Game1AutoNode::control_loop()
{
  if (!is_enabled_ || state_ == Game1State::STANDBY) {
    publish_commands(
      geometry_msgs::msg::Twist{}, false, robot_msgs::msg::ArmPosition::DRIBBLE,
      false);
    return;
  }

  geometry_msgs::msg::Twist cmd;
  bool dribble_enabled = false;
  uint8_t arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;
  bool spring_fire = false;

  const double elapsed = (now() - state_start_time_).seconds();

  switch (state_) {
    case Game1State::NAV_TO_GATE: {
        // 1. ゲート射出位置へ移動 (yaw合わせは次のステートで行う)
        Waypoint nav_target = wp_gate_;
        nav_target.yaw = current_yaw_;  // 移動中は向き問わず位置だけ合わせる
        const double dist_err = std::hypot(wp_gate_.x - current_x_, wp_gate_.y - current_y_);
        cmd = compute_pure_pursuit(nav_target);
        if (dist_err <= pos_tolerance_ || elapsed > 10.0) {
          RCLCPP_INFO(get_logger(), "Arrived at gate WP. Aligning to gate tag ID=%d.", gate_tag_id_);
          state_ = Game1State::ALIGN_TO_GATE;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::ALIGN_TO_GATE: {
        // 2. ゲートのAprilTagを見てゲート方向に向き合わせ
        const std::string tag_frame = tag_prefix_ + std::to_string(gate_tag_id_);
        bool aligned = false;

        try {
          // base_link → gate_tag の変換を取得
          const auto t = tf_buffer_->lookupTransform(
            "base_link", tag_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));

          const double tx = t.transform.translation.x;
          const double ty = t.transform.translation.y;

          // タグがbase_linkからどの方向にあるか
          const double yaw_to_tag = std::atan2(ty, tx);
          const double yaw_err = std::remainder(yaw_to_tag, 2.0 * M_PI);

          cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);

          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[ALIGN_TO_GATE] tag=%s  yaw_err=%.3f rad", tag_frame.c_str(), yaw_err);

          if (std::abs(yaw_err) <= yaw_tolerance_) {
            aligned = true;
          }
        } catch (const tf2::TransformException & ex) {
          // タグが見えない場合: wp_gate_.yaw にフォールバック
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[ALIGN_TO_GATE] Tag not visible (%s). Falling back to wp_gate_.yaw.", ex.what());
          const double yaw_err = std::remainder(wp_gate_.yaw - current_yaw_, 2.0 * M_PI);
          cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);
          if (std::abs(yaw_err) <= yaw_tolerance_) {
            aligned = true;
          }
        }

        // 向き合わせ完了 or タイムアウト → 射出へ
        if (aligned || elapsed > align_timeout_) {
          RCLCPP_INFO(
            get_logger(), "Gate alignment done (aligned=%s, t=%.1fs). Firing spring!",
            aligned ? "true" : "timeout", elapsed);
          state_ = Game1State::FIRE_GATE_SPRING;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::FIRE_GATE_SPRING: {
        // 2. ゲートへスプリング発射 (1発目)
        spring_fire = true;
        if ((now() - state_start_time_).seconds() > 1.0) {
          RCLCPP_INFO(get_logger(), "Gate Shot Complete. Navigating AROUND gate (free area).");
          state_ = Game1State::NAV_AROUND_GATE;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::NAV_AROUND_GATE: {
        // 3. ロボットはゲートの横を通って向こう側へ回り込む
        cmd = compute_pure_pursuit(wp_around_gate_);
        if ((now() - state_start_time_).seconds() > 3.0) {
          RCLCPP_INFO(
            get_logger(),
            "Around gate complete. Searching and catching ball dynamically with DRIBBLE ON (backspin).");
          state_ = Game1State::SEARCH_AND_CATCH_BALL;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::SEARCH_AND_CATCH_BALL: {
        // 4. カメラ(YOLO)で転がったボールの位置を認識して動的追従キャッチ＋ドリブルON
        dribble_enabled = true; // バックスピンでボールをしっかり吸い寄せる
        arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;

        const bool is_recent_detection = ball_detected_ &&
          (now() - last_ball_detection_time_).seconds() < 1.0;
        if (is_recent_detection) {
          // カメラでボールを発見：リアルタイム相対座標に向かって追従
          cmd.linear.x =
            std::clamp(kp_linear_ * detected_ball_x_, -max_linear_vel_, max_linear_vel_);
          cmd.linear.y =
            std::clamp(kp_linear_ * detected_ball_y_, -max_linear_vel_, max_linear_vel_);
          cmd.angular.z = std::clamp(
            -kp_angular_ * detected_ball_y_, -max_angular_vel_,
            max_angular_vel_);
        } else {
          // ボール未検出：予想ターゲット位置へ向かって走行
          cmd = compute_pure_pursuit(wp_ball_);
        }

        if ((now() - state_start_time_).seconds() > 4.0) {
          RCLCPP_INFO(get_logger(), "Ball caught! Navigating to Pass Area.");
          state_ = Game1State::NAV_TO_PASS_AREA;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::NAV_TO_PASS_AREA: {
        // 5. ボール保持のままパスエリア射出位置へ移動
        cmd = compute_pure_pursuit(wp_pass_area_);
        dribble_enabled = true;
        arm_pos = robot_msgs::msg::ArmPosition::OPEN; // 射出前にアームを開く

        if (is_aligned_to_target(wp_pass_area_) || elapsed > 5.0) {
          RCLCPP_INFO(get_logger(), "Arrived & Yaw-Aligned at Pass Area. Firing 2nd Spring!");
          state_ = Game1State::FIRE_PASS_SPRING;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::FIRE_PASS_SPRING: {
        // 5. パスエリアへスプリング発射 (2発目)
        arm_pos = robot_msgs::msg::ArmPosition::FEED; // ベルトへボールを押し込む
        spring_fire = true;
        if ((now() - state_start_time_).seconds() > 1.0) {
          RCLCPP_INFO(get_logger(), "Pass Area Shot Complete. Returning to Start position.");
          state_ = Game1State::NAV_TO_START;
          state_start_time_ = now();
        }
        break;
      }

    case Game1State::NAV_TO_START: {
        // 6. スタート位置へ自動復帰
        cmd = compute_pure_pursuit(wp_start_);
        if ((now() - state_start_time_).seconds() > 4.0) {
          RCLCPP_INFO(get_logger(), "Game 1 Auto Sequence COMPLETED!");
          state_ = Game1State::COMPLETED;
        }
        break;
      }

    case Game1State::TEST_SINGLE_WP: {
        cmd = compute_pure_pursuit(wp_test_);
        if (is_aligned_to_target(wp_test_) || elapsed > 15.0) {
          RCLCPP_INFO(get_logger(), "Test Single Waypoint Movement COMPLETED!");
          state_ = Game1State::COMPLETED;
        }
        break;
      }

    case Game1State::COMPLETED: {
        is_enabled_ = false;
        state_ = Game1State::STANDBY;
        std_msgs::msg::Bool comp;
        comp.data = true;
        completed_pub_->publish(comp);
        break;
      }

    default:
      break;
  }

  publish_commands(cmd, dribble_enabled, arm_pos, spring_fire);
}

void Game1AutoNode::publish_commands(
  const geometry_msgs::msg::Twist & cmd_vel,
  bool dribble_enabled,
  uint8_t arm_position,
  bool spring_fire)
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
}

}  // namespace robot_controller
