#include "game1_shooter/game1_auto_shooter_node.hpp"

#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game1AutoShooterNode::Game1AutoShooterNode(const rclcpp::NodeOptions & options)
: Node("game1_auto_shooter_node", options)
{
  kp_linear_       = declare_parameter<double>("kp_linear", 1.0);
  kp_angular_      = declare_parameter<double>("kp_angular", 1.5);
  max_linear_vel_  = declare_parameter<double>("max_linear_vel", 1.5);
  max_angular_vel_ = declare_parameter<double>("max_angular_vel", 1.0);
  pos_tolerance_   = declare_parameter<double>("pos_tolerance", 0.08);
  yaw_tolerance_   = declare_parameter<double>("yaw_tolerance", 0.05);

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game1/command_start", 10,
    std::bind(&Game1AutoShooterNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game1AutoShooterNode::imu_callback, this, std::placeholders::_1));

  cmd_vel_pub_         = create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", 10);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", 10);
  arm_position_pub_    = create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", 10);
  spring_fire_pub_     = create_publisher<std_msgs::msg::Bool>("/spring/fire_request", 10);
  completed_pub_       = create_publisher<std_msgs::msg::Bool>("/game1/completed", 10);

  // 20 Hz 制御ループ
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game1AutoShooterNode::control_loop, this));

  RCLCPP_INFO(get_logger(), "Game1AutoShooterNode initialized.");
}

void Game1AutoShooterNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    state_ = Game1State::NAV_TO_GATE;
    state_start_time_ = now();
    // スタート時のIMU生角度をオフセットとして記録し、スタート位置の向きを 0.0 rad にゼロリセット
    yaw_offset_ = raw_yaw_;
    current_yaw_ = 0.0;
    RCLCPP_INFO(get_logger(), "Game 1 Auto Sequence STARTED. IMU Yaw Zero-Reset (Offset: %.3f rad).", yaw_offset_);
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = Game1State::STANDBY;
    RCLCPP_INFO(get_logger(), "Game 1 Auto Sequence STOPPED.");
  }
}

void Game1AutoShooterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  // クォータニオンから Yaw 角 [rad] を計算
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  current_yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
}

geometry_msgs::msg::Twist Game1AutoShooterNode::compute_pure_pursuit(const Waypoint & target)
{
  geometry_msgs::msg::Twist cmd;
  const double yaw_err = std::remainder(target.yaw - current_yaw_, 2.0 * M_PI);

  cmd.linear.x = std::clamp(kp_linear_ * target.x, -max_linear_vel_, max_linear_vel_);
  cmd.linear.y = std::clamp(kp_linear_ * target.y, -max_linear_vel_, max_linear_vel_);
  cmd.angular.z = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);

  return cmd;
}

void Game1AutoShooterNode::control_loop()
{
  if (!is_enabled_ || state_ == Game1State::STANDBY) {
    publish_commands(geometry_msgs::msg::Twist{}, false, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  geometry_msgs::msg::Twist cmd;
  bool dribble_enabled = false;
  uint8_t arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;
  bool spring_fire = false;

  switch (state_) {
    case Game1State::NAV_TO_GATE: {
      // 1. ゲート射出位置へ移動
      cmd = compute_pure_pursuit(wp_gate_);
      if ((now() - state_start_time_).seconds() > 3.0) {
        RCLCPP_INFO(get_logger(), "Arrived at Gate shooting position. Firing 1st Spring!");
        state_ = Game1State::FIRE_GATE_SPRING;
        state_start_time_ = now();
      }
      break;
    }

    case Game1State::FIRE_GATE_SPRING: {
      // 2. ゲートへスプリング発射 (1発目)
      spring_fire = true;
      if ((now() - state_start_time_).seconds() > 1.0) {
        RCLCPP_INFO(get_logger(), "Gate Shot Complete. Navigating around gate with DRIBBLE ON (backspin).");
        state_ = Game1State::NAV_TO_BALL_DRIBBLE_ON;
        state_start_time_ = now();
      }
      break;
    }

    case Game1State::NAV_TO_BALL_DRIBBLE_ON: {
      // 3. ドリブルON（バックスピン）でゲート向こうのボールに接近してキャッチ
      cmd = compute_pure_pursuit(wp_ball_);
      dribble_enabled = true; // バックスピンでボールをしっかり吸い寄せる
      arm_pos = robot_msgs::msg::ArmPosition::DRIBBLE;

      if ((now() - state_start_time_).seconds() > 4.0) {
        RCLCPP_INFO(get_logger(), "Ball caught! Navigating to Pass Area.");
        state_ = Game1State::NAV_TO_PASS_AREA;
        state_start_time_ = now();
      }
      break;
    }

    case Game1State::NAV_TO_PASS_AREA: {
      // 4. ボール保持のままパスエリア射出位置へ移動
      cmd = compute_pure_pursuit(wp_pass_area_);
      dribble_enabled = true;
      arm_pos = robot_msgs::msg::ArmPosition::OPEN; // 射出前にアームを開く

      if ((now() - state_start_time_).seconds() > 3.0) {
        RCLCPP_INFO(get_logger(), "Arrived at Pass Area. Firing 2nd Spring!");
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

void Game1AutoShooterNode::publish_commands(
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
