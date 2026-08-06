#include "bingo_shooter/bingo_tactical_shooter_node.hpp"

#include <cmath>

namespace robot_controller
{

BingoTacticalShooterNode::BingoTacticalShooterNode(const rclcpp::NodeOptions & options)
: Node("bingo_tactical_shooter_node", options)
{
  // Declare Parameters
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  tag_prefix_ = this->declare_parameter<std::string>("tag_prefix", "tag36h11:");
  kp_yaw_ = this->declare_parameter<double>("kp_yaw", 1.2);
  kp_y_ = this->declare_parameter<double>("kp_y", 0.8);
  kp_dist_ = this->declare_parameter<double>("kp_dist", 1.0);
  target_distance_ = this->declare_parameter<double>("target_distance", 1.5); // 1.5m射程
  yaw_tolerance_ = this->declare_parameter<double>("yaw_tolerance", 0.05);   // rad (約2.8度)
  dist_tolerance_ = this->declare_parameter<double>("dist_tolerance", 0.03); // 3cm
  rpm_bottom_ = this->declare_parameter<double>("rpm_bottom", 3000.0);
  rpm_middle_ = this->declare_parameter<double>("rpm_middle", 4500.0);
  rpm_top_ = this->declare_parameter<double>("rpm_top", 6000.0);
  shoot_hold_duration_ = this->declare_parameter<double>("shoot_hold_duration", 1.0);

  // Initialize Panel Grid Map (3x3)
  // Row 0: Bottom (Tag 6, 7, 8)
  panel_grid_[6] = {6, 0, 0};
  panel_grid_[7] = {7, 0, 1};
  panel_grid_[8] = {8, 0, 2};
  // Row 1: Middle (Tag 3, 4, 5)
  panel_grid_[3] = {3, 1, 0};
  panel_grid_[4] = {4, 1, 1};
  panel_grid_[5] = {5, 1, 2};
  // Row 2: Top (Tag 0, 1, 2)
  panel_grid_[0] = {0, 2, 0};
  panel_grid_[1] = {1, 2, 1};
  panel_grid_[2] = {2, 2, 2};

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mecanum/cmd_vel", 10);
  belt_rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/belt/target_rpm", 10);
  shoot_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", 10);

  // Control Loop Timer (20Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&BingoTacticalShooterNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "BingoTacticalShooterNode initialized.");
}

void BingoTacticalShooterNode::update_panel_states()
{
  const auto now = this->now();

  for (auto & [tag_id, info] : panel_grid_) {
    std::string frame_name = tag_prefix_ + std::to_string(tag_id);
    try {
      geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform(base_frame_, frame_name, tf2::TimePointZero);

      info.detected = true;
      info.x = tf.transform.translation.x;
      info.y = tf.transform.translation.y;
      info.z = tf.transform.translation.z;
      info.last_seen = now;
    } catch (const tf2::TransformException & ex) {
      if ((now - info.last_seen).seconds() > 2.0) {
        info.detected = false;
      }
    }
  }
}

void BingoTacticalShooterNode::select_target_and_aim()
{
  target_valid_ = false;

  // 下段 -> 中段 -> 上段 の順に攻略
  while (active_row_ <= 2) {
    std::vector<PanelTagInfo *> row_panels;
    for (auto & [tag_id, info] : panel_grid_) {
      if (info.row == active_row_) {
        row_panels.push_back(&info);
      }
    }

    // 列（col: 0:Left, 1:Center, 2:Right）でソート
    std::sort(row_panels.begin(), row_panels.end(), [](PanelTagInfo * a, PanelTagInfo * b) {
      return a->col < b->col;
    });

    PanelTagInfo * left = row_panels[0];
    PanelTagInfo * center = row_panels[1];
    PanelTagInfo * right = row_panels[2];

    // パターン1: 左と中央が残っている -> L&Cの境目を狙って2枚抜き！
    if (left->detected && center->detected) {
      target_x_ = (left->x + center->x) / 2.0;
      target_y_ = (left->y + center->y) / 2.0;
      target_z_ = (left->z + center->z) / 2.0;
      target_valid_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Row %d: Aiming at boundary between Tag %d and Tag %d (Double Knockdown!)",
        active_row_, left->tag_id, center->tag_id);
      break;
    }

    // パターン2: 中央と右が残っている -> C&Rの境目を狙って2枚抜き！
    if (center->detected && right->detected) {
      target_x_ = (center->x + right->x) / 2.0;
      target_y_ = (center->y + right->y) / 2.0;
      target_z_ = (center->z + right->z) / 2.0;
      target_valid_ = true;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Row %d: Aiming at boundary between Tag %d and Tag %d (Double Knockdown!)",
        active_row_, center->tag_id, right->tag_id);
      break;
    }

    // パターン3: 1枚だけ残っている -> その1枚の真芯を狙撃！
    for (auto * panel : row_panels) {
      if (panel->detected) {
        target_x_ = panel->x;
        target_y_ = panel->y;
        target_z_ = panel->z;
        target_valid_ = true;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Row %d: Aiming at remaining single Tag %d (Pinpoint Shot!)",
          active_row_, panel->tag_id);
        break;
      }
    }

    if (target_valid_) {
      break;
    }

    // その段が全滅していたら次の段へ進む
    RCLCPP_INFO(this->get_logger(), "Row %d completed! Moving to next row.", active_row_);
    active_row_++;
  }

  // 段に応じた目標ベルト RPM の決定
  if (active_row_ == 0) {
    target_rpm_ = rpm_bottom_;
  } else if (active_row_ == 1) {
    target_rpm_ = rpm_middle_;
  } else {
    target_rpm_ = rpm_top_;
  }
}

void BingoTacticalShooterNode::control_loop()
{
  update_panel_states();
  select_target_and_aim();

  geometry_msgs::msg::Twist cmd_vel;
  std_msgs::msg::Float32 rpm_msg;
  std_msgs::msg::Bool trigger_msg;
  rpm_msg.data = static_cast<float>(target_rpm_);
  trigger_msg.data = false;

  if (active_row_ > 2) {
    state_ = State::COMPLETED;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "ALL BINGO PANELS COMPLETED! Good job!");
    cmd_vel_pub_->publish(cmd_vel);
    return;
  }

  if (!target_valid_) {
    state_ = State::SEARCHING;
    // ターゲット探索中：その場低速旋回
    cmd_vel.angular.z = 0.3;
    cmd_vel_pub_->publish(cmd_vel);
    belt_rpm_pub_->publish(rpm_msg);
    return;
  }

  const double dist_err = target_x_ - target_distance_;
  const double y_err = target_y_;

  switch (state_) {
    case State::SEARCHING:
    case State::ALIGNING: {
      state_ = State::ALIGNING;

      // 横スライド補正
      cmd_vel.linear.y = -kp_y_ * y_err;
      // 前後距離補正
      cmd_vel.linear.x = kp_dist_ * dist_err;
      // 旋回アライメント補正
      cmd_vel.angular.z = -kp_yaw_ * y_err;

      // 照準完了チェック (誤差判定)
      if (std::abs(y_err) < yaw_tolerance_ && std::abs(dist_err) < dist_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Target Alignment ACQUIRED! Preparing to shoot...");
        state_ = State::PREPARING_SHOOT;
      }
      break;
    }

    case State::PREPARING_SHOOT: {
      // 照準完了：機体完全静止 ＆ ベルト加圧
      cmd_vel = geometry_msgs::msg::Twist{};
      state_ = State::SHOOTING;
      shoot_start_time_ = this->now();
      break;
    }

    case State::SHOOTING: {
      cmd_vel = geometry_msgs::msg::Twist{};
      trigger_msg.data = true; // 発射トリガーオン！

      if ((this->now() - shoot_start_time_).seconds() > shoot_hold_duration_) {
        RCLCPP_INFO(this->get_logger(), "Shot RELEASED! Waiting for panel result...");
        state_ = State::WAITING_RESULT;
        shoot_start_time_ = this->now();
      }
      break;
    }

    case State::WAITING_RESULT: {
      cmd_vel = geometry_msgs::msg::Twist{};
      if ((this->now() - shoot_start_time_).seconds() > 1.5) {
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
}

}  // namespace robot_controller
