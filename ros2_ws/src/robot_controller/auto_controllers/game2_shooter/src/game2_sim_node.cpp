#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <robot_msgs/msg/game2_state.hpp>
#include <robot_msgs/msg/arm_position.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "game2_shooter/panel_types.hpp"
#include "game2_shooter/target_selector.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <chrono>

namespace robot_controller
{

struct InFlightBall
{
  double start_x, start_y, start_z;
  double target_x, target_y, target_z;
  double flight_duration;
  double elapsed;
  std::vector<int> target_tag_ids;
};

class Game2SimNode : public rclcpp::Node
{
public:
  explicit Game2SimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("game2_sim_node", options),
    gen_(rd_())
  {
    field_side_ = declare_parameter<std::string>("field_side", "left");
    target_dist_ = declare_parameter<double>("target_distance", 4.0);
    sigma_x_ = declare_parameter<double>("dispersion_sigma_x", 0.05); // 5cm
    sigma_z_ = declare_parameter<double>("dispersion_sigma_z", 0.125); // 10-15cm (平均12.5cm)

    target_config_.shooter_offset_x = declare_parameter<double>("shooter_offset_x", -0.265);
    target_config_.shooter_offset_y = declare_parameter<double>("shooter_offset_y", 0.0);
    target_config_.yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.015);
    target_config_.enable_double_panel_midpoint_targeting =
      declare_parameter<bool>("enable_double_panel_midpoint_targeting", true);
    target_config_.prefer_same_row_first =
      declare_parameter<bool>("prefer_same_row_first", false);
    target_config_.enable_vertical_sweep =
      declare_parameter<bool>("enable_vertical_sweep", true);
    target_config_.enable_nearest_angle_search =
      declare_parameter<bool>("enable_nearest_angle_search", true);

    mirror_x_ = (field_side_ == "right" || field_side_ == "blue") ? -1.0 : 1.0;

    footprint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/robot/footprint_marker", rclcpp::QoS(10));
    ball_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/sim/ball_marker", rclcpp::QoS(10));
    g2_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/game2_sim/markers", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom/simulated", rclcpp::QoS(10));
    state_pub_ = create_publisher<robot_msgs::msg::Game2State>(
      "/game2/state", rclcpp::QoS(1).reliable().transient_local());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // AprilTag ID & パネル初期化 (Side A: 左から 16,15,14 / 19,18,17 / 22,21,20)
    init_panels();

    // ロボット初期位置: パネル中心から 4.0m 手前
    robot_x_ = panel_center_x_;
    robot_y_ = panel_center_y_ + target_dist_;
    robot_yaw_ = -M_PI / 2.0;

    state_ = robot_msgs::msg::Game2State::SEARCHING;
    state_timer_ = 0.0;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&Game2SimNode::sim_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "Game2SimNode running with official TargetSelector algorithms (DoubleMidpoint + VerticalSweep + NearestAngle)!");
  }

private:
  void init_panels()
  {
    panel_center_x_ = (mirror_x_ > 0.0) ? -3.23 : 3.63;
    panel_center_y_ = -5.525;

    const std::vector<int64_t> bottom_tags = {22, 21, 20}; // Col 0, 1, 2
    const std::vector<int64_t> middle_tags = {19, 18, 17};
    const std::vector<int64_t> top_tags    = {14, 15, 16};

    vision_tracker_.initialize_grid(bottom_tags, middle_tags, top_tags, this->now());

    // 各タグの絶対座標を登録 & detected を true に
    auto & grid = vision_tracker_.get_panel_grid();
    for (auto & [id, p] : grid) {
      p.detected = true;
      p.shot_completed = false;
      p.shot_count = 0;
      // map座標系での実位置
      p.x = panel_center_x_ + (p.col - 1) * 0.41;
      p.y = panel_center_y_;
      p.z = 0.18 + p.row * 0.41;
    }
  }

  void sim_loop()
  {
    const double dt = 0.03;
    state_timer_ += dt;
    const auto now_stamp = this->now();

    // 1. game2_auto_node と完全同一の TargetSelector アルゴリズム実行
    step_state_machine(dt);

    // 2. TF 配信 (map -> base_footprint)
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now_stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = robot_x_;
    tf_msg.transform.translation.y = robot_y_;
    tf_msg.transform.translation.z = 0.05;
    tf_msg.transform.rotation.z = std::sin(robot_yaw_ / 2.0);
    tf_msg.transform.rotation.w = std::cos(robot_yaw_ / 2.0);
    tf_broadcaster_->sendTransform(tf_msg);

    // 3. オドメトリ配信
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now_stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = robot_x_;
    odom.pose.pose.position.y = robot_y_;
    odom.pose.pose.position.z = 0.05;
    odom.pose.pose.orientation = tf_msg.transform.rotation;
    odom_pub_->publish(odom);

    // 4. 飛翔中のボールの物理シミュレーション & 着弾判定
    auto & grid = vision_tracker_.get_panel_grid();
    for (auto it = active_balls_.begin(); it != active_balls_.end(); ) {
      it->elapsed += dt;
      const double t = std::min(1.0, it->elapsed / it->flight_duration);

      if (t >= 1.0) {
        // ボール着弾物理判定 (ボール半径 R = 0.10m, パネル半幅 = 0.18m, 半高 = 0.18m)
        // 中点に命中した場合、ボールの幅(直径20cm)が両側のパネルに同時に接触するため2枚同時に倒れる
        const double ball_radius = 0.10;
        const double panel_half_w = 0.18;
        const double panel_half_h = 0.18;

        for (auto & [id, p] : grid) {
          if (!p.shot_completed) {
            const double dx = std::abs(it->target_x - p.x);
            const double dz = std::abs(it->target_z - p.z);
            // ボール球体とパネル矩形の接触判定
            if (dx <= (panel_half_w + ball_radius) && dz <= (panel_half_h + ball_radius)) {
              p.shot_completed = true;
              RCLCPP_INFO(
                get_logger(),
                "[PHYSICAL HIT!] Ball (R=10cm) struck Panel Tag #%d (Row %d, Col %d) -> KNOCKED DOWN!",
                id, p.row, p.col);
            }
          }
        }
        it = active_balls_.erase(it);
      } else {
        ++it;
      }
    }

    // 5. ステート通知
    robot_msgs::msg::Game2State state_msg;
    state_msg.state = state_;
    state_pub_->publish(state_msg);

    // 6. 各種マーカー配信
    publish_all_markers(now_stamp);
  }

  void step_state_machine(double dt)
  {
    auto & grid = vision_tracker_.get_panel_grid();

    // 全パネル撃破チェック
    bool all_cleared = true;
    for (const auto & [id, p] : grid) {
      if (!p.shot_completed) { all_cleared = false; break; }
    }
    if (all_cleared) {
      state_ = robot_msgs::msg::Game2State::COMPLETED;
      if (state_timer_ > 3.0) {
        init_panels();
        state_ = robot_msgs::msg::Game2State::SEARCHING;
        state_timer_ = 0.0;
        current_target_tag_ids_.clear();
      }
      return;
    }

    switch (state_) {
      case robot_msgs::msg::Game2State::SEARCHING: {
        // TargetSelector で最適ターゲットを選択 (2枚抜き中点 / 垂直スイープ / 最小角度移動)
        // base_link 座標系での相対位置を TargetSelector に供給
        std::unordered_map<int, PanelTagInfo> local_grid = grid;
        for (auto & [id, p] : local_grid) {
          // base_link 座標系: ロボットから見た相対位置
          const double dx = p.x - robot_x_;
          const double dy = p.y - robot_y_;
          // ロボットの yaw 基準に変換
          p.x =  std::cos(robot_yaw_) * dx + std::sin(robot_yaw_) * dy;
          p.y = -std::sin(robot_yaw_) * dx + std::cos(robot_yaw_) * dy;
        }

        auto best = target_selector_.select_best_target(
          local_grid, active_row_, current_target_heading_err_,
          current_target_tag_ids_, target_config_, get_logger(), get_clock());

        if (best) {
          active_row_ = best->row;
          current_target_tag_ids_ = best->tag_ids;
          target_aim_x_ = best->x; // map座標系
          target_aim_y_ = best->y;
          target_aim_z_ = best->z;
          // map座標系での真の目標位置を復元
          if (best->tag_ids.size() == 2) {
            target_aim_x_ = (grid[best->tag_ids[0]].x + grid[best->tag_ids[1]].x) * 0.5;
            target_aim_y_ = (grid[best->tag_ids[0]].y + grid[best->tag_ids[1]].y) * 0.5;
            target_aim_z_ = (grid[best->tag_ids[0]].z + grid[best->tag_ids[1]].z) * 0.5;
          } else if (!best->tag_ids.empty()) {
            target_aim_x_ = grid[best->tag_ids[0]].x;
            target_aim_y_ = grid[best->tag_ids[0]].y;
            target_aim_z_ = grid[best->tag_ids[0]].z;
          }
          state_ = robot_msgs::msg::Game2State::ALIGNING;
          state_timer_ = 0.0;
          RCLCPP_INFO(
            get_logger(),
            "[Game2 Target Selected] %s (Row %d) -> Aligning heading...",
            best->desc.c_str(), active_row_);
        }
        break;
      }

      case robot_msgs::msg::Game2State::ALIGNING: {
        // 目標位置への照準角計算
        const double target_angle = std::atan2(target_aim_y_ - robot_y_, target_aim_x_ - robot_x_);
        const double yaw_err = std::remainder(target_angle - robot_yaw_, 2.0 * M_PI);
        current_target_heading_err_ = yaw_err;

        // PD旋回制御
        const double wz = std::clamp(4.0 * yaw_err, -1.80, 1.80);
        robot_yaw_ = std::remainder(robot_yaw_ + wz * dt, 2.0 * M_PI);

        // 角度許容誤差 0.015 rad (約0.85度) ＆ ベルト回転数安定待ち (0.5s)
        if (std::abs(yaw_err) < target_config_.yaw_tolerance && state_timer_ >= 0.5) {
          state_ = robot_msgs::msg::Game2State::PREPARING_SHOOT;
          state_timer_ = 0.0;
        }
        break;
      }

      case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
        // 次弾装填・アーム展開・ボール安定化 (約1.0秒)
        arm_mode_ = robot_msgs::msg::ArmPosition::OPEN;
        if (state_timer_ >= 1.0) {
          state_ = robot_msgs::msg::Game2State::SHOOTING;
          state_timer_ = 0.0;
          fire_current_ball();
        }
        break;
      }

      case robot_msgs::msg::Game2State::SHOOTING: {
        // フライホイール送り込み・発射 (約0.6秒)
        arm_mode_ = robot_msgs::msg::ArmPosition::FEED;
        if (state_timer_ >= 0.6) {
          state_ = robot_msgs::msg::Game2State::WAITING_RESULT;
          state_timer_ = 0.0;
        }
        break;
      }

      case robot_msgs::msg::Game2State::WAITING_RESULT: {
        // 飛翔・着弾・パネル倒れ確認判定 (約0.9秒 -> 1発計2.5秒)
        arm_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
        if (state_timer_ >= 0.9) {
          state_ = robot_msgs::msg::Game2State::SEARCHING;
          state_timer_ = 0.0;
        }
        break;
      }

      default:
        state_ = robot_msgs::msg::Game2State::SEARCHING;
        break;
    }
  }

  void fire_current_ball()
  {
    // ガウス分布による射出ばらつき (横: std=5cm, 縦: std=12.5cm)
    std::normal_distribution<double> dist_x(0.0, sigma_x_);
    std::normal_distribution<double> dist_z(0.0, sigma_z_);

    const double offset_x = dist_x(gen_);
    const double offset_z = dist_z(gen_);

    const double impact_x = target_aim_x_ + offset_x;
    const double impact_y = target_aim_y_;
    const double impact_z = target_aim_z_ + offset_z;

    InFlightBall ball;
    ball.start_x = robot_x_ + 0.25 * std::cos(robot_yaw_);
    ball.start_y = robot_y_ + 0.25 * std::sin(robot_yaw_);
    ball.start_z = 0.35;
    ball.target_x = impact_x;
    ball.target_y = impact_y;
    ball.target_z = impact_z;
    ball.flight_duration = 0.65;
    ball.elapsed = 0.0;
    ball.target_tag_ids = current_target_tag_ids_;

    active_balls_.push_back(ball);

    HitRecord hr;
    hr.x = impact_x;
    hr.y = impact_y;
    hr.z = impact_z;
    hr.is_hit = true;
    hit_history_.push_back(hr);
    if (hit_history_.size() > 30) {
      hit_history_.erase(hit_history_.begin());
    }

    RCLCPP_INFO(
      get_logger(),
      "Fired at Target [Aim: (%.2f, %.2f)] | Error: dx=%+.1fcm, dz=%+.1fcm",
      target_aim_x_, target_aim_z_, offset_x * 100.0, offset_z * 100.0);
  }

  void publish_all_markers(const rclcpp::Time & now_stamp)
  {
    const auto & grid = vision_tracker_.get_panel_grid();

    // A. ロボット車体フットプリント
    {
      visualization_msgs::msg::MarkerArray fp_msg;
      visualization_msgs::msg::Marker fp;
      fp.header.stamp = now_stamp;
      fp.header.frame_id = "base_footprint";
      fp.ns = "robot_footprint";
      fp.id = 0;
      fp.type = visualization_msgs::msg::Marker::CUBE;
      fp.action = visualization_msgs::msg::Marker::ADD;
      fp.pose.position.z = 0.10;
      fp.pose.orientation.w = 1.0;
      fp.scale.x = 0.65;
      fp.scale.y = 0.50;
      fp.scale.z = 0.20;
      fp.color.r = 0.2f;
      fp.color.g = 0.8f;
      fp.color.b = 1.0f;
      fp.color.a = 0.70f;
      fp_msg.markers.push_back(fp);
      footprint_pub_->publish(fp_msg);
    }

    // B. ボールマーカー (待機ボール ＆ 飛翔放物線ボール)
    {
      visualization_msgs::msg::MarkerArray ball_msg;
      int32_t b_id = 0;

      for (const auto & b : active_balls_) {
        const double t = std::min(1.0, b.elapsed / b.flight_duration);
        const double cur_x = b.start_x + t * (b.target_x - b.start_x);
        const double cur_y = b.start_y + t * (b.target_y - b.start_y);
        const double arc_h = 4.0 * 0.15 * t * (1.0 - t);
        const double cur_z = b.start_z + t * (b.target_z - b.start_z) + arc_h;

        visualization_msgs::msg::Marker bm;
        bm.header.stamp = now_stamp;
        bm.header.frame_id = "map";
        bm.ns = "soccer_ball";
        bm.id = b_id++;
        bm.type = visualization_msgs::msg::Marker::SPHERE;
        bm.action = visualization_msgs::msg::Marker::ADD;
        bm.pose.position.x = cur_x;
        bm.pose.position.y = cur_y;
        bm.pose.position.z = cur_z;
        bm.pose.orientation.w = 1.0;
        bm.scale.x = 0.22;
        bm.scale.y = 0.22;
        bm.scale.z = 0.22;
        bm.color.r = 1.0f;
        bm.color.g = 0.85f;
        bm.color.b = 0.10f;
        bm.color.a = 1.0f;
        ball_msg.markers.push_back(bm);
      }

      if (state_ != robot_msgs::msg::Game2State::SHOOTING) {
        visualization_msgs::msg::Marker ready_ball;
        ready_ball.header.stamp = now_stamp;
        ready_ball.header.frame_id = "base_footprint";
        ready_ball.ns = "ready_ball";
        ready_ball.id = 999;
        ready_ball.type = visualization_msgs::msg::Marker::SPHERE;
        ready_ball.action = visualization_msgs::msg::Marker::ADD;
        ready_ball.pose.position.x = 0.25;
        ready_ball.pose.position.z = 0.15;
        ready_ball.pose.orientation.w = 1.0;
        ready_ball.scale.x = 0.22;
        ready_ball.scale.y = 0.22;
        ready_ball.scale.z = 0.22;
        ready_ball.color.r = 1.0f;
        ready_ball.color.g = 0.85f;
        ready_ball.color.b = 0.10f;
        ready_ball.color.a = 0.85f;
        ball_msg.markers.push_back(ready_ball);
      }

      ball_pub_->publish(ball_msg);
    }

    // C. 3x3 パネル ＆ AprilTag ＆ 着弾履歴
    {
      visualization_msgs::msg::MarkerArray g2_msg;

      for (const auto & [id, p] : grid) {
        visualization_msgs::msg::Marker pm;
        pm.header.stamp = now_stamp;
        pm.header.frame_id = "map";
        pm.ns = "game2_shoot_panels";
        pm.id = id;
        pm.type = visualization_msgs::msg::Marker::CUBE;
        pm.action = visualization_msgs::msg::Marker::ADD;
        pm.scale.x = 0.36;
        pm.scale.y = 0.03;
        pm.scale.z = 0.36;

        visualization_msgs::msg::Marker tag_m;
        tag_m.header.stamp = now_stamp;
        tag_m.header.frame_id = "map";
        tag_m.ns = "apriltags_plate";
        tag_m.id = 100 + id;
        tag_m.type = visualization_msgs::msg::Marker::CUBE;
        tag_m.action = visualization_msgs::msg::Marker::ADD;
        tag_m.scale.x = 0.18;
        tag_m.scale.y = 0.012;
        tag_m.scale.z = 0.18;
        tag_m.color.r = 0.05f; tag_m.color.g = 0.05f; tag_m.color.b = 0.07f; tag_m.color.a = 0.95f;

        if (p.shot_completed) {
          // 倒れた状態: パネルの下端ヒンジを中心に後ろへ90度倒れる
          pm.pose.position.x = p.x;
          pm.pose.position.y = p.y - 0.18;
          pm.pose.position.z = p.z - 0.18;
          pm.pose.orientation.x = -0.7071;
          pm.pose.orientation.y = 0.0;
          pm.pose.orientation.z = 0.0;
          pm.pose.orientation.w = 0.7071;
          pm.color.r = 0.35f;
          pm.color.g = 0.35f;
          pm.color.b = 0.35f;
          pm.color.a = 0.40f;

          tag_m.pose.position.x = p.x;
          tag_m.pose.position.y = p.y - 0.18;
          tag_m.pose.position.z = p.z - 0.18 + 0.02;
          tag_m.pose.orientation = pm.pose.orientation;
          tag_m.color.a = 0.30f;
        } else {
          pm.pose.position.x = p.x;
          pm.pose.position.y = p.y;
          pm.pose.position.z = p.z;
          pm.pose.orientation.w = 1.0;
          pm.pose.orientation.x = 0.0;
          pm.pose.orientation.y = 0.0;
          pm.pose.orientation.z = 0.0;
          pm.color.r = 0.90f;
          pm.color.g = 0.15f;
          pm.color.b = 0.20f;
          pm.color.a = 0.95f;

          tag_m.pose.position.x = p.x;
          tag_m.pose.position.y = p.y + 0.025;
          tag_m.pose.position.z = p.z;
          tag_m.pose.orientation.w = 1.0;
          tag_m.pose.orientation.x = 0.0;
          tag_m.pose.orientation.y = 0.0;
          tag_m.pose.orientation.z = 0.0;
        }
        g2_msg.markers.push_back(pm);
        g2_msg.markers.push_back(tag_m);
      }

      for (size_t i = 0; i < hit_history_.size(); ++i) {
        const auto & hr = hit_history_[i];
        visualization_msgs::msg::Marker hm;
        hm.header.stamp = now_stamp;
        hm.header.frame_id = "map";
        hm.ns = "g2_hit_dispersion";
        hm.id = 500 + static_cast<int32_t>(i);
        hm.type = visualization_msgs::msg::Marker::SPHERE;
        hm.action = visualization_msgs::msg::Marker::ADD;
        hm.pose.position.x = hr.x;
        hm.pose.position.y = hr.y + 0.02;
        hm.pose.position.z = hr.z;
        hm.pose.orientation.w = 1.0;
        hm.scale.x = 0.08;
        hm.scale.y = 0.02;
        hm.scale.z = 0.08;
        hm.color.r = 0.0f; hm.color.g = 1.0f; hm.color.b = 0.3f; hm.color.a = 0.85f;
        g2_msg.markers.push_back(hm);
      }
      g2_marker_pub_->publish(g2_msg);
    }
  }

  struct HitRecord {
    double x, y, z;
    bool is_hit;
  };

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ball_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr g2_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<robot_msgs::msg::Game2State>::SharedPtr state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string field_side_{"left"};
  double target_dist_{4.0};
  double sigma_x_{0.05};
  double sigma_z_{0.125};
  double mirror_x_{1.0};

  double robot_x_{-3.23};
  double robot_y_{-1.525};
  double robot_yaw_{-1.5708};

  double panel_center_x_{-3.23};
  double panel_center_y_{-5.525};

  uint8_t state_{robot_msgs::msg::Game2State::SEARCHING};
  uint8_t arm_mode_{robot_msgs::msg::ArmPosition::DRIBBLE};
  int active_row_{0};
  double current_target_heading_err_{0.0};
  std::vector<int> current_target_tag_ids_;
  double target_aim_x_{0.0};
  double target_aim_y_{0.0};
  double target_aim_z_{0.0};
  double state_timer_{0.0};

  VisionTracker vision_tracker_;
  TargetSelector target_selector_;
  TargetSelectionConfig target_config_;

  std::vector<InFlightBall> active_balls_;
  std::vector<HitRecord> hit_history_;

  std::random_device rd_;
  std::mt19937 gen_;
};

} // namespace robot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::Game2SimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
