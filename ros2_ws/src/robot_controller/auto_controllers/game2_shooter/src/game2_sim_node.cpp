#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <chrono>

namespace robot_controller
{

struct PanelState
{
  int id;
  int row; // 0: bottom, 1: middle, 2: top
  int col; // 0: left, 1: center, 2: right
  double x, y, z;
  bool is_knocked_down;
};

struct InFlightBall
{
  double start_x, start_y, start_z;
  double target_x, target_y, target_z;
  double flight_duration;
  double elapsed;
  int target_panel_idx;
  bool is_hit;
};

class Game2SimNode : public rclcpp::Node
{
public:
  explicit Game2SimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("game2_sim_node", options),
    gen_(rd_())
  {
    field_side_ = declare_parameter<std::string>("field_side", "left");
    target_dist_ = declare_parameter<double>("target_distance", 4.0); // 4.0m 手前から射出
    sigma_x_ = declare_parameter<double>("dispersion_sigma_x", 0.05); // 横標準偏差 5cm (0.05m)
    sigma_z_ = declare_parameter<double>("dispersion_sigma_z", 0.125); // 縦標準偏差 10-15cm (平均 0.125m)

    mirror_x_ = (field_side_ == "right" || field_side_ == "blue") ? -1.0 : 1.0;

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/game2_sim/markers", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // パネル配置初期化 (3x3 パネル)
    init_panels();

    // ロボット初期位置: パネル群の中心手前 4.0m
    robot_x_ = panel_center_x_;
    robot_y_ = panel_center_y_ + target_dist_; // Y = -5.525 + 4.0 = -1.525
    robot_yaw_ = -M_PI / 2.0; // パネル(南側: -Y)を向く

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&Game2SimNode::sim_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "Game2SimNode initialized! Robot at (%.2f, %.2f) facing panel at (%.2f, %.2f), Dist=%.1fm, Sigma_X=%.2fm, Sigma_Z=%.2fm",
      robot_x_, robot_y_, panel_center_x_, panel_center_y_, target_dist_, sigma_x_, sigma_z_);
  }

private:
  void init_panels()
  {
    panels_.clear();
    panel_center_x_ = (mirror_x_ > 0.0) ? -3.23 : 3.63;
    panel_center_y_ = -5.525;

    // AprilTag ID:
    // Top (Row 2): 16(L), 15(C), 14(R)
    // Middle (Row 1): 19(L), 18(C), 17(R)
    // Bottom (Row 0): 22(L), 21(C), 20(R)
    const int tag_ids[3][3] = {
      {22, 21, 20}, // Row 0 (Bottom)
      {19, 18, 17}, // Row 1 (Middle)
      {16, 15, 14}  // Row 2 (Top)
    };

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        PanelState p;
        p.id = tag_ids[r][c];
        p.row = r;
        p.col = c;
        p.x = panel_center_x_ + (c - 1) * 0.41;
        p.y = panel_center_y_;
        p.z = 0.18 + r * 0.41;
        p.is_knocked_down = false;
        panels_.push_back(p);
      }
    }
  }

  void sim_loop()
  {
    const double dt = 0.03;
    state_timer_ += dt;
    const auto now_stamp = this->now();

    // TF 配信 (map -> base_footprint)
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

    // 自動射出シーケンス (下段 -> 中段 -> 上段、または残存パネルへ1.2秒おきに射出)
    if (state_timer_ >= 1.2) {
      state_timer_ = 0.0;
      shoot_next_ball();
    }

    // 飛翔中のボールの物理シミュレーション & 着弾判定
    for (auto it = active_balls_.begin(); it != active_balls_.end(); ) {
      it->elapsed += dt;
      const double t = std::min(1.0, it->elapsed / it->flight_duration);

      if (t >= 1.0) {
        // 着弾判定
        if (it->is_hit && it->target_panel_idx >= 0 && it->target_panel_idx < static_cast<int>(panels_.size())) {
          panels_[it->target_panel_idx].is_knocked_down = true;
          RCLCPP_INFO(
            get_logger(),
            "[HIT!] Panel Tag #%d (Row %d, Col %d) KNOCKED DOWN!",
            panels_[it->target_panel_idx].id,
            panels_[it->target_panel_idx].row,
            panels_[it->target_panel_idx].col);
        }
        it = active_balls_.erase(it);
      } else {
        ++it;
      }
    }

    // 全パネル倒れたら3秒後にリセットしてサイクルを継続
    bool all_cleared = true;
    for (const auto & p : panels_) {
      if (!p.is_knocked_down) { all_cleared = false; break; }
    }
    if (all_cleared) {
      reset_timer_ += dt;
      if (reset_timer_ > 3.0) {
        init_panels();
        reset_timer_ = 0.0;
        RCLCPP_INFO(get_logger(), "All 9 Panels cleared! Resetting panels for next practice cycle.");
      }
    }

    // RViz / Foxglove 可視化マーカー配信
    publish_markers(now_stamp);
  }

  void shoot_next_ball()
  {
    // 未撃破のパネルを探索 (下段 -> 中段 -> 上段)
    int target_idx = -1;
    for (size_t i = 0; i < panels_.size(); ++i) {
      if (!panels_[i].is_knocked_down) {
        target_idx = static_cast<int>(i);
        break;
      }
    }

    if (target_idx < 0) { return; } // 全て倒れている

    const auto & target_panel = panels_[target_idx];

    // 正規分布（ガウス分布）による射出ばらつきシミュレーション
    // 横方向 (X): std = 5cm (0.05m)
    // 縦方向 (Z): std = 10-15cm (0.125m)
    std::normal_distribution<double> dist_x(0.0, sigma_x_);
    std::normal_distribution<double> dist_z(0.0, sigma_z_);

    const double offset_x = dist_x(gen_);
    const double offset_z = dist_z(gen_);

    const double impact_x = target_panel.x + offset_x;
    const double impact_y = target_panel.y;
    const double impact_z = target_panel.z + offset_z;

    // パネルサイズ (36cm x 36cm = ±0.18m) への命中判定
    const bool is_hit = (std::abs(impact_x - target_panel.x) <= 0.18) &&
                        (std::abs(impact_z - target_panel.z) <= 0.18);

    // 飛翔オブジェクト生成 (初速約15m/s -> 4mを約0.28秒で着弾)
    InFlightBall ball;
    ball.start_x = robot_x_;
    ball.start_y = robot_y_ - 0.25;
    ball.start_z = 0.35; // 射出口高さ
    ball.target_x = impact_x;
    ball.target_y = impact_y;
    ball.target_z = impact_z;
    ball.flight_duration = 0.30; // 0.30秒
    ball.elapsed = 0.0;
    ball.target_panel_idx = target_idx;
    ball.is_hit = is_hit;

    active_balls_.push_back(ball);

    // 着弾履歴マーカー用に保存
    HitRecord hr;
    hr.x = impact_x;
    hr.y = impact_y;
    hr.z = impact_z;
    hr.is_hit = is_hit;
    hit_history_.push_back(hr);
    if (hit_history_.size() > 30) {
      hit_history_.erase(hit_history_.begin());
    }

    RCLCPP_INFO(
      get_logger(),
      "Shooting ball -> Panel #%d (Row %d) | Error: dx=%+.1fcm, dz=%+.1fcm -> %s",
      target_panel.id, target_panel.row, offset_x * 100.0, offset_z * 100.0,
      is_hit ? "HIT" : "MISS");
  }

  void publish_markers(const rclcpp::Time & now_stamp)
  {
    visualization_msgs::msg::MarkerArray msg;
    int32_t id = 0;

    // 1. ロボット車体マーカー (base_footprintフレーム直結)
    {
      visualization_msgs::msg::Marker fp;
      fp.header.stamp = now_stamp;
      fp.header.frame_id = "base_footprint";
      fp.ns = "g2_robot";
      fp.id = id++;
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
      fp.color.a = 0.60f;
      msg.markers.push_back(fp);
    }

    // 2. 3x3 パネルマーカー (倒れたパネルは倒れるアニメーション)
    for (const auto & p : panels_) {
      visualization_msgs::msg::Marker panel_m;
      panel_m.header.stamp = now_stamp;
      panel_m.header.frame_id = "map";
      panel_m.ns = "g2_panels";
      panel_m.id = id++;
      panel_m.type = visualization_msgs::msg::Marker::CUBE;
      panel_m.action = visualization_msgs::msg::Marker::ADD;
      panel_m.pose.position.x = p.x;
      panel_m.pose.position.y = p.y;
      panel_m.pose.position.z = p.z;

      if (p.is_knocked_down) {
        // 倒れたパネル: 後ろへ90度倒れる
        panel_m.pose.position.y -= 0.18;
        panel_m.pose.position.z -= 0.18;
        panel_m.pose.orientation.x = 0.7071;
        panel_m.pose.orientation.w = 0.7071;
        panel_m.color.r = 0.3f;
        panel_m.color.g = 0.3f;
        panel_m.color.b = 0.3f;
        panel_m.color.a = 0.30f;
      } else {
        panel_m.pose.orientation.w = 1.0;
        panel_m.color.r = 0.90f;
        panel_m.color.g = 0.15f;
        panel_m.color.b = 0.20f;
        panel_m.color.a = 0.95f;
      }
      panel_m.scale.x = 0.36;
      panel_m.scale.y = 0.03;
      panel_m.scale.z = 0.36;
      msg.markers.push_back(panel_m);
    }

    // 3. 飛翔中のボール
    for (const auto & b : active_balls_) {
      const double t = std::min(1.0, b.elapsed / b.flight_duration);
      // 放物線弾道
      const double cur_x = b.start_x + t * (b.target_x - b.start_x);
      const double cur_y = b.start_y + t * (b.target_y - b.start_y);
      const double arc_h = 4.0 * 0.15 * t * (1.0 - t); // 15cmの山なり
      const double cur_z = b.start_z + t * (b.target_z - b.start_z) + arc_h;

      visualization_msgs::msg::Marker ball_m;
      ball_m.header.stamp = now_stamp;
      ball_m.header.frame_id = "map";
      ball_m.ns = "g2_flight_balls";
      ball_m.id = id++;
      ball_m.type = visualization_msgs::msg::Marker::SPHERE;
      ball_m.action = visualization_msgs::msg::Marker::ADD;
      ball_m.pose.position.x = cur_x;
      ball_m.pose.position.y = cur_y;
      ball_m.pose.position.z = cur_z;
      ball_m.pose.orientation.w = 1.0;
      ball_m.scale.x = 0.22;
      ball_m.scale.y = 0.22;
      ball_m.scale.z = 0.22;
      ball_m.color.r = 1.0f;
      ball_m.color.g = 0.85f;
      ball_m.color.b = 0.10f;
      ball_m.color.a = 1.0f;
      msg.markers.push_back(ball_m);
    }

    // 4. 着弾履歴・ばらつき分布マーカー (緑: HIT, 赤: MISS)
    for (size_t i = 0; i < hit_history_.size(); ++i) {
      const auto & hr = hit_history_[i];
      visualization_msgs::msg::Marker hit_m;
      hit_m.header.stamp = now_stamp;
      hit_m.header.frame_id = "map";
      hit_m.ns = "g2_hit_dispersion";
      hit_m.id = id++;
      hit_m.type = visualization_msgs::msg::Marker::SPHERE;
      hit_m.action = visualization_msgs::msg::Marker::ADD;
      hit_m.pose.position.x = hr.x;
      hit_m.pose.position.y = hr.y + 0.02; // パネルの直前
      hit_m.pose.position.z = hr.z;
      hit_m.pose.orientation.w = 1.0;
      hit_m.scale.x = 0.08;
      hit_m.scale.y = 0.02;
      hit_m.scale.z = 0.08;
      if (hr.is_hit) {
        hit_m.color.r = 0.0f; hit_m.color.g = 1.0f; hit_m.color.b = 0.3f; hit_m.color.a = 0.8f;
      } else {
        hit_m.color.r = 1.0f; hit_m.color.g = 0.1f; hit_m.color.b = 0.1f; hit_m.color.a = 0.8f;
      }
      msg.markers.push_back(hit_m);
    }

    marker_pub_->publish(msg);
  }

  struct HitRecord {
    double x, y, z;
    bool is_hit;
  };

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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

  double state_timer_{0.0};
  double reset_timer_{0.0};

  std::vector<PanelState> panels_;
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
