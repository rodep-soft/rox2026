#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <vector>

namespace robot_controller
{

struct Waypoint {
  double x;
  double y;
  double yaw;
  std::string name;
  double duration; // seconds
};

class TrajectorySimNode : public rclcpp::Node
{
public:
  explicit TrajectorySimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trajectory_sim_node", options)
  {
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/robot/trajectory_plan", rclcpp::QoS(1).reliable().transient_local());
    current_path_pub_ = create_publisher<nav_msgs::msg::Path>("/robot/trajectory_executed", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/simulated", 10);
    ball_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/sim/ball_marker", 10);
    footprint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/robot/footprint_marker", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── ROS 2 パラメータ読み込み (game1.yaml から直接注入) ──
    kp_linear_ = declare_parameter<double>("kp_linear", 1.0);
    kp_angular_ = declare_parameter<double>("kp_angular", 2.0);   // 高すぎると発振するので2.0に抑える
    max_linear_vel_ = declare_parameter<double>("max_linear_vel", 3.5);
    max_angular_vel_ = declare_parameter<double>("max_angular_vel", 3.5); // 実機と同一: 高速旋回
    pos_tolerance_ = declare_parameter<double>("pos_tolerance", 0.08);
    yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.05);

    const std::string field_side = declare_parameter<std::string>("field_side", "left");
    mirror_x_ = (field_side == "right" || field_side == "blue") ? -1.0 : 1.0;
    const double mirror_x = mirror_x_;

    const double wp_start_x = declare_parameter<double>("wp_start_x", -5.925) * mirror_x;
    const double wp_start_y = declare_parameter<double>("wp_start_y", 4.950);
    const double wp_start_yaw = (mirror_x < 0.0) ? -1.5708 : declare_parameter<double>("wp_start_yaw", -1.5708);

    const double wp_gate_x = declare_parameter<double>("wp_gate_x", -5.925) * mirror_x;
    const double wp_gate_y = declare_parameter<double>("wp_gate_y", 1.500);
    const double wp_gate_yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_gate_yaw", 0.0);

    const double wp_around_x = declare_parameter<double>("wp_around_gate_x", -4.500) * mirror_x;
    const double wp_around_y = declare_parameter<double>("wp_around_gate_y", 0.500);
    const double wp_around_yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_around_gate_yaw", 0.0);

    const double wp_ball_x = declare_parameter<double>("wp_ball_x", -2.700) * mirror_x;
    const double wp_ball_y = declare_parameter<double>("wp_ball_y", 1.500);
    const double wp_ball_yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_ball_yaw", 0.0);

    const double wp_pass_x = declare_parameter<double>("wp_pass_area_x", -2.030) * mirror_x;
    const double wp_pass_y = declare_parameter<double>("wp_pass_area_y", 1.641);
    const double wp_pass_yaw = (mirror_x < 0.0) ? M_PI : declare_parameter<double>("wp_pass_area_yaw", 0.0);

    const double wp_apex_x = declare_parameter<double>("wp_return_apex_x", -3.800) * mirror_x;
    const double wp_apex_y = declare_parameter<double>("wp_return_apex_y", 3.200);
    const double wp_apex_yaw = (mirror_x < 0.0) ? 0.0 : declare_parameter<double>("wp_return_apex_yaw", 0.0);

    // 帰還時・初期時ともに常にゲート射出方向 (yaw = 0.0) を向き、全行程で旋回ゼロ
    const double return_yaw = 0.0;
    waypoints_ = {
      {wp_start_x,  wp_start_y,  wp_start_yaw,  "Start Area", 0.0},
      {wp_gate_x,   wp_gate_y,   wp_gate_yaw,   "Shoot Outside Gate", 1.78},
      {wp_around_x, wp_around_y, wp_around_yaw, "Bypass Gate Bottom", 1.40},
      {wp_ball_x,   wp_ball_y,   wp_ball_yaw,   "Catch Ball & Dribble", 1.20},
      {wp_pass_x,   wp_pass_y,   wp_pass_yaw,   "Pass Area Drop", 1.80},
      {wp_apex_x,   wp_apex_y,   wp_apex_yaw,   "Optimal Clearance Apex", 1.30},
      {wp_start_x,  wp_start_y,  return_yaw,    "Fast Straight Dash to Start", 1.50}
    };

    // Waypoint リスト: Start(0) -> Gate(1:Fly) -> Around(2:Fly) -> Ball(3:Fly) -> Pass(4:Touch) -> Apex(5:Fly) -> Start(6:Stop)
    curr_x_ = waypoints_[0].x;
    curr_y_ = waypoints_[0].y;
    // 初期向き: ゲート正面 (yaw = 0.0)
    curr_yaw_ = 0.0;
    current_segment_ = 1;

    publish_static_planned_path();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30), // 33Hz
      std::bind(&TrajectorySimNode::sim_loop, this));

    RCLCPP_INFO(get_logger(), "TrajectorySimNode: Configured for field_side='%s' from game1.yaml!", field_side.c_str());
  }

private:
  void publish_static_planned_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();

    for (const auto & wp : waypoints_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.position.z = 0.05;
      ps.pose.orientation.z = std::sin(wp.yaw / 2.0);
      ps.pose.orientation.w = std::cos(wp.yaw / 2.0);
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  void sim_loop()
  {
    const double dt = 0.03; // 33Hz
    current_time_ += dt;

    // スタート位置で人がボールを渡す待機フェーズ
    bool skip_movement = false;
    if (current_segment_ >= static_cast<int>(waypoints_.size())) {
      wp_wait_timer_ += dt;
      if (wp_wait_timer_ >= 1.5) {
        current_segment_ = 1;
        wp_wait_timer_ = 0.0;
        executed_path_.poses.clear();
      }
      vx_ = 0.0; vy_ = 0.0; vyaw_ = 0.0;
      skip_movement = true;
    }

    if (!skip_movement) {
      const auto & target = waypoints_[current_segment_];
      const double dx_world = target.x - curr_x_;
      const double dy_world = target.y - curr_y_;
      const double dist = std::hypot(dx_world, dy_world);
      const double yaw_err = std::remainder(target.yaw - curr_yaw_, 2.0 * M_PI);
      // フライスルー判定: 1=Gate(走りながら射出), 2=Around(gate bypass), 3=Ball(走りながらキャッチ), 5=Apex(return)
      // 停止・減速調整を一切挟まず、パスエリア (segment 4) までノンストップで滑らかに直行
      const bool is_fly_through = (current_segment_ == 1 || current_segment_ == 2 || current_segment_ == 3 || current_segment_ == 5);
      const double arrive_threshold = (current_segment_ == 1 || current_segment_ == 3) ? 0.35 : (is_fly_through ? 0.35 : pos_tolerance_);

      double target_vx = 0.0, target_vy = 0.0;
      if (dist > 1e-4) {
        const double speed_limit = is_fly_through ? max_linear_vel_ : std::min(max_linear_vel_, kp_linear_ * dist);
        target_vx = speed_limit * (dx_world / dist);
        target_vy = speed_limit * (dy_world / dist);
      }
      const double target_vyaw = std::clamp(kp_angular_ * yaw_err, -max_angular_vel_, max_angular_vel_);
      const double alpha_lin = dt / 0.40;
      const double alpha_ang = dt / 0.35;
      vx_ += (target_vx - vx_) * std::min(1.0, alpha_lin);
      vy_ += (target_vy - vy_) * std::min(1.0, alpha_lin);
      vyaw_ += (target_vyaw - vyaw_) * std::min(1.0, alpha_ang);
      curr_x_ += vx_ * dt;
      curr_y_ += vy_ * dt;
      curr_yaw_ = std::remainder(curr_yaw_ + vyaw_ * dt, 2.0 * M_PI);

      if (is_fly_through) {
        if (dist <= arrive_threshold) { current_segment_++; }
      } else {
        // パスエリア (segment 4) はぶつける勢いで寄せるだけなので許容半径0.25mで即時判定
        const double tolerance = (current_segment_ == 4) ? 0.25 : pos_tolerance_;
        if (dist <= tolerance && (current_segment_ == 4 || std::abs(yaw_err) <= yaw_tolerance_)) {
          wp_wait_timer_ += dt;
          const double wait_time = (current_segment_ == 4) ? 0.8 : 0.0;
          if (wp_wait_timer_ >= wait_time) { current_segment_++; wp_wait_timer_ = 0.0; }
        } else {
          wp_wait_timer_ = 0.0;
        }
      }
    }


    const auto now_stamp = this->now();

    // 1. TF 配信 (map -> base_footprint)
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now_stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = curr_x_;
    tf_msg.transform.translation.y = curr_y_;
    tf_msg.transform.translation.z = 0.05;
    tf_msg.transform.rotation.z = std::sin(curr_yaw_ / 2.0);
    tf_msg.transform.rotation.w = std::cos(curr_yaw_ / 2.0);
    tf_broadcaster_->sendTransform(tf_msg);

    // 2. 走行軌跡 (Executed Path)
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now_stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = curr_x_;
    ps.pose.position.y = curr_y_;
    ps.pose.position.z = 0.05;
    ps.pose.orientation = tf_msg.transform.rotation;
    executed_path_.header.stamp = now_stamp;
    executed_path_.header.frame_id = "map";
    executed_path_.poses.push_back(ps);
    if (executed_path_.poses.size() > 600) {
      executed_path_.poses.erase(executed_path_.poses.begin());
    }
    current_path_pub_->publish(executed_path_);

    // 3. ロボット車体フットプリント (0.65m x 0.50m)
    visualization_msgs::msg::MarkerArray fp_array;
    visualization_msgs::msg::Marker fp;
    fp.header.stamp = now_stamp;
    fp.header.frame_id = "map";
    fp.ns = "robot_footprint";
    fp.id = 0;
    fp.type = visualization_msgs::msg::Marker::CUBE;
    fp.action = visualization_msgs::msg::Marker::ADD;
    fp.pose = ps.pose;
    fp.pose.position.z = 0.15;
    fp.scale.x = 0.65;
    fp.scale.y = 0.50;
    fp.scale.z = 0.25;
    fp.color.r = 0.2f;
    fp.color.g = 0.8f;
    fp.color.b = 1.0f;
    fp.color.a = 0.35f;
    fp_array.markers.push_back(fp);
    footprint_pub_->publish(fp_array);

    // 4. ボールのリアルタイム 3D 軌跡
    visualization_msgs::msg::MarkerArray ball_array;
    visualization_msgs::msg::Marker ball;
    ball.header.stamp = now_stamp;
    ball.header.frame_id = "map";
    ball.ns = "soccer_ball";
    ball.id = 0;
    ball.type = visualization_msgs::msg::Marker::SPHERE;
    ball.action = visualization_msgs::msg::Marker::ADD;
    ball.scale.x = 0.22;
    ball.scale.y = 0.22;
    ball.scale.z = 0.22;
    ball.color.r = 1.0f;
    ball.color.g = 0.85f;
    ball.color.b = 0.10f;
    ball.color.a = 1.0f;

    const double target_ball_stop_x = waypoints_[3].x;
    const double target_ball_stop_y = waypoints_[3].y;

    if (current_segment_ >= static_cast<int>(waypoints_.size())) {
      // スタート待機中: 人がボールをロボットに渡すアニメーション (0.7s後にボールがロボット前方に現れる)
      if (wp_wait_timer_ > 0.7) {
        ball_x_ = curr_x_ + 0.25 * std::cos(curr_yaw_);
        ball_y_ = curr_y_ + 0.25 * std::sin(curr_yaw_);
      }
      // 0.7s未満はパスエリアのボールがフェードせず残る（前サイクルの投下位置）
    } else if (current_segment_ <= 1) {
      // 射出前: ロボット前方に保持 (スタート時から保持済み)
      ball_x_ = curr_x_ + 0.25 * std::cos(curr_yaw_);
      ball_y_ = curr_y_ + 0.25 * std::sin(curr_yaw_);
      ball_shot_time_ = 0.0;
      ball_is_caught_ = false;
    } else if (!ball_is_caught_) {
      // ゲート射出後: 実機のバネ力に合わせた適度な初速でゲートを潜り抜け、芝/床摩擦で自然にスーッと減速停止 (2.5秒かけてゆっくり転がる)
      ball_shot_time_ += dt;
      const double shoot_start_x = waypoints_[1].x;
      double roll_t = std::min(1.0, ball_shot_time_ / 2.6);
      // 3次イージング曲線（初速が穏やかで、徐々に滑らかに減速停止）
      double smooth_roll = 1.0 - std::pow(1.0 - roll_t, 3.0);
      ball_x_ = shoot_start_x + smooth_roll * (target_ball_stop_x - shoot_start_x);
      ball_y_ = waypoints_[1].y;

      // ロボットがボールに接触した段階でドリブルキャッチ判定
      const double dist_robot_to_ball = std::hypot(curr_x_ - ball_x_, curr_y_ - ball_y_);
      if (current_segment_ >= 3 && dist_robot_to_ball < 0.35) {
        ball_is_caught_ = true;
      }
    } else if (current_segment_ == 3 || (current_segment_ == 4 && wp_wait_timer_ < 0.1)) {
      // ドリブルキャッチ後: ロボット前方の保持位置へワープせず滑らかに吸着追従 (1階遅れで手元へスッと収まる)
      const double target_hold_x = curr_x_ + 0.25 * std::cos(curr_yaw_);
      const double target_hold_y = curr_y_ + 0.25 * std::sin(curr_yaw_);
      ball_x_ += (target_hold_x - ball_x_) * std::min(1.0, 15.0 * dt);
      ball_y_ += (target_hold_y - ball_y_) * std::min(1.0, 15.0 * dt);
    } else if (current_segment_ == 4) {
      // パスエリアぶつけ停止 & L1ゆっくり押し出し: アームからパスエリア中心 (X = ±1.316m, Y = 1.641m) へボールを優しく置く
      double progress = std::min(1.0, wp_wait_timer_ / 0.7);
      const double target_pass_x = (mirror_x_ < 0.0) ? 1.316 : -1.316;
      ball_x_ = (curr_x_ + 0.25 * std::cos(curr_yaw_)) + progress * (target_pass_x - (curr_x_ + 0.25 * std::cos(curr_yaw_)));
      ball_y_ = 1.641;
    } else {
      // パスエリア投下完了後: パスエリア内にボールが静止
      ball_x_ = (mirror_x_ < 0.0) ? 1.316 : -1.316;
      ball_y_ = 1.641;
    }

    ball.pose.position.x = ball_x_;
    ball.pose.position.y = ball_y_;
    ball.pose.position.z = 0.11;

    ball_array.markers.push_back(ball);
    ball_pub_->publish(ball_array);

    // 5. オドメトリ配信
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now_stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose = ps.pose;
    odom_pub_->publish(odom);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ball_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Waypoint> waypoints_;
  double gate_center_x_{-4.5};
  double gate_center_y_{1.5};
  double ball_catch_x_{-3.5};
  double ball_catch_y_{1.5};
  double pass_drop_x_{-1.3};
  double pass_drop_y_{1.5};

  size_t current_segment_{0};
  double segment_elapsed_{0.0};
  double current_time_{0.0};
  double wp_wait_timer_{0.0};
  double mirror_x_{1.0};

  double curr_x_{-5.925};
  double curr_y_{4.950};
  double curr_yaw_{-1.5708};
  double vx_{0.0};
  double vy_{0.0};
  double vyaw_{0.0};

  double ball_x_{-5.925};
  double ball_y_{4.950};
  double ball_vx_{0.0};
  double ball_shot_time_{0.0};
  bool ball_is_caught_{false};

  double kp_linear_{1.2};
  double kp_angular_{2.0};
  double max_linear_vel_{1.5};
  double max_angular_vel_{1.5};
  double pos_tolerance_{0.08};
  double yaw_tolerance_{0.05};

  nav_msgs::msg::Path executed_path_;
};

}  // namespace robot_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_controller::TrajectorySimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
