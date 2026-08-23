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

    // ロボット寸法: 全長 0.60m, 全幅 0.45m (マージン込み 0.70m x 0.55m)
    // 上側ゲート: X = -3.20m, Y = +3.80m (横向き |--|, 支柱: X=-3.50〜-2.90)
    // 縦向きゲート: X = -4.50m, Y = +1.50m (縦向き I, 支柱: Y=1.20〜1.80)
    //
    // 帰還ルート:
    // パスエリア (-1.30, 1.50) から左上スタート (-5.925, 4.950) へ一直線に戻ると、
    // 上側ゲート (-3.20, 3.80) の右下角にロボット外周（幅0.55m）が接触する恐れがあるため、
    // 帰還中間経由点 (-2.20, 3.00) または (-1.80, 3.50) を通って上側ゲートの【右側】を余裕（0.8m以上）をもって安全回避！
    waypoints_ = {
      {-5.925, 4.950, -1.5708, "Start Area", 0.0},
      {-5.925, 1.500,  0.0000, "Shoot Outside Gate", 1.78},
      {-4.500, 0.400,  0.0000, "Bypass Gate Bottom Side", 1.63},
      {-3.500, 1.500,  0.0000, "Catch Ball & Re-dribble", 1.35},
      {-1.300, 1.500,  0.0000, "Pass Area Drop", 2.00},
      {-2.000, 3.200,  2.0000, "Safe Waypoint (Clear Top Gate)", 1.80},
      {-5.925, 4.950,  2.3562, "Straight Return to Start", 2.20}
    };

    publish_static_planned_path();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30), // 33Hz
      std::bind(&TrajectorySimNode::sim_loop, this));

    RCLCPP_INFO(get_logger(), "TrajectorySimNode: Running Collision-Free Return Simulation");
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
    const double dt = 0.03;
    current_time_ += dt;

    if (current_segment_ >= waypoints_.size() - 1) {
      current_segment_ = 0;
      segment_elapsed_ = 0.0;
      executed_path_.poses.clear();
    }

    const auto & p0 = waypoints_[current_segment_];
    const auto & p1 = waypoints_[current_segment_ + 1];
    const double dur = std::max(0.5, p1.duration);

    segment_elapsed_ += dt;
    double t = std::min(1.0, segment_elapsed_ / dur);
    double s = t * t * (3.0 - 2.0 * t); // Smooth-step

    double curr_x = p0.x + (p1.x - p0.x) * s;
    double curr_y = p0.y + (p1.y - p0.y) * s;
    double curr_yaw = p0.yaw + (p1.yaw - p0.yaw) * s;

    if (t >= 1.0) {
      current_segment_++;
      segment_elapsed_ = 0.0;
    }

    const auto now_stamp = this->now();

    // 1. TF 配信 (map -> base_footprint)
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now_stamp;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = curr_x;
    tf_msg.transform.translation.y = curr_y;
    tf_msg.transform.translation.z = 0.05;
    tf_msg.transform.rotation.z = std::sin(curr_yaw / 2.0);
    tf_msg.transform.rotation.w = std::cos(curr_yaw / 2.0);
    tf_broadcaster_->sendTransform(tf_msg);

    // 2. 走行軌跡 (Executed Path)
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now_stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = curr_x;
    ps.pose.position.y = curr_y;
    ps.pose.position.z = 0.05;
    ps.pose.orientation = tf_msg.transform.rotation;
    executed_path_.header.stamp = now_stamp;
    executed_path_.header.frame_id = "map";
    executed_path_.poses.push_back(ps);
    if (executed_path_.poses.size() > 600) {
      executed_path_.poses.erase(executed_path_.poses.begin());
    }
    current_path_pub_->publish(executed_path_);

    // 3. ロボット車体フットプリント (実寸 0.60m x 0.45m の可視化バウンディングボックス)
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
    fp.scale.x = 0.65; // 全長 (マージン込み)
    fp.scale.y = 0.50; // 全幅 (マージン込み)
    fp.scale.z = 0.25;
    fp.color.r = 0.2f;
    fp.color.g = 0.7f;
    fp.color.b = 1.0f;
    fp.color.a = 0.35f; // 半透明
    fp_array.markers.push_back(fp);
    footprint_pub_->publish(fp_array);

    // 4. ボールのリアルタイム 3D 軌跡 (黄色いボール)
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

    if (current_segment_ == 0) {
      // スタート〜シュート位置: ロボットが保持
      ball.pose.position.x = curr_x + 0.25 * std::cos(curr_yaw);
      ball.pose.position.y = curr_y + 0.25 * std::sin(curr_yaw);
      ball.pose.position.z = 0.11;
    } else if (current_segment_ == 1) {
      // シュート〜迂回中: ボールがゲート (-4.50) をくぐって合流地点 (-3.50, 1.50) まで転がる
      double progress = std::min(1.0, segment_elapsed_ / 1.63);
      ball.pose.position.x = -5.925 + progress * (-3.50 - (-5.925));
      ball.pose.position.y = 1.50;
      ball.pose.position.z = 0.11;
    } else if (current_segment_ == 2 || current_segment_ == 3) {
      // ボール再保持〜パスエリアまでドリブル運搬
      ball.pose.position.x = curr_x + 0.25 * std::cos(curr_yaw);
      ball.pose.position.y = curr_y + 0.25 * std::sin(curr_yaw);
      ball.pose.position.z = 0.11;
    } else {
      // パスエリア投下後: パスエリア内に静止
      ball.pose.position.x = -1.30;
      ball.pose.position.y = 1.50;
      ball.pose.position.z = 0.11;
    }
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
  size_t current_segment_{0};
  double segment_elapsed_{0.0};
  double current_time_{0.0};
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
