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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 縦向きゲート位置: (X = -4.50, Y = 1.50) [通過方向: 左から右（東向き +X 方向）]
    // 1. スタート枠 (-5.925, 4.950) から発進
    // 2. ゲートの左手前 1.78m (-5.925, 1.50) で停止して東向き (+X) にロングシュート！
    // 3. ボールがゲート (-4.50, 1.50) を股抜き通過してパスエリア (-1.30, 1.50) へ直進
    // 4. ロボットはゲートの上側 (-4.50, 2.50) を迂回旋回してパスエリアへ合流 (1.63s)
    // 5. パスエリア (-1.30, 1.50) でボール回収・投下 (2.50s)
    // 6. スタート地点 (-5.925, 4.950) へ斜め直線で帰還 (3.50s)
    waypoints_ = {
      {-5.925, 4.950, -1.5708, "Start Area", 0.0},
      {-5.925, 1.500,  0.0000, "Shoot Position (1.78m before Gate)", 1.78},
      {-4.500, 2.500,  0.0000, "Loop Around Gate Top Side", 1.63},
      {-1.300, 1.500,  0.0000, "Pass Area Catch & Drop", 2.50},
      {-5.925, 4.950,  2.3562, "Straight Return to Start", 3.50}
    };

    publish_static_planned_path();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30), // 33Hz
      std::bind(&TrajectorySimNode::sim_loop, this));

    RCLCPP_INFO(get_logger(), "TrajectorySimNode: Running Correct West-to-East Shoot & Bypass Simulation");
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

    // 3. ボールのリアルタイム 3D 軌跡 (黄色いボール)
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
      // スタート〜シュート位置: ロボットが保持して南下
      ball.pose.position.x = curr_x + 0.25 * std::cos(curr_yaw);
      ball.pose.position.y = curr_y + 0.25 * std::sin(curr_yaw);
      ball.pose.position.z = 0.11;
    } else if (current_segment_ == 1 || current_segment_ == 2) {
      // シュート後: 西(-5.925)から東(-1.30)に向けてゲート(-4.50)をくぐり直進ローリング
      double progress = (current_segment_ == 1) ? (segment_elapsed_ / 1.78) : 1.0;
      ball.pose.position.x = -5.925 + progress * (-1.30 - (-5.925));
      ball.pose.position.y = 1.50;
      ball.pose.position.z = 0.11;
    } else {
      // パスエリア投下後: パスエリア内に静止
      ball.pose.position.x = -1.30;
      ball.pose.position.y = 1.50;
      ball.pose.position.z = 0.11;
    }
    ball_array.markers.push_back(ball);
    ball_pub_->publish(ball_array);

    // 4. オドメトリ配信
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
