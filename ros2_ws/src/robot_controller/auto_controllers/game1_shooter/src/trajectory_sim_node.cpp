#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // GAME1 ルート（上側スタート -> 縦向きゲート X=-4.5, Y=1.5 -> パスエリア X=-1.3, Y=1.5 -> スタート帰還）
    // Start Area: (-5.50, 4.50)
    // 縦向きゲート前アプローチ: (-4.50, 2.80)
    // ゲートくぐり/シュート: (-4.50, 1.50)
    // ゲート横回り込み: (-4.00, 0.80)
    // パスエリア投入: (-1.30, 1.50)
    // スタート地点へ斜め直線帰還: (-5.50, 4.50)
    waypoints_ = {
      {-5.925, 4.950, -1.5708, "Start Area", 0.0},
      {-4.500, 2.800, -1.5708, "Vertical Gate Approach", 1.78},
      {-4.500, 1.500, -1.5708, "Vertical Gate Pass Through", 1.00},
      {-4.000, 0.800,  0.0000, "Gate Loop Around", 1.63},
      {-1.300, 1.500,  0.0000, "Pass Area Drop", 2.50},
      {-5.925, 4.950,  2.3562, "Straight Return", 3.50}
    };

    publish_static_planned_path();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30), // 33Hz
      std::bind(&TrajectorySimNode::sim_loop, this));

    RCLCPP_INFO(get_logger(), "TrajectorySimNode: Running Real-time Trajectory Simulation Loop");
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
      // ループ再生（スタート地点に戻ったらリセットして再走）
      current_segment_ = 0;
      segment_elapsed_ = 0.0;
      executed_path_.poses.clear();
    }

    const auto & p0 = waypoints_[current_segment_];
    const auto & p1 = waypoints_[current_segment_ + 1];
    const double dur = std::max(0.5, p1.duration);

    segment_elapsed_ += dt;
    double t = std::min(1.0, segment_elapsed_ / dur);
    // Smooth step interpolation
    double s = t * t * (3.0 - 2.0 * t);

    double curr_x = p0.x + (p1.x - p0.x) * s;
    double curr_y = p0.y + (p1.y - p0.y) * s;
    double curr_yaw = p0.yaw + (p1.yaw - p0.yaw) * s;

    if (t >= 1.0) {
      current_segment_++;
      segment_elapsed_ = 0.0;
    }

    const auto now_stamp = this->now();

    // 1. TF 配信 (map -> base_footprint) でロボット 3D モデルをリアルタイム移動！
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

    // 2. 走行軌跡 (Executed Path) を記録・配信
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
    if (executed_path_.poses.size() > 500) {
      executed_path_.poses.erase(executed_path_.poses.begin());
    }
    current_path_pub_->publish(executed_path_);

    // 3. オドメトリ配信
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
