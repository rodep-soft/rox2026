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
    const double wp_start_x = declare_parameter<double>("wp_start_x", -5.925);
    const double wp_start_y = declare_parameter<double>("wp_start_y", 4.950);
    const double wp_start_yaw = declare_parameter<double>("wp_start_yaw", -1.5708);

    const double wp_gate_x = declare_parameter<double>("wp_gate_x", -5.925);
    const double wp_gate_y = declare_parameter<double>("wp_gate_y", 1.500);
    const double wp_gate_yaw = declare_parameter<double>("wp_gate_yaw", 0.0);

    const double wp_around_x = declare_parameter<double>("wp_around_gate_x", -4.500);
    const double wp_around_y = declare_parameter<double>("wp_around_gate_y", 0.500);
    const double wp_around_yaw = declare_parameter<double>("wp_around_gate_yaw", 0.0);

    const double wp_ball_x = declare_parameter<double>("wp_ball_x", -3.500);
    const double wp_ball_y = declare_parameter<double>("wp_ball_y", 1.500);
    const double wp_ball_yaw = declare_parameter<double>("wp_ball_yaw", 0.0);

    const double wp_pass_x = declare_parameter<double>("wp_pass_area_x", -1.300);
    const double wp_pass_y = declare_parameter<double>("wp_pass_area_y", 1.500);
    const double wp_pass_yaw = declare_parameter<double>("wp_pass_area_yaw", 0.0);

    const double wp_apex_x = declare_parameter<double>("wp_return_apex_x", -3.800);
    const double wp_apex_y = declare_parameter<double>("wp_return_apex_y", 2.800);
    const double wp_apex_yaw = declare_parameter<double>("wp_return_apex_yaw", 2.200);

    // game1.yaml の全パラメータから完全構築
    waypoints_ = {
      {wp_start_x,  wp_start_y,  wp_start_yaw,  "Start Area", 0.0},
      {wp_gate_x,   wp_gate_y,   wp_gate_yaw,   "Shoot Outside Gate", 1.78},
      {wp_around_x, wp_around_y, wp_around_yaw, "Bypass Gate Bottom", 1.40},
      {wp_ball_x,   wp_ball_y,   wp_ball_yaw,   "Catch Ball & Dribble", 1.20},
      {wp_pass_x,   wp_pass_y,   wp_pass_yaw,   "Pass Area Drop", 1.80},
      {wp_apex_x,   wp_apex_y,   wp_apex_yaw,   "Optimal Clearance Apex", 1.30},
      {wp_start_x,  wp_start_y,  2.3562,        "Fast Straight Dash to Start", 1.50}
    };

    gate_center_x_ = (wp_gate_x + wp_ball_x) / 2.0;
    gate_center_y_ = wp_gate_y;
    ball_catch_x_ = wp_ball_x;
    ball_catch_y_ = wp_ball_y;
    pass_drop_x_ = wp_pass_x;
    pass_drop_y_ = wp_pass_y;

    publish_static_planned_path();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30), // 33Hz
      std::bind(&TrajectorySimNode::sim_loop, this));

    RCLCPP_INFO(get_logger(), "TrajectorySimNode: Configured DIRECTLY from game1.yaml ROS parameters!");
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
    const double dur = std::max(0.3, p1.duration);

    segment_elapsed_ += dt;
    double t = std::min(1.0, segment_elapsed_ / dur);
    double s = t * t * (3.0 - 2.0 * t); // Smooth-step curve

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

    if (current_segment_ == 0) {
      ball.pose.position.x = curr_x + 0.25 * std::cos(curr_yaw);
      ball.pose.position.y = curr_y + 0.25 * std::sin(curr_yaw);
      ball.pose.position.z = 0.11;
    } else if (current_segment_ == 1) {
      double progress = std::min(1.0, segment_elapsed_ / 1.40);
      const auto & p_shoot = waypoints_[1];
      ball.pose.position.x = p_shoot.x + progress * (ball_catch_x_ - p_shoot.x);
      ball.pose.position.y = p_shoot.y;
      ball.pose.position.z = 0.11;
    } else if (current_segment_ == 2 || current_segment_ == 3) {
      ball.pose.position.x = curr_x + 0.25 * std::cos(curr_yaw);
      ball.pose.position.y = curr_y + 0.25 * std::sin(curr_yaw);
      ball.pose.position.z = 0.11;
    } else {
      ball.pose.position.x = pass_drop_x_;
      ball.pose.position.y = pass_drop_y_;
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
  double gate_center_x_{-4.5};
  double gate_center_y_{1.5};
  double ball_catch_x_{-3.5};
  double ball_catch_y_{1.5};
  double pass_drop_x_{-1.3};
  double pass_drop_y_{1.5};

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
