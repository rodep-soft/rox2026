#ifndef GAME1_SHOOTER__GAME1_AUTO_NODE_HPP_
#define GAME1_SHOOTER__GAME1_AUTO_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace robot_controller
{

struct Waypoint
{
  double x;
  double y;
  double yaw;
};

enum class Game1State : uint8_t
{
  STANDBY = 0,
  NAV_TO_GATE = 1,
  FIRE_GATE_SPRING = 2,
  NAV_AROUND_GATE = 3,         // ゲートの横を通って回り込む
  SEARCH_AND_CATCH_BALL = 4,   // YOLOカメラでボールを動的発見＆バックスピン追従キャッチ
  NAV_TO_PASS_AREA = 5,
  FIRE_PASS_SPRING = 6,
  NAV_TO_START = 7,
  COMPLETED = 8,
  TEST_SINGLE_WP = 9,          // 目標1つの移動テストモード
};

class Game1AutoNode : public rclcpp::Node
{
public:
  explicit Game1AutoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void control_loop();
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void ball_detection_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void publish_commands(
    const geometry_msgs::msg::Twist & cmd_vel,
    bool dribble_enabled,
    uint8_t arm_position,
    bool spring_fire,
    bool spring_slow_fire = false);

  // メカナム特化型 全方位ホロノミック追従制御 (Field-Oriented to Body-Frame)
  geometry_msgs::msg::Twist compute_holonomic_pursuit(const Waypoint & target, double speed_limit = -1.0);
  bool is_aligned_to_target(const Waypoint & target);

  // Subscriptions & Publishers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ball_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_slow_fire_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double kp_linear_{1.0};
  double kp_angular_{2.0};
  double max_linear_vel_{3.5};
  double max_angular_vel_{3.5};
  double pos_tolerance_{0.05};
  double yaw_tolerance_{0.05};
  std::string field_side_{"left"};

  // Test Mode Parameters
  bool test_mode_{false};
  double test_dist_x_{1.0};
  double test_dist_y_{0.0};
  double test_max_vel_{0.5};
  double test_start_x_{0.0};
  double test_start_y_{0.0};
  double test_start_yaw_{0.0};
  double imu_yaw_{0.0};
  Waypoint wp_test_{1.0, 0.0, 0.0};

  // State Variables
  Game1State state_{Game1State::STANDBY};
  bool is_enabled_{false};
  rclcpp::Time state_start_time_;

  // Waypoints (x, y, yaw)
  Waypoint wp_gate_{1.5, 0.0, 0.0};          // ゲート射出位置
  Waypoint wp_around_gate_{2.5, 1.0, 0.0};    // 回り込み中間点
  Waypoint wp_ball_{3.5, 0.0, 0.0};           // ボール吸い寄せ位置
  Waypoint wp_pass_area_{2.0, -1.0, -1.57};   // パスエリア射出位置
  Waypoint wp_start_{0.0, 0.0, 0.0};          // スタート位置

  // Dynamic Ball Detection (YOLO / Camera)
  bool ball_detected_{false};
  double detected_ball_x_{0.0};
  double detected_ball_y_{0.0};
  rclcpp::Time last_ball_detection_time_;

  // EKF Filtered Odometry Feedback (/odometry/filtered)
  bool odom_received_{false};
  double current_x_{0.0};
  double current_y_{0.0};

  // IMU Feedback
  bool imu_received_{false};
  double raw_yaw_{0.0};
};

}  // namespace robot_controller

#endif  // GAME1_SHOOTER__GAME1_AUTO_NODE_HPP_
