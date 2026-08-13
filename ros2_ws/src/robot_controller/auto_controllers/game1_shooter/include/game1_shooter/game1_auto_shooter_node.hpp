#ifndef GAME1_SHOOTER__GAME1_AUTO_SHOOTER_NODE_HPP_
#define GAME1_SHOOTER__GAME1_AUTO_SHOOTER_NODE_HPP_

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
  NAV_TO_BALL_DRIBBLE_ON = 3,
  NAV_TO_PASS_AREA = 4,
  FIRE_PASS_SPRING = 5,
  NAV_TO_START = 6,
  COMPLETED = 7,
};

class Game1AutoShooterNode : public rclcpp::Node
{
public:
  explicit Game1AutoShooterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void control_loop();
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void publish_commands(
    const geometry_msgs::msg::Twist & cmd_vel,
    bool dribble_enabled,
    uint8_t arm_position,
    bool spring_fire);

  // Pure Pursuit 車体制御計算
  geometry_msgs::msg::Twist compute_pure_pursuit(const Waypoint & target);

  // Subscriptions & Publishers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double kp_linear_{1.0};
  double kp_angular_{1.5};
  double max_linear_vel_{1.5};
  double max_angular_vel_{1.0};
  double pos_tolerance_{0.08};
  double yaw_tolerance_{0.05};

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

  // IMU Feedback
  bool imu_received_{false};
  double raw_yaw_{0.0};
  double yaw_offset_{0.0};
  double current_yaw_{0.0};
};

}  // namespace robot_controller

#endif  // GAME1_SHOOTER__GAME1_AUTO_SHOOTER_NODE_HPP_
