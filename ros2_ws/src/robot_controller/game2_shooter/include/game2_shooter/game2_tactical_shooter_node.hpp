#ifndef GAME2_SHOOTER__GAME2_TACTICAL_SHOOTER_NODE_HPP_
#define GAME2_SHOOTER__GAME2_TACTICAL_SHOOTER_NODE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_controller
{

struct PanelTagInfo
{
  int tag_id;
  int row;     // 0: Bottom, 1: Middle, 2: Top
  int col;     // 0: Left, 1: Center, 2: Right
  bool detected{false};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  rclcpp::Time last_seen;
};

enum class State
{
  STANDBY,
  SEARCHING,
  ALIGNING,
  PREPARING_SHOOT,
  SHOOTING,
  WAITING_RESULT,
  COMPLETED
};

class Game2TacticalShooterNode : public rclcpp::Node
{
public:
  explicit Game2TacticalShooterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void update_panel_states();
  void select_target_and_aim();
  void control_loop();
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // TF Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions & Publishers & Timers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr belt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string base_frame_;
  std::string tag_prefix_;
  double kp_yaw_;
  double kd_yaw_;
  double kp_y_;
  double kp_dist_;
  double max_angular_z_;
  double target_distance_;
  double yaw_tolerance_;
  double dist_tolerance_;
  double rpm_bottom_;
  double rpm_middle_;
  double rpm_top_;
  double shoot_hold_duration_;

  // State Variables
  State state_{State::STANDBY};
  bool is_enabled_{false};
  std::unordered_map<int, PanelTagInfo> panel_grid_;
  int active_row_{0}; // 0: Bottom, 1: Middle, 2: Top
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_rpm_{0.0};
  bool target_valid_{false};
  rclcpp::Time shoot_start_time_;

  // IMU Feedback State
  bool imu_received_{false};
  double current_gyro_z_{0.0}; // rad/s
  rclcpp::Time last_imu_time_;
};

}  // namespace robot_controller

#endif  // GAME2_SHOOTER__GAME2_TACTICAL_SHOOTER_NODE_HPP_
