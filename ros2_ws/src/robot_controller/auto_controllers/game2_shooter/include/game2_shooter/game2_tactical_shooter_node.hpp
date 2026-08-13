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
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/game2_state.hpp"
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
  void publish_all(
    const geometry_msgs::msg::Twist & cmd_vel,
    float belt_rpm,
    bool shoot_trigger,
    bool dribble_enabled,
    uint8_t arm_mode,
    bool completed);

  // TF Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions & Publishers & Timers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr belt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_pub_;
  rclcpp::Publisher<robot_msgs::msg::Game2State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string base_frame_;
  std::string tag_prefix_;
  double kp_yaw_;
  double kd_yaw_;
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
  uint8_t state_{robot_msgs::msg::Game2State::STANDBY};
  bool is_enabled_{false};
  std::unordered_map<int, PanelTagInfo> panel_grid_;
  int active_row_{0}; // 0: Bottom, 1: Middle, 2: Top
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_rpm_{0.0};
  bool target_valid_{false};
  rclcpp::Time shoot_start_time_;

  // IMU Feedback State (Full Telemetry)
  bool imu_received_{false};
  double roll_{0.0};
  double pitch_{0.0};
  double raw_yaw_{0.0};
  double yaw_offset_{0.0};
  double yaw_{0.0};
  double gyro_x_{0.0};
  double gyro_y_{0.0};
  double gyro_z_{0.0};
  double accel_x_{0.0};
  double accel_y_{0.0};
  double accel_z_{0.0};
  rclcpp::Time last_imu_time_;
};

}  // namespace robot_controller

#endif  // GAME2_SHOOTER__GAME2_TACTICAL_SHOOTER_NODE_HPP_
