#ifndef GAME2_SHOOTER__GAME2_AUTO_NODE_HPP_
#define GAME2_SHOOTER__GAME2_AUTO_NODE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/game2_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_controller
{

struct PanelTagInfo
{
  int tag_id{0};
  int row{0};     // 0: Bottom, 1: Middle, 2: Top
  int col{0};     // 0: Left, 1: Center, 2: Right
  bool detected{false};
  bool shot_completed{false};
  int shot_count{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double pixel_x{0.0};
  double pixel_y{0.0};
  rclcpp::Time last_seen;
};

class Game2AutoNode : public rclcpp::Node
{
public:
  explicit Game2AutoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void load_parameters();
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void tag_detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ball_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void update_panel_states();
  void select_target_and_aim();
  void control_loop();
  void reset_sequence();

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
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr belt_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_pub_;
  rclcpp::Publisher<robot_msgs::msg::Game2State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Parameters
  std::string base_frame_{"base_link"};
  std::string cmd_vel_topic_{"/mecanum/cmd_vel_heading"};
  std::string tag_prefix_{"tag16h5:"};
  double kp_yaw_{2.2};
  double kd_yaw_{0.10};
  double min_angular_z_{0.12};
  double max_angular_z_{0.80};
  double max_angular_accel_{4.0}; // [rad/s^2]
  double target_distance_{4.0};

  // Camera Optical / Physical Parameters
  double camera_offset_x_{0.265};
  double camera_offset_y_{0.035};
  double camera_offset_z_{0.193};
  double camera_image_width_{1920.0};
  double camera_image_height_{1080.0};
  double camera_fx_{800.0};
  double camera_fy_{800.0};
  double camera_cx_{960.0};
  double camera_cy_{540.0};

  // Tolerances & Timings
  double yaw_tolerance_{0.015}; // rad (~0.85 deg)
  double dist_tolerance_{0.05};
  double rpm_bottom_{3000.0};
  double rpm_middle_{4500.0};
  double rpm_top_{6000.0};
  double open_duration_{0.3};
  double shoot_hold_duration_{0.8};
  double ball_settle_duration_{0.3};
  double tag_lost_timeout_{0.5};
  double aligning_timeout_{10.0};
  double shooting_timeout_{3.0};
  int max_shots_per_panel_{1};
  bool require_ball_detected_{true};
  bool test_alignment_only_{false};
  bool auto_advance_rows_{true};

  // State Variables
  uint8_t state_{robot_msgs::msg::Game2State::STANDBY};
  bool is_enabled_{false};
  bool emergency_stop_active_{false};
  bool ball_detected_{false};
  rclcpp::Time ball_detected_time_;
  std::unordered_map<int, PanelTagInfo> panel_grid_;
  int active_row_{0}; // 0: Bottom, 1: Middle, 2: Top
  int active_target_id_{-1};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_heading_err_{0.0};
  double target_rpm_{0.0};
  bool target_valid_{false};
  int locked_target_id_{-1};
  rclcpp::Time state_start_time_;
  rclcpp::Time shoot_start_time_;
  rclcpp::Time last_loop_time_;
  double last_cmd_wz_{0.0};

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

#endif  // GAME2_SHOOTER__GAME2_AUTO_NODE_HPP_
