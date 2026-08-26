#ifndef GAME2_AIM__GAME2_AIM_NODE_HPP_
#define GAME2_AIM__GAME2_AIM_NODE_HPP_

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
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/game2_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
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
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double pixel_x{0.0};
  double pixel_y{0.0};
  double yaw_at_detection{0.0};
  rclcpp::Time last_seen;
};

class Game2AimNode : public rclcpp::Node
{
public:
  explicit Game2AimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void load_parameters();
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void tag_detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void update_panel_states();
  void select_target_and_aim();
  void control_loop();
  void reset_sequence();

  uint8_t get_target_belt_mode(int row) const;

  void publish_all(
    const geometry_msgs::msg::Twist & cmd_vel,
    uint8_t belt_mode,
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_pub_;
  rclcpp::Publisher<robot_msgs::msg::ArmPosition>::SharedPtr arm_position_pub_;
  rclcpp::Publisher<robot_msgs::msg::Game2State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Parameters
  std::string base_frame_{"base_link"};
  std::string cmd_vel_topic_{"/mecanum/cmd_vel_heading"};
  std::string detections_topic_{"/detections"};
  std::string tag_prefix_{"tag16h5:"};
  double kp_yaw_{1.8};
  double kd_yaw_{0.12};
  double min_angular_z_{0.12};
  double max_angular_z_{0.40};
  double max_angular_accel_{2.5}; // [rad/s^2]
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
  double yaw_tolerance_{0.030}; // rad (~1.7 deg)
  double dist_tolerance_{0.05};

  double tag_lost_timeout_{1.5};
  bool test_alignment_only_{false};
  bool enable_double_panel_midpoint_targeting_{true}; // 2枚連続時に中点を狙い1発2枚抜き

  // State Variables
  uint8_t state_{robot_msgs::msg::Game2State::STANDBY};
  bool is_enabled_{false};
  bool emergency_stop_active_{false};
  std::unordered_map<int, PanelTagInfo> panel_grid_;
  int active_row_{2}; // 0: Bottom, 1: Middle, 2: Top (優先順: 2 -> 1 -> 0)
  int active_target_id_{-1};
  std::vector<int> current_target_tag_ids_; // 狙っているタグIDリスト（中点狙い時は2個）
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_heading_err_{0.0};
  uint8_t target_belt_mode_{robot_msgs::msg::BeltMode::STOP};
  bool target_valid_{false};
  int locked_target_id_{-1};
  rclcpp::Time state_start_time_;
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

#endif  // GAME2_AIM__GAME2_AIM_NODE_HPP_
