#ifndef GAME2_AIM__GAME2_AIM_NODE_HPP_
#define GAME2_AIM__GAME2_AIM_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "game2_aim/target_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/game2_state.hpp"
#include "robot_msgs/msg/shot_cycle_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_controller
{

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
  void shot_cycle_state_callback(const robot_msgs::msg::ShotCycleState::SharedPtr msg);
  void shot_cycle_req_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void transition_to(uint8_t new_state, const std::string & reason);
  void control_loop();

  void publish_all(
    const geometry_msgs::msg::Twist & cmd_vel,
    uint8_t belt_mode,
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
  rclcpp::Subscription<robot_msgs::msg::ShotCycleState>::SharedPtr shot_cycle_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_req_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<robot_msgs::msg::Game2State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Parameters
  std::string base_frame_{"base_link"};
  std::string cmd_vel_topic_{"/drive/cmd_vel"};
  std::string detections_topic_{"/detections"};
  std::string camera_info_topic_{"/camera/camera_info"};
  std::string tag_prefix_{"tag16h5:"};
  double kp_yaw_{1.8};
  double yaw_command_sign_{-1.0};
  double kd_yaw_{0.18};
  double min_angular_z_{0.08};
  double max_angular_z_{0.40};
  double max_angular_accel_{2.5}; // [rad/s^2]
  double target_distance_{4.0};

  // Tolerances & Timings
  double yaw_tolerance_{0.030}; // rad (~1.7 deg)
  double dist_tolerance_{0.05};

  double search_angular_z_{0.15};
  bool test_alignment_only_{false};
  double shot_fallback_timeout_{5.0}; // [s] 射出ボタン押下後の保険タイムアウト

  // ── 新規パラメータ: 信頼度判定・タイムアウト ──
  int min_detection_frames_{2};
  double visual_valid_timeout_{0.3}; // [s] 照準完了判定に必要な直近視覚有効時間
  double align_lost_timeout_{1.0};   // [s] ALIGNING中の完全ロスト判定時間

  // ── ターゲット追従・認識エンジン ──
  TargetTracker tracker_;

  // ── State Machine State (唯一の状態変数) ──
  uint8_t state_{robot_msgs::msg::Game2State::STANDBY};
  bool emergency_stop_active_{false};

  // ── Shot Detection & Insurance Timer State ──
  uint8_t prev_shot_cycle_state_{robot_msgs::msg::ShotCycleState::IDLE};
  bool shot_requested_{false};
  rclcpp::Time shot_requested_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Time last_loop_time_{0, 0, RCL_ROS_TIME};
  double last_cmd_wz_{0.0};

  // IMU Feedback State
  bool imu_received_{false};
  double raw_yaw_{0.0};
  double yaw_offset_{0.0};
  double yaw_{0.0};
  double gyro_z_{0.0};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace robot_controller

#endif  // GAME2_AIM__GAME2_AIM_NODE_HPP_
