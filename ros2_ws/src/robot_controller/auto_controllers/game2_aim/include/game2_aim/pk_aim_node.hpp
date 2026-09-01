#ifndef GAME2_AIM__PK_AIM_NODE_HPP_
#define GAME2_AIM__PK_AIM_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "game2_aim/pk_target_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/game2_state.hpp"
#include "robot_msgs/msg/shot_cycle_state.hpp"
#include "robot_msgs/msg/target_grid_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_controller
{

class PKAimNode : public rclcpp::Node
{
public:
  explicit PKAimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void load_parameters();
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void tag_detections_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void shot_cycle_state_callback(const robot_msgs::msg::ShotCycleState::SharedPtr msg);
  void shot_cycle_req_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // PK 特有コールバック
  void pk_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void pk_confirm_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void pk_next_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void pk_prev_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void pk_set_target_index_callback(const std_msgs::msg::Int32::SharedPtr msg);

  void transition_to(uint8_t new_state, const std::string & reason);
  void control_loop();

  void publish_all(
    const geometry_msgs::msg::Twist & cmd_vel,
    uint8_t belt_mode,
    bool completed);
  void publish_target_status(const rclcpp::Time & now);
  void log_target_decision(const std::string & title, const std::string & reason);

  // TF Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions & Publishers & Timers
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<robot_msgs::msg::ShotCycleState>::SharedPtr shot_cycle_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shot_cycle_req_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pk_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pk_confirm_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pk_next_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pk_prev_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pk_set_target_index_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<robot_msgs::msg::BeltMode>::SharedPtr belt_mode_pub_;
  rclcpp::Publisher<robot_msgs::msg::Game2State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completed_pub_;
  rclcpp::Publisher<robot_msgs::msg::TargetGridState>::SharedPtr target_grid_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_index_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr target_indices_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fallen_indices_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_tag_id_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Parameters
  std::string base_frame_{"base_link"};
  std::string cmd_vel_topic_{"/drive/cmd_vel"};
  std::string detections_topic_{"/detections"};
  std::string camera_info_topic_{"/camera/camera_info"};
  std::string tag_prefix_{"tag16h5:"};
  double kp_yaw_{1.1};
  double yaw_command_sign_{-1.0};
  double kd_yaw_{0.35};
  double min_angular_z_{0.015};
  double max_angular_z_{0.25};
  double max_angular_accel_{1.5}; // [rad/s^2]
  double target_distance_{4.0};

  // Tolerances & Timings
  double yaw_tolerance_{0.008}; // rad (~0.46 deg)
  double dist_tolerance_{0.05};
  bool test_alignment_only_{false};
  bool test_panel_state_display_{true};
  double shot_fallback_timeout_{5.0};
  double visual_valid_timeout_{0.5};
  double align_lost_timeout_{2.5};
  double aim_yaw_offset_deg_{2.0};
  double outer_col_offset_m_{0.10}; // [m] 外側列(Col 0/2)の単体狙い外側シフト量

  // Internal State
  uint8_t state_{robot_msgs::msg::Game2State::STANDBY};
  bool emergency_stop_{false};
  bool imu_received_{false};
  double yaw_{0.0};
  double gyro_z_{0.0};
  double last_cmd_wz_{0.0};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_entry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_shot_req_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_loop_time_{0, 0, RCL_ROS_TIME};
  uint8_t prev_shot_cycle_state_{robot_msgs::msg::ShotCycleState::IDLE};
  bool is_target_confirmed_{false};
  bool shot_requested_{false};

  PKTargetTracker tracker_;
};

}  // namespace robot_controller

#endif  // GAME2_AIM__PK_AIM_NODE_HPP_
