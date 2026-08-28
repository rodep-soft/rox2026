#include "game2_aim/game2_aim_node.hpp"

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace robot_controller
{

Game2AimNode::Game2AimNode(const rclcpp::NodeOptions & options)
: Node("game2_aim", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  load_parameters();

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&Game2AimNode::parameter_callback, this, std::placeholders::_1));

  const auto cmd_qos = rclcpp::QoS(10);

  // AprilTag Detections Subscription
  detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    detections_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Game2AimNode::tag_detections_callback, this, std::placeholders::_1));

  // /camera_info Subscription
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, cmd_qos,
    std::bind(&Game2AimNode::camera_info_callback, this, std::placeholders::_1));

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game2/command_start", cmd_qos,
    std::bind(&Game2AimNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2AimNode::imu_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Game2AimNode::emergency_stop_callback, this, std::placeholders::_1));

  shot_cycle_state_sub_ = create_subscription<robot_msgs::msg::ShotCycleState>(
    "/dribble/shot_cycle_state", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Game2AimNode::shot_cycle_state_callback, this, std::placeholders::_1));

  shot_cycle_req_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", cmd_qos,
    std::bind(&Game2AimNode::shot_cycle_req_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, cmd_qos);
  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>("/belt/command_mode", cmd_qos);
  state_pub_ =
    create_publisher<robot_msgs::msg::Game2State>("/game2/state", rclcpp::QoS(1).reliable().transient_local());
  completed_pub_ =
    create_publisher<std_msgs::msg::Bool>("/game2/completed", rclcpp::QoS(1).reliable().transient_local());

  // Control Loop Timer (20Hz = 50ms)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2AimNode::control_loop, this));

  last_loop_time_ = now();

  RCLCPP_INFO(
    get_logger(),
    "🚀 Game2AimNode initialized with TargetTracker. Target Distance: %.2fm | State: STANDBY",
    target_distance_);
}

void Game2AimNode::load_parameters()
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/drive/cmd_vel");
  detections_topic_ = declare_parameter<std::string>("detections_topic", "/detections");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");
  camera_info_topic_ =
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");

  // Control gains & Limits
  kp_yaw_ = declare_parameter<double>("kp_yaw", 1.8);
  yaw_command_sign_ = declare_parameter<double>("yaw_command_sign", -1.0);
  kd_yaw_ = declare_parameter<double>("kd_yaw", 0.18);
  min_angular_z_ = declare_parameter<double>("min_angular_z", 0.08);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.40);
  max_angular_accel_ = declare_parameter<double>("max_angular_accel", 2.5);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.030);
  dist_tolerance_ = declare_parameter<double>("dist_tolerance", 0.05);

  target_distance_ = declare_parameter<double>("target_distance", 4.0);
  search_angular_z_ = declare_parameter<double>("search_angular_z", 0.15);
  test_alignment_only_ = declare_parameter<bool>("test_alignment_only", false);
  shot_fallback_timeout_ = declare_parameter<double>("shot_fallback_timeout", 5.0);

  min_detection_frames_ = declare_parameter<int>("min_detection_frames", 2);
  visual_valid_timeout_ = declare_parameter<double>("visual_valid_timeout", 0.3);
  align_lost_timeout_ = declare_parameter<double>("align_lost_timeout", 1.0);
  aim_yaw_offset_deg_ = declare_parameter<double>("aim_yaw_offset_deg", 0.0);
  min_standing_aspect_ratio_ = declare_parameter<double>("min_standing_aspect_ratio", 0.80);
  max_standing_tilt_deg_ = declare_parameter<double>("max_standing_tilt_deg", 20.0);
  max_standing_height_drop_ = declare_parameter<double>("max_standing_height_drop", 0.07);
  post_shot_delay_sec_ = declare_parameter<double>("post_shot_delay_sec", 0.8);
  shot_target_cooldown_sec_ = declare_parameter<double>("shot_target_cooldown_sec", 2.0);
  midpoint_blend_ratio_ = declare_parameter<double>("midpoint_blend_ratio", 0.65);

  // Tracker Config Setup
  TargetTracker::Config tracker_cfg;
  tracker_cfg.tag_prefix = tag_prefix_;
  tracker_cfg.base_frame = base_frame_;
  tracker_cfg.target_distance = target_distance_;
  tracker_cfg.camera_offset_x = declare_parameter<double>("camera_offset_x", 0.265);
  tracker_cfg.camera_offset_y = declare_parameter<double>("camera_offset_y", 0.035);
  tracker_cfg.camera_offset_z = declare_parameter<double>("camera_offset_z", 0.193);
  tracker_cfg.camera_image_width = declare_parameter<double>("camera_image_width", 1920.0);
  tracker_cfg.camera_image_height = declare_parameter<double>("camera_image_height", 1080.0);
  tracker_cfg.camera_fx = declare_parameter<double>("camera_fx", 800.0);
  tracker_cfg.camera_fy = declare_parameter<double>("camera_fy", 800.0);
  tracker_cfg.camera_cx = declare_parameter<double>("camera_cx", 960.0);
  tracker_cfg.camera_cy = declare_parameter<double>("camera_cy", 540.0);
  tracker_cfg.tag_lost_timeout = declare_parameter<double>("tag_lost_timeout", 1.5);
  tracker_cfg.enable_double_panel_midpoint_targeting =
    declare_parameter<bool>("enable_double_panel_midpoint_targeting", true);
  tracker_cfg.test_alignment_only = test_alignment_only_;
  tracker_cfg.min_detection_frames = min_detection_frames_;
  tracker_cfg.aim_yaw_offset_rad = aim_yaw_offset_deg_ * M_PI / 180.0;
  tracker_cfg.min_standing_aspect_ratio = min_standing_aspect_ratio_;
  tracker_cfg.max_standing_tilt_deg = max_standing_tilt_deg_;
  tracker_cfg.max_standing_height_drop = max_standing_height_drop_;
  tracker_cfg.shot_target_cooldown_sec = shot_target_cooldown_sec_;
  tracker_cfg.midpoint_blend_ratio = midpoint_blend_ratio_;
  tracker_.set_config(tracker_cfg);

  // AprilTag Panel IDs
  const auto bottom_tags = declare_parameter<std::vector<int64_t>>("bottom_tags", {20, 21, 22});
  const auto middle_tags = declare_parameter<std::vector<int64_t>>("middle_tags", {17, 18, 19});
  const auto top_tags = declare_parameter<std::vector<int64_t>>("top_tags", {14, 15, 16});

  tracker_.init_panel_grid(bottom_tags, middle_tags, top_tags);
}

rcl_interfaces::msg::SetParametersResult Game2AimNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto tracker_cfg = tracker_.config();
  bool tracker_cfg_changed = false;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "kp_yaw") {
      kp_yaw_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Param updated: kp_yaw = %.3f", kp_yaw_);
    } else if (name == "yaw_command_sign") {
      yaw_command_sign_ = param.as_double();
    } else if (name == "kd_yaw") {
      kd_yaw_ = param.as_double();
      RCLCPP_INFO(get_logger(), "Param updated: kd_yaw = %.3f", kd_yaw_);
    } else if (name == "min_angular_z") {
      min_angular_z_ = param.as_double();
    } else if (name == "max_angular_z") {
      max_angular_z_ = param.as_double();
    } else if (name == "max_angular_accel") {
      max_angular_accel_ = param.as_double();
    } else if (name == "yaw_tolerance") {
      yaw_tolerance_ = param.as_double();
    } else if (name == "dist_tolerance") {
      dist_tolerance_ = param.as_double();
    } else if (name == "search_angular_z") {
      search_angular_z_ = param.as_double();
    } else if (name == "test_alignment_only") {
      test_alignment_only_ = param.as_bool();
      tracker_cfg.test_alignment_only = test_alignment_only_;
      tracker_cfg_changed = true;
    } else if (name == "shot_fallback_timeout") {
      shot_fallback_timeout_ = param.as_double();
    } else if (name == "min_detection_frames") {
      min_detection_frames_ = param.as_int();
      tracker_cfg.min_detection_frames = min_detection_frames_;
      tracker_cfg_changed = true;
    } else if (name == "visual_valid_timeout") {
      visual_valid_timeout_ = param.as_double();
    } else if (name == "align_lost_timeout") {
      align_lost_timeout_ = param.as_double();
    } else if (name == "aim_yaw_offset_deg") {
      aim_yaw_offset_deg_ = param.as_double();
      tracker_cfg.aim_yaw_offset_rad = aim_yaw_offset_deg_ * M_PI / 180.0;
      tracker_cfg_changed = true;
      RCLCPP_INFO(get_logger(), "Param updated: aim_yaw_offset_deg = %+.2f deg", aim_yaw_offset_deg_);
    } else if (name == "enable_double_panel_midpoint_targeting") {
      tracker_cfg.enable_double_panel_midpoint_targeting = param.as_bool();
      tracker_cfg_changed = true;
    } else if (name == "min_standing_aspect_ratio") {
      min_standing_aspect_ratio_ = param.as_double();
      tracker_cfg.min_standing_aspect_ratio = min_standing_aspect_ratio_;
      tracker_cfg_changed = true;
    } else if (name == "max_standing_tilt_deg") {
      max_standing_tilt_deg_ = param.as_double();
      tracker_cfg.max_standing_tilt_deg = max_standing_tilt_deg_;
      tracker_cfg_changed = true;
    } else if (name == "max_standing_height_drop") {
      max_standing_height_drop_ = param.as_double();
      tracker_cfg.max_standing_height_drop = max_standing_height_drop_;
      tracker_cfg_changed = true;
    } else if (name == "post_shot_delay_sec") {
      post_shot_delay_sec_ = param.as_double();
    } else if (name == "shot_target_cooldown_sec") {
      shot_target_cooldown_sec_ = param.as_double();
      tracker_cfg.shot_target_cooldown_sec = shot_target_cooldown_sec_;
      tracker_cfg_changed = true;
    } else if (name == "midpoint_blend_ratio") {
      midpoint_blend_ratio_ = param.as_double();
      tracker_cfg.midpoint_blend_ratio = midpoint_blend_ratio_;
      tracker_cfg_changed = true;
    }
  }

  if (tracker_cfg_changed) {
    tracker_.set_config(tracker_cfg);
  }

  return result;
}

void Game2AimNode::tag_detections_callback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  tracker_.update_detections(*msg, *tf_buffer_, yaw_, now());
}

void Game2AimNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  tracker_.update_camera_info(*msg);
}

void Game2AimNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    const auto start_time = this->now();
    last_imu_time_ = start_time;
    last_loop_time_ = start_time;
    tracker_.reset_fallen_states();
    transition_to(robot_msgs::msg::Game2State::SEARCHING, "Start command received (true)");
  } else {
    if (state_ != robot_msgs::msg::Game2State::STANDBY) {
      transition_to(robot_msgs::msg::Game2State::STANDBY, "Start command received (false)");
      publish_all(geometry_msgs::msg::Twist{}, robot_msgs::msg::BeltMode::STOP, false);
    }
  }
}

void Game2AimNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_ && state_ != robot_msgs::msg::Game2State::STANDBY) {
    RCLCPP_WARN(get_logger(), "Emergency Stop Triggered! Game 2 Auto Sequence ABORTED.");
    transition_to(robot_msgs::msg::Game2State::STANDBY, "Emergency stop active");
    publish_all(geometry_msgs::msg::Twist{}, robot_msgs::msg::BeltMode::STOP, false);
  }
}

void Game2AimNode::shot_cycle_state_callback(
  const robot_msgs::msg::ShotCycleState::SharedPtr msg)
{
  const uint8_t current_state = msg->state;
  if (state_ == robot_msgs::msg::Game2State::PREPARING_SHOOT ||
      state_ == robot_msgs::msg::Game2State::ALIGNING)
  {
    // ボール射出完了（FEEDING -> RETURNING）または サイクル完全終了（-> IDLE）で次の探索へ移行
    if ((current_state == robot_msgs::msg::ShotCycleState::RETURNING &&
         prev_shot_cycle_state_ == robot_msgs::msg::ShotCycleState::FEEDING) ||
        (current_state == robot_msgs::msg::ShotCycleState::IDLE &&
         prev_shot_cycle_state_ == robot_msgs::msg::ShotCycleState::RETURNING))
    {
      shot_requested_ = false;
      last_shot_completed_time_ = now();
      tracker_.mark_active_target_shot(last_shot_completed_time_);
      transition_to(
        robot_msgs::msg::Game2State::SEARCHING,
        "Shot completed (ejected). Transitioning to SEARCHING for next target");
    }
  }
  prev_shot_cycle_state_ = current_state;
}

void Game2AimNode::shot_cycle_req_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && state_ != robot_msgs::msg::Game2State::STANDBY) {
    shot_requested_ = true;
    shot_requested_time_ = now();
    RCLCPP_INFO(
      get_logger(),
      "🎯 [Game2] Shot cycle requested (L2+Circle). Insurance timer started (%.1fs timeout)",
      shot_fallback_timeout_);
  }
}

void Game2AimNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  last_imu_time_ = now();

  gyro_z_ = msg->angular_velocity.z;

  tf2::Quaternion q(
    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double r, p, raw_y;
  m.getRPY(r, p, raw_y);
  raw_yaw_ = raw_y;

  yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
}

void Game2AimNode::transition_to(uint8_t new_state, const std::string & reason)
{
  if (state_ == new_state) {
    return;
  }

  const char * state_names[] = {
    "STANDBY", "SEARCHING", "ALIGNING", "PREPARING_SHOOT",
    "SHOOTING", "WAITING_RESULT", "COMPLETED"
  };

  const char * old_name = (state_ <= 6) ? state_names[state_] : "UNKNOWN";
  const char * new_name = (new_state <= 6) ? state_names[new_state] : "UNKNOWN";

  RCLCPP_INFO(
    get_logger(),
    "🔄 [Game2 State Transition] %s -> %s (Reason: %s)",
    old_name, new_name, reason.c_str());

  if (new_state == robot_msgs::msg::Game2State::STANDBY ||
      new_state == robot_msgs::msg::Game2State::SEARCHING)
  {
    tracker_.clear_target();
    shot_requested_ = false;
  }

  state_ = new_state;

  robot_msgs::msg::Game2State state_msg;
  state_msg.state = state_;
  state_pub_->publish(state_msg);
}

void Game2AimNode::control_loop()
{
  const auto current_time = now();
  double dt = (current_time - last_loop_time_).seconds();
  if (dt <= 0.001 || dt > 0.2) {
    dt = 0.05;
  }
  last_loop_time_ = current_time;

  // 非常停止チェック
  if (emergency_stop_active_ && state_ != robot_msgs::msg::Game2State::STANDBY) {
    transition_to(robot_msgs::msg::Game2State::STANDBY, "Emergency stop active");
  }

  // STANDBY時は手動操作にトピックを譲るため何も出力せずreturn
  if (state_ == robot_msgs::msg::Game2State::STANDBY) {
    last_cmd_wz_ = 0.0;
    robot_msgs::msg::Game2State state_msg;
    state_msg.state = state_;
    state_pub_->publish(state_msg);
    return;
  }

  // 保険タイマー: 射出ボタン押下から一定時間経過しても完了ステートが来ない場合のフォールバック
  if (shot_requested_ && (current_time - shot_requested_time_).seconds() >= shot_fallback_timeout_) {
    transition_to(
      robot_msgs::msg::Game2State::SEARCHING,
      "Shot fallback timeout elapsed (insurance timer fired)");
  }

  geometry_msgs::msg::Twist cmd;
  uint8_t current_belt_mode = robot_msgs::msg::BeltMode::STOP;

  switch (state_) {
    case robot_msgs::msg::Game2State::SEARCHING: {
      current_belt_mode = robot_msgs::msg::BeltMode::STOP;
      cmd.angular.z = search_angular_z_;
      last_cmd_wz_ = search_angular_z_;

      // 射出直後の着弾・打倒待ちディレイ（0.8秒間は探索を待機し、的が倒れるのを待つ）
      if (last_shot_completed_time_.nanoseconds() > 0 &&
          (current_time - last_shot_completed_time_).seconds() < post_shot_delay_sec_)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "⏳ [Game2 SEARCHING] Waiting for target to fall (%.2fs / %.2fs)... Standing still",
          (current_time - last_shot_completed_time_).seconds(), post_shot_delay_sec_);
        break;
      }

      if (tracker_.find_and_lock_target(current_time, get_logger())) {
        transition_to(robot_msgs::msg::Game2State::ALIGNING, "Target panel confirmed by vision");
      } else {
        const std::string summary = tracker_.get_detection_summary(current_time);
        if (std::abs(search_angular_z_) > 0.001) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "🔍 [Game2 SEARCHING] Rotating %+.2f rad/s | %s",
            search_angular_z_, summary.c_str());
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "🔍 [Game2 SEARCHING] Waiting for target AprilTags... | %s",
            summary.c_str());
        }
      }
      break;
    }

    case robot_msgs::msg::Game2State::ALIGNING: {
      current_belt_mode = robot_msgs::msg::BeltMode::STOP;
      tracker_.update_tracking(yaw_, current_time);

      // 1. 完全見失いタイムアウト判定（1.0秒以上ロストしたら再探索）
      if (tracker_.is_lost_timeout(current_time, align_lost_timeout_)) {
        transition_to(
          robot_msgs::msg::Game2State::SEARCHING,
          "Target lost timeout in ALIGNING (no vision > 1.0s)");
        break;
      }

      const double heading_err = tracker_.heading_error();
      const bool is_aligned = (std::abs(heading_err) < yaw_tolerance_);
      const bool is_visible = tracker_.is_currently_visible(current_time, visual_valid_timeout_);

      // 2. 「今カメラで見えており」かつ「角度誤差が許容値内」なら発射準備へ
      if (is_visible && is_aligned) {
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;
        transition_to(
          robot_msgs::msg::Game2State::PREPARING_SHOOT,
          "Target aligned with visual confirmation");
        current_belt_mode = test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : tracker_.target_belt_mode();
      } else {
        // PD Control (IMU Gyro damping)
        double desired_wz = yaw_command_sign_ * kp_yaw_ * heading_err;
        if (imu_received_ && (current_time - last_imu_time_).seconds() < 0.5) {
          desired_wz -= kd_yaw_ * gyro_z_;
        }

        if (std::abs(desired_wz) < min_angular_z_) {
          desired_wz = std::copysign(min_angular_z_, desired_wz);
        }
        desired_wz = std::clamp(desired_wz, -max_angular_z_, max_angular_z_);

        const double max_wz_step = max_angular_accel_ * dt;
        if (desired_wz > last_cmd_wz_ + max_wz_step) {
          desired_wz = last_cmd_wz_ + max_wz_step;
        } else if (desired_wz < last_cmd_wz_ - max_wz_step) {
          desired_wz = last_cmd_wz_ - max_wz_step;
        }

        cmd.angular.z = desired_wz;
        last_cmd_wz_ = cmd.angular.z;

        const double target_yaw = std::remainder(yaw_ + heading_err, 2.0 * M_PI);

        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "🎯 [Game2 ALIGNING | %s] Target: %+.2f deg | Current: %+.2f deg | Err: %+.2f deg | Cmd wz: %+.3f rad/s | %s",
          tracker_.target_description().c_str(),
          target_yaw * 180.0 / M_PI, yaw_ * 180.0 / M_PI, heading_err * 180.0 / M_PI,
          cmd.angular.z, is_visible ? "👁️ VISIBLE" : "📡 IMU DEAD-RECKONING");
      }
      break;
    }

    case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
      cmd.angular.z = 0.0;
      last_cmd_wz_ = 0.0;
      current_belt_mode = test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : tracker_.target_belt_mode();
      tracker_.update_tracking(yaw_, current_time);

      // 外力等で誤差が許容値を超えてズレた場合は ALIGNING へ戻って再照準
      if (std::abs(tracker_.heading_error()) > yaw_tolerance_ * 2.0) {
        transition_to(
          robot_msgs::msg::Game2State::ALIGNING,
          "Heading error exceeded tolerance in PREPARING_SHOOT");
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "🚀 [Game2 PREPARING_SHOOT | %s] Aligned! Spinning Belt (Mode: %u) | Ready for Shot",
          tracker_.target_description().c_str(), current_belt_mode);
      }
      break;
    }

    default:
      break;
  }

  publish_all(cmd, current_belt_mode, false);
}

void Game2AimNode::publish_all(
  const geometry_msgs::msg::Twist & cmd_vel,
  uint8_t belt_mode,
  bool completed)
{
  cmd_vel_pub_->publish(cmd_vel);

  robot_msgs::msg::BeltMode mode_msg;
  mode_msg.mode = belt_mode;
  belt_mode_pub_->publish(mode_msg);

  std_msgs::msg::Bool completed_msg;
  completed_msg.data = completed;
  completed_pub_->publish(completed_msg);
}

}  // namespace robot_controller
