#include "game2_aim/pk_aim_node.hpp"

#include <chrono>
#include <cmath>

namespace robot_controller
{

PKAimNode::PKAimNode(const rclcpp::NodeOptions & options)
: Node("pk_aim_node", options)
{
  load_parameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    detections_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PKAimNode::tag_detections_callback, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PKAimNode::camera_info_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&PKAimNode::imu_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/robot/emergency_stop", 10,
    std::bind(&PKAimNode::emergency_stop_callback, this, std::placeholders::_1));

  shot_cycle_state_sub_ = create_subscription<robot_msgs::msg::ShotCycleState>(
    "/shot_cycle/state", 10,
    std::bind(&PKAimNode::shot_cycle_state_callback, this, std::placeholders::_1));

  shot_cycle_req_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/shot_cycle/request", 10,
    std::bind(&PKAimNode::shot_cycle_req_callback, this, std::placeholders::_1));

  // PK 特有サブスクリプション
  pk_start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/pk/start", 10,
    std::bind(&PKAimNode::pk_start_callback, this, std::placeholders::_1));

  pk_next_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/pk/next", 10,
    std::bind(&PKAimNode::pk_next_callback, this, std::placeholders::_1));

  pk_prev_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/pk/prev", 10,
    std::bind(&PKAimNode::pk_prev_callback, this, std::placeholders::_1));

  pk_set_target_index_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/pk/set_target_index", 10,
    std::bind(&PKAimNode::pk_set_target_index_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>("/belt/mode", 10);
  state_pub_ = create_publisher<robot_msgs::msg::Game2State>("/pk/state", 10);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/pk/completed", 10);
  target_index_pub_ = create_publisher<std_msgs::msg::Int32>("/target_index", 10);
  target_indices_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/target_indices", 10);
  fallen_indices_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/fallen_indices", 10);
  target_tag_id_pub_ = create_publisher<std_msgs::msg::Int32>("/pk/target_tag_id", 10);

  // Timer (50 Hz Control Loop)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&PKAimNode::control_loop, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PKAimNode::parameter_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "🚀 PKAimNode initialized. Initial Target: [Idx 0] %s (Tag #%d | Row %d Col %d)",
    tracker_.get_selected_panel().name.c_str(),
    tracker_.get_selected_panel().tag_id,
    tracker_.get_selected_panel().row,
    tracker_.get_selected_panel().col);
}

void PKAimNode::load_parameters()
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/drive/cmd_vel");
  detections_topic_ = declare_parameter<std::string>("detections_topic", "/detections");
  camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");

  kp_yaw_ = declare_parameter<double>("kp_yaw", 1.1);
  yaw_command_sign_ = declare_parameter<double>("yaw_command_sign", -1.0);
  kd_yaw_ = declare_parameter<double>("kd_yaw", 0.35);
  min_angular_z_ = declare_parameter<double>("min_angular_z", 0.015);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.25);
  max_angular_accel_ = declare_parameter<double>("max_angular_accel", 1.5);
  target_distance_ = declare_parameter<double>("target_distance", 4.0);

  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.008);
  dist_tolerance_ = declare_parameter<double>("dist_tolerance", 0.05);
  test_alignment_only_ = declare_parameter<bool>("test_alignment_only", false);
  test_panel_state_display_ = declare_parameter<bool>("test_panel_state_display", true);
  shot_fallback_timeout_ = declare_parameter<double>("shot_fallback_timeout", 5.0);
  visual_valid_timeout_ = declare_parameter<double>("visual_valid_timeout", 0.5);
  align_lost_timeout_ = declare_parameter<double>("align_lost_timeout", 2.5);
  aim_yaw_offset_deg_ = declare_parameter<double>("aim_yaw_offset_deg", 2.0);

  auto bottom_tags = declare_parameter<std::vector<int64_t>>("bottom_tags", {20, 21, 22});
  auto middle_tags = declare_parameter<std::vector<int64_t>>("middle_tags", {17, 18, 19});
  auto top_tags = declare_parameter<std::vector<int64_t>>("top_tags", {14, 15, 16});

  PKTargetTracker::Config tracker_cfg;
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
  tracker_cfg.tag_lost_timeout = declare_parameter<double>("tag_lost_timeout", 2.5);
  tracker_cfg.aim_yaw_offset_rad = aim_yaw_offset_deg_ * M_PI / 180.0;

  tracker_.set_config(tracker_cfg);
  tracker_.init_custom_tags(bottom_tags, middle_tags, top_tags);
}

rcl_interfaces::msg::SetParametersResult PKAimNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto cfg = tracker_.config();

  for (const auto & p : parameters) {
    if (p.get_name() == "kp_yaw") {
      kp_yaw_ = p.as_double();
    } else if (p.get_name() == "kd_yaw") {
      kd_yaw_ = p.as_double();
    } else if (p.get_name() == "yaw_command_sign") {
      yaw_command_sign_ = p.as_double();
    } else if (p.get_name() == "min_angular_z") {
      min_angular_z_ = p.as_double();
    } else if (p.get_name() == "max_angular_z") {
      max_angular_z_ = p.as_double();
    } else if (p.get_name() == "max_angular_accel") {
      max_angular_accel_ = p.as_double();
    } else if (p.get_name() == "yaw_tolerance") {
      yaw_tolerance_ = p.as_double();
    } else if (p.get_name() == "aim_yaw_offset_deg") {
      aim_yaw_offset_deg_ = p.as_double();
      cfg.aim_yaw_offset_rad = aim_yaw_offset_deg_ * M_PI / 180.0;
    } else if (p.get_name() == "target_distance") {
      target_distance_ = p.as_double();
      cfg.target_distance = target_distance_;
    } else if (p.get_name() == "test_alignment_only") {
      test_alignment_only_ = p.as_bool();
    }
  }

  tracker_.set_config(cfg);
  return result;
}

void PKAimNode::tag_detections_callback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  tracker_.update_detections(*msg, *tf_buffer_, yaw_, now());
}

void PKAimNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  tracker_.update_camera_info(*msg);
}

void PKAimNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw_ = std::atan2(siny_cosp, cosy_cosp);
  gyro_z_ = msg->angular_velocity.z;

  last_imu_time_ = now();
  imu_received_ = true;
}

void PKAimNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_ = msg->data;
  if (emergency_stop_ && state_ != robot_msgs::msg::Game2State::STANDBY) {
    transition_to(robot_msgs::msg::Game2State::STANDBY, "Emergency stop engaged");
  }
}

void PKAimNode::shot_cycle_state_callback(const robot_msgs::msg::ShotCycleState::SharedPtr msg)
{
  const uint8_t current_state = msg->state;
  if (state_ == robot_msgs::msg::Game2State::PREPARING_SHOOT ||
    state_ == robot_msgs::msg::Game2State::ALIGNING)
  {
    // ボール射出完了（FEEDING -> RETURNING）または サイクル終了（-> IDLE）でSTANDBYへ移行
    if ((current_state == robot_msgs::msg::ShotCycleState::RETURNING &&
      prev_shot_cycle_state_ == robot_msgs::msg::ShotCycleState::FEEDING) ||
      (current_state == robot_msgs::msg::ShotCycleState::IDLE &&
      prev_shot_cycle_state_ == robot_msgs::msg::ShotCycleState::RETURNING))
    {
      transition_to(
        robot_msgs::msg::Game2State::STANDBY,
        "Shot completed (ejected). Returned to STANDBY for next target selection");
      tracker_.reset();
    }
  }
  prev_shot_cycle_state_ = current_state;
}

void PKAimNode::shot_cycle_req_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && state_ == robot_msgs::msg::Game2State::PREPARING_SHOOT) {
    last_shot_req_time_ = now();
    RCLCPP_INFO(get_logger(), "🎯 [PK] Shot cycle requested (L2+Circle). Insurance timer started.");
  }
}

void PKAimNode::pk_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    if (state_ == robot_msgs::msg::Game2State::STANDBY) {
      if (tracker_.lock_selected_target(now(), get_logger())) {
        transition_to(
          robot_msgs::msg::Game2State::ALIGNING,
          "PK Start command received (Target locked)");
      }
    }
  } else {
    if (state_ != robot_msgs::msg::Game2State::STANDBY) {
      transition_to(
        robot_msgs::msg::Game2State::STANDBY,
        "PK Stop command received (Cancelled)");
      tracker_.reset();
    }
  }
}

void PKAimNode::pk_next_callback(const std_msgs::msg::Empty::SharedPtr)
{
  if (state_ == robot_msgs::msg::Game2State::STANDBY) {
    int idx = tracker_.select_next();
    const auto & p = tracker_.get_selected_panel();
    RCLCPP_INFO(
      get_logger(),
      "➡️ [PK TARGET NEXT] [Idx %d] %s (Tag #%d | Row %d Col %d | Belt: LEVEL_%d)",
      idx, p.name.c_str(), p.tag_id, p.row, p.col, p.belt_mode);
    publish_target_index();
  }
}

void PKAimNode::pk_prev_callback(const std_msgs::msg::Empty::SharedPtr)
{
  if (state_ == robot_msgs::msg::Game2State::STANDBY) {
    int idx = tracker_.select_prev();
    const auto & p = tracker_.get_selected_panel();
    RCLCPP_INFO(
      get_logger(),
      "⬅️ [PK TARGET PREV] [Idx %d] %s (Tag #%d | Row %d Col %d | Belt: LEVEL_%d)",
      idx, p.name.c_str(), p.tag_id, p.row, p.col, p.belt_mode);
    publish_target_index();
  }
}

void PKAimNode::pk_set_target_index_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (state_ == robot_msgs::msg::Game2State::STANDBY) {
    tracker_.set_selected_index(msg->data);
    const auto & p = tracker_.get_selected_panel();
    RCLCPP_INFO(
      get_logger(),
      "🎯 [PK TARGET SET] [Idx %d] %s (Tag #%d | Row %d Col %d | Belt: LEVEL_%d)",
      p.index, p.name.c_str(), p.tag_id, p.row, p.col, p.belt_mode);
    publish_target_index();
  }
}

void PKAimNode::transition_to(uint8_t new_state, const std::string & reason)
{
  if (state_ == new_state) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "🔄 [PK State Transition] %s -> %s (Reason: %s)",
    (state_ == robot_msgs::msg::Game2State::STANDBY ? "STANDBY" :
    (state_ == robot_msgs::msg::Game2State::ALIGNING ? "ALIGNING" : "PREPARING_SHOOT")),
    (new_state == robot_msgs::msg::Game2State::STANDBY ? "STANDBY" :
    (new_state == robot_msgs::msg::Game2State::ALIGNING ? "ALIGNING" : "PREPARING_SHOOT")),
    reason.c_str());

  state_ = new_state;
  state_entry_time_ = now();

  robot_msgs::msg::Game2State s_msg;
  s_msg.state = state_;
  state_pub_->publish(s_msg);
}

void PKAimNode::publish_target_index()
{
  const auto current_time = now();

  // 1. 主ターゲットIndex (0〜8)
  std_msgs::msg::Int32 idx_msg;
  idx_msg.data = tracker_.selected_index();
  target_index_pub_->publish(idx_msg);

  // 2. 狙っている的インデックス配列 ([selected_index])
  std_msgs::msg::Int32MultiArray target_indices_msg;
  target_indices_msg.data = {tracker_.selected_index()};
  target_indices_pub_->publish(target_indices_msg);

  // 3. 倒れていると判定されている的インデックス配列
  std_msgs::msg::Int32MultiArray fallen_msg;
  auto fallen_indices = tracker_.get_fallen_indices(current_time);
  fallen_msg.data.assign(fallen_indices.begin(), fallen_indices.end());
  fallen_indices_pub_->publish(fallen_msg);

  // 4. Tag ID
  std_msgs::msg::Int32 tag_msg;
  tag_msg.data = tracker_.get_selected_panel().tag_id;
  target_tag_id_pub_->publish(tag_msg);
}

void PKAimNode::control_loop()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.angular.z = 0.0;
  uint8_t current_belt_mode = robot_msgs::msg::BeltMode::STOP;

  if (emergency_stop_) {
    publish_all(cmd, robot_msgs::msg::BeltMode::STOP, false);
    return;
  }

  const auto current_time = now();
  const double dt = 0.02;

  // 定期的に選択中インデックスをパブリッシュ
  publish_target_index();

  // 9マス起立診断表示 (STANDBY 中に0.5秒おきに出力)
  if (test_panel_state_display_ && state_ == robot_msgs::msg::Game2State::STANDBY) {
    const auto & grid = tracker_.panel_grid();
    int sel_idx = tracker_.selected_index();

    auto format_cell = [&](int tag_id, int idx) -> std::string {
        auto it = grid.find(tag_id);
        if (it == grid.end()) {return "#?? 🔴 ( --)";}
        const auto & p = it->second;
        bool visible = p.detected && (current_time - p.last_seen).seconds() <= 1.5;
        std::string prefix = (idx == sel_idx) ? "👉[#" : "  #";
        std::string suffix = (idx == sel_idx) ? "]" : " ";
        char buf[64];
        if (visible) {
          snprintf(buf, sizeof(buf), "%s%d%s🟢 (OK)", prefix.c_str(), tag_id, suffix.c_str());
        } else {
          snprintf(buf, sizeof(buf), "%s%d%s🔴 (--) ", prefix.c_str(), tag_id, suffix.c_str());
        }
        return std::string(buf);
      };

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "\n═══════════════ 🎯 PK 9マス起立・選択モニター ═══════════════\n"
      " [上段 L3] │ %s │ %s │ %s │\n"
      " [中段 L2] │ %s │ %s │ %s │\n"
      " [下段 L1] │ %s │ %s │ %s │\n"
      " 選択中: [Idx %d] %s (Tag #%d) | 👉 = 選択中\n"
      "════════════════════════════════════════════════════════",
      format_cell(14, 0).c_str(), format_cell(15, 1).c_str(), format_cell(16, 2).c_str(),
      format_cell(17, 3).c_str(), format_cell(18, 4).c_str(), format_cell(19, 5).c_str(),
      format_cell(20, 6).c_str(), format_cell(21, 7).c_str(), format_cell(22, 8).c_str(),
      sel_idx, tracker_.get_selected_panel().name.c_str(), tracker_.get_selected_panel().tag_id);
  }

  switch (state_) {
    case robot_msgs::msg::Game2State::STANDBY: {
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;
        current_belt_mode = robot_msgs::msg::BeltMode::STOP;
        break;
      }

    case robot_msgs::msg::Game2State::ALIGNING: {
        tracker_.update_tracking(yaw_, current_time);

        if (!tracker_.is_target_locked()) {
          transition_to(robot_msgs::msg::Game2State::STANDBY, "Target lost in ALIGNING");
          break;
        }

        const double heading_err = tracker_.heading_error();
        const bool is_aligned = (std::abs(heading_err) < yaw_tolerance_);
        const bool is_visible = tracker_.is_currently_visible(current_time, visual_valid_timeout_);

        if (is_visible && is_aligned) {
          cmd.angular.z = 0.0;
          last_cmd_wz_ = 0.0;
          transition_to(
            robot_msgs::msg::Game2State::PREPARING_SHOOT,
            "Target aligned with visual confirmation");
          current_belt_mode =
            test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : tracker_.target_belt_mode();
        } else {
          // PD 旋回制御 (IMU ジャイロ制動ダンピング + Slew Rate)
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
            get_logger(),
            *get_clock(), 500,
            "🎯 [PK ALIGNING | %s] Target: %+.2f deg | Current: %+.2f deg | Err: %+.2f deg | Cmd wz: %+.3f rad/s | %s",
            tracker_.target_description().c_str(),
            target_yaw * 180.0 / M_PI, yaw_ * 180.0 / M_PI, heading_err * 180.0 / M_PI,
            cmd.angular.z, is_visible ? "👁️ VISIBLE" : "📡 IMU DEAD-RECKONING");
        }
        break;
      }

    case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
        cmd.angular.z = 0.0;
        last_cmd_wz_ = 0.0;
        current_belt_mode =
          test_alignment_only_ ? robot_msgs::msg::BeltMode::STOP : tracker_.target_belt_mode();
        tracker_.update_tracking(yaw_, current_time);

        // 外力等で誤差が大きくズレた場合（約3.4度以上）のみ ALIGNING へ戻る（ヒステリシスでチャタリング防止）
        const double prep_abort_tolerance = std::max(yaw_tolerance_ * 3.0, 0.060);
        if (std::abs(tracker_.heading_error()) > prep_abort_tolerance) {
          transition_to(
            robot_msgs::msg::Game2State::ALIGNING,
            "Heading error exceeded tolerance in PREPARING_SHOOT");
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "🚀 [PK PREPARING_SHOOT | %s] Aligned! Spinning Belt (Mode: %u) | Ready for Shot",
            tracker_.target_description().c_str(), current_belt_mode);
        }
        break;
      }

    default:
      break;
  }

  publish_all(cmd, current_belt_mode, false);
}

void PKAimNode::publish_all(
  const geometry_msgs::msg::Twist & cmd_vel,
  uint8_t belt_mode,
  bool completed)
{
  // STANDBY 状態のときは手動走行（joy_controller）と衝突しないよう cmd_vel をパブリッシュしない
  if (state_ != robot_msgs::msg::Game2State::STANDBY) {
    cmd_vel_pub_->publish(cmd_vel);
  }

  robot_msgs::msg::BeltMode mode_msg;
  mode_msg.mode = belt_mode;
  belt_mode_pub_->publish(mode_msg);

  std_msgs::msg::Bool completed_msg;
  completed_msg.data = completed;
  completed_pub_->publish(completed_msg);
}

}  // namespace robot_controller
