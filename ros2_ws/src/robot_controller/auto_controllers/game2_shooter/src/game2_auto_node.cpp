#include "game2_shooter/game2_auto_node.hpp"
#include <algorithm>
#include <cmath>

namespace robot_controller
{

Game2AutoNode::Game2AutoNode(const rclcpp::NodeOptions & options)
: Node("game2_auto_node", options)
{
  load_parameters();

  const auto init_now = this->now();
  state_start_time_ = init_now;
  shoot_start_time_ = init_now;
  ball_detected_time_ = init_now;
  last_imu_time_ = init_now;
  last_loop_time_ = init_now;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto cmd_qos = rclcpp::QoS(10);
  const auto estop_qos = rclcpp::QoS(1).reliable().transient_local();

  // Subscriptions
  detections_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "/detections", cmd_qos,
    std::bind(&Game2AutoNode::tag_detections_callback, this, std::placeholders::_1));

  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/game2/command_start", cmd_qos,
    std::bind(&Game2AutoNode::start_callback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&Game2AutoNode::imu_callback, this, std::placeholders::_1));

  ball_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/ball_detected", cmd_qos,
    std::bind(&Game2AutoNode::ball_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", estop_qos,
    std::bind(&Game2AutoNode::emergency_stop_callback, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, cmd_qos);
  underbelt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/belt/command_underbelt_rpm", cmd_qos);
  upperbelt_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/belt/command_upperbelt_rpm", cmd_qos);
  shoot_trigger_pub_ = create_publisher<std_msgs::msg::Bool>("/belt/shoot_trigger", cmd_qos);
  dribble_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/dribble/command_enabled", cmd_qos);
  arm_position_pub_ =
    create_publisher<robot_msgs::msg::ArmPosition>("/dribble/command_position", cmd_qos);
  completed_pub_ = create_publisher<std_msgs::msg::Bool>("/game2/completed", cmd_qos);
  state_pub_ = create_publisher<robot_msgs::msg::Game2State>(
    "/game2/state", rclcpp::QoS(1).reliable().transient_local());

  // Dynamic Parameter Callback
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&Game2AutoNode::parameter_callback, this, std::placeholders::_1));

  // 20 Hz Control Loop Timer (50 ms)
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Game2AutoNode::control_loop, this));

  RCLCPP_INFO(
    get_logger(),
    "Game2AutoNode initialized. Output CmdVel: %s, TestAlignmentOnly: %s",
    cmd_vel_topic_.c_str(), test_alignment_only_ ? "true" : "false");
}

void Game2AutoNode::load_parameters()
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel_heading");
  tag_prefix_ = declare_parameter<std::string>("tag_prefix", "tag16h5:");

  // Control gains & Limits
  kp_yaw_ = declare_parameter<double>("kp_yaw", 4.0);
  kd_yaw_ = declare_parameter<double>("kd_yaw", 0.30);
  min_angular_z_ = declare_parameter<double>("min_angular_z", 0.12);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 1.80);
  max_angular_accel_ = declare_parameter<double>("max_angular_accel", 8.0);
  target_distance_ = declare_parameter<double>("target_distance", 4.0);

  // Camera & Shooter Physical Parameters (base_link)
  camera_offset_x_ = declare_parameter<double>("camera_offset_x", -0.265);
  camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.035);
  camera_offset_z_ = declare_parameter<double>("camera_offset_z", 0.193);
  target_config_.shooter_offset_x = declare_parameter<double>("shooter_offset_x", -0.265);
  target_config_.shooter_offset_y = declare_parameter<double>("shooter_offset_y", 0.0);
  camera_image_width_ = declare_parameter<double>("camera_image_width", 1920.0);
  camera_image_height_ = declare_parameter<double>("camera_image_height", 1080.0);
  camera_fx_ = declare_parameter<double>("camera_fx", 723.47);
  camera_fy_ = declare_parameter<double>("camera_fy", 723.47);
  camera_cx_ = declare_parameter<double>("camera_cx", 746.35);
  camera_cy_ = declare_parameter<double>("camera_cy", 578.43);

  // Tolerances & Timings
  target_config_.yaw_tolerance = declare_parameter<double>("yaw_tolerance", 0.015);
  dist_tolerance_ = declare_parameter<double>("dist_tolerance", 0.05);
  underbelt_rpm_bottom_ = declare_parameter<double>("underbelt_rpm_bottom", 2050.0);
  upperbelt_rpm_bottom_ = declare_parameter<double>("upperbelt_rpm_bottom", 2050.0);
  underbelt_rpm_middle_ = declare_parameter<double>("underbelt_rpm_middle", 2300.0);
  upperbelt_rpm_middle_ = declare_parameter<double>("upperbelt_rpm_middle", 2300.0);
  underbelt_rpm_top_ = declare_parameter<double>("underbelt_rpm_top", 2700.0);
  upperbelt_rpm_top_ = declare_parameter<double>("upperbelt_rpm_top", 2700.0);
  open_duration_ = declare_parameter<double>("open_duration", 0.3);
  shoot_hold_duration_ = declare_parameter<double>("shoot_hold_duration", 0.8);
  ball_settle_duration_ = declare_parameter<double>("ball_settle_duration", 0.3);
  belt_spinup_duration_ = declare_parameter<double>("belt_spinup_duration", 0.5);
  tag_pitch_x_ = declare_parameter<double>("tag_pitch_x", 0.40);
  tag_pitch_y_ = declare_parameter<double>("tag_pitch_y", 0.43);
  tag_lost_timeout_ = declare_parameter<double>("tag_lost_timeout", 0.5);
  aligning_timeout_ = declare_parameter<double>("aligning_timeout", 10.0);
  shooting_timeout_ = declare_parameter<double>("shooting_timeout", 3.0);
  result_wait_duration_ = declare_parameter<double>("result_wait_duration", 1.0);
  max_shots_per_panel_ = declare_parameter<int>("max_shots_per_panel", 0);
  max_total_balls_ = declare_parameter<int>("max_total_balls", 15);
  enable_ball_limit_ = declare_parameter<bool>("enable_ball_limit", false);
  require_ball_detected_ = declare_parameter<bool>("require_ball_detected", true);
  test_alignment_only_ = declare_parameter<bool>("test_alignment_only", false);
  auto_advance_rows_ = declare_parameter<bool>("auto_advance_rows", true);

  target_config_.enable_double_panel_midpoint_targeting =
    declare_parameter<bool>("enable_double_panel_midpoint_targeting", true);
  target_config_.prefer_same_row_first =
    declare_parameter<bool>("prefer_same_row_first", false);
  target_config_.enable_vertical_sweep =
    declare_parameter<bool>("enable_vertical_sweep", true);
  target_config_.enable_nearest_angle_search =
    declare_parameter<bool>("enable_nearest_angle_search", true);

  // AprilTag Panel Configuration (Row 0: Bottom, 1: Middle, 2: Top)
  const std::vector<int64_t> default_bottom = {20, 21, 22};
  const std::vector<int64_t> default_middle = {17, 18, 19};
  const std::vector<int64_t> default_top = {14, 15, 16};
  const auto bottom_tags = declare_parameter<std::vector<int64_t>>("bottom_tags", default_bottom);
  const auto middle_tags = declare_parameter<std::vector<int64_t>>("middle_tags", default_middle);
  const auto top_tags = declare_parameter<std::vector<int64_t>>("top_tags", default_top);

  vision_tracker_.initialize_grid(bottom_tags, middle_tags, top_tags, this->now());
}

rcl_interfaces::msg::SetParametersResult Game2AutoNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  static const std::unordered_map<std::string, std::function<void(Game2AutoNode*, const rclcpp::Parameter &)>> handlers = {
    {"kp_yaw", [](auto * n, const auto & p) { n->kp_yaw_ = p.as_double(); }},
    {"kd_yaw", [](auto * n, const auto & p) { n->kd_yaw_ = p.as_double(); }},
    {"min_angular_z", [](auto * n, const auto & p) { n->min_angular_z_ = p.as_double(); }},
    {"max_angular_z", [](auto * n, const auto & p) { n->max_angular_z_ = p.as_double(); }},
    {"max_angular_accel", [](auto * n, const auto & p) { n->max_angular_accel_ = p.as_double(); }},
    {"target_distance", [](auto * n, const auto & p) { n->target_distance_ = p.as_double(); }},
    {"shooting_timeout", [](auto * n, const auto & p) { n->shooting_timeout_ = p.as_double(); }},
    {"belt_spinup_duration", [](auto * n, const auto & p) { n->belt_spinup_duration_ = p.as_double(); }},
    {"result_wait_duration", [](auto * n, const auto & p) { n->result_wait_duration_ = p.as_double(); }},
    {"max_total_balls", [](auto * n, const auto & p) { n->max_total_balls_ = p.as_int(); }},
    {"enable_ball_limit", [](auto * n, const auto & p) { n->enable_ball_limit_ = p.as_bool(); }},
    {"max_shots_per_panel", [](auto * n, const auto & p) { n->max_shots_per_panel_ = p.as_int(); }},
    {"require_ball_detected", [](auto * n, const auto & p) { n->require_ball_detected_ = p.as_bool(); }},
    {"test_alignment_only", [](auto * n, const auto & p) { n->test_alignment_only_ = p.as_bool(); }},
    {"tag_pitch_x", [](auto * n, const auto & p) { n->tag_pitch_x_ = p.as_double(); }},
    {"tag_pitch_y", [](auto * n, const auto & p) { n->tag_pitch_y_ = p.as_double(); }},
    {"camera_fx", [](auto * n, const auto & p) { n->camera_fx_ = p.as_double(); }},
    {"camera_offset_x", [](auto * n, const auto & p) { n->camera_offset_x_ = p.as_double(); }},
    {"camera_offset_y", [](auto * n, const auto & p) { n->camera_offset_y_ = p.as_double(); }},
    {"camera_offset_z", [](auto * n, const auto & p) { n->camera_offset_z_ = p.as_double(); }},
    {"underbelt_rpm_bottom", [](auto * n, const auto & p) { n->underbelt_rpm_bottom_ = p.as_double(); }},
    {"upperbelt_rpm_bottom", [](auto * n, const auto & p) { n->upperbelt_rpm_bottom_ = p.as_double(); }},
    {"underbelt_rpm_middle", [](auto * n, const auto & p) { n->underbelt_rpm_middle_ = p.as_double(); }},
    {"upperbelt_rpm_middle", [](auto * n, const auto & p) { n->upperbelt_rpm_middle_ = p.as_double(); }},
    {"underbelt_rpm_top", [](auto * n, const auto & p) { n->underbelt_rpm_top_ = p.as_double(); }},
    {"upperbelt_rpm_top", [](auto * n, const auto & p) { n->upperbelt_rpm_top_ = p.as_double(); }},
    {"yaw_tolerance", [](auto * n, const auto & p) { n->target_config_.yaw_tolerance = p.as_double(); }},
    {"shooter_offset_x", [](auto * n, const auto & p) { n->target_config_.shooter_offset_x = p.as_double(); }},
    {"shooter_offset_y", [](auto * n, const auto & p) { n->target_config_.shooter_offset_y = p.as_double(); }},
    {"enable_double_panel_midpoint_targeting", [](auto * n, const auto & p) {
      n->target_config_.enable_double_panel_midpoint_targeting = p.as_bool();
    }},
    {"prefer_same_row_first", [](auto * n, const auto & p) {
      n->target_config_.prefer_same_row_first = p.as_bool();
    }},
    {"enable_vertical_sweep", [](auto * n, const auto & p) {
      n->target_config_.enable_vertical_sweep = p.as_bool();
    }},
    {"enable_nearest_angle_search", [](auto * n, const auto & p) {
      n->target_config_.enable_nearest_angle_search = p.as_bool();
    }}
  };

  for (const auto & param : parameters) {
    auto it = handlers.find(param.get_name());
    if (it != handlers.end()) {
      it->second(this, param);
    }
  }

  return result;
}

void Game2AutoNode::reset_sequence()
{
  active_row_ = 0;
  active_target_id_ = -1;
  current_target_tag_ids_.clear();
  target_valid_ = false;
  last_cmd_wz_ = 0.0;
  total_shots_fired_ = 0;
  last_target_underbelt_rpm_ = 0.0;
  last_target_upperbelt_rpm_ = 0.0;
  rpm_changed_time_ = this->now();

  for (auto & [id, panel] : vision_tracker_.get_panel_grid()) {
    panel.shot_completed = false;
    panel.shot_count = 0;
    panel.detected = false;
  }
}

void Game2AutoNode::start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !is_enabled_) {
    is_enabled_ = true;
    reset_sequence();
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    yaw_offset_ = raw_yaw_;
    yaw_ = 0.0;
    const auto start_time = this->now();
    state_start_time_ = start_time;
    ball_detected_time_ = start_time;
    shoot_start_time_ = start_time;
    last_imu_time_ = start_time;
    last_loop_time_ = start_time;
    RCLCPP_INFO(
      get_logger(),
      "=== [Game2 START] Sequence Activated (IMU Yaw Zero-Reset Offset: %.3f rad, TestMode: %s) ===",
      yaw_offset_, test_alignment_only_ ? "ON" : "OFF");
  } else if (!msg->data && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    reset_sequence();
    RCLCPP_INFO(get_logger(), "=== [Game2 STOP] Sequence Disengaged ===");
  }
}

void Game2AutoNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_ && is_enabled_) {
    is_enabled_ = false;
    state_ = robot_msgs::msg::Game2State::STANDBY;
    reset_sequence();
    RCLCPP_WARN(get_logger(), "Emergency Stop Triggered! Game 2 Auto Sequence ABORTED.");
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f, 0.0f, false, false,
      robot_msgs::msg::ArmPosition::DRIBBLE, false);
  }
}

void Game2AutoNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_received_ = true;
  last_imu_time_ = now();
  gyro_x_ = msg->angular_velocity.x;
  gyro_y_ = msg->angular_velocity.y;
  gyro_z_ = msg->angular_velocity.z;
  accel_x_ = msg->linear_acceleration.x;
  accel_y_ = msg->linear_acceleration.y;
  accel_z_ = msg->linear_acceleration.z;

  const double qx = msg->orientation.x;
  const double qy = msg->orientation.y;
  const double qz = msg->orientation.z;
  const double qw = msg->orientation.w;

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll_ = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  pitch_ = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  raw_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  yaw_ = std::remainder(raw_yaw_ - yaw_offset_, 2.0 * M_PI);
}

void Game2AutoNode::tag_detections_callback(
  const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  const auto current_time = now();

  vision_tracker_.update_detections(
    *msg, current_time, target_distance_, tag_lost_timeout_,
    tag_pitch_x_, camera_fx_, camera_cx_,
    camera_offset_x_, camera_offset_y_, camera_offset_z_);

  // テストモード時: 射出口正面に最も近いタグを追従
  if (test_alignment_only_) {
    int best_id = -1;
    double min_heading_err_abs = 1e9;
    double best_heading_err = 0.0;
    double best_pixel_x = 0.0;

    for (const auto & detection : msg->detections) {
      const int id = detection.id;
      const auto & grid = vision_tracker_.get_panel_grid();
      if (grid.find(id) != grid.end()) {
        const double u = static_cast<double>(detection.centre.x);
        const double z_cam = target_distance_;
        const double y_offset_from_cam = -(u - camera_cx_) * z_cam / camera_fx_;
        const double x_target = camera_offset_x_ - z_cam;
        const double y_target = camera_offset_y_ + y_offset_from_cam;

        const double dx = x_target - target_config_.shooter_offset_x;
        const double dy = y_target - target_config_.shooter_offset_y;
        const double heading_err = std::remainder(std::atan2(dy, dx) - M_PI, 2.0 * M_PI);

        if (std::abs(heading_err) < min_heading_err_abs) {
          min_heading_err_abs = std::abs(heading_err);
          best_id = id;
          best_heading_err = heading_err;
          best_pixel_x = u;
        }
      }
    }

    if (best_id != -1) {
      active_target_id_ = best_id;
      const auto & target = vision_tracker_.get_panel_grid().at(best_id);
      target_x_ = target.x;
      target_y_ = target.y;
      target_z_ = target.z;
      target_heading_err_ = best_heading_err;
      target_valid_ = true;

      const double aligned_target_pixel =
        camera_cx_ + ((camera_offset_y_ - target_config_.shooter_offset_y) / target_distance_) * camera_fx_;

      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[VisualServo Track Tag #%d] HeadingErr: %+.2f deg | Pixel: %.1f px (TargetAligned: %.1f px)",
        best_id, best_heading_err * 180.0 / M_PI, best_pixel_x, aligned_target_pixel);
    }
  }
}

void Game2AutoNode::ball_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool prev = ball_detected_;
  ball_detected_ = msg->data;
  if (!prev && ball_detected_) {
    ball_detected_time_ = now();
    RCLCPP_INFO(get_logger(), "Game2: Ball DETECTED in dribble intake!");
  } else if (prev && !ball_detected_) {
    RCLCPP_INFO(get_logger(), "Game2: Ball LEFT dribble intake (Shot or Lost)!");
  }
}

void Game2AutoNode::select_target_and_aim()
{
  target_valid_ = false;

  auto best = target_selector_.select_best_target(
    vision_tracker_.get_panel_grid(), active_row_, target_heading_err_,
    current_target_tag_ids_, target_config_, get_logger(), get_clock());

  if (!best) {
    return;
  }

  active_row_ = best->row;
  current_target_tag_ids_ = best->tag_ids;
  target_x_ = best->x;
  target_y_ = best->y;
  target_z_ = best->z;
  target_heading_err_ = best->heading_err;
  active_target_id_ = current_target_tag_ids_.empty() ? -1 : current_target_tag_ids_[0];
  target_valid_ = true;

  double next_under_rpm = underbelt_rpm_bottom_;
  double next_upper_rpm = upperbelt_rpm_bottom_;

  switch (active_row_) {
    case 0:
      next_under_rpm = underbelt_rpm_bottom_;
      next_upper_rpm = upperbelt_rpm_bottom_;
      break;
    case 1:
      next_under_rpm = underbelt_rpm_middle_;
      next_upper_rpm = upperbelt_rpm_middle_;
      break;
    case 2:
      next_under_rpm = underbelt_rpm_top_;
      next_upper_rpm = upperbelt_rpm_top_;
      break;
    default:
      next_under_rpm = underbelt_rpm_bottom_;
      next_upper_rpm = upperbelt_rpm_bottom_;
      break;
  }

  if (std::abs(next_under_rpm - last_target_underbelt_rpm_) > 10.0 ||
    std::abs(next_upper_rpm - last_target_upperbelt_rpm_) > 10.0)
  {
    rpm_changed_time_ = now();
    last_target_underbelt_rpm_ = next_under_rpm;
    last_target_upperbelt_rpm_ = next_upper_rpm;
  }

  target_underbelt_rpm_ = next_under_rpm;
  target_upperbelt_rpm_ = next_upper_rpm;
}

void Game2AutoNode::control_loop()
{
  const auto current_time = now();
  double dt = (current_time - last_loop_time_).seconds();
  if (dt <= 0.001 || dt > 0.2) dt = 0.05;
  last_loop_time_ = current_time;

  // Publish current state
  robot_msgs::msg::Game2State state_msg;
  state_msg.state = state_;
  state_pub_->publish(state_msg);

  if (!is_enabled_ || emergency_stop_active_ || state_ == robot_msgs::msg::Game2State::STANDBY) {
    last_cmd_wz_ = 0.0;
    return;
  }

  vision_tracker_.timeout_unseen_tags(current_time, tag_lost_timeout_);
  select_target_and_aim();

  // Check if all panels are completed or all balls fired (if limit is enabled)
  bool all_panels_completed = true;
  for (const auto & [id, panel] : vision_tracker_.get_panel_grid()) {
    if (!panel.shot_completed) {
      all_panels_completed = false;
      break;
    }
  }

  const bool balls_exhausted = enable_ball_limit_ && (total_shots_fired_ >= max_total_balls_);
  if (all_panels_completed || balls_exhausted) {
    state_ = robot_msgs::msg::Game2State::COMPLETED;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Game 2: Sequence FINISHED! (Total shots: %d%s, All Panels Cleared: %s)",
      total_shots_fired_,
      enable_ball_limit_ ? ("/" + std::to_string(max_total_balls_)).c_str() : " (No limit)",
      all_panels_completed ? "YES" : "NO");
    publish_all(
      geometry_msgs::msg::Twist{}, 0.0f, 0.0f,
      false, false, robot_msgs::msg::ArmPosition::DRIBBLE, true);
    return;
  }

  // Target not detected: Search mode
  if (!target_valid_) {
    state_ = robot_msgs::msg::Game2State::SEARCHING;
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.0;
    last_cmd_wz_ = 0.0;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[Game2 Search] Waiting for target AprilTags (Active Row: %d)... Standing still",
      active_row_);

    publish_all(
      cmd,
      test_alignment_only_ ? 0.0f : static_cast<float>(target_underbelt_rpm_),
      test_alignment_only_ ? 0.0f : static_cast<float>(target_upperbelt_rpm_),
      false, !test_alignment_only_, robot_msgs::msg::ArmPosition::DRIBBLE, false);
    return;
  }

  geometry_msgs::msg::Twist cmd;
  bool shoot_trigger = false;
  uint8_t arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;

  execute_state_machine(current_time, dt, cmd, shoot_trigger, arm_mode);

  publish_all(
    cmd,
    test_alignment_only_ ? 0.0f : static_cast<float>(target_underbelt_rpm_),
    test_alignment_only_ ? 0.0f : static_cast<float>(target_upperbelt_rpm_),
    test_alignment_only_ ? false : shoot_trigger,
    test_alignment_only_ ? false : true,
    arm_mode, state_ == robot_msgs::msg::Game2State::COMPLETED);
}

void Game2AutoNode::execute_state_machine(
  const rclcpp::Time & current_time, double dt,
  geometry_msgs::msg::Twist & cmd, bool & shoot_trigger, uint8_t & arm_mode)
{
  const double state_elapsed = (current_time - state_start_time_).seconds();

  switch (state_) {
    case robot_msgs::msg::Game2State::SEARCHING:
    case robot_msgs::msg::Game2State::ALIGNING: {
      state_ = robot_msgs::msg::Game2State::ALIGNING;

      const double heading_err = target_heading_err_;
      const bool is_aligned = (std::abs(heading_err) < target_config_.yaw_tolerance);

      if (is_aligned) {
        cmd.angular.z = 0.0;
      } else {
        double desired_wz = kp_yaw_ * heading_err;
        if (imu_received_ && (current_time - last_imu_time_).seconds() < 0.5) {
          desired_wz -= kd_yaw_ * gyro_z_;
        }

        const double err_abs = std::abs(heading_err);
        const double slow_zone = target_config_.yaw_tolerance * 3.0;
        double eff_min_wz = min_angular_z_;
        if (err_abs < slow_zone) {
          eff_min_wz = min_angular_z_ * (err_abs / slow_zone);
        }

        if (std::abs(desired_wz) < eff_min_wz) {
          desired_wz = std::copysign(eff_min_wz, desired_wz);
        }
        desired_wz = std::clamp(desired_wz, -max_angular_z_, max_angular_z_);

        const double max_wz_step = max_angular_accel_ * dt;
        if (desired_wz > last_cmd_wz_ + max_wz_step) {
          desired_wz = last_cmd_wz_ + max_wz_step;
        } else if (desired_wz < last_cmd_wz_ - max_wz_step) {
          desired_wz = last_cmd_wz_ - max_wz_step;
        }

        cmd.angular.z = desired_wz;
      }
      last_cmd_wz_ = cmd.angular.z;

      const bool is_ball_settled = !require_ball_detected_ || (
        ball_detected_ &&
        ((current_time - ball_detected_time_).seconds() >= ball_settle_duration_));

      const bool is_belt_ready =
        ((current_time - rpm_changed_time_).seconds() >= belt_spinup_duration_);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "[Game2 Track Tag #%d (Row %d)] Err: %+.2f deg | Cmd wz: %+.3f rad/s | %s | Ball: %s | Belt: %s",
        active_target_id_, active_row_, heading_err * 180.0 / M_PI, cmd.angular.z,
        is_aligned ? "ALIGNED" : "TURNING",
        ball_detected_ ? "YES" : "NO",
        is_belt_ready ? "READY" : "SPINUP");

      if (test_alignment_only_) {
        if (is_aligned) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[Game2 TEST] Target Perfect Aligned! Holding heading.");
        }
      } else {
        if (is_aligned && is_ball_settled && is_belt_ready) {
          RCLCPP_INFO(
            get_logger(),
            "[Game2] Target Aligned & Belt Stable & Ball Settled! Moving arm to OPEN for shooting.");
          state_ = robot_msgs::msg::Game2State::PREPARING_SHOOT;
          state_start_time_ = current_time;
          shoot_start_time_ = current_time;
        } else if (is_aligned && !is_belt_ready) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[Game2] Aligned to Target Tag #%d, waiting for flywheel belt to stabilize (%.0f / %.0f RPM)...",
            active_target_id_, target_underbelt_rpm_, target_upperbelt_rpm_);
        } else if (is_aligned && !is_ball_settled && require_ball_detected_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "[Game2] Aligned to Target Tag #%d, waiting for ball to settle in dribble...",
            active_target_id_);
        }
      }
      arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
      break;
    }

    case robot_msgs::msg::Game2State::PREPARING_SHOOT: {
      arm_mode = robot_msgs::msg::ArmPosition::OPEN;
      cmd.angular.z = 0.0;
      last_cmd_wz_ = 0.0;

      if (state_elapsed >= open_duration_) {
        RCLCPP_INFO(
          get_logger(),
          "[Game2] Arm OPEN complete. Feeding ball to dual flywheel belt (Under RPM: %.0f, Upper RPM: %.0f)!",
          target_underbelt_rpm_, target_upperbelt_rpm_);
        state_ = robot_msgs::msg::Game2State::SHOOTING;
        state_start_time_ = current_time;
        shoot_start_time_ = current_time;
      }
      break;
    }

    case robot_msgs::msg::Game2State::SHOOTING: {
      arm_mode = robot_msgs::msg::ArmPosition::FEED;
      shoot_trigger = true;
      cmd.angular.z = 0.0;
      last_cmd_wz_ = 0.0;

      if (state_elapsed >= shoot_hold_duration_) {
        total_shots_fired_++;
        if (current_target_tag_ids_.size() >= 2) {
          RCLCPP_INFO(
            get_logger(),
            "[Game2 SPLIT SHOT!] Shot #%d/%d triggered for Tags #%d & #%d. Waiting for impact...",
            total_shots_fired_, max_total_balls_,
            current_target_tag_ids_[0], current_target_tag_ids_[1]);
        } else if (!current_target_tag_ids_.empty()) {
          RCLCPP_INFO(
            get_logger(),
            "[Game2] Shot #%d/%d triggered for Tag #%d. Waiting for impact...",
            total_shots_fired_, max_total_balls_,
            current_target_tag_ids_[0]);
        }

        auto & grid = vision_tracker_.get_panel_grid();
        for (const int tid : current_target_tag_ids_) {
          auto it = grid.find(tid);
          if (it != grid.end()) {
            it->second.shot_count++;
          }
        }

        state_ = robot_msgs::msg::Game2State::WAITING_RESULT;
        state_start_time_ = current_time;
        shoot_start_time_ = current_time;
      }
      break;
    }

    case robot_msgs::msg::Game2State::WAITING_RESULT: {
      arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
      cmd.angular.z = 0.0;
      last_cmd_wz_ = 0.0;

      if (state_elapsed >= result_wait_duration_) {
        auto & grid = vision_tracker_.get_panel_grid();
        for (const int tid : current_target_tag_ids_) {
          auto it = grid.find(tid);
          if (it != grid.end()) {
            if (!it->second.detected) {
              it->second.shot_completed = true;
              RCLCPP_INFO(
                get_logger(),
                "[Game2 KNOCKDOWN CONFIRMED!] Tag #%d vanished (panel fell down). Marked as completed! (Shot attempts: %d)",
                tid, it->second.shot_count);
            } else {
              if (max_shots_per_panel_ > 0 && it->second.shot_count >= max_shots_per_panel_) {
                it->second.shot_completed = true;
                RCLCPP_WARN(
                  get_logger(),
                  "[Game2 MAX SHOTS REACHED] Tag #%d reached max attempts (%d/%d). Advancing target.",
                  tid, it->second.shot_count, max_shots_per_panel_);
              } else {
                RCLCPP_WARN(
                  get_logger(),
                  "[Game2 SHOT MISSED / STANDING!] Tag #%d still visible. Will retry until knockdown! (Shot attempts: %d%s)",
                  tid, it->second.shot_count,
                  max_shots_per_panel_ > 0 ? ("/" + std::to_string(max_shots_per_panel_)).c_str() : "");
              }
            }
          }
        }

        state_ = robot_msgs::msg::Game2State::ALIGNING;
        state_start_time_ = current_time;
      }
      break;
    }

    case robot_msgs::msg::Game2State::COMPLETED: {
      arm_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
      cmd.angular.z = 0.0;
      last_cmd_wz_ = 0.0;
      break;
    }

    default:
      break;
  }
}

void Game2AutoNode::publish_all(
  const geometry_msgs::msg::Twist & cmd_vel,
  float underbelt_rpm,
  float upperbelt_rpm,
  bool shoot_trigger,
  bool dribble_enabled,
  uint8_t arm_mode,
  bool completed)
{
  cmd_vel_pub_->publish(cmd_vel);

  std_msgs::msg::Float32 under_rpm_msg;
  under_rpm_msg.data = underbelt_rpm;
  underbelt_rpm_pub_->publish(under_rpm_msg);

  std_msgs::msg::Float32 upper_rpm_msg;
  upper_rpm_msg.data = upperbelt_rpm;
  upperbelt_rpm_pub_->publish(upper_rpm_msg);

  std_msgs::msg::Bool shoot_msg;
  shoot_msg.data = shoot_trigger;
  shoot_trigger_pub_->publish(shoot_msg);

  std_msgs::msg::Bool dribble_msg;
  dribble_msg.data = dribble_enabled;
  dribble_enabled_pub_->publish(dribble_msg);

  robot_msgs::msg::ArmPosition arm_msg;
  arm_msg.position = arm_mode;
  arm_position_pub_->publish(arm_msg);

  std_msgs::msg::Bool completed_msg;
  completed_msg.data = completed;
  completed_pub_->publish(completed_msg);
}

}  // namespace robot_controller
