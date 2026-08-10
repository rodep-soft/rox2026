#include "dribble_controller/dribble_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>

// ────────────────────────────────────────────────────────────────────────────
// コンストラクタ
// ────────────────────────────────────────────────────────────────────────────

DribbleControllerNode::DribbleControllerNode()
    : Node("dribble_controller_node") {
  declare_parameters();
  get_parameters();

  const auto position_target_topic = get_parameter("position_target_topic").as_string();
  const auto roller_target_topic = get_parameter("roller_target_topic").as_string();
  if (position_target_topic.empty() || roller_target_topic.empty()) {
    throw std::runtime_error("target topics must not be empty");
  }

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos = rclcpp::QoS(qos_depth_);

  position_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
      position_target_topic, command_qos);
  roller_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
      roller_target_topic, command_qos);

  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/dribble/position_mode", command_qos,
      std::bind(&DribbleControllerNode::position_mode_callback, this,
                std::placeholders::_1));

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dribble/enabled", command_qos,
      std::bind(&DribbleControllerNode::dribble_enabled_callback, this,
                std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/shot_cycle/request", command_qos,
      std::bind(&DribbleControllerNode::shot_cycle_callback, this,
                std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", state_qos,
      std::bind(&DribbleControllerNode::emergency_stop_callback, this,
                std::placeholders::_1));

  timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&DribbleControllerNode::timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&DribbleControllerNode::parameter_callback, this,
                std::placeholders::_1));
}

// ────────────────────────────────────────────────────────────────────────────
// パラメータ
// ────────────────────────────────────────────────────────────────────────────

void DribbleControllerNode::declare_parameters() {
  declare_parameter<double>("dribble_position_rad", 0.35);
  declare_parameter<double>("open_position_rad", -1.0);
  declare_parameter<double>("feed_position_rad", 1.3);
  declare_parameter<double>("open_duration_sec", 0.3);
  declare_parameter<double>("feed_duration_sec", 0.6);
  declare_parameter<double>("opening_max_velocity_rad_s", 4.0);
  declare_parameter<double>("feeding_max_velocity_rad_s", 6.0);
  declare_parameter<double>("returning_max_velocity_rad_s", 4.0);
  declare_parameter<int>("dribble_on_rpm", 800);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
  declare_parameter<int>("position_logical_id", 5);
  declare_parameter<int>("roller_logical_id", 12);
  declare_parameter<std::string>("position_target_topic", "/edulite/target");
  declare_parameter<std::string>("roller_target_topic", "/vesc/target");
}

void DribbleControllerNode::get_parameters() {
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("open_position_rad", open_position_rad_);
  get_parameter("feed_position_rad", feed_position_rad_);
  get_parameter("open_duration_sec", open_duration_sec_);
  get_parameter("feed_duration_sec", feed_duration_sec_);
  get_parameter("opening_max_velocity_rad_s", opening_max_velocity_rad_s_);
  get_parameter("feeding_max_velocity_rad_s", feeding_max_velocity_rad_s_);
  get_parameter("returning_max_velocity_rad_s", returning_max_velocity_rad_s_);
  get_parameter("dribble_on_rpm", dribble_on_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
  const auto position_logical_id = get_parameter("position_logical_id").as_int();
  const auto roller_logical_id = get_parameter("roller_logical_id").as_int();

  if (position_logical_id < 0 || position_logical_id > 65535 ||
      roller_logical_id < 0 || roller_logical_id > 65535) {
    throw std::runtime_error("logical IDs must be in [0, 65535]");
  }
  if (command_period_ms_ <= 0 || qos_depth_ <= 0 || dribble_on_rpm_ < 0) {
    throw std::runtime_error("period and QoS must be positive, and RPM must be nonnegative");
  }
  if (!std::isfinite(dribble_position_rad_) || !std::isfinite(open_position_rad_) ||
      !std::isfinite(feed_position_rad_) || !std::isfinite(open_duration_sec_) ||
      !std::isfinite(feed_duration_sec_) || open_duration_sec_ < 0.0 ||
      feed_duration_sec_ < 0.0 || !std::isfinite(opening_max_velocity_rad_s_) ||
      !std::isfinite(feeding_max_velocity_rad_s_) ||
      !std::isfinite(returning_max_velocity_rad_s_) || opening_max_velocity_rad_s_ <= 0.0 ||
      feeding_max_velocity_rad_s_ <= 0.0 || returning_max_velocity_rad_s_ <= 0.0) {
    throw std::runtime_error("position parameters, durations, or velocities are invalid");
  }
  position_logical_id_ = static_cast<uint16_t>(position_logical_id);
  roller_logical_id_ = static_cast<uint16_t>(roller_logical_id);
  last_command_position_rad_ = dribble_position_rad_;
}

// ────────────────────────────────────────────────────────────────────────────
// コールバック
// ────────────────────────────────────────────────────────────────────────────

void DribbleControllerNode::position_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (msg->data > static_cast<uint8_t>(PositionMode::FEED)) {
    return;
  }
  const auto new_mode = static_cast<PositionMode>(msg->data);
  if (new_mode != current_position_mode_ || in_shot_cycle_) {
    in_shot_cycle_ = false; // 手動割り込みで自動サイクルを解除
    manual_transition_active_ = true;
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_command_position_rad_;
    RCLCPP_INFO(get_logger(), "Arm Mode Changed: %s -> %s",
                mode_name(current_position_mode_), mode_name(new_mode));
    current_position_mode_ = new_mode;
  }
}

void DribbleControllerNode::dribble_enabled_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  dribble_enabled_ = msg->data;
  timer_callback();
}

void DribbleControllerNode::shot_cycle_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || emergency_stop_active_) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Starting Auto Shot Cycle: OPEN -> FEED -> DRIBBLE");
  manual_transition_active_ = false;
  in_shot_cycle_ = true;
  shot_cycle_phase_ = ShotCyclePhase::OPENING;
  shot_cycle_start_time_ = now();
  shot_cycle_start_position_rad_ = last_command_position_rad_;
  current_position_mode_ = PositionMode::OPEN;
}

void DribbleControllerNode::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data != emergency_stop_active_) {
    if (msg->data) {
      in_shot_cycle_ = false;
      manual_transition_active_ = false;
      RCLCPP_WARN(get_logger(),
                  "Emergency stop activated in dribble controller");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Emergency stop released in dribble controller");
    }
  }
  emergency_stop_active_ = msg->data;
  timer_callback();
}

rcl_interfaces::msg::SetParametersResult DribbleControllerNode::parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters) {
  auto dribble_position_rad = dribble_position_rad_;
  auto open_position_rad = open_position_rad_;
  auto feed_position_rad = feed_position_rad_;
  auto open_duration_sec = open_duration_sec_;
  auto feed_duration_sec = feed_duration_sec_;
  auto opening_max_vel_rad_s = opening_max_velocity_rad_s_;
  auto feeding_max_vel_rad_s = feeding_max_velocity_rad_s_;
  auto returning_max_vel_rad_s = returning_max_velocity_rad_s_;
  int64_t dribble_on_rpm = dribble_on_rpm_;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "dribble_position_rad") dribble_position_rad = parameter.as_double();
    else if (name == "open_position_rad") open_position_rad = parameter.as_double();
    else if (name == "feed_position_rad") feed_position_rad = parameter.as_double();
    else if (name == "open_duration_sec") open_duration_sec = parameter.as_double();
    else if (name == "feed_duration_sec") feed_duration_sec = parameter.as_double();
    else if (name == "opening_max_velocity_rad_s") opening_max_vel_rad_s = parameter.as_double();
    else if (name == "feeding_max_velocity_rad_s") feeding_max_vel_rad_s = parameter.as_double();
    else if (name == "returning_max_velocity_rad_s") returning_max_vel_rad_s = parameter.as_double();
    else if (name == "dribble_on_rpm") dribble_on_rpm = parameter.as_int();
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful =
      std::isfinite(dribble_position_rad) && std::isfinite(open_position_rad) &&
      std::isfinite(feed_position_rad) && std::isfinite(open_duration_sec) &&
      std::isfinite(feed_duration_sec) && open_duration_sec >= 0.0 &&
      feed_duration_sec >= 0.0 && std::isfinite(opening_max_vel_rad_s) &&
      std::isfinite(feeding_max_vel_rad_s) && std::isfinite(returning_max_vel_rad_s) &&
      opening_max_vel_rad_s > 0.0 && feeding_max_vel_rad_s > 0.0 &&
      returning_max_vel_rad_s > 0.0 && dribble_on_rpm >= 0 &&
      dribble_on_rpm <= std::numeric_limits<int>::max();
  if (!result.successful) {
    result.reason = "positions must be finite, durations/RPM nonnegative, and velocities positive";
    return result;
  }

  dribble_position_rad_ = dribble_position_rad;
  open_position_rad_ = open_position_rad;
  feed_position_rad_ = feed_position_rad;
  open_duration_sec_ = open_duration_sec;
  feed_duration_sec_ = feed_duration_sec;
  opening_max_velocity_rad_s_ = opening_max_vel_rad_s;
  feeding_max_velocity_rad_s_ = feeding_max_vel_rad_s;
  returning_max_velocity_rad_s_ = returning_max_vel_rad_s;
  dribble_on_rpm_ = static_cast<int>(dribble_on_rpm);

  if (manual_transition_active_) {
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_command_position_rad_;
  }
  if (in_shot_cycle_) {
    shot_cycle_start_time_ = now();
    shot_cycle_start_position_rad_ = last_command_position_rad_;
  }
  return result;
}

// ────────────────────────────────────────────────────────────────────────────
// タイマーコールバック（ステートマシン進行 + publish）
// ────────────────────────────────────────────────────────────────────────────

void DribbleControllerNode::timer_callback() {
  actuator_msgs::msg::ActuatorTarget roller_command;
  roller_command.logical_id = roller_logical_id_;
  roller_command.target = static_cast<float>(
      dribble_enabled_ && !emergency_stop_active_ ? dribble_on_rpm_ : 0);
  roller_command_pub_->publish(roller_command);

  if (emergency_stop_active_) {
    in_shot_cycle_ = false;
    manual_transition_active_ = false;
    actuator_msgs::msg::ActuatorTarget msg;
    msg.logical_id = position_logical_id_;
    msg.target = static_cast<float>(dribble_position_rad_);
    last_command_position_rad_ = dribble_position_rad_;
    position_command_pub_->publish(msg);
    return;
  }

  double command_position_rad = target_position_rad();

  if (manual_transition_active_ && !in_shot_cycle_) {
    const double target = target_position_rad();
    double max_velocity = returning_max_velocity_rad_s_;
    if (current_position_mode_ == PositionMode::OPEN) {
      max_velocity = opening_max_velocity_rad_s_;
    } else if (current_position_mode_ == PositionMode::FEED) {
      max_velocity = feeding_max_velocity_rad_s_;
    }

    const double elapsed =
        (now() - manual_transition_start_time_).seconds();
    const double duration = transition_duration_sec(
        manual_transition_start_position_rad_, target, max_velocity);
    command_position_rad = interpolated_position_rad(
        manual_transition_start_position_rad_, target, elapsed, max_velocity);
    if (elapsed >= duration) {
      manual_transition_active_ = false;
      command_position_rad = target;
    }
  }

  // Progress the automatic shot cycle with a smooth velocity-limited
  // trajectory.
  if (in_shot_cycle_) {
    const double elapsed = (now() - shot_cycle_start_time_).seconds();
    switch (shot_cycle_phase_) {
    case ShotCyclePhase::OPENING: {
      current_position_mode_ = PositionMode::OPEN;
      const double movement_duration = transition_duration_sec(
          shot_cycle_start_position_rad_, open_position_rad_,
          opening_max_velocity_rad_s_);
      command_position_rad = interpolated_position_rad(
          shot_cycle_start_position_rad_, open_position_rad_, elapsed,
          opening_max_velocity_rad_s_);
      if (elapsed >= movement_duration + open_duration_sec_) {
        shot_cycle_start_position_rad_ = command_position_rad;
        shot_cycle_phase_ = ShotCyclePhase::FEEDING;
        shot_cycle_start_time_ = now();
        current_position_mode_ = PositionMode::FEED;
        RCLCPP_INFO(get_logger(), "Shot Cycle: OPEN -> FEED");
      }
      break;
    }
    case ShotCyclePhase::FEEDING: {
      current_position_mode_ = PositionMode::FEED;
      const double movement_duration = transition_duration_sec(
          shot_cycle_start_position_rad_, feed_position_rad_, feeding_max_velocity_rad_s_);
      command_position_rad = interpolated_position_rad(
          shot_cycle_start_position_rad_, feed_position_rad_, elapsed,
          feeding_max_velocity_rad_s_);
      if (elapsed >= movement_duration + feed_duration_sec_) {
        shot_cycle_start_position_rad_ = command_position_rad;
        shot_cycle_phase_ = ShotCyclePhase::RETURNING;
        shot_cycle_start_time_ = now();
        current_position_mode_ = PositionMode::DRIBBLE;
        RCLCPP_INFO(get_logger(), "Shot Cycle: FEED -> DRIBBLE");
      }
      break;
    }
    case ShotCyclePhase::RETURNING: {
      current_position_mode_ = PositionMode::DRIBBLE;
      const double movement_duration =
          transition_duration_sec(shot_cycle_start_position_rad_, dribble_position_rad_,
                                  returning_max_velocity_rad_s_);
      command_position_rad = interpolated_position_rad(
          shot_cycle_start_position_rad_, dribble_position_rad_, elapsed,
          returning_max_velocity_rad_s_);
      if (elapsed >= movement_duration) {
        in_shot_cycle_ = false;
        command_position_rad = dribble_position_rad_;
        RCLCPP_INFO(get_logger(), "Shot Cycle Completed: Returned to DRIBBLE");
      }
      break;
    }
    }
  }

  actuator_msgs::msg::ActuatorTarget msg;
  msg.logical_id = position_logical_id_;
  msg.target = static_cast<float>(command_position_rad);
  last_command_position_rad_ = command_position_rad;
  position_command_pub_->publish(msg);
}

// ────────────────────────────────────────────────────────────────────────────
// ヘルパー
// ────────────────────────────────────────────────────────────────────────────

const char *DribbleControllerNode::mode_name(PositionMode mode) const {
  switch (mode) {
  case PositionMode::DRIBBLE:
    return "DRIBBLE";
  case PositionMode::OPEN:
    return "OPEN";
  case PositionMode::FEED:
    return "FEED";
  }
  return "UNKNOWN";
}

float DribbleControllerNode::target_position_rad() const {
  switch (current_position_mode_) {
  case PositionMode::DRIBBLE:
    return static_cast<float>(dribble_position_rad_);
  case PositionMode::OPEN:
    return static_cast<float>(open_position_rad_);
  case PositionMode::FEED:
    return static_cast<float>(feed_position_rad_);
  }
  return static_cast<float>(dribble_position_rad_);
}

float DribbleControllerNode::interpolated_position_rad(
    double from, double to, double elapsed_sec,
    double max_velocity_rad_s) const {
  const double duration = transition_duration_sec(from, to, max_velocity_rad_s);
  if (duration <= 0.0) {
    return static_cast<float>(to);
  }
  const double progress = std::clamp(elapsed_sec / duration, 0.0, 1.0);
  // Quintic smootherstep: velocity and acceleration are both zero at each
  // endpoint, avoiding a mechanical shock when the shot phase changes.
  const double smooth_progress =
      progress * progress * progress *
      (progress * (6.0 * progress - 15.0) + 10.0);
  return static_cast<float>(from + (to - from) * smooth_progress);
}

double DribbleControllerNode::transition_duration_sec(
    double from, double to, double max_velocity_rad_s) const {
  // Quintic smootherstep's peak derivative is 1.875.
  return 1.875 * std::abs(to - from) / max_velocity_rad_s;
}
