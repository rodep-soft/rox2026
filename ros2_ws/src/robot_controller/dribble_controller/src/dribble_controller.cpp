#include "dribble_controller/dribble_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>

DribbleControllerNode::DribbleControllerNode()
: Node("dribble_controller_node")
{
  const auto position_target_topic = declare_parameter<std::string>(
    "position_target_topic",
    "/edulite/target");
  const auto roller_target_topic = declare_parameter<std::string>(
    "roller_target_topic",
    "/vesc/target");
  const auto command_period_ms = declare_parameter<int>("command_period_ms", 20);
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  if (position_target_topic.empty() || roller_target_topic.empty()) {
    throw std::runtime_error("target topics must not be empty");
  }
  if (command_period_ms <= 0 || qos_depth <= 0) {
    throw std::runtime_error("command_period_ms and qos_depth must be positive");
  }

  load_parameters();

  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos = rclcpp::QoS(qos_depth);

  position_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    position_target_topic, command_qos);
  roller_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    roller_target_topic, command_qos);
  // dribble_controller -> led_controller: shot cycle state.
  shot_cycle_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/shot_cycle/state", rclcpp::QoS(1).reliable().transient_local());

  // dribble_controller -> joy_controller (belt_mode override during shot cycle).
  belt_mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos);

  // dribble_controller <- belt_controller: belt の現在 mode を把握する。
  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos,
    std::bind(
      &DribbleControllerNode::belt_mode_callback, this,
      std::placeholders::_1));


  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/dribble/position_mode", command_qos,
    std::bind(
      &DribbleControllerNode::position_mode_callback, this,
      std::placeholders::_1));

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/enabled", command_qos,
    std::bind(
      &DribbleControllerNode::dribble_enabled_callback, this,
      std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/shot_cycle/request", command_qos,
    std::bind(
      &DribbleControllerNode::shot_cycle_callback, this,
      std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", emergency_stop_qos,
    std::bind(
      &DribbleControllerNode::emergency_stop_callback, this,
      std::placeholders::_1));

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms),
    std::bind(&DribbleControllerNode::control_timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(
      &DribbleControllerNode::parameter_callback, this,
      std::placeholders::_1));
}

void DribbleControllerNode::load_parameters()
{
  dribble_position_rad_ = declare_parameter<double>("dribble_position_rad", 0.35);
  open_position_rad_ = declare_parameter<double>("open_position_rad", -1.0);
  feed_position_rad_ = declare_parameter<double>("feed_position_rad", 1.3);
  open_duration_sec_ = declare_parameter<double>("open_duration_sec", 0.3);
  feed_duration_sec_ = declare_parameter<double>("feed_duration_sec", 0.6);
  opening_max_velocity_rad_s_ = declare_parameter<double>("opening_max_velocity_rad_s", 4.0);
  feeding_max_velocity_rad_s_ = declare_parameter<double>("feeding_max_velocity_rad_s", 6.0);
  returning_max_velocity_rad_s_ = declare_parameter<double>("returning_max_velocity_rad_s", 4.0);
  dribble_on_rpm_ = declare_parameter<int>("dribble_on_rpm", 800);
  shot_cycle_opening_rpm_ = declare_parameter<int>("shot_cycle_opening_rpm", 800);
  shot_cycle_feeding_rpm_ = declare_parameter<int>("shot_cycle_feeding_rpm", 500);
  shot_cycle_returning_rpm_ = declare_parameter<int>("shot_cycle_returning_rpm", 800);
  shot_cycle_belt_spinup_level_ = static_cast<uint8_t>(
    declare_parameter<int>("shot_cycle_belt_spinup_level", 1));
  belt_spinup_delay_sec_ = declare_parameter<double>("belt_spinup_delay_sec", 0.5);
  const auto position_logical_id = declare_parameter<int>("position_logical_id", 5);
  const auto roller_logical_id = declare_parameter<int>("roller_logical_id", 12);

  if (position_logical_id < 0 || position_logical_id > 65535 || roller_logical_id < 0 ||
    roller_logical_id > 65535)
  {
    throw std::runtime_error("logical IDs must be in [0, 65535]");
  }
  if (dribble_on_rpm_ < 0 || shot_cycle_opening_rpm_ < 0 ||
    shot_cycle_feeding_rpm_ < 0 || shot_cycle_returning_rpm_ < 0)
  {
    throw std::runtime_error("roller RPM parameters must be nonnegative");
  }
  if (shot_cycle_belt_spinup_level_ < 1 || shot_cycle_belt_spinup_level_ > 4) {
    throw std::runtime_error("shot_cycle_belt_spinup_level must be in [1, 4]");
  }
  if (belt_spinup_delay_sec_ < 0.0) {
    throw std::runtime_error("belt_spinup_delay_sec must be nonnegative");
  }
  if (!std::isfinite(dribble_position_rad_) || !std::isfinite(open_position_rad_) ||
    !std::isfinite(feed_position_rad_) || !std::isfinite(open_duration_sec_) ||
    !std::isfinite(feed_duration_sec_) || open_duration_sec_ < 0.0 ||
    feed_duration_sec_ < 0.0 || !std::isfinite(opening_max_velocity_rad_s_) ||
    !std::isfinite(feeding_max_velocity_rad_s_) ||
    !std::isfinite(returning_max_velocity_rad_s_) || opening_max_velocity_rad_s_ <= 0.0 ||
    feeding_max_velocity_rad_s_ <= 0.0 || returning_max_velocity_rad_s_ <= 0.0)
  {
    throw std::runtime_error("position parameters, durations, or velocities are invalid");
  }
  position_logical_id_ = static_cast<uint16_t>(position_logical_id);
  roller_logical_id_ = static_cast<uint16_t>(roller_logical_id);
  last_position_command_rad_ = dribble_position_rad_;
}

void DribbleControllerNode::position_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > static_cast<uint8_t>(PositionMode::FEED)) {
    return;
  }
  const auto new_mode = static_cast<PositionMode>(msg->data);
  if (new_mode != position_mode_ || shot_cycle_active_) {
    shot_cycle_active_ = false; // 手動割り込みで自動サイクルを解除
    manual_transition_active_ = true;
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_position_command_rad_;
    position_mode_ = new_mode;
  }
}

void DribbleControllerNode::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

void DribbleControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  // 自分がpublishした値も含め、/belt/mode の最新値を記録する。
  current_belt_mode_ = msg->data;
}

void DribbleControllerNode::shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || emergency_stop_active_) {
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "Starting Auto Shot Cycle: OPEN -> FEED -> DRIBBLE");
  manual_transition_active_ = false;
  shot_cycle_active_ = true;
  shot_cycle_start_time_ = now();
  shot_cycle_start_position_rad_ = last_position_command_rad_;

  if (current_belt_mode_ == 0) {
    // belt が STOP のとき: 自動で belt を ON にして spin-up 待ちフェーズへ
    belt_auto_started_ = true;
    std_msgs::msg::UInt8 belt_msg;
    belt_msg.data = shot_cycle_belt_spinup_level_;
    belt_mode_pub_->publish(belt_msg);
    RCLCPP_INFO(
      get_logger(),
      "Belt was STOP: auto-starting belt LEVEL_%u, waiting %.2f s for spin-up",
      shot_cycle_belt_spinup_level_, belt_spinup_delay_sec_);
    shot_cycle_phase_ = ShotCyclePhase::BELT_SPINUP;
    // position は DRIBBLE のまま保持（アームは動かさない）
    position_mode_ = PositionMode::DRIBBLE;
  } else {
    // belt が既に回っている: すぐに OPENING 開始
    belt_auto_started_ = false;
    shot_cycle_phase_ = ShotCyclePhase::OPENING;
    position_mode_ = PositionMode::OPEN;
  }
}

void DribbleControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data != emergency_stop_active_) {
    if (msg->data) {
      shot_cycle_active_ = false;
      manual_transition_active_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Emergency stop activated in dribble controller");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Emergency stop released in dribble controller");
    }
  }
  emergency_stop_active_ = msg->data;
  control_timer_callback();
}

rcl_interfaces::msg::SetParametersResult DribbleControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto dribble_position_rad = dribble_position_rad_;
  auto open_position_rad = open_position_rad_;
  auto feed_position_rad = feed_position_rad_;
  auto open_duration_sec = open_duration_sec_;
  auto feed_duration_sec = feed_duration_sec_;
  auto opening_max_vel_rad_s = opening_max_velocity_rad_s_;
  auto feeding_max_vel_rad_s = feeding_max_velocity_rad_s_;
  auto returning_max_vel_rad_s = returning_max_velocity_rad_s_;
  int64_t dribble_on_rpm = dribble_on_rpm_;
  int64_t shot_cycle_opening_rpm = shot_cycle_opening_rpm_;
  int64_t shot_cycle_feeding_rpm = shot_cycle_feeding_rpm_;
  int64_t shot_cycle_returning_rpm = shot_cycle_returning_rpm_;
  bool trajectory_changed = false;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "dribble_on_rpm") {
      dribble_on_rpm = parameter.as_int();
      continue;
    }
    if (name == "shot_cycle_opening_rpm") {
      shot_cycle_opening_rpm = parameter.as_int();
      continue;
    }
    if (name == "shot_cycle_feeding_rpm") {
      shot_cycle_feeding_rpm = parameter.as_int();
      continue;
    }
    if (name == "shot_cycle_returning_rpm") {
      shot_cycle_returning_rpm = parameter.as_int();
      continue;
    }

    bool restart_required = true;
    bool value_unchanged = false;
    if (name == "command_period_ms" || name == "qos_depth") {
      value_unchanged =
        parameter.as_int() == get_parameter(name).as_int();
    } else if (name == "position_logical_id") {
      value_unchanged = parameter.as_int() == position_logical_id_;
    } else if (name == "roller_logical_id") {
      value_unchanged = parameter.as_int() == roller_logical_id_;
    } else if (name == "position_target_topic" ||
      name == "roller_target_topic")
    {
      value_unchanged =
        parameter.as_string() == get_parameter(name).as_string();
    } else {
      restart_required = false;
    }

    if (restart_required) {
      if (value_unchanged) {
        continue;
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = name + " requires a node restart";
      return result;
    }

    trajectory_changed = true;
    if (name == "dribble_position_rad") {
      dribble_position_rad = parameter.as_double();
    } else if (name == "open_position_rad") {
      open_position_rad = parameter.as_double();
    } else if (name == "feed_position_rad") {
      feed_position_rad = parameter.as_double();
    } else if (name == "open_duration_sec") {
      open_duration_sec = parameter.as_double();
    } else if (name == "feed_duration_sec") {
      feed_duration_sec = parameter.as_double();
    } else if (name == "opening_max_velocity_rad_s") {
      opening_max_vel_rad_s = parameter.as_double();
    } else if (name == "feeding_max_velocity_rad_s") {
      feeding_max_vel_rad_s = parameter.as_double();
    } else if (name == "returning_max_velocity_rad_s") {
      returning_max_vel_rad_s = parameter.as_double();
    } else {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = name + " is not a supported parameter";
      return result;
    }
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
    dribble_on_rpm <= std::numeric_limits<int>::max() &&
    shot_cycle_opening_rpm >= 0 &&
    shot_cycle_opening_rpm <= std::numeric_limits<int>::max() &&
    shot_cycle_feeding_rpm >= 0 &&
    shot_cycle_feeding_rpm <= std::numeric_limits<int>::max() &&
    shot_cycle_returning_rpm >= 0 &&
    shot_cycle_returning_rpm <= std::numeric_limits<int>::max();
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
  shot_cycle_opening_rpm_ = static_cast<int>(shot_cycle_opening_rpm);
  shot_cycle_feeding_rpm_ = static_cast<int>(shot_cycle_feeding_rpm);
  shot_cycle_returning_rpm_ = static_cast<int>(shot_cycle_returning_rpm);

  if (trajectory_changed && manual_transition_active_) {
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_position_command_rad_;
  }
  if (trajectory_changed && shot_cycle_active_) {
    shot_cycle_start_time_ = now();
    shot_cycle_start_position_rad_ = last_position_command_rad_;
  }
  return result;
}

// ────────────────────────────────────────────────────────────────────────────
// タイマーコールバック（ステートマシン進行 + publish）
// ────────────────────────────────────────────────────────────────────────────

void DribbleControllerNode::control_timer_callback()
{
  publish_shot_cycle_state();
  actuator_msgs::msg::ActuatorTarget roller_command;
  roller_command.logical_id = roller_logical_id_;
  roller_command.target = static_cast<float>(roller_target_rpm());
  roller_command_pub_->publish(roller_command);

  if (emergency_stop_active_) {
    shot_cycle_active_ = false;
    manual_transition_active_ = false;
    actuator_msgs::msg::ActuatorTarget position_command;
    position_command.logical_id = position_logical_id_;
    position_command.target = static_cast<float>(dribble_position_rad_);
    last_position_command_rad_ = dribble_position_rad_;
    position_command_pub_->publish(position_command);
    return;
  }

  double position_command_rad = target_position_rad();

  if (manual_transition_active_ && !shot_cycle_active_) {
    const double mode_target_rad = target_position_rad();
    double max_vel_rad_s = returning_max_velocity_rad_s_;
    if (position_mode_ == PositionMode::OPEN) {
      max_vel_rad_s = opening_max_velocity_rad_s_;
    } else if (position_mode_ == PositionMode::FEED) {
      max_vel_rad_s = feeding_max_velocity_rad_s_;
    }

    const double elapsed_sec =
      (now() - manual_transition_start_time_).seconds();
    const double move_duration_sec = transition_duration_sec(
      manual_transition_start_position_rad_, mode_target_rad, max_vel_rad_s);
    position_command_rad = interpolated_position_rad(
      manual_transition_start_position_rad_, mode_target_rad, elapsed_sec,
      max_vel_rad_s);
    if (elapsed_sec >= move_duration_sec) {
      manual_transition_active_ = false;
      position_command_rad = mode_target_rad;
    }
  }

  if (shot_cycle_active_) {
    // ── BELT_SPINUP フェーズ ───────────────────────────────────────────────
    if (shot_cycle_phase_ == ShotCyclePhase::BELT_SPINUP) {
      const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
      if (elapsed_sec >= belt_spinup_delay_sec_) {
        // spin-up 完了 → OPENING へ移行
        shot_cycle_phase_ = ShotCyclePhase::OPENING;
        shot_cycle_start_time_ = now();
        shot_cycle_start_position_rad_ = last_position_command_rad_;
        position_mode_ = PositionMode::OPEN;
        RCLCPP_INFO(get_logger(), "Shot Cycle: BELT_SPINUP -> OPEN");
      }
      // BELT_SPINUP 中はアームを動かさない: position_command_rad はそのまま
    }

    if (shot_cycle_phase_ != ShotCyclePhase::BELT_SPINUP) {
      double phase_target_rad = dribble_position_rad_;
      double phase_max_vel_rad_s = returning_max_velocity_rad_s_;
      double hold_duration_sec = 0.0;

      switch (shot_cycle_phase_) {
        case ShotCyclePhase::OPENING:
          position_mode_ = PositionMode::OPEN;
          phase_target_rad = open_position_rad_;
          phase_max_vel_rad_s = opening_max_velocity_rad_s_;
          hold_duration_sec = open_duration_sec_;
          break;
        case ShotCyclePhase::FEEDING:
          position_mode_ = PositionMode::FEED;
          phase_target_rad = feed_position_rad_;
          phase_max_vel_rad_s = feeding_max_velocity_rad_s_;
          hold_duration_sec = feed_duration_sec_;
          break;
        case ShotCyclePhase::RETURNING:
          position_mode_ = PositionMode::DRIBBLE;
          break;
      }

      const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
      const double move_duration_sec = transition_duration_sec(
        shot_cycle_start_position_rad_, phase_target_rad,
        phase_max_vel_rad_s);
      position_command_rad = interpolated_position_rad(
        shot_cycle_start_position_rad_, phase_target_rad, elapsed_sec,
        phase_max_vel_rad_s);

      if (elapsed_sec >= move_duration_sec + hold_duration_sec) {
        position_command_rad = phase_target_rad;
        shot_cycle_start_position_rad_ = phase_target_rad;
        shot_cycle_start_time_ = now();

        if (shot_cycle_phase_ == ShotCyclePhase::OPENING) {
          shot_cycle_phase_ = ShotCyclePhase::FEEDING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: OPEN -> FEED");
        } else if (shot_cycle_phase_ == ShotCyclePhase::FEEDING) {
          shot_cycle_phase_ = ShotCyclePhase::RETURNING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: FEED -> DRIBBLE");
        } else {
          shot_cycle_active_ = false;
          // 自動でbeltをONした場合は完了後にSTOPへ戻す
          if (belt_auto_started_) {
            belt_auto_started_ = false;
            std_msgs::msg::UInt8 belt_msg;
            belt_msg.data = 0; // STOP
            belt_mode_pub_->publish(belt_msg);
            RCLCPP_INFO(get_logger(), "Shot Cycle Completed: Belt auto-stopped");
          }
          RCLCPP_INFO(
            get_logger(),
            "Shot Cycle Completed: Returned to DRIBBLE");
        }
      }
    }  // if (!= BELT_SPINUP)
  }  // if (shot_cycle_active_)

  actuator_msgs::msg::ActuatorTarget position_command;
  position_command.logical_id = position_logical_id_;
  position_command.target = static_cast<float>(position_command_rad);
  last_position_command_rad_ = position_command_rad;
  position_command_pub_->publish(position_command);
}

int DribbleControllerNode::roller_target_rpm() const
{
  if (!dribble_enabled_ || emergency_stop_active_) {
    return 0;
  }
  if (!shot_cycle_active_) {
    return dribble_on_rpm_;
  }

  switch (shot_cycle_phase_) {
    case ShotCyclePhase::BELT_SPINUP:
      return dribble_on_rpm_; // dribble roller は通常 ON のまま
    case ShotCyclePhase::OPENING:
      return shot_cycle_opening_rpm_;
    case ShotCyclePhase::FEEDING:
      return shot_cycle_feeding_rpm_;
    case ShotCyclePhase::RETURNING:
      return shot_cycle_returning_rpm_;
  }
  return dribble_on_rpm_;
}

void DribbleControllerNode::publish_shot_cycle_state()
{
  std_msgs::msg::UInt8 state;
  state.data = shot_cycle_active_ ?
    static_cast<uint8_t>(shot_cycle_phase_) + 1U :
    0U;
  shot_cycle_state_pub_->publish(state);
}

// ────────────────────────────────────────────────────────────────────────────
// ヘルパー
// ────────────────────────────────────────────────────────────────────────────

double DribbleControllerNode::target_position_rad() const
{
  switch (position_mode_) {
    case PositionMode::DRIBBLE:
      return dribble_position_rad_;
    case PositionMode::OPEN:
      return open_position_rad_;
    case PositionMode::FEED:
      return feed_position_rad_;
  }
  return dribble_position_rad_;
}

double DribbleControllerNode::interpolated_position_rad(
  double start_rad, double target_rad, double elapsed_sec,
  double max_vel_rad_s) const
{
  const double duration_sec =
    transition_duration_sec(start_rad, target_rad, max_vel_rad_s);
  if (duration_sec <= 0.0) {
    return target_rad;
  }
  const double progress =
    std::clamp(elapsed_sec / duration_sec, 0.0, 1.0);
  // Quintic smootherstep: velocity and acceleration are both zero at each
  // endpoint, avoiding a mechanical shock when the shot phase changes.
  const double smooth_progress =
    progress * progress * progress *
    (progress * (6.0 * progress - 15.0) + 10.0);
  return start_rad + (target_rad - start_rad) * smooth_progress;
}

double DribbleControllerNode::transition_duration_sec(
  double start_rad, double target_rad, double max_vel_rad_s) const
{
  // Quintic smootherstep's peak derivative is 1.875.
  return 1.875 * std::abs(target_rad - start_rad) / max_vel_rad_s;
}
