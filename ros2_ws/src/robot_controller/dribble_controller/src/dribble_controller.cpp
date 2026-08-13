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
  shot_cycle_state_pub_ = create_publisher<robot_msgs::msg::ShotCycleState>(
    "/dribble/shot_cycle_state", rclcpp::QoS(1).reliable().transient_local());

  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", command_qos);

  belt_mode_sub_ = create_subscription<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", command_qos,
    std::bind(&DribbleControllerNode::belt_mode_callback, this, std::placeholders::_1));

  position_mode_sub_ = create_subscription<robot_msgs::msg::ArmPosition>(
    "/dribble/command_position", command_qos,
    std::bind(&DribbleControllerNode::position_mode_callback, this, std::placeholders::_1));

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/command_enabled", command_qos,
    std::bind(&DribbleControllerNode::dribble_enabled_callback, this, std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", command_qos,
    std::bind(&DribbleControllerNode::shot_cycle_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", emergency_stop_qos,
    std::bind(&DribbleControllerNode::emergency_stop_callback, this, std::placeholders::_1));

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
  const robot_msgs::msg::ArmPosition::SharedPtr msg)
{
  if (msg->position > robot_msgs::msg::ArmPosition::FEED) {return;}

  if (msg->position != position_mode_ || shot_cycle_active_) {
    shot_cycle_active_ = false;
    manual_transition_active_ = true;
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_position_command_rad_;
    position_mode_ = msg->position;
  }
}

void DribbleControllerNode::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

void DribbleControllerNode::belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg)
{
  current_belt_mode_ = msg->mode;
}

void DribbleControllerNode::shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || emergency_stop_active_) {return;}

  RCLCPP_INFO(get_logger(), "Starting Auto Shot Cycle: OPEN -> FEED -> DRIBBLE");
  manual_transition_active_ = false;
  shot_cycle_active_ = true;
  shot_cycle_start_time_ = now();
  shot_cycle_start_position_rad_ = last_position_command_rad_;

  if (current_belt_mode_ == robot_msgs::msg::BeltMode::STOP) {
    belt_auto_started_ = true;
    robot_msgs::msg::BeltMode belt_msg;
    belt_msg.mode = shot_cycle_belt_spinup_level_;
    belt_mode_pub_->publish(belt_msg);
    RCLCPP_INFO(
      get_logger(),
      "Belt was STOP: auto-starting belt LEVEL_%u, waiting %.2f s for spin-up",
      shot_cycle_belt_spinup_level_, belt_spinup_delay_sec_);
    shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::BELT_SPINUP;
    position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
  } else {
    belt_auto_started_ = false;
    shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::OPENING;
    position_mode_ = robot_msgs::msg::ArmPosition::OPEN;
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
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool trajectory_changed = false;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    // 再起動が必要なパラメータ
    if (name == "command_period_ms" || name == "qos_depth" ||
      name == "position_logical_id" || name == "roller_logical_id" ||
      name == "position_target_topic" || name == "roller_target_topic")
    {
      result.successful = false;
      result.reason = name + " requires a node restart";
      return result;
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      const int val = static_cast<int>(param.as_int());
      if (val < 0) {
        result.successful = false; result.reason = name + " must be non-negative"; return result;
      }
      if (name == "dribble_on_rpm") {
        dribble_on_rpm_ = val;
      } else if (name == "shot_cycle_opening_rpm") {
        shot_cycle_opening_rpm_ = val;
      } else if (name == "shot_cycle_feeding_rpm") {
        shot_cycle_feeding_rpm_ = val;
      } else if (name == "shot_cycle_returning_rpm") {
        shot_cycle_returning_rpm_ = val;
      } else if (name == "shot_cycle_belt_spinup_level") {
        shot_cycle_belt_spinup_level_ = static_cast<uint8_t>(val);
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double val = param.as_double();
      if (!std::isfinite(val)) {
        result.successful = false; result.reason = name + " must be finite"; return result;
      }
      trajectory_changed = true;

      if (name == "dribble_position_rad") {
        dribble_position_rad_ = val;
      } else if (name == "open_position_rad") {
        open_position_rad_ = val;
      } else if (name == "feed_position_rad") {
        feed_position_rad_ = val;
      } else if (name == "open_duration_sec") {
        open_duration_sec_ = val;
      } else if (name == "feed_duration_sec") {
        feed_duration_sec_ = val;
      } else if (name == "opening_max_velocity_rad_s") {
        opening_max_velocity_rad_s_ = val;
      } else if (name == "feeding_max_velocity_rad_s") {
        feeding_max_velocity_rad_s_ = val;
      } else if (name == "returning_max_velocity_rad_s") {
        returning_max_velocity_rad_s_ = val;
      } else if (name == "belt_spinup_delay_sec") {belt_spinup_delay_sec_ = val;}
    }
  }

  if (trajectory_changed) {
    if (manual_transition_active_) {
      manual_transition_start_time_ = now();
      manual_transition_start_position_rad_ = last_position_command_rad_;
    }
    if (shot_cycle_active_) {
      shot_cycle_start_time_ = now();
      shot_cycle_start_position_rad_ = last_position_command_rad_;
    }
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
    if (position_mode_ == robot_msgs::msg::ArmPosition::OPEN) {
      max_vel_rad_s = opening_max_velocity_rad_s_;
    } else if (position_mode_ == robot_msgs::msg::ArmPosition::FEED) {
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
    if (belt_auto_started_) {
      robot_msgs::msg::BeltMode belt_msg;
      belt_msg.mode = shot_cycle_belt_spinup_level_;
      belt_mode_pub_->publish(belt_msg);
    }
    if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::BELT_SPINUP) {
      const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
      if (elapsed_sec >= belt_spinup_delay_sec_) {
        shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::OPENING;
        shot_cycle_start_time_ = now();
        shot_cycle_start_position_rad_ = last_position_command_rad_;
        position_mode_ = robot_msgs::msg::ArmPosition::OPEN;
        RCLCPP_INFO(get_logger(), "Shot Cycle: BELT_SPINUP -> OPEN");
      }
    } else {
      double phase_target_rad = dribble_position_rad_;
      double phase_max_vel_rad_s = returning_max_velocity_rad_s_;
      double hold_duration_sec = 0.0;

      switch (shot_cycle_phase_) {
        case robot_msgs::msg::ShotCycleState::OPENING:
          position_mode_ = robot_msgs::msg::ArmPosition::OPEN;
          phase_target_rad = open_position_rad_;
          phase_max_vel_rad_s = opening_max_velocity_rad_s_;
          hold_duration_sec = open_duration_sec_;
          break;
        case robot_msgs::msg::ShotCycleState::FEEDING:
          position_mode_ = robot_msgs::msg::ArmPosition::FEED;
          phase_target_rad = feed_position_rad_;
          phase_max_vel_rad_s = feeding_max_velocity_rad_s_;
          hold_duration_sec = feed_duration_sec_;
          break;
        case robot_msgs::msg::ShotCycleState::RETURNING:
          position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
          break;
        default:
          break;
      }

      const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
      const double move_duration_sec = transition_duration_sec(
        shot_cycle_start_position_rad_, phase_target_rad, phase_max_vel_rad_s);
      position_command_rad = interpolated_position_rad(
        shot_cycle_start_position_rad_, phase_target_rad, elapsed_sec, phase_max_vel_rad_s);

      if (elapsed_sec >= move_duration_sec + hold_duration_sec) {
        position_command_rad = phase_target_rad;
        shot_cycle_start_position_rad_ = phase_target_rad;
        shot_cycle_start_time_ = now();

        if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::OPENING) {
          shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::FEEDING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: OPEN -> FEED");
        } else if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING) {
          shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::RETURNING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: FEED -> DRIBBLE");
        } else {
          shot_cycle_active_ = false;
          if (belt_auto_started_) {
            belt_auto_started_ = false;
            robot_msgs::msg::BeltMode belt_msg;
            belt_msg.mode = robot_msgs::msg::BeltMode::STOP;
            belt_mode_pub_->publish(belt_msg);
            RCLCPP_INFO(get_logger(), "Shot Cycle Completed: Belt auto-stopped");
          }
          RCLCPP_INFO(get_logger(), "Shot Cycle Completed: Returned to DRIBBLE");
        }
      }
    }
  }

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
    case robot_msgs::msg::ShotCycleState::BELT_SPINUP:
      return dribble_on_rpm_;
    case robot_msgs::msg::ShotCycleState::OPENING:
      return shot_cycle_opening_rpm_;
    case robot_msgs::msg::ShotCycleState::FEEDING: {
        // OPENING 時の回転数から FEEDING 目標回転数 (0 RPM) へアーム動作に合わせて滑らかに減速
        const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
        const double move_duration_sec = transition_duration_sec(
          shot_cycle_start_position_rad_, feed_position_rad_, feeding_max_velocity_rad_s_);
        const double progress = move_duration_sec > 0.0 ?
          std::clamp(elapsed_sec / move_duration_sec, 0.0, 1.0) : 1.0;
        return static_cast<int>(
          shot_cycle_opening_rpm_ +
          (shot_cycle_feeding_rpm_ - shot_cycle_opening_rpm_) * progress);
      }
    case robot_msgs::msg::ShotCycleState::RETURNING:
      return shot_cycle_returning_rpm_;
  }
  return dribble_on_rpm_;
}

void DribbleControllerNode::publish_shot_cycle_state()
{
  robot_msgs::msg::ShotCycleState state;
  state.state = shot_cycle_active_ ? shot_cycle_phase_ : robot_msgs::msg::ShotCycleState::IDLE;
  shot_cycle_state_pub_->publish(state);
}

double DribbleControllerNode::target_position_rad() const
{
  switch (position_mode_) {
    case robot_msgs::msg::ArmPosition::OPEN:
      return open_position_rad_;
    case robot_msgs::msg::ArmPosition::FEED:
      return feed_position_rad_;
    default:
      return dribble_position_rad_;
  }
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
  // Septic (7th-order) smootherstep: Extremely smooth acceleration start ("gradual acceleration")
  // while preserving the specified peak velocity limit.
  const double smooth_progress =
    progress * progress * progress * progress *
    (35.0 + progress * (-84.0 + progress * (70.0 - 20.0 * progress)));
  return start_rad + (target_rad - start_rad) * smooth_progress;
}

double DribbleControllerNode::transition_duration_sec(
  double start_rad, double target_rad, double max_vel_rad_s) const
{
  // Septic smootherstep's peak derivative is 2.1875.
  return 2.1875 * std::abs(target_rad - start_rad) / max_vel_rad_s;
}
