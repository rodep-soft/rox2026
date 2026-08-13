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

  ball_detected_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/ball_detected", command_qos);

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

  spring_decel_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/spring_decel", command_qos,
    std::bind(&DribbleControllerNode::spring_decel_callback, this, std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", command_qos,
    std::bind(&DribbleControllerNode::shot_cycle_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", emergency_stop_qos,
    std::bind(&DribbleControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  opening_rpm_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/dribble/command_opening_rpm", command_qos,
    std::bind(&DribbleControllerNode::opening_rpm_callback, this, std::placeholders::_1));

  actuator_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
    "/edulite/state", command_qos,
    std::bind(&DribbleControllerNode::actuator_state_callback, this, std::placeholders::_1));

  vesc_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
    "/vesc/state", command_qos,
    std::bind(&DribbleControllerNode::vesc_state_callback, this, std::placeholders::_1));

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
  bottom_position_rad_ = declare_parameter<double>("bottom_position_rad", 0.0);
  feed_position_rad_ = declare_parameter<double>("feed_position_rad", 1.3);
  open_duration_sec_ = declare_parameter<double>("open_duration_sec", 0.3);
  feed_duration_sec_ = declare_parameter<double>("feed_duration_sec", 0.6);
  opening_max_velocity_rad_s_ = declare_parameter<double>("opening_max_velocity_rad_s", 4.0);
  feeding_max_velocity_rad_s_ = declare_parameter<double>("feeding_max_velocity_rad_s", 6.0);
  returning_max_velocity_rad_s_ = declare_parameter<double>("returning_max_velocity_rad_s", 4.0);
  opening_accel_factor_ = declare_parameter<double>("opening_accel_factor", 1.8);
  ball_detection_threshold_a_ = declare_parameter<double>("ball_detection_threshold_a", 3.5);
  dribble_on_rpm_ = declare_parameter<int>("dribble_on_rpm", 800);
  shot_cycle_opening_rpm_ = declare_parameter<int>("shot_cycle_opening_rpm", 800);
  shot_cycle_feeding_rpm_ = declare_parameter<int>("shot_cycle_feeding_rpm", 500);
  shot_cycle_returning_rpm_ = declare_parameter<int>("shot_cycle_returning_rpm", 800);
  shot_cycle_belt_spinup_level_ = static_cast<uint8_t>(
    declare_parameter<int>("shot_cycle_belt_spinup_level", 1));
  belt_spinup_delay_sec_ = declare_parameter<double>("belt_spinup_delay_sec", 0.5);
  const auto position_logical_id = declare_parameter<int>("position_logical_id", 5);
  const auto roller_logical_id = declare_parameter<int>("roller_logical_id", 12);
  const auto upper_belt_logical_id = declare_parameter<int>("upper_belt_logical_id", 10);
  const auto under_belt_logical_id = declare_parameter<int>("under_belt_logical_id", 11);

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
  upper_belt_logical_id_ = static_cast<uint16_t>(upper_belt_logical_id);
  under_belt_logical_id_ = static_cast<uint16_t>(under_belt_logical_id);
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

void DribbleControllerNode::spring_decel_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  spring_decel_active_ = msg->data;
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

void DribbleControllerNode::opening_rpm_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data >= 0 && msg->data <= 5600) {
    if (shot_cycle_opening_rpm_ != msg->data) {
      shot_cycle_opening_rpm_ = msg->data;
      RCLCPP_INFO(get_logger(), "Updated shot cycle opening RPM: %d RPM", shot_cycle_opening_rpm_);
    }
  }
}

void DribbleControllerNode::actuator_state_callback(
  const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  if (msg->logical_id == position_logical_id_) {
    current_arm_position_rad_ = msg->position;
  }
}

void DribbleControllerNode::vesc_state_callback(
  const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  if (msg->logical_id == upper_belt_logical_id_) {
    upper_belt_measured_rpm_ = msg->velocity;
    if (shot_cycle_active_ && shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING) {
      upper_belt_min_shot_rpm_ = std::min(upper_belt_min_shot_rpm_, std::abs(msg->velocity));
    }
  } else if (msg->logical_id == under_belt_logical_id_) {
    under_belt_measured_rpm_ = msg->velocity;
    if (shot_cycle_active_ && shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING) {
      under_belt_min_shot_rpm_ = std::min(under_belt_min_shot_rpm_, std::abs(msg->velocity));
    }
  }

  if (msg->logical_id == roller_logical_id_) {
    // 電流値によるボール保持判定 (ヒステリシス + 連続カウントによるディバウンスノイズフィルタ)
    const bool previous_has_ball = has_ball_;
    if (msg->current_a >= ball_detection_threshold_a_) {
      ball_detected_counter_++;
      ball_lost_counter_ = 0;
      if (ball_detected_counter_ >= ball_detection_debounce_count_) {
        has_ball_ = true;
      }
    } else if (msg->current_a <= 2.0) {
      ball_lost_counter_++;
      ball_detected_counter_ = 0;
      if (ball_lost_counter_ >= ball_detection_debounce_count_) {
        has_ball_ = false;
      }
    } else {
      ball_detected_counter_ = 0;
      ball_lost_counter_ = 0;
    }

    if (previous_has_ball != has_ball_) {
      if (has_ball_) {
        RCLCPP_INFO(
          get_logger(), ">>> BALL DETECTED (Current: %.2f A) <<<", msg->current_a);
      } else {
        RCLCPP_INFO(
          get_logger(), "--- BALL LOST (Current: %.2f A) ---", msg->current_a);
      }
    }

    std_msgs::msg::Bool ball_msg;
    ball_msg.data = has_ball_;
    ball_detected_pub_->publish(ball_msg);
  }
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
  const int target_rpm = roller_target_rpm();
  // 50ms 周期の制御タイマーごとに、最大 150 RPM ずつ滑らかに立ち上げ/減速する Ramp Filter
  constexpr int max_rpm_step = 150;
  if (current_filtered_roller_rpm_ < target_rpm) {
    current_filtered_roller_rpm_ = std::min(target_rpm, current_filtered_roller_rpm_ + max_rpm_step);
  } else if (current_filtered_roller_rpm_ > target_rpm) {
    current_filtered_roller_rpm_ = std::max(target_rpm, current_filtered_roller_rpm_ - max_rpm_step);
  }

  actuator_msgs::msg::ActuatorTarget roller_command;
  roller_command.logical_id = roller_logical_id_;
  roller_command.target = static_cast<float>(current_filtered_roller_rpm_);
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
        upper_belt_min_shot_rpm_ = 99999.0f;
        under_belt_min_shot_rpm_ = 99999.0f;
        RCLCPP_INFO(
          get_logger(),
          "Shot Cycle: BELT_SPINUP -> OPEN | Spinup Check (%.1fs) -> Upper Belt: %.1f RPM, Under Belt: %.1f RPM",
          belt_spinup_delay_sec_, upper_belt_measured_rpm_, under_belt_measured_rpm_);
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
      const double accel_factor = (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::OPENING) ?
        opening_accel_factor_ : 1.0;
      const double move_duration_sec = transition_duration_sec(
        shot_cycle_start_position_rad_, phase_target_rad, phase_max_vel_rad_s, accel_factor);
      position_command_rad = interpolated_position_rad(
        shot_cycle_start_position_rad_, phase_target_rad, elapsed_sec, phase_max_vel_rad_s,
        accel_factor);

      if (elapsed_sec >= move_duration_sec + hold_duration_sec) {
        position_command_rad = phase_target_rad;
        shot_cycle_start_position_rad_ = phase_target_rad;
        shot_cycle_start_time_ = now();

        if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::OPENING) {
          shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::FEEDING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: OPEN -> FEED");
        } else if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING) {
          shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::RETURNING;
          RCLCPP_INFO(
            get_logger(),
            "Shot Cycle: FEED -> DRIBBLE | Shot Impact Min Belt RPM -> Upper: %.1f RPM, Under: %.1f RPM",
            upper_belt_min_shot_rpm_ == 99999.0f ? 0.0f : upper_belt_min_shot_rpm_,
            under_belt_min_shot_rpm_ == 99999.0f ? 0.0f : under_belt_min_shot_rpm_);
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
  if (emergency_stop_active_) {
    return 0;
  }
  if (spring_decel_active_) {
    // ばね発射シーケンス中のみ：300 RPM へ滑らか減速してボール案内
    return spring_fire_dribble_rpm_;
  }
  if (!dribble_enabled_) {
    // 通常 OFF 時（何もしていない時）：当然 0 RPM（完全停止）
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
        // FEEDING 移動中、アーム角度が真下 (bottom_position_rad_) を通過するまでは回転数を 100% 維持
        // 真下を通過して逆側 (FEED) へ向かう間に 0 RPM へ滑らかに減速
        const double arm_pos = current_arm_position_rad_;
        if (arm_pos < bottom_position_rad_) {
          // まだ真下に到達していない（OPEN 姿勢から真下へ移動中）
          return shot_cycle_opening_rpm_;
        }
        // 真下を通過して逆側の FEED へ進行中 -> 0 RPM へ減速
        const double total_range = std::max(0.001, feed_position_rad_ - bottom_position_rad_);
        const double progress =
          std::clamp((arm_pos - bottom_position_rad_) / total_range, 0.0, 1.0);
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
  double max_vel_rad_s, double accel_factor) const
{
  const double duration_sec =
    transition_duration_sec(start_rad, target_rad, max_vel_rad_s, accel_factor);
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
  double start_rad, double target_rad, double max_vel_rad_s, double accel_factor) const
{
  // Septic smootherstep's peak derivative is 2.1875.
  return (2.1875 * accel_factor) * std::abs(target_rad - start_rad) / max_vel_rad_s;
}
