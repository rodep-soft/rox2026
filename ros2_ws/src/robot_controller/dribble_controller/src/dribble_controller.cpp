#include "dribble_controller/dribble_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace {
constexpr int kMaxRollerRpmStepPerTick = 150;
} // namespace

DribbleControllerNode::DribbleControllerNode()
    : Node("dribble_controller_node") {
  const auto position_target_topic = declare_parameter<std::string>(
      "position_target_topic", "/edulite/target");
  const auto roller_target_topic =
      declare_parameter<std::string>("roller_target_topic", "/vesc/target");
  const auto command_period_ms =
      declare_parameter<int>("command_period_ms", 20);
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  if (position_target_topic.empty() || roller_target_topic.empty()) {
    throw std::runtime_error("target topics must not be empty");
  }
  if (command_period_ms <= 0 || qos_depth <= 0) {
    throw std::runtime_error(
        "command_period_ms and qos_depth must be positive");
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
  belt_clearance_request_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/spring/belt_clearance_request",
      rclcpp::QoS(1).reliable().transient_local());

  ball_detected_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/dribble/ball_detected", rclcpp::QoS(1).reliable().transient_local());

  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>(
      "/belt/command_mode", command_qos);

  position_mode_sub_ = create_subscription<robot_msgs::msg::ArmPosition>(
      "/dribble/command_position", command_qos,
      std::bind(&DribbleControllerNode::position_mode_callback, this,
                std::placeholders::_1));

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dribble/command_enabled", command_qos,
      std::bind(&DribbleControllerNode::dribble_enabled_callback, this,
                std::placeholders::_1));

  dribble_reverse_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dribble/command_reverse", command_qos,
      std::bind(&DribbleControllerNode::dribble_reverse_callback, this,
                std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/dribble/shot_cycle_request", command_qos,
      std::bind(&DribbleControllerNode::shot_cycle_callback, this,
                std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/emergency_stop", emergency_stop_qos,
      std::bind(&DribbleControllerNode::emergency_stop_callback, this,
                std::placeholders::_1));

  opening_rpm_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/dribble/command_opening_rpm", command_qos,
      std::bind(&DribbleControllerNode::opening_rpm_callback, this,
                std::placeholders::_1));

  actuator_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
      "/edulite/state", command_qos,
      std::bind(&DribbleControllerNode::actuator_state_callback, this,
                std::placeholders::_1));

  vesc_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
      "/vesc/state", command_qos,
      std::bind(&DribbleControllerNode::vesc_state_callback, this,
                std::placeholders::_1));

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DribbleControllerNode::odometry_callback, this,
                std::placeholders::_1));
  spring_operation_state_sub_ =
      create_subscription<robot_msgs::msg::SpringOperationState>(
          "/spring/operation_state",
          rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&DribbleControllerNode::spring_operation_state_callback,
                    this, std::placeholders::_1));

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms),
      std::bind(&DribbleControllerNode::control_timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &DribbleControllerNode::parameter_callback, this, std::placeholders::_1));
}

void DribbleControllerNode::position_mode_callback(
    const robot_msgs::msg::ArmPosition::SharedPtr msg) {
  if (emergency_stop_active_ ||
      (msg->position != robot_msgs::msg::ArmPosition::DRIBBLE &&
       msg->position != robot_msgs::msg::ArmPosition::OPEN)) {
    return;
  }

  const uint8_t target_mode = msg->position;

  if (target_mode != position_mode_ || shot_cycle_active_) {
    shot_cycle_active_ = false;
    publish_belt_clearance_request(false);
    manual_transition_active_ = true;
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_position_command_rad_;
    manual_transition_start_rpm_ = current_filtered_roller_rpm_;
    position_mode_ = target_mode;
  }
}

void DribbleControllerNode::dribble_enabled_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  dribble_enabled_ = msg->data;
}

void DribbleControllerNode::dribble_reverse_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (emergency_stop_active_) {
    return;
  }

  if (msg->data != dribble_reverse_enabled_) {
    dribble_reverse_enabled_ = msg->data;
    reverse_transition_active_ = true;
    reverse_transition_start_time_ = now();
    reverse_transition_start_rpm_ = current_filtered_roller_rpm_;
  }
}

void DribbleControllerNode::shot_cycle_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || emergency_stop_active_ || shot_cycle_active_ ||
      shot_prepare_from_open_) {
    return;
  }

  if (position_mode_ == robot_msgs::msg::ArmPosition::OPEN) {
    position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
    manual_transition_active_ = true;
    manual_transition_start_time_ = now();
    manual_transition_start_position_rad_ = last_position_command_rad_;
    manual_transition_start_rpm_ = current_filtered_roller_rpm_;
    shot_prepare_from_open_ = true;
    shot_prepare_delay_started_ = false;
    RCLCPP_INFO(get_logger(),
                "Belt shot requested from OPEN: returning to DRIBBLE first");
    return;
  }
  start_shot_cycle();
}

void DribbleControllerNode::publish_belt_clearance_request(bool requested) {
  std_msgs::msg::Bool request;
  request.data = requested;
  belt_clearance_request_pub_->publish(request);
}

void DribbleControllerNode::start_shot_cycle() {
  RCLCPP_INFO(get_logger(), "Starting belt spin-up at DRIBBLE posture");
  manual_transition_active_ = false;
  shot_cycle_active_ = true;
  shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::BELT_SPINUP;
  shot_cycle_start_time_ = now();
  shot_cycle_start_position_rad_ = last_position_command_rad_;
  position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;

  constexpr float stopped_threshold_rpm = 100.0f;
  const bool belt_is_stopped =
      std::abs(upper_belt_measured_rpm_) < stopped_threshold_rpm &&
      std::abs(under_belt_measured_rpm_) < stopped_threshold_rpm;
  belt_auto_started_ = belt_is_stopped;
  if (belt_auto_started_) {
    robot_msgs::msg::BeltMode belt_msg;
    belt_msg.mode = shot_cycle_belt_spinup_level_;
    belt_mode_pub_->publish(belt_msg);
  }
}

void DribbleControllerNode::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  const bool initial_release = startup_waiting_for_emergency_release_ &&
                               startup_emergency_seen_active_ && !msg->data;
  if (msg->data == emergency_stop_active_ && !initial_release) {
    return;
  }

  emergency_stop_active_ = msg->data;

  if (emergency_stop_active_) {
    startup_emergency_seen_active_ = true;
    emergency_hold_position_rad_ = arm_state_received_
                                       ? current_arm_position_rad_
                                       : last_position_command_rad_;
    last_position_command_rad_ = emergency_hold_position_rad_;
    current_filtered_roller_rpm_ = 0;
    RCLCPP_WARN(get_logger(),
                "Emergency stop activated: holding dribble arm at %.3f rad",
                emergency_hold_position_rad_);
  } else {
    if (startup_waiting_for_emergency_release_) {
      startup_waiting_for_emergency_release_ = false;
      shot_cycle_active_ = false;
      position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
      RCLCPP_INFO(get_logger(),
                  "Initial emergency stop released: moving to DRIBBLE posture");
    }

    const double resume_position_rad = arm_state_received_
                                           ? current_arm_position_rad_
                                           : emergency_hold_position_rad_;

    // Preserve the pre-stop mode and shot-cycle phase. Restart interpolation
    // from the measured position so that releasing emergency stop cannot apply
    // a discontinuous target.
    if (shot_cycle_active_) {
      shot_cycle_start_position_rad_ = resume_position_rad;
      shot_cycle_start_time_ = now();
    } else {
      manual_transition_active_ = true;
      manual_transition_start_position_rad_ = resume_position_rad;
      manual_transition_start_time_ = now();
      manual_transition_start_rpm_ = 0;
    }
    if (dribble_reverse_enabled_) {
      reverse_transition_active_ = true;
      reverse_transition_start_time_ = now();
      reverse_transition_start_rpm_ = 0;
    }
    last_position_command_rad_ = resume_position_rad;
    RCLCPP_INFO(
        get_logger(),
        "Emergency stop released: resuming dribble operation from %.3f rad",
        resume_position_rad);
  }
  control_timer_callback();
}

void DribbleControllerNode::opening_rpm_callback(
    const std_msgs::msg::Int32::SharedPtr msg) {
  if (msg->data >= 0 && msg->data <= 5600) {
    if (shot_cycle_opening_rpm_ != msg->data) {
      shot_cycle_opening_rpm_ = msg->data;
      RCLCPP_INFO(get_logger(), "Updated shot cycle opening RPM: %d RPM",
                  shot_cycle_opening_rpm_);
    }
  }
}

void DribbleControllerNode::actuator_state_callback(
    const actuator_msgs::msg::ActuatorState::SharedPtr msg) {
  if (msg->logical_id != position_logical_id_) {
    return;
  }

  const bool ready =
      msg->state == actuator_msgs::msg::ActuatorState::STATE_READY;
  if (!ready) {
    if (arm_actuator_ready_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Dribble EduLite disconnected; pausing position commands");
    }
    last_published_position_rad_.reset();
    return;
  }

  const bool reconnected = arm_state_received_ && !arm_actuator_ready_;
  arm_actuator_ready_ = true;
  current_arm_position_rad_ = msg->position;

  if (!arm_state_received_) {
    arm_state_received_ = true;
    last_position_command_rad_ = msg->position;
    emergency_hold_position_rad_ = msg->position;
    position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
    if (!startup_waiting_for_emergency_release_) {
      manual_transition_active_ = true;
      manual_transition_start_time_ = now();
      manual_transition_start_position_rad_ = msg->position;
      manual_transition_start_rpm_ = 0;
    }
    RCLCPP_INFO(get_logger(),
                "Arm initial position received: %.3f rad; waiting for "
                "emergency release: %s",
                msg->position,
                startup_waiting_for_emergency_release_ ? "yes" : "no");
  } else if (reconnected) {
    last_position_command_rad_ = msg->position;
    emergency_hold_position_rad_ = msg->position;
    if (shot_cycle_active_) {
      shot_cycle_start_position_rad_ = msg->position;
      shot_cycle_start_time_ = now();
    } else {
      manual_transition_active_ = true;
      manual_transition_start_position_rad_ = msg->position;
      manual_transition_start_time_ = now();
      manual_transition_start_rpm_ = current_filtered_roller_rpm_;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Dribble EduLite reconnected at %.3f rad; resuming smoothly",
        msg->position);
  }
}

void DribbleControllerNode::vesc_state_callback(
    const actuator_msgs::msg::ActuatorState::SharedPtr msg) {
  if (msg->logical_id == upper_belt_logical_id_) {
    upper_belt_measured_rpm_ = msg->velocity;
  } else if (msg->logical_id == under_belt_logical_id_) {
    under_belt_measured_rpm_ = msg->velocity;
  }

  if (msg->logical_id == roller_logical_id_) {
    // ローラー電流値に一次ローパスフィルタを適用 (最新値係数:
    // current_lpf_alpha_)
    if (!roller_current_initialized_) {
      filtered_roller_current_a_ = msg->current_a;
      roller_current_initialized_ = true;
    } else {
      filtered_roller_current_a_ =
          current_lpf_alpha_ * msg->current_a +
          (1.0 - current_lpf_alpha_) * filtered_roller_current_a_;
    }

    // 電流値によるボール保持判定 (ヒステリシス +
    // 連続カウントによるディバウンスノイズフィルタ)
    const bool previous_has_ball = has_ball_;
    if (filtered_roller_current_a_ >= ball_detection_threshold_a_) {
      ball_detected_counter_++;
      ball_lost_counter_ = 0;
      if (ball_detected_counter_ >= ball_detection_debounce_count_) {
        has_ball_ = true;
      }
    } else if (filtered_roller_current_a_ <= ball_lost_threshold_a_) {
      ball_lost_counter_++;
      ball_detected_counter_ = 0;
      if (ball_lost_counter_ >= ball_lost_debounce_count_) {
        has_ball_ = false;
      }
    } else {
      ball_detected_counter_ = 0;
      ball_lost_counter_ = 0;
    }

    if (previous_has_ball != has_ball_) {
      if (has_ball_) {
        RCLCPP_INFO(get_logger(),
                    ">>> BALL DETECTED (Current: %.2f A, Filtered: %.2f A) <<<",
                    msg->current_a, filtered_roller_current_a_);
      } else {
        RCLCPP_INFO(get_logger(),
                    "--- BALL LOST (Current: %.2f A, Filtered: %.2f A) ---",
                    msg->current_a, filtered_roller_current_a_);
      }
    }

    if (!last_published_ball_state_.has_value() ||
        *last_published_ball_state_ != has_ball_) {
      std_msgs::msg::Bool ball_msg;
      ball_msg.data = has_ball_;
      ball_detected_pub_->publish(ball_msg);
      last_published_ball_state_ = has_ball_;
    }
  }
}

void DribbleControllerNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto current_time = now();
  const double current_vx = msg->twist.twist.linear.x;
  if (last_odometry_time_.nanoseconds() > 0) {
    const double dt = (current_time - last_odometry_time_).seconds();
    if (dt > 0.001 && dt < 0.5) {
      const double raw_acceleration = (current_vx - last_measured_vx_m_s_) / dt;
      measured_ax_m_s2_ =
          measured_acceleration_lpf_alpha_ * raw_acceleration +
          (1.0 - measured_acceleration_lpf_alpha_) * measured_ax_m_s2_;
    }
  }
  measured_vx_m_s_ = current_vx;
  last_measured_vx_m_s_ = current_vx;
  last_odometry_time_ = current_time;
}

void DribbleControllerNode::spring_operation_state_callback(
    const robot_msgs::msg::SpringOperationState::SharedPtr msg) {
  const bool was_slow = spring_operation_state_ ==
                        robot_msgs::msg::SpringOperationState::SLOW_FIRE;
  spring_operation_state_ = msg->state;
  const bool is_slow = spring_operation_state_ ==
                       robot_msgs::msg::SpringOperationState::SLOW_FIRE;
  if (was_slow == is_slow || shot_cycle_active_) {
    return;
  }
  manual_transition_active_ = true;
  manual_transition_start_time_ = now();
  manual_transition_start_position_rad_ = last_position_command_rad_;
  manual_transition_start_rpm_ = current_filtered_roller_rpm_;
}

void DribbleControllerNode::update_motion_compensation() {
  const bool odometry_fresh =
      last_odometry_time_.nanoseconds() > 0 &&
      (now() - last_odometry_time_).seconds() <= odometry_timeout_sec_;
  if (!enable_motion_compensation_ || emergency_stop_active_ ||
      !odometry_fresh) {
    current_motion_boost_rpm_ = 0;
    return;
  }

  const double backward_speed = std::max(0.0, -measured_vx_m_s_);
  const double backward_acceleration = std::max(0.0, -measured_ax_m_s2_);
  const double raw_boost =
      backward_speed * backward_velocity_boost_rpm_per_mps_ +
      backward_acceleration * backward_acceleration_rpm_per_mps2_;
  current_motion_boost_rpm_ =
      std::min(max_boost_rpm_, static_cast<int>(std::round(raw_boost)));
}

void DribbleControllerNode::update_and_publish_roller_command() {
  const int target_rpm = roller_target_rpm();
  if (emergency_stop_active_) {
    current_filtered_roller_rpm_ = 0;
    reverse_transition_active_ = false;
  } else if (reverse_transition_active_) {
    const double elapsed_sec =
        (now() - reverse_transition_start_time_).seconds();
    const double ramp_duration = std::max(0.001, dribble_reverse_ramp_sec_);
    const double progress = std::clamp(elapsed_sec / ramp_duration, 0.0, 1.0);
    current_filtered_roller_rpm_ = static_cast<int>(
        std::round(reverse_transition_start_rpm_ +
                   (target_rpm - reverse_transition_start_rpm_) * progress));
    if (elapsed_sec >= ramp_duration) {
      reverse_transition_active_ = false;
      current_filtered_roller_rpm_ = target_rpm;
    }
  } else if (manual_transition_active_ && !shot_cycle_active_ &&
             dribble_enabled_ &&
             spring_operation_state_ !=
                 robot_msgs::msg::SpringOperationState::SLOW_FIRE) {
    const double mode_target_rad = target_position_rad();
    const double max_vel_rad_s = manual_transition_max_velocity_rad_s();
    const double max_acceleration_rad_s2 =
        manual_transition_max_acceleration_rad_s2();

    const double elapsed_sec =
        (now() - manual_transition_start_time_).seconds();
    const auto trajectory = sample_trajectory(
        manual_transition_start_position_rad_, mode_target_rad, elapsed_sec,
        max_vel_rad_s, max_acceleration_rad_s2);
    if (trajectory.duration_sec > 0.0) {
      const double progress =
          std::clamp(elapsed_sec / trajectory.duration_sec, 0.0, 1.0);
      const double smooth_progress =
          progress * progress * progress * progress *
          (35.0 + progress * (-84.0 + progress * (70.0 - 20.0 * progress)));
      current_filtered_roller_rpm_ = static_cast<int>(std::round(
          manual_transition_start_rpm_ +
          (target_rpm - manual_transition_start_rpm_) * smooth_progress));
    } else {
      current_filtered_roller_rpm_ = target_rpm;
    }
  } else {
    // Limit the RPM change on every control tick.
    if (current_filtered_roller_rpm_ < target_rpm) {
      current_filtered_roller_rpm_ = std::min(
          target_rpm, current_filtered_roller_rpm_ + kMaxRollerRpmStepPerTick);
    } else if (current_filtered_roller_rpm_ > target_rpm) {
      current_filtered_roller_rpm_ = std::max(
          target_rpm, current_filtered_roller_rpm_ - kMaxRollerRpmStepPerTick);
    }
  }

  if (last_published_roller_rpm_.has_value() &&
      *last_published_roller_rpm_ == current_filtered_roller_rpm_) {
    return;
  }

  actuator_msgs::msg::ActuatorTarget roller_command;
  roller_command.logical_id = roller_logical_id_;
  roller_command.target = static_cast<float>(current_filtered_roller_rpm_);
  roller_command_pub_->publish(roller_command);
  last_published_roller_rpm_ = current_filtered_roller_rpm_;
}

double DribbleControllerNode::update_manual_position_command(
    double position_command_rad) {
  if (!manual_transition_active_ || shot_cycle_active_) {
    return position_command_rad;
  }

  const double mode_target_rad = target_position_rad();
  const double max_vel_rad_s = manual_transition_max_velocity_rad_s();
  const double max_acceleration_rad_s2 =
      manual_transition_max_acceleration_rad_s2();
  const double elapsed_sec = (now() - manual_transition_start_time_).seconds();
  const auto trajectory =
      sample_trajectory(manual_transition_start_position_rad_, mode_target_rad,
                        elapsed_sec, max_vel_rad_s, max_acceleration_rad_s2);
  position_command_rad = trajectory.position_rad;
  if (elapsed_sec >= trajectory.duration_sec) {
    manual_transition_active_ = false;
    position_command_rad = mode_target_rad;
  }
  return position_command_rad;
}
void DribbleControllerNode::publish_position_command(double position_rad) {
  if (last_published_position_rad_.has_value() &&
      std::fabs(*last_published_position_rad_ - position_rad) <= 1e-6) {
    last_position_command_rad_ = position_rad;
    return;
  }
  last_published_position_rad_ = position_rad;
  actuator_msgs::msg::ActuatorTarget position_command;
  position_command.logical_id = position_logical_id_;
  position_command.target = static_cast<float>(position_rad);
  last_position_command_rad_ = position_rad;
  position_command_pub_->publish(position_command);
}

// ────────────────────────────────────────────────────────────────────────────
// タイマーコールバック（ステートマシン進行 + publish）
// ────────────────────────────────────────────────────────────────────────────

void DribbleControllerNode::control_timer_callback() {
  publish_shot_cycle_state();
  update_motion_compensation();
  update_and_publish_roller_command();

  if (!arm_actuator_ready_) {
    return;
  }

  if (startup_waiting_for_emergency_release_) {
    // Keep the arm at the first measured position until the initial
    // emergency-stop release. Periodic targets keep the EduLite in READY
    // without moving toward DRIBBLE prematurely.
    publish_position_command(emergency_hold_position_rad_);
    return;
  }

  if (emergency_stop_active_) {
    publish_position_command(emergency_hold_position_rad_);
    return;
  }

  if (shot_prepare_from_open_) {
    if (!manual_transition_active_) {
      if (!shot_prepare_delay_started_) {
        shot_prepare_delay_started_ = true;
        shot_prepare_start_time_ = now();
      } else if ((now() - shot_prepare_start_time_).seconds() >=
                 prepare_from_open_delay_sec_) {
        shot_prepare_from_open_ = false;
        shot_prepare_delay_started_ = false;
        start_shot_cycle();
      }
    }
  }

  double position_command_rad = target_position_rad();
  position_command_rad = update_manual_position_command(position_command_rad);

  if (shot_cycle_active_) {
    if (belt_auto_started_) {
      robot_msgs::msg::BeltMode belt_msg;
      belt_msg.mode = shot_cycle_belt_spinup_level_;
      belt_mode_pub_->publish(belt_msg);
    }
    if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::BELT_SPINUP) {
      const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
      const double required_rpm =
          std::abs(shot_cycle_opening_rpm_) * belt_ready_ratio_;
      const bool roller_ready = std::abs(roller_measured_rpm_) >= required_rpm;
      const bool minimum_delay_elapsed =
          elapsed_sec >= belt_spinup_min_delay_sec_;
      if ((roller_ready && minimum_delay_elapsed) ||
          elapsed_sec >= belt_spinup_delay_sec_) {
        publish_belt_clearance_request(true);
        shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::FEEDING;
        shot_cycle_start_time_ = now();
        shot_cycle_start_position_rad_ = last_position_command_rad_;
        position_mode_ = robot_msgs::msg::ArmPosition::FEED;
        RCLCPP_INFO(
            get_logger(),
            "Shot Cycle: BELT_SPINUP -> FEED (roller %.1f / required %.1f RPM)",
            roller_measured_rpm_, required_rpm);
      }
    } else {
      const uint8_t return_mode = robot_msgs::msg::ArmPosition::DRIBBLE;
      const double return_target_rad = dribble_position_rad_;

      double phase_target_rad = return_target_rad;
      double phase_max_vel_rad_s = returning_max_velocity_rad_s_;
      double hold_duration_sec = 0.0;

      switch (shot_cycle_phase_) {
      case robot_msgs::msg::ShotCycleState::FEEDING:
        position_mode_ = robot_msgs::msg::ArmPosition::FEED;
        phase_target_rad = feed_position_rad_;
        phase_max_vel_rad_s = feeding_max_velocity_rad_s_;
        hold_duration_sec = feed_duration_sec_;
        break;
      case robot_msgs::msg::ShotCycleState::RETURNING:
        position_mode_ = return_mode;
        phase_target_rad = return_target_rad;
        phase_max_vel_rad_s = returning_max_velocity_rad_s_;
        break;
      default:
        break;
      }

      const double elapsed_sec = (now() - shot_cycle_start_time_).seconds();
      double phase_max_acceleration_rad_s2 = returning_max_acceleration_rad_s2_;
      if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING) {
        phase_max_acceleration_rad_s2 = feeding_max_acceleration_rad_s2_;
      }
      const auto trajectory = sample_trajectory(
          shot_cycle_start_position_rad_, phase_target_rad, elapsed_sec,
          phase_max_vel_rad_s, phase_max_acceleration_rad_s2);
      position_command_rad = trajectory.position_rad;

      if (elapsed_sec >= trajectory.duration_sec + hold_duration_sec) {
        position_command_rad = phase_target_rad;
        shot_cycle_start_position_rad_ = phase_target_rad;
        shot_cycle_start_time_ = now();

        if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING) {
          shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::RETURNING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: FEED -> RETURNING");
        } else {
          shot_cycle_active_ = false;
          publish_belt_clearance_request(false);
          has_ball_ = false;
          ball_detected_counter_ = 0;
          ball_lost_counter_ = ball_detection_debounce_count_;
          position_mode_ = return_mode;
          if (belt_auto_started_) {
            belt_auto_started_ = false;
            robot_msgs::msg::BeltMode belt_msg;
            belt_msg.mode = robot_msgs::msg::BeltMode::STOP;
            belt_mode_pub_->publish(belt_msg);
            RCLCPP_INFO(get_logger(),
                        "Shot Cycle Completed: Belt auto-stopped");
          }
          RCLCPP_INFO(get_logger(),
                      "Shot Cycle Completed: Returned to DRIBBLE");
        }
      }
    }
  }

  publish_position_command(position_command_rad);
}

int DribbleControllerNode::roller_target_rpm() const {
  if (emergency_stop_active_) {
    return 0;
  }
  if (spring_operation_state_ ==
      robot_msgs::msg::SpringOperationState::SLOW_FIRE) {
    return slow_fire_dribble_rpm_;
  }
  if (dribble_reverse_enabled_) {
    // 逆回転モード: 一定の逆回転RPMを出力
    return -std::abs(dribble_reverse_rpm_);
  }
  if (shot_cycle_active_) {
    switch (shot_cycle_phase_) {
    case robot_msgs::msg::ShotCycleState::BELT_SPINUP:
      return shot_cycle_opening_rpm_;
    case robot_msgs::msg::ShotCycleState::FEEDING: {
      // FEEDING 移動中、アーム角度が真下 (bottom_position_rad_)
      // を通過するまでは回転数を 100% 維持 真下を通過して逆側 (FEED)
      // へ向かう間に 0 RPM へ滑らかに減速
      const double arm_pos = current_arm_position_rad_;
      if (arm_pos < bottom_position_rad_) {
        // まだ真下に到達していない
        return shot_cycle_opening_rpm_;
      }
      // 真下を通過して逆側の FEED へ進行中 -> 0 RPM へ減速
      const double total_range =
          std::max(0.001, feed_position_rad_ - bottom_position_rad_);
      const double progress =
          std::clamp((arm_pos - bottom_position_rad_) / total_range, 0.0, 1.0);
      return static_cast<int>(
          shot_cycle_opening_rpm_ +
          (shot_cycle_feeding_rpm_ - shot_cycle_opening_rpm_) * progress);
    }
    case robot_msgs::msg::ShotCycleState::RETURNING:
      return shot_cycle_returning_rpm_;
    }
  }

  if (!dribble_enabled_) {
    // 通常 OFF 時（何もしていない時）：当然 0 RPM（完全停止）
    return 0;
  }

  // ボールを出すとき (OPEN姿勢) はローラー回転を 0 RPM にする
  if (position_mode_ == robot_msgs::msg::ArmPosition::OPEN) {
    return 0;
  }

  int base_rpm = dribble_on_rpm_;

  // 運動補正ブーストを加算 (後退・急減速時の慣性力対策)
  if (enable_motion_compensation_ && current_motion_boost_rpm_ > 0) {
    return std::min(4000, base_rpm + current_motion_boost_rpm_);
  }

  return base_rpm;
}

void DribbleControllerNode::publish_shot_cycle_state() {
  const uint8_t current_state = shot_cycle_active_
                                    ? shot_cycle_phase_
                                    : robot_msgs::msg::ShotCycleState::IDLE;
  if (current_state == last_published_shot_cycle_state_) {
    return;
  }

  robot_msgs::msg::ShotCycleState state;
  state.state = current_state;
  shot_cycle_state_pub_->publish(state);
  last_published_shot_cycle_state_ = current_state;
}

double DribbleControllerNode::target_position_rad() const {
  if (spring_operation_state_ ==
          robot_msgs::msg::SpringOperationState::SLOW_FIRE &&
      !shot_cycle_active_) {
    return slow_fire_dribble_position_rad_;
  }
  switch (position_mode_) {
  case robot_msgs::msg::ArmPosition::OPEN:
    return open_position_rad_;
  case robot_msgs::msg::ArmPosition::FEED:
    return feed_position_rad_;
  case robot_msgs::msg::ArmPosition::DRIBBLE:
  default:
    return dribble_position_rad_;
  }
}

double DribbleControllerNode::manual_transition_max_velocity_rad_s() const {
  switch (position_mode_) {
  case robot_msgs::msg::ArmPosition::OPEN:
    return opening_max_velocity_rad_s_;
  case robot_msgs::msg::ArmPosition::FEED:
    return feeding_max_velocity_rad_s_;
  case robot_msgs::msg::ArmPosition::DRIBBLE:
    return dribbling_max_velocity_rad_s_;
  default:
    return returning_max_velocity_rad_s_;
  }
}

double
DribbleControllerNode::manual_transition_max_acceleration_rad_s2() const {
  if (spring_operation_state_ ==
      robot_msgs::msg::SpringOperationState::SLOW_FIRE) {
    return dribbling_max_acceleration_rad_s2_;
  }
  switch (position_mode_) {
  case robot_msgs::msg::ArmPosition::OPEN:
    return opening_max_acceleration_rad_s2_;
  case robot_msgs::msg::ArmPosition::FEED:
    return feeding_max_acceleration_rad_s2_;
  case robot_msgs::msg::ArmPosition::DRIBBLE:
  default:
    return dribbling_max_acceleration_rad_s2_;
  }
}

DribbleControllerNode::TrajectorySample
DribbleControllerNode::sample_trajectory(double start_rad, double target_rad,
                                         double elapsed_sec,
                                         double max_velocity_rad_s,
                                         double max_acceleration_rad_s2) const {
  const double distance = std::abs(target_rad - start_rad);
  if (distance <= 1e-9) {
    return {target_rad, 0.0};
  }
  const double acceleration = std::max(1e-6, max_acceleration_rad_s2);
  const double velocity = std::max(1e-6, max_velocity_rad_s);
  const double acceleration_time =
      std::min(velocity / acceleration, std::sqrt(distance / acceleration));
  const double peak_velocity = acceleration * acceleration_time;
  const double acceleration_distance =
      0.5 * acceleration * acceleration_time * acceleration_time;
  const double cruise_time =
      std::max(0.0, (distance - 2.0 * acceleration_distance) / peak_velocity);
  const double duration = 2.0 * acceleration_time + cruise_time;
  const double time = std::clamp(elapsed_sec, 0.0, duration);

  double traveled = 0.0;
  if (time < acceleration_time) {
    traveled = 0.5 * acceleration * time * time;
  } else if (time < acceleration_time + cruise_time) {
    traveled =
        acceleration_distance + peak_velocity * (time - acceleration_time);
  } else {
    const double remaining = duration - time;
    traveled = distance - 0.5 * acceleration * remaining * remaining;
  }
  return {start_rad + std::copysign(traveled, target_rad - start_rad),
          duration};
}

void DribbleControllerNode::load_parameters() {
  const auto load_double = [this](const char *name, double &value) {
    value = declare_parameter<double>(name, value);
  };
  load_double("dribble_position_rad", dribble_position_rad_);
  load_double("open_position_rad", open_position_rad_);
  load_double("bottom_position_rad", bottom_position_rad_);
  load_double("feed_position_rad", feed_position_rad_);
  load_double("slow_fire_dribble_position_rad",
              slow_fire_dribble_position_rad_);
  load_double("feed_duration_sec", feed_duration_sec_);
  load_double("belt_spinup_timeout_sec", belt_spinup_delay_sec_);
  load_double("belt_spinup_min_delay_sec", belt_spinup_min_delay_sec_);
  load_double("prepare_from_open_delay_sec", prepare_from_open_delay_sec_);
  load_double("belt_ready_ratio", belt_ready_ratio_);
  load_double("opening_max_velocity_rad_s", opening_max_velocity_rad_s_);
  load_double("feeding_max_velocity_rad_s", feeding_max_velocity_rad_s_);
  load_double("returning_max_velocity_rad_s", returning_max_velocity_rad_s_);
  load_double("dribbling_max_velocity_rad_s", dribbling_max_velocity_rad_s_);
  load_double("opening_max_acceleration_rad_s2",
              opening_max_acceleration_rad_s2_);
  load_double("feeding_max_acceleration_rad_s2",
              feeding_max_acceleration_rad_s2_);
  load_double("returning_max_acceleration_rad_s2",
              returning_max_acceleration_rad_s2_);
  load_double("dribbling_max_acceleration_rad_s2",
              dribbling_max_acceleration_rad_s2_);
  load_double("dribble_reverse_ramp_sec", dribble_reverse_ramp_sec_);
  load_double("ball_detection_threshold_a", ball_detection_threshold_a_);
  load_double("ball_lost_threshold_a", ball_lost_threshold_a_);
  load_double("current_lpf_alpha", current_lpf_alpha_);
  load_double("odometry_timeout_sec", odometry_timeout_sec_);
  load_double("measured_acceleration_lpf_alpha",
              measured_acceleration_lpf_alpha_);
  load_double("backward_velocity_boost_rpm_per_mps",
              backward_velocity_boost_rpm_per_mps_);
  load_double("backward_acceleration_rpm_per_mps2",
              backward_acceleration_rpm_per_mps2_);

  const auto load_int = [this](const char *name, int &value) {
    value = declare_parameter<int>(name, value);
  };
  load_int("dribble_on_rpm", dribble_on_rpm_);
  load_int("slow_fire_dribble_rpm", slow_fire_dribble_rpm_);
  load_int("dribble_reverse_rpm", dribble_reverse_rpm_);
  load_int("shot_cycle_opening_rpm", shot_cycle_opening_rpm_);
  load_int("shot_cycle_feeding_rpm", shot_cycle_feeding_rpm_);
  load_int("shot_cycle_returning_rpm", shot_cycle_returning_rpm_);
  load_int("ball_detection_debounce_count", ball_detection_debounce_count_);
  load_int("ball_lost_debounce_count", ball_lost_debounce_count_);
  load_int("max_boost_rpm", max_boost_rpm_);

  shot_cycle_belt_spinup_level_ = static_cast<uint8_t>(declare_parameter<int>(
      "shot_cycle_belt_spinup_level", shot_cycle_belt_spinup_level_));
  odometry_topic_ =
      declare_parameter<std::string>("odometry_topic", odometry_topic_);
  enable_motion_compensation_ = declare_parameter<bool>(
      "enable_motion_compensation", enable_motion_compensation_);

  const auto load_id = [this](const char *name, uint16_t default_value) {
    const int value = declare_parameter<int>(name, default_value);
    if (value < 0 || value > 65535) {
      throw std::runtime_error(std::string(name) + " must be in [0, 65535]");
    }
    return static_cast<uint16_t>(value);
  };
  position_logical_id_ = load_id("position_logical_id", position_logical_id_);
  roller_logical_id_ = load_id("roller_logical_id", roller_logical_id_);
  upper_belt_logical_id_ =
      load_id("upper_belt_logical_id", upper_belt_logical_id_);
  under_belt_logical_id_ =
      load_id("under_belt_logical_id", under_belt_logical_id_);

  if (shot_cycle_belt_spinup_level_ < 1 || shot_cycle_belt_spinup_level_ > 4 ||
      ball_detection_debounce_count_ < 1 || ball_lost_debounce_count_ < 1) {
    throw std::runtime_error(
        "integer parameters are outside their valid ranges");
  }
  const std::vector<double> positive_values{opening_max_velocity_rad_s_,
                                            feeding_max_velocity_rad_s_,
                                            returning_max_velocity_rad_s_,
                                            dribbling_max_velocity_rad_s_,
                                            opening_max_acceleration_rad_s2_,
                                            feeding_max_acceleration_rad_s2_,
                                            returning_max_acceleration_rad_s2_,
                                            dribbling_max_acceleration_rad_s2_,
                                            odometry_timeout_sec_};
  if (std::any_of(
          positive_values.begin(), positive_values.end(),
          [](double value) { return !std::isfinite(value) || value <= 0.0; })) {
    throw std::runtime_error(
        "velocity, acceleration and timeout parameters must be positive");
  }
  if (current_lpf_alpha_ <= 0.0 || current_lpf_alpha_ > 1.0 ||
      measured_acceleration_lpf_alpha_ <= 0.0 ||
      measured_acceleration_lpf_alpha_ > 1.0 || belt_ready_ratio_ <= 0.0 ||
      belt_ready_ratio_ > 1.0) {
    throw std::runtime_error(
        "filter alpha and belt_ready_ratio must be in (0, 1]");
  }

  position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
  last_position_command_rad_ = dribble_position_rad_;
}

rcl_interfaces::msg::SetParametersResult
DribbleControllerNode::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool trajectory_changed = false;
  const std::vector<std::string> restart_parameters{
      "command_period_ms",     "qos_depth",
      "position_logical_id",   "roller_logical_id",
      "upper_belt_logical_id", "under_belt_logical_id",
      "position_target_topic", "roller_target_topic",
      "odometry_topic"};

  for (const auto &parameter : parameters) {
    const auto &name = parameter.get_name();
    if (std::find(restart_parameters.begin(), restart_parameters.end(), name) !=
        restart_parameters.end()) {
      result.successful = false;
      result.reason = name + " requires a node restart";
      return result;
    }

    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "enable_motion_compensation") {
        enable_motion_compensation_ = parameter.as_bool();
      }
      continue;
    }

    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      const int value = static_cast<int>(parameter.as_int());
      const bool positive_count = name == "ball_detection_debounce_count" ||
                                  name == "ball_lost_debounce_count";
      if (value < 0 || (positive_count && value == 0) ||
          (name == "shot_cycle_belt_spinup_level" &&
           (value < 1 || value > 4))) {
        result.successful = false;
        result.reason = name + " is outside its valid range";
        return result;
      }
      if (name == "shot_cycle_belt_spinup_level") {
        shot_cycle_belt_spinup_level_ = static_cast<uint8_t>(value);
        continue;
      }
      const std::pair<const char *, int *> integer_parameters[] = {
          {"dribble_on_rpm", &dribble_on_rpm_},
          {"slow_fire_dribble_rpm", &slow_fire_dribble_rpm_},
          {"dribble_reverse_rpm", &dribble_reverse_rpm_},
          {"shot_cycle_opening_rpm", &shot_cycle_opening_rpm_},
          {"shot_cycle_feeding_rpm", &shot_cycle_feeding_rpm_},
          {"shot_cycle_returning_rpm", &shot_cycle_returning_rpm_},
          {"max_boost_rpm", &max_boost_rpm_},
          {"ball_detection_debounce_count", &ball_detection_debounce_count_},
          {"ball_lost_debounce_count", &ball_lost_debounce_count_},
      };
      const auto entry = std::find_if(
          std::begin(integer_parameters), std::end(integer_parameters),
          [&name](const auto &candidate) { return name == candidate.first; });
      if (entry != std::end(integer_parameters)) {
        *entry->second = value;
      }
      continue;
    }

    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    const double value = parameter.as_double();
    const bool positive =
        name.find("_max_velocity_rad_s") != std::string::npos ||
        name.find("_max_acceleration_rad_s2") != std::string::npos ||
        name == "odometry_timeout_sec";
    const bool unit_interval = name == "current_lpf_alpha" ||
                               name == "measured_acceleration_lpf_alpha" ||
                               name == "belt_ready_ratio";
    const bool signed_position =
        name.find("_position_rad") != std::string::npos;
    if (!std::isfinite(value) || (positive && value <= 0.0) ||
        (unit_interval && (value <= 0.0 || value > 1.0)) ||
        (!positive && !unit_interval && !signed_position && value < 0.0)) {
      result.successful = false;
      result.reason = name + " is outside its valid range";
      return result;
    }

    const std::pair<const char *, double *> double_parameters[] = {
        {"dribble_position_rad", &dribble_position_rad_},
        {"open_position_rad", &open_position_rad_},
        {"bottom_position_rad", &bottom_position_rad_},
        {"feed_position_rad", &feed_position_rad_},
        {"slow_fire_dribble_position_rad", &slow_fire_dribble_position_rad_},
        {"feed_duration_sec", &feed_duration_sec_},
        {"belt_spinup_timeout_sec", &belt_spinup_delay_sec_},
        {"belt_spinup_min_delay_sec", &belt_spinup_min_delay_sec_},
        {"prepare_from_open_delay_sec", &prepare_from_open_delay_sec_},
        {"belt_ready_ratio", &belt_ready_ratio_},
        {"opening_max_velocity_rad_s", &opening_max_velocity_rad_s_},
        {"feeding_max_velocity_rad_s", &feeding_max_velocity_rad_s_},
        {"returning_max_velocity_rad_s", &returning_max_velocity_rad_s_},
        {"dribbling_max_velocity_rad_s", &dribbling_max_velocity_rad_s_},
        {"opening_max_acceleration_rad_s2", &opening_max_acceleration_rad_s2_},
        {"feeding_max_acceleration_rad_s2", &feeding_max_acceleration_rad_s2_},
        {"returning_max_acceleration_rad_s2",
         &returning_max_acceleration_rad_s2_},
        {"dribbling_max_acceleration_rad_s2",
         &dribbling_max_acceleration_rad_s2_},
        {"dribble_reverse_ramp_sec", &dribble_reverse_ramp_sec_},
        {"ball_detection_threshold_a", &ball_detection_threshold_a_},
        {"ball_lost_threshold_a", &ball_lost_threshold_a_},
        {"current_lpf_alpha", &current_lpf_alpha_},
        {"odometry_timeout_sec", &odometry_timeout_sec_},
        {"measured_acceleration_lpf_alpha", &measured_acceleration_lpf_alpha_},
        {"backward_velocity_boost_rpm_per_mps",
         &backward_velocity_boost_rpm_per_mps_},
        {"backward_acceleration_rpm_per_mps2",
         &backward_acceleration_rpm_per_mps2_},
    };
    const auto entry = std::find_if(
        std::begin(double_parameters), std::end(double_parameters),
        [&name](const auto &candidate) { return name == candidate.first; });
    if (entry == std::end(double_parameters)) {
      continue;
    }
    *entry->second = value;
    trajectory_changed =
        trajectory_changed || signed_position ||
        name.find("_max_velocity_rad_s") != std::string::npos ||
        name.find("_max_acceleration_rad_s2") != std::string::npos ||
        name == "feed_duration_sec";
  }

  if (trajectory_changed) {
    const auto current_time = now();
    if (manual_transition_active_) {
      manual_transition_start_time_ = current_time;
      manual_transition_start_position_rad_ = last_position_command_rad_;
    }
    if (shot_cycle_active_) {
      shot_cycle_start_time_ = current_time;
      shot_cycle_start_position_rad_ = last_position_command_rad_;
    }
  }
  return result;
}
