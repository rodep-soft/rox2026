#include "spring_controller/spring_edulite_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

SpringEduliteController::SpringEduliteController()
    : Node("spring_controller_node") {
  auto declare_double_parameter = [this](const std::string &name,
                                         double default_value) -> double {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
    rclcpp::Parameter param;
    if (get_parameter(name, param)) {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(param.as_int());
      }
    }
    return default_value;
  };

  standby_offset_rad_ = declare_double_parameter("standby_offset_rad", 0.0);
  position_tolerance_rad_ =
      declare_double_parameter("position_tolerance_rad", 0.05);
  limit_switch_bit_offset_ =
      declare_parameter<int>("limit_switch_bit_offset", 0);
  fire_increment_rad_ =
      declare_double_parameter("fire_increment_rad", -6.283185307);
  slow_fire_target_position_rad_ =
      declare_double_parameter("slow_fire_target_position_rad", 13.5);
  slow_fire_base_velocity_rad_s_ =
      declare_double_parameter("slow_fire_base_velocity_rad_s", 12.0);
  slow_fire_velocity_gain_rad_per_m_ =
      declare_double_parameter("slow_fire_velocity_gain_rad_per_m", 0.0);
  slow_fire_min_velocity_rad_s_ =
      declare_double_parameter("slow_fire_min_velocity_rad_s", 1.0);
  slow_fire_max_velocity_rad_s_ =
      declare_double_parameter("slow_fire_max_velocity_rad_s", 20.0);
  slow_fire_return_velocity_rad_s_ =
      declare_double_parameter("slow_fire_return_velocity_rad_s", 6.0);
  slow_fire_settle_timeout_sec_ =
      declare_double_parameter("slow_fire_settle_timeout_sec", 3.0);
  homing_velocity_rad_s_ =
      declare_double_parameter("homing_velocity_rad_s", 0.5);
  homing_timeout_sec_ = declare_double_parameter("homing_timeout_sec", 30.0);
  stopped_velocity_threshold_rad_s_ =
      declare_double_parameter("stopped_velocity_threshold_rad_s", 0.05);
  required_stable_feedback_count_ =
      declare_parameter<int>("required_stable_feedback_count", 3);
  cmd_vel_timeout_sec_ = declare_double_parameter("cmd_vel_timeout_sec", 0.2);
  command_period_ms_ = declare_parameter<int>("command_period_ms", 10);

  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto logical_id = declare_parameter<int>("logical_id", 4);
  const auto target_topic =
      declare_parameter<std::string>("target_topic", "/edulite/target");
  const auto state_topic =
      declare_parameter<std::string>("state_topic", "/edulite/state");
  const auto set_position_service = declare_parameter<std::string>(
      "set_position_service", "/edulite/set_position");
  const auto cmd_vel_topic = declare_parameter<std::string>(
      "cmd_vel_topic", "/mecanum/cmd_vel_heading");

  if (logical_id < 0 || logical_id > 65535 || target_topic.empty() ||
      state_topic.empty() || set_position_service.empty() ||
      cmd_vel_topic.empty()) {
    throw std::runtime_error("Invalid parameter configurations");
  }

  logical_id_ = static_cast<uint16_t>(logical_id);
  const auto command_qos = rclcpp::QoS(qos_depth);
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();

  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/spring/fire_request", command_qos,
      std::bind(&SpringEduliteController::fire_request_callback, this,
                std::placeholders::_1));

  slow_fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/spring/slow_fire_request", command_qos,
      std::bind(&SpringEduliteController::slow_fire_request_callback, this,
                std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/emergency_stop", emergency_stop_qos,
      std::bind(&SpringEduliteController::emergency_stop_callback, this,
                std::placeholders::_1));

  belt_clearance_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/spring/belt_clearance_request", command_qos,
      std::bind(&SpringEduliteController::belt_clearance_request_callback, this,
                std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, command_qos,
      std::bind(&SpringEduliteController::cmd_vel_callback, this,
                std::placeholders::_1));

  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/hardware/limit_switches", command_qos,
      std::bind(&SpringEduliteController::limit_switch_callback, this,
                std::placeholders::_1));

  actuator_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
      state_topic, command_qos,
      std::bind(&SpringEduliteController::actuator_state_callback, this,
                std::placeholders::_1));

  position_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
      target_topic, command_qos);

  // 装填・待機完了しているかどうかを見るtopic
  actuator_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/spring/actuator_ready", rclcpp::QoS(1).reliable().transient_local());
  operation_state_pub_ =
      create_publisher<robot_msgs::msg::SpringOperationState>(
          "/spring/operation_state",
          rclcpp::QoS(1).reliable().transient_local());

  // spring_controller -> hardware_driver: 原点検出後の現在位置を0
  // radに設定する。
  set_position_client_ =
      create_client<actuator_msgs::srv::SetPosition>(set_position_service);

  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&SpringEduliteController::control_timer_callback, this));

  params_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&SpringEduliteController::parameters_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "SpringEduliteController initialized. Parameters: "
              "standby_offset_rad=%.3f rad, slow_fire_target_position_rad=%.3f "
              "rad, fire_increment_rad=%.3f rad.",
              standby_offset_rad_, slow_fire_target_position_rad_,
              fire_increment_rad_);
}

void SpringEduliteController::fire_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  const bool rising_edge = msg->data && !fire_request_active_;
  fire_request_active_ = msg->data;

  if (!rising_edge) {
    return;
  }

  if (emergency_stop_active_ || belt_clearance_requested_ ||
      state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Fire request rejected: emergency stop, belt "
                              "clearance, or non-READY state.");
    return;
  }

  // 1回転を加算して発射し、回転完了後に待機位置へ戻る
  target_position_rad_ += fire_increment_rad_;
  state_ = State::FIRING;
  stable_feedback_count_ = 0;
  publish_target(target_position_rad_);
  RCLCPP_INFO(get_logger(),
              "Spring firing: commanded rotation (target: %.3f rad). Waiting "
              "to return to standby.",
              target_position_rad_);
}

void SpringEduliteController::slow_fire_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  const bool rising_edge = msg->data && !slow_fire_request_active_;
  slow_fire_request_active_ = msg->data;

  if (!rising_edge) {
    return;
  }

  if (emergency_stop_active_ || belt_clearance_requested_ ||
      state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Slow fire request rejected: emergency stop, "
                              "belt clearance, or non-READY state.");
    return;
  }

  // Normal firing accumulates the multi-turn position. Keep that accumulated
  // revolution as the base and apply the configured zero-referenced slow-fire
  // stroke relative to it. Returning to the absolute standby offset here would
  // command many reverse turns after repeated normal shots.
  slow_fire_base_rad_ = target_position_rad_;
  const double slow_fire_stroke_rad =
      std::max(0.0, slow_fire_target_position_rad_ - standby_offset_rad_);
  slow_fire_peak_rad_ = slow_fire_base_rad_ + slow_fire_stroke_rad;
  state_ = State::SLOW_FIRING_EXTENDING;
  stable_feedback_count_ = 0;
  slow_fire_phase_start_time_ = now();

  RCLCPP_INFO(get_logger(),
              "Spring slow fire started: Base=%.3f rad -> Peak=%.3f rad "
              "(stroke: +%.3f rad, vel: %.2f rad/s).",
              slow_fire_base_rad_, slow_fire_peak_rad_,
              slow_fire_peak_rad_ - slow_fire_base_rad_,
              slow_fire_base_velocity_rad_s_);
}

void SpringEduliteController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data == emergency_stop_active_) {
    return;
  }

  emergency_stop_active_ = msg->data;
  const bool slow_fire_interrupted = state_ == State::SLOW_FIRING_EXTENDING ||
                                     state_ == State::SLOW_FIRING_RETURNING;
  if (!emergency_stop_active_ &&
      (state_ == State::HOMING || state_ == State::WAITING_FOR_STOP)) {
    homing_start_time_ = now();
  }
  if (!slow_fire_interrupted) {
    return;
  }

  if (emergency_stop_active_) {
    stable_feedback_count_ = 0;
    RCLCPP_WARN(get_logger(),
                "Emergency stop activated during slow fire: trajectory paused "
                "at %.3f rad",
                target_position_rad_);
    return;
  }

  slow_fire_phase_start_time_ = now();
  stable_feedback_count_ = 0;
  RCLCPP_INFO(get_logger(),
              "Emergency stop released: resuming unchanged slow-fire "
              "trajectory at %.3f rad",
              target_position_rad_);
}

void SpringEduliteController::belt_clearance_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data == belt_clearance_requested_) {
    return;
  }

  belt_clearance_requested_ = msg->data;
  if (state_ != State::READY && state_ != State::MOVING_TO_STANDBY) {
    return;
  }

  if (belt_clearance_requested_) {
    belt_clearance_return_position_rad_ = target_position_rad_;
    belt_clearance_position_rad_ =
        belt_clearance_return_position_rad_ - standby_offset_rad_;
    target_position_rad_ = belt_clearance_position_rad_;
  } else {
    target_position_rad_ = belt_clearance_return_position_rad_;
  }
  state_ = State::MOVING_TO_STANDBY;
  stable_feedback_count_ = 0;
  publish_target(target_position_rad_);
  RCLCPP_INFO(get_logger(),
              "Moving spring to %.3f rad for belt-shot clearance: %s "
              "(return: %.3f rad)",
              target_position_rad_,
              belt_clearance_requested_ ? "requested" : "released",
              belt_clearance_return_position_rad_);
}

void SpringEduliteController::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  commanded_forward_speed_m_s_ = std::max(0.0, msg->linear.x);
  last_cmd_vel_time_ = now();
}

void SpringEduliteController::limit_switch_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  limit_switch_active_ =
      ((msg->data >> limit_switch_bit_offset_) & 0x01U) != 0U;
}

void SpringEduliteController::actuator_state_callback(
    const actuator_msgs::msg::ActuatorState::SharedPtr msg) {
  if (msg->logical_id != logical_id_) {
    return;
  }

  const bool actuator_state_is_ready =
      msg->state == actuator_msgs::msg::ActuatorState::STATE_READY;
  std_msgs::msg::Bool ready_msg;
  ready_msg.data =
    actuator_state_is_ready && state_ == State::READY && !belt_clearance_requested_;
  actuator_ready_pub_->publish(ready_msg);

  if (!actuator_state_is_ready) {
    if (actuator_ready_ || position_reference_set_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Spring EduLite disconnected. Clearing target and homing state.");
    }
    last_published_target_rad_.reset();
    position_reference_set_ = false;
    actuator_position_received_ = false;
    homing_required_ = true;
    state_ = State::UNINITIALIZED;
    return;
  }
  actuator_ready_ = true;
  position_reference_set_ = msg->position_reference_set;
  actuator_position_rad_ = msg->position;
  actuator_position_received_ = true;

  // A restored driver reference is not proof that the spring limit switch has
  // been used. Home once after startup, beginning at feedback position after
  // emergency stop is released.
  if (state_ == State::UNINITIALIZED && homing_required_ &&
      !emergency_stop_active_) {
    RCLCPP_WARN(get_logger(),
                "Spring EduLite startup detected. Starting one-time HOMING.");
    start_homing();
  }
  // Feedback remains useful for the release position, but state completion and
  // timeout checks must stay paused until emergency stop is released.
  if (state_ == State::READY && belt_clearance_requested_ &&
      std::fabs(target_position_rad_ - belt_clearance_position_rad_) >
          position_tolerance_rad_) {
    target_position_rad_ = belt_clearance_position_rad_;
    state_ = State::MOVING_TO_STANDBY;
    stable_feedback_count_ = 0;
    publish_target(target_position_rad_);
  }

  const bool emergency_motion_paused =
      emergency_stop_active_ &&
      (state_ == State::HOMING || state_ == State::WAITING_FOR_STOP ||
       state_ == State::SLOW_FIRING_EXTENDING ||
       state_ == State::SLOW_FIRING_RETURNING);
  if (emergency_motion_paused) {
    return;
  }

  // リミットスイッチ検知後の静止検出（HOMING中）
  if (state_ == State::WAITING_FOR_STOP && limit_switch_active_ &&
      !zero_service_pending_) {
    if (std::fabs(msg->velocity) <= stopped_velocity_threshold_rad_s_) {
      ++stable_feedback_count_;
    } else {
      stable_feedback_count_ = 0;
    }

    if (stable_feedback_count_ >= required_stable_feedback_count_) {
      request_zero_reference();
    }
  }

  // 待機位置への移動完了判定 (MOVING_TO_STANDBY) または 発射回転完了判定
  // (FIRING)
  if (state_ == State::MOVING_TO_STANDBY || state_ == State::FIRING) {
    if (update_settled(*msg)) {
      if (state_ == State::MOVING_TO_STANDBY) {
        RCLCPP_INFO(get_logger(),
                    "Reached standby position (%.3f rad). Spring is READY.",
                    target_position_rad_);
      } else {
        RCLCPP_INFO(
            get_logger(),
            "Spring fire complete (reached %.3f rad). Ready for next fire.",
            target_position_rad_);
      }
      state_ = State::READY;
      stable_feedback_count_ = 0;
    }
  }

  // ゆっくり射出: 前進完了判定 (SLOW_FIRING_EXTENDING -> SLOW_FIRING_RETURNING)
  if (state_ == State::SLOW_FIRING_EXTENDING) {
    const double elapsed_sec = (now() - slow_fire_phase_start_time_).seconds();
    const double stroke_rad =
        std::fabs(slow_fire_peak_rad_ - slow_fire_base_rad_);
    // Forward-speed compensation can reduce the extension velocity. Use the
    // configured minimum for the timeout estimate so valid slow motion is not
    // treated as a failure.
    const double expected_duration_sec =
        (slow_fire_min_velocity_rad_s_ > 0.0)
            ? (stroke_rad / slow_fire_min_velocity_rad_s_)
            : 1.0;

    if (elapsed_sec >= expected_duration_sec + slow_fire_settle_timeout_sec_) {
      enter_error_with_position_hold(msg->position,
                                     "Slow fire extension timed out");
      return;
    }

    const bool pos_reached =
        msg->position >= slow_fire_peak_rad_ - position_tolerance_rad_;
    if (pos_reached && target_position_rad_ >= slow_fire_peak_rad_) {
      ++stable_feedback_count_;
    } else {
      stable_feedback_count_ = 0;
    }

    if (stable_feedback_count_ >= required_stable_feedback_count_) {
      RCLCPP_INFO(get_logger(),
                  "Reached slow fire peak (target: %.3f rad, feedback: %.3f "
                  "rad). Returning to base (%.3f rad).",
                  slow_fire_peak_rad_, msg->position, slow_fire_base_rad_);
      state_ = State::SLOW_FIRING_RETURNING;
      stable_feedback_count_ = 0;
      slow_fire_phase_start_time_ = now();
    }
  }

  // ゆっくり射出: 待機位置への復帰完了判定 (SLOW_FIRING_RETURNING -> READY)
  if (state_ == State::SLOW_FIRING_RETURNING) {
    const double elapsed_sec = (now() - slow_fire_phase_start_time_).seconds();
    const double stroke_rad =
        std::fabs(slow_fire_peak_rad_ - slow_fire_base_rad_);
    const double expected_duration_sec =
        (slow_fire_return_velocity_rad_s_ > 0.0)
            ? (stroke_rad / slow_fire_return_velocity_rad_s_)
            : 1.0;

    if (elapsed_sec >= expected_duration_sec + slow_fire_settle_timeout_sec_) {
      enter_error_with_position_hold(msg->position,
                                     "Slow fire return timed out");
      return;
    }

    const bool pos_reached = std::fabs(msg->position - slow_fire_base_rad_) <=
                             position_tolerance_rad_;
    const bool vel_stopped =
        std::fabs(msg->velocity) <= stopped_velocity_threshold_rad_s_;

    if (pos_reached && vel_stopped &&
        target_position_rad_ <= slow_fire_base_rad_ + 1e-4) {
      ++stable_feedback_count_;
    } else {
      stable_feedback_count_ = 0;
    }

    if (stable_feedback_count_ >= required_stable_feedback_count_) {
      target_position_rad_ = slow_fire_base_rad_;
      publish_target(target_position_rad_);
      RCLCPP_INFO(get_logger(),
                  "Spring slow fire complete (returned to %.3f rad). Ready for "
                  "next action.",
                  target_position_rad_);
      state_ = State::READY;
      stable_feedback_count_ = 0;
    }
  }
}

void SpringEduliteController::control_timer_callback() {
  publish_operation_state();
  if (state_ == State::UNINITIALIZED || state_ == State::ERROR ||
      zero_service_pending_) {
    return;
  }

  const bool emergency_motion_paused =
      emergency_stop_active_ &&
      (state_ == State::HOMING || state_ == State::WAITING_FOR_STOP ||
       state_ == State::SLOW_FIRING_EXTENDING ||
       state_ == State::SLOW_FIRING_RETURNING);
  if (emergency_motion_paused) {
    return;
  }

  // ホーミングタイムアウト判定（非常停止解除中のみカウント）
  if (state_ == State::HOMING || state_ == State::WAITING_FOR_STOP) {
    if ((now() - homing_start_time_).seconds() >= homing_timeout_sec_) {
      state_ = State::ERROR;
      RCLCPP_ERROR(get_logger(), "Spring homing timed out!");
      return;
    }
  }

  // HOMING中：リミット判定されるまで低速回転
  if (state_ == State::HOMING) {
    if (limit_switch_active_) {
      state_ = State::WAITING_FOR_STOP;
      stable_feedback_count_ = 0;
      RCLCPP_INFO(get_logger(),
                  "Limit switch activated. Waiting for motion to settle.");
    } else {
      const double period_sec =
          static_cast<double>(command_period_ms_) / 1000.0;
      target_position_rad_ -= homing_velocity_rad_s_ * period_sec;
      publish_target(target_position_rad_);
    }
  } else if (state_ == State::SLOW_FIRING_EXTENDING) {
    const double period_sec = static_cast<double>(command_period_ms_) / 1000.0;
    const bool cmd_vel_fresh =
        last_cmd_vel_time_.nanoseconds() > 0 &&
        (now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_sec_;
    const double forward_speed =
        cmd_vel_fresh ? commanded_forward_speed_m_s_ : 0.0;
    const double extension_velocity = std::clamp(
        slow_fire_base_velocity_rad_s_ -
            forward_speed * slow_fire_velocity_gain_rad_per_m_,
        slow_fire_min_velocity_rad_s_, slow_fire_max_velocity_rad_s_);
    target_position_rad_ += extension_velocity * period_sec;
    if (target_position_rad_ >= slow_fire_peak_rad_) {
      target_position_rad_ = slow_fire_peak_rad_;
    }
    publish_target(target_position_rad_);
  } else if (state_ == State::SLOW_FIRING_RETURNING) {
    const double period_sec = static_cast<double>(command_period_ms_) / 1000.0;
    target_position_rad_ -= slow_fire_return_velocity_rad_s_ * period_sec;
    if (target_position_rad_ <= slow_fire_base_rad_) {
      target_position_rad_ = slow_fire_base_rad_;
    }
    publish_target(target_position_rad_);
  } else {
    publish_target(target_position_rad_);
  }
}

void SpringEduliteController::start_homing() {
  state_ = State::HOMING;
  target_position_rad_ =
      actuator_position_received_ ? actuator_position_rad_ : 0.0;
  stable_feedback_count_ = 0;
  zero_service_pending_ = false;
  homing_start_time_ = now();
}

void SpringEduliteController::request_zero_reference() {
  if (zero_service_pending_ || !set_position_client_->service_is_ready()) {
    return;
  }

  zero_service_pending_ = true;
  auto request = std::make_shared<actuator_msgs::srv::SetPosition::Request>();
  request->logical_id = logical_id_;
  request->position = 0.0f;

  set_position_client_->async_send_request(
      request,
      [this](rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedFuture
                 future) {
        zero_service_pending_ = false;
        const auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(),
                      "Spring position successfully zeroed to 0.0 rad.");
          homing_required_ = false;
          if (std::fabs(standby_offset_rad_) > 1e-4) {
            target_position_rad_ =
                belt_clearance_requested_ ? 0.0 : standby_offset_rad_;
            state_ = State::MOVING_TO_STANDBY;
            stable_feedback_count_ = 0;
            publish_target(target_position_rad_);
            RCLCPP_INFO(get_logger(), "Moving to standby position: %.3f rad.",
                        target_position_rad_);
          } else {
            target_position_rad_ = 0.0;
            state_ = State::READY;
            publish_target(0.0);
          }
        } else {
          state_ = State::ERROR;
          RCLCPP_ERROR(get_logger(), "Failed to zero spring position: %s",
                       response->message.c_str());
        }
      });
}

void SpringEduliteController::enter_error_with_position_hold(
    double current_position_rad, const char *reason) {
  target_position_rad_ = current_position_rad;
  stable_feedback_count_ = 0;
  state_ = State::ERROR;
  publish_target(target_position_rad_);
  RCLCPP_ERROR(get_logger(), "%s. Holding current position at %.3f rad.",
               reason, target_position_rad_);
}

bool SpringEduliteController::update_settled(
    const actuator_msgs::msg::ActuatorState &feedback) {
  const bool settled =
      std::fabs(feedback.position - target_position_rad_) <=
          position_tolerance_rad_ &&
      std::fabs(feedback.velocity) <= stopped_velocity_threshold_rad_s_;
  stable_feedback_count_ = settled ? stable_feedback_count_ + 1 : 0;
  return stable_feedback_count_ >= required_stable_feedback_count_;
}

void SpringEduliteController::publish_target(double target_rad, bool force) {
  if (!force && last_published_target_rad_.has_value() &&
      std::fabs(*last_published_target_rad_ - target_rad) <= 1e-6) {
    return;
  }
  last_published_target_rad_ = target_rad;
  actuator_msgs::msg::ActuatorTarget cmd;
  cmd.logical_id = logical_id_;
  cmd.target = static_cast<float>(target_rad);
  position_command_pub_->publish(cmd);
}

void SpringEduliteController::publish_operation_state() {
  uint8_t operation = robot_msgs::msg::SpringOperationState::IDLE;
  if (state_ == State::HOMING || state_ == State::WAITING_FOR_STOP) {
    operation = robot_msgs::msg::SpringOperationState::HOMING;
  } else if (state_ == State::FIRING) {
    operation = robot_msgs::msg::SpringOperationState::NORMAL_FIRE;
  } else if (state_ == State::SLOW_FIRING_EXTENDING ||
             state_ == State::SLOW_FIRING_RETURNING) {
    operation = robot_msgs::msg::SpringOperationState::SLOW_FIRE;
  } else if (belt_clearance_requested_ && state_ == State::READY) {
    // BELT_CLEARANCE means the zero-offset target has been reached and settled,
    // not merely that a retraction request is active.
    operation = robot_msgs::msg::SpringOperationState::BELT_CLEARANCE;
  } else if (state_ == State::ERROR) {
    operation = robot_msgs::msg::SpringOperationState::ERROR;
  }
  if (operation == last_published_operation_state_) {
    return;
  }
  robot_msgs::msg::SpringOperationState msg;
  msg.state = operation;
  operation_state_pub_->publish(msg);
  last_published_operation_state_ = operation;
}

rcl_interfaces::msg::SetParametersResult
SpringEduliteController::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    const auto &name = param.get_name();
    if (name == "standby_offset_rad") {
      double new_standby = 0.0;
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        new_standby = param.as_double();
      } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        new_standby = static_cast<double>(param.as_int());
      } else {
        result.successful = false;
        result.reason = "standby_offset_rad must be a number";
        return result;
      }
      standby_offset_rad_ = new_standby;
      RCLCPP_INFO(get_logger(), "Updated standby_offset_rad to %.3f rad",
                  standby_offset_rad_);

      // READY または MOVING_TO_STANDBY 状態であれば、即座に新しい待機位置へ移動
      if (state_ == State::READY || state_ == State::MOVING_TO_STANDBY) {
        target_position_rad_ =
            belt_clearance_requested_ ? 0.0 : standby_offset_rad_;
        stable_feedback_count_ = 0;
        state_ = State::MOVING_TO_STANDBY;
        publish_target(target_position_rad_);
        RCLCPP_INFO(get_logger(), "Applying new standby position: %.3f rad.",
                    target_position_rad_);
      }
    } else if (name == "position_tolerance_rad") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        position_tolerance_rad_ = param.as_double();
      }
    } else if (name == "fire_increment_rad") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        fire_increment_rad_ = param.as_double();
      }
    } else if (name == "slow_fire_target_position_rad") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_target_position_rad_ = param.as_double();
      }
    } else if (name == "slow_fire_base_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_base_velocity_rad_s_ = param.as_double();
      }
    } else if (name == "slow_fire_velocity_gain_rad_per_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_velocity_gain_rad_per_m_ = param.as_double();
      }
    } else if (name == "slow_fire_min_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_min_velocity_rad_s_ = param.as_double();
      }
    } else if (name == "slow_fire_max_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_max_velocity_rad_s_ = param.as_double();
      }
    } else if (name == "slow_fire_settle_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        slow_fire_settle_timeout_sec_ = param.as_double();
      }
    } else if (name == "cmd_vel_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        cmd_vel_timeout_sec_ = param.as_double();
      }
    } else if (name == "homing_velocity_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        homing_velocity_rad_s_ = param.as_double();
      }
    } else if (name == "homing_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        homing_timeout_sec_ = param.as_double();
      }
    } else if (name == "stopped_velocity_threshold_rad_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        stopped_velocity_threshold_rad_s_ = param.as_double();
      }
    } else if (name == "required_stable_feedback_count") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        required_stable_feedback_count_ = static_cast<int>(param.as_int());
      }
    }
  }

  return result;
}
