#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cstddef>
#include <cmath>
#include <functional>
#include <memory>

SpringEduliteController::SpringEduliteController()
    : Node("spring_controller_node") {
  declare_parameters();
  get_parameters();
  if (limit_switch_index_ < 0) {
    RCLCPP_ERROR(get_logger(), "limit_switch_index must be zero or greater: %d",
                 limit_switch_index_);
    is_configuration_valid_ = false;
  }
  if (fire_duration_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "fire_duration_sec must be greater than zero: %.3f",
                 fire_duration_sec_);
    is_configuration_valid_ = false;
  }
  if (load_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "load_timeout_sec must be greater than zero: %.3f",
                 load_timeout_sec_);
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero: %d",
                 command_period_ms_);
    command_period_ms_ = 10;
    is_configuration_valid_ = false;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive: %d. Using 1.",
                qos_depth_);
    qos_depth_ = 1;
  }
  if (!std::isfinite(loading_velocity_rad_s_) ||
      !std::isfinite(fire_velocity_rad_s_)) {
    RCLCPP_ERROR(
        get_logger(),
        "Spring velocities must be finite: loading=%.6f rad/s, fire=%.6f rad/s",
        loading_velocity_rad_s_, fire_velocity_rad_s_);
    is_configuration_valid_ = false;
  }
  if (!is_configuration_valid_) {
    RCLCPP_ERROR(get_logger(),
                 "Spring controller configuration is invalid. Velocity command is fixed at 0 rad/s.");
  }

  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  operation_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/operation_mode", state_qos,
      std::bind(&SpringEduliteController::operation_mode_callback, this,
                std::placeholders::_1));
  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/spring/fire_request", command_qos,
      std::bind(&SpringEduliteController::fire_request_callback, this,
                std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", state_qos,
      std::bind(&SpringEduliteController::emergency_stop_callback, this,
                std::placeholders::_1));
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/limit_switches", command_qos,
      std::bind(&SpringEduliteController::limit_switch_callback, this,
                std::placeholders::_1));
  spring_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/spring/vel_command", command_qos);
  load_start_time_ = now();
  if (is_configuration_valid_) {
    RCLCPP_INFO(get_logger(),
                "Spring controller started in LOAD: velocity=%.3f rad/s.",
                loading_velocity_rad_s_);
  }
  timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::declare_parameters() {
  declare_parameter<int>("limit_switch_index", 0);
  declare_parameter<double>("loading_velocity_rad_s", -5.0);
  declare_parameter<double>("fire_velocity_rad_s", -20.0);
  declare_parameter<double>("fire_duration_sec", 5.0);
  declare_parameter<double>("load_timeout_sec", 5.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void SpringEduliteController::get_parameters() {
  get_parameter("limit_switch_index", limit_switch_index_);
  get_parameter("loading_velocity_rad_s", loading_velocity_rad_s_);
  get_parameter("fire_velocity_rad_s", fire_velocity_rad_s_);
  get_parameter("fire_duration_sec", fire_duration_sec_);
  get_parameter("load_timeout_sec", load_timeout_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void SpringEduliteController::operation_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (msg->data <= static_cast<uint8_t>(OperationMode::BELT_ONLY)) {
    operation_mode_ = static_cast<OperationMode>(msg->data);
  } else {
    RCLCPP_WARN(get_logger(), "Invalid operation mode received: %u. Treating as STOP.",
                msg->data);
    operation_mode_ = OperationMode::STOP;
  }
  if (!spring_fire_allowed()) {
    if (now_state_ == State::FIRE) {
      RCLCPP_WARN(get_logger(),
                  "Spring fire interrupted: operation mode is %s.",
                  operation_mode_name(operation_mode_));
    }
    prepare_spring_for_stop();
  }
}

void SpringEduliteController::fire_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  const bool is_rising_edge = msg->data && !previous_fire_request_;
  if (is_rising_edge) {
    if (spring_fire_allowed() && now_state_ == State::READY && is_loaded_) {
      fire_pending_ = true;
    } else {
      log_fire_request_rejection();
    }
  }
  previous_fire_request_ = msg->data;
}

void SpringEduliteController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data != emergency_stop_active_) {
    if (msg->data) {
      RCLCPP_WARN(get_logger(), "Emergency stop activated: state=%s.",
                  state_name(now_state_));
    } else {
      RCLCPP_INFO(get_logger(), "Emergency stop released.");
    }
  }
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    if (now_state_ == State::FIRE) {
      RCLCPP_WARN(get_logger(), "Spring fire interrupted by emergency stop.");
    }
    prepare_spring_for_stop();
    if (now_state_ == State::LOAD) {
      RCLCPP_WARN(
          get_logger(),
          "Emergency stop is active, but the current controller behavior continues loading at %.3f rad/s.",
          loading_velocity_rad_s_);
    }
  }
}

void SpringEduliteController::limit_switch_callback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  if (limit_switch_index_ < 0) {
    return;
  }
  const auto index = static_cast<std::size_t>(limit_switch_index_);
  if (index >= msg->data.size()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Limit switch data is too short: index=%d, size=%zu.",
        limit_switch_index_, msg->data.size());
    return;
  }

  const bool previous_loaded = is_loaded_;
  last_limit_switch_value_ = msg->data[index];
  is_loaded_ = last_limit_switch_value_ != 0;
  if (!limit_switch_received_ || is_loaded_ != previous_loaded) {
    RCLCPP_INFO(get_logger(), "Spring limit switch[%d] is %s (value=%u).",
                limit_switch_index_, is_loaded_ ? "ON" : "OFF",
                last_limit_switch_value_);
  }
  limit_switch_received_ = true;
}

bool SpringEduliteController::spring_fire_allowed() const {
  return is_configuration_valid_ && !emergency_stop_active_ &&
         operation_mode_ == OperationMode::DRIVE;
}

void SpringEduliteController::prepare_spring_for_stop() {
  fire_pending_ = false;
  if (now_state_ == State::ERROR) {
    return;
  }
  if (is_loaded_) {
    now_state_ = State::READY;
  } else {
    start_loading();
  }
}

void SpringEduliteController::start_loading() {
  const bool was_loading = now_state_ == State::LOAD;
  if (!was_loading) {
    load_start_time_ = now();
  }
  now_state_ = State::LOAD;
  fire_pending_ = false;
  if (!was_loading) {
    RCLCPP_INFO(get_logger(), "Spring loading started: velocity=%.3f rad/s.",
                loading_velocity_rad_s_);
  }
}

void SpringEduliteController::start_fire() {
  now_state_ = State::FIRE;
  fire_start_time_ = now();
  fire_pending_ = false;
  RCLCPP_INFO(get_logger(),
              "Spring fire started: velocity=%.3f rad/s, duration=%.3f s.",
              fire_velocity_rad_s_, fire_duration_sec_);
}

void SpringEduliteController::timer_callback() {
  std_msgs::msg::Float32 command;
  command.data = 0.0F;
  if (!is_configuration_valid_) {
    fire_pending_ = false;
    spring_velocity_pub_->publish(command);
    return;
  }
  if (!spring_fire_allowed()) {
    fire_pending_ = false;
    if (now_state_ == State::FIRE) {
      RCLCPP_WARN(get_logger(), "Spring fire interrupted while command was being sent.");
      start_loading();
    }
  }
  switch (now_state_) {
    case State::LOAD:
      if (is_loaded_) {
        now_state_ = State::READY;
        RCLCPP_INFO(get_logger(), "Spring loading completed. Spring is ready.");
      } else if ((now() - load_start_time_).seconds() >= load_timeout_sec_) {
        now_state_ = State::ERROR;
        RCLCPP_ERROR(
            get_logger(),
            "Spring loading timed out after %.3f s: limit_switch_index=%d, value=%u, "
            "velocity=%.3f rad/s. Stopping spring motor.",
            load_timeout_sec_, limit_switch_index_, last_limit_switch_value_,
            loading_velocity_rad_s_);
      } else {
        command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;
    case State::READY:
      if (!is_loaded_) {
        RCLCPP_WARN(get_logger(),
                    "Spring limit switch turned OFF while READY. Restarting loading.");
        start_loading();
        command.data = static_cast<float>(loading_velocity_rad_s_);
      } else if (fire_pending_ && spring_fire_allowed()) {
        start_fire();
        command.data = static_cast<float>(fire_velocity_rad_s_);
      }
      break;
    case State::FIRE:
      command.data = static_cast<float>(fire_velocity_rad_s_);
      if ((now() - fire_start_time_).seconds() >= fire_duration_sec_) {
        RCLCPP_INFO(get_logger(), "Spring fire completed. Restarting loading.");
        start_loading();
      }
      break;
    case State::ERROR:
      if (is_loaded_) {
        now_state_ = State::READY;
        RCLCPP_INFO(get_logger(),
                    "Spring load switch is active. Loading error cleared.");
      }
      break;
  }
  spring_velocity_pub_->publish(command);
}

const char* SpringEduliteController::state_name(State state) const {
  switch (state) {
    case State::READY:
      return "READY";
    case State::LOAD:
      return "LOAD";
    case State::FIRE:
      return "FIRE";
    case State::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

const char* SpringEduliteController::operation_mode_name(
    OperationMode mode) const {
  switch (mode) {
    case OperationMode::STOP:
      return "STOP";
    case OperationMode::DRIVE:
      return "DRIVE";
    case OperationMode::SHOT_CYCLE:
      return "SHOT_CYCLE";
    case OperationMode::BELT_ONLY:
      return "BELT_ONLY";
  }
  return "UNKNOWN";
}

void SpringEduliteController::log_fire_request_rejection() const {
  if (!is_configuration_valid_) {
    RCLCPP_WARN(get_logger(),
                "Spring fire request rejected: controller configuration is invalid.");
    return;
  }
  if (emergency_stop_active_) {
    RCLCPP_WARN(get_logger(),
                "Spring fire request rejected: emergency stop is active.");
    return;
  }
  if (operation_mode_ != OperationMode::DRIVE) {
    RCLCPP_WARN(get_logger(),
                "Spring fire request rejected: operation mode is %s, not DRIVE.",
                operation_mode_name(operation_mode_));
    return;
  }
  if (now_state_ != State::READY) {
    RCLCPP_WARN(get_logger(),
                "Spring fire request rejected: state is %s, not READY.",
                state_name(now_state_));
    return;
  }
  if (!is_loaded_) {
    RCLCPP_WARN(get_logger(),
                "Spring fire request rejected: limit switch is OFF (not loaded).");
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
