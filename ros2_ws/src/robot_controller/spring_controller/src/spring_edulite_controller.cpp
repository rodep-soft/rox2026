#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>

SpringEduliteController::SpringEduliteController()
    : Node("spring_controller_node") {
  declare_parameters();
  get_parameters();

  if (logical_id_ < 0 || logical_id_ > 65535 || target_topic_.empty() ||
      state_topic_.empty() || set_position_service_.empty()) {
    RCLCPP_ERROR(get_logger(), "logical_id or ROS interface name is invalid");
    config_valid_ = false;
  }
  if (limit_switch_bit_offset_ < 0 || limit_switch_bit_offset_ >= 8) {
    RCLCPP_ERROR(get_logger(),
                 "limit_switch_bit_offset must be between 0 and 7: %d",
                 limit_switch_bit_offset_);
    config_valid_ = false;
  }
  if (!std::isfinite(fire_increment_rad_) || fire_increment_rad_ >= 0.0) {
    RCLCPP_ERROR(get_logger(), "fire_increment_rad must be negative");
    config_valid_ = false;
  }
  if (!std::isfinite(homing_velocity_rad_s_) || homing_velocity_rad_s_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "homing_velocity_rad_s must be positive");
    config_valid_ = false;
  }
  if (!std::isfinite(homing_timeout_sec_) || homing_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "homing_timeout_sec must be positive");
    config_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    command_period_ms_ = 10;
    config_valid_ = false;
  }
  if (qos_depth_ <= 0) {
    qos_depth_ = 1;
  }

  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/spring/fire_request", command_qos,
      std::bind(&SpringEduliteController::fire_request_callback, this,
                std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", state_qos,
      std::bind(&SpringEduliteController::emergency_stop_callback, this,
                std::placeholders::_1));
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "/limit_switchs", command_qos,
      std::bind(&SpringEduliteController::limit_switch_callback, this,
                std::placeholders::_1));
  actuator_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
      state_topic_, command_qos,
      std::bind(&SpringEduliteController::actuator_state_callback, this,
                std::placeholders::_1));

  spring_position_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
      target_topic_, command_qos);
  set_position_client_ =
      create_client<actuator_msgs::srv::SetPosition>(set_position_service_);

  homing_start_time_ = now();
  timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::declare_parameters() {
  declare_parameter<int>("limit_switch_bit_offset", 0);
  declare_parameter<double>("fire_increment_rad", -6.283185307);
  declare_parameter<double>("homing_velocity_rad_s", 0.5);
  declare_parameter<double>("homing_timeout_sec", 30.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
  declare_parameter<int>("logical_id", 4);
  declare_parameter<std::string>("target_topic", "/edulite/target");
  declare_parameter<std::string>("state_topic", "/edulite/state");
  declare_parameter<std::string>("set_position_service",
                                 "/edulite/set_position");
}

void SpringEduliteController::get_parameters() {
  get_parameter("limit_switch_bit_offset", limit_switch_bit_offset_);
  get_parameter("fire_increment_rad", fire_increment_rad_);
  get_parameter("homing_velocity_rad_s", homing_velocity_rad_s_);
  get_parameter("homing_timeout_sec", homing_timeout_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
  get_parameter("logical_id", logical_id_);
  get_parameter("target_topic", target_topic_);
  get_parameter("state_topic", state_topic_);
  get_parameter("set_position_service", set_position_service_);
}

void SpringEduliteController::fire_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  const bool is_rising_edge = msg->data && !previous_fire_request_;
  previous_fire_request_ = msg->data;
  if (!is_rising_edge) {
    return;
  }

  if (!config_valid_ || emergency_stop_active_ ||
      current_state_ != State::READY || !position_reference_set_) {
    RCLCPP_WARN(get_logger(), "Spring fire rejected: homing incomplete, "
                              "emergency stop active, or config invalid.");
    return;
  }

  target_position_rad_ += fire_increment_rad_;
  publish_target();
  RCLCPP_INFO(get_logger(), "Spring target advanced by %.6f rad to %.6f rad.",
              fire_increment_rad_, target_position_rad_);
}

void SpringEduliteController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    RCLCPP_WARN(
        get_logger(),
        "Emergency stop active. Spring position target remains unchanged.");
  }
}

void SpringEduliteController::limit_switch_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  const auto selected_bit =
      static_cast<uint8_t>((msg->data >> limit_switch_bit_offset_) & 0x01U);
  is_limit_switch_on_ = selected_bit != 0U;

  if (is_limit_switch_on_ && current_state_ == State::HOMING && driver_ready_ &&
      !position_reference_set_) {
    request_zero_reference();
  }
}

void SpringEduliteController::actuator_state_callback(
    const actuator_msgs::msg::ActuatorState::SharedPtr msg) {
  if (msg->logical_id != static_cast<uint16_t>(logical_id_)) {
    return;
  }

  const bool is_ready =
      msg->state == actuator_msgs::msg::ActuatorState::STATE_READY;

  if (!is_ready) {
    if (driver_ready_ || position_reference_set_) {
      RCLCPP_WARN(
          get_logger(),
          "Spring EduLite disconnected. Clearing target and homing state.");
    }
    driver_ready_ = false;
    position_reference_set_ = false;
    zero_request_pending_ = false;
    target_position_rad_ = 0.0;
    current_state_ = State::HOMING;
    return;
  }

  if (!driver_ready_) {
    driver_ready_ = true;
    if (!msg->position_reference_set) {
      reset_for_homing();
      RCLCPP_WARN(get_logger(),
                  "Spring position reference is not set. Starting homing.");
    }
  }

  if (!msg->position_reference_set) {
    if (position_reference_set_) {
      reset_for_homing();
      RCLCPP_WARN(get_logger(), "Spring position reference was lost. Target "
                                "reset and homing restarted.");
    }
    position_reference_set_ = false;
    return;
  }

  position_reference_set_ = true;
  if (!zero_request_pending_) {
    current_state_ = State::READY;
  }
}

void SpringEduliteController::timer_callback() {
  if (!config_valid_ || !driver_ready_ || zero_request_pending_) {
    return;
  }

  if (current_state_ == State::HOMING) {
    if (is_limit_switch_on_) {
      request_zero_reference();
      return;
    }

    if ((now() - homing_start_time_).seconds() >= homing_timeout_sec_) {
      current_state_ = State::ERROR;
      RCLCPP_ERROR(get_logger(),
                   "Spring homing timed out. Target is held at %.6f rad.",
                   target_position_rad_);
      return;
    }

    if (!emergency_stop_active_) {
      const double period_sec =
          static_cast<double>(command_period_ms_) / 1000.0;
      target_position_rad_ -= homing_velocity_rad_s_ * period_sec;
    }
    publish_target();
    return;
  }

  // READY and ERROR both keep the current accumulated position target.
  publish_target();
}

void SpringEduliteController::reset_for_homing() {
  target_position_rad_ = 0.0;
  position_reference_set_ = false;
  zero_request_pending_ = false;
  current_state_ = State::HOMING;
  homing_start_time_ = now();
}

void SpringEduliteController::request_zero_reference() {
  if (zero_request_pending_ || !set_position_client_->service_is_ready()) {
    return;
  }

  zero_request_pending_ = true;
  auto request = std::make_shared<actuator_msgs::srv::SetPosition::Request>();
  request->logical_id = static_cast<uint16_t>(logical_id_);
  request->position = 0.0F;

  set_position_client_->async_send_request(
      request,
      [this](rclcpp::Client<actuator_msgs::srv::SetPosition>::SharedFuture
                 future) {
        zero_request_pending_ = false;
        const auto response = future.get();
        if (!response->success) {
          current_state_ = State::ERROR;
          RCLCPP_ERROR(get_logger(), "Failed to zero spring position: %s",
                       response->message.c_str());
          return;
        }

        target_position_rad_ = 0.0;
        position_reference_set_ = true;
        current_state_ = State::READY;
        publish_target();
        RCLCPP_INFO(get_logger(), "Spring homing completed. Target and limit "
                                  "position reset to 0 rad.");
      });
}

void SpringEduliteController::publish_target() {
  actuator_msgs::msg::ActuatorTarget command;
  command.logical_id = static_cast<uint16_t>(logical_id_);
  command.target = static_cast<float>(target_position_rad_);
  spring_position_pub_->publish(command);
}
