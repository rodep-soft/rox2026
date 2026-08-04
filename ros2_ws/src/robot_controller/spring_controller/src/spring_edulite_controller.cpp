#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

SpringEduliteController::SpringEduliteController()
: Node("spring_controller_node")
{
  declare_parameters();
  get_parameters();

  if (limit_switch_bit_offset_ < 0 || limit_switch_bit_offset_ >= 8) {
    RCLCPP_ERROR(get_logger(), "limit_switch_bit_offset must be between 0 and 7: %d",
      limit_switch_bit_offset_);
    config_valid_ = false;
  }
  if (!std::isfinite(fire_duration_sec_) || fire_duration_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "fire_duration_sec must be positive: %.3f", fire_duration_sec_);
    config_valid_ = false;
  }
  if (!std::isfinite(load_timeout_sec_) || load_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "load_timeout_sec must be positive: %.3f", load_timeout_sec_);
    config_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    command_period_ms_ = 10;
    config_valid_ = false;
  }
  if (qos_depth_ <= 0) {qos_depth_ = 1;}

  constexpr double edulite_velocity_limit_rad_s = 50.0;
  if (std::abs(loading_velocity_rad_s_) > edulite_velocity_limit_rad_s ||
    std::abs(fire_velocity_rad_s_) > edulite_velocity_limit_rad_s)
  {
    RCLCPP_ERROR(get_logger(), "Spring velocity magnitude must not exceed 50 rad/s");
    config_valid_ = false;
  }

  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos,
    std::bind(&SpringEduliteController::fire_request_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&SpringEduliteController::emergency_stop_callback, this, std::placeholders::_1));

  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/limit_switchs", command_qos,
    std::bind(&SpringEduliteController::limit_switch_callback, this, std::placeholders::_1));

  spring_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
    "/spring/vel_command", command_qos);

  load_start_time_ = now();
  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::declare_parameters()
{
  declare_parameter<int>("limit_switch_bit_offset", 0);
  declare_parameter<double>("loading_velocity_rad_s", -5.0);
  declare_parameter<double>("fire_velocity_rad_s", -20.0);
  declare_parameter<double>("fire_duration_sec", 5.0);
  declare_parameter<double>("load_timeout_sec", 5.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void SpringEduliteController::get_parameters()
{
  get_parameter("limit_switch_bit_offset", limit_switch_bit_offset_);
  get_parameter("loading_velocity_rad_s", loading_velocity_rad_s_);
  get_parameter("fire_velocity_rad_s", fire_velocity_rad_s_);
  get_parameter("fire_duration_sec", fire_duration_sec_);
  get_parameter("load_timeout_sec", load_timeout_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void SpringEduliteController::fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool is_rising_edge = msg->data && !previous_fire_request_;
  if (is_rising_edge) {
    if (is_fire_allowed() && current_state_ == State::READY && is_loaded_) {
      fire_pending_ = true;
    } else {
      log_fire_request_rejection();
    }
  }
  previous_fire_request_ = msg->data;
}

void SpringEduliteController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    reset_spring_state();
  }
}

void SpringEduliteController::limit_switch_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const bool previous_loaded = is_loaded_;
  last_limit_switch_value_ = msg->data;
  const auto selected_bit = static_cast<uint8_t>(
    (last_limit_switch_value_ >> limit_switch_bit_offset_) & 0x01U);
  is_loaded_ = selected_bit != 0U;

  if (!limit_switch_received_ || is_loaded_ != previous_loaded) {
    RCLCPP_INFO(get_logger(), "Spring limit switch is %s.", is_loaded_ ? "ON" : "OFF");
  }
  limit_switch_received_ = true;
}

void SpringEduliteController::timer_callback()
{
  std_msgs::msg::Float32 command;
  command.data = 0.0F;

  if (!config_valid_ || !is_fire_allowed()) {
    fire_pending_ = false;
    if (current_state_ == State::FIRE) {start_loading();}
  }

  switch (current_state_) {
    case State::LOAD:
      if (is_loaded_) {
        current_state_ = State::READY;
        RCLCPP_INFO(get_logger(), "Spring loading completed. Ready to fire.");
      } else if ((now() - load_start_time_).seconds() >= load_timeout_sec_) {
        current_state_ = State::ERROR;
        RCLCPP_ERROR(get_logger(), "Spring loading timed out. Stopping spring motor.");
      } else {
        command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;

    case State::READY:
      if (!is_loaded_) {
        start_loading();
        command.data = static_cast<float>(loading_velocity_rad_s_);
      } else if (fire_pending_ && is_fire_allowed()) {
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
        current_state_ = State::READY;
      }
      break;
  }

  spring_velocity_pub_->publish(command);
}

bool SpringEduliteController::is_fire_allowed() const
{
  return config_valid_ && !emergency_stop_active_;
}

void SpringEduliteController::reset_spring_state()
{
  fire_pending_ = false;
  if (current_state_ == State::ERROR) {return;}
  if (is_loaded_) {
    current_state_ = State::READY;
  } else {
    start_loading();
  }
}

void SpringEduliteController::start_loading()
{
  const bool was_loading = current_state_ == State::LOAD;
  if (!was_loading || load_start_time_.nanoseconds() == 0) {
    load_start_time_ = now();
  }
  current_state_ = State::LOAD;
  fire_pending_ = false;
}

void SpringEduliteController::start_fire()
{
  current_state_ = State::FIRE;
  fire_start_time_ = now();
  fire_pending_ = false;
  RCLCPP_INFO(get_logger(), "Spring fire started!");
}

const char * SpringEduliteController::state_name(State state) const
{
  switch (state) {
    case State::READY: return "READY";
    case State::LOAD:  return "LOAD";
    case State::FIRE:  return "FIRE";
    case State::ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

void SpringEduliteController::log_fire_request_rejection() const
{
  if (!config_valid_) {
    RCLCPP_WARN(get_logger(), "Spring fire rejected: config invalid.");
  } else if (emergency_stop_active_) {
    RCLCPP_WARN(get_logger(), "Spring fire rejected: emergency stop active.");
  } else if (current_state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Spring fire rejected: state is %s, not READY.",
      state_name(current_state_));
  } else if (!is_loaded_) {
    RCLCPP_WARN(get_logger(), "Spring fire rejected: limit switch is OFF.");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
