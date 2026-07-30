#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>

SpringEduliteController::SpringEduliteController()
    : Node("spring_controller_node") {
  declare_parameters();
  get_parameters();
  if (limit_switch_index_ < 0 || fire_duration_sec_ <= 0.0 ||
      load_timeout_sec_ <= 0.0) {
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    command_period_ms_ = 10;
    is_configuration_valid_ = false;
  }
  if (qos_depth_ <= 0) {
    qos_depth_ = 1;
  }

  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  operation_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      operation_mode_topic_, state_qos,
      std::bind(&SpringEduliteController::operation_mode_callback, this,
                std::placeholders::_1));
  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      fire_request_topic_, command_qos,
      std::bind(&SpringEduliteController::fire_request_callback, this,
                std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, state_qos,
      std::bind(&SpringEduliteController::emergency_stop_callback, this,
                std::placeholders::_1));
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      limit_switch_topic_, command_qos,
      std::bind(&SpringEduliteController::limit_switch_callback, this,
                std::placeholders::_1));
  spring_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
      spring_velocity_command_topic_, command_qos);
  load_start_time_ = now();
  timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::declare_parameters() {
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("fire_request_topic", "/spring/fire_request");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  declare_parameter<std::string>("limit_switch_topic", "/limit_switches");
  declare_parameter<std::string>("spring_velocity_command_topic",
                                 "/spring/vel_command");
  declare_parameter<int>("limit_switch_index", 0);
  declare_parameter<double>("loading_velocity_rad_s", -5.0);
  declare_parameter<double>("fire_velocity_rad_s", -20.0);
  declare_parameter<double>("fire_duration_sec", 5.0);
  declare_parameter<double>("load_timeout_sec", 5.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void SpringEduliteController::get_parameters() {
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("fire_request_topic", fire_request_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("limit_switch_topic", limit_switch_topic_);
  get_parameter("spring_velocity_command_topic",
                spring_velocity_command_topic_);
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
  operation_mode_ = msg->data <= static_cast<uint8_t>(OperationMode::BELT_ONLY)
                        ? static_cast<OperationMode>(msg->data)
                        : OperationMode::STOP;
  if (!spring_fire_allowed()) {
    prepare_spring_for_stop();
  }
}

void SpringEduliteController::fire_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (spring_fire_allowed() && now_state_ == State::READY && is_loaded_ &&
      msg->data && !previous_fire_request_) {
    fire_pending_ = true;
  }
  previous_fire_request_ = msg->data;
}

void SpringEduliteController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    prepare_spring_for_stop();
  }
}

void SpringEduliteController::limit_switch_callback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  const auto index = static_cast<std::size_t>(limit_switch_index_);
  if (limit_switch_index_ < 0 || index >= msg->data.size()) {
    return;
  }
  is_loaded_ = msg->data[index] != 0;
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
  if (now_state_ != State::LOAD) {
    load_start_time_ = now();
  }
  now_state_ = State::LOAD;
  fire_pending_ = false;
}

void SpringEduliteController::start_fire() {
  now_state_ = State::FIRE;
  fire_start_time_ = now();
  fire_pending_ = false;
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
      start_loading();
    }
  }
  switch (now_state_) {
    case State::LOAD:
      if (is_loaded_) {
        now_state_ = State::READY;
      } else if ((now() - load_start_time_).seconds() >= load_timeout_sec_) {
        now_state_ = State::ERROR;
        RCLCPP_ERROR(get_logger(),
                     "Spring loading timed out. Stopping spring motor.");
      } else {
        command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;
    case State::READY:
      if (!is_loaded_) {
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
