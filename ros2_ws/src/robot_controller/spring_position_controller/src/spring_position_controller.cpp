#include "spring_position_controller/spring_position_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

SpringPositionController::SpringPositionController()
    : Node("spring_position_controller_node") {
  declare_parameters();
  get_parameters();
  validate_parameters();
  create_interfaces();

  spring_timer_ = create_wall_timer(
      std::chrono::milliseconds(spring_command_period_ms_),
      std::bind(&SpringPositionController::spring_timer_callback, this));
  position_timer_ = create_wall_timer(
      std::chrono::milliseconds(position_command_period_ms_),
      std::bind(&SpringPositionController::position_timer_callback, this));
  set_target_position(dribble_position_rad_, PositionState::RETURN_TO_DRIBBLE);
}

void SpringPositionController::declare_parameters() {
  // 共通topic
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("operation_mode_complete_topic",
                                 "/operation_mode_complete");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");

  // Spring
  declare_parameter<std::string>("fire_request_topic", "/spring/fire_request");
  declare_parameter<std::string>("limit_switch_topic", "/limit_switches");
  declare_parameter<std::string>("spring_velocity_command_topic",
                                 "/spring/vel_command");
  declare_parameter<int>("limit_switch_index", 0);
  declare_parameter<double>("loading_velocity_rad_s", -5.0);
  declare_parameter<double>("fire_velocity_rad_s", -20.0);
  declare_parameter<double>("fire_duration_sec", 5.0);
  declare_parameter<int>("spring_command_period_ms", 10);

  // position
  declare_parameter<std::string>("position_mode_topic",
                                 "/dribble/position_mode");
  declare_parameter<std::string>("intake_and_shoot_topic", "/intake_and_shoot");
  declare_parameter<std::string>("position_sequence_active_topic",
                                 "/position_sequence_active");
  declare_parameter<std::string>("position_command_topic",
                                 "/dribble/position_command");
  declare_parameter<std::string>("position_feedback_topic",
                                 "/dribble/position_feedback");
  declare_parameter<double>("dribble_position_rad", 0.0);
  declare_parameter<double>("intake_position_rad", 1.5);
  declare_parameter<double>("shoot_position_rad", 2.0);
  declare_parameter<double>("max_open_position_rad", -1.3);
  declare_parameter<double>("position_tolerance_rad", 0.02);
  declare_parameter<double>("shoot_to_dribble_delay_sec", 1.0);
  declare_parameter<double>("move_timeout_sec", 3.0);
  declare_parameter<double>("feedback_timeout_sec", 0.5);
  declare_parameter<int>("position_command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
}

void SpringPositionController::get_parameters() {
  // 共通topic
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("operation_mode_complete_topic",
                operation_mode_complete_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);

  // Spring
  get_parameter("fire_request_topic", fire_request_topic_);
  get_parameter("limit_switch_topic", limit_switch_topic_);
  get_parameter("spring_velocity_command_topic",
                spring_velocity_command_topic_);
  get_parameter("limit_switch_index", limit_switch_index_);
  get_parameter("loading_velocity_rad_s", loading_velocity_rad_s_);
  get_parameter("fire_velocity_rad_s", fire_velocity_rad_s_);
  get_parameter("fire_duration_sec", fire_duration_sec_);
  get_parameter("spring_command_period_ms", spring_command_period_ms_);

  // position
  get_parameter("position_mode_topic", position_mode_topic_);
  get_parameter("intake_and_shoot_topic", intake_and_shoot_topic_);
  get_parameter("position_sequence_active_topic",
                position_sequence_active_topic_);
  get_parameter("position_command_topic", position_command_topic_);
  get_parameter("position_feedback_topic", position_feedback_topic_);
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("intake_position_rad", intake_position_rad_);
  get_parameter("shoot_position_rad", shoot_position_rad_);
  get_parameter("max_open_position_rad", max_open_position_rad_);
  get_parameter("position_tolerance_rad", position_tolerance_rad_);
  get_parameter("shoot_to_dribble_delay_sec", shoot_to_dribble_delay_sec_);
  get_parameter("move_timeout_sec", move_timeout_sec_);
  get_parameter("feedback_timeout_sec", feedback_timeout_sec_);
  get_parameter("position_command_period_ms", position_command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void SpringPositionController::validate_parameters() {
  if (limit_switch_index_ < 0) {
    RCLCPP_ERROR(get_logger(), "limit_switch_index must be zero or greater");
    configuration_valid_ = false;
  }
  if (fire_duration_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "fire_duration_sec must be greater than zero");
    configuration_valid_ = false;
  }
  if (spring_command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(),
                 "spring_command_period_ms must be greater than zero");
    configuration_valid_ = false;
    spring_command_period_ms_ = 10;
  }
  if (!std::isfinite(position_tolerance_rad_) ||
      position_tolerance_rad_ <= 0.0) {
    RCLCPP_ERROR(get_logger(),
                 "position_tolerance_rad must be finite and positive");
    configuration_valid_ = false;
    position_tolerance_rad_ = 0.02;
  }
  if (position_command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(),
                 "position_command_period_ms must be greater than zero");
    configuration_valid_ = false;
    position_command_period_ms_ = 20;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using 1.");
    qos_depth_ = 1;
  }
}

void SpringPositionController::create_interfaces() {
  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  operation_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      operation_mode_topic_, state_qos,
      std::bind(&SpringPositionController::operation_mode_callback, this,
                std::placeholders::_1));
  fire_request_subscription_ = create_subscription<std_msgs::msg::Bool>(
      fire_request_topic_, command_qos,
      std::bind(&SpringPositionController::fire_request_callback, this,
                std::placeholders::_1));
  emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, state_qos,
      std::bind(&SpringPositionController::emergency_stop_callback, this,
                std::placeholders::_1));
  limit_switch_subscription_ =
      create_subscription<std_msgs::msg::UInt8MultiArray>(
          limit_switch_topic_, command_qos,
          std::bind(&SpringPositionController::limit_switch_callback, this,
                    std::placeholders::_1));
  position_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      position_mode_topic_, command_qos,
      std::bind(&SpringPositionController::position_mode_callback, this,
                std::placeholders::_1));
  intake_and_shoot_subscription_ = create_subscription<std_msgs::msg::Bool>(
      intake_and_shoot_topic_, command_qos,
      std::bind(&SpringPositionController::intake_and_shoot_callback, this,
                std::placeholders::_1));
  position_feedback_subscription_ = create_subscription<std_msgs::msg::Float32>(
      position_feedback_topic_, command_qos,
      std::bind(&SpringPositionController::position_feedback_callback, this,
                std::placeholders::_1));

  spring_velocity_publisher_ = create_publisher<std_msgs::msg::Float32>(
      spring_velocity_command_topic_, command_qos);
  position_command_publisher_ = create_publisher<std_msgs::msg::Float32>(
      position_command_topic_, command_qos);
  operation_mode_complete_publisher_ = create_publisher<std_msgs::msg::Bool>(
      operation_mode_complete_topic_, command_qos);
  position_sequence_active_publisher_ = create_publisher<std_msgs::msg::Bool>(
      position_sequence_active_topic_, command_qos);
}

void SpringPositionController::operation_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  OperationMode new_mode = OperationMode::STOP;
  if (msg->data <= static_cast<uint8_t>(OperationMode::GAME2_MODE)) {
    new_mode = static_cast<OperationMode>(msg->data);
  }
  if (new_mode == operation_mode_) {
    return;
  }

  const OperationMode previous_mode = operation_mode_;
  operation_mode_ = new_mode;

  if (!spring_operation_allowed()) {
    stop_spring_state_transition();
  }
  if (operation_mode_ == OperationMode::STOP) {
    position_sequence_active_ = false;
    publish_position_sequence_active(false);
    set_target_position(dribble_position_rad_,
                        PositionState::RETURN_TO_DRIBBLE);
  } else if (operation_mode_ == OperationMode::GAME2_MODE) {
    position_sequence_active_ = false;
    publish_position_sequence_active(false);
    set_target_position(max_open_position_rad_, PositionState::MAX_OPEN);
  } else if (previous_mode == OperationMode::GAME2_MODE &&
             operation_mode_ == OperationMode::DRIVE) {
    set_target_position(dribble_position_rad_,
                        PositionState::RETURN_TO_DRIBBLE);
  }
}

void SpringPositionController::fire_request_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (spring_operation_allowed() && spring_state_ == SpringState::READY &&
      is_loaded_ && msg->data && !previous_fire_request_) {
    fire_pending_ = true;
  }
  previous_fire_request_ = msg->data;
}

void SpringPositionController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  emergency_stop_active_ = msg->data;
  if (!emergency_stop_active_) {
    return;
  }

  stop_spring_state_transition();
  position_sequence_active_ = false;
  publish_position_sequence_active(false);
  set_target_position(dribble_position_rad_, PositionState::RETURN_TO_DRIBBLE);
}

void SpringPositionController::limit_switch_callback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  const auto index = static_cast<std::size_t>(limit_switch_index_);
  if (limit_switch_index_ < 0 || index >= msg->data.size()) {
    RCLCPP_ERROR(
        get_logger(),
        "limit_switch_index %d is outside the received array of %zu elements",
        limit_switch_index_, msg->data.size());
    return;
  }
  is_loaded_ = msg->data[index] != 0;
}

void SpringPositionController::position_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (!manual_position_allowed()) {
    return;
  }

  if (msg->data == 0) {
    set_target_position(dribble_position_rad_,
                        PositionState::RETURN_TO_DRIBBLE);
  } else if (msg->data == 1) {
    set_target_position(shoot_position_rad_, PositionState::MANUAL_MOVE);
  } else if (msg->data == 2) {
    set_target_position(max_open_position_rad_, PositionState::MANUAL_MOVE);
  }
}

void SpringPositionController::intake_and_shoot_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || emergency_stop_active_ ||
      operation_mode_ != OperationMode::INTAKE_AND_SHOOT ||
      position_state_ != PositionState::IDLE) {
    return;
  }

  position_sequence_active_ = true;
  publish_position_sequence_active(true);
  set_target_position(intake_position_rad_, PositionState::INTAKE);
}

void SpringPositionController::position_feedback_callback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  if (!std::isfinite(msg->data) || position_state_ == PositionState::IDLE) {
    return;
  }

  last_position_feedback_time_ = now();
  if (std::abs(static_cast<double>(msg->data) - target_position_rad_) >
      position_tolerance_rad_) {
    return;
  }

  switch (position_state_) {
    case PositionState::INTAKE:
      set_target_position(shoot_position_rad_, PositionState::SHOOT);
      break;
    case PositionState::SHOOT:
      position_state_ = PositionState::HOLD_SHOOT;
      position_phase_start_time_ = now();
      break;
    case PositionState::RETURN_TO_DRIBBLE:
      finish_position_move();
      break;
    case PositionState::MANUAL_MOVE:
    case PositionState::MAX_OPEN:
      position_state_ = PositionState::IDLE;
      break;
    case PositionState::IDLE:
    case PositionState::HOLD_SHOOT:
      break;
  }
}

void SpringPositionController::spring_timer_callback() {
  std_msgs::msg::Float32 velocity_command;
  velocity_command.data = 0.0F;

  if (!configuration_valid_ || !spring_operation_allowed()) {
    fire_pending_ = false;
    spring_velocity_publisher_->publish(velocity_command);
    return;
  }

  switch (spring_state_) {
    case SpringState::LOAD:
      if (is_loaded_) {
        spring_state_ = SpringState::READY;
      } else {
        velocity_command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;
    case SpringState::READY:
      if (fire_pending_ && is_loaded_) {
        start_fire();
        velocity_command.data = static_cast<float>(fire_velocity_rad_s_);
      } else if (!is_loaded_) {
        spring_state_ = SpringState::LOAD;
        velocity_command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;
    case SpringState::FIRE:
      velocity_command.data = static_cast<float>(fire_velocity_rad_s_);
      if ((now() - fire_start_time_).seconds() >= fire_duration_sec_) {
        spring_state_ = SpringState::LOAD;
      }
      break;
  }

  spring_velocity_publisher_->publish(velocity_command);
}

void SpringPositionController::position_timer_callback() {
  if (position_state_ == PositionState::IDLE) {
    return;
  }

  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(target_position_rad_);
  position_command_publisher_->publish(command);

  const auto current_time = now();
  const bool feedback_timed_out =
      (current_time - last_position_feedback_time_).seconds() >
      feedback_timeout_sec_;
  const bool move_timed_out =
      position_state_ != PositionState::HOLD_SHOOT &&
      (current_time - position_phase_start_time_).seconds() > move_timeout_sec_;

  if (feedback_timed_out || move_timed_out) {
    set_target_position(dribble_position_rad_,
                        PositionState::RETURN_TO_DRIBBLE);
    return;
  }
  if (position_state_ == PositionState::HOLD_SHOOT &&
      (current_time - position_phase_start_time_).seconds() >=
          shoot_to_dribble_delay_sec_) {
    set_target_position(dribble_position_rad_,
                        PositionState::RETURN_TO_DRIBBLE);
  }
}

bool SpringPositionController::spring_operation_allowed() const {
  return configuration_valid_ && !emergency_stop_active_ &&
         operation_mode_ == OperationMode::DRIVE;
}

bool SpringPositionController::manual_position_allowed() const {
  return !emergency_stop_active_ && position_state_ == PositionState::IDLE &&
         (operation_mode_ == OperationMode::DRIVE ||
          operation_mode_ == OperationMode::INTAKE_AND_SHOOT);
}

void SpringPositionController::stop_spring_state_transition() {
  if (spring_state_ == SpringState::LOAD) {
    spring_state_ = SpringState::READY;
  } else if (spring_state_ == SpringState::FIRE) {
    spring_state_ = SpringState::LOAD;
  }
  fire_pending_ = false;
}

void SpringPositionController::start_fire() {
  spring_state_ = SpringState::FIRE;
  fire_start_time_ = now();
  fire_pending_ = false;
}

void SpringPositionController::set_target_position(double position_rad,
                                                   PositionState state) {
  target_position_rad_ = position_rad;
  position_state_ = state;
  position_phase_start_time_ = now();
  last_position_feedback_time_ = position_phase_start_time_;

  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_publisher_->publish(command);
}

void SpringPositionController::finish_position_move() {
  position_state_ = PositionState::IDLE;
  if (position_sequence_active_) {
    position_sequence_active_ = false;
    publish_position_sequence_active(false);
    publish_operation_mode_complete();
  }
}

void SpringPositionController::publish_operation_mode_complete() {
  std_msgs::msg::Bool message;
  message.data = true;
  operation_mode_complete_publisher_->publish(message);
}

void SpringPositionController::publish_position_sequence_active(bool active) {
  std_msgs::msg::Bool message;
  message.data = active;
  position_sequence_active_publisher_->publish(message);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringPositionController>());
  rclcpp::shutdown();
  return 0;
}
