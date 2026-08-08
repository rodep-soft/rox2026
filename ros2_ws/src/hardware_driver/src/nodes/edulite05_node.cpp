#include "edulite05_driver/edulite05_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace edulite05_driver {
using namespace std::chrono_literals;

Node::Node() : rclcpp::Node("edulite05_driver") {
  declare_and_load_parameters();
  create_interfaces();
}

void Node::declare_and_load_parameters() {
  can_tx_topic_ =
      declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
  can_rx_topic_ =
      declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
  target_topic_ =
      declare_parameter<std::string>("target_topic", "/edulite/target");
  target_array_topic_ = declare_parameter<std::string>("target_array_topic",
                                                       "/edulite/target_array");
  state_topic_ =
      declare_parameter<std::string>("state_topic", "/edulite/state");
  state_array_topic_ = declare_parameter<std::string>("state_array_topic",
                                                      "/edulite/state_array");
  set_position_service_name_ = declare_parameter<std::string>(
      "set_position_service", "/edulite/set_position");

  const auto motor_names = declare_parameter<std::vector<std::string>>(
      "motors", std::vector<std::string>{});
  for (const auto &motor_name : motor_names) {
    const auto prefix = motor_name + ".";
    const auto logical_id =
        declare_parameter<int64_t>(prefix + "logical_id", -1);
    const auto can_id = declare_parameter<int64_t>(prefix + "can_id", -1);
    const auto control_mode_name =
        declare_parameter<std::string>(prefix + "control_mode", "velocity");
    const auto current_limit =
        declare_parameter<double>(prefix + "current_limit", 11.0);
    const auto acceleration =
        declare_parameter<double>(prefix + "acceleration", 150.0);
    const auto speed_limit =
        declare_parameter<double>(prefix + "speed_limit", 50.0);
    const auto command_period_ms =
        declare_parameter<int64_t>(prefix + "command_period_ms", 10);
    const auto target_timeout_ms =
        declare_parameter<int64_t>(prefix + "target_timeout_ms", 200);
    const auto feedback_timeout_ms =
        declare_parameter<int64_t>(prefix + "feedback_timeout_ms", 500);
    const auto reference_mode_name = declare_parameter<std::string>(
        prefix + "position_reference_mode", "service");
    const auto startup_absolute_position_rad = declare_parameter<double>(
        prefix + "startup_absolute_position_rad", 0.0);
    const auto minimum_position_rad =
        declare_parameter<double>(prefix + "minimum_position_rad", -1000.0);
    const auto maximum_position_rad =
        declare_parameter<double>(prefix + "maximum_position_rad", 1000.0);

    if (logical_id < 0 || logical_id > 65535) {
      throw std::runtime_error(motor_name +
                               ": logical_id must be in [0, 65535]");
    }
    if (can_id < 0 || can_id > 255) {
      throw std::runtime_error(motor_name + ": can_id must be in [0, 255]");
    }
    if (command_period_ms <= 0 || target_timeout_ms <= 0 ||
        feedback_timeout_ms <= 0) {
      throw std::runtime_error(motor_name +
                               ": periods and timeouts must be positive");
    }
    if (current_limit <= 0.0 || acceleration <= 0.0 || speed_limit <= 0.0) {
      throw std::runtime_error(motor_name + ": motor limits must be positive");
    }
    if (minimum_position_rad > maximum_position_rad) {
      throw std::runtime_error(
          motor_name +
          ": minimum_position_rad must not exceed maximum_position_rad");
    }

    ControlMode control_mode;
    if (control_mode_name == "velocity") {
      control_mode = ControlMode::VELOCITY;
    } else if (control_mode_name == "pp") {
      control_mode = ControlMode::PROFILE_POSITION;
    } else if (control_mode_name == "csp") {
      control_mode = ControlMode::CYCLIC_SYNCHRONOUS_POSITION;
    } else {
      throw std::runtime_error(motor_name +
                               ": unknown control_mode: " + control_mode_name);
    }

    PositionReferenceMode position_reference_mode;
    if (reference_mode_name == "service") {
      position_reference_mode = PositionReferenceMode::SERVICE;
    } else if (reference_mode_name == "yaml_absolute") {
      position_reference_mode = PositionReferenceMode::YAML_ABSOLUTE;
    } else {
      throw std::runtime_error(
          motor_name +
          ": position_reference_mode must be service or yaml_absolute");
    }
    if (control_mode == ControlMode::VELOCITY &&
        position_reference_mode != PositionReferenceMode::SERVICE) {
      throw std::runtime_error(
          motor_name +
          ": position_reference_mode is only valid for PP/CSP motors");
    }

    const auto duplicate_logical_id = std::any_of(
        motors_.cbegin(), motors_.cend(), [logical_id](const Protocol &motor) {
          return motor.logical_id() == static_cast<uint16_t>(logical_id);
        });
    const auto duplicate_can_id = std::any_of(
        motors_.cbegin(), motors_.cend(), [can_id](const Protocol &motor) {
          return motor.can_id() == static_cast<uint8_t>(can_id);
        });
    if (duplicate_logical_id || duplicate_can_id) {
      throw std::runtime_error(motor_name +
                               ": logical_id and can_id must be unique");
    }

    motors_.emplace_back(MotorConfig{
        motor_name, static_cast<uint16_t>(logical_id),
        static_cast<uint8_t>(can_id), control_mode,
        static_cast<float>(current_limit), static_cast<float>(acceleration),
        static_cast<float>(speed_limit),
        static_cast<uint32_t>(command_period_ms),
        static_cast<uint32_t>(target_timeout_ms),
        static_cast<uint32_t>(feedback_timeout_ms), position_reference_mode,
        static_cast<float>(startup_absolute_position_rad),
        static_cast<float>(minimum_position_rad),
        static_cast<float>(maximum_position_rad)});

    RCLCPP_INFO(
        get_logger(),
        "%s: logical=%ld CAN=0x%02lX control_mode=%s position_reference=%s",
        motor_name.c_str(), logical_id, can_id, control_mode_name.c_str(),
        reference_mode_name.c_str());
  }
}

void Node::create_interfaces() {
  const auto can_tx_qos =
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
  const auto can_rx_qos = rclcpp::SensorDataQoS().keep_last(50);

  // EduLite 05向けCANフレームをsocketcan bridgeへ送信する。
  can_frame_publisher_ =
      create_publisher<can_msgs::msg::Frame>(can_tx_topic_, can_tx_qos);
  // socketcan bridgeから全CANフレームを受信し、対象モーターの状態を復元する。
  can_frame_subscription_ = create_subscription<can_msgs::msg::Frame>(
      can_rx_topic_, can_rx_qos,
      std::bind(&Node::can_frame_callback, this, std::placeholders::_1));
  // 上位controllerから単体モーターの速度または位置指令を受信する。
  target_subscription_ =
      create_subscription<actuator_msgs::msg::ActuatorTarget>(
          target_topic_, 10,
          std::bind(&Node::target_callback, this, std::placeholders::_1));
  // 上位controllerから複数モーターの指令をまとめて受信する。
  target_array_subscription_ =
      create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
          target_array_topic_, 10,
          std::bind(&Node::target_array_callback, this, std::placeholders::_1));
  // CAN受信時に更新された単体モーター状態を上位ノードへ送信する。
  state_publisher_ =
      create_publisher<actuator_msgs::msg::ActuatorState>(state_topic_, 10);
  // 全モーター状態の周期スナップショットを上位ノードへ送信する。
  state_array_publisher_ =
      create_publisher<actuator_msgs::msg::ActuatorStateArray>(
          state_array_topic_, 10);
  // 上位ノードからPP/CSPモーターの現在位置を指定して位置基準を設定する。
  set_position_service_ = create_service<actuator_msgs::srv::SetPosition>(
      set_position_service_name_,
      std::bind(&Node::set_position_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  command_timer_ =
      create_wall_timer(10ms, std::bind(&Node::command_timer_callback, this));
  state_timer_ =
      create_wall_timer(50ms, std::bind(&Node::state_timer_callback, this));
}

void Node::can_frame_callback(can_msgs::msg::Frame::SharedPtr message) {
  if (!message->is_extended || message->dlc < 8) {
    return;
  }
  const auto type = static_cast<uint8_t>((message->id >> 24) & 0x1F);
  if (type != TYPE_FEEDBACK && type != TYPE_READ) {
    return;
  }
  const auto motor_id = static_cast<uint8_t>((message->id >> 8) & 0xFF);
  if (auto *motor = find_motor_by_can_id(motor_id);
      motor != nullptr && motor->receive(*message)) {
    state_publisher_->publish(make_state_message(*motor));
  }
}

void Node::target_callback(
    actuator_msgs::msg::ActuatorTarget::SharedPtr message) {
  if (auto *motor = find_motor_by_logical_id(message->logical_id)) {
    motor->set_target(message->target);
    return;
  }
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                       "Unknown logical_id: %u",
                       static_cast<unsigned>(message->logical_id));
}

void Node::target_array_callback(
    actuator_msgs::msg::ActuatorTargetArray::SharedPtr message) {
  for (const auto &target : message->actuators) {
    if (auto *motor = find_motor_by_logical_id(target.logical_id)) {
      motor->set_target(target.target);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unknown logical_id: %u",
                           static_cast<unsigned>(target.logical_id));
    }
  }
}

void Node::set_position_callback(
    const std::shared_ptr<actuator_msgs::srv::SetPosition::Request> request,
    std::shared_ptr<actuator_msgs::srv::SetPosition::Response> response) {
  auto *motor = find_motor_by_logical_id(request->logical_id);
  if (motor == nullptr) {
    response->success = false;
    response->message = "Unknown logical_id";
    return;
  }
  if (!std::isfinite(request->position)) {
    response->success = false;
    response->message = "Position must be finite";
    return;
  }
  if (motor->control_mode() == ControlMode::VELOCITY) {
    response->success = false;
    response->message = "Position reference is only supported in PP/CSP mode";
    return;
  }
  if (!motor->set_current_position(request->position)) {
    response->success = false;
    response->message = "Motor feedback has not been received";
    return;
  }
  response->success = true;
  response->message = "Position reference updated";
}

void Node::command_timer_callback() {
  if (motors_.empty()) {
    return;
  }
  for (auto &motor : motors_) {
    motor.watchdog();
  }
  auto &initialization_motor = motors_[initialization_motor_index_];
  if (auto frame = initialization_motor.create_initialization_frame()) {
    can_frame_publisher_->publish(*frame);
  }
  initialization_motor_index_ =
      (initialization_motor_index_ + 1) % motors_.size();
  for (auto &motor : motors_) {
    if (auto frame = motor.create_target_frame()) {
      can_frame_publisher_->publish(*frame);
    }
  }
}

void Node::state_timer_callback() {
  actuator_msgs::msg::ActuatorStateArray message;
  message.header.stamp = now();
  message.actuators.reserve(motors_.size());
  for (const auto &motor : motors_) {
    message.actuators.push_back(make_state_message(motor));
  }
  state_array_publisher_->publish(message);
}

actuator_msgs::msg::ActuatorState
Node::make_state_message(const Protocol &motor) const {
  actuator_msgs::msg::ActuatorState message;
  message.logical_id = motor.logical_id();
  message.connected = motor.is_connected();
  message.configured = motor.is_configured();
  message.enabled = motor.is_enabled();
  message.position_reference_set = motor.position_reference_is_set();
  const auto &feedback = motor.feedback();
  message.position = feedback.position;
  message.velocity = feedback.velocity;
  message.effort = feedback.effort;
  message.temperature = feedback.temperature;
  message.fault_code = feedback.fault_code;
  switch (motor.state()) {
  case MotorState::OFFLINE:
    message.state = actuator_msgs::msg::ActuatorState::STATE_OFFLINE;
    break;
  case MotorState::INITIALIZING:
    message.state = actuator_msgs::msg::ActuatorState::STATE_INITIALIZING;
    break;
  case MotorState::READY:
    message.state = actuator_msgs::msg::ActuatorState::STATE_READY;
    break;
  case MotorState::ERROR:
    message.state = actuator_msgs::msg::ActuatorState::STATE_ERROR;
    break;
  }
  return message;
}

Protocol *Node::find_motor_by_can_id(uint8_t can_id) {
  const auto iterator = std::find_if(
      motors_.begin(), motors_.end(),
      [can_id](const Protocol &motor) { return motor.can_id() == can_id; });
  return iterator == motors_.end() ? nullptr : &*iterator;
}

Protocol *Node::find_motor_by_logical_id(uint16_t logical_id) {
  const auto iterator = std::find_if(motors_.begin(), motors_.end(),
                                     [logical_id](const Protocol &motor) {
                                       return motor.logical_id() == logical_id;
                                     });
  return iterator == motors_.end() ? nullptr : &*iterator;
}
} // namespace edulite05_driver

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edulite05_driver::Node>());
  rclcpp::shutdown();
  return 0;
}
