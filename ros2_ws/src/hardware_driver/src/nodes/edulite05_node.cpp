#include "edulite05_driver/edulite05_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace edulite05_driver
{
using namespace std::chrono_literals;

namespace
{
constexpr char CAN_PUB_TOPIC[] = "/socketcan_bridge/tx";
constexpr char CAN_SUB_TOPIC[] = "/socketcan_bridge/rx";
}  // namespace

Node::Node()
: rclcpp::Node("edulite05_driver")
{
  get_parameters();

  const auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
  const auto can_qos_sub = rclcpp::SensorDataQoS().keep_last(50);

  can_pub_ = create_publisher<can_msgs::msg::Frame>(CAN_PUB_TOPIC, can_qos_pub);
  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    CAN_SUB_TOPIC, can_qos_sub, std::bind(&Node::can_callback, this, std::placeholders::_1));

  target_sub_ = create_subscription<actuator_msgs::msg::ActuatorTarget>(
    target_topic, 10, std::bind(&Node::target_callback, this, std::placeholders::_1));
  target_array_sub_ = create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
    target_array_topic, 10, std::bind(&Node::target_array_callback, this, std::placeholders::_1));

  state_pub_ = create_publisher<actuator_msgs::msg::ActuatorState>(state_topic, 10);
  state_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorStateArray>(state_array_topic, 10);

  update_timer_ = create_wall_timer(10ms, std::bind(&Node::update_callback, this));
  state_timer_ = create_wall_timer(50ms, std::bind(&Node::state_array_callback, this));
}

void Node::get_parameters()
{

  target_topic = declare_parameter<std::string>("target_topic", "/edulite/target");
  target_array_topic = declare_parameter<std::string>("target_array_topic", "/edulite/target_array");
  state_topic = declare_parameter<std::string>("state_topic", "/edulite/state");
  state_array_topic = declare_parameter<std::string>("state_array_topic", "/edulite/state_array");

  const auto names =
    declare_parameter<std::vector<std::string>>("motors", std::vector<std::string>{});

  for (const auto & name : names) {
    const auto prefix = name + ".";
    const auto logical_id = declare_parameter<int64_t>(prefix + "logical_id", -1);
    const auto can_id = declare_parameter<int64_t>(prefix + "can_id", -1);
    const auto mode_name = declare_parameter<std::string>(prefix + "mode", "velocity");
    const auto current_limit = declare_parameter<double>(prefix + "current_limit", 11.0);
    const auto acceleration = declare_parameter<double>(prefix + "acceleration", 150.0);
    const auto speed_limit = declare_parameter<double>(prefix + "speed_limit", 50.0);
    const auto command_period_ms =
      declare_parameter<int64_t>(prefix + "command_period_ms", 10);
    const auto target_timeout_ms =
      declare_parameter<int64_t>(prefix + "target_timeout_ms", 200);
    const auto feedback_timeout_ms =
      declare_parameter<int64_t>(prefix + "feedback_timeout_ms", 500);

    if (logical_id < 0 || logical_id > 65535) {
      throw std::runtime_error(name + ": logical_id must be in [0, 65535]");
    }
    if (can_id < 0 || can_id > 255) {
      throw std::runtime_error(name + ": can_id must be in [0, 255]");
    }
    if (command_period_ms <= 0 || target_timeout_ms <= 0 || feedback_timeout_ms <= 0) {
      throw std::runtime_error(name + ": timeout and period parameters must be positive");
    }
    if (current_limit <= 0.0 || acceleration <= 0.0 || speed_limit <= 0.0) {
      throw std::runtime_error(name + ": motor limits must be positive");
    }

    Mode mode;
    if (mode_name == "velocity") {
      mode = Mode::VELOCITY;
    } else if (mode_name == "pp") {
      mode = Mode::PP;
    } else if (mode_name == "csp") {
      mode = Mode::CSP;
    } else {
      throw std::runtime_error(name + ": unknown mode: " + mode_name);
    }

    const auto duplicate_logical_id = std::any_of(
      motors_.cbegin(), motors_.cend(),
      [logical_id](const Protocol & motor) {
        return motor.get_logical_id() == static_cast<uint16_t>(logical_id);
      });
    const auto duplicate_can_id = std::any_of(
      motors_.cbegin(), motors_.cend(),
      [can_id](const Protocol & motor) {
        return motor.get_can_id() == static_cast<uint8_t>(can_id);
      });
    if (duplicate_logical_id || duplicate_can_id) {
      throw std::runtime_error(name + ": logical_id and can_id must be unique");
    }

    motors_.emplace_back(
      MotorConfig{
        name,
        static_cast<uint16_t>(logical_id),
        static_cast<uint8_t>(can_id),
        mode,
        static_cast<float>(current_limit),
        static_cast<float>(acceleration),
        static_cast<float>(speed_limit),
        static_cast<uint32_t>(command_period_ms),
        static_cast<uint32_t>(target_timeout_ms),
        static_cast<uint32_t>(feedback_timeout_ms)});

    RCLCPP_INFO(
      get_logger(), "%s: logical=%ld CAN=0x%02lX mode=%s period=%ldms",
      name.c_str(), logical_id, can_id, mode_name.c_str(), command_period_ms);
  }
}

Protocol * Node::find_can_id(uint8_t can_id)
{
  const auto it = std::find_if(
    motors_.begin(), motors_.end(),
    [can_id](const Protocol & motor) {return motor.get_can_id() == can_id;});
  return it == motors_.end() ? nullptr : &*it;
}

Protocol * Node::find_logical_id(uint16_t logical_id)
{
  const auto it = std::find_if(
    motors_.begin(), motors_.end(),
    [logical_id](const Protocol & motor) {return motor.get_logical_id() == logical_id;});
  return it == motors_.end() ? nullptr : &*it;
}

void Node::can_callback(can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_extended || msg->dlc < 8) {
    return;
  }

  const auto type = static_cast<uint8_t>((msg->id >> 24) & 0x1F);
  if (type != TYPE_FEEDBACK && type != TYPE_READ) {
    return;
  }

  const auto motor_id = static_cast<uint8_t>((msg->id >> 8) & 0xFF);
  if (auto * motor = find_can_id(motor_id)) {
    if (motor->receive(*msg)) {
      state_pub_->publish(make_state(*motor));
    }
  }
}

void Node::target_callback(actuator_msgs::msg::ActuatorTarget::SharedPtr msg)
{
  if (auto * motor = find_logical_id(msg->logical_id)) {
    motor->set_target(msg->target);
    return;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000, "Unknown logical_id: %u",
    static_cast<unsigned>(msg->logical_id));
}

void Node::target_array_callback(actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg)
{
  for (const auto & actuator : msg->actuators) {
    if (auto * motor = find_logical_id(actuator.logical_id)) {
      motor->set_target(actuator.target);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unknown logical_id: %u",
        static_cast<unsigned>(actuator.logical_id));
    }
  }
}

void Node::update_callback()
{
  if (motors_.empty()) {
    return;
  }

  for (auto & motor : motors_) {
    motor.watchdog();
  }

  auto & init_motor = motors_[init_index_];
  if (auto frame = init_motor.create_initialization_frame()) {
    can_pub_->publish(*frame);
  }
  init_index_ = (init_index_ + 1) % motors_.size();

  for (auto & motor : motors_) {
    if (auto frame = motor.create_target_frame()) {
      can_pub_->publish(*frame);
    }
  }
}

actuator_msgs::msg::ActuatorState Node::make_state(const Protocol & motor) const
{
  actuator_msgs::msg::ActuatorState msg;
  msg.logical_id = motor.get_logical_id();
  msg.connected = motor.is_connected();
  msg.configured = motor.is_configured();
  msg.enabled = motor.is_enabled();

  const auto & feedback = motor.get_feedback();
  msg.position = feedback.position;
  msg.velocity = feedback.velocity;
  msg.effort = feedback.effort;
  msg.temperature = feedback.temperature;
  msg.fault_code = feedback.fault_code;

  switch (motor.get_state()) {
    case MotorState::OFFLINE:
      msg.state = actuator_msgs::msg::ActuatorState::STATE_OFFLINE;
      break;
    case MotorState::INITIALIZING:
      msg.state = actuator_msgs::msg::ActuatorState::STATE_INITIALIZING;
      break;
    case MotorState::READY:
      msg.state = actuator_msgs::msg::ActuatorState::STATE_READY;
      break;
    case MotorState::ERROR:
      msg.state = actuator_msgs::msg::ActuatorState::STATE_ERROR;
      break;
  }
  return msg;
}

void Node::state_array_callback()
{
  actuator_msgs::msg::ActuatorStateArray msg;
  msg.header.stamp = now();
  msg.actuators.reserve(motors_.size());
  for (const auto & motor : motors_) {
    msg.actuators.push_back(make_state(motor));
  }
  state_array_pub_->publish(msg);
}

}  // namespace edulite05_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edulite05_driver::Node>());
  rclcpp::shutdown();
  return 0;
}
