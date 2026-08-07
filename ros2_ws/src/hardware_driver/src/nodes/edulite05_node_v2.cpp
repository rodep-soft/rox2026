#include "edulite05_driver/node.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

constexpr char CAN_PUB_TOPIC[] = "/socketcan_bridge/tx";
constexpr char CAN_SUB_TOPIC[] = "/socketcan_bridge/rx";


Ed05DriverNode::Ed05DriverNode()
: rclcpp::Node("edulite05_driver")
{
  get_parameters();

  auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(50))
      .reliable()
      .durability_volatile();
  auto can_qos_sub = rclcpp::SensorDataQoS();

  can_pub_ =create_publisher<can_msgs::msg::Frame>(CAN_PUB_TOPIC,can_qos_pub);
  can_sub_ =create_subscription<can_msgs::msg::Frame>(CAN_SUB_TOPIC,can_qos_sub,
      std::bind(&Ed05DriverNode::can_callback,this,std::placeholders::_1));


  target_sub_ = create_subscription<actuator_msgs::msg::ActuatorTarget>(
      "/edulite/target",
      10,
      std::bind(&Ed05DriverNode::target_callback,this,std::placeholders::_1));
  target_array_sub_ = create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
      "/edulite/target",
      10,
      std::bind(&Ed05DriverNode::targets_callback,this,std::placeholders::_1));

  state_pub_ = create_publisher<actuator_msgs::msg::ActuatorState>(
      "/edulite/state",
      10);
  state_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorStateArray>(
      "/edulite/state",
      10);

  update_timer_ = create_wall_timer(
      10ms,
      std::bind(&Ed05DriverNode::update_callback,this));

  state_timer_ = create_wall_timer(
      50ms,
      std::bind(&Ed05DriverNode::state_callback,this));
}


void Ed05DriverNode::get_parameters()
{
  const auto names = declare_parameter<std::vector<std::string>>("motors");

  for (const auto & name : names) {
    const auto prefix = name + ".";

    const auto logical_id = declare_parameter<uint16_t>(prefix + "logical_id",-1);

    const auto can_id = declare_parameter<uint8_t>(prefix + "can_id",-1);

    const auto mode_name = declare_parameter<std::string>(prefix + "mode","velocity");

    const auto current_limit = declare_parameter<double>(prefix + "current_limit",11.0);

    const auto acceleration = declare_parameter<double>(prefix + "acceleration",150.0);

    const auto speed_limit = declare_parameter<double>(prefix + "speed_limit",50.0);

    const auto command_period_ms = declare_parameter<uint32_t>(prefix + "command_period_ms",10);

    const auto target_timeout_ms = declare_parameter<uint32_t>(prefix + "target_timeout_ms",200);

    const auto feedback_timeout_ms =declare_parameter<uint32_t>(prefix + "feedback_timeout_ms",500);

    if (logical_id < 0 || logical_id > 65535)
    {
      throw std::runtime_error(name + ": invalid logical_id");
    }
    if (can_id < 0 || can_id > 255)
    {
      throw std::runtime_error(name +": invalid can_id");
    }
    
    Mode mode;

    if (mode_name == "velocity") {
      mode = Mode::VELOCITY;
    } else if (mode_name == "pp") {
      mode = Mode::PP;
    } else if (mode_name == "csp") {
      mode = Mode::CSP;
    } else {
      throw std::runtime_error(
        "Unknown mode: " + mode_name);
    }

    MotorConfig config{
      name,
      logical_id,
      can_id,
      mode,
      static_cast<float>(current_limit),
      static_cast<float>(acceleration),
      static_cast<float>(speed_limit),
      command_period_ms,
      target_timeout_ms,
      feedback_timeout_ms,
    };

    motors_.emplace_back(config);

    RCLCPP_INFO(get_logger(),
      "%s: logical=%d CAN=0x%02X mode=%s period=%ldms",
      name.c_str(),logical_id,can_id,mode_name.c_str(),command_period_ms);
  }
}


Protocol * Ed05DriverNode::find_can_id(uint8_t can_id)
{
  const auto it = std::find_if(
      motors_.begin(),
      motors_.end(),
      [can_id](const Protocol & motor) {
        return motor.can_id() == can_id;
      });

  return it == motors_.end()? nullptr : &*it;
}

Protocol * Ed05DriverNode::find_logical_id(uint16_t logical_id)
{
  const auto it = std::find_if(
      motors_.begin(),
      motors_.end(),
      [logical_id](const Protocol & motor) {
        return motor.logical_id() == logical_id;
      });

  return it == motors_.end() ? nullptr : &*it;
}


void Ed05DriverNode::can_callback(can_msgs::msg::Frame::SharedPtr msg)
{
  if (!msg->is_extended) {
    return;
  }
  const uint8_t type = static_cast<uint8_t>((msg->id >> 24) & 0x1F);
  // 必要な応答だけ処理を行う
  if (type != 0x02 && type != 0x11)
  {
    return;
  }
  // Type2 / Type17 replyではbit15:8がmotor CAN ID
  const uint8_t motor_id = static_cast<uint8_t>((msg->id >> 8) & 0xFF);
  if (auto * motor = find_can_id(motor_id)) {
    const bool feedback_updated = motor->receive(*msg);
    // Type2を受けた瞬間にそのモータだけ配信する
    if (feedback_updated) {
      state_pub_->publish(make_state(*motor));
    }
  }
}


void Ed05DriverNode::target_callback(actuator_msgs::msg::ActuatorTarget::SharedPtr msg)
{
  if (auto * motor = find_logical_id(msg->logical_id))
  {
      motor->set_target(msg->target);
  }
}

void Ed05DriverNode::targets_callback(actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg)
{
  for (const auto & actuator : msg->actuators) {
    if (auto * motor = find_logical_id(actuator.logical_id)) {
      motor->set_target(actuator.target);
    }
  }
}

void Ed05DriverNode::set_target(uint16_t logical_id,float target)
{
  auto * motor = find_logical_id(logical_id);
  if (!motor) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unknown logical_id: %u", static_cast<unsigned>(logical_id));
    return;
  }
    motor->set_target(target);
}

void Ed05DriverNode::update_callback()
{
  if (motors_.empty()) {
    return;
  }

  const auto time = now();

  // 通信断監視
  for (auto & motor : motors_) {
    motor.watchdog(time);
  }

  // 初期化は1周期につき1台だけ進める
   auto & init_motor = motors_[init_index_];
  if (auto frame = init_motor.initialization_frame(time)) {
    can_pub_->publish(*frame);
  }
  init_index_ = (init_index_ + 1) % motors_.size();

  // READYモータへ通常指令
  for (const auto & motor : motors_) {
    if (auto frame = motor.target_frame()) {
      can_pub_->publish(*frame);
    }
  }
}

actuator_msgs::msg::ActuatorState Ed05DriverNode::make_state(const Protocol & motor) const
{
  actuator_msgs::msg::ActuatorState msg;

  msg.logical_id = motor.logical_id();
  msg.connected = motor.connected();
  msg.configured = motor.configured();
  msg.enabled = motor.enabled();
  const auto & feedback = motor.feedback();
  msg.position = feedback.position;
  msg.velocity = feedback.velocity;
  msg.effort = feedback.effort;
  msg.temperature = feedback.temperature;
  msg.fault_code = feedback.fault_code;

  switch (motor.state()) {
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

void Ed05DriverNode::states_callback()
{
  actuator_msgs::msg::ActuatorStateArray msg;
  msg.header.stamp = now();
  msg.actuators.reserve(motors_.size());

  for (const auto & motor : motors_) {
    msg.actuators.push_back(make_state(motor));
  }
    states_pub_->publish(msg);
}  

// namespace edulite05_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<edulite05_driver::Ed05DriverNode>());
  rclcpp::shutdown();
  return 0;
}