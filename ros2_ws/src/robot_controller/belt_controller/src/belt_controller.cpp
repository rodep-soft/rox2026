#include "belt_controller/belt_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  declare_parameters();
  get_parameters();
  if (command_period_ms_ <= 0) {command_period_ms_ = 10;}

  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&BeltControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos,
    std::bind(&BeltControllerNode::belt_mode_callback, this, std::placeholders::_1));

  belt_target_rpm_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/belt/target_rpm", command_qos,
    std::bind(&BeltControllerNode::belt_target_rpm_callback, this, std::placeholders::_1));

  target_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorTargetArray>(
    target_array_topic_, command_qos);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&BeltControllerNode::timer_callback, this));
}

void BeltControllerNode::declare_parameters()
{
  declare_parameter<int>("level_1_rpm", 3000);
  declare_parameter<int>("level_2_rpm", 3500);
  declare_parameter<int>("level_3_rpm", 4000);
  declare_parameter<int>("level_4_rpm", 4500);
  declare_parameter<int>("level_5_rpm", 5000);
  declare_parameter<int>("level_6_rpm", 5500);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
  declare_parameter<int>("underbelt_logical_id", 11);
  declare_parameter<int>("upperbelt_logical_id", 10);
  declare_parameter<std::string>("target_array_topic", "/vesc/target_array");
}

void BeltControllerNode::get_parameters()
{
  int rpm = 0;
  get_parameter("level_1_rpm", rpm); level_rpms_[0] = rpm;
  get_parameter("level_2_rpm", rpm); level_rpms_[1] = rpm;
  get_parameter("level_3_rpm", rpm); level_rpms_[2] = rpm;
  get_parameter("level_4_rpm", rpm); level_rpms_[3] = rpm;
  get_parameter("level_5_rpm", rpm); level_rpms_[4] = rpm;
  get_parameter("level_6_rpm", rpm); level_rpms_[5] = rpm;
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
  const auto underbelt_logical_id = get_parameter("underbelt_logical_id").as_int();
  const auto upperbelt_logical_id = get_parameter("upperbelt_logical_id").as_int();
  get_parameter("target_array_topic", target_array_topic_);
  if (underbelt_logical_id < 0 || underbelt_logical_id > 65535 ||
    upperbelt_logical_id < 0 || upperbelt_logical_id > 65535 ||
    underbelt_logical_id == upperbelt_logical_id)
  {
    throw std::runtime_error("belt logical IDs must be unique values in [0, 65535]");
  }
  if (target_array_topic_.empty()) {
    throw std::runtime_error("target_array_topic must not be empty");
  }
  underbelt_logical_id_ = static_cast<uint16_t>(underbelt_logical_id);
  upperbelt_logical_id_ = static_cast<uint16_t>(upperbelt_logical_id);
}

void BeltControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  use_direct_target_rpm_ = false;
  belt_mode_ = (msg->data <= static_cast<uint8_t>(BeltMode::LEVEL_6)) ?
    static_cast<BeltMode>(msg->data) :
    BeltMode::STOP;
}

void BeltControllerNode::belt_target_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (msg->data > 0.0f) {
    use_direct_target_rpm_ = true;
    direct_target_rpm_ = static_cast<int>(msg->data);
  } else {
    use_direct_target_rpm_ = false;
    direct_target_rpm_ = 0;
  }
}

void BeltControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
}

void BeltControllerNode::timer_callback()
{
  const auto target_rpm = static_cast<float>(emergency_stop_active_ ? 0 : belt_target_rpm());
  actuator_msgs::msg::ActuatorTargetArray command;
  command.header.stamp = now();
  command.actuators.resize(2);
  command.actuators[0].logical_id = underbelt_logical_id_;
  command.actuators[0].target = target_rpm;
  command.actuators[1].logical_id = upperbelt_logical_id_;
  command.actuators[1].target = target_rpm;
  target_array_pub_->publish(command);
}

int BeltControllerNode::belt_target_rpm() const
{
  if (use_direct_target_rpm_) {
    return direct_target_rpm_;
  }

  const auto level = static_cast<uint8_t>(belt_mode_);
  if (level == 0 || level > kNumLevels) {
    return 0;
  }
  return level_rpms_[level - 1];
}
