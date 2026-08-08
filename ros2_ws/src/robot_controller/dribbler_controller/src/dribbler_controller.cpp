#include "dribbler_controller/dribbler_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

DribblerControllerNode::DribblerControllerNode()
: Node("dribbler_controller_node")
{
  declare_parameters();
  get_parameters();

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos = rclcpp::QoS(qos_depth_);

  dribble_target_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    target_topic_, command_qos);

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/enabled", command_qos,
    std::bind(&DribblerControllerNode::dribble_enabled_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&DribblerControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribblerControllerNode::timer_callback, this));
}

void DribblerControllerNode::declare_parameters()
{
  declare_parameter<int>("dribble_on_rpm", 2000);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
  declare_parameter<int>("logical_id", 12);
  declare_parameter<std::string>("target_topic", "/vesc/target");
}

void DribblerControllerNode::get_parameters()
{
  get_parameter("dribble_on_rpm", dribble_on_rpm_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
  const auto logical_id = get_parameter("logical_id").as_int();
  get_parameter("target_topic", target_topic_);
  if (logical_id < 0 || logical_id > 65535) {
    throw std::runtime_error("logical_id must be in [0, 65535]");
  }
  if (target_topic_.empty()) {
    throw std::runtime_error("target_topic must not be empty");
  }
  logical_id_ = static_cast<uint16_t>(logical_id);
}

void DribblerControllerNode::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

void DribblerControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
}

void DribblerControllerNode::timer_callback()
{
  actuator_msgs::msg::ActuatorTarget target;
  target.logical_id = logical_id_;
  target.target = static_cast<float>(
    dribble_enabled_ && !emergency_stop_active_ ? dribble_on_rpm_ : 0);
  dribble_target_pub_->publish(target);
}
