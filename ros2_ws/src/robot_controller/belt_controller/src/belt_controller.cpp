#include "belt_controller/belt_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  underbelt_rpms_[0] = declare_parameter<int>("underbelt_level_1_rpm", 3000);
  underbelt_rpms_[1] = declare_parameter<int>("underbelt_level_2_rpm", 3500);
  underbelt_rpms_[2] = declare_parameter<int>("underbelt_level_3_rpm", 4000);
  underbelt_rpms_[3] = declare_parameter<int>("underbelt_level_4_rpm", 4500);
  upperbelt_rpms_[0] = declare_parameter<int>("upperbelt_level_1_rpm", 3000);
  upperbelt_rpms_[1] = declare_parameter<int>("upperbelt_level_2_rpm", 3500);
  upperbelt_rpms_[2] = declare_parameter<int>("upperbelt_level_3_rpm", 4000);
  upperbelt_rpms_[3] = declare_parameter<int>("upperbelt_level_4_rpm", 4500);

  const auto emergency_stop_period_ms = declare_parameter<int>("emergency_stop_period_ms", 50);
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto underbelt_logical_id = declare_parameter<int>("underbelt_logical_id", 11);
  const auto upperbelt_logical_id = declare_parameter<int>("upperbelt_logical_id", 10);
  const auto target_array_topic   = declare_parameter<std::string>("target_array_topic", "/vesc/target_array");

  if (emergency_stop_period_ms <= 0 || qos_depth <= 0 || target_array_topic.empty()) {
    throw std::runtime_error("Invalid belt parameters");
  }

  underbelt_logical_id_ = static_cast<uint16_t>(underbelt_logical_id);
  upperbelt_logical_id_ = static_cast<uint16_t>(upperbelt_logical_id);

  const auto command_qos = rclcpp::QoS(qos_depth);
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();

  belt_mode_sub_ = create_subscription<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", command_qos,
    std::bind(&BeltControllerNode::belt_mode_callback, this, std::placeholders::_1));

  belt_target_rpm_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/belt/command_rpm", command_qos,
    std::bind(&BeltControllerNode::belt_target_rpm_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", emergency_stop_qos,
    std::bind(&BeltControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  vesc_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorStateArray>(
    "/vesc/state_array", command_qos,
    std::bind(&BeltControllerNode::vesc_state_callback, this, std::placeholders::_1));

  target_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorTargetArray>(
    target_array_topic, command_qos);

  // 定量評価用ベルトステータストピック (/belt/status)
  status_pub_ = create_publisher<robot_msgs::msg::BeltStatus>(
    "/belt/status", command_qos);

  emergency_stop_timer_ = create_wall_timer(
    std::chrono::milliseconds(emergency_stop_period_ms),
    std::bind(&BeltControllerNode::emergency_stop_timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&BeltControllerNode::parameter_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "BeltControllerNode initialized. Monitoring topic: /belt/status");
}

void BeltControllerNode::belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg)
{
  use_direct_target_rpm_ = false;
  belt_mode_ = msg->mode <= robot_msgs::msg::BeltMode::LEVEL_4 ? msg->mode : robot_msgs::msg::BeltMode::STOP;
  publish_command();
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
  publish_command();
}

void BeltControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  publish_command();
}

void BeltControllerNode::vesc_state_callback(const actuator_msgs::msg::ActuatorStateArray::SharedPtr msg)
{
  for (const auto & state : msg->actuators) {
    if (state.logical_id == underbelt_logical_id_) {
      underbelt_measured_rpm_ = state.velocity;
    } else if (state.logical_id == upperbelt_logical_id_) {
      upperbelt_measured_rpm_ = state.velocity;
    }
  }

  // 実測値受信時に /belt/status をリアルタイム配信（定量評価用）
  robot_msgs::msg::BeltStatus status;
  status.header.stamp = now();
  status.underbelt_target_rpm = last_underbelt_target_rpm_;
  status.underbelt_measured_rpm = underbelt_measured_rpm_;
  status.upperbelt_target_rpm = last_upperbelt_target_rpm_;
  status.upperbelt_measured_rpm = upperbelt_measured_rpm_;
  status.rpm_difference = std::abs(underbelt_measured_rpm_ - upperbelt_measured_rpm_);
  status_pub_->publish(status);
}

void BeltControllerNode::emergency_stop_timer_callback()
{
  if (emergency_stop_active_) {
    publish_command();
  }
}

rcl_interfaces::msg::SetParametersResult BeltControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    if (name == "emergency_stop_period_ms" || name == "qos_depth" ||
        name == "underbelt_logical_id" || name == "upperbelt_logical_id" ||
        name == "target_array_topic")
    {
      result.successful = false;
      result.reason = name + " requires a node restart";
      return result;
    }

    for (std::size_t level = 0; level < num_levels; ++level) {
      const auto level_str = std::to_string(level + 1);
      if (name == "underbelt_level_" + level_str + "_rpm") {
        const int val = static_cast<int>(param.as_int());
        if (val < 0) { result.successful = false; result.reason = "RPM must be non-negative"; return result; }
        underbelt_rpms_[level] = val;
      } else if (name == "upperbelt_level_" + level_str + "_rpm") {
        const int val = static_cast<int>(param.as_int());
        if (val < 0) { result.successful = false; result.reason = "RPM must be non-negative"; return result; }
        upperbelt_rpms_[level] = val;
      }
    }
  }

  publish_command();
  return result;
}

void BeltControllerNode::publish_command()
{
  int underbelt_rpm = 0;
  int upperbelt_rpm = 0;

  if (!emergency_stop_active_) {
    if (use_direct_target_rpm_) {
      underbelt_rpm = direct_target_rpm_;
      upperbelt_rpm = direct_target_rpm_;
    } else {
      const auto level = static_cast<std::size_t>(belt_mode_);
      if (level >= 1 && level <= num_levels) {
        underbelt_rpm = underbelt_rpms_[level - 1];
        upperbelt_rpm = upperbelt_rpms_[level - 1];
      }
    }
  }

  last_underbelt_target_rpm_ = static_cast<float>(underbelt_rpm);
  last_upperbelt_target_rpm_ = static_cast<float>(upperbelt_rpm);

  actuator_msgs::msg::ActuatorTargetArray command;
  command.header.stamp = now();
  command.actuators.resize(2);
  command.actuators[0].logical_id = underbelt_logical_id_;
  command.actuators[0].target = last_underbelt_target_rpm_;
  command.actuators[1].logical_id = upperbelt_logical_id_;
  command.actuators[1].target = last_upperbelt_target_rpm_;
  target_array_pub_->publish(command);
}
