#include "belt_controller/belt_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  load_parameters();

  const auto command_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  const auto request_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();

  belt_mode_sub_ = create_subscription<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", request_qos,
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
    target_array_topic_, command_qos);

  // 定量評価用ベルトステータストピック (/belt/status)
  status_pub_ = create_publisher<robot_msgs::msg::BeltStatus>(
    "/belt/status", command_qos);

  command_timer_ = create_wall_timer(
    std::chrono::milliseconds(emergency_stop_period_ms_),
    std::bind(&BeltControllerNode::command_timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&BeltControllerNode::parameter_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "BeltControllerNode initialized. Monitoring topic: /belt/status");
}

void BeltControllerNode::load_parameters()
{
  underbelt_rpms_[0] = declare_parameter<int>("underbelt_level_1_rpm", 3000);
  underbelt_rpms_[1] = declare_parameter<int>("underbelt_level_2_rpm", 3500);
  underbelt_rpms_[2] = declare_parameter<int>("underbelt_level_3_rpm", 4000);
  underbelt_rpms_[3] = declare_parameter<int>("underbelt_level_4_rpm", 4500);
  upperbelt_rpms_[0] = declare_parameter<int>("upperbelt_level_1_rpm", 3000);
  upperbelt_rpms_[1] = declare_parameter<int>("upperbelt_level_2_rpm", 3500);
  upperbelt_rpms_[2] = declare_parameter<int>("upperbelt_level_3_rpm", 4000);
  upperbelt_rpms_[3] = declare_parameter<int>("upperbelt_level_4_rpm", 4500);

  emergency_stop_period_ms_ = declare_parameter<int>("emergency_stop_period_ms", 50);
  const auto underbelt_logical_id = declare_parameter<int>("underbelt_logical_id", 11);
  const auto upperbelt_logical_id = declare_parameter<int>("upperbelt_logical_id", 10);
  target_array_topic_ = declare_parameter<std::string>(
    "target_array_topic",
    "/vesc/target_array");

  if (emergency_stop_period_ms_ <= 0 || target_array_topic_.empty()) {
    throw std::runtime_error("Invalid belt parameters: timing or topic is invalid");
  }
  if (underbelt_logical_id < 0 || underbelt_logical_id > 65535 ||
    upperbelt_logical_id < 0 || upperbelt_logical_id > 65535)
  {
    throw std::runtime_error("Invalid belt parameters: logical IDs must be in [0, 65535]");
  }
  for (std::size_t i = 0; i < num_levels; ++i) {
    if (underbelt_rpms_[i] < 0 || upperbelt_rpms_[i] < 0) {
      throw std::runtime_error("Invalid belt parameters: RPM must be non-negative");
    }
  }

  underbelt_logical_id_ = static_cast<uint16_t>(underbelt_logical_id);
  upperbelt_logical_id_ = static_cast<uint16_t>(upperbelt_logical_id);
}

void BeltControllerNode::belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg)
{
  use_direct_target_rpm_ = false;
  belt_mode_ = msg->mode <=
    robot_msgs::msg::BeltMode::LEVEL_4 ? msg->mode : robot_msgs::msg::BeltMode::STOP;
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

void BeltControllerNode::vesc_state_callback(
  const actuator_msgs::msg::ActuatorStateArray::SharedPtr msg)
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

void BeltControllerNode::command_timer_callback()
{
  // Keep refreshing the VESC target before hardware_driver's command timeout.
  // publish_command() already sends zero RPM while emergency stop is active.
  publish_command();
}

rcl_interfaces::msg::SetParametersResult BeltControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool rpm_updated = false;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    // 再起動が必要なパラメータ（同じ値なら許可、変更時は拒否）
    if (name == "emergency_stop_period_ms") {
      if (param.as_int() != emergency_stop_period_ms_) {
        result.successful = false;
        result.reason = name + " requires a node restart";
        return result;
      }
      continue;
    }
    if (name == "underbelt_logical_id") {
      if (static_cast<uint16_t>(param.as_int()) != underbelt_logical_id_) {
        result.successful = false;
        result.reason = name + " requires a node restart";
        return result;
      }
      continue;
    }
    if (name == "upperbelt_logical_id") {
      if (static_cast<uint16_t>(param.as_int()) != upperbelt_logical_id_) {
        result.successful = false;
        result.reason = name + " requires a node restart";
        return result;
      }
      continue;
    }
    if (name == "target_array_topic") {
      if (param.as_string() != target_array_topic_) {
        result.successful = false;
        result.reason = name + " requires a node restart";
        return result;
      }
      continue;
    }

    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      for (std::size_t level = 0; level < num_levels; ++level) {
        const auto level_str = std::to_string(level + 1);
        if (name == "underbelt_level_" + level_str + "_rpm") {
          const int val = static_cast<int>(param.as_int());
          if (val < 0) {
            result.successful = false;
            result.reason = "RPM must be non-negative";
            return result;
          }
          if (underbelt_rpms_[level] != val) {
            underbelt_rpms_[level] = val;
            rpm_updated = true;
            RCLCPP_INFO(
              get_logger(), "Updated %s: %d RPM", name.c_str(), val);
          }
        } else if (name == "upperbelt_level_" + level_str + "_rpm") {
          const int val = static_cast<int>(param.as_int());
          if (val < 0) {
            result.successful = false;
            result.reason = "RPM must be non-negative";
            return result;
          }
          if (upperbelt_rpms_[level] != val) {
            upperbelt_rpms_[level] = val;
            rpm_updated = true;
            RCLCPP_INFO(
              get_logger(), "Updated %s: %d RPM", name.c_str(), val);
          }
        }
      }
    }
  }

  if (rpm_updated) {
    publish_command();
  }
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
