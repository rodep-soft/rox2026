#include "belt_controller/belt_controller.hpp"

#include <chrono>
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

  const auto emergency_stop_period_ms =
    declare_parameter<int>("emergency_stop_period_ms", 50);
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto underbelt_logical_id =
    declare_parameter<int>("underbelt_logical_id", 11);
  const auto upperbelt_logical_id =
    declare_parameter<int>("upperbelt_logical_id", 10);
  const auto target_array_topic =
    declare_parameter<std::string>("target_array_topic", "/vesc/target_array");

  for (std::size_t level = 0; level < num_levels; ++level) {
    if (underbelt_rpms_[level] < 0 || upperbelt_rpms_[level] < 0) {
      throw std::runtime_error("belt RPM parameters must be nonnegative");
    }
  }
  if (emergency_stop_period_ms <= 0 || qos_depth <= 0) {
    throw std::runtime_error("emergency stop period and QoS depth must be positive");
  }
  if (underbelt_logical_id < 0 || underbelt_logical_id > 65535 ||
    upperbelt_logical_id < 0 || upperbelt_logical_id > 65535 ||
    underbelt_logical_id == upperbelt_logical_id)
  {
    throw std::runtime_error("belt logical IDs must be unique values in [0, 65535]");
  }
  if (target_array_topic.empty()) {
    throw std::runtime_error("target_array_topic must not be empty");
  }

  underbelt_logical_id_ = static_cast<uint16_t>(underbelt_logical_id);
  upperbelt_logical_id_ = static_cast<uint16_t>(upperbelt_logical_id);

  const auto command_qos = rclcpp::QoS(qos_depth);
  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();

  // joy_controller -> belt_controller: 0=STOP, 1..4=速度段階。
  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos,
    std::bind(&BeltControllerNode::belt_mode_callback, this, std::placeholders::_1));

  // game2_shooter -> belt_controller: 上下ベルトへ同じRPMを直接指定する。
  belt_target_rpm_sub_ = create_subscription<std_msgs::msg::Float32>(
    "/belt/target_rpm", command_qos,
    std::bind(&BeltControllerNode::belt_target_rpm_callback, this, std::placeholders::_1));

  // joy_controller -> belt_controller: 非常停止状態をラッチしてゼロ出力を継続する。
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", emergency_stop_qos,
    std::bind(&BeltControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  // belt_controller -> vesc_driver: 上下ベルトの目標RPM。
  target_array_pub_ = create_publisher<actuator_msgs::msg::ActuatorTargetArray>(
    target_array_topic, command_qos);

  emergency_stop_timer_ = create_wall_timer(
    std::chrono::milliseconds(emergency_stop_period_ms),
    std::bind(&BeltControllerNode::emergency_stop_timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&BeltControllerNode::parameter_callback, this, std::placeholders::_1));
}

// /belt/modeを受信したら直接RPM指定を解除し、選択段階を即時送信する。
// 範囲外の値は安全のためSTOPとして扱う。非常停止中はゼロを送信する。
void BeltControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  use_direct_target_rpm_ = false;
  belt_mode_ = msg->data <= static_cast<uint8_t>(BeltMode::LEVEL_4) ?
    static_cast<BeltMode>(msg->data) : BeltMode::STOP;
  publish_command();
}

// /belt/target_rpmを受信したら上下共通の直接RPMを即時送信する。
// 0以下なら直接指定を解除して、現在の段階設定へ戻す。
void BeltControllerNode::belt_target_rpm_callback(
  const std_msgs::msg::Float32::SharedPtr msg)
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

// /emergency_stopを受信したら状態を更新し、ゼロまたは現在指令を即時送信する。
// 非常停止中はtimer callbackもゼロを継続送信する。
void BeltControllerNode::emergency_stop_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  publish_command();
}

// 非常停止中だけ /vesc/target_array へゼロを定期送信する。
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

  actuator_msgs::msg::ActuatorTargetArray command;
  command.header.stamp = now();
  command.actuators.resize(2);
  command.actuators[0].logical_id = underbelt_logical_id_;
  command.actuators[0].target = static_cast<float>(underbelt_rpm);
  command.actuators[1].logical_id = upperbelt_logical_id_;
  command.actuators[1].target = static_cast<float>(upperbelt_rpm);
  target_array_pub_->publish(command);
}
