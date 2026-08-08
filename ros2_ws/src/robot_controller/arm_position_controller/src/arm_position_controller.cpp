#include "arm_position_controller/arm_position_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

// ────────────────────────────────────────────────────────────────────────────
// コンストラクタ
// ────────────────────────────────────────────────────────────────────────────

ArmPositionControllerNode::ArmPositionControllerNode()
: Node("arm_position_controller_node")
{
  declare_parameters();
  get_parameters();
  if (logical_id_ < 0 || logical_id_ > 65535 || target_topic_.empty()) {
    throw std::runtime_error("logical_id or target_topic is invalid");
  }

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos = rclcpp::QoS(qos_depth_);

  position_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    target_topic_, command_qos);

  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/dribble/position_mode", command_qos,
    std::bind(&ArmPositionControllerNode::position_mode_callback, this, std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/shot_cycle/request", command_qos,
    std::bind(&ArmPositionControllerNode::shot_cycle_callback, this, std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(
      &ArmPositionControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&ArmPositionControllerNode::timer_callback, this));
}

// ────────────────────────────────────────────────────────────────────────────
// パラメータ
// ────────────────────────────────────────────────────────────────────────────

void ArmPositionControllerNode::declare_parameters()
{
  declare_parameter<double>("dribble_position_rad", 0.35);
  declare_parameter<double>("open_position_rad", -1.0);
  declare_parameter<double>("feed_position_rad", 1.3);
  declare_parameter<double>("open_duration_sec", 0.3);
  declare_parameter<double>("feed_duration_sec", 0.6);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
  declare_parameter<int>("logical_id", 5);
  declare_parameter<std::string>("target_topic", "/edulite/target");
}

void ArmPositionControllerNode::get_parameters()
{
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("open_position_rad", open_position_rad_);
  get_parameter("feed_position_rad", feed_position_rad_);
  get_parameter("open_duration_sec", open_duration_sec_);
  get_parameter("feed_duration_sec", feed_duration_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
  get_parameter("logical_id", logical_id_);
  get_parameter("target_topic", target_topic_);
}

// ────────────────────────────────────────────────────────────────────────────
// コールバック
// ────────────────────────────────────────────────────────────────────────────

void ArmPositionControllerNode::position_mode_callback(
  const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (msg->data > static_cast<uint8_t>(PositionMode::FEED)) {
    return;
  }
  const auto new_mode = static_cast<PositionMode>(msg->data);
  if (new_mode != current_position_mode_ || in_shot_cycle_) {
    in_shot_cycle_ = false;  // 手動割り込みで自動サイクルを解除
    RCLCPP_INFO(
      get_logger(), "Arm Mode Changed: %s -> %s",
      mode_name(current_position_mode_), mode_name(new_mode));
    current_position_mode_ = new_mode;
  }
}

void ArmPositionControllerNode::shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data || emergency_stop_active_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Starting Auto Shot Cycle: OPEN -> FEED -> DRIBBLE");
  in_shot_cycle_ = true;
  shot_cycle_phase_ = ShotCyclePhase::OPENING;
  shot_cycle_start_time_ = now();
  current_position_mode_ = PositionMode::OPEN;
}

void ArmPositionControllerNode::emergency_stop_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data != emergency_stop_active_) {
    if (msg->data) {
      in_shot_cycle_ = false;
      RCLCPP_WARN(get_logger(), "Emergency Stop Activated in ArmPositionController!");
    } else {
      RCLCPP_INFO(get_logger(), "Emergency Stop Released in ArmPositionController.");
    }
  }
  emergency_stop_active_ = msg->data;
}

// ────────────────────────────────────────────────────────────────────────────
// タイマーコールバック（ステートマシン進行 + publish）
// ────────────────────────────────────────────────────────────────────────────

void ArmPositionControllerNode::timer_callback()
{
  if (emergency_stop_active_) {
    in_shot_cycle_ = false;
    actuator_msgs::msg::ActuatorTarget msg;
    msg.logical_id = static_cast<uint16_t>(logical_id_);
    msg.target = static_cast<float>(dribble_position_rad_);
    position_command_pub_->publish(msg);
    return;
  }

  // 自動シュートサイクルのフェーズ進行
  if (in_shot_cycle_) {
    const double elapsed = (now() - shot_cycle_start_time_).seconds();
    switch (shot_cycle_phase_) {
      case ShotCyclePhase::OPENING:
        current_position_mode_ = PositionMode::OPEN;
        if (elapsed >= open_duration_sec_) {
          shot_cycle_phase_ = ShotCyclePhase::FEEDING;
          current_position_mode_ = PositionMode::FEED;
          RCLCPP_INFO(get_logger(), "Shot Cycle: OPEN -> FEED");
        }
        break;
      case ShotCyclePhase::FEEDING:
        current_position_mode_ = PositionMode::FEED;
        if (elapsed >= open_duration_sec_ + feed_duration_sec_) {
          in_shot_cycle_ = false;
          current_position_mode_ = PositionMode::DRIBBLE;
          RCLCPP_INFO(get_logger(), "Shot Cycle Completed: Returned to DRIBBLE");
        }
        break;
    }
  }

  actuator_msgs::msg::ActuatorTarget msg;
  msg.logical_id = static_cast<uint16_t>(logical_id_);
  msg.target = target_position_rad();
  position_command_pub_->publish(msg);
}

// ────────────────────────────────────────────────────────────────────────────
// ヘルパー
// ────────────────────────────────────────────────────────────────────────────

const char * ArmPositionControllerNode::mode_name(PositionMode mode) const
{
  switch (mode) {
    case PositionMode::DRIBBLE: return "DRIBBLE";
    case PositionMode::OPEN:    return "OPEN";
    case PositionMode::FEED:    return "FEED";
  }
  return "UNKNOWN";
}

float ArmPositionControllerNode::target_position_rad() const
{
  switch (current_position_mode_) {
    case PositionMode::DRIBBLE: return static_cast<float>(dribble_position_rad_);
    case PositionMode::OPEN:    return static_cast<float>(open_position_rad_);
    case PositionMode::FEED:    return static_cast<float>(feed_position_rad_);
  }
  return static_cast<float>(dribble_position_rad_);
}
