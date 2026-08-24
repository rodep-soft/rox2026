#include "led_controller/led_controller.hpp"

#include <functional>
#include <stdexcept>

namespace
{
constexpr uint8_t BELT_LEVEL_MASK = 0x07;
constexpr uint8_t DRIBBLE_ENABLED_FLAG = 1U << 3U;
constexpr uint8_t DRIVE_REVERSED_FLAG = 1U << 4U;
constexpr uint8_t GAME2_ENABLED_FLAG = 1U << 5U;
}  // namespace

LedControllerNode::LedControllerNode()
: Node("led_controller_node")
{
  const auto publish_period_ms = declare_parameter<int>("publish_period_ms", 100);
  const auto firing_display_ms = declare_parameter<int>("firing_display_ms", 500);
  if (publish_period_ms <= 0 || firing_display_ms < 0) {
    throw std::runtime_error("LED timer parameters are invalid");
  }
  firing_display_duration_ = std::chrono::milliseconds(firing_display_ms);

  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos = rclcpp::QoS(10);

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", state_qos,
    std::bind(&LedControllerNode::emergency_stop_callback, this, std::placeholders::_1));

  belt_mode_sub_ = create_subscription<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", command_qos,
    std::bind(&LedControllerNode::belt_mode_callback, this, std::placeholders::_1));

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/command_enabled", command_qos,
    std::bind(&LedControllerNode::dribble_enabled_callback, this, std::placeholders::_1));

  drive_reversed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/drive/reversed", state_qos,
    std::bind(&LedControllerNode::drive_reversed_callback, this, std::placeholders::_1));

  shot_cycle_state_sub_ = create_subscription<robot_msgs::msg::ShotCycleState>(
    "/dribble/shot_cycle_state", state_qos,
    std::bind(&LedControllerNode::shot_cycle_state_callback, this, std::placeholders::_1));

  game2_state_sub_ = create_subscription<robot_msgs::msg::Game2State>(
    "/game2/state", state_qos,
    std::bind(&LedControllerNode::game2_state_callback, this, std::placeholders::_1));

  spring_fire_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos,
    std::bind(&LedControllerNode::spring_fire_callback, this, std::placeholders::_1));

  led_command_pub_ = create_publisher<std_msgs::msg::UInt16>("/hardware/led_cmd", command_qos);
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms),
    std::bind(&LedControllerNode::publish_timer_callback, this));
}

void LedControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_received_ = true;
  emergency_stop_active_ = msg->data;
}

void LedControllerNode::belt_mode_callback(const robot_msgs::msg::BeltMode::SharedPtr msg)
{
  belt_mode_ = msg->mode <=
    robot_msgs::msg::BeltMode::LEVEL_4 ? msg->mode : robot_msgs::msg::BeltMode::STOP;
}

void LedControllerNode::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

void LedControllerNode::drive_reversed_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  drive_reversed_ = msg->data;
}

void LedControllerNode::shot_cycle_state_callback(
  const robot_msgs::msg::ShotCycleState::SharedPtr msg)
{
  shot_cycle_state_ = msg->state;
}

void LedControllerNode::game2_state_callback(const robot_msgs::msg::Game2State::SharedPtr msg)
{
  game2_state_ = msg->state;
}

void LedControllerNode::spring_fire_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool rising_edge = msg->data && !spring_fire_request_active_;
  spring_fire_request_active_ = msg->data;
  if (rising_edge) {
    firing_display_until_ = std::chrono::steady_clock::now() + firing_display_duration_;
  }
}

void LedControllerNode::publish_timer_callback()
{
  std_msgs::msg::UInt16 command;
  command.data = static_cast<uint16_t>(select_display_mode()) |
    (static_cast<uint16_t>(make_status_flags()) << 8U);
  led_command_pub_->publish(command);
}

LedControllerNode::DisplayMode LedControllerNode::select_display_mode() const
{
  if (!emergency_stop_received_) {
    return DisplayMode::STARTUP;
  }
  if (emergency_stop_active_) {
    return DisplayMode::EMERGENCY_STOP;
  }
  if (std::chrono::steady_clock::now() < firing_display_until_) {
    return DisplayMode::FIRING;
  }

  switch (game2_state_) {
    case robot_msgs::msg::Game2State::SEARCHING:       return DisplayMode::GAME2_SEARCHING;
    case robot_msgs::msg::Game2State::ALIGNING:        return DisplayMode::GAME2_ALIGNING;
    case robot_msgs::msg::Game2State::PREPARING_SHOOT: return DisplayMode::LOADING;
    case robot_msgs::msg::Game2State::SHOOTING:        return DisplayMode::FIRING;
    case robot_msgs::msg::Game2State::WAITING_RESULT:  return DisplayMode::RETURNING;
    default: break;
  }

  switch (shot_cycle_state_) {
    case robot_msgs::msg::ShotCycleState::FEEDING:
      return DisplayMode::LOADING;
    case robot_msgs::msg::ShotCycleState::RETURNING:
      return DisplayMode::RETURNING;
    default:
      break;
  }

  return DisplayMode::READY;
}

uint8_t LedControllerNode::make_status_flags() const
{
  uint8_t flags = belt_mode_ & BELT_LEVEL_MASK;
  if (dribble_enabled_) {
    flags |= DRIBBLE_ENABLED_FLAG;
  }
  if (drive_reversed_) {
    flags |= DRIVE_REVERSED_FLAG;
  }
  if (game2_state_ != robot_msgs::msg::Game2State::STANDBY &&
    game2_state_ != robot_msgs::msg::Game2State::COMPLETED)
  {
    flags |= GAME2_ENABLED_FLAG;
  }
  return flags;
}
