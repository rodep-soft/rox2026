#include "led_controller/led_controller.hpp"

#include <functional>
#include <stdexcept>

namespace
{
constexpr uint8_t BELT_LEVEL_MASK = 0x07;
constexpr uint8_t DRIBBLE_ENABLED_FLAG = 1U << 3U;
constexpr uint8_t DRIVE_REVERSED_FLAG = 1U << 4U;
constexpr uint8_t GAME2_ENABLED_FLAG = 1U << 5U;
constexpr uint8_t SHOT_CYCLE_OPENING = 1;
constexpr uint8_t SHOT_CYCLE_LOADING = 2;
constexpr uint8_t SHOT_CYCLE_RETURNING = 3;
constexpr uint8_t GAME2_SEARCHING = 1;
constexpr uint8_t GAME2_ALIGNING = 2;
constexpr uint8_t GAME2_PREPARING_SHOOT = 3;
constexpr uint8_t GAME2_SHOOTING = 4;
constexpr uint8_t GAME2_WAITING_RESULT = 5;
constexpr uint8_t GAME2_COMPLETED = 6;
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

  // joy_controller -> led_controller: ?????????????
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&LedControllerNode::emergency_stop_callback, this, std::placeholders::_1));
  // joy_controller -> led_controller: ??????????0?4??????
  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos,
    std::bind(&LedControllerNode::belt_mode_callback, this, std::placeholders::_1));
  // joy_controller/game2 -> led_controller: ????????????????
  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/enabled", command_qos,
    std::bind(&LedControllerNode::dribble_enabled_callback, this, std::placeholders::_1));
  // joy_controller -> led_controller: LED????????????
  drive_reversed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/drive/reversed", state_qos,
    std::bind(&LedControllerNode::drive_reversed_callback, this, std::placeholders::_1));
  // dribble_controller -> led_controller: ??OPEN??????????????????
  shot_cycle_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/shot_cycle/state", state_qos,
    std::bind(&LedControllerNode::shot_cycle_state_callback, this, std::placeholders::_1));
  // game2_shooter -> led_controller: ????????????????????
  game2_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/game2/state", state_qos,
    std::bind(&LedControllerNode::game2_state_callback, this, std::placeholders::_1));
  // joy_controller -> led_controller: ??????????????????
  spring_fire_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos,
    std::bind(&LedControllerNode::spring_fire_callback, this, std::placeholders::_1));

  // led_controller -> stm32_driver: ??byte=??????byte=?????
  led_command_pub_ = create_publisher<std_msgs::msg::UInt16>("/led/cmd", command_qos);
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms),
    std::bind(&LedControllerNode::publish_timer_callback, this));
}

void LedControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_received_ = true;
  emergency_stop_active_ = msg->data;
}

void LedControllerNode::belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  belt_mode_ = msg->data <= 4 ? msg->data : 0;
}

void LedControllerNode::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

void LedControllerNode::drive_reversed_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  drive_reversed_ = msg->data;
}

void LedControllerNode::shot_cycle_state_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  shot_cycle_state_ = msg->data <= SHOT_CYCLE_RETURNING ? msg->data : 0;
}

void LedControllerNode::game2_state_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  game2_state_ = msg->data <= GAME2_COMPLETED ? msg->data : 0;
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
  if (game2_state_ == GAME2_SEARCHING) {
    return DisplayMode::GAME2_SEARCHING;
  }
  if (game2_state_ == GAME2_ALIGNING) {
    return DisplayMode::GAME2_ALIGNING;
  }
  if (game2_state_ == GAME2_PREPARING_SHOOT) {
    return DisplayMode::LOADING;
  }
  if (game2_state_ == GAME2_SHOOTING) {
    return DisplayMode::FIRING;
  }
  if (game2_state_ == GAME2_WAITING_RESULT) {
    return DisplayMode::RETURNING;
  }
  if (shot_cycle_state_ == SHOT_CYCLE_OPENING) {
    return DisplayMode::SHOT_OPENING;
  }
  if (shot_cycle_state_ == SHOT_CYCLE_LOADING) {
    return DisplayMode::LOADING;
  }
  if (shot_cycle_state_ == SHOT_CYCLE_RETURNING) {
    return DisplayMode::RETURNING;
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
  if (game2_state_ != 0 && game2_state_ != GAME2_COMPLETED) {
    flags |= GAME2_ENABLED_FLAG;
  }
  return flags;
}
