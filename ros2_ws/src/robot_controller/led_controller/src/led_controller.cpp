#include "led_controller/led_controller.hpp"

#include <functional>
#include <stdexcept>

namespace
{
// /belt/mode の下位3ビットがレベル (0=STOP, 1〜4=LEVEL)
constexpr uint8_t BELT_LEVEL_MASK = 0x07;
// ステータスフラグのビット配置 (/led/cmd の上位バイト)
constexpr uint8_t DRIBBLE_ENABLED_FLAG = 1U << 3U;
constexpr uint8_t DRIVE_REVERSED_FLAG  = 1U << 4U;
constexpr uint8_t GAME2_ENABLED_FLAG   = 1U << 5U;

// /shot_cycle/state の値。0=非アクティブ、1〜4=ShotCyclePhase+1
constexpr uint8_t SHOT_STATE_BELT_SPINUP = 1;
constexpr uint8_t SHOT_STATE_OPENING     = 2;
constexpr uint8_t SHOT_STATE_FEEDING     = 3;
constexpr uint8_t SHOT_STATE_RETURNING   = 4;

// /game2/state の値
constexpr uint8_t GAME2_SEARCHING      = 1;
constexpr uint8_t GAME2_ALIGNING       = 2;
constexpr uint8_t GAME2_PREPARING_SHOOT = 3;
constexpr uint8_t GAME2_SHOOTING       = 4;
constexpr uint8_t GAME2_WAITING_RESULT = 5;
constexpr uint8_t GAME2_COMPLETED      = 6;
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

  // joy_controller -> led_controller: 非常停止状態
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", state_qos,
    std::bind(&LedControllerNode::emergency_stop_callback, this, std::placeholders::_1));
  // joy_controller -> led_controller: beltレベル (0〜4)
  belt_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/belt/mode", command_qos,
    std::bind(&LedControllerNode::belt_mode_callback, this, std::placeholders::_1));
  // joy_controller/game2 -> led_controller: dribble ON/OFF
  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/enabled", command_qos,
    std::bind(&LedControllerNode::dribble_enabled_callback, this, std::placeholders::_1));
  // joy_controller -> led_controller: 走行方向反転フラグ
  drive_reversed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/drive/reversed", state_qos,
    std::bind(&LedControllerNode::drive_reversed_callback, this, std::placeholders::_1));
  // dribble_controller -> led_controller: shot cycleフェーズ (0=非アクティブ)
  shot_cycle_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/shot_cycle/state", state_qos,
    std::bind(&LedControllerNode::shot_cycle_state_callback, this, std::placeholders::_1));
  // game2_shooter -> led_controller: Game2 シーケンス状態
  game2_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/game2/state", state_qos,
    std::bind(&LedControllerNode::game2_state_callback, this, std::placeholders::_1));
  // joy_controller -> led_controller: Spring発射要求 (立ち上がりで発射エフェクト)
  spring_fire_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spring/fire_request", command_qos,
    std::bind(&LedControllerNode::spring_fire_callback, this, std::placeholders::_1));

  // led_controller -> stm32_driver: 下位バイト=表示モード、上位バイト=ステータスフラグ
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
  shot_cycle_state_ = msg->data <= SHOT_STATE_RETURNING ? msg->data : 0;
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

  // Game2 状態を優先して表示
  switch (game2_state_) {
    case GAME2_SEARCHING:       return DisplayMode::GAME2_SEARCHING;
    case GAME2_ALIGNING:        return DisplayMode::GAME2_ALIGNING;
    case GAME2_PREPARING_SHOOT: return DisplayMode::LOADING;
    case GAME2_SHOOTING:        return DisplayMode::FIRING;
    case GAME2_WAITING_RESULT:  return DisplayMode::RETURNING;
    default: break;
  }

  // shot cycle フェーズを表示 (BELT_SPINUP中はREADYのまま)
  switch (shot_cycle_state_) {
    case SHOT_STATE_OPENING:  return DisplayMode::SHOT_OPENING;
    case SHOT_STATE_FEEDING:  return DisplayMode::LOADING;
    case SHOT_STATE_RETURNING: return DisplayMode::RETURNING;
    default: break;
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
