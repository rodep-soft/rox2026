#include "motor_can_bridge/motor_can_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace
{
constexpr uint8_t kCanDataSize = 8;
constexpr int16_t kMabuchiMinPwm = -255;
constexpr int16_t kMabuchiMaxPwm = 255;
constexpr int16_t kMadMotorMinPwm = 0;
constexpr int16_t kMadMotorMaxPwm = 255;

int16_t ClampToInt16Range(int value, int16_t min_value, int16_t max_value)
{
  return static_cast<int16_t>(
    std::clamp(value, static_cast<int>(min_value), static_cast<int>(max_value)));
}
}  // namespace

MotorCanBridgeNode::MotorCanBridgeNode()
: Node("motor_can_bridge_node")
{
  DeclareParameters();
  GetParameters();

  latest_mabuchi_pwm_ = 0;
  latest_mad_motor_pwm_ = 0;
  last_mabuchi_time_ = this->now();
  last_mad_motor_time_ = this->now();

  mabuchi_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    mabuchi_pwm_topic_, 10,
    std::bind(&MotorCanBridgeNode::MabuchiPwmCallback, this, std::placeholders::_1));

  mad_motor_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    mad_motor_pwm_topic_, 10,
    std::bind(&MotorCanBridgeNode::MadMotorPwmCallback, this, std::placeholders::_1));

  can_publisher_ = this->create_publisher<motor_can_bridge::msg::CanFrame>(
    can_tx_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&MotorCanBridgeNode::TimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "MotorCanBridgeNode started.");
}

void MotorCanBridgeNode::DeclareParameters()
{
  this->declare_parameter<std::string>("mabuchi_pwm_topic", "/mabuchi555/pwm_value");
  this->declare_parameter<std::string>("mad_motor_pwm_topic", "/mad_motor/pwm_value");
  this->declare_parameter<std::string>("can_tx_topic", "/can_tx");
  this->declare_parameter<int>("mabuchi_can_id", 0x201);
  this->declare_parameter<int>("mad_motor_can_id", 0x202);
  this->declare_parameter<int>("send_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);
}

void MotorCanBridgeNode::GetParameters()
{
  mabuchi_pwm_topic_ = this->get_parameter("mabuchi_pwm_topic").as_string();
  mad_motor_pwm_topic_ = this->get_parameter("mad_motor_pwm_topic").as_string();
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  mabuchi_can_id_ = static_cast<uint32_t>(this->get_parameter("mabuchi_can_id").as_int());
  mad_motor_can_id_ = static_cast<uint32_t>(this->get_parameter("mad_motor_can_id").as_int());
  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  timeout_ms_ = std::max(1, static_cast<int>(this->get_parameter("timeout_ms").as_int()));
}

void MotorCanBridgeNode::MabuchiPwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  latest_mabuchi_pwm_ = ClampToInt16Range(msg->data, kMabuchiMinPwm, kMabuchiMaxPwm);
  last_mabuchi_time_ = this->now();
}

void MotorCanBridgeNode::MadMotorPwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  latest_mad_motor_pwm_ = ClampToInt16Range(msg->data, kMadMotorMinPwm, kMadMotorMaxPwm);
  last_mad_motor_time_ = this->now();
}

void MotorCanBridgeNode::TimerCallback()
{
  const rclcpp::Time now = this->now();

  const int16_t mabuchi_pwm = GetPwmOrZeroOnTimeout(
    latest_mabuchi_pwm_, last_mabuchi_time_, now);
  const int16_t mad_motor_pwm = GetPwmOrZeroOnTimeout(
    latest_mad_motor_pwm_, last_mad_motor_time_, now);

  can_publisher_->publish(CreateMotorCanFrame(mabuchi_can_id_, mabuchi_pwm, now));
  can_publisher_->publish(CreateMotorCanFrame(mad_motor_can_id_, mad_motor_pwm, now));
}

int16_t MotorCanBridgeNode::GetPwmOrZeroOnTimeout(
  int16_t latest_pwm,
  const rclcpp::Time & last_update_time,
  const rclcpp::Time & now) const
{
  const auto elapsed = now - last_update_time;
  if (elapsed > rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0)) {
    return 0;
  }

  return latest_pwm;
}

motor_can_bridge::msg::CanFrame MotorCanBridgeNode::CreateMotorCanFrame(
  uint32_t can_id,
  int16_t pwm,
  const rclcpp::Time & stamp) const
{
  motor_can_bridge::msg::CanFrame frame;
  frame.header.stamp = stamp;
  frame.id = can_id;
  frame.extended = false;
  frame.fd = false;
  frame.brs = false;
  frame.esi = false;
  frame.rtr = false;
  frame.size = kCanDataSize;
  frame.data.fill(0);

  // data[0] is reserved for future flags. data[1..2] is int16 little-endian PWM.
  frame.data[0] = 0x00;
  frame.data[1] = static_cast<uint8_t>(pwm & 0xFF);
  frame.data[2] = static_cast<uint8_t>((pwm >> 8) & 0xFF);

  return frame;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorCanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
