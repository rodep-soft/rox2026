#include "motor_can_bridge/motor_can_command_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace
{
constexpr uint8_t kCanDataSize = 8;
constexpr int kMinMotorId = 0;
constexpr int kMaxMotorId = 255;
}  // namespace

MotorCanCommandNode::MotorCanCommandNode()
: Node("motor_can_command_node")
{
  DeclareParameters();
  GetParameters();

  latest_pwm_ = 0;
  last_pwm_time_ = this->now();

  pwm_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    pwm_topic_, 10,
    std::bind(&MotorCanCommandNode::PwmCallback, this, std::placeholders::_1));

  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&MotorCanCommandNode::TimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "MotorCanCommandNode started. pwm_topic=%s, can_tx_topic=%s, can_id=0x%X, motor_id=%u",
    pwm_topic_.c_str(),
    can_tx_topic_.c_str(),
    can_id_,
    motor_id_);
}

void MotorCanCommandNode::DeclareParameters()
{
  this->declare_parameter<std::string>("pwm_topic", "/motor/pwm_value");
  this->declare_parameter<std::string>("can_tx_topic", "/can_tx");
  this->declare_parameter<int>("can_id", 0x201);
  this->declare_parameter<int>("motor_id", 1);
  this->declare_parameter<bool>("is_extended", false);
  this->declare_parameter<int>("min_pwm", -255);
  this->declare_parameter<int>("max_pwm", 255);
  this->declare_parameter<int>("send_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);
}

void MotorCanCommandNode::GetParameters()
{
  pwm_topic_ = this->get_parameter("pwm_topic").as_string();
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  can_id_ = static_cast<uint32_t>(this->get_parameter("can_id").as_int());
  motor_id_ = static_cast<uint8_t>(
    std::clamp(
      static_cast<int>(this->get_parameter("motor_id").as_int()),
      kMinMotorId,
      kMaxMotorId));
  is_extended_ = this->get_parameter("is_extended").as_bool();
  min_pwm_ = static_cast<int>(this->get_parameter("min_pwm").as_int());
  max_pwm_ = static_cast<int>(this->get_parameter("max_pwm").as_int());
  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  timeout_ms_ = std::max(1, static_cast<int>(this->get_parameter("timeout_ms").as_int()));

  if (min_pwm_ > max_pwm_) {
    RCLCPP_WARN(
      this->get_logger(),
      "min_pwm is greater than max_pwm. Swapping values.");
    std::swap(min_pwm_, max_pwm_);
  }
}

void MotorCanCommandNode::PwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  latest_pwm_ = ClampPwm(msg->data);
  last_pwm_time_ = this->now();
}

void MotorCanCommandNode::TimerCallback()
{
  const rclcpp::Time now = this->now();
  can_publisher_->publish(CreateCanFrame(GetPwmOrZeroOnTimeout(now), now));
}

int16_t MotorCanCommandNode::ClampPwm(int value) const
{
  return static_cast<int16_t>(std::clamp(value, min_pwm_, max_pwm_));
}

int16_t MotorCanCommandNode::GetPwmOrZeroOnTimeout(const rclcpp::Time & now) const
{
  const auto elapsed = now - last_pwm_time_;
  if (elapsed > rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0)) {
    return 0;
  }

  return latest_pwm_;
}

can_msgs::msg::Frame MotorCanCommandNode::CreateCanFrame(
  int16_t pwm,
  const rclcpp::Time & stamp) const
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = stamp;
  frame.id = can_id_;
  frame.is_rtr = false;
  frame.is_extended = is_extended_;
  frame.is_error = false;
  frame.dlc = kCanDataSize;
  frame.data.fill(0);

  frame.data[0] = motor_id_;
  frame.data[1] = static_cast<uint8_t>(pwm & 0xFF);
  frame.data[2] = static_cast<uint8_t>((pwm >> 8) & 0xFF);

  return frame;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorCanCommandNode>());
  rclcpp::shutdown();
  return 0;
}
