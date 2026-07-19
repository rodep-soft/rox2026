#include "roller_belt_can_packer/roller_belt_can_packer_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

namespace
{
constexpr uint8_t OutputDlc = 4;
constexpr uint8_t BeltDataOffset = 0;
constexpr uint8_t RollerDataOffset = 2;
}  // namespace

RollerBeltCanPackerNode::RollerBeltCanPackerNode()
: Node("roller_belt_can_packer_node"),
  roller_rpm_(0),
  belt_rpm_(0)
{
  DeclareParameters();
  GetParameters();

  roller_last_received_time_ = this->now();
  belt_last_received_time_ = this->now();

  roller_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    roller_rpm_topic_, 10,
    std::bind(&RollerBeltCanPackerNode::RollerCallback, this, std::placeholders::_1));
  belt_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    belt_rpm_topic_, 10,
    std::bind(&RollerBeltCanPackerNode::BeltCallback, this, std::placeholders::_1));
  can_frame_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_transmit_topic_, 10);
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&RollerBeltCanPackerNode::TimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "RollerBeltCanPackerNode started. roller_rpm_topic=%s, belt_rpm_topic=%s, can_transmit_topic=%s, stm_can_id=0x%X",
    roller_rpm_topic_.c_str(), belt_rpm_topic_.c_str(), can_transmit_topic_.c_str(), stm_can_id_);
}

void RollerBeltCanPackerNode::DeclareParameters()
{
  this->declare_parameter<std::string>("can_transmit_topic", "/CAN/can0/transmit");
  this->declare_parameter<int>("stm_can_id", 0x201);
  this->declare_parameter<bool>("is_extended", false);
  this->declare_parameter<std::string>("roller_rpm_topic", "/roller/rpm");
  this->declare_parameter<std::string>("belt_rpm_topic", "/belt/rpm");
  this->declare_parameter<int>("min_rpm", 0);
  this->declare_parameter<int>("max_rpm", 5000);
  this->declare_parameter<int>("publish_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);
}

void RollerBeltCanPackerNode::GetParameters()
{
  can_transmit_topic_ = this->get_parameter("can_transmit_topic").as_string();
  stm_can_id_ = static_cast<uint32_t>(this->get_parameter("stm_can_id").as_int());
  is_extended_ = this->get_parameter("is_extended").as_bool();
  roller_rpm_topic_ = this->get_parameter("roller_rpm_topic").as_string();
  belt_rpm_topic_ = this->get_parameter("belt_rpm_topic").as_string();
  min_rpm_ = static_cast<int>(this->get_parameter("min_rpm").as_int());
  max_rpm_ = static_cast<int>(this->get_parameter("max_rpm").as_int());
  publish_period_ms_ = std::max(
    1, static_cast<int>(this->get_parameter("publish_period_ms").as_int()));
  timeout_ms_ = std::max(1, static_cast<int>(this->get_parameter("timeout_ms").as_int()));

  if (min_rpm_ > max_rpm_) {
    RCLCPP_WARN(this->get_logger(), "min_rpm is greater than max_rpm. Swapping values.");
    std::swap(min_rpm_, max_rpm_);
  }
}

void RollerBeltCanPackerNode::RollerCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  roller_rpm_ = ClampRpm(msg->data);
  roller_last_received_time_ = this->now();
}

void RollerBeltCanPackerNode::BeltCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  belt_rpm_ = ClampRpm(msg->data);
  belt_last_received_time_ = this->now();
}

void RollerBeltCanPackerNode::TimerCallback()
{
  const rclcpp::Time now = this->now();
  can_frame_publisher_->publish(CreateStmCanFrame(now));
}

int16_t RollerBeltCanPackerNode::ClampRpm(int value) const
{
  return static_cast<int16_t>(std::clamp(value, min_rpm_, max_rpm_));
}

int16_t RollerBeltCanPackerNode::GetRpmOrZeroOnTimeout(
  int16_t rpm,
  const rclcpp::Time & last_received_time,
  const rclcpp::Time & now) const
{
  const auto elapsed = now - last_received_time;
  if (elapsed > rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0)) {
    return 0;
  }
  return rpm;
}

can_msgs::msg::Frame RollerBeltCanPackerNode::CreateStmCanFrame(
  const rclcpp::Time & stamp) const
{
  const int16_t belt_rpm = GetRpmOrZeroOnTimeout(belt_rpm_, belt_last_received_time_, stamp);
  const int16_t roller_rpm = GetRpmOrZeroOnTimeout(
    roller_rpm_, roller_last_received_time_, stamp);

  can_msgs::msg::Frame frame;
  frame.header.stamp = stamp;
  frame.id = stm_can_id_;
  frame.is_rtr = false;
  frame.is_extended = is_extended_;
  frame.is_error = false;
  frame.dlc = OutputDlc;
  frame.data.fill(0);

  frame.data[BeltDataOffset] = static_cast<uint8_t>(belt_rpm & 0xFF);
  frame.data[BeltDataOffset + 1] = static_cast<uint8_t>((belt_rpm >> 8) & 0xFF);
  frame.data[RollerDataOffset] = static_cast<uint8_t>(roller_rpm & 0xFF);
  frame.data[RollerDataOffset + 1] = static_cast<uint8_t>((roller_rpm >> 8) & 0xFF);

  return frame;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RollerBeltCanPackerNode>());
  rclcpp::shutdown();
  return 0;
}
