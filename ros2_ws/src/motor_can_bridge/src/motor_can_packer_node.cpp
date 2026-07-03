#include "motor_can_bridge/motor_can_packer_node.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace
{
constexpr uint8_t kRollerReceivedIndex = 0;
constexpr uint8_t kBeltReceivedIndex = 1;
constexpr uint8_t kOutputDlc = 4;
constexpr uint8_t kBytesPerInput = 2;
constexpr uint8_t kBeltDataOffset = 0;
constexpr uint8_t kRollerDataOffset = 2;
}  // namespace

MotorCanPackerNode::MotorCanPackerNode()
: Node("motor_can_packer_node")
{
  DeclareParameters();
  GetParameters();
  SetupRosInterfaces();

  RCLCPP_INFO(
    this->get_logger(),
    "MotorCanPackerNode started. can_tx_topic=%s, can_id=0x%X",
    can_tx_topic_.c_str(),
    can_id_);
}

void MotorCanPackerNode::DeclareParameters()
{
  this->declare_parameter<std::string>("can_tx_topic", "/CAN/can0/transmit");
  this->declare_parameter<int>("can_id", 0x201);
  this->declare_parameter<bool>("is_extended", false);
  this->declare_parameter<bool>("roller_enabled", true);
  this->declare_parameter<std::string>("roller_topic", "/roller/can_frame");
  this->declare_parameter<bool>("belt_enabled", true);
  this->declare_parameter<std::string>("belt_topic", "/belt/can_frame");
}

void MotorCanPackerNode::GetParameters()
{
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  can_id_ = static_cast<uint32_t>(this->get_parameter("can_id").as_int());
  is_extended_ = this->get_parameter("is_extended").as_bool();

  ChannelConfig roller;
  roller.name = "roller";
  roller.topic = this->get_parameter("roller_topic").as_string();
  roller.enabled = this->get_parameter("roller_enabled").as_bool();
  roller.received_index = kRollerReceivedIndex;
  roller.data_offset = kRollerDataOffset;
  channels_.push_back(roller);

  ChannelConfig belt;
  belt.name = "belt";
  belt.topic = this->get_parameter("belt_topic").as_string();
  belt.enabled = this->get_parameter("belt_enabled").as_bool();
  belt.received_index = kBeltReceivedIndex;
  belt.data_offset = kBeltDataOffset;
  channels_.push_back(belt);
}

void MotorCanPackerNode::SetupRosInterfaces()
{
  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);

  for (const auto & channel : channels_) {
    if (!channel.enabled) {
      RCLCPP_INFO(this->get_logger(), "Channel disabled: %s", channel.name.c_str());
      continue;
    }

    subscriptions_.push_back(
      this->create_subscription<can_msgs::msg::Frame>(
        channel.topic,
        10,
        [this, channel](const can_msgs::msg::Frame::SharedPtr msg) {
          FrameCallback(msg, channel);
        }));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed channel=%s topic=%s data_offset=%u",
      channel.name.c_str(),
      channel.topic.c_str(),
      channel.data_offset);
  }

  if (subscriptions_.empty()) {
    RCLCPP_WARN(this->get_logger(), "All input channels are disabled. No CAN frames will be packed.");
  }
}

void MotorCanPackerNode::FrameCallback(
  const can_msgs::msg::Frame::SharedPtr msg,
  const ChannelConfig & channel)
{
  if (msg->dlc < kBytesPerInput) {
    RCLCPP_WARN(
      this->get_logger(),
      "Dropping frame from %s: dlc=%u is shorter than required bytes=%u",
      channel.topic.c_str(),
      msg->dlc,
      kBytesPerInput);
    return;
  }

  for (uint8_t i = 0; i < kBytesPerInput; ++i) {
    frame_data_[channel.data_offset + i] = msg->data[i];
  }
  received_[channel.received_index] = true;

  if (!HasReceivedAllEnabledChannels()) {
    return;
  }

  can_publisher_->publish(CreatePackedFrame(this->now()));
}

bool MotorCanPackerNode::HasReceivedAllEnabledChannels() const
{
  for (const auto & channel : channels_) {
    if (channel.enabled && !received_[channel.received_index]) {
      return false;
    }
  }

  return true;
}

can_msgs::msg::Frame MotorCanPackerNode::CreatePackedFrame(const rclcpp::Time & stamp) const
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = stamp;
  frame.id = can_id_;
  frame.is_rtr = false;
  frame.is_extended = is_extended_;
  frame.is_error = false;
  frame.dlc = kOutputDlc;
  frame.data.fill(0);

  for (uint8_t i = 0; i < kOutputDlc; ++i) {
    frame.data[i] = frame_data_[i];
  }

  return frame;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorCanPackerNode>());
  rclcpp::shutdown();
  return 0;
}
