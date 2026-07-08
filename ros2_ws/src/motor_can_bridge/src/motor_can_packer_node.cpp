#include "motor_can_bridge/motor_can_packer_node.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace
{
constexpr uint8_t RollerReceivedIndex = 0;
constexpr uint8_t BeltReceivedIndex = 1;
constexpr uint8_t OutputDlc = 4;
constexpr uint8_t BytesPerInput = 2;
constexpr uint8_t BeltDataOffset = 0;
constexpr uint8_t RollerDataOffset = 2;
constexpr char ExpectedCanFrameTopicType[] = "can_msgs/msg/Frame";
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
  this->declare_parameter<std::string>("can_tx_topic_type", ExpectedCanFrameTopicType);
  this->declare_parameter<int>("can_id", 0x201);
  this->declare_parameter<bool>("is_extended", false);
  this->declare_parameter<bool>("roller_enabled", true);
  this->declare_parameter<std::string>("roller_topic", "/roller/can_frame");
  this->declare_parameter<std::string>("roller_topic_type", ExpectedCanFrameTopicType);
  this->declare_parameter<bool>("belt_enabled", true);
  this->declare_parameter<std::string>("belt_topic", "/belt/can_frame");
  this->declare_parameter<std::string>("belt_topic_type", ExpectedCanFrameTopicType);
}

void MotorCanPackerNode::GetParameters()
{
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  const auto can_tx_topic_type = this->get_parameter("can_tx_topic_type").as_string();
  can_id_ = static_cast<uint32_t>(this->get_parameter("can_id").as_int());
  is_extended_ = this->get_parameter("is_extended").as_bool();

  ChannelConfig roller;
  roller.name = "roller";
  roller.topic = this->get_parameter("roller_topic").as_string();
  roller.enabled = this->get_parameter("roller_enabled").as_bool();
  roller.received_index = RollerReceivedIndex;
  roller.data_offset = RollerDataOffset;
  channels_.push_back(roller);
  const auto roller_topic_type = this->get_parameter("roller_topic_type").as_string();

  ChannelConfig belt;
  belt.name = "belt";
  belt.topic = this->get_parameter("belt_topic").as_string();
  belt.enabled = this->get_parameter("belt_enabled").as_bool();
  belt.received_index = BeltReceivedIndex;
  belt.data_offset = BeltDataOffset;
  channels_.push_back(belt);
  const auto belt_topic_type = this->get_parameter("belt_topic_type").as_string();

  if (can_tx_topic_type != ExpectedCanFrameTopicType) {
    RCLCPP_WARN(
      this->get_logger(),
      "can_tx_topic_type is '%s', but this node publishes as '%s'.",
      can_tx_topic_type.c_str(),
      ExpectedCanFrameTopicType);
  }
  if (roller_topic_type != ExpectedCanFrameTopicType) {
    RCLCPP_WARN(
      this->get_logger(),
      "roller_topic_type is '%s', but this node subscribes as '%s'.",
      roller_topic_type.c_str(),
      ExpectedCanFrameTopicType);
  }
  if (belt_topic_type != ExpectedCanFrameTopicType) {
    RCLCPP_WARN(
      this->get_logger(),
      "belt_topic_type is '%s', but this node subscribes as '%s'.",
      belt_topic_type.c_str(),
      ExpectedCanFrameTopicType);
  }
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
    RCLCPP_WARN(
      this->get_logger(),
      "All input channels are disabled. No CAN frames will be packed.");
  }
}

void MotorCanPackerNode::FrameCallback(
  const can_msgs::msg::Frame::SharedPtr msg,
  const ChannelConfig & channel)
{
  if (msg->dlc < BytesPerInput) {
    RCLCPP_WARN(
      this->get_logger(),
      "Dropping frame from %s: dlc=%u is shorter than required bytes=%u",
      channel.topic.c_str(),
      msg->dlc,
      BytesPerInput);
    return;
  }

  for (uint8_t i = 0; i < BytesPerInput; ++i) {
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
  frame.dlc = OutputDlc;
  frame.data.fill(0);

  for (uint8_t i = 0; i < OutputDlc; ++i) {
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
