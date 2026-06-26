#include "stm_can_send/stm_can_send.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr uint8_t kMaxClassicCanDlc = 8;

canid_t BuildCanId(const can_msgs::msg::Frame & msg, uint32_t configured_id, bool override_id)
{
  canid_t can_id = static_cast<canid_t>(override_id ? configured_id : msg.id);

  if (msg.is_extended) {
    can_id |= CAN_EFF_FLAG;
  }
  if (msg.is_rtr) {
    can_id |= CAN_RTR_FLAG;
  }
  if (msg.is_error) {
    can_id |= CAN_ERR_FLAG;
  }

  return can_id;
}
}  // namespace

StmCanSendNode::StmCanSendNode()
: Node("stm_can_send_node"), can_socket_(-1)
{
  DeclareParameters();
  LoadParameters();
  OpenCanSocket();
  SetupSubscriptions();

  RCLCPP_INFO(
    this->get_logger(),
    "StmCanSendNode started. interface=%s, channels=%zu",
    can_interface_.c_str(),
    channels_.size());
}

StmCanSendNode::~StmCanSendNode()
{
  CloseCanSocket();
}

void StmCanSendNode::DeclareParameters()
{
  this->declare_parameter<std::string>("can_interface", "can0");

  for (const auto & channel_name : {"roller", "belt"}) {
    this->declare_parameter<bool>(std::string(channel_name) + ".enabled", true);
    this->declare_parameter<bool>(std::string(channel_name) + ".enable", true);
    this->declare_parameter<std::string>(std::string(channel_name) + ".topic", "");
    this->declare_parameter<int>(std::string(channel_name) + ".can_id", 0);
    this->declare_parameter<bool>(std::string(channel_name) + ".override_can_id", true);
  }
}

void StmCanSendNode::LoadParameters()
{
  can_interface_ = this->get_parameter("can_interface").as_string();

  channels_.clear();
  channels_.push_back(LoadChannelConfig("roller", 0x201));
  channels_.push_back(LoadChannelConfig("belt", 0x202));
}

bool StmCanSendNode::GetChannelEnabled(const std::string & channel_name, bool default_value)
{
  const std::string enabled_name = channel_name + ".enabled";
  const std::string enable_name = channel_name + ".enable";

  bool enabled = default_value;
  bool enable = default_value;
  this->get_parameter(enabled_name, enabled);
  this->get_parameter(enable_name, enable);

  return enabled && enable;
}

StmCanSendNode::ChannelConfig StmCanSendNode::LoadChannelConfig(
  const std::string & channel_name,
  uint32_t default_can_id)
{
  ChannelConfig channel;
  channel.name = channel_name;
  channel.enabled = GetChannelEnabled(channel_name, true);
  channel.topic = this->get_parameter(channel_name + ".topic").as_string();
  channel.can_id = static_cast<uint32_t>(
    this->get_parameter(channel_name + ".can_id").as_int());
  channel.override_can_id = this->get_parameter(channel_name + ".override_can_id").as_bool();

  if (channel.topic.empty()) {
    channel.topic = "/" + channel_name + "/can_frame";
  }
  if (channel.can_id == 0) {
    channel.can_id = default_can_id;
  }

  return channel;
}

void StmCanSendNode::OpenCanSocket()
{
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    throw std::runtime_error(
            "failed to open CAN socket: " + std::string(std::strerror(errno)));
  }

  ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    const std::string error_message = std::strerror(errno);
    CloseCanSocket();
    throw std::runtime_error(
            "failed to find CAN interface " + can_interface_ + ": " + error_message);
  }

  sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    const std::string error_message = std::strerror(errno);
    CloseCanSocket();
    throw std::runtime_error(
            "failed to bind CAN interface " + can_interface_ + ": " + error_message);
  }
}

void StmCanSendNode::CloseCanSocket()
{
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
  }
}

void StmCanSendNode::SetupSubscriptions()
{
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
          SendFrame(msg, channel);
        }));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed channel=%s topic=%s can_id=0x%X override_can_id=%s",
      channel.name.c_str(),
      channel.topic.c_str(),
      channel.can_id,
      channel.override_can_id ? "true" : "false");
  }
}

void StmCanSendNode::SendFrame(
  const can_msgs::msg::Frame::SharedPtr msg,
  const ChannelConfig & channel)
{
  if (can_socket_ < 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "CAN socket is not open");
    return;
  }

  if (msg->dlc > kMaxClassicCanDlc) {
    RCLCPP_WARN(
      this->get_logger(),
      "Dropping frame from %s: dlc=%u exceeds classic CAN limit",
      channel.topic.c_str(),
      msg->dlc);
    return;
  }

  can_frame frame {};
  frame.can_id = BuildCanId(*msg, channel.can_id, channel.override_can_id);
  frame.can_dlc = msg->dlc;
  for (uint8_t i = 0; i < msg->dlc; ++i) {
    frame.data[i] = msg->data[i];
  }

  const ssize_t written = write(can_socket_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to send CAN frame on %s: %s",
      can_interface_.c_str(),
      std::strerror(errno));
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StmCanSendNode>());
  rclcpp::shutdown();
  return 0;
}
