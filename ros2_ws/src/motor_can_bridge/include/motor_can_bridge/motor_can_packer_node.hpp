#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

class MotorCanPackerNode : public rclcpp::Node
{
public:
  MotorCanPackerNode();

private:
  struct ChannelConfig
  {
    std::string name;
    std::string topic;
    bool enabled;
    uint8_t received_index;
    uint8_t data_offset;
  };

  void DeclareParameters();
  void GetParameters();
  void SetupRosInterfaces();

  void FrameCallback(const can_msgs::msg::Frame::SharedPtr msg, const ChannelConfig & channel);
  bool HasReceivedAllEnabledChannels() const;
  can_msgs::msg::Frame CreatePackedFrame(const rclcpp::Time & stamp) const;

  std::string can_tx_topic_;
  uint32_t can_id_;
  bool is_extended_;

  std::vector<ChannelConfig> channels_;
  std::vector<rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr> subscriptions_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;

  std::array<uint8_t, 8> frame_data_ {};
  std::array<bool, 2> received_ {};
};
