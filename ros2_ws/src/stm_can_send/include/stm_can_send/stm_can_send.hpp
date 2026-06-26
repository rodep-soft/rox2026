#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

class StmCanSendNode : public rclcpp::Node
{
public:
  StmCanSendNode();
  ~StmCanSendNode() override;

private:
  struct ChannelConfig
  {
    std::string name;
    bool enabled;
    std::string topic;
    uint32_t can_id;
    bool override_can_id;
  };

  void DeclareParameters();
  void LoadParameters();
  void OpenCanSocket();
  void CloseCanSocket();
  void SetupSubscriptions();
  void SendFrame(const can_msgs::msg::Frame::SharedPtr msg, const ChannelConfig & channel);

  bool GetChannelEnabled(const std::string & channel_name, bool default_value);
  ChannelConfig LoadChannelConfig(const std::string & channel_name, uint32_t default_can_id);

  std::string can_interface_;
  std::vector<ChannelConfig> channels_;
  std::vector<rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr> subscriptions_;
  int can_socket_;
};
