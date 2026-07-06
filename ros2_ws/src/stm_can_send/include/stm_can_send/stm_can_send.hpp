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
    // received_ の添字。data_offset とは別にして、受信待ち管理を明示する。
    uint8_t index;
    // STM へ送る 1 つの CAN frame 内で、このチャンネルの 2 byte を置く先頭位置。
    uint8_t data_offset;
  };

  void DeclareParameters();
  void GetParameters();

  // CAN
  void OpenCanSocket();
  void CloseCanSocket();

  // CallBack
  void FrameCallback(const can_msgs::msg::Frame::SharedPtr msg, const ChannelConfig & channel);
  void SendFrame();

  std::string can_interface_;
  uint32_t can_id_;

  // roller / belt それぞれの有効状態、入力topic、frame_data_内の配置をまとめた設定。
  std::vector<ChannelConfig> channels_;

  // create_subscription() の戻り値を保持する。保持しないとsubscriptionが破棄される。
  std::vector<rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr> subscriptions_;

  // STMへ送る4byte payload。
  // 固定配置: roller=data[0..1], belt=data[2..3]。無効チャンネルの領域は0のまま送る。
  uint8_t frame_data_[4] {};

  // 有効な入力が一度は届いたかを管理する。
  // received_[0]=roller, received_[1]=belt。全部届くまでSendFrame()しない。
  bool received_[2] {};

  // SocketCANのfile descriptor。-1は未オープン、0以上は有効なsocket。
  int can_socket_;
};
