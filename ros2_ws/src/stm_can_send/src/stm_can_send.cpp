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
// received_ と frame_data_ は固定添字で扱う。
// roller は frame_data_[0..1]、belt は frame_data_[2..3] に入れる。
constexpr uint8_t kRollerIndex = 0;
constexpr uint8_t kBeltIndex = 1;
constexpr uint8_t kRollerDataOffset = 0;
constexpr uint8_t kBeltDataOffset = 2;

canid_t BuildCanId(uint32_t configured_id)
{
  return static_cast<canid_t>(configured_id);
}
}  // namespace

StmCanSendNode::StmCanSendNode()
: Node("stm_can_send_node"), can_socket_(-1)
{
  // パラメータを読み込んで CAN ソケットを開いたあと、各チャンネルの購読を開始する。
  DeclareParameters();
  GetParameters();

  OpenCanSocket();

  // 有効なチャンネルだけ購読する。購読先topicとpayload配置はchannels_に入っている。
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

    // ちゃんとTopic流れてることが確認出来たら削除してよろしい
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed channel=%s topic=%s data_offset=%u",
      channel.name.c_str(), channel.topic.c_str(), channel.data_offset);
  }

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
  // launch/configから上書きできる値をここで宣言する。
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<int>("can_id", 0x201);
  this->declare_parameter<bool>("roller_enabled", true);
  this->declare_parameter<std::string>("roller_topic", "/roller/can_frame");
  this->declare_parameter<bool>("belt_enabled", true);
  this->declare_parameter<std::string>("belt_topic", "/belt/can_frame");
}

void StmCanSendNode::GetParameters()
{
  // SocketCANの出力先と、STMへ送るCAN IDを読み込む。
  can_interface_ = this->get_parameter("can_interface").as_string();
  can_id_ = static_cast<uint32_t>(this->get_parameter("can_id").as_int());

  channels_.clear();

  // STM 側の受信処理を単純にするため、チャンネル位置は launch 構成に関係なく固定する。
  ChannelConfig roller;
  roller.name = "roller";
  roller.enabled = this->get_parameter("roller_enabled").as_bool();
  roller.topic = this->get_parameter("roller_topic").as_string();
  roller.index = kRollerIndex;
  roller.data_offset = kRollerDataOffset;
  if (roller.topic.empty()) {
    roller.topic = "/roller/can_frame";
  }
  channels_.push_back(roller);

  ChannelConfig belt;
  belt.name = "belt";
  belt.enabled = this->get_parameter("belt_enabled").as_bool();
  belt.topic = this->get_parameter("belt_topic").as_string();
  belt.index = kBeltIndex;
  belt.data_offset = kBeltDataOffset;
  if (belt.topic.empty()) {
    belt.topic = "/belt/can_frame";
  }
  channels_.push_back(belt);
}

void StmCanSendNode::OpenCanSocket()
{
  // Linux の SocketCAN を使って、指定された can_interface_ に直接 CAN フレームを書き込む。
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    throw std::runtime_error(
            "failed to open CAN socket: " + std::string(std::strerror(errno)));
  }

  ifreq ifr {};
  // "can0" や "vcan0" のようなinterface名を、kernelへ渡す構造体にコピーする。
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  // interface名からkernel内部のinterface indexを取得する。
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    const std::string error_message = std::strerror(errno);
    CloseCanSocket();
    throw std::runtime_error(
            "failed to find CAN interface " + can_interface_ + ": " + error_message);
  }

  sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  // socketを指定CAN interfaceに結びつける。以降のwrite()はこのinterfaceへ出る。
  if (bind(can_socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    const std::string error_message = std::strerror(errno);
    CloseCanSocket();
    throw std::runtime_error(
            "failed to bind CAN interface " + can_interface_ + ": " + error_message);
  }
}

void StmCanSendNode::CloseCanSocket()
{
  // destructorから呼ばれる。二重closeを避けるため、閉じた後は-1に戻す。
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
  }
}

void StmCanSendNode::FrameCallback(
  const can_msgs::msg::Frame::SharedPtr msg,
  const ChannelConfig & channel)
{
  if (msg->dlc < 2) {
    RCLCPP_WARN(
      this->get_logger(),
      "Dropping frame from %s: dlc=%u is too short",
      channel.topic.c_str(),
      msg->dlc);
    return;
  }

  // 入力frameの先頭2byteだけを、そのチャンネルに割り当てられた位置へコピーする。
  frame_data_[channel.data_offset] = msg->data[0];
  frame_data_[channel.data_offset + 1] = msg->data[1];
  received_[channel.index] = true;

  // enabled な入力が全部一度は届くまで送らない。
  // 片側を launch で無効化した場合、そのチャンネルは受信待ち対象から外れる。
  for (const auto & configured_channel : channels_) {
    if (configured_channel.enabled && !received_[configured_channel.index]) {
      return;
    }
  }

  SendFrame();
}

void StmCanSendNode::SendFrame()
{
  if (can_socket_ < 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "CAN socket is not open");
    return;
  }

  can_frame frame {};
  frame.can_id = BuildCanId(can_id_);
  frame.can_dlc = sizeof(frame_data_);

  // frame_data_ は常に 4 byte 固定。無効チャンネルの領域は初期値 0 のまま残る。
  for (uint8_t i = 0; i < sizeof(frame_data_); ++i) {
    frame.data[i] = frame_data_[i];
  }

  // SocketCANへ直接writeする。ここではROS topicではなく実CAN interfaceへ送信する。
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
