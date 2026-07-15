#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

// モータ通信プロトコルの共通部分。
// 派生クラスは initialize(), set_velocity(), stop() を実装する。
class EduLite05Protocol
{
public:
  EduLite05Protocol(
    std::uint8_t motor_id,
    const rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr & can_publisher)
  : motor_id_(motor_id), can_publisher_(can_publisher)
  {
  }

  virtual ~EduLite05Protocol() = default;

  virtual void initialize() = 0;
  virtual void set_velocity(float velocity_rad_s) = 0;
  virtual void stop() = 0;

protected:
  // 送信CAN ID: bit 24--28 = communication type,
  // bit 8--23 = host ID, bit 0--7 = motor ID
  static constexpr std::uint32_t kHostId = 0xFD00;

  can_msgs::msg::Frame make_frame(std::uint8_t communication_type) const
  {
    can_msgs::msg::Frame frame{};
    frame.id = (static_cast<std::uint32_t>(communication_type) << 24) + kHostId + motor_id_;
    frame.is_extended = true;
    frame.dlc = 8;
    std::fill(frame.data.begin(), frame.data.end(), 0U);
    return frame;
  }

  void publish(can_msgs::msg::Frame frame) const
  {
    can_publisher_->publish(std::move(frame));
  }

  static void put_float_le(can_msgs::msg::Frame & frame, std::size_t offset, float value)
  {
    const auto bytes = std::bit_cast<std::array<std::uint8_t, 4>>(value);
    std::copy(bytes.begin(), bytes.end(), frame.data.begin() + offset);
  }

  static void put_index(can_msgs::msg::Frame & frame, std::uint16_t index)
  {
    // EDULITE05の単一パラメータ書込み: data[0..1] = index (little-endian)
    frame.data[0] = static_cast<std::uint8_t>(index & 0xFFU);
    frame.data[1] = static_cast<std::uint8_t>((index >> 8) & 0xFFU);
  }

  std::uint8_t motor_id_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
};
