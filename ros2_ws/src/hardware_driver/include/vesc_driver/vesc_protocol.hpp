#include <array>
#include <cstdint>
#include "can_msgs/msg/frame.hpp"

namespace vesc_driver::protocol
{
constexpr uint32_t SET_RPM = 3;
constexpr uint32_t STATUS_1 = 9;
constexpr int64_t MOTOR_POLES = 14;   // モーターの極数

struct Status1
{
  uint8_t controller_id;
  int32_t erpm;
};

/// @brief escに送る速度制御用のcanFrame生成
/// @param id モータID
/// @param erpm 実際のrpm * 極数
/// @return canFrame
can_msgs::msg::Frame make_set_rpm_frame(uint8_t id, int32_t erpm)
{
  can_msgs::msg::Frame frame{};
  frame.id = (SET_RPM << 8) | id;
  frame.is_extended = true;
  frame.dlc = 4;

  const auto value = static_cast<uint32_t>(erpm);
  for (std::size_t i = 0; i < 4; ++i) {
    frame.data[i] = static_cast<uint8_t>((value >> (24 - 8 * i)) & 0xFF);
  }
  return frame;
}

/// @brief status1のcanFrameをデコードして，速度データの取り出し
/// @param frame
/// @param status
/// @return true:デコードした false:デコードしていない
bool decode_status_1(const can_msgs::msg::Frame & frame, Status1 & status)
{
  if (!frame.is_extended || frame.is_rtr || frame.is_error || frame.dlc != 8 ||
    (frame.id >> 8) != STATUS_1)
  {
    return false;
  }
  status.controller_id = static_cast<uint8_t>(frame.id & 0xFF);
  const uint32_t rpm =
    (static_cast<uint32_t>(frame.data[0]) << 24) |
    (static_cast<uint32_t>(frame.data[1]) << 16) |
    (static_cast<uint32_t>(frame.data[2]) << 8) |
    static_cast<uint32_t>(frame.data[3]);
  status.erpm = static_cast<int32_t>(rpm);

  return true;
}
} // namespace vesc_driver::protocol
