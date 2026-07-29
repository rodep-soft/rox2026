#include "stm32_driver/stm32_protocol.hpp"

#include <array>
#include <cstddef>

namespace stm32_driver::protocol
{
namespace
{

can_msgs::msg::Frame make_data_frame(uint32_t id, uint8_t dlc)
{
  can_msgs::msg::Frame frame{};
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;
  frame.dlc = dlc;
  return frame;
}

int16_t decode_int16_le(const std::array<uint8_t, 8> & data, std::size_t offset)
{
  const auto raw = static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8U);
  return static_cast<int16_t>(raw);
}

}  // namespace

can_msgs::msg::Frame make_alive_frame()
{
  return make_data_frame(HEARTBEAT_TO_STM, 0);
}

can_msgs::msg::Frame make_led_frame(uint8_t command)
{
  auto frame = make_data_frame(LED_CMD, 1);
  frame.data[0] = command;
  return frame;
}

bool is_standard_data_frame(const can_msgs::msg::Frame & frame)
{
  return !frame.is_extended && !frame.is_rtr && !frame.is_error;
}

bool decode_quaternion(
  const can_msgs::msg::Frame & frame, int16_t & x, int16_t & y, int16_t & z, int16_t & w)
{
  if (frame.id != QUATERNION || frame.dlc != 4 * sizeof(int16_t)) {
    return false;
  }

  x = decode_int16_le(frame.data, 0);
  y = decode_int16_le(frame.data, sizeof(int16_t));
  z = decode_int16_le(frame.data, 2 * sizeof(int16_t));
  w = decode_int16_le(frame.data, 3 * sizeof(int16_t));
  return true;
}

bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state)
{
  if (frame.id != LIMIT_SWITCH_STATE || frame.dlc != 1) {
    return false;
  }

  state = frame.data[0];
  return true;
}

bool is_heartbeat_response(const can_msgs::msg::Frame & frame)
{
  return frame.id == HEARTBEAT_FROM_STM && frame.dlc == 0;
}

}  // namespace stm32_driver::protocol
