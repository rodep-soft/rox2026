#include "stm32_driver/stm32_protocol.hpp"

#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace stm32_driver
{
namespace protocol
{

static_assert(sizeof(float) == 4, "The CAN protocol requires 32-bit float");
static_assert(
  std::numeric_limits<float>::is_iec559,
  "The CAN protocol requires IEEE-754 float");

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

void encode_float_le(float value, std::array<uint8_t, 8> & data)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  for (std::size_t i = 0; i < sizeof(raw); ++i) {
    data[i] = static_cast<uint8_t>((raw >> (i * 8U)) & 0xffU);
  }
}

float decode_float_le(const std::array<uint8_t, 8> & data)
{
  uint32_t raw = 0;
  for (std::size_t i = 0; i < sizeof(raw); ++i) {
    raw |= static_cast<uint32_t>(data[i]) << (i * 8U);
  }

  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

bool is_standard_data_frame(const can_msgs::msg::Frame & frame)
{
  return !frame.is_extended && !frame.is_rtr && !frame.is_error;
}

}  // namespace

can_msgs::msg::Frame make_motor_target_frame(std::size_t motor, float rpm)
{
  if (motor >= MOTOR_NUM) {
    throw std::out_of_range("motor index is outside the CAN protocol range");
  }

  auto frame = make_data_frame(
    MOTOR_TARGET_BASE + static_cast<uint32_t>(motor), sizeof(float));
  encode_float_le(rpm, frame.data);
  return frame;
}

can_msgs::msg::Frame make_alive_frame()
{
  return make_data_frame(HEARTBEAT_TX, 0);
}

can_msgs::msg::Frame make_led_frame(bool enable)
{
  auto frame = make_data_frame(LED_CMD, 1);
  frame.data[0] = enable ? 1U : 0U;
  return frame;
}

bool decode_motor_current(
  const can_msgs::msg::Frame & frame, std::size_t & motor, float & rpm)
{
  if (!is_standard_data_frame(frame) || frame.dlc != sizeof(float) ||
    frame.id < MOTOR_CURRENT_RPM_BASE ||
    frame.id >= MOTOR_CURRENT_RPM_BASE + MOTOR_NUM)
  {
    return false;
  }

  motor = static_cast<std::size_t>(frame.id - MOTOR_CURRENT_RPM_BASE);
  rpm = decode_float_le(frame.data);
  return true;
}

bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state)
{
  if (!is_standard_data_frame(frame) || frame.id != LIMIT_SWITCH_STATE || frame.dlc != 1) {
    return false;
  }

  state = frame.data[0];
  return true;
}

bool is_heartbeat_response(const can_msgs::msg::Frame & frame)
{
  return is_standard_data_frame(frame) && frame.id == HEARTBEAT_RX && frame.dlc == 0;
}

}  // namespace stm32_driver::protocol
