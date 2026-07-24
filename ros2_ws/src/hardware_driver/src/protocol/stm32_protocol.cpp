#include "stm32_driver/stm32_protocol.hpp"

#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace stm32_driver::protocol
{

static_assert(sizeof(float) == 4, "The CAN protocol requires 32-bit float");
static_assert(
  std::numeric_limits<float>::is_iec559,
  "The CAN protocol requires IEEE-754 float");

/// @brief IDとデータ長よりcanFrameの根幹を作る
/// @param id ID
/// @param dlc データ長
/// @return 送信用のcanFrame
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

/// @brief int16_t型の数値をuint8_tの配列に変換するお互いにリトルエンディアンのためバイト順は変えずそのまま
/// @param value int16_t型の数値
/// @param data uint8_tの配列
void encode_int16_le(int16_t value, std::array<uint8_t, 8> & data)
{
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}
/// @brief uint8_tの配列よりint16_t型の数値をデコードする
/// @param data uint8_t型の配列
/// @return int16_t型の値
int16_t decode_int16_le(const std::array<uint8_t, 8> & data)
{
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |(static_cast<uint16_t>(data[1]) << 8));
}

/// @brief　正常なcanFrameかを判断する
/// @param frame　受信したcanFrame
/// @return true:正常なcanFrameである false:求めているcanFrameではない
bool is_standard_data_frame(const can_msgs::msg::Frame & frame)
{
  return !frame.is_extended && !frame.is_rtr && !frame.is_error;
}

can_msgs::msg::Frame make_motor_target_frame(std::size_t motor, int16_t rpm)
{
  if (motor >= MOTOR_NUM) {
    throw std::out_of_range("motor index is outside the CAN protocol range");
  }

  auto frame = make_data_frame(
    MOTOR_TARGET_RPM_BASE + static_cast<uint32_t>(motor), sizeof(int16_t));
  encode_int16_le(rpm, frame.data);
  return frame;
}

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

bool decode_motor_current(const can_msgs::msg::Frame & frame, std::size_t & motor, int16_t & rpm)
{
  if (frame.dlc != sizeof(int16_t) ||
    frame.id < MOTOR_CURRENT_RPM_BASE ||
    frame.id >= MOTOR_CURRENT_RPM_BASE + MOTOR_NUM)
  {
    return false;
  }

  motor = static_cast<std::size_t>(frame.id - MOTOR_CURRENT_RPM_BASE);
  rpm = decode_int16_le(frame.data);
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

} // namespace stm32_driver::protocol
