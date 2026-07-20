#pragma once

#include <cstddef>
#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace stm32_driver::protocol
{

// Classic CAN (11-bit ID) protocol shared with the STM32 firmware.
constexpr uint32_t HEARTBEAT_TX = 0x100;
constexpr uint32_t HEARTBEAT_RX = 0x101;
constexpr uint32_t LED_CMD = 0x201;
constexpr uint32_t LIMIT_SWITCH_STATE = 0x202;
constexpr uint32_t MOTOR_TARGET_BASE = 0x230;
constexpr uint32_t MOTOR_PID_BASE = 0x240;
constexpr uint32_t MOTOR_CURRENT_RPM_BASE = 0x330;
constexpr uint32_t PID_ACK_BASE = 0x340;
constexpr std::size_t MOTOR_NUM = 3;

// float values are encoded as IEEE-754 binary32, least-significant byte first.
can_msgs::msg::Frame make_motor_target_frame(std::size_t motor, float rpm);
can_msgs::msg::Frame make_alive_frame();
can_msgs::msg::Frame make_led_frame(bool enable);

bool decode_motor_current(
  const can_msgs::msg::Frame & frame, std::size_t & motor, float & rpm);
bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state);
bool is_heartbeat_response(const can_msgs::msg::Frame & frame);

}  // namespace stm32_driver::protocol
