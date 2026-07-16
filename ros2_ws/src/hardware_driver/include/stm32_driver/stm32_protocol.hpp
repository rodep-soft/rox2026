#pragma once

#include <cstdint>
#include "can_msgs/msg/frame.hpp"

//通信仕様について変更可能性が低いため，yamlではなくここにまとめる
namespace stm32_driver::protocol
{
constexpr uint32_t HURT_BEAT_TX = 0x100;//タイムアウト
constexpr uint32_t HURT_BEAT_RX = 0x101;

constexpr uint32_t LED_CMD = 0x201;

constexpr uint32_t MOTOR_TARGET_BASE = 0x230;
constexpr uint32_t MOTOR_PID_BASE    = 0x240;

constexpr uint32_t MOTOR_CURRENT_RPM_BASE = 0x330;
constexpr uint32_t PID_ACK_BASE           = 0x340;

constexpr size_t MOTOR_NUM = 3;//モーターデータの数

can_msgs::msg::Frame make_otor_target_frame(uint8_t motor,float rpm);

can_msgs::msg::Frame make_alive_frame();

can_msgs::msg::Frame make_led_frame(bool enable);

bool decodeMotorCurrent(const can_msgs::msg::Frame & frame,uint8_t & motor,float & rpm);
}