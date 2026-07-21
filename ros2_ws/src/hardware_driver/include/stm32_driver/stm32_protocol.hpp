#pragma once

#include <cstddef>
#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace stm32_driver::protocol
{
// Classic CAN (11-bit ID) protocol shared with the STM32 firmware.
// stm32から受け取るタイムアウト検知信号
constexpr uint32_t HEARTBEAT_TX = 0x100;
// rdkから送るタイムアウト検知信号
constexpr uint32_t HEARTBEAT_RX = 0x101;

//LED点灯用のコマンド送信
constexpr uint32_t LED_CMD = 0x201;
//ブラシレスモータRPM送信用ベース [0x230 , 0x231 , 0x232]
constexpr uint32_t MOTOR_TARGET_BASE = 0x230;
//PIDゲイン送信用
constexpr uint32_t MOTOR_PID_BASE = 0x240;

//リミットスイッチデータ受け取り
constexpr uint32_t LIMIT_SWITCH_STATE = 0x310;
//ブラシレスモータPRM受信用ベース [0x230 , 0x231 , 0x232]
constexpr uint32_t MOTOR_CURRENT_RPM_BASE = 0x330;
//PIDゲイン設定完了ACK
constexpr uint32_t PID_ACK_BASE = 0x340;

//モーター数
constexpr std::size_t MOTOR_NUM = 3;

// float values are encoded as IEEE-754 binary32, least-significant byte first.

/// @brief ブラシレスモーター用のcanFrameの生成
/// @param motor　ブラシレスモーターのインデックス 
/// @param rpm 目標回転速度
/// @return canFrame 
can_msgs::msg::Frame make_motor_target_frame(std::size_t motor, float rpm);

/// @brief タイムアウト検知用の空のcanFrame生成
/// @return 
can_msgs::msg::Frame make_alive_frame();

/// @brief ledコマンド送信用にcanFrame生成
/// @param enable 
/// @return canFrame
can_msgs::msg::Frame make_led_frame(bool enable);

/// @brief 現在のモータのrpmのcanFrameのデコード
/// @param frame canFrame
/// @param motor 
/// @param rpm 
/// @return 
bool decode_motor_current(
  const can_msgs::msg::Frame & frame, std::size_t & motor, float & rpm);

/// @brief 
/// @param frame 
/// @param state 
/// @return 
bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state);

/// @brief 
/// @param frame 
/// @return 
bool is_heartbeat_response(const can_msgs::msg::Frame & frame);

}  // namespace stm32_driver::protocol
