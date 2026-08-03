#pragma once

#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace stm32_driver::protocol
{

// STM32から受信するheartbeat
constexpr uint32_t HEARTBEAT_FROM_STM = 0x100;
// STM32へ送信するheartbeat
constexpr uint32_t HEARTBEAT_TO_STM = 0x101;
// LED点灯用コマンド
constexpr uint32_t LED_CMD = 0x201;

// リミットスイッチの状態
constexpr uint32_t LIMIT_SWITCH_STATE = 0x310;
// クォータニオン（X, Y, Z, W）
constexpr uint32_t QUATERNION = 0x320;

constexpr double QUATERNION_SCALE_INV = 1.0 / 16384.0;

/// @brief 生存報告用CANフレームを生成する
/// @return 送信用CANフレーム
can_msgs::msg::Frame make_alive_frame();

/// @brief LEDコマンド用CANフレームを生成する
/// @param command LEDコマンド
/// @return 送信用CANフレーム
can_msgs::msg::Frame make_led_frame(uint8_t command);

/// @brief 標準CANデータフレームかを確認する
/// @param frame 受信したCANフレーム
/// @return true: 標準CANデータフレーム、false: それ以外
bool is_standard_data_frame(const can_msgs::msg::Frame & frame);

/// @brief 1フレームのクォータニオンをデコードする
/// @pre is_standard_data_frame(frame)がtrueであること
/// @param frame 受信したCANフレーム
/// @param x X成分
/// @param y Y成分
/// @param z Z成分
/// @param w W成分
/// @return true: IDとDLCが正しい、false: それ以外
bool decode_quaternion(
  const can_msgs::msg::Frame & frame, int16_t & x, int16_t & y, int16_t & z, int16_t & w);

/// @brief リミットスイッチの状態をデコードする
/// @pre is_standard_data_frame(frame)がtrueであること
/// @param frame 受信したCANフレーム
/// @param state デコードしたリミットスイッチの状態
/// @return true: IDとDLCが正しい、false: それ以外
bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state);

/// @brief STM32からのheartbeat応答かを確認する
/// @pre is_standard_data_frame(frame)がtrueであること
/// @param frame 受信したCANフレーム
/// @return true: IDとDLCが正しい、false: それ以外
bool is_heartbeat_response(const can_msgs::msg::Frame & frame);

}  // namespace stm32_driver::protocol
