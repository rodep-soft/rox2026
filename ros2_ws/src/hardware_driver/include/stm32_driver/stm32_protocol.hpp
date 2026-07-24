#pragma once

#include <cstddef>
#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace stm32_driver::protocol
{
// stm32から受け取るタイムアウト検知信号
constexpr uint32_t HEARTBEAT_FROM_STM = 0x100;

// rdkから送るタイムアウト検知信号
constexpr uint32_t HEARTBEAT_TO_STM = 0x101;

//LED点灯用のコマンド送信
constexpr uint32_t LED_CMD = 0x201;
//ブラシレスモータRPM送信用ベース [0x230 , 0x231 , 0x232]
constexpr uint32_t MOTOR_TARGET_RPM_BASE = 0x230;
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

/// @brief ブラシレスモーター用のcanFrameの生成
/// @param motor　ブラシレスモーターのインデックス
/// @param rpm 目標回転速度
/// @return canFrame
can_msgs::msg::Frame make_motor_target_frame(std::size_t motor, float rpm);

/// @brief 自身の生存報告用のcanFrame生成
/// @return canFrame
can_msgs::msg::Frame make_alive_frame();

/// @brief LEDコマンド送信用のcanFrame生成
/// @param command uint8_t型
/// @return canFrame
can_msgs::msg::Frame make_led_frame(uint8_t command);

/// @brief 標準CANデータフレームかを確認する
/// @param frame 受信したCANフレーム
/// @return true: 標準CANデータフレーム、false: それ以外
bool is_standard_data_frame(const can_msgs::msg::Frame & frame);

/// @brief ブラシレスモーターの現在RPMをデコードする
/// @pre is_standard_data_frame(frame)がtrueであること
/// @param frame CANフレーム
/// @param motor モーターのインデックス
/// @param rpm デコードした現在RPM
/// @return true: IDとDLCが正しい、false: それ以外
bool decode_motor_current(
  const can_msgs::msg::Frame & frame, std::size_t & motor, float & rpm);

/// @brief リミットスイッチの状態をデコードする
/// @pre is_standard_data_frame(frame)がtrueであること
/// @param frame CANフレーム
/// @param state デコードしたリミットスイッチの状態
/// @return true: IDとDLCが正しい、false: それ以外
bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state);

/// @brief STM32からのheartbeat応答かを確認する
/// @pre is_standard_data_frame(frame)がtrueであること
/// @param frame CANフレーム
/// @return true: IDとDLCが正しい、false: それ以外
bool is_heartbeat_response(const can_msgs::msg::Frame & frame);

}  // namespace stm32_driver::protocol
