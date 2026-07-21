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

/// @brief 現在のモータのrpmのcanFrameのデコード
/// @param frame canFrame
/// @param motor モータのインデックス[0 - num]
/// @param rpm 現在のモータのrpm
/// @return true:正常にデータを受け取れた false:受信していない
bool decode_motor_current(
  const can_msgs::msg::Frame & frame, std::size_t & motor, float & rpm);

/// @brief リミットスイッチのデータを受信しstateに保存
/// @param frame 受信したcanFrame
/// @param state 保存先の変数
/// @return true:正常なデータを受け取れた false:受信していない
bool decode_limit_switch(const can_msgs::msg::Frame & frame, uint8_t & state);

/// @brief canFrameの形式があっているかを確認し，受信できていることを判断する
/// @param frame canFrame
/// @return true:受信した false:受信していない
bool is_heartbeat_response(const can_msgs::msg::Frame & frame);

}  // namespace stm32_driver::protocol
