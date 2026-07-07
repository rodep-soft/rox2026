#include "limit_switch.h"

// 最後にCAN送信した時刻を記録しておく
static uint32_t last_send_time = 0;

void LimitSwitch_UpdateAndSend(CAN_HandleTypeDef *hcan) {

    // 現在の時刻を取得（ミリ秒）
    uint32_t current_time = HAL_GetTick();

    // 前回送信した時刻から10ms以上経過しているかチェック
    if ((current_time - last_send_time) >= 10) {

        // 送信時刻を更新
        last_send_time = current_time;

        // ピンの状態を読み取る
        GPIO_PinState current_state = HAL_GPIO_ReadPin(LIMIT_SW_GPIO_Port, LIMIT_SW_Pin);

        CAN_TxHeaderTypeDef TxHeader;
        uint8_t TxData[8] = {0};
        uint32_t TxMailbox;

        TxHeader.StdId = 0x202;
        TxHeader.ExtId = 0;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 1; // 1バイト送信

        // スイッチが押された(RESET)なら1、離された(SET)なら0
        if (current_state == GPIO_PIN_RESET) {
            TxData[0] = 1;
        } else {
            TxData[0] = 0;
        }

        HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    }
}
