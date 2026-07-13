#include "limit_switch.h"
#include "main.h" // GPIOの定義(LIMIT_SW1_Pinなど)を読み込むため必要

// 最後にCAN送信した時刻を記録しておく
static uint32_t last_send_time = 0;

// 送信回数を記録するデバッグ用変数
volatile uint32_t debug_tx_count = 0;

void LimitSwitch_UpdateAndSend(CAN_HandleTypeDef *hcan) {

    // 現在の時刻を取得（ミリ秒）
    uint32_t current_time = HAL_GetTick();

    // 前回送信した時刻から10ms以上経過しているかチェック
    if ((current_time - last_send_time) >= 10) {

        // 送信時刻を更新
        last_send_time = current_time;

        // 4つのピンの状態を読み取る
        GPIO_PinState sw1 = HAL_GPIO_ReadPin(LIMIT_SW1_GPIO_Port, LIMIT_SW1_Pin);
        GPIO_PinState sw2 = HAL_GPIO_ReadPin(LIMIT_SW2_GPIO_Port, LIMIT_SW2_Pin);
        GPIO_PinState sw3 = HAL_GPIO_ReadPin(LIMIT_SW3_GPIO_Port, LIMIT_SW3_Pin);
        GPIO_PinState sw4 = HAL_GPIO_ReadPin(LIMIT_SW4_GPIO_Port, LIMIT_SW4_Pin);

        CAN_TxHeaderTypeDef TxHeader;
        uint8_t TxData[8] = {0};
        uint32_t TxMailbox;

        TxHeader.StdId = 0x202;
        TxHeader.ExtId = 0;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 1; // 1バイト送信で十分

        // TxData[0] を一度 0 で初期化
        TxData[0] = 0;

        // スイッチが押された(RESET)なら、該当するビットを 1 にする
        // (離されている時は 0 のままになる)
        if (sw1 == GPIO_PIN_RESET) { TxData[0] |= (1 << 0); } // 0ビット目を立てる
        if (sw2 == GPIO_PIN_RESET) { TxData[0] |= (1 << 1); } // 1ビット目を立てる
        if (sw3 == GPIO_PIN_RESET) { TxData[0] |= (1 << 2); } // 2ビット目を立てる
        if (sw4 == GPIO_PIN_RESET) { TxData[0] |= (1 << 3); } // 3ビット目を立てる

        // 送信をリクエストし、無事にバッファに入った場合のみカウントを増やす
        if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) {
            debug_tx_count++;
        }
    }
}
