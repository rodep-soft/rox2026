#include "LED_lite.h"

/* main.c で定義されているTIM15のハンドルを外部参照 */
extern TIM_HandleTypeDef htim15;

/* * 前回の計算に基づくパルス幅（Duty）の設定
 * Timer Clock: 8MHz, ARR: 9 (周期1.25us)
 * '0' のパルス幅 ≒ 3カウント (0.375us)
 * '1' のパルス幅 ＝ 8カウント (1.0us)
 */
#define PWM_LOW_PULSE  3
#define PWM_HIGH_PULSE 7

/* * DMA転送用のPWMバッファ
 * 30個 × 24ビット + リセット信号(Treset)用の余白50回分
 */
uint16_t pwmData[(NUM_LEDS * 24) + 50];

/* 各LEDの色データを保持する配列 (Green, Red, Blue) */
uint8_t LED_Data[NUM_LEDS][3];


/**
 * @brief すべてのLEDの色データを「赤」にセットする（この時点ではまだ光りません）
 */
void LED_SetAllRed(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        // SK6812等のNeoPixel系は、Green -> Red -> Blue の順(GRB)でデータを送ります
        LED_Data[i][0] = 0;     // Green
        LED_Data[i][1] = 255;   // Red
        LED_Data[i][2] = 0;     // Blue
    }
}

/**
 * @brief 色データをPWMのDuty値に変換し、DMAを使って一気に送信する
 */
void LED_Update(void) {
    uint32_t indx = 0;

    // 各LEDの色データをビット単位に分解し、PWMのパルス幅(0か1か)に変換
    for (int i = 0; i < NUM_LEDS; i++) {
        // 1. Green
        for (int j = 7; j >= 0; j--) {
            pwmData[indx++] = (LED_Data[i][0] & (1 << j)) ? PWM_HIGH_PULSE : PWM_LOW_PULSE;
        }
        // 2. Red
        for (int j = 7; j >= 0; j--) {
            pwmData[indx++] = (LED_Data[i][1] & (1 << j)) ? PWM_HIGH_PULSE : PWM_LOW_PULSE;
        }
        // 3. Blue
        for (int j = 7; j >= 0; j--) {
            pwmData[indx++] = (LED_Data[i][2] & (1 << j)) ? PWM_HIGH_PULSE : PWM_LOW_PULSE;
        }
    }

    // 最後にリセットコード（Treset）として、Duty 0 のパルスを連続して送信
    for (int i = 0; i < 50; i++) {
        pwmData[indx++] = 0;
    }

    // TIM15 Channel 1 を使ってDMA転送を開始
    HAL_TIM_PWM_Start_DMA(&htim15, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
}

/**
 * @brief DMAの転送完了コールバック
 * 転送が終わったら自動的にPWM出力を停止して次の送信に備える
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM15) {
        HAL_TIM_PWM_Stop_DMA(&htim15, TIM_CHANNEL_1);
    }
}
