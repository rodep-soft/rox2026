#include "LED_lite.h"

/* main.c で定義されているTIM15のハンドルを外部参照 */
extern TIM_HandleTypeDef htim3;

/* * 前回の計算に基づくパルス幅（Duty）の設定
 * Timer Clock: 8MHz, ARR: 9 (周期1.25us)
 * '0' のパルス幅 ≒ 3カウント (0.375us)
 * '1' のパルス幅 ＝ 8カウント (1.0us)
 */
#define LED_LOW  18
#define LED_HIGH 38
#define RESET_SLOTS 100


uint32_t index = 0;

#define LED_NUM 10

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB_t;

RGB_t ledBuffer[LED_NUM];

/* * DMA転送用のPWMバッファ
 * 30個 × 24ビット + リセット信号(Treset)用の余白50回分
 */
uint16_t pwmData[(NUM_LEDS * 24) + RESET_SLOTS];

/* 各LEDの色データを保持する配列 (Green, Red, Blue) */
uint8_t LED_Data[NUM_LEDS];


/**
 * @brief すべてのLEDの色データを「赤」にセットする（この時点ではまだ光りません）
 */


static void appendByte(uint8_t value,uint32_t *index)
{
    for(int i=7;i>=0;i--)
    {
        if(value&(1<<i))
        {
            pwmData[(*index)++] = LED_HIGH;
        }
        else
        {
            pwmData[(*index)++] = LED_LOW;
        }
    }
}


void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if(index >= LED_NUM)
        return;

    ledBuffer[index].r = r;
    ledBuffer[index].g = g;
    ledBuffer[index].b = b;
}
void show(void)
{
    uint32_t index = 0;

    for(uint16_t led=0; led<LED_NUM; led++)
    {
        // GRBの順番
        appendByte(ledBuffer[led].g, &index);
        appendByte(ledBuffer[led].r, &index);
        appendByte(ledBuffer[led].b, &index);
    }

    // Reset
    while(index < LED_NUM*24 + RESET_SLOTS)
    {
        pwmData[index++] = 0;
    }

    HAL_TIM_PWM_Start_DMA(
        &htim3,
        TIM_CHANNEL_1,
        (uint32_t *)pwmData,
        index);
}


void clear(void)
{
    for(uint16_t i=0;i<LED_NUM;i++)
    {
        ledBuffer[i].r = 0;
        ledBuffer[i].g = 0;
        ledBuffer[i].b = 0;
    }
}







//void LED_SetAllZero(void) {
//}
//
//void LED_SetAllRed(void) {
//    for (int i = 0; i < NUM_LEDS; i++) {
//        // SK6812等のNeoPixel系は、Green -> Red -> Blue の順(GRB)でデータを送ります
//        LED_Data[i][0] = 0;     // Green
//        LED_Data[i][1] = 0;   // Red
//        LED_Data[i][2] = 255;     // Blue
//    }
//}
//
///**
// * @brief 色データをPWMのDuty値に変換し、DMAを使って一気に送信する
// */
//void LED_Update(void) {
//    uint32_t indx = 0;
//
//    // 各LEDの色データをビット単位に分解し、PWMのパルス幅(0か1か)に変換
//    for (int i = 0; i < NUM_LEDS; i++) {
//        // 1. Green
//        for (int j = 7; j >= 0; j--) {
//            pwmData[indx++] = (LED_Data[i][0] & (1 << j)) ? PWM_HIGH_PULSE : PWM_LOW_PULSE;
//        }
//        // 2. Red
//        for (int j = 7; j >= 0; j--) {
//            pwmData[indx++] = (LED_Data[i][1] & (1 << j)) ? PWM_HIGH_PULSE : PWM_LOW_PULSE;
//        }
//        // 3. Blue
//        for (int j = 7; j >= 0; j--) {
//            pwmData[indx++] = (LED_Data[i][2] & (1 << j)) ? PWM_HIGH_PULSE : PWM_LOW_PULSE;
//        }
//    }
//
//    // 最後にリセットコード（Treset）として、Duty 0 のパルスを連続して送信
//    for (int i = 0; i < 50; i++) {
//        pwmData[indx++] = 0;
//    }
//
//    // TIM15 Channel 1 を使ってDMA転送を開始
//    HAL_TIM_PWM_Start_DMA(&htim15, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
//}
//
//*

/**
 * @brief DMAの転送完了コールバック
 * 転送が終わったら自動的にPWM出力を停止して次の送信に備える
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
    }
}
