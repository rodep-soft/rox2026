#include "LED_lite.h"
#include <stdbool.h>
#include <string.h>

/* main.c で定義されているTIM15のハンドルを外部参照 */
extern TIM_HandleTypeDef htim3;

//LEDテープ制御用ファイル
//sk6812用

//送信間の間隔保持用バッファサイズ
#define RESET_SLOTS 100

//bitの0,1を送るための時間管理のカウント
#define LED_LOW  19
#define LED_HIGH 38

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB_t;
RGB_t ledBuffer[LED_NUM];
uint16_t pwmData[(LED_NUM * 24) + RESET_SLOTS];
bool ledChanged =true;

//DMAが複数呼ばれたときに，競合しないように制御する変数
volatile bool dmaBusy = false;


void appendByte(uint8_t value,uint32_t *index)
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
    // 既存値と比較
    if(ledBuffer[index].r == r && ledBuffer[index].g == g && ledBuffer[index].b == b)
    {//阿多愛に変化がない場合は何もせずに終了
        return;
    }else{
    ledBuffer[index].r = r;
    ledBuffer[index].g = g;
    ledBuffer[index].b = b;
    ledChanged=true;
    }
}

void show(void)
{
	if(!ledChanged)
		return;
	if(dmaBusy)
	    return;
	dmaBusy = true;

	 memset(pwmData, 0, sizeof(pwmData));

    uint32_t pwmIndex = 0;

    for(uint16_t led=0; led<LED_NUM; led++)
    {
        // GRBの順番
        appendByte(ledBuffer[led].g, &pwmIndex);
        appendByte(ledBuffer[led].r, &pwmIndex);
        appendByte(ledBuffer[led].b, &pwmIndex);
    }
    // Reset
    while(pwmIndex < LED_NUM*24 + RESET_SLOTS)
    {
        pwmData[pwmIndex++] = 0;
    }

    HAL_TIM_PWM_Start_DMA(
        &htim3,
        TIM_CHANNEL_1,
        (uint32_t *)pwmData,
		pwmIndex);
    ledChanged=false;
}

//LEDの色管理用配列の初期化
void clear(void)
{
    for(uint16_t i=0;i<LED_NUM;i++)
    {
        ledBuffer[i].r = 0;
        ledBuffer[i].g = 0;
        ledBuffer[i].b = 0;
    }
}

/**
 * @brief DMAの転送完了コールバック
 * 転送が終わったら自動的にPWM出力を停止して次の送信に備える
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        dmaBusy = false;
    }
}
