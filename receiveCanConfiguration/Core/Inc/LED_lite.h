#ifndef LED_LITE_H
#define LED_LITE_H

#include "main.h" // HALの定義を読み込むため

/* 制御するLEDの数 */
#define NUM_LEDS 30

/* 関数プロトタイプ宣言 */
void LED_SetAllRed(void);
void LED_Update(void);

#endif /* LED_LITE_H */
