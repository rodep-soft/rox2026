#ifndef LED_LITE_H
#define LED_LITE_H

#include "main.h" // HALの定義を読み込むため

/* 制御するLEDの数 */
#define LED_NUM 30

/* 関数プロトタイプ宣言 */
void appendByte(uint8_t value,uint32_t *index);
void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void show(void);
void clear(void);

#endif /* LED_LITE_H */
