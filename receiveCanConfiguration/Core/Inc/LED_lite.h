#ifndef LED_LITE_H
#define LED_LITE_H

#include "main.h"

/* Independent SK6812/WS2812-compatible RGB strips (NEO_GRB, 800 kHz). */
#define PA6_LED_NUM 29U
#define PA7_LED_NUM 38U
#define PA2_LED_NUM  9U

/* Existing setPixel() calls address the PA7 chassis strip. */
#define LED_NUM PA7_LED_NUM

void LED_Init(void);
void setPixelPA6(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void setPixelPA7(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void setPixelPA2(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void getPixelPA6(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b);
void getPixelPA7(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b);
void getPixelPA2(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b);
void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void show(void);
void clear(void);

#endif /* LED_LITE_H */
