#ifndef LED_LITE_H
#define LED_LITE_H

#include "main.h"

/*
 * Set to 1 only after BNO055 communication is confirmed with LEDs disabled.
 * This switch prevents both LED initialization and periodic LED updates.
 */
#ifndef LED_FEATURE_ENABLED
#define LED_FEATURE_ENABLED 1U
#endif

/* Two physical LED outputs: launcher/middle on PB4, chassis on PB5. */
#define LED_OUTPUT_PB4 (1U << 0)
#define LED_OUTPUT_PB5 (1U << 1)
#ifndef LED_OUTPUT_MASK
#define LED_OUTPUT_MASK (LED_OUTPUT_PB4 | LED_OUTPUT_PB5)
#endif

/* Limit each RGB component to 25% while diagnosing supply-voltage/noise issues. */
#ifndef LED_MAX_CHANNEL_VALUE
#define LED_MAX_CHANNEL_VALUE 64U
#endif

/* Diagnostic: keep LED GPIO/clock initialization but suppress all PWM DMA. */
#ifndef LED_DMA_ENABLED
#define LED_DMA_ENABLED 1U
#endif

/* Independent SK6812/WS2812-compatible RGB strips (NEO_GRB, 800 kHz). */
#define PB4_LED_NUM 64U
#define PB5_LED_NUM 38U

/* Compatibility names retained so the existing animation code is unchanged. */
#define PA6_LED_NUM PB4_LED_NUM
#define PA7_LED_NUM PB5_LED_NUM
#define PA2_LED_NUM 9U /* maps to the original PB4[44..36], root to front */

/* Existing setPixel()/setPixelPA7() calls now address the PB5 chassis strip. */
#define LED_NUM PA7_LED_NUM

/* Debugger watch values used to verify that disabled LED code starts no DMA. */
extern volatile uint8_t led_initialized;
extern volatile uint32_t led_uninitialized_show_count;
extern volatile uint32_t led_show_count;
extern volatile uint32_t led_dma_timeout_count;
extern volatile uint32_t led_dma_disabled_show_count;

void LED_Init(void);
void setPixelPA6(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void setPixelPA7(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void setPixelPA2(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void getPixelPA6(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b);
void getPixelPA7(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b);
void getPixelPA2(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b);
void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void show(void);
void LED_WaitForIdle(void);
void clear(void);

#endif /* LED_LITE_H */
