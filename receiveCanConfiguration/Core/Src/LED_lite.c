#include "LED_lite.h"
#include "tim.h"
#include <stdbool.h>
#include <string.h>

/* PB4/TIM16/DMA1 Channel3 and PB5/TIM17/DMA1 Channel1 only. */
typedef struct { uint8_t r, g, b; } RGB_t;

#define WS_PERIOD_TICKS 80U
#define WS_T0H_TICKS 26U
#define WS_T1H_TICKS 51U
#define WS_RESET_SLOTS 80U
#define PB4_PWM_COUNT (WS_RESET_SLOTS + PB4_LED_NUM * 24U + WS_RESET_SLOTS)
#define PB5_PWM_COUNT (WS_RESET_SLOTS + PB5_LED_NUM * 24U + WS_RESET_SLOTS)
#define LED_MIN_FRAME_INTERVAL_MS 20U

static RGB_t pb4Buffer[PB4_LED_NUM];
static RGB_t pb5Buffer[PB5_LED_NUM];
static uint16_t pb4PwmData[PB4_PWM_COUNT];
static uint16_t pb5PwmData[PB5_PWM_COUNT];
static bool ledChanged = true;
static volatile bool ledDmaBusy = false;
static volatile bool pb4DmaDone = true;
static volatile bool pb5DmaDone = true;
static uint32_t ledDmaStartMs;
static uint32_t lastLedFrameMs;

volatile uint8_t led_initialized;
volatile uint32_t led_uninitialized_show_count;
volatile uint32_t led_show_count;
volatile uint32_t led_dma_timeout_count;
volatile uint32_t led_dma_disabled_show_count;

static uint8_t limitBrightness(uint8_t v) {
    return (uint8_t)(((uint16_t)v * LED_MAX_CHANNEL_VALUE + 127U) / 255U);
}

static void setBuffer(RGB_t *p, uint16_t count, uint16_t i,
                      uint8_t r, uint8_t g, uint8_t b) {
    if (i >= count) return;
    if (p[i].r == r && p[i].g == g && p[i].b == b) return;
    p[i].r = r; p[i].g = g; p[i].b = b;
    ledChanged = true;
}

static void appendByte(uint16_t *pwm, uint32_t *i, uint8_t v) {
    for (int32_t bit = 7; bit >= 0; --bit)
        pwm[(*i)++] = (v & (1U << bit)) ? WS_T1H_TICKS : WS_T0H_TICKS;
}

static void buildData(const RGB_t *rgb, uint16_t count,
                      uint16_t *pwm, uint32_t pwmCount) {
    memset(pwm, 0, pwmCount * sizeof(*pwm));
    uint32_t i = WS_RESET_SLOTS;
    for (uint16_t n = 0; n < count; ++n) {
        appendByte(pwm, &i, limitBrightness(rgb[n].g));
        appendByte(pwm, &i, limitBrightness(rgb[n].r));
        appendByte(pwm, &i, limitBrightness(rgb[n].b));
    }
}

static void configureTimer(TIM_TypeDef *tim) {
    tim->CR1 &= ~TIM_CR1_CEN;
    tim->DIER &= ~(TIM_DIER_CC1DE | TIM_DIER_UDE);
    tim->PSC = 0U;
    tim->ARR = WS_PERIOD_TICKS - 1U;
    tim->CCR1 = 0U;
    tim->CCMR1 = (tim->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M))
               | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    tim->CCER |= TIM_CCER_CC1E;
    tim->CR1 |= TIM_CR1_ARPE;
    tim->BDTR |= TIM_BDTR_MOE;
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0U;
    tim->CNT = 0U;
}

static void stopTransfer(void) {
    (void)HAL_TIM_PWM_Stop_DMA(&htim16, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop_DMA(&htim17, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0U);
    pb4DmaDone = true;
    pb5DmaDone = true;
    ledDmaBusy = false;
}

static bool startTransfer(void) {
    configureTimer(TIM16);
    configureTimer(TIM17);
    pb4DmaDone = false;
    pb5DmaDone = false;

    if (HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1,
                              (uint32_t *)pb4PwmData,
                              PB4_PWM_COUNT) != HAL_OK) {
        stopTransfer();
        return false;
    }
    if (HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1,
                              (uint32_t *)pb5PwmData,
                              PB5_PWM_COUNT) != HAL_OK) {
        stopTransfer();
        return false;
    }

    ledDmaStartMs = HAL_GetTick();
    ledDmaBusy = true;
    return true;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM16) {
        (void)HAL_TIM_PWM_Stop_DMA(&htim16, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0U);
        pb4DmaDone = true;
    } else if (htim->Instance == TIM17) {
        (void)HAL_TIM_PWM_Stop_DMA(&htim17, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0U);
        pb5DmaDone = true;
    }
}
static bool serviceDma(void) {
    if (!ledDmaBusy) return true;
    if (pb4DmaDone && pb5DmaDone) {
        ledDmaBusy = false;
        ++led_show_count;
        return true;
    }
    if ((uint32_t)(HAL_GetTick() - ledDmaStartMs) > 10U) {
        stopTransfer();
        ++led_dma_timeout_count;
        ledChanged = true;
        return true;
    }
    return false;
}

void LED_WaitForIdle(void) {
    if (!led_initialized) return;
    while (!serviceDma()) HAL_Delay(1U);
}

void LED_Init(void) {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = GPIO_PIN_4;
    gpio.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_5;
    gpio.Alternate = GPIO_AF10_TIM17;
    HAL_GPIO_Init(GPIOB, &gpio);

    led_initialized = 1U;
#if LED_DMA_ENABLED
    clear();
    show();
#endif
}

/* Existing launcher calls now drive the physical PB4 strip. */
void setPixelPA6(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    setBuffer(pb4Buffer, PB4_LED_NUM, i, r, g, b);
}
void getPixelPA6(uint16_t i, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (i >= PB4_LED_NUM || !r || !g || !b) return;
    *r = pb4Buffer[i].r; *g = pb4Buffer[i].g; *b = pb4Buffer[i].b;
}
void setPixelPA7(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    setBuffer(pb5Buffer, PB5_LED_NUM, i, r, g, b);
}
void getPixelPA7(uint16_t i, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (i >= PB5_LED_NUM || !r || !g || !b) return;
    *r = pb5Buffer[i].r; *g = pb5Buffer[i].g; *b = pb5Buffer[i].b;
}
/* Former PA2 left-middle pixels now continue on PB4 at physical 44..36. */
void setPixelPA2(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    if (i >= PA2_LED_NUM) return;
    setBuffer(pb4Buffer, PB4_LED_NUM,
              (uint16_t)(PB4_LED_NUM - 1U - i), r, g, b);
}
void getPixelPA2(uint16_t i, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (i >= PA2_LED_NUM || !r || !g || !b) return;
    const uint16_t physical = (uint16_t)(PB4_LED_NUM - 1U - i);
    *r = pb4Buffer[physical].r;
    *g = pb4Buffer[physical].g;
    *b = pb4Buffer[physical].b;
}
void setPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    setPixelPA7(i, r, g, b);
}
void clear(void) {
    memset(pb4Buffer, 0, sizeof(pb4Buffer));
    memset(pb5Buffer, 0, sizeof(pb5Buffer));
    ledChanged = true;
}
void show(void) {
    if (!led_initialized) { ++led_uninitialized_show_count; return; }
#if !LED_DMA_ENABLED
    ++led_dma_disabled_show_count;
    return;
#endif
    if (!serviceDma() || !ledChanged) return;
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastLedFrameMs) < LED_MIN_FRAME_INTERVAL_MS) return;
    buildData(pb4Buffer, PB4_LED_NUM, pb4PwmData, PB4_PWM_COUNT);
    buildData(pb5Buffer, PB5_LED_NUM, pb5PwmData, PB5_PWM_COUNT);
    ledChanged = false;
    if (!startTransfer()) {
        ++led_dma_timeout_count;
        ledChanged = true;
    }
}
