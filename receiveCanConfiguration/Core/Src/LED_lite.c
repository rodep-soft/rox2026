#include "LED_lite.h"
#include <stdbool.h>
#include <string.h>

/*
 * SK6812/WS2812-compatible RGB strips, NEO_GRB at 800 kHz:
 *   PA6 / TIM3_CH1  / DMA1 Channel 6: 29 pixels
 *   PA7 / TIM17_CH1 / DMA1 Channel 1: 38 pixels
 *   PA2 / TIM15_CH1 / DMA1 Channel 5: 9 pixels
 */

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB_t;

/* All three timer clocks are 64 MHz: 80 ticks = 1.25 us. */
#define WS_PERIOD_TICKS 80U
#define WS_T0H_TICKS    26U
#define WS_T1H_TICKS    51U
#define WS_RESET_SLOTS  80U

#define PA6_PWM_COUNT (WS_RESET_SLOTS + PA6_LED_NUM * 24U + WS_RESET_SLOTS)
#define PA7_PWM_COUNT (WS_RESET_SLOTS + PA7_LED_NUM * 24U + WS_RESET_SLOTS)
#define PA2_PWM_COUNT (WS_RESET_SLOTS + PA2_LED_NUM * 24U + WS_RESET_SLOTS)

static RGB_t pa6Buffer[PA6_LED_NUM];
static RGB_t pa7Buffer[PA7_LED_NUM];
static RGB_t pa2Buffer[PA2_LED_NUM];

static uint16_t pa6PwmData[PA6_PWM_COUNT];
static uint16_t pa7PwmData[PA7_PWM_COUNT];
static uint16_t pa2PwmData[PA2_PWM_COUNT];

static bool ledChanged = true;
static bool ledDmaBusy = false;
static uint32_t ledDmaStartMs = 0U;

/* Debugger watch values: show count must increase and timeout must stay zero. */
volatile uint32_t led_show_count = 0U;
volatile uint32_t led_dma_timeout_count = 0U;

static void LED_SetBufferPixel(RGB_t *buffer, uint16_t count,
                               uint16_t index,
                               uint8_t r, uint8_t g, uint8_t b) {
    if (index >= count) {
        return;
    }
    if ((buffer[index].r == r) &&
        (buffer[index].g == g) &&
        (buffer[index].b == b)) {
        return;
    }

    buffer[index].r = r;
    buffer[index].g = g;
    buffer[index].b = b;
    ledChanged = true;
}

static void LED_AppendByte(uint16_t *pwmData, uint32_t *index,
                           uint8_t value) {
    for (int32_t bit = 7; bit >= 0; bit--) {
        pwmData[(*index)++] = ((value & (1U << bit)) != 0U)
                ? WS_T1H_TICKS : WS_T0H_TICKS;
    }
}

static void LED_BuildPwmData(const RGB_t *buffer, uint16_t pixelCount,
                             uint16_t *pwmData, uint32_t pwmCount) {
    memset(pwmData, 0, pwmCount * sizeof(uint16_t));
    uint32_t index = WS_RESET_SLOTS;

    for (uint16_t pixel = 0; pixel < pixelCount; pixel++) {
        /* NEO_GRB: green, red, blue. */
        LED_AppendByte(pwmData, &index, buffer[pixel].g);
        LED_AppendByte(pwmData, &index, buffer[pixel].r);
        LED_AppendByte(pwmData, &index, buffer[pixel].b);
    }
}

static void LED_ConfigurePins(void) {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = GPIO_PIN_6;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_7;
    gpio.Alternate = GPIO_AF1_TIM17;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_2;
    gpio.Alternate = GPIO_AF9_TIM15;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void LED_ConfigureTimer(TIM_TypeDef *timer) {
    timer->CR1 &= ~TIM_CR1_CEN;
    timer->DIER &= ~(TIM_DIER_CC1DE | TIM_DIER_UDE);
    timer->PSC = 0U;
    timer->ARR = WS_PERIOD_TICKS - 1U;
    timer->CCR1 = 0U;

    timer->CCMR1 = (timer->CCMR1 &
            ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M)) |
            TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    timer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    timer->CCER |= TIM_CCER_CC1E;
    timer->CR1 |= TIM_CR1_ARPE;

    if ((timer == TIM15) || (timer == TIM17)) {
        timer->BDTR |= TIM_BDTR_MOE;
    }

    timer->EGR = TIM_EGR_UG;
    timer->SR = 0U;
    timer->CNT = 0U;
}

static void LED_ConfigureDma(DMA_Channel_TypeDef *channel,
                             TIM_TypeDef *timer,
                             uint16_t *pwmData,
                             uint32_t count) {
    channel->CCR &= ~DMA_CCR_EN;
    channel->CNDTR = count;
    channel->CPAR = (uint32_t)&timer->CCR1;
    channel->CMAR = (uint32_t)pwmData;
    channel->CCR = DMA_CCR_DIR | DMA_CCR_MINC |
            DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_1;
}

static void LED_StopTransfer(void) {
    TIM3->DIER &= ~TIM_DIER_CC1DE;
    TIM15->DIER &= ~TIM_DIER_CC1DE;
    TIM17->DIER &= ~TIM_DIER_CC1DE;

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM15->CR1 &= ~TIM_CR1_CEN;
    TIM17->CR1 &= ~TIM_CR1_CEN;

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel6->CCR &= ~DMA_CCR_EN;

    TIM3->CCR1 = 0U;
    TIM15->CCR1 = 0U;
    TIM17->CCR1 = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM15->EGR = TIM_EGR_UG;
    TIM17->EGR = TIM_EGR_UG;

    ledDmaBusy = false;
}

static void LED_StartAllDma(void) {
    LED_ConfigureTimer(TIM3);
    LED_ConfigureTimer(TIM15);
    LED_ConfigureTimer(TIM17);

    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF5 | DMA_IFCR_CGIF6;
    LED_ConfigureDma(DMA1_Channel6, TIM3, pa6PwmData, PA6_PWM_COUNT);
    LED_ConfigureDma(DMA1_Channel1, TIM17, pa7PwmData, PA7_PWM_COUNT);
    LED_ConfigureDma(DMA1_Channel5, TIM15, pa2PwmData, PA2_PWM_COUNT);

    DMA1_Channel6->CCR |= DMA_CCR_EN;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    TIM3->DIER |= TIM_DIER_CC1DE;
    TIM15->DIER |= TIM_DIER_CC1DE;
    TIM17->DIER |= TIM_DIER_CC1DE;

    TIM3->CNT = 0U;
    TIM15->CNT = 0U;
    TIM17->CNT = 0U;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM15->CR1 |= TIM_CR1_CEN;
    TIM17->CR1 |= TIM_CR1_CEN;

    ledDmaStartMs = HAL_GetTick();
    ledDmaBusy = true;
}

/* Check transfer state without waiting; the main loop must never block here. */
static bool LED_ServiceDma(void) {
    if (!ledDmaBusy) {
        return true;
    }

    if ((DMA1_Channel6->CNDTR == 0U) &&
        (DMA1_Channel1->CNDTR == 0U) &&
        (DMA1_Channel5->CNDTR == 0U)) {
        LED_StopTransfer();
        led_show_count++;
        return true;
    }

    if ((uint32_t)(HAL_GetTick() - ledDmaStartMs) > 10U) {
        LED_StopTransfer();
        led_dma_timeout_count++;
        ledChanged = true;
        return true;
    }

    return false;
}

void LED_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    LED_ConfigurePins();
    clear();
    show();
}

void setPixelPA6(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    LED_SetBufferPixel(pa6Buffer, PA6_LED_NUM, index, r, g, b);
}

void getPixelPA6(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b) {
    if ((index >= PA6_LED_NUM) || (r == NULL) || (g == NULL) || (b == NULL)) {
        return;
    }
    *r = pa6Buffer[index].r;
    *g = pa6Buffer[index].g;
    *b = pa6Buffer[index].b;
}

void setPixelPA7(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    LED_SetBufferPixel(pa7Buffer, PA7_LED_NUM, index, r, g, b);
}

void getPixelPA7(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b) {
    if ((index >= PA7_LED_NUM) || (r == NULL) || (g == NULL) || (b == NULL)) {
        return;
    }
    *r = pa7Buffer[index].r;
    *g = pa7Buffer[index].g;
    *b = pa7Buffer[index].b;
}

void setPixelPA2(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    LED_SetBufferPixel(pa2Buffer, PA2_LED_NUM, index, r, g, b);
}

void getPixelPA2(uint16_t index, uint8_t *r, uint8_t *g, uint8_t *b) {
    if ((index >= PA2_LED_NUM) || (r == NULL) || (g == NULL) || (b == NULL)) {
        return;
    }
    *r = pa2Buffer[index].r;
    *g = pa2Buffer[index].g;
    *b = pa2Buffer[index].b;
}

void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    setPixelPA7(index, r, g, b);
}

void clear(void) {
    memset(pa6Buffer, 0, sizeof(pa6Buffer));
    memset(pa7Buffer, 0, sizeof(pa7Buffer));
    memset(pa2Buffer, 0, sizeof(pa2Buffer));
    ledChanged = true;
}

void show(void) {
    if (!LED_ServiceDma()) {
        return;
    }

    if (!ledChanged) {
        return;
    }

    LED_BuildPwmData(pa6Buffer, PA6_LED_NUM, pa6PwmData, PA6_PWM_COUNT);
    LED_BuildPwmData(pa7Buffer, PA7_LED_NUM, pa7PwmData, PA7_PWM_COUNT);
    LED_BuildPwmData(pa2Buffer, PA2_LED_NUM, pa2PwmData, PA2_PWM_COUNT);

    ledChanged = false;
    LED_StartAllDma();
}
