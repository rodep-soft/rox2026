#include "limit_switch.h"

static uint32_t next_switch_time_ms;


static uint8_t read_switch_bits(void)
{
    uint8_t bits = 0U;

    /* Inputs are active-low: bit=1 means that the switch is pressed. */
    if (HAL_GPIO_ReadPin(LIMIT_SW1_GPIO_Port, LIMIT_SW1_Pin) == GPIO_PIN_RESET) {
        bits |= (1U << 0);
    }
    if (HAL_GPIO_ReadPin(LIMIT_SW2_GPIO_Port, LIMIT_SW2_Pin) == GPIO_PIN_RESET) {
        bits |= (1U << 1);
    }
    if (HAL_GPIO_ReadPin(LIMIT_SW3_GPIO_Port, LIMIT_SW3_Pin) == GPIO_PIN_RESET) {
        bits |= (1U << 2);
    }
    return bits;
}

void LimitSwitch_Init(uint32_t now_ms)
{
    next_switch_time_ms = now_ms + LIMIT_SWITCH_PHASE_MS;
}

bool LimitSwitch_Task(uint32_t now_ms)
{
    bool ok = true;

    if ((int32_t)(now_ms - next_switch_time_ms) >= 0) {
        uint8_t switch_bits = read_switch_bits();
        next_switch_time_ms = now_ms + LIMIT_SWITCH_PERIOD_MS;
        if (CAN_SendLimitSwitch(switch_bits) == CAN_SEND_ERROR) {
            ok = false;
        }
    }

    return ok;
}
