#ifndef INC_LED_EFFECTS_H_
#define INC_LED_EFFECTS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CAN 0x201 data[0]: primary LED display mode. */
#define LED_MODE_STARTUP              0U
#define LED_MODE_READY                1U
#define LED_MODE_EMERGENCY_STOP       2U
#define LED_MODE_ARM_OPEN             3U
#define LED_MODE_LOADING              4U
#define LED_MODE_FIRING               5U
#define LED_MODE_RETURNING            6U
#define LED_MODE_GAME2_SEARCHING      7U
#define LED_MODE_GAME2_ALIGNING       8U
#define LED_MODE_ERROR                9U
#define LED_MODE_ARM_DRIBBLE         10U
#define LED_MODE_SLOW_FIRING         11U
#define LED_MODE_ARM_FEED            12U
#define LED_MODE_ARM_RECEIVE         13U
#define LED_MODE_ARM_HOME            14U
#define LED_MODE_BELT_SPINUP         15U
#define LED_MODE_BELT_OFFSET_MINUS_3 16U
#define LED_MODE_BELT_OFFSET_MINUS_2 17U
#define LED_MODE_BELT_OFFSET_MINUS_1 18U
#define LED_MODE_BELT_OFFSET_ZERO    19U
#define LED_MODE_BELT_OFFSET_PLUS_1  20U
#define LED_MODE_BELT_OFFSET_PLUS_2  21U
#define LED_MODE_BELT_OFFSET_PLUS_3  22U

void LED_Effects_Init(void);
void LED_Effects_Render(uint8_t mode, uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* INC_LED_EFFECTS_H_ */
