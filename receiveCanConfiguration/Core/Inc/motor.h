#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

extern volatile float target_rpm1;
extern volatile float target_rpm2;
extern volatile float target_rpm3;

extern float current_rpm1;
extern float current_rpm2;
extern float output_pwm1;
extern float output_pwm2;
extern float output_pwm3;

HAL_StatusTypeDef motor_init(void);
void motor_set_safety_stop(bool stop);
void motor_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
