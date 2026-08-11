#ifndef INC_LIMIT_SWITCH_H_
#define INC_LIMIT_SWITCH_H_

#include "can.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIMIT_SWITCH_PERIOD_MS       20U
#define LIMIT_SWITCH_PHASE_MS        20U

/* Call once after CAN startup/restart. */
void LimitSwitch_Init(uint32_t now_ms);

/*
 * Non-blocking periodic task.
 * - Reads and sends the two limit switches every 10 ms.
 * Returns false only when HAL reports a CAN transmit error. A full mailbox is
 * treated as a temporary busy condition and retried on the next main-loop pass.
 */
bool LimitSwitch_Task(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_LIMIT_SWITCH_H_ */
