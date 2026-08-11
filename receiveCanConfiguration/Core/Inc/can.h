/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* Application CAN protocol (11-bit standard IDs). */
#define CAN_ID_STM32_HEARTBEAT  0x100U
#define CAN_ID_LIMIT_SWITCH     0x310U
#define CAN_ID_BNO055_QUATERNION          0x320U
#define CAN_ID_BNO055_ANGULAR_VELOCITY    0x321U
#define CAN_ID_BNO055_LINEAR_ACCELERATION 0x322U
#define CAN_HEARTBEAT_PERIOD_MS  100U
#define CAN_HEARTBEAT_PHASE_MS    50U
#define CAN_TX_STALL_TIMEOUT_MS  100U

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

typedef enum {
  CAN_SEND_OK = 0,
  CAN_SEND_BUSY,
  CAN_SEND_ERROR
} CAN_SendResult;

typedef struct {
  uint32_t sent_count;
  uint32_t busy_count;
  uint32_t error_count;
  uint32_t stall_recovery_count;
  HAL_StatusTypeDef last_hal_status;
  uint32_t last_can_error;
} CAN_TxDiagnostics;

/* All application CAN transmission is encoded through these functions. */
CAN_SendResult CAN_SendHeartbeat(void);
CAN_SendResult CAN_SendLimitSwitch(uint8_t switch_bits);
CAN_SendResult CAN_SendBno055Quaternion(int16_t x, int16_t y,
                                        int16_t z, int16_t w);
CAN_SendResult CAN_SendBno055AngularVelocity(int16_t x, int16_t y, int16_t z);
CAN_SendResult CAN_SendBno055LinearAcceleration(int16_t x, int16_t y, int16_t z);
CAN_SendResult CAN_HeartbeatTask(uint32_t now_ms);
/* Returns false after aborting mailboxes that stayed full for 100 ms. */
bool CAN_TxHealthTask(uint32_t now_ms);
void CAN_ResetTxDiagnostics(void);
const volatile CAN_TxDiagnostics *CAN_GetTxDiagnostics(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

