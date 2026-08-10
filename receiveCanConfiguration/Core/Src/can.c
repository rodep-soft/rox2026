/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

volatile static CAN_TxDiagnostics tx_diagnostics;
static uint32_t next_heartbeat_time_ms;
static uint32_t tx_busy_since_ms;
static bool tx_busy_active;

static bool CAN_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static CAN_SendResult CAN_SendFrame(uint32_t std_id,
                                    const uint8_t *data,
                                    uint32_t dlc)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox = 0U;

  if ((dlc > 8U) || ((dlc > 0U) && (data == NULL)) ||
      (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_LISTENING)) {
    tx_diagnostics.last_hal_status = HAL_ERROR;
    tx_diagnostics.error_count++;
    return CAN_SEND_ERROR;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
    if (!tx_busy_active) {
      tx_busy_active = true;
      tx_busy_since_ms = HAL_GetTick();
    }
    tx_diagnostics.busy_count++;
    return CAN_SEND_BUSY;
  }

  /* At least one mailbox became available, so this is not a persistent jam. */
  tx_busy_active = false;

  header.StdId = std_id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = dlc;
  header.TransmitGlobalTime = DISABLE;

  tx_diagnostics.last_hal_status =
      HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)data, &mailbox);
  tx_diagnostics.last_can_error = HAL_CAN_GetError(&hcan);
  if (tx_diagnostics.last_hal_status != HAL_OK) {
    tx_diagnostics.error_count++;
    return CAN_SEND_ERROR;
  }

  tx_diagnostics.sent_count++;
  return CAN_SEND_OK;
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(CAN_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN_TX_IRQn);
    HAL_NVIC_SetPriority(CAN_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

CAN_SendResult CAN_SendHeartbeat(void)
{
  return CAN_SendFrame(CAN_ID_STM32_HEARTBEAT, NULL, 0U);
}

CAN_SendResult CAN_SendLimitSwitch(uint8_t switch_bits)
{
  return CAN_SendFrame(CAN_ID_LIMIT_SWITCH, &switch_bits, 1U);
}

CAN_SendResult CAN_SendBno055Quaternion(int16_t x, int16_t y,
                                        int16_t z, int16_t w)
{
  const uint16_t values[4] = {
      (uint16_t)x, (uint16_t)y, (uint16_t)z, (uint16_t)w
  };
  uint8_t data[8];

  for (uint32_t i = 0U; i < 4U; ++i) {
    data[i * 2U] = (uint8_t)(values[i] & 0xFFU);
    data[i * 2U + 1U] = (uint8_t)(values[i] >> 8);
  }
  return CAN_SendFrame(CAN_ID_BNO055_QUATERNION, data, sizeof(data));
}

CAN_SendResult CAN_HeartbeatTask(uint32_t now_ms)
{
  if (CAN_TimeReached(now_ms, next_heartbeat_time_ms) == 0) {
    return CAN_SEND_OK;
  }

  next_heartbeat_time_ms += CAN_HEARTBEAT_PERIOD_MS;
  if (CAN_TimeReached(now_ms, next_heartbeat_time_ms) != 0) {
    next_heartbeat_time_ms = now_ms + CAN_HEARTBEAT_PERIOD_MS;
  }
  return CAN_SendHeartbeat();
}

bool CAN_TxHealthTask(uint32_t now_ms)
{
  if (!tx_busy_active) {
    return true;
  }

  if ((uint32_t)(now_ms - tx_busy_since_ms) < CAN_TX_STALL_TIMEOUT_MS) {
    return true;
  }

  /* Discard stale frames before main.c performs a complete CAN restart. */
  (void)HAL_CAN_AbortTxRequest(&hcan,
      CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  tx_diagnostics.last_can_error = HAL_CAN_GetError(&hcan);
  tx_diagnostics.stall_recovery_count++;
  tx_busy_active = false;
  return false;
}

void CAN_ResetTxDiagnostics(void)
{
  const uint32_t stall_recovery_count =
      tx_diagnostics.stall_recovery_count;
  tx_diagnostics = (CAN_TxDiagnostics){0};
  /* Keep this lifetime counter visible across CAN peripheral restarts. */
  tx_diagnostics.stall_recovery_count = stall_recovery_count;
  tx_diagnostics.last_hal_status = HAL_OK;
  next_heartbeat_time_ms = HAL_GetTick();
  tx_busy_since_ms = 0U;
  tx_busy_active = false;
}

const volatile CAN_TxDiagnostics *CAN_GetTxDiagnostics(void)
{
  return &tx_diagnostics;
}

/* USER CODE END 1 */
