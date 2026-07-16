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
#define RX_CALLBACK_CAPACITY 64U
#define HEARTBEAT_TIMEOUT_MS 500U

static RxCallback rxCallbacks[RX_CALLBACK_CAPACITY];
static uint16_t rxCanIds[RX_CALLBACK_CAPACITY];
static uint8_t rxCallbackCount = 0U;
static volatile uint32_t heartbeatLastRxTime = 0U;
static volatile uint8_t heartbeatValue = 0U;
static volatile bool heartbeatReceived = false;
static volatile bool heartbeatResponsePending = false;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 2;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (canHandle->Instance == CAN) {
		/* USER CODE BEGIN CAN_MspInit 0 */

		/* USER CODE END CAN_MspInit 0 */
		/* CAN clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN GPIO Configuration
		 PA11     ------> CAN_RX
		 PA12     ------> CAN_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
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

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {

	if (canHandle->Instance == CAN) {
		/* USER CODE BEGIN CAN_MspDeInit 0 */

		/* USER CODE END CAN_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN GPIO Configuration
		 PA11     ------> CAN_RX
		 PA12     ------> CAN_TX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

		/* CAN interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN_TX_IRQn);
		HAL_NVIC_DisableIRQ(CAN_RX0_IRQn);
		/* USER CODE BEGIN CAN_MspDeInit 1 */

		/* USER CODE END CAN_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef canAddRxCallback(uint16_t canId, RxCallback rxCallback) {
	if ((rxCallback == NULL) || (rxCallbackCount >= RX_CALLBACK_CAPACITY)) {
		return HAL_ERROR;
	}

	rxCanIds[rxCallbackCount] = canId;
	rxCallbacks[rxCallbackCount] = rxCallback;
	rxCallbackCount++;

	return HAL_OK;
}

static void HeartbeatRxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	if (rxHeader->DLC < 1U) {
		return;
	}

	heartbeatValue = rxData[0];
	heartbeatLastRxTime = HAL_GetTick();
	heartbeatReceived = true;
	__DMB();
	heartbeatResponsePending = true;
}

HAL_StatusTypeDef CAN_HeartbeatInit(void) {
	return canAddRxCallback(0x100U, HeartbeatRxCallback);
}

bool CAN_HeartbeatIsTimedOut(void) {
	if (!heartbeatReceived) {
		return true;
	}

	return (HAL_GetTick() - heartbeatLastRxTime) > HEARTBEAT_TIMEOUT_MS;
}

void CAN_HeartbeatProcess(void) {
	CAN_TxHeaderTypeDef txHeader = { 0 };
	uint8_t txData[1];
	uint32_t txMailbox;
	uint32_t primask;
	uint8_t value;

	primask = __get_PRIMASK();
	__disable_irq();
	if (!heartbeatResponsePending) {
		if (primask == 0U) {
			__enable_irq();
		}
		return;
	}
	value = heartbeatValue;
	heartbeatResponsePending = false;
	if (primask == 0U) {
		__enable_irq();
	}

	txHeader.StdId = 0x101U;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 1U;
	txData[0] = value;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK) {
		primask = __get_PRIMASK();
		__disable_irq();
		if (!heartbeatResponsePending) {
			heartbeatValue = value;
			heartbeatResponsePending = true;
		}
		if (primask == 0U) {
			__enable_irq();
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// CANの受信バッファ(FIFO0)からデータを読み出す
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
			break;
		}
		if (RxHeader.IDE != CAN_ID_STD) {
			continue;
		}
		// --- callbackの受信処理 ---
		for (uint8_t i = 0U; i < rxCallbackCount; i++) {
			if (rxCanIds[i] == RxHeader.StdId) {
				rxCallbacks[i](&RxHeader, RxData);
			}
		}
	}
}
/* USER CODE END 1 */
