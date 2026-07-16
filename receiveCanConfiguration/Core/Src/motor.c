/*
 * motor.c
 *
 *  Created on: 2026/07/17
 *      Author: yamato
 */

#include "motor.h"
#include "tim.h"
#include "can.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

// --- PID制御用の構造体と変数 ---
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float prev_error;
	float integral;
	float out_min; // PWM最小値
	float out_max; // PWM最大値
} PID_Controller;

static PID_Controller pid1 = { 0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 2000.0f };
static PID_Controller pid2 = { 0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 2000.0f };

// --- エンコーダとRPM計算用の変数 ---
#define ENCODER_PPR 2048U
#define DT_SEC 0.01f // PID制御の周期 (10ms = 0.01秒)
// 割り算をコンパイル時に終わらせる高速化マクロ
#define RPM_CALC_FACTOR (60.0f / DT_SEC / ((float)ENCODER_PPR * 4.0f))
#define PWM_MIN 1000.0f
#define PWM_MAX 2000.0f
#define PID_RESPONSE_QUEUE_CAPACITY 16U

typedef struct {
	uint8_t motor_index;
	uint8_t parameter;
	uint8_t success;
} PID_Response;

volatile float target_rpm1 = 0.0f;
volatile float target_rpm2 = 0.0f;
volatile float target_rpm3 = PWM_MIN;

float current_rpm1 = 0.0f;
float current_rpm2 = 0.0f;
float output_pwm1 = PWM_MIN;
float output_pwm2 = PWM_MIN;
float output_pwm3 = PWM_MIN;

static uint16_t prev_pos1 = 0U;
static uint16_t prev_pos2 = 0U;
static uint8_t feedback_index = 0U;
static PID_Response pid_response_queue[PID_RESPONSE_QUEUE_CAPACITY];
static volatile uint8_t pid_response_read_index = 0U;
static volatile uint8_t pid_response_write_index = 0U;
static volatile uint32_t pid_response_overflow_count = 0U;
static volatile bool motor_safety_stop = true;

static int16_t ReadInt16LittleEndian(const uint8_t data[]) {
	return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
}

static void Motor1RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	if (!motor_safety_stop && (rxHeader->DLC >= 2U)) {
		target_rpm1 = (float) ReadInt16LittleEndian(rxData);
	}
}

static void Motor2RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	if (!motor_safety_stop && (rxHeader->DLC >= 2U)) {
		target_rpm2 = (float) ReadInt16LittleEndian(rxData);
	}
}

static void Motor3RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	if (!motor_safety_stop && (rxHeader->DLC >= 1U)) {
		uint8_t output_percent = rxData[0];
		if (output_percent > 100U) {
			output_percent = 100U;
		}
		target_rpm3 = PWM_MIN + ((float) output_percent * 10.0f);
	}
}

static void QueuePidResponse(uint8_t motor_index, uint8_t parameter,
		bool success) {
	uint8_t write_index = pid_response_write_index;
	uint8_t next_index = (uint8_t) ((write_index + 1U)
			% PID_RESPONSE_QUEUE_CAPACITY);

	if (next_index == pid_response_read_index) {
		pid_response_overflow_count++;
		return;
	}

	pid_response_queue[write_index].motor_index = motor_index;
	pid_response_queue[write_index].parameter = parameter;
	pid_response_queue[write_index].success = success ? 1U : 0U;
	__DMB();
	pid_response_write_index = next_index;
}

static void SetPidParameter(PID_Controller *pid, uint8_t motor_index,
		CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	float value;
	uint8_t parameter = 0xFFU;
	bool success = false;

	if (rxHeader->DLC >= 1U) {
		parameter = rxData[0];
	}

	if ((rxHeader->DLC >= 5U) && (parameter <= 2U)) {
		memcpy(&value, &rxData[1], sizeof(value));
		if (isfinite(value)) {
			switch (parameter) {
			case 0U:
				pid->Kp = value;
				break;
			case 1U:
				pid->Ki = value;
				break;
			default:
				pid->Kd = value;
				break;
			}
			success = true;
		}
	}

	QueuePidResponse(motor_index, parameter, success);
}

static void Motor1PidRxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	SetPidParameter(&pid1, 0U, rxHeader, rxData);
}

static void Motor2PidRxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	SetPidParameter(&pid2, 1U, rxHeader, rxData);
}

static int16_t ClampRpmForCan(float rpm) {
	if (rpm > 32767.0f) {
		return INT16_MAX;
	}
	if (rpm < -32768.0f) {
		return INT16_MIN;
	}
	return (int16_t) rpm;
}

static bool MotorSendPidFeedback(void) {
	CAN_TxHeaderTypeDef txHeader = { 0 };
	uint8_t txData[2];
	uint32_t txMailbox;
	uint8_t read_index = pid_response_read_index;
	PID_Response response;

	if (read_index == pid_response_write_index) {
		return false;
	}

	response = pid_response_queue[read_index];

	txHeader.StdId = (response.motor_index == 0U) ? 0x340U : 0x341U;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 2U;
	txData[0] = response.parameter;
	txData[1] = response.success;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) == HAL_OK) {
		__DMB();
		pid_response_read_index = (uint8_t) ((read_index + 1U)
				% PID_RESPONSE_QUEUE_CAPACITY);
	}

	return true;
}

static void MotorSendFeedback(void) {
	CAN_TxHeaderTypeDef txHeader = { 0 };
	uint8_t txData[2];
	uint32_t txMailbox;
	uint16_t value;

	if (MotorSendPidFeedback()) {
		return;
	}

	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;

	switch (feedback_index) {
	case 0U:
		txHeader.StdId = 0x330U;
		txHeader.DLC = 2U;
		value = (uint16_t) ClampRpmForCan(current_rpm1);
		txData[0] = (uint8_t) (value & 0xFFU);
		txData[1] = (uint8_t) (value >> 8);
		break;
	case 1U:
		txHeader.StdId = 0x331U;
		txHeader.DLC = 2U;
		value = (uint16_t) ClampRpmForCan(current_rpm2);
		txData[0] = (uint8_t) (value & 0xFFU);
		txData[1] = (uint8_t) (value >> 8);
		break;
	default:
		txHeader.StdId = 0x332U;
		txHeader.DLC = 1U;
		txData[0] = (uint8_t) ((output_pwm3 - PWM_MIN) / 10.0f);
		break;
	}

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) == HAL_OK) {
		feedback_index = (uint8_t) ((feedback_index + 1U) % 3U);
	}
}

static float Calc_RPM(uint16_t now, uint16_t *prev_val) {
	int16_t diff = (int16_t) (now - *prev_val);
	*prev_val = now;

	// 高速化のため割り算を使わず係数を掛ける
	return (float) diff * RPM_CALC_FACTOR;
}

static float Compute_PID(PID_Controller *pid, float target, float current,
		float dt) {
	float error = target - current;
	pid->integral += error * dt;

	// 積分項の発散防止
	float max_integral = 500.0f;
	if (pid->integral > max_integral)
		pid->integral = max_integral;
	if (pid->integral < -max_integral)
		pid->integral = -max_integral;

	// 微分項の計算
	float derivative = (error - pid->prev_error) * (1.0f / dt);
	pid->prev_error = error;

	float output = 1000.0f + (pid->Kp * error) + (pid->Ki * pid->integral)
			+ (pid->Kd * derivative);

	// 出力制限
	if (output > pid->out_max)
		output = pid->out_max;
	if (output < pid->out_min)
		output = pid->out_min;

	if (target <= 0.0f) {
		output = pid->out_min;
		pid->integral = 0.0f;
	}

	return output;
}

HAL_StatusTypeDef motor_init(void) {
	if (canAddRxCallback(0x230U, Motor1RxCallback) != HAL_OK) {
		return HAL_ERROR;
	}
	if (canAddRxCallback(0x231U, Motor2RxCallback) != HAL_OK) {
		return HAL_ERROR;
	}
	if (canAddRxCallback(0x232U, Motor3RxCallback) != HAL_OK) {
		return HAL_ERROR;
	}
	if (canAddRxCallback(0x240U, Motor1PidRxCallback) != HAL_OK) {
		return HAL_ERROR;
	}
	if (canAddRxCallback(0x241U, Motor2PidRxCallback) != HAL_OK) {
		return HAL_ERROR;
	}

	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)PWM_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)PWM_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)PWM_MIN);

	if (HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1) != HAL_OK) {
		return HAL_ERROR;
	}
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
		return HAL_ERROR;
	}
	if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
		return HAL_ERROR;
	}
	if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) {
		return HAL_ERROR;
	}
	if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
		return HAL_ERROR;
	}

	prev_pos1 = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
	prev_pos2 = (uint16_t) __HAL_TIM_GET_COUNTER(&htim2);
	pid1.prev_error = 0.0f;
	pid1.integral = 0.0f;
	pid2.prev_error = 0.0f;
	pid2.integral = 0.0f;

	return HAL_OK;
}

void motor_set_safety_stop(bool stop) {
	if (stop) {
		motor_safety_stop = true;
		target_rpm1 = 0.0f;
		target_rpm2 = 0.0f;
		target_rpm3 = PWM_MIN;
		pid1.integral = 0.0f;
		pid1.prev_error = 0.0f;
		pid2.integral = 0.0f;
		pid2.prev_error = 0.0f;
	} else {
		motor_safety_stop = false;
	}
}

void motor_loop(void) {
	uint16_t pos1 = (uint16_t) __HAL_TIM_GET_COUNTER(&htim1);
	uint16_t pos2 = (uint16_t) __HAL_TIM_GET_COUNTER(&htim2);
	float target1 = target_rpm1;
	float target2 = target_rpm2;
	float target3 = target_rpm3;
	bool safety_stop = motor_safety_stop;

	if (safety_stop) {
		target1 = 0.0f;
		target2 = 0.0f;
		target3 = PWM_MIN;
	}

	current_rpm1 = Calc_RPM(pos1, &prev_pos1);
	current_rpm2 = Calc_RPM(pos2, &prev_pos2);

	// --- 3. 出力PWMの決定 ---
	// モーター1, 2 はPIDで計算
	output_pwm1 = Compute_PID(&pid1, target1, current_rpm1, DT_SEC);
	output_pwm2 = Compute_PID(&pid2, target2, current_rpm2, DT_SEC);
	// モーター3 は直接PWM値を使用 (1000〜2000の制限をかける)
	output_pwm3 = target3;
	if (output_pwm3 > PWM_MAX)
		output_pwm3 = PWM_MAX;
	if (output_pwm3 < PWM_MIN)
		output_pwm3 = PWM_MIN;

	// --- 4. モーターへPWM出力 ---
	//TIM3 CH1  PA6
	//TIM3 CH2  PA4
	//TIM15 CH1 PA2
	//TIM17 CH1 PA7 はLED用
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t )output_pwm2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t )output_pwm3);
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t )output_pwm1);

	MotorSendFeedback();
}
