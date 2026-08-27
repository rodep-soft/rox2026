/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (CAN + BNO055 + Limit Switch + LED)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "limit_switch.h"
#include "LED_lite.h"
#include "LED_effects.h"
#include "bno055_hal.h"
#include "bno055_calibration.h"
//#include "bno055_uart.h"
#include <stdbool.h>

// --- CANから受け取る指令値 ---
volatile uint8_t emergency_stop_flag = 0; // 遠隔非常停止フラグ (1で停止)
volatile int received_LED_cmd;
volatile int received_LED_status;

// --- CAN通信用の変数 ---
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
volatile uint8_t DataReadyFlag = 0;
#define CAN_TIMEOUT_MS 500
volatile uint32_t last_can_rx_time = 0;
volatile uint8_t is_timeout = 1;
volatile int32_t debug_can_rx_elapsed_ms = 0;

volatile uint32_t debug_last_id = 0;       // 最後に受信したID
volatile uint8_t debug_last_data[8] = { 0 };       // 最後に受信したデータ
volatile uint32_t debug_last_dlc = 0;      // 最後に受信したデータ長 (0〜8)
volatile uint32_t debug_rx_count = 0;


#define LOOP_TIME                   10U

#define BNO055_ACTIVE_MODE BNO055_MODE_IMUPLUS
#define BNO055_RETRY_INTERVAL_MS  1000U
#define BNO055_MAX_READ_ERRORS    3U
#define BNO055_CALIBRATION_CHECK_PERIOD_MS 1000U
#define BNO055_CALIBRATION_STABLE_SAMPLES 5U
#define CAN_RETRY_INTERVAL_MS       1000U
#define CAN_MAX_TX_ERRORS           10U

volatile bool can_connected = false;
volatile uint32_t last_can_retry_time = 0;
uint32_t can_tx_error_count = 0;
uint32_t can_init_error_count = 0;
uint32_t can_restart_count = 0;

bool bno055_connected = false;
uint32_t last_bno055_retry_time = 0;
uint32_t bno055_init_error_count = 0;
uint32_t bno055_read_error_count = 0;
uint32_t bno055_tx_count = 0;
uint8_t bno055_sample_phase = 0U;
volatile BNO055_Status debug_bno_init_status = BNO055_ERROR;
volatile BNO055_Status debug_bno_read_status = BNO055_ERROR;
volatile uint16_t debug_bno_i2c_address = 0U;
volatile uint32_t debug_bno_i2c_error = HAL_I2C_ERROR_NONE;
uint32_t last_bno055_calibration_check_time = 0;
uint32_t bno055_calibration_save_error_count = 0;
bool bno055_calibration_profile_saved = false;
bool bno055_calibration_profile_restored = false;
uint8_t bno055_calibration_stable_count = 0U;

BNO055_Handle bno055;
BNO055_Quaternion imu_quat;
BNO055_Vector3Int16 imu_angular_velocity;
BNO055_Vector3Int16 imu_linear_acceleration;
BNO055_Euler imu_euler;
BNO055_Calibration imu_calib;

uint32_t last_loop_time;
uint32_t last_bno_com_time;
uint32_t bno_com_priod;

volatile uint32_t debug_invalid_quat_count = 0;
volatile int16_t debug_bad_qx = 0;
volatile int16_t debug_bad_qy = 0;
volatile int16_t debug_bad_qz = 0;
volatile int16_t debug_bad_qw = 0;
volatile uint32_t debug_bad_norm_sq = 0;

volatile uint8_t debug_page_id = 0;
volatile uint8_t debug_system_status = 0;
//volatile uint8_t debug_self_test = 0;
volatile uint8_t debug_system_error = 0;

volatile uint8_t debug_operation_mode  = 0xFFU;
volatile uint8_t debug_sys_status = 0xFFU;
volatile uint8_t debug_self_test = 0xFFU;
volatile uint8_t debug_sys_error = 0xFFU;

volatile int16_t debug_gyro_x = 0;
volatile int16_t debug_gyro_y = 0;
volatile int16_t debug_gyro_z = 0;
volatile BNO055_Status debug_gyro_status = BNO055_ERROR;
volatile int16_t debug_accel_x = 0;
volatile int16_t debug_accel_y = 0;
volatile int16_t debug_accel_z = 0;
volatile BNO055_Status debug_accel_status = BNO055_ERROR;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Recover an I2C bus held busy by a slave: release SDA, pulse SCL up to
 * nine times, generate a STOP condition, then restore I2C1 alternate pins.
 */
static void BNO055_RecoverI2CBus(void) {
	GPIO_InitTypeDef gpio = { 0 };

	(void)HAL_I2C_DeInit(&hi2c1);
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(2U);

	for (uint32_t pulse = 0U;
			(pulse < 9U) &&
			(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
			++pulse) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1U);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(1U);
	}

	/* STOP: SDA low -> SCL high -> SDA high. */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1U);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1U);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(2U);

	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
	MX_I2C1_Init();
}
//BNO055の初期化関数
static bool BNO055_TryInitialize(void) {
	BNO055_Status status;
	uint8_t calibration_profile[BNO055_CALIBRATION_PROFILE_SIZE];

	/*
	 * IMUPLUS uses the accelerometer and gyroscope without the magnetometer.
	 * This provides a gravity-referenced relative orientation and avoids magnetic
	 * disturbances from motors and the robot frame.
	 */
	status = BNO055_Init(&bno055, &hi2c1,
		BNO055_I2C_ADDR_LOW, BNO055_ACTIVE_MODE);
	if (status != BNO055_OK) {
		status = BNO055_Init(&bno055, &hi2c1,
			BNO055_I2C_ADDR_HIGH, BNO055_ACTIVE_MODE);
	}
	debug_bno_init_status = status;
	debug_bno_i2c_address = bno055.address;
	debug_bno_i2c_error = HAL_I2C_GetError(&hi2c1);

	/* A valid Flash profile removes the need for manual calibration at boot. */
	bno055_calibration_profile_restored = false;
	bno055_calibration_profile_saved =
			BNO055_CalibrationStorageLoad(calibration_profile);
	if ((status == BNO055_OK) && bno055_calibration_profile_saved) {
		const BNO055_Status calibration_status =
			BNO055_ApplyCalibrationProfile(&bno055, calibration_profile);
		bno055_calibration_profile_restored =
			(calibration_status == BNO055_OK);
		/* Optional calibration restore must not mark the sensor disconnected. */
		if (calibration_status != BNO055_OK) {
			(void)BNO055_SetMode(&bno055, BNO055_ACTIVE_MODE);
		}
	}

	//初期化に成功したら，デバック用に各種ステータスを読み取って終了
	if (status == BNO055_OK) {
		bno055_connected = true;
		bno055_read_error_count = 0;
		HAL_Delay(10);
		(void)BNO055_ReadOperationMode(&bno055,
				(uint8_t*) &debug_operation_mode );
		HAL_Delay(2U);
		(void)BNO055_ReadSystemStatus(&bno055,
				(uint8_t*) &debug_sys_status, (uint8_t*) &debug_self_test,
				(uint8_t*) &debug_sys_error);
		return true;
	}
	// 失敗したら，再度呼ばれるまで待機
	bno055_connected = false;
	bno055_init_error_count++;

	return false;
}

static void BNO055_CalibrationTask(uint32_t now_ms) {
	uint8_t calibration_profile[BNO055_CALIBRATION_PROFILE_SIZE];

	if (!bno055_connected || bno055_calibration_profile_saved) {
		return;
	}
	if ((uint32_t)(now_ms - last_bno055_calibration_check_time) <
			BNO055_CALIBRATION_CHECK_PERIOD_MS) {
		return;
	}
	last_bno055_calibration_check_time = now_ms;

	if (BNO055_ReadCalibration(&bno055, &imu_calib) != BNO055_OK) {
		bno055_calibration_stable_count = 0U;
		return;
	}
	/* IMUPLUS does not use the magnetometer, so mag calibration is irrelevant. */
	if ((imu_calib.system != 3U) || (imu_calib.gyro != 3U) ||
		(imu_calib.accel != 3U)) {
		bno055_calibration_stable_count = 0U;
		return;
	}
	if (bno055_calibration_stable_count < BNO055_CALIBRATION_STABLE_SAMPLES) {
		bno055_calibration_stable_count++;
	}
	if (bno055_calibration_stable_count < BNO055_CALIBRATION_STABLE_SAMPLES) {
		return;
	}

	/* This path runs only once in the lifetime of the saved profile. */
	if ((BNO055_ReadCalibrationProfile(&bno055, calibration_profile) == BNO055_OK) &&
			BNO055_CalibrationStorageSave(calibration_profile)) {
		bno055_calibration_profile_saved = true;
	} else {
		bno055_calibration_save_error_count++;
	}
}

/* Reset a wedged I2C peripheral before attempting to configure the sensor. */
static bool BNO055_Recover(void) {
	BNO055_RecoverI2CBus();
	return BNO055_TryInitialize();
}
static bool CAN_TryInitialize(void) {
	CAN_FilterTypeDef filter = { 0 };
	HAL_CAN_DeactivateNotification(&hcan,
	CAN_IT_RX_FIFO0_MSG_PENDING |
	CAN_IT_BUSOFF |
	CAN_IT_ERROR);
	HAL_CAN_Stop(&hcan);
	HAL_CAN_DeInit(&hcan);
	MX_CAN_Init();

	// Filter 0
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = (0x200 << 5);
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = (0x7F0 << 5);
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
		can_connected = false;
		can_init_error_count++;
		return false;
	}

	// Filter 1
	filter.FilterBank = 1;
	filter.FilterIdHigh = (0x100 << 5);

	if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
		can_connected = false;
		can_init_error_count++;
		return false;
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		can_connected = false;
		can_init_error_count++;
		return false;
	}

	if (HAL_CAN_ActivateNotification(&hcan,
	CAN_IT_RX_FIFO0_MSG_PENDING |
	CAN_IT_BUSOFF |
	CAN_IT_ERROR) != HAL_OK) {
		can_connected = false;
		can_init_error_count++;
		return false;
	}

	can_connected = true;
	can_tx_error_count = 0;
	can_restart_count++;
	CAN_ResetTxDiagnostics();
	LimitSwitch_Init(HAL_GetTick());

	return true;
}

static bool BNO055_ReadAndSend(void) {
	BNO055_Status status = BNO055_ReadQuaternion(&bno055, &imu_quat);
	debug_bno_read_status = status;
	if (status != BNO055_OK) {
		debug_bno_i2c_error = HAL_I2C_GetError(&hi2c1);
		return false;
	}

	/* Quaternion is sampled every slot (100 Hz). Read only one vector in each
	 * slot so that I2C work and CAN traffic remain bounded: gyro and linear
	 * acceleration alternate at 50 Hz each. */
	const bool read_gyro = (bno055_sample_phase == 0U);
	if (read_gyro) {
		debug_gyro_status = BNO055_ReadGyroscope(
				&bno055, &imu_angular_velocity);
		status = debug_gyro_status;
		if (status == BNO055_OK) {
			debug_gyro_x = imu_angular_velocity.x;
			debug_gyro_y = imu_angular_velocity.y;
			debug_gyro_z = imu_angular_velocity.z;
		}
	} else {
		debug_accel_status = BNO055_ReadLinearAcceleration(
				&bno055, &imu_linear_acceleration);
		status = debug_accel_status;
		if (status == BNO055_OK) {
			debug_accel_x = imu_linear_acceleration.x;
			debug_accel_y = imu_linear_acceleration.y;
			debug_accel_z = imu_linear_acceleration.z;
		}
	}
	const bool vector_valid = (status == BNO055_OK);
	if (!vector_valid) {
		debug_bno_read_status = status;
		debug_bno_i2c_error = HAL_I2C_GetError(&hi2c1);
	}

	debug_bno_i2c_error = HAL_I2C_GetError(&hi2c1);

	const CAN_SendResult quat_result = CAN_SendBno055Quaternion(
			imu_quat.x, imu_quat.y, imu_quat.z, imu_quat.w);
	/* A valid quaternion is sent even when the alternating vector read failed.
	 * This keeps the 100 Hz attitude stream continuous while the I2C recovery
	 * logic handles the failed gyro/acceleration transaction. */
	const CAN_SendResult vector_result = vector_valid && read_gyro
			? CAN_SendBno055AngularVelocity(
					imu_angular_velocity.x, imu_angular_velocity.y,
					imu_angular_velocity.z)
			: vector_valid
			? CAN_SendBno055LinearAcceleration(
					imu_linear_acceleration.x, imu_linear_acceleration.y,
					imu_linear_acceleration.z)
			: CAN_SEND_OK;

	if ((quat_result == CAN_SEND_ERROR) ||
			(vector_result == CAN_SEND_ERROR)) {
		can_tx_error_count++;
	} else if ((quat_result == CAN_SEND_OK) &&
			(vector_result == CAN_SEND_OK)) {
		bno055_sample_phase ^= 1U;
		if (vector_valid) {
			bno055_read_error_count = 0U;
		}
		can_tx_error_count = 0U;
		bno055_tx_count++;
		const uint32_t tx_now = HAL_GetTick();
		bno_com_priod = tx_now - last_bno_com_time;
		last_bno_com_time = tx_now;
	}
	return vector_valid;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	BNO055_RecoverI2CBus();
	HAL_Delay(1500);
	//BNO055初期化
	bno055_connected = BNO055_TryInitialize();
	last_bno055_retry_time = HAL_GetTick();

	can_connected = CAN_TryInitialize();
	last_can_retry_time = HAL_GetTick();

	//駆動電源の出力をON,OFFするピン
	//HIGH出力で，電源入る，LOW出力でOFFに
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// LEDの初期化
#if LED_FEATURE_ENABLED
	LED_Effects_Init();
#endif
	last_can_rx_time = HAL_GetTick();
	last_loop_time = HAL_GetTick();
	last_bno_com_time = last_loop_time;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//ループ周期調整
		uint32_t now = HAL_GetTick();
		if ((uint32_t)(now - last_loop_time) < LOOP_TIME) {
			HAL_Delay(1U);
			continue;
		}
		last_loop_time = now;

		/* Stop the previous LED DMA/timers before any BNO055 I2C access. */
		LED_WaitForIdle();

		// CANが切れていないかの確認
		if (!can_connected) {
			if ((uint32_t) (now - last_can_retry_time) >= CAN_RETRY_INTERVAL_MS) {
				last_can_retry_time = now;
				CAN_TryInitialize();
			}
		}
		if (can_tx_error_count >= CAN_MAX_TX_ERRORS) {
			can_connected = false;
			can_tx_error_count = 0;
			last_can_retry_time = now;
		}
		if (can_connected && !CAN_TxHealthTask(now)) {
			/* Mailboxes stayed full: abort stale frames and restart immediately. */
			can_connected = false;
			can_tx_error_count = 0;
			last_can_retry_time = now;
			(void)CAN_TryInitialize();
		}
		if (!bno055_connected) {
			if ((uint32_t) (now - last_bno055_retry_time)
					>= BNO055_RETRY_INTERVAL_MS) {
				last_bno055_retry_time = now;
				bno055_connected = BNO055_Recover();
			}
			// errorの許容値を超えたら再起動を試みる
		}
		if (bno055_read_error_count >= BNO055_MAX_READ_ERRORS) {
			bno055_connected = false;
			bno055_read_error_count = 0;
			last_bno055_retry_time = now;
		}

		BNO055_CalibrationTask(now);

		/* ---- Deterministic 10 ms CAN slot ----
		 * LimitSW/heartbeat are due on different slots and consume at most one
		 * mailbox. Quaternion plus one vector consume the remaining two. */
		if (can_connected) {
			if (CAN_HeartbeatTask(now) == CAN_SEND_ERROR) {
				can_tx_error_count++;
			}
			if (!LimitSwitch_Task(now)) {
				can_tx_error_count++;
			}
			if (bno055_connected && !BNO055_ReadAndSend()) {
				bno055_read_error_count++;
			}
		}
#if LED_FEATURE_ENABLED
			/* Separate the final BNO055 I2C edge from the WS2812 DMA start. */
			if (bno055_connected) {
				HAL_Delay(2U);
			}
			/* HAL_GetTick() must be sampled here, not at the beginning of the
			 * loop. A CAN IRQ may make last_can_rx_time newer than the old loop
			 * timestamp; signed subtraction treats that race as "not timed out"
			 * instead of wrapping to a huge uint32_t value. */
			const uint32_t timeout_now = HAL_GetTick();
			debug_can_rx_elapsed_ms =
					(int32_t)(timeout_now - last_can_rx_time);
			if (debug_can_rx_elapsed_ms > (int32_t)CAN_TIMEOUT_MS) {
				is_timeout = 1U;
				LED_Effects_Render(LED_MODE_ERROR, (uint8_t)received_LED_status);
			} else {
				// --- 6. LEDの点灯更新 ---
				LED_Effects_Render((uint8_t)received_LED_cmd, (uint8_t)received_LED_status);
			}
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// CANの受信バッファ(FIFO0)からデータを読み出す
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

		// --- デバッグ用：受信した全メッセージを無条件で記録 ---
//        debug_last_id = RxHeader.StdId;
//        debug_last_dlc = RxHeader.DLC;
//        for(uint8_t i = 0; i < RxHeader.DLC; i++) {
//            debug_last_data[i] = RxData[i];
//        }
		debug_rx_count++;
		/* Any successfully received CAN frame proves that the bus is alive.
		 * Do not flash the LEDs merely because heartbeat ID 0x101 was delayed. */
		last_can_rx_time = HAL_GetTick();
		is_timeout = 0U;
		// ------------------------------------------------------------
		// --- 指定フォーマットの解析処理（ID: 0x20n） ---
		if (RxHeader.StdId == 0x101) {
			//空送信による通信状態管理
			last_can_rx_time = HAL_GetTick();
			is_timeout = 0;
		} else if ((RxHeader.StdId == 0x201U) && (RxHeader.DLC == 2U)) {
			//LEDの光り方指定
			received_LED_cmd = (int) RxData[0];
			received_LED_status = RxData[1];
			//emergency_stop_flag = (bool)(RxData[0] && 0b00000001);
		}
	}
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	uint32_t err = HAL_CAN_GetError(hcan);
	if ((err & HAL_CAN_ERROR_BOF) != 0U) {
		can_connected = false;
		last_can_retry_time = HAL_GetTick();
	} else if (err != HAL_CAN_ERROR_NONE) {
		can_tx_error_count++;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
