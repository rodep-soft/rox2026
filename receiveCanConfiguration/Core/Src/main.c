/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (CAN + PID Integration)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "limit_switch.h"

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

PID_Controller pid1 = {0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 2000.0f};

// --- エンコーダとRPM計算用の変数 ---
#define ENCODER_PPR 2048
#define DT_SEC 0.01f // PID制御の周期 (10ms = 0.01秒)
// 割り算をコンパイル時に終わらせる高速化マクロ
#define RPM_CALC_FACTOR (60.0f / DT_SEC / ((float)ENCODER_PPR * 4.0f))

uint16_t prev_pos = 0;
float current_rpm = 0.0f;
float target_rpm = 0.0f;  // CANから受け取る目標RPM

// --- CAN通信用の変数 ---
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint8_t DataReadyFlag = 0;
#define CAN_TIMEOUT_MS 500
uint32_t last_can_rx_time = 0;
uint8_t is_timeout = 1;

uint32_t debug_last_id = 0;       // 最後に受信したID
uint8_t  debug_last_data[8] = {0};// 最後に受信したデータ
uint32_t debug_last_dlc = 0;      // 最後に受信したデータ長 (0〜8)
uint32_t debug_rx_count = 0;
uint32_t aa=0;
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

// RPM計算関数 (inline展開で高速化)
float Calc_RPM(uint16_t now, uint16_t *prev_val)
{
    int16_t diff = now - *prev_val;
    *prev_val = now;

    // オーバーフロー補正
    if (diff > 32767)  diff -= 65536;
    if (diff < -32768) diff += 65536;

    // 高速化のため割り算を使わず係数を掛ける
    return (float)diff * RPM_CALC_FACTOR;
}

// PID計算関数 (inline展開で高速化)
float Compute_PID(PID_Controller *pid, float target, float current, float dt)
{
    float error = target - current;
    pid->integral += error * dt;

    // 積分項の発散防止
    float max_integral = 500.0f;
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;

    // 微分項の計算
    float derivative = (error - pid->prev_error) * (1.0f / dt);
    pid->prev_error = error;

    float output = 1000.0f + (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 出力制限
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    if (target <= 1000.0f) {
        output = pid->out_min;
        pid->integral = 0.0f;
    }

    return output;
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // エンコーダのカウント開始 (htim4を使用すると仮定)
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  if(HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. CAN通信のタイムアウト監視
      if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
          is_timeout = 1;
          target_rpm = 0.0f; // タイムアウト時は目標RPMを0にして安全停止
      }

      // 2. エンコーダから現在RPMを取得
      uint16_t pos = __HAL_TIM_GET_COUNTER(&htim1);
      current_rpm = Calc_RPM(pos, &prev_pos);

      // 3. PIDを計算してPWM出力値を決定
      float output_pwm = Compute_PID(&pid1, target_rpm, current_rpm, DT_SEC);

      // 4. モーターへ出力
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)output_pwm);

      // 5. リミットスイッチの状態を更新＆送信
      LimitSwitch_UpdateAndSend(&hcan);

      // 6. PID周期を安定させるために10ms待機 (DT_SECと一致させること)
      HAL_Delay(10);
aa++;
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

        if(RxHeader.StdId == 0x202) {
            if(RxData[0] == 2) {
                // 変更点: RxData[1]と[2]の2バイトを使って、0〜65535のRPMを受信する
                // リトルエンディアン(下位バイトが先)で送られてくると仮定
                uint16_t received_rpm = (uint16_t)(RxData[1] | (RxData[2] << 8));
                target_rpm = (float)received_rpm;

                last_can_rx_time = HAL_GetTick();
                is_timeout = 0;
                DataReadyFlag = 1;
            }
        }
    }
}

*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // CANの受信バッファ(FIFO0)からデータを読み出す
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

        // --- 【追加】デバッグ用：受信した全メッセージを無条件で記録 ---
        debug_last_id = RxHeader.StdId;       // IDを記録
        debug_last_dlc = RxHeader.DLC;        // データ長を記録
        for(uint8_t i = 0; i < RxHeader.DLC; i++) {
            debug_last_data[i] = RxData[i];   // データをコピー
        }
        debug_rx_count++;                     // カウンターを増やす
        // -------------------------------------------------------------

        // --- 従来の処理（ID: 0x202 専用の処理） ---
        if(RxHeader.StdId == 0x202) {
            if(RxData[0] == 2) {
                // RxData[1]と[2]の2バイトを使って、0〜65535のRPMを受信する
                uint16_t received_rpm = (uint16_t)(RxData[1] | (RxData[2] << 8));
                target_rpm = (float)received_rpm;

                last_can_rx_time = HAL_GetTick();
                is_timeout = 0;
                DataReadyFlag = 1;
            }
        }
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
  while (1)
  {
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
