/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (CAN + PID Integration + 3 Motors + LED)
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
PID_Controller pid2 = {0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 2000.0f};

// --- エンコーダとRPM計算用の変数 ---
#define ENCODER_PPR 2048
#define DT_SEC 0.01f // PID制御の周期 (10ms = 0.01秒)
// 割り算をコンパイル時に終わらせる高速化マクロ
#define RPM_CALC_FACTOR (60.0f / DT_SEC / ((float)ENCODER_PPR * 4.0f))

uint16_t prev_pos1 = 0;
uint16_t prev_pos2 = 0;
float current_rpm1 = 0.0f;
float current_rpm2 = 0.0f;

// --- CANから受け取る指令値 ---
volatile float target_rpm1 = 0.0f;  // モーター1用 (RPM)
volatile float target_rpm2 = 0.0f;  // モーター2用 (RPM)
volatile float target_rpm3 = 1000.0f; // モーター3用 (PWM値: 1000〜2000)

volatile uint8_t emergency_stop_flag = 0; // 遠隔非常停止フラグ (1で停止)
volatile uint8_t led_r = 0;               // LED 赤色値 (0-255)
volatile uint8_t led_g = 0;               // LED 緑色値 (0-255)
volatile uint8_t led_b = 0;               // LED 青色値 (0-255)

// --- CAN通信用の変数 ---
CAN_RxHeaderTypeDef RxHeader;
volatile uint8_t RxData[8];
volatile uint8_t DataReadyFlag = 0;
#define CAN_TIMEOUT_MS 500
volatile uint32_t last_can_rx_time = 0;
volatile uint8_t is_timeout = 1;

volatile uint32_t debug_last_id = 0;       // 最後に受信したID
volatile uint8_t  debug_last_data[8] = {0};// 最後に受信したデータ
volatile uint32_t debug_last_dlc = 0;      // 最後に受信したデータ長 (0〜8)
volatile uint32_t debug_rx_count = 0;
float output_pwm1=1000;
float output_pwm2=1000;
float output_pwm3 =1000;
uint32_t pos1;
uint32_t pos2;
uint8_t r;
uint8_t g;
uint8_t b;
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

// RPM計算関数
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

// PID計算関数
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

    if (target <= 0.0f) {
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // LEDの初期化
  clear();
  for(int i = 0; i < 30; i++) {
      setPixel(i, 255, 0, 0); // 最初は消灯
  }
  show();

  // PWMスタート
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // モーター3 (MAD PWM直結)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // モーター2 (MAD RPM制御)
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // モーター1 (MAD RPM制御)
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // モーター3 (MAD PWM直結)

  // エンコーダのカウント開始
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // CANフィルタ設定 (ID 0x201 のみ受信する設定)
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x201 << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
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
      uint32_t loop_start_time = HAL_GetTick();

      // --- 1. 通信のタイムアウト＆非常停止の監視 ---
      if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
          is_timeout = 1;
      }

      // タイムアウト、または非常停止フラグが立っている場合はモーターを強制停止
      if (is_timeout || emergency_stop_flag != 0) {
          target_rpm1 = 0.0f;
          target_rpm2 = 0.0f;
          target_rpm3 = 1000.0f; // ESC停止のPWM値
      }

      // --- 2. エンコーダから現在RPMを取得 (モーター1, 2) ---
      pos1 = __HAL_TIM_GET_COUNTER(&htim1);
      current_rpm1 = Calc_RPM(pos1, &prev_pos1);

      pos2 = __HAL_TIM_GET_COUNTER(&htim2);
      current_rpm2 = Calc_RPM(pos2, &prev_pos2);

      // --- 3. 出力PWMの決定 ---
      // モーター1, 2 はPIDで計算
      output_pwm1 = Compute_PID(&pid1, target_rpm1, current_rpm1, DT_SEC);
      output_pwm2 = Compute_PID(&pid2, target_rpm2, current_rpm2, DT_SEC);
      // モーター3 は直接PWM値を使用 (1000〜2000の制限をかける)
      output_pwm3 = target_rpm3;
      if (output_pwm3 > 2000.0f) output_pwm3 = 2000.0f;
      if (output_pwm3 < 1000.0f) output_pwm3 = 1000.0f;

      // --- 4. モーターへPWM出力 ---
      //TIM3 CH1  PA6
      //TIM3 CH2  PA4
      //TIM15 CH1 PA2
      //TIN17 CH1 PA7 ←LEDにした
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)output_pwm2);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)output_pwm3);
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)output_pwm1);

      // --- 5. リミットスイッチの状態を更新＆送信 ---
      LimitSwitch_UpdateAndSend(&hcan);

      // --- 6. LEDの点灯更新 ---
      // 受信したRGB値を使ってLEDの色を更新する
      clear();
      for(int i = 0; i < 30; i++) {
          setPixel(i, r, g, b); // 最初は消灯
      }
      show();

      // --- 7. PID周期を安定させるための待機 (DT_SEC=10ms) ---
      // 処理にかかった時間を差し引いて正確に10msループを作る
      while ((HAL_GetTick() - loop_start_time) < 10) {
          // 待機
      }

      (void)debug_rx_count;

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // CANの受信バッファ(FIFO0)からデータを読み出す
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

        // --- デバッグ用：受信した全メッセージを無条件で記録 ---
        debug_last_id = RxHeader.StdId;
        debug_last_dlc = RxHeader.DLC;
        for(uint8_t i = 0; i < RxHeader.DLC; i++) {
            debug_last_data[i] = RxData[i];
        }
        debug_rx_count++;
        // -------------------------------------------------------------

        // --- 指定フォーマットの解析処理（ID: 0x201） ---
        // 8バイトのデータが来ていることを前提として中身を取り出す
        if(RxHeader.StdId == 0x201 && RxHeader.DLC == 8) {

            // Byte [0],[1]: モーター1,2用 RPM
            uint16_t received_rpm_1_2 = (uint16_t)(RxData[0] | (RxData[1] << 8));
            target_rpm1 = (float)received_rpm_1_2;
            target_rpm2 = (float)received_rpm_1_2;

            // Byte [2],[3]: モーター3用 PWM (1000-2000)
            uint16_t received_pwm_3 = (uint16_t)(RxData[2] | (RxData[3] << 8));
            target_rpm3 = (float)received_pwm_3;

            // Byte [4]: 遠隔非常停止フラグ (1:停止, 0:通常)
            emergency_stop_flag = RxData[4];

            // Byte [5],[6],[7]: LEDの RGB値
            led_r = RxData[5];
            led_g = RxData[6];
            led_b = RxData[7];

            // 通信成功の証としてタイムアウト時間をリセット
            last_can_rx_time = HAL_GetTick();
            is_timeout = 0;
            DataReadyFlag = 1;
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
