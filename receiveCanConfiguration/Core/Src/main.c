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
#include <stdbool.h>

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

PID_Controller upper_belt_pid = {0.02f, 0.1f, 0.00f, 0.0f, 0.0f, 0.0f, 500.0f};
PID_Controller under_belt_pid = {0.04f, 0.1f, 0.00f, 0.0f, 0.0f, 0.0f, 500.0f};

// --- エンコーダとRPM計算用の変数 ---
#define ENCODER_PPR 2048
#define DT_SEC 0.01f // PID制御の周期 (10ms = 0.01秒)
// 割り算をコンパイル時に終わらせる高速化マクロ
#define RPM_CALC_FACTOR (60.0f / DT_SEC / ((float)ENCODER_PPR * 4.0f))

volatile int is_called_PID = 0;

// --- CANから受け取る指令値 ---


volatile uint8_t emergency_stop_flag = 0; // 遠隔非常停止フラグ (1で停止)
volatile int received_LED_cmd;

// --- CAN通信用の変数 ---
volatile uint16_t received_rpm_1_2;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
volatile uint8_t DataReadyFlag = 0;
#define CAN_TIMEOUT_MS 500
volatile uint32_t last_can_rx_time = 0;
volatile uint8_t is_timeout = 1;

volatile uint32_t debug_last_id = 0;       // 最後に受信したID
volatile uint8_t  debug_last_data[8] = {0};// 最後に受信したデータ
volatile uint32_t debug_last_dlc = 0;      // 最後に受信したデータ長 (0〜8)
volatile uint32_t debug_rx_count = 0;



int16_t  prev_pos1 = 0;
int16_t  prev_pos2 = 0;
volatile float upper_belt_target_rpm = 0;
volatile float under_belt_target_rpm = 0;
volatile float dribble_target_rpm = 0;
float upper_belt_target_pwm = 1000;
float under_belt_target_pwm = 1000;
float dribble_target_pwm    = 1000;
uint32_t upper_belt_current_rpm;
uint32_t under_belt_current_rpm;

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
//map関数
int16_t map(int16_t x,int16_t in_min, int16_t in_max,int16_t out_min, int16_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static inline float clampf(float value, float min, float max)
{
    if (value < min) {return min;}
    if (value > max) {return max;}
    return value;
}
// RPM計算関数
float Calc_RPM(int16_t  now, int16_t  *prev_val)
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
	is_called_PID = 1;
    float error = target - current;
    pid->integral += error * dt;

    // 積分項の発散防止
    float max_integral = 500.0f;
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;

    // 微分項の計算
    float derivative = (error - pid->prev_error) * (1.0f / dt);
    pid->prev_error = error;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 出力制限
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    if (target <= 0.0f) {
        output = pid->out_min;
        pid->integral = 0.0f;
    }

    return output;
}

void shining_LED(int received_LED_cmd) {
	 static unsigned int count_led = 0;
	 int i_count;
	 int phase;
	 int step_val;

	 switch (received_LED_cmd) {
	 	 case 1:/*虹色グラデーション*/
	 		 for(int i = 0; i < 30; i++) {
	 			 i_count = count_led + (10 * i);
	 			 i_count %= 256 * 6;
	 			 phase = i_count / 256;
	 			 step_val = i_count % 256;
	 			 switch (phase) {
		 	     	 case 0: r = 255;            g = step_val;       b = 0;              break;
		 	     	 case 1: r = 255 - step_val; g = 255;            b = 0;              break;
		 	     	 case 2: r = 0;              g = 255;            b = step_val;       break;
		 			 case 3: r = 0;              g = 255 - step_val; b = 255;            break;
		 			 case 4: r = step_val;       g = 0;              b = 255;            break;
		 			 case 5: r = 255;            g = 0;              b = 255 - step_val; break;
	 			 }
		    	 setPixel(i, r, g, b);
	 		 }
	 		 break;

	 	 case 2:/*緊急停止用真っ赤*/
	 		 for(int i = 0; i < 30; i++) {
	 			 r = 255; g = 0; b = 0;
	 			 setPixel(i, r, g, b);
	 		 }
	 		 break;

	 	 case 3:/*異常事態用赤点滅*/
	 		 for(int i = 0; i < 30; i++) {
	 			 if((count_led % 20) < 10) {
	 				 r = 0; g = 0; b = 0;
	 			 } else {
	 				 r = 255; g = 0; b = 0;
	 			 }
	 			 setPixel(i, r, g, b);
	 		 }
	 		 break;

	 	 case 4:/*上下のグラデーション*/
	 		int r = 0, g = 0, b = 0;
	 		int step_time = count_led / 10;
	 		int d = step_time % 11;
	 		int color_idx = (step_time / 11) % 6;

	 		switch (color_idx) {
	 	 		case 0: r = 255; g = 0;   b = 0;   break; // R
	 	 		case 1: r = 0;   g = 255; b = 0;   break; // G
	 	 		case 2: r = 0;   g = 0;   b = 255; break; // B
	 	 		case 3: r = 170; g = 170; b = 0;   break; // RG (黄)
	 	 		case 4: r = 0;   g = 170; b = 170; break; // GB (水色)
	 	 		case 5: r = 170; g = 0;   b = 170; break; // BR (紫)
	 		}
	 		for(int i = 0; i < 30; i++) {
	 			setPixel(i, 0, 0, 0);
	 		}
	 		// --- 1〜20の縦列 (Index: 0〜9 と 19〜10) ---
	 		for(int i = 0; i < 10; i++) {
	 			if (i >= d && i <= d + 2) {
	 				setPixel(i, r, g, b);
	 				setPixel(19 - i, r, g, b);
	 			}
	 		}
	 		// --- 21〜30の横列 (Index: 20〜29) ---
	 		for(int j = 0; j < 5; j++) {
	 			if (j == d || j == d - 1) {
	 				setPixel(24 - j, r, g, b);
	 				setPixel(25 + j, r, g, b);
	 			}
	 		}
	 		break;
	 case 5:/*点滅*/
		 int wave = (count_led * 2) % 512;
		 if (wave > 255) {
			 wave = 511 - wave; // 256を超えたら折り返して減らす
		 }

		 int bright_even = wave;         // 0 -> 255 -> 0
		 int bright_odd  = 255 - wave;   // 255 -> 0 -> 255

		 for(int i = 0; i < 30; i++) {
			 int r = 0, g = 0, b = 0;

			 if (i % 2 == 0) {
				 // 偶数(Index: 0, 2, 4...)：シアン（水色）
				 r = 0;
				 g = bright_even;
				 b = bright_even;
			 } else {
				 // 奇数(Index: 1, 3, 5...)：マゼンタ（紫）
				 r = bright_odd;
				 g = 0;
				 b = bright_odd;
			 }

			 setPixel(i, r, g, b);
		 }
		 break;
	 case 6:/*RODEPというモールス信号*/
	 {
		 int seq[56] = {
				 1,0,2,2,2,0,1,0,0,0,
				 2,2,2,0,2,2,2,0,2,2,2,0,0,0,
				 2,2,2,0,1,0,1,0,0,0,
				 1,0,0,0,
				 1,0,2,2,2,0,2,2,2,0,1,0,0,0,0,0,0,0
		 };
		 int tu = count_led / 8;

		 for(int i = 0; i < 30; i++) {
			 int idx = (tu - i) % 56;
			 if (idx < 0) {
				 idx += 56;
			 }

			 int state = seq[idx];
			 int r = 0, g = 0, b = 0;

			 if (state == 1) {
				 b = 255;
			 } else if (state == 2) {
				 r = 255;
				 g = 255;
			 }

			 setPixel(i, r, g, b);
		 }
		 break;
	 }
	}
	 count_led += 1; //色が変わる速さ
     show();
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
      setPixel(i, 0, 0, 0); // 最初は消灯
  }
  show();

  // PWMスタート
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // モーター1 (MAD PWM直結)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // モーター2 (MAD RPM制御)
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // モーター2 (MAD RPM制御)
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

  // エンコーダのカウント開始
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  // CANフィルタ設定 (ID 0x100番台と0x200番台 のみ受信する設定)
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x200 << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x7F0 << 5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  sFilterConfig.FilterBank = 1;
  sFilterConfig.FilterIdHigh = (0x100 << 5);
  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }

  if(HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }


  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000.0f);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000.0f);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1000.0f);

  HAL_Delay(4000);
  last_can_rx_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t loop_start_time = HAL_GetTick();

      // ==========================================
      // 【デバッグ用】CAN通信を無視して目標値を強制設定
      // ==========================================
      //target_rpm1 = 5000.0f; // モーター1: 2000 RPM
      //target_rpm2 = 5000.0f; // モーター2: 2000 RPM
      //target_rpm3 = 1100.0f;

      // --- 1. 通信のタイムアウト＆非常停止の監視 ---
      if ((HAL_GetTick() - last_can_rx_time) > CAN_TIMEOUT_MS) {
          is_timeout = 1;
          shining_LED(3);
      }
      else{
          // --- 6. LEDの点灯更新 ---
          shining_LED(received_LED_cmd);
      }

      // タイムアウト、または非常停止フラグが立っている場合はモーターを強制停止
      if (is_timeout || emergency_stop_flag != 0) {
    	  upper_belt_target_rpm = 0;
    	  under_belt_target_rpm = 0;
    	  dribble_target_rpm = 0;
      }

      // --- 2. エンコーダから現在RPMを取得 (モーター1, 2) ---
      int16_t  pos1 = __HAL_TIM_GET_COUNTER(&htim1);
      under_belt_current_rpm = Calc_RPM(pos1, &prev_pos1);

      int16_t  pos2 = __HAL_TIM_GET_COUNTER(&htim2);
      upper_belt_current_rpm = Calc_RPM(pos2, &prev_pos2);

      // --- 3. 出力PWMの決定 ---
      // モーター1, 2 はPIDで計算,近似式の追加でフィードフォワードせぎょっぽく
      float upper_belt_cal_target = Compute_PID(&upper_belt_pid, upper_belt_target_rpm, upper_belt_current_rpm, DT_SEC);
      float under_belt_cal_target = Compute_PID(&under_belt_pid, under_belt_target_rpm, under_belt_current_rpm, DT_SEC);
//      float under_belt_cal_target =0;
//      float upper_belt_cal_target =0;
      upper_belt_target_pwm = clampf(0.1433 * (float)upper_belt_target_rpm + 1030 + upper_belt_cal_target, 1000, 2000);
      under_belt_target_pwm = clampf(0.1400 * (float)under_belt_target_rpm + 1043 + under_belt_cal_target, 1000, 2000);

      if(upper_belt_target_rpm <= 400){
    	  upper_belt_target_pwm = 1000;
      }
      if(under_belt_target_rpm <= 400){
    	  under_belt_target_pwm = 1000;
      }

      dribble_target_pwm = map(dribble_target_rpm, 0, 5000, 1000, 2000);
      // --- 4. モーターへPWM出力 ---
      //TIM3 CH1  PA6
      //TIM3 CH2  PA4
      //TIM15 CH1 PA2
      //TIN17 CH1 PA7 ←LEDにした
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)upper_belt_target_pwm);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)under_belt_target_pwm);
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)dribble_target_pwm);
      //ドリブルは赤ブラシへ変更したため消した。

      // --- 5. リミットスイッチの状態を更新＆送信 ---
      LimitSwitch_UpdateAndSend(&hcan);

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
//        debug_last_id = RxHeader.StdId;
//        debug_last_dlc = RxHeader.DLC;
//        for(uint8_t i = 0; i < RxHeader.DLC; i++) {
//            debug_last_data[i] = RxData[i];
//        }
        debug_rx_count++;
        // ------------------------------------------------------------
        // --- 指定フォーマットの解析処理（ID: 0x20n） ---
         if(RxHeader.StdId == 0x101) {
        	//空送信による通信状態管理
        	last_can_rx_time = HAL_GetTick();
        	is_timeout = 0;
        }
         else if(RxHeader.StdId == 0x201) {
         	//LEDの光り方指定
         	received_LED_cmd = (int)RxData[0];
         	emergency_stop_flag = (bool)(RxData[0] && 0b00000001);
         }
         else if(RxHeader.StdId == 0x230 && RxHeader.DLC == 2) {
        	//MADモーターのRPM指定(ベルト下側)
            upper_belt_target_rpm = (int16_t)(RxData[0] | (RxData[1] << 8));
        }
         else if(RxHeader.StdId == 0x231 && RxHeader.DLC == 2) {
        	//MADモーターのRPM指定(ベルト上側)
        	under_belt_target_rpm = (int16_t)(RxData[0] | (RxData[1] << 8));
        }
         else if(RxHeader.StdId == 0x232 && RxHeader.DLC == 2) {
        	//MADモーターのRPM指定(ドリブル)
        	dribble_target_rpm = (int16_t)(RxData[0] | (RxData[1] << 8));
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
