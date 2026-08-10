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
#include "bno055_hal.h"
#include <stdbool.h>

// --- CANから受け取る指令値 ---
volatile uint8_t emergency_stop_flag = 0; // 遠隔非常停止フラグ (1で停止)
volatile int received_LED_cmd;
volatile uint8_t received_LED_status = 0U;

// --- CAN通信用の変数 ---
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
volatile uint8_t DataReadyFlag = 0;
#define CAN_TIMEOUT_MS 500
volatile uint32_t last_can_rx_time = 0;
volatile uint8_t is_timeout = 1;

volatile uint32_t debug_last_id = 0;       // 最後に受信したID
volatile uint8_t debug_last_data[8] = { 0 };       // 最後に受信したデータ
volatile uint32_t debug_last_dlc = 0;      // 最後に受信したデータ長 (0〜8)
volatile uint32_t debug_rx_count = 0;

uint8_t r;
uint8_t g;
uint8_t b;

#define LOOP_TIME                   10U

#define BNO055_RETRY_INTERVAL_MS  1000U
#define BNO055_MAX_READ_ERRORS    3U
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
volatile BNO055_Status debug_bno_init_status = BNO055_ERROR;
volatile uint16_t debug_bno_i2c_address = 0U;
volatile uint8_t debug_bno_chip_id = 0U;
volatile uint32_t debug_bno_init_duration_ms = 0U;

BNO055_Handle bno055;
BNO055_Quaternion imu_quat;
BNO055_Euler imu_euler;
BNO055_Calibration imu_calib;

uint32_t last_loop_time;
uint32_t last_bno_com_time;
uint32_t bno_com_priod;

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
static void __attribute__((unused)) shining_LED_legacy(int received_LED_cmd) {
	static unsigned int count_led = 0;
	int i_count;
	int phase;
	int step_val;

	switch (received_LED_cmd) {
	case 1:/*虹色グラデーション*/
		for (int i = 0; i < 30; i++) {
			i_count = count_led + (10 * i);
			i_count %= 256 * 6;
			phase = i_count / 256;
			step_val = i_count % 256;
			switch (phase) {
			case 0:
				r = 255;
				g = step_val;
				b = 0;
				break;
			case 1:
				r = 255 - step_val;
				g = 255;
				b = 0;
				break;
			case 2:
				r = 0;
				g = 255;
				b = step_val;
				break;
			case 3:
				r = 0;
				g = 255 - step_val;
				b = 255;
				break;
			case 4:
				r = step_val;
				g = 0;
				b = 255;
				break;
			case 5:
				r = 255;
				g = 0;
				b = 255 - step_val;
				break;
			}
			setPixel(i, r, g, b);
		}
		break;

	case 2:/*緊急停止用真っ赤*/
		for (int i = 0; i < 30; i++) {
			r = 255;
			g = 0;
			b = 0;
			setPixel(i, r, g, b);
		}
		break;

	case 3:/*異常事態用赤点滅*/
		for (int i = 0; i < 30; i++) {
			if ((count_led % 20) < 10) {
				r = 0;
				g = 0;
				b = 0;
			} else {
				r = 255;
				g = 0;
				b = 0;
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
		case 0:
			r = 255;
			g = 0;
			b = 0;
			break; // R
		case 1:
			r = 0;
			g = 255;
			b = 0;
			break; // G
		case 2:
			r = 0;
			g = 0;
			b = 255;
			break; // B
		case 3:
			r = 170;
			g = 170;
			b = 0;
			break; // RG (黄)
		case 4:
			r = 0;
			g = 170;
			b = 170;
			break; // GB (水色)
		case 5:
			r = 170;
			g = 0;
			b = 170;
			break; // BR (紫)
		}
		for (int i = 0; i < 30; i++) {
			setPixel(i, 0, 0, 0);
		}
		// --- 1〜20の縦列 (Index: 0〜9 と 19〜10) ---
		for (int i = 0; i < 10; i++) {
			if (i >= d && i <= d + 2) {
				setPixel(i, r, g, b);
				setPixel(19 - i, r, g, b);
			}
		}
		// --- 21〜30の横列 (Index: 20〜29) ---
		for (int j = 0; j < 5; j++) {
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
		int bright_odd = 255 - wave;   // 255 -> 0 -> 255

		for (int i = 0; i < 30; i++) {
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
		int seq[56] = { 1, 0, 2, 2, 2, 0, 1, 0, 0, 0, 2, 2, 2, 0, 2, 2, 2, 0, 2,
				2, 2, 0, 0, 0, 2, 2, 2, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
				2, 2, 2, 0, 2, 2, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0 };
		int tu = count_led / 8;

		for (int i = 0; i < 30; i++) {
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

#define EMERGENCY_SWEEP_MS        600U
#define EMERGENCY_WAVE_COUNT        2U
#define EMERGENCY_BRIGHTNESS      220U
#define PA6_MAIN_LED_COUNT         20U
#define LAUNCHER_SIDE_COUNT         9U
#define LAUNCHER_SIDE_LOOP_COUNT   20U
#define LAUNCHER_LEVEL_COUNT       10U
#define LED_FRONT_CENTER           15U
#define LED_REAR_CENTER            34U
#define SHOT_OPENING_HEAD_SPEED_Q8_PER_MS 16U
#define SHOT_OPENING_FILL_SPEED_Q8_PER_MS 32U
#define DRIBBLE_SWEEP_MS         1200U
#define DRIBBLE_WAVE_COUNT          3U
#define DRIBBLE_BASE_INTENSITY     12U
#define DRIBBLE_PEAK_INTENSITY    255U
#define DRIBBLE_GLOW_RADIUS_Q8 (4 * 256)
#define FIRING_WAVE_MS             233U
#define FIRING_LAUNCHER_OUTER_RADIUS_Q8 (14 * 256)
#define FIRING_LAUNCHER_FULL_RADIUS_Q8  (10 * 256)
#define FIRING_CHASSIS_OUTER_RADIUS_Q8  (22 * 256)
#define FIRING_CHASSIS_FULL_RADIUS_Q8   (17 * 256)

/*
 * 投射全体が紫になった後、全LEDを赤に切り替えるまでの待ち時間。
 * 待ち時間を変更したい場合は、この300U（ミリ秒）だけを書き換える。
 */
#define LOADING_AFTER_PURPLE_HOLD_MS 50U

static bool belt_wave_active = false;
static int32_t belt_wave_head_q8 = -512;
static uint32_t belt_wave_last_update_ms = 0U;

static uint8_t shot_opening_phase = 0U;
static int32_t shot_opening_head_q8 = 0;
static int32_t shot_opening_fill_q8 = 0;
static uint32_t shot_opening_last_update_ms = 0U;

static uint8_t firing_pa7_background[PA7_LED_NUM][3];
static uint8_t firing_pa6_background[PA6_LED_NUM][3];
static uint8_t firing_pa2_background[PA2_LED_NUM][3];
static uint32_t firing_start_ms = 0U;

static uint8_t Emergency_GetIntensityFor(uint16_t pixel,
		uint16_t pixel_count, uint32_t now_ms, bool reverse_direction) {
	const uint16_t animation_pixel = reverse_direction
			? (uint16_t)(pixel_count - 1U - pixel) : pixel;
	const int32_t circumference_q8 = (int32_t)pixel_count * 256;
	const int32_t glow_radius_q8 = circumference_q8 / 8;
	const int32_t phase_q8 = (int32_t)((now_ms % EMERGENCY_SWEEP_MS)
			* (uint32_t)circumference_q8 / EMERGENCY_SWEEP_MS);
	int32_t nearest_wave_q8 = circumference_q8;

	for (uint8_t wave = 0U; wave < EMERGENCY_WAVE_COUNT; wave++) {
		const int32_t head_q8 = (phase_q8
				+ (int32_t)wave * circumference_q8 / EMERGENCY_WAVE_COUNT)
				% circumference_q8;
		int32_t distance_q8 = (int32_t)animation_pixel * 256 - head_q8;

		if (distance_q8 < 0) {
			distance_q8 = -distance_q8;
		}
		const int32_t wrapped_distance_q8 = circumference_q8 - distance_q8;
		if (wrapped_distance_q8 < distance_q8) {
			distance_q8 = wrapped_distance_q8;
		}
		if (distance_q8 < nearest_wave_q8) {
			nearest_wave_q8 = distance_q8;
		}
	}

	if (nearest_wave_q8 >= glow_radius_q8) {
		return 0U;
	}

	const uint32_t strength = (uint32_t)(glow_radius_q8 - nearest_wave_q8);
	const uint32_t radius_squared =
			(uint32_t)glow_radius_q8 * (uint32_t)glow_radius_q8;
	const uint32_t intensity = strength * strength * 255U / radius_squared;
	return (uint8_t)(intensity * EMERGENCY_BRIGHTNESS / 255U);
}

static uint16_t LED_CircularDistance(uint16_t a, uint16_t b) {
	uint16_t distance = (a > b) ? (a - b) : (b - a);
	uint16_t wrapped = LED_NUM - distance;
	return (wrapped < distance) ? wrapped : distance;
}

static uint8_t LED_TriangleWave(uint32_t now_ms, uint32_t period_ms) {
	uint32_t phase = now_ms % period_ms;
	uint32_t half = period_ms / 2U;
	if (phase < half) {
		return (uint8_t)(phase * 255U / half);
	}
	return (uint8_t)((period_ms - phase) * 255U / half);
}

/* Smooth 0..255..0 curve with no abrupt change at either end. */
static uint8_t LED_SmoothPulse(uint32_t now_ms, uint32_t period_ms) {
	const uint32_t x = LED_TriangleWave(now_ms, period_ms);
	return (uint8_t)(x * x * (765U - 2U * x) / (255U * 255U));
}

static uint8_t LED_GlowIntensity(uint16_t distance) {
	switch (distance) {
	case 0U:
		return 220U;
	case 1U:
		return 110U;
	case 2U:
		return 35U;
	default:
		return 0U;
	}
}

static void LED_SetAll(uint8_t red, uint8_t green, uint8_t blue) {
	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		setPixelPA7(i, red, green, blue);
	}
}

static void LED_SetLauncherAll(uint8_t red, uint8_t green, uint8_t blue) {
	for (uint16_t i = 0U; i < PA6_LED_NUM; i++) {
		setPixelPA6(i, red, green, blue);
	}
	for (uint16_t i = 0U; i < PA2_LED_NUM; i++) {
		setPixelPA2(i, red, green, blue);
	}
}

static void LED_SetLauncherLevel(uint16_t level,
		uint8_t red, uint8_t green, uint8_t blue) {
	if (level >= LAUNCHER_LEVEL_COUNT) {
		return;
	}

	setPixelPA6(level, red, green, blue);
	setPixelPA6(PA6_MAIN_LED_COUNT - 1U - level, red, green, blue);
	if (level < LAUNCHER_SIDE_COUNT) {
		setPixelPA6(PA6_MAIN_LED_COUNT + level, red, green, blue);
		setPixelPA2(level, red, green, blue);
	}
}

static void LED_RenderLauncherWave(uint32_t now_ms, uint32_t period_ms,
		bool toward_tip, uint8_t red, uint8_t green, uint8_t blue) {
	const uint32_t elapsed = now_ms % period_ms;
	uint16_t head = (uint16_t)(elapsed * (LAUNCHER_LEVEL_COUNT - 1U)
			/ period_ms);
	if (!toward_tip) {
		head = (LAUNCHER_LEVEL_COUNT - 1U) - head;
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		const uint16_t distance = (level > head)
				? (level - head) : (head - level);
		const uint8_t glow = LED_GlowIntensity(distance);
		LED_SetLauncherLevel(level,
				(uint8_t)((uint16_t)red * glow / 255U),
				(uint8_t)((uint16_t)green * glow / 255U),
				(uint8_t)((uint16_t)blue * glow / 255U));
	}
}

/* data[1] bits 0..2: belt level shown on the launcher toward its tip. */
static void LED_RenderBeltLauncher(uint8_t status, uint32_t now_ms) {
	const uint8_t belt_level = status & 0x07U;

	/* STOP and reserved values 5..7 keep the main display. */
	if ((belt_level == 0U) || (belt_level > 4U)) {
		belt_wave_active = false;
		belt_wave_last_update_ms = now_ms;
		return;
	}

	/* Center: 4/6/8/10 levels. Sides: 3/5/7/9 levels. */
	const uint16_t center_lit_count = (uint16_t)(2U + 2U * belt_level);
	const uint16_t side_lit_count = (uint16_t)(1U + 2U * belt_level);
	static const uint8_t speed_q8_per_ms[4] = { 1U, 2U, 4U, 7U };
	const int32_t radius_q8 = 640; /* 2.5 LED levels */

	if (!belt_wave_active) {
		belt_wave_active = true;
		belt_wave_head_q8 = -512;
		belt_wave_last_update_ms = now_ms;
	} else {
		const uint32_t elapsed_ms = now_ms - belt_wave_last_update_ms;
		belt_wave_last_update_ms = now_ms;
		belt_wave_head_q8 +=
				(int32_t)(elapsed_ms * speed_q8_per_ms[belt_level - 1U]);
	}

	/* A level change changes range and speed, but never resets the wave position. */
	const int32_t travel_end_q8 = (int32_t)(center_lit_count + 2U) * 256;
	if (belt_wave_head_q8 > travel_end_q8) {
		belt_wave_head_q8 = -512;
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		uint16_t intensity = 0U;
		if (level < center_lit_count) {
			int32_t distance_q8 =
					(int32_t)level * 256 - belt_wave_head_q8;
			if (distance_q8 < 0) {
				distance_q8 = -distance_q8;
			}
			intensity = 6U;
			if (distance_q8 < radius_q8) {
				const uint32_t strength = (uint32_t)(radius_q8 - distance_q8);
				intensity += (uint16_t)(214U * strength * strength
						/ ((uint32_t)radius_q8 * (uint32_t)radius_q8));
			}
		}

		setPixelPA6(level, (uint8_t)(intensity * 185U / 255U), 0U,
				(uint8_t)intensity);
		setPixelPA6(PA6_MAIN_LED_COUNT - 1U - level,
				(uint8_t)(intensity * 185U / 255U), 0U,
				(uint8_t)intensity);
	}

	for (uint16_t level = 0U; level < LAUNCHER_SIDE_COUNT; level++) {
		uint16_t intensity = 0U;
		if (level < side_lit_count) {
			int32_t distance_q8 =
					(int32_t)level * 256 - belt_wave_head_q8;
			if (distance_q8 < 0) {
				distance_q8 = -distance_q8;
			}
			intensity = 6U;
			if (distance_q8 < radius_q8) {
				const uint32_t strength = (uint32_t)(radius_q8 - distance_q8);
				intensity += (uint16_t)(214U * strength * strength
						/ ((uint32_t)radius_q8 * (uint32_t)radius_q8));
			}
		}

		setPixelPA6(PA6_MAIN_LED_COUNT + level,
				(uint8_t)(intensity * 185U / 255U), 0U,
				(uint8_t)intensity);
		setPixelPA2(level, (uint8_t)(intensity * 185U / 255U), 0U,
				(uint8_t)intensity);
	}
}

/*
 * SHOT_OPENING launcher sequence:
 *  1. Continue the current belt-purple wave to the tip.
 *  2. Fill purple gradually from the tip back toward the root.
 */
static bool LED_RenderShotOpeningLauncher(uint32_t now_ms, bool entering) {
	const int32_t tip_q8 = (LAUNCHER_LEVEL_COUNT - 1U) * 256;
	const int32_t wave_radius_q8 = 640;

	if (entering) {
		shot_opening_phase = 0U;
		shot_opening_head_q8 = belt_wave_active ? belt_wave_head_q8 : 0;
		if (shot_opening_head_q8 < 0) {
			shot_opening_head_q8 = 0;
		} else if (shot_opening_head_q8 > tip_q8) {
			shot_opening_head_q8 = tip_q8;
		}
		shot_opening_fill_q8 = tip_q8;
		shot_opening_last_update_ms = now_ms;
		belt_wave_active = false;
	}

	const uint32_t elapsed_ms = now_ms - shot_opening_last_update_ms;
	shot_opening_last_update_ms = now_ms;

	if (shot_opening_phase == 0U) {
		shot_opening_head_q8 += (int32_t)(elapsed_ms
				* SHOT_OPENING_HEAD_SPEED_Q8_PER_MS);
		if (shot_opening_head_q8 >= tip_q8) {
			shot_opening_head_q8 = tip_q8;
			shot_opening_phase = 1U;
			shot_opening_fill_q8 = tip_q8;
		}
	} else if (shot_opening_phase == 1U) {
		shot_opening_fill_q8 -= (int32_t)(elapsed_ms
				* SHOT_OPENING_FILL_SPEED_Q8_PER_MS);
		if (shot_opening_fill_q8 <= 0) {
			shot_opening_fill_q8 = 0;
			shot_opening_phase = 2U;
		}
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		const int32_t position_q8 = (int32_t)level * 256;
		uint16_t intensity = 6U;

		if (shot_opening_phase == 0U) {
			int32_t distance_q8 = position_q8 - shot_opening_head_q8;
			if (distance_q8 < 0) {
				distance_q8 = -distance_q8;
			}
			if (distance_q8 < wave_radius_q8) {
				const uint32_t strength =
						(uint32_t)(wave_radius_q8 - distance_q8);
				intensity += (uint16_t)(214U * strength * strength
						/ ((uint32_t)wave_radius_q8
								* (uint32_t)wave_radius_q8));
			}
		} else {
			if (position_q8 >= shot_opening_fill_q8) {
				intensity = 220U;
			} else {
				const int32_t edge_distance_q8 =
						shot_opening_fill_q8 - position_q8;
				if (edge_distance_q8 < 512) {
					intensity += (uint16_t)(214U
							* (uint32_t)(512 - edge_distance_q8) / 512U);
				}
			}
		}

		LED_SetLauncherLevel(level,
				(uint8_t)(intensity * 185U / 255U), 0U,
				(uint8_t)intensity);
	}

	return shot_opening_phase == 2U;
}

/* PA6[20..28] -> PA6[10] -> PA6[9] -> PA2[8..0]. */
static void LED_RenderLauncherSideEmergency(uint32_t now_ms) {
	for (uint16_t loop_pixel = 0U;
			loop_pixel < LAUNCHER_SIDE_LOOP_COUNT; loop_pixel++) {
		const uint8_t red = Emergency_GetIntensityFor(loop_pixel,
				LAUNCHER_SIDE_LOOP_COUNT, now_ms, false);

		if (loop_pixel < LAUNCHER_SIDE_COUNT) {
			setPixelPA6(PA6_MAIN_LED_COUNT + loop_pixel, red, 0U, 0U);
		} else if (loop_pixel == LAUNCHER_SIDE_COUNT) {
			setPixelPA6(10U, red, 0U, 0U);
		} else if (loop_pixel == (LAUNCHER_SIDE_COUNT + 1U)) {
			setPixelPA6(9U, red, 0U, 0U);
		} else {
			const uint16_t left_pixel =
					LAUNCHER_SIDE_LOOP_COUNT - 1U - loop_pixel;
			setPixelPA2(left_pixel, red, 0U, 0U);
		}
	}
}

static void LED_ApplyAuxiliaryStatus(uint8_t status) {
	uint8_t belt_level = status & 0x07U;
	const bool dribble_enabled = (status & 0x08U) != 0U;
	const bool drive_reversed = (status & 0x10U) != 0U;
	const bool game2_enabled = (status & 0x20U) != 0U;

	if (belt_level > 4U) {
		belt_level = 0U;
	}
	for (uint16_t i = 0U; i < 4U; i++) {
		setPixelPA7(i, 0U, (i < belt_level) ? 180U : 0U, 0U);
	}
	setPixelPA7(4U, 0U, dribble_enabled ? 180U : 0U,
			dribble_enabled ? 180U : 0U);
	setPixelPA7(5U, game2_enabled ? 150U : 0U, 0U,
			game2_enabled ? 200U : 0U);

	if (drive_reversed) {
		setPixelPA7(LED_FRONT_CENTER, 220U, 45U, 0U);
		setPixelPA7(LED_REAR_CENTER, 0U, 100U, 220U);
	} else {
		setPixelPA7(LED_FRONT_CENTER, 0U, 100U, 220U);
		setPixelPA7(LED_REAR_CENTER, 220U, 45U, 0U);
	}
}

/* data[1] bit 3: three symmetric deep-blue waves from rear to front. */
static void LED_RenderDribbleChassis(uint32_t now_ms) {
	const uint16_t max_distance = PA7_LED_NUM / 2U;
	const int32_t travel_levels = (int32_t)max_distance + 8;
	const uint32_t elapsed_ms = now_ms % DRIBBLE_SWEEP_MS;

	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		const uint16_t clockwise_distance =
				(uint16_t)((i + PA7_LED_NUM - LED_FRONT_CENTER)
						% PA7_LED_NUM);
		const uint16_t counterclockwise_distance =
				PA7_LED_NUM - clockwise_distance;
		const uint16_t symmetric_distance =
				(clockwise_distance < counterclockwise_distance)
						? clockwise_distance : counterclockwise_distance;
		int32_t nearest_wave_distance_q8 = travel_levels * 256;

		for (uint16_t wave = 0U; wave < DRIBBLE_WAVE_COUNT; wave++) {
			const uint32_t wave_elapsed_ms = (elapsed_ms
					+ (uint32_t)wave * DRIBBLE_SWEEP_MS
							/ DRIBBLE_WAVE_COUNT) % DRIBBLE_SWEEP_MS;
			const int32_t head_q8 = (int32_t)max_distance * 256
					+ DRIBBLE_GLOW_RADIUS_Q8
					- (int32_t)(wave_elapsed_ms
							* (uint32_t)travel_levels * 256U
							/ DRIBBLE_SWEEP_MS);
			int32_t wave_distance_q8 =
					(int32_t)symmetric_distance * 256 - head_q8;
			if (wave_distance_q8 < 0) {
				wave_distance_q8 = -wave_distance_q8;
			}
			if (wave_distance_q8 < nearest_wave_distance_q8) {
				nearest_wave_distance_q8 = wave_distance_q8;
			}
		}

		uint16_t intensity = DRIBBLE_BASE_INTENSITY;
		if (nearest_wave_distance_q8 < DRIBBLE_GLOW_RADIUS_Q8) {
			const uint32_t strength =
					(uint32_t)(DRIBBLE_GLOW_RADIUS_Q8
							- nearest_wave_distance_q8);
			const uint32_t radius_squared =
					(uint32_t)DRIBBLE_GLOW_RADIUS_Q8
							* (uint32_t)DRIBBLE_GLOW_RADIUS_Q8;
			const uint32_t highlight = strength * strength
					* (DRIBBLE_PEAK_INTENSITY - DRIBBLE_BASE_INTENSITY)
					/ radius_squared;
			intensity += (uint16_t)highlight;
		}
		setPixelPA7(i, 0U, (uint8_t)(intensity / 12U),
				(uint8_t)intensity);
	}
}

static void LED_CaptureFiringBackground(uint32_t now_ms) {
	firing_start_ms = now_ms;
	for (uint16_t i = 0U; i < PA6_LED_NUM; i++) {
		getPixelPA6(i, &firing_pa6_background[i][0],
				&firing_pa6_background[i][1],
				&firing_pa6_background[i][2]);
	}
	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		getPixelPA7(i, &firing_pa7_background[i][0],
				&firing_pa7_background[i][1],
				&firing_pa7_background[i][2]);
	}
	for (uint16_t i = 0U; i < PA2_LED_NUM; i++) {
		getPixelPA2(i, &firing_pa2_background[i][0],
				&firing_pa2_background[i][1],
				&firing_pa2_background[i][2]);
	}
}

static void LED_BlendFiringPA6(uint16_t pixel, uint16_t red_strength) {
	const uint16_t keep_strength = 255U - red_strength;
	const uint8_t background_red = firing_pa6_background[pixel][0];
	const uint8_t background_green = firing_pa6_background[pixel][1];
	const uint8_t background_blue = firing_pa6_background[pixel][2];
	setPixelPA6(pixel,
			(uint8_t)(background_red
					+ ((255U - background_red) * red_strength / 255U)),
			(uint8_t)(background_green * keep_strength / 255U),
			(uint8_t)(background_blue * keep_strength / 255U));
}

static void LED_BlendFiringPA2(uint16_t pixel, uint16_t red_strength) {
	const uint16_t keep_strength = 255U - red_strength;
	const uint8_t background_red = firing_pa2_background[pixel][0];
	const uint8_t background_green = firing_pa2_background[pixel][1];
	const uint8_t background_blue = firing_pa2_background[pixel][2];
	setPixelPA2(pixel,
			(uint8_t)(background_red
					+ ((255U - background_red) * red_strength / 255U)),
			(uint8_t)(background_green * keep_strength / 255U),
			(uint8_t)(background_blue * keep_strength / 255U));
}

static uint16_t LED_FiringRedStrength(int32_t distance_q8,
		int32_t full_radius_q8, int32_t outer_radius_q8) {
	if (distance_q8 <= full_radius_q8) {
		return 255U;
	}
	if (distance_q8 >= outer_radius_q8) {
		return 0U;
	}

	const uint32_t fade_width_q8 =
			(uint32_t)(outer_radius_q8 - full_radius_q8);
	const uint32_t strength_q8 =
			(uint32_t)(outer_radius_q8 - distance_q8);
	return (uint16_t)(strength_q8 * strength_q8 * 255U
			/ (fade_width_q8 * fade_width_q8));
}

/* Preserve the launcher frame and blend one red wave from tip to root. */
static void LED_RenderFiringLauncher(uint32_t elapsed_ms) {
	const int32_t travel_q8 =
			(int32_t)(LAUNCHER_LEVEL_COUNT - 1U) * 256
			+ 2 * FIRING_LAUNCHER_OUTER_RADIUS_Q8;
	int32_t head_q8 = -FIRING_LAUNCHER_OUTER_RADIUS_Q8;

	if (elapsed_ms < FIRING_WAVE_MS) {
		head_q8 = (int32_t)(LAUNCHER_LEVEL_COUNT - 1U) * 256
				+ FIRING_LAUNCHER_OUTER_RADIUS_Q8
				- (int32_t)(elapsed_ms * (uint32_t)travel_q8
						/ FIRING_WAVE_MS);
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		int32_t distance_q8 = (int32_t)level * 256 - head_q8;
		if (distance_q8 < 0) {
			distance_q8 = -distance_q8;
		}

		const uint16_t red_strength = LED_FiringRedStrength(distance_q8,
				FIRING_LAUNCHER_FULL_RADIUS_Q8,
				FIRING_LAUNCHER_OUTER_RADIUS_Q8);

		LED_BlendFiringPA6(level, red_strength);
		LED_BlendFiringPA6(PA6_MAIN_LED_COUNT - 1U - level,
				red_strength);
		if (level < LAUNCHER_SIDE_COUNT) {
			LED_BlendFiringPA6(PA6_MAIN_LED_COUNT + level,
					red_strength);
			LED_BlendFiringPA2(level, red_strength);
		}
	}
}

/* Preserve the previous chassis frame and blend one symmetric red wave over it. */
static void LED_RenderFiringChassis(uint32_t elapsed_ms) {
	const uint16_t max_distance = PA7_LED_NUM / 2U;
	const int32_t travel_q8 = (int32_t)max_distance * 256
			+ 2 * FIRING_CHASSIS_OUTER_RADIUS_Q8;
	int32_t head_q8 = (int32_t)max_distance * 256
			+ FIRING_CHASSIS_OUTER_RADIUS_Q8;

	if (elapsed_ms < FIRING_WAVE_MS) {
		head_q8 = -FIRING_CHASSIS_OUTER_RADIUS_Q8
				+ (int32_t)(elapsed_ms * (uint32_t)travel_q8
						/ FIRING_WAVE_MS);
	}

	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		const uint16_t clockwise_distance =
				(uint16_t)((i + PA7_LED_NUM - LED_FRONT_CENTER)
						% PA7_LED_NUM);
		const uint16_t counterclockwise_distance =
				PA7_LED_NUM - clockwise_distance;
		const uint16_t symmetric_distance =
				(clockwise_distance < counterclockwise_distance)
						? clockwise_distance : counterclockwise_distance;
		int32_t distance_q8 = (int32_t)symmetric_distance * 256 - head_q8;
		if (distance_q8 < 0) {
			distance_q8 = -distance_q8;
		}

		const uint16_t red_strength = LED_FiringRedStrength(distance_q8,
				FIRING_CHASSIS_FULL_RADIUS_Q8,
				FIRING_CHASSIS_OUTER_RADIUS_Q8);

		const uint16_t keep_strength = 255U - red_strength;
		const uint8_t background_red = firing_pa7_background[i][0];
		const uint8_t background_green = firing_pa7_background[i][1];
		const uint8_t background_blue = firing_pa7_background[i][2];
		const uint8_t red = (uint8_t)(background_red
				+ ((255U - background_red) * red_strength / 255U));
		const uint8_t green = (uint8_t)(background_green
				* keep_strength / 255U);
		const uint8_t blue = (uint8_t)(background_blue
				* keep_strength / 255U);
		setPixelPA7(i, red, green, blue);
	}
}

/* Render the CAN 0x201 LED command defined in LED_BIT_ASSIGNMENT.md. */
void shining_LED(int received_LED_cmd) {
	const uint32_t now_ms = HAL_GetTick();
	const uint8_t mode = (uint8_t)received_LED_cmd;
	static uint8_t previous_mode = 0xFFU;
	static bool loading_wait_started = false;
	static uint32_t loading_wait_start_ms = 0U;
	const uint8_t mode_before_change = previous_mode;
	const bool mode_changed = mode != previous_mode;

	if (mode_changed) {
		if (mode == 5U) {
			LED_CaptureFiringBackground(now_ms);
		}
		previous_mode = mode;
		if (mode == 4U) {
			loading_wait_started = false;
		}
	}

	switch (mode) {
	case 0U: /* STARTUP: steady light cyan with a gentle brightness sway */
	{
		const uint8_t pulse = LED_SmoothPulse(now_ms, 2400U);
		const uint8_t green = (uint8_t)(145U + (uint16_t)pulse * 30U / 255U);
		const uint8_t blue = (uint8_t)(185U + (uint16_t)pulse * 35U / 255U);
		LED_SetAll(0U, green, blue);
		LED_SetLauncherAll(0U, green, blue);
		break;
	}
	case 1U: /* READY: steady light cyan with a gentle brightness sway */
	{
		const uint8_t pulse = LED_SmoothPulse(now_ms, 2400U);
		const uint8_t green = (uint8_t)(145U + (uint16_t)pulse * 30U / 255U);
		const uint8_t blue = (uint8_t)(185U + (uint16_t)pulse * 35U / 255U);
		LED_SetAll(0U, green, blue);
		LED_SetLauncherAll(0U, green, blue);
		break;
	}
	case 2U: /* EMERGENCY_STOP: two red gradients, 600 ms per lap */
		for (uint16_t i = 0U; i < PA6_MAIN_LED_COUNT; i++) {
			const uint8_t red = Emergency_GetIntensityFor(i,
					PA6_MAIN_LED_COUNT, now_ms, true);
			setPixelPA6(i, red, 0U, 0U);
		}
		for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
			const uint8_t red = Emergency_GetIntensityFor(i,
					PA7_LED_NUM, now_ms, true);
			setPixelPA7(i, red, 0U, 0U);
		}
		LED_RenderLauncherSideEmergency(now_ms);
		break;
	case 3U: /* SHOT_OPENING: cyan, rear to front */
	{
		const uint16_t max_distance = PA7_LED_NUM / 2U;
		const uint16_t head = (uint16_t)(max_distance
				- (now_ms % 900U) * max_distance / 900U);
		for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
			const uint16_t position = LED_CircularDistance(i, LED_FRONT_CENTER);
			const uint16_t distance = (position > head)
					? (position - head) : (head - position);
			const uint8_t glow = LED_GlowIntensity(distance);
			setPixelPA7(i, 0U, (uint8_t)(glow * 3U / 4U), glow);
		}
		(void)LED_RenderShotOpeningLauncher(now_ms, mode_changed);
		break;
	}
	case 4U: /* LOADING: finish purple fill, wait briefly, then solid red */
	{
		/* Continue SHOT_OPENING even after the mode changes to LOADING. */
		const bool purple_complete = LED_RenderShotOpeningLauncher(now_ms,
				mode_changed && (mode_before_change != 3U));

		if (purple_complete && !loading_wait_started) {
			loading_wait_started = true;
			loading_wait_start_ms = now_ms;
		}

		if (loading_wait_started &&
				((uint32_t)(now_ms - loading_wait_start_ms)
						>= LOADING_AFTER_PURPLE_HOLD_MS)) {
			LED_SetAll(255U, 0U, 0U);
			LED_SetLauncherAll(255U, 0U, 0U);
		}
		break;
	}
	case 5U: /* FIRING: one red wave over the previous chassis display */
	{
		const uint32_t firing_elapsed_ms = now_ms - firing_start_ms;
		LED_RenderFiringChassis(firing_elapsed_ms);
		LED_RenderFiringLauncher(firing_elapsed_ms);
		break;
	}
	case 6U: /* RETURNING: blue, front to rear */
	{
		const uint16_t max_distance = PA7_LED_NUM / 2U;
		const uint16_t head = (uint16_t)((now_ms % 900U)
				* max_distance / 900U);
		for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
			const uint16_t position = LED_CircularDistance(i, LED_FRONT_CENTER);
			const uint16_t distance = (position > head)
					? (position - head) : (head - position);
			const uint8_t glow = LED_GlowIntensity(distance);
			setPixelPA7(i, 0U, (uint8_t)(glow / 4U), glow);
		}
		LED_RenderLauncherWave(now_ms, 900U, false, 0U, 64U, 255U);
		break;
	}
	case 7U: /* GAME2_SEARCHING: purple scanner */
	{
		const uint16_t head = (uint16_t)((now_ms % 1200U)
				* PA7_LED_NUM / 1200U);
		for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
			const uint8_t glow = LED_GlowIntensity(LED_CircularDistance(i, head));
			setPixelPA7(i, (uint8_t)(glow * 2U / 3U), 0U, glow);
		}
		LED_RenderLauncherWave(now_ms, 1200U, true, 170U, 0U, 255U);
		break;
	}
	case 8U: /* GAME2_ALIGNING: cyan pulse at front */
	{
		const uint8_t pulse = LED_TriangleWave(now_ms, 500U);
		for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
			const uint16_t distance = LED_CircularDistance(i, LED_FRONT_CENTER);
			uint8_t intensity = 0U;
			if (distance <= 3U) {
				intensity = (uint8_t)((uint16_t)pulse * (4U - distance) / 4U);
			}
			setPixelPA7(i, 0U, intensity, intensity);
		}
		LED_SetLauncherAll(0U, pulse, pulse);
		break;
	}
	case 9U: /* ERROR: fast red blink, 100 ms on/off */
	default:
	{
		const uint8_t red = ((now_ms / 100U) & 1U) ? 255U : 0U;
		LED_SetAll(red, 0U, 0U);
		LED_SetLauncherAll(red, 0U, 0U);
		break;
	}
	}

	if ((mode != 0U) && (mode != 2U) && (mode != 4U) && (mode != 5U) &&
			(mode != 9U) && (mode <= 8U)) {
		const bool dribble_enabled =
				(received_LED_status & 0x08U) != 0U;
		if (dribble_enabled) {
			LED_RenderDribbleChassis(now_ms);
		} else if (mode != 1U) {
			LED_ApplyAuxiliaryStatus(received_LED_status);
		}
		/* SHOT_OPENING and LOADING keep their dedicated launcher effects. */
		if ((mode != 3U) && (mode != 4U)) {
			LED_RenderBeltLauncher(received_LED_status, now_ms);
		}
	}
	show();
}

#define BNO055_REG_CHIP_ID_LOCAL 0x00U
#define BNO055_CHIP_ID_LOCAL     0xA0U
#define BNO055_PROBE_TIMEOUT_MS  5U

static bool BNO055_ProbeAddress(uint16_t address, uint8_t *chip_id) {
	if (HAL_I2C_IsDeviceReady(&hi2c1, address, 1U,
			BNO055_PROBE_TIMEOUT_MS) != HAL_OK) {
		return false;
	}

	if (HAL_I2C_Mem_Read(&hi2c1, address, BNO055_REG_CHIP_ID_LOCAL,
			I2C_MEMADD_SIZE_8BIT, chip_id, 1U,
			BNO055_PROBE_TIMEOUT_MS) != HAL_OK) {
		return false;
	}

	return *chip_id == BNO055_CHIP_ID_LOCAL;
}

static bool BNO055_TryInitialize(void) {
	const uint32_t start_ms = HAL_GetTick();
	uint16_t address = 0U;
	uint8_t chip_id = 0U;
	BNO055_Status status = BNO055_ERROR;

	/* Detect both legal BNO055 addresses without a long blocking reset. */
	if (BNO055_ProbeAddress(BNO055_I2C_ADDR_LOW, &chip_id)) {
		address = BNO055_I2C_ADDR_LOW;
	} else if (BNO055_ProbeAddress(BNO055_I2C_ADDR_HIGH, &chip_id)) {
		address = BNO055_I2C_ADDR_HIGH;
	}

	debug_bno_i2c_address = address;
	debug_bno_chip_id = chip_id;

	if (address != 0U) {
		bno055.hi2c = &hi2c1;
		bno055.address = address;
		bno055.mode = BNO055_MODE_CONFIG;

		/* Restore the operating mode; do not reset and wait 700 ms on retry. */
		status = BNO055_SetMode(&bno055, BNO055_MODE_CONFIG);
		if (status == BNO055_OK) {
			status = BNO055_SetMode(&bno055, BNO055_MODE_NDOF);
		}
	}

	debug_bno_init_status = status;
	debug_bno_init_duration_ms = HAL_GetTick() - start_ms;

	if (status == BNO055_OK) {
		bno055_connected = true;
		bno055_read_error_count = 0;

		return true;
	}

	bno055_connected = false;
	bno055_init_error_count++;

	return false;
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

	return true;
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
  /* USER CODE BEGIN 2 */

	LED_Init();
	HAL_Delay(100);
	//BNO055初期化
	bno055_connected = BNO055_TryInitialize();
	last_bno055_retry_time = HAL_GetTick();

	can_connected = CAN_TryInitialize();
	last_can_retry_time = HAL_GetTick();

	//駆動電源の出力をON,OFFするピン
	//HIGH出力で，電源入る，LOW出力でOFFに
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// LEDの初期化
	last_can_rx_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// --- 1. 通信のタイムアウト＆非常停止の監視 ---
		uint32_t now = HAL_GetTick();
		if ((now - last_loop_time) >= LOOP_TIME) {
			last_loop_time += LOOP_TIME;

			if (!can_connected) {
				if ((uint32_t) (now - last_can_retry_time)
						>= CAN_RETRY_INTERVAL_MS) {
					last_can_retry_time = now;
					CAN_TryInitialize();
				}
			}
			if (!bno055_connected) {
				if ((uint32_t) (now - last_bno055_retry_time)
						>= BNO055_RETRY_INTERVAL_MS) {
					last_bno055_retry_time = now;
					BNO055_TryInitialize();
				}
			}

			if (bno055_connected) {
				BNO055_Status read_status = BNO055_ReadQuaternion(&bno055,
						&imu_quat);
				if (read_status == BNO055_OK) {
					bno055_read_error_count = 0;
					if (can_connected) {
						uint8_t TxData[8];
						CAN_TxHeaderTypeDef TxHeader1 = { 0 };
						uint32_t TxMailbox1;
						TxHeader1.StdId = 0x320;
						TxHeader1.ExtId = 0;
						TxHeader1.RTR = CAN_RTR_DATA;
						TxHeader1.IDE = CAN_ID_STD;
						TxHeader1.DLC = 8;
						TxHeader1.TransmitGlobalTime = DISABLE;

						TxData[0] = (uint8_t) (imu_quat.x);
						TxData[1] = (uint8_t) (imu_quat.x >> 8);
						TxData[2] = (uint8_t) (imu_quat.y);
						TxData[3] = (uint8_t) (imu_quat.y >> 8);
						TxData[4] = (uint8_t) (imu_quat.z);
						TxData[5] = (uint8_t) (imu_quat.z >> 8);
						TxData[6] = (uint8_t) (imu_quat.w);
						TxData[7] = (uint8_t) (imu_quat.w >> 8);

						if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U) {
							if (HAL_CAN_AddTxMessage(&hcan, &TxHeader1, TxData,
									&TxMailbox1) == HAL_OK) {
								can_tx_error_count = 0;
								bno055_tx_count++;
								bno_com_priod = HAL_GetTick()- last_bno_com_time;
								last_bno_com_time = HAL_GetTick();
							} else {
								can_tx_error_count++;
							}
						}

						if (can_tx_error_count >= CAN_MAX_TX_ERRORS) {
							can_connected = false;
							can_tx_error_count = 0;
							last_can_retry_time = now;
						}
					}
				} else {
					bno055_read_error_count++;
					if (bno055_read_error_count >= BNO055_MAX_READ_ERRORS) {
						bno055_connected = false;
						bno055_read_error_count = 0;
						last_bno055_retry_time = now;
					}
				}
			}

			// --- 5. リミットスイッチの状態を更新＆送信 ---
			if (can_connected) {
				LimitSwitch_UpdateAndSend(&hcan);
			}

			if ((now - last_can_rx_time) > CAN_TIMEOUT_MS) {
				is_timeout = 1; //timeout中
				/* 通信タイムアウト中は、他の表示よりcase 9を優先する。 */
				shining_LED(9);
			} else {
				// --- 6. LEDの点灯更新 ---
				shining_LED(received_LED_cmd);
			}

		}
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
	if (err & HAL_CAN_ERROR_BOF) {
		can_connected = false;
		last_can_retry_time = HAL_GetTick();
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
