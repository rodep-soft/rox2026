#include "LED_effects.h"
#include "LED_lite.h"
#include "main.h"
#include <stdbool.h>

/* CAN 0x201 LED protocol and physical layout. */
#define LED_STATUS_BELT_MASK          0x07U
#define LED_STATUS_DRIBBLE_ENABLED    0x08U
#define LED_STATUS_DRIVE_REVERSED     0x10U
#define LED_STATUS_GAME2_ENABLED      0x20U
#define LED_STATUS_ROLLER_FORWARD     0x40U
#define LED_STATUS_ROLLER_REVERSE     0x80U
#define GRID_STATE_STANDING             0U
#define GRID_STATE_TARGET               1U
#define GRID_STATE_FALLEN               2U
#define GRID_CELL_COUNT                 9U
#define GRID_TARGET_BLINK_MS           300U

#define LAUNCHER_MAIN_LED_COUNT       20U
#define LAUNCHER_SIDE_LED_COUNT        9U
#define MIDDLE_FRONT_LED_COUNT         7U
#define LAUNCHER_LEVEL_COUNT          10U
#define MIDDLE_CHAIN_START            20U
#define CHASSIS_FRONT_CENTER          15U
#define CHASSIS_REAR_CENTER           34U
#define IMU_DATA_TIMEOUT_MS          200U
#define IMU_WARNING_BLINK_MS         250U
#define IMU_WARNING_COLOR_R          255U
#define IMU_WARNING_COLOR_G           80U
#define IMU_WARNING_COLOR_B            0U

#define EMERGENCY_SWEEP_MS           600U
#define EMERGENCY_WAVE_COUNT           2U
#define EMERGENCY_BRIGHTNESS         220U
#define CHASSIS_GLOW_RADIUS_Q8   (4L * 256L)
#define CHASSIS_BASE_INTENSITY        12U
#define CHASSIS_PEAK_INTENSITY       255U
#define ARM_OPEN_HEAD_SPEED_Q8_PER_MS 16U
#define ARM_OPEN_FILL_SPEED_Q8_PER_MS 32U
#define FIRING_WAVE_MS               233U
#define FIRING_START_DELAY_MS        100U /* Wait after FIRING begins before wave. */
#define AUTO_FIRING_EFFECT_MS       1000U
#define AUTO_FIRING_INITIAL_WAVE_MS (4U * FIRING_WAVE_MS)
#define FIRING_COLOR_R               255U
#define FIRING_COLOR_G               255U
#define FIRING_COLOR_B               255U
#define SLOW_FIRING_WAVE_MS          900U
#define FIRING_LAUNCHER_OUTER_RADIUS_Q8 (14L * 256L)
#define FIRING_LAUNCHER_FULL_RADIUS_Q8  (10L * 256L)
#define FIRING_CHASSIS_OUTER_RADIUS_Q8  (22L * 256L)
#define FIRING_CHASSIS_FULL_RADIUS_Q8   (17L * 256L)

static bool belt_wave_active = false;
static int32_t belt_wave_head_q8 = -512L;
static uint32_t belt_wave_last_update_ms;
static uint8_t arm_open_phase;
static int32_t arm_open_head_q8;
static int32_t arm_open_fill_q8;
static uint32_t arm_open_last_update_ms;
static uint32_t arm_open_head_speed_remainder;
static uint32_t arm_open_fill_speed_remainder;
static uint8_t firing_launcher_background[PA6_LED_NUM][3];
static uint8_t firing_chassis_background[PA7_LED_NUM][3];
static uint32_t firing_start_ms;
volatile uint8_t debug_led_game2_search_active;
extern uint32_t last_bno_com_time;

static uint16_t LED_CircularDistance(uint16_t a, uint16_t b) {
	const uint16_t distance = (a > b) ? (a - b) : (b - a);
	const uint16_t wrapped = LED_NUM - distance;
	return (wrapped < distance) ? wrapped : distance;
}

static uint8_t LED_TriangleWave(uint32_t now_ms, uint32_t period_ms) {
	const uint32_t phase = now_ms % period_ms;
	const uint32_t half = period_ms / 2U;
	if (phase < half) {
		return (uint8_t)(phase * 255U / half);
	}
	return (uint8_t)((period_ms - phase) * 255U / half);
}

static uint8_t LED_SmoothPulse(uint32_t now_ms, uint32_t period_ms) {
	const uint32_t x = LED_TriangleWave(now_ms, period_ms);
	return (uint8_t)(x * x * (765U - 2U * x) / (255U * 255U));
}

static uint8_t LED_GlowIntensity(uint16_t distance) {
	switch (distance) {
	case 0U: return 220U;
	case 1U: return 110U;
	case 2U: return 35U;
	default: return 0U;
	}
}

static void LED_SetChassisAll(uint8_t red, uint8_t green, uint8_t blue) {
	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		setPixelPA7(i, red, green, blue);
	}
}

static void LED_SetLauncherAll(uint8_t red, uint8_t green, uint8_t blue) {
	for (uint16_t i = 0U; i < PA6_LED_NUM; i++) {
		setPixelPA6(i, red, green, blue);
	}
}

static void LED_ApplyImuDisconnectedWarning(uint32_t now_ms) {
	if ((uint32_t)(now_ms - last_bno_com_time) < IMU_DATA_TIMEOUT_MS) {
		return;
	}
	const bool warning_on =
			((now_ms / IMU_WARNING_BLINK_MS) & 1U) != 0U;
	const uint8_t red = warning_on ? IMU_WARNING_COLOR_R : 0U;
	const uint8_t green = warning_on ? IMU_WARNING_COLOR_G : 0U;
	const uint8_t blue = warning_on ? IMU_WARNING_COLOR_B : 0U;
	LED_SetLauncherAll(red, green, blue);
	LED_SetChassisAll(red, green, blue);
}

static void LED_SetLauncherLevel(uint16_t level,
		uint8_t red, uint8_t green, uint8_t blue) {
	if (level >= LAUNCHER_LEVEL_COUNT) {
		return;
	}

	setPixelPA6(level, red, green, blue);
	setPixelPA6(LAUNCHER_MAIN_LED_COUNT - 1U - level, red, green, blue);
	if (level < LAUNCHER_SIDE_LED_COUNT) {
		setPixelPA6(LAUNCHER_MAIN_LED_COUNT + level, red, green, blue);
		setPixelPA2(level, red, green, blue);
	} else {
		for (uint16_t i = 0U; i < MIDDLE_FRONT_LED_COUNT; i++) {
			setPixelPA6(LAUNCHER_MAIN_LED_COUNT +
					LAUNCHER_SIDE_LED_COUNT + i, red, green, blue);
		}
	}
}

static void LED_OverlayBeltOffset(uint8_t mode, bool automatic) {
	const bool zero = mode == LED_MODE_BELT_OFFSET_ZERO;
	const bool positive = mode > LED_MODE_BELT_OFFSET_ZERO;
	const uint8_t steps = zero ? 3U : (positive ?
			(uint8_t)(mode - LED_MODE_BELT_OFFSET_ZERO) :
			(uint8_t)(LED_MODE_BELT_OFFSET_ZERO - mode));
	const uint8_t red = (zero || positive) ? 0U : 255U;
	const uint8_t green = zero ? 80U : (positive ? 255U : 80U);
	const uint8_t blue = zero ? 255U : 0U;
	const uint16_t left_start = automatic ? 3U : 0U;
	const uint16_t right_start = automatic ? 16U :
			(LAUNCHER_MAIN_LED_COUNT - 1U);
	for (uint16_t level = 0U; level < steps; level++) {
		setPixelPA6((uint16_t)(left_start + level), red, green, blue);
		setPixelPA6((uint16_t)(right_start - level), red, green, blue);
	}
}

static void LED_RenderLauncherWave(uint32_t now_ms, uint32_t period_ms,
		bool toward_tip, uint8_t red, uint8_t green, uint8_t blue) {
	uint16_t head = (uint16_t)((now_ms % period_ms) *
			(LAUNCHER_LEVEL_COUNT - 1U) / period_ms);
	if (!toward_tip) {
		head = LAUNCHER_LEVEL_COUNT - 1U - head;
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		const uint16_t distance = (level > head) ?
				(level - head) : (head - level);
		const uint8_t glow = LED_GlowIntensity(distance);
		LED_SetLauncherLevel(level,
				(uint8_t)((uint16_t)red * glow / 255U),
				(uint8_t)((uint16_t)green * glow / 255U),
				(uint8_t)((uint16_t)blue * glow / 255U));
	}
}

/* Normal/reversed upper indication: identical gradient, color and direction differ. */
static void LED_RenderDriveLauncherGradient(uint32_t now_ms,
		bool toward_tip, bool blue_gradient) {
	const uint32_t period_ms = 1500U;
	const int32_t radius_q8 = 3L * 256L;
	const int32_t travel_q8 = (LAUNCHER_LEVEL_COUNT + 6L) * 256L;
	const int32_t travelled_q8 = (int32_t)((now_ms % period_ms) *
			(uint32_t)travel_q8 / period_ms);
	const int32_t head_q8 = toward_tip ?
			(-2L * 256L + travelled_q8) :
			((LAUNCHER_LEVEL_COUNT + 2L) * 256L - travelled_q8);

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		int32_t distance_q8 = (int32_t)level * 256L - head_q8;
		if (distance_q8 < 0L) {
			distance_q8 = -distance_q8;
		}

		uint16_t glow = 0U;
		if (distance_q8 < radius_q8) {
			const uint32_t strength = (uint32_t)(radius_q8 - distance_q8);
			glow = (uint16_t)(strength * strength * 255U /
					((uint32_t)radius_q8 * (uint32_t)radius_q8));
		}

		/* Blue is the same gradient as Gold with the primary channel swapped. */
		const uint8_t primary = (uint8_t)(18U + 237U * glow / 255U);
		uint8_t accent;
		if (glow <= 160U) {
			accent = (uint8_t)(10U + glow);
		} else {
			accent = (uint8_t)(170U -
					(uint16_t)(glow - 160U) * 130U / 95U);
		}
		if (blue_gradient) {
			LED_SetLauncherLevel(level, 0U, accent, primary);
		} else {
			LED_SetLauncherLevel(level, primary, accent, 0U);
		}
	}
}

static uint8_t LED_EmergencyIntensity(uint16_t pixel, uint16_t pixel_count,
		uint32_t now_ms, bool reverse_direction) {
	const uint16_t animation_pixel = reverse_direction ?
			(uint16_t)(pixel_count - 1U - pixel) : pixel;
	const int32_t circumference_q8 = (int32_t)pixel_count * 256L;
	const int32_t glow_radius_q8 = circumference_q8 / 8L;
	const int32_t phase_q8 = (int32_t)((now_ms % EMERGENCY_SWEEP_MS) *
			(uint32_t)circumference_q8 / EMERGENCY_SWEEP_MS);
	int32_t nearest_wave_q8 = circumference_q8;

	for (uint8_t wave = 0U; wave < EMERGENCY_WAVE_COUNT; wave++) {
		const int32_t head_q8 = (phase_q8 +
				(int32_t)wave * circumference_q8 / EMERGENCY_WAVE_COUNT) %
				circumference_q8;
		int32_t distance_q8 = (int32_t)animation_pixel * 256L - head_q8;
		if (distance_q8 < 0L) {
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

static void LED_RenderEmergency(uint32_t now_ms) {
	for (uint16_t i = 0U; i < LAUNCHER_MAIN_LED_COUNT; i++) {
		setPixelPA6(i, LED_EmergencyIntensity(i,
				LAUNCHER_MAIN_LED_COUNT, now_ms, true), 0U, 0U);
	}
	for (uint16_t i = MIDDLE_CHAIN_START; i < PA6_LED_NUM; i++) {
		setPixelPA6(i, LED_EmergencyIntensity(
				(uint16_t)(i - MIDDLE_CHAIN_START),
				(uint16_t)(PA6_LED_NUM - MIDDLE_CHAIN_START),
				now_ms, false), 0U, 0U);
	}
	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		setPixelPA7(i, LED_EmergencyIntensity(i,
				PA7_LED_NUM, now_ms, true), 0U, 0U);
	}
}

/* Symmetric waves use chassis pixel 15 as the front and 34 as the rear. */
static void LED_RenderChassisWaves(uint32_t now_ms, uint32_t period_ms,
		uint8_t wave_count, bool from_front,
		uint8_t color_r, uint8_t color_g, uint8_t color_b,
		uint8_t base_intensity) {
	const uint16_t max_distance = PA7_LED_NUM / 2U;
	const int32_t travel_levels = (int32_t)max_distance + 8L;
	const uint32_t elapsed_ms = now_ms % period_ms;

	for (uint16_t pixel = 0U; pixel < PA7_LED_NUM; pixel++) {
		const uint16_t clockwise_distance = (uint16_t)((pixel + PA7_LED_NUM -
				CHASSIS_FRONT_CENTER) % PA7_LED_NUM);
		const uint16_t counterclockwise_distance =
				(uint16_t)(PA7_LED_NUM - clockwise_distance);
		const uint16_t symmetric_distance =
				(clockwise_distance < counterclockwise_distance) ?
				clockwise_distance : counterclockwise_distance;
		int32_t nearest_wave_distance_q8 = travel_levels * 256L;

		for (uint8_t wave = 0U; wave < wave_count; wave++) {
			const uint32_t wave_elapsed_ms = (elapsed_ms +
					(uint32_t)wave * period_ms / wave_count) % period_ms;
			const int32_t travelled_q8 = (int32_t)(wave_elapsed_ms *
					(uint32_t)travel_levels * 256UL / period_ms);
			const int32_t head_q8 = from_front ?
					(travelled_q8 - CHASSIS_GLOW_RADIUS_Q8) :
					((int32_t)max_distance * 256L +
					 CHASSIS_GLOW_RADIUS_Q8 - travelled_q8);
			int32_t distance_q8 =
					(int32_t)symmetric_distance * 256L - head_q8;
			if (distance_q8 < 0L) {
				distance_q8 = -distance_q8;
			}
			if (distance_q8 < nearest_wave_distance_q8) {
				nearest_wave_distance_q8 = distance_q8;
			}
		}

		uint16_t intensity = base_intensity;
		if (nearest_wave_distance_q8 < CHASSIS_GLOW_RADIUS_Q8) {
			const uint32_t strength =
					(uint32_t)(CHASSIS_GLOW_RADIUS_Q8 - nearest_wave_distance_q8);
			const uint32_t radius_squared =
					(uint32_t)CHASSIS_GLOW_RADIUS_Q8 * CHASSIS_GLOW_RADIUS_Q8;
			intensity += (uint16_t)(strength * strength *
					(CHASSIS_PEAK_INTENSITY - base_intensity) /
					radius_squared);
		}

		setPixelPA7(pixel,
				(uint8_t)((uint16_t)color_r * intensity / 255U),
				(uint8_t)((uint16_t)color_g * intensity / 255U),
				(uint8_t)((uint16_t)color_b * intensity / 255U));
	}
}

/* Belt levels 1..4: 4/6/8/10 center levels and 3/5/7/9 side levels. */
static void LED_RenderBeltLauncher(uint8_t status, uint32_t now_ms) {
	const uint8_t belt_level = status & LED_STATUS_BELT_MASK;
	static const uint8_t speed_q8_per_ms[4] = { 1U, 2U, 4U, 7U };
	uint8_t center_intensity[LAUNCHER_LEVEL_COUNT] = {0U};

	if ((belt_level == 0U) || (belt_level > 4U)) {
		belt_wave_active = false;
		belt_wave_last_update_ms = now_ms;
		return;
	}

	const uint16_t center_lit_count = (uint16_t)(2U + 2U * belt_level);
	const uint16_t side_lit_count = (uint16_t)(1U + 2U * belt_level);
	const int32_t radius_q8 = 640L;
	if (!belt_wave_active) {
		belt_wave_active = true;
		belt_wave_head_q8 = -512L;
		belt_wave_last_update_ms = now_ms;
	} else {
		const uint32_t elapsed_ms = now_ms - belt_wave_last_update_ms;
		belt_wave_last_update_ms = now_ms;
		belt_wave_head_q8 +=
				(int32_t)(elapsed_ms * speed_q8_per_ms[belt_level - 1U]);
	}
	if (belt_wave_head_q8 > (int32_t)(center_lit_count + 2U) * 256L) {
		belt_wave_head_q8 = -512L;
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		uint16_t intensity = 0U;
		if (level < center_lit_count) {
			int32_t distance_q8 = (int32_t)level * 256L - belt_wave_head_q8;
			if (distance_q8 < 0L) distance_q8 = -distance_q8;
			intensity = 6U;
			if (distance_q8 < radius_q8) {
				const uint32_t strength = (uint32_t)(radius_q8 - distance_q8);
				intensity += (uint16_t)(214U * strength * strength /
						((uint32_t)radius_q8 * (uint32_t)radius_q8));
			}
		}
		center_intensity[level] = (uint8_t)intensity;
		setPixelPA6(level, 0U, (uint8_t)intensity,
				(uint8_t)(intensity * 90U / 255U));
		setPixelPA6(LAUNCHER_MAIN_LED_COUNT - 1U - level,
				0U, (uint8_t)intensity,
				(uint8_t)(intensity * 90U / 255U));
	}

	for (uint16_t level = 0U; level < LAUNCHER_SIDE_LED_COUNT; level++) {
		uint16_t intensity = 0U;
		if (level < side_lit_count) {
			int32_t distance_q8 = (int32_t)level * 256L - belt_wave_head_q8;
			if (distance_q8 < 0L) distance_q8 = -distance_q8;
			intensity = 6U;
			if (distance_q8 < radius_q8) {
				const uint32_t strength = (uint32_t)(radius_q8 - distance_q8);
				intensity += (uint16_t)(214U * strength * strength /
						((uint32_t)radius_q8 * (uint32_t)radius_q8));
			}
		}
		setPixelPA6(LAUNCHER_MAIN_LED_COUNT + level,
				0U, (uint8_t)intensity,
				(uint8_t)(intensity * 90U / 255U));
		setPixelPA2(level, 0U, (uint8_t)intensity,
				(uint8_t)(intensity * 90U / 255U));
	}

	for (uint16_t i = 0U; i < MIDDLE_FRONT_LED_COUNT; i++) {
		const uint8_t intensity = center_intensity[LAUNCHER_LEVEL_COUNT - 1U];
		setPixelPA6(LAUNCHER_MAIN_LED_COUNT + LAUNCHER_SIDE_LED_COUNT + i,
				0U, intensity,
				(uint8_t)((uint16_t)intensity * 90U / 255U));
	}
}

/* ARM_OPEN: carry the Emerald belt wave to the tip, then fill back to root. */
static bool LED_RenderArmOpenLauncher(uint32_t now_ms, bool entering,
		uint8_t speed_divisor) {
	const int32_t tip_q8 = (LAUNCHER_LEVEL_COUNT - 1U) * 256L;
	const int32_t wave_radius_q8 = 640L;
	if (speed_divisor == 0U) speed_divisor = 1U;
	if (entering) {
		arm_open_phase = 0U;
		arm_open_head_q8 = belt_wave_active ? belt_wave_head_q8 : 0L;
		if (arm_open_head_q8 < 0L) arm_open_head_q8 = 0L;
		if (arm_open_head_q8 > tip_q8) arm_open_head_q8 = tip_q8;
		arm_open_fill_q8 = tip_q8;
		arm_open_last_update_ms = now_ms;
		arm_open_head_speed_remainder = 0U;
		arm_open_fill_speed_remainder = 0U;
		belt_wave_active = false;
	}

	const uint32_t elapsed_ms = now_ms - arm_open_last_update_ms;
	arm_open_last_update_ms = now_ms;
	if (arm_open_phase == 0U) {
		const uint32_t scaled_step =
				elapsed_ms * ARM_OPEN_HEAD_SPEED_Q8_PER_MS +
				arm_open_head_speed_remainder;
		arm_open_head_q8 += (int32_t)(scaled_step / speed_divisor);
		arm_open_head_speed_remainder = scaled_step % speed_divisor;
		if (arm_open_head_q8 >= tip_q8) {
			arm_open_head_q8 = tip_q8;
			arm_open_phase = 1U;
			arm_open_fill_q8 = tip_q8;
			arm_open_fill_speed_remainder = 0U;
		}
	} else if (arm_open_phase == 1U) {
		const uint32_t scaled_step =
				elapsed_ms * ARM_OPEN_FILL_SPEED_Q8_PER_MS +
				arm_open_fill_speed_remainder;
		arm_open_fill_q8 -= (int32_t)(scaled_step / speed_divisor);
		arm_open_fill_speed_remainder = scaled_step % speed_divisor;
		if (arm_open_fill_q8 <= 0L) {
			arm_open_fill_q8 = 0L;
			arm_open_phase = 2U;
		}
	}

	for (uint16_t level = 0U; level < LAUNCHER_LEVEL_COUNT; level++) {
		const int32_t position_q8 = (int32_t)level * 256L;
		uint16_t intensity = 6U;
		if (arm_open_phase == 0U) {
			int32_t distance_q8 = position_q8 - arm_open_head_q8;
			if (distance_q8 < 0L) distance_q8 = -distance_q8;
			if (distance_q8 < wave_radius_q8) {
				const uint32_t strength =
						(uint32_t)(wave_radius_q8 - distance_q8);
				intensity += (uint16_t)(214U * strength * strength /
						((uint32_t)wave_radius_q8 * wave_radius_q8));
			}
		} else if (position_q8 >= arm_open_fill_q8) {
			intensity = 220U;
		} else {
			const int32_t edge_distance_q8 = arm_open_fill_q8 - position_q8;
			if (edge_distance_q8 < 512L) {
				intensity += (uint16_t)(214U *
						(uint32_t)(512L - edge_distance_q8) / 512U);
			}
		}
		LED_SetLauncherLevel(level,
				0U, (uint8_t)intensity,
				(uint8_t)(intensity * 90U / 255U));
	}
	return arm_open_phase == 2U;
}

static void LED_CaptureFiringBackground(uint32_t now_ms) {
	firing_start_ms = now_ms;
	for (uint16_t i = 0U; i < PA6_LED_NUM; i++) {
		getPixelPA6(i, &firing_launcher_background[i][0],
				&firing_launcher_background[i][1],
				&firing_launcher_background[i][2]);
	}
	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		getPixelPA7(i, &firing_chassis_background[i][0],
				&firing_chassis_background[i][1],
				&firing_chassis_background[i][2]);
	}
}

static uint16_t LED_FiringWaveStrength(int32_t distance_q8,
		int32_t full_radius_q8, int32_t outer_radius_q8) {
	if (distance_q8 <= full_radius_q8) return 255U;
	if (distance_q8 >= outer_radius_q8) return 0U;
	const uint32_t fade_width = (uint32_t)(outer_radius_q8 - full_radius_q8);
	const uint32_t strength = (uint32_t)(outer_radius_q8 - distance_q8);
	return (uint16_t)(strength * strength * 255U /
			(fade_width * fade_width));
}

static uint16_t LED_LauncherPhysicalLevel(uint16_t pixel) {
	if (pixel < 10U) return pixel;
	if (pixel < 20U) return (uint16_t)(19U - pixel);
	if (pixel < 29U) return (uint16_t)(pixel - 20U);
	if (pixel < 36U) return 9U;
	if (pixel >= 45U) return (uint16_t)(47U - pixel);
	return (uint16_t)(44U - pixel);
}

static void LED_BlendFiringLauncher(uint16_t pixel, uint16_t wave_strength) {
	const uint16_t keep = 255U - wave_strength;
	const uint8_t bg_r = firing_launcher_background[pixel][0];
	const uint8_t bg_g = firing_launcher_background[pixel][1];
	const uint8_t bg_b = firing_launcher_background[pixel][2];
	setPixelPA6(pixel,
			(uint8_t)(((uint16_t)bg_r * keep +
					FIRING_COLOR_R * wave_strength) / 255U),
			(uint8_t)(((uint16_t)bg_g * keep +
					FIRING_COLOR_G * wave_strength) / 255U),
			(uint8_t)(((uint16_t)bg_b * keep +
					FIRING_COLOR_B * wave_strength) / 255U));
}

static void LED_RenderFiring(uint32_t elapsed_ms, uint32_t wave_ms) {
	const int32_t launcher_travel_q8 =
			(LAUNCHER_LEVEL_COUNT - 1U) * 256L +
			2L * FIRING_LAUNCHER_OUTER_RADIUS_Q8;
	int32_t launcher_head_q8 = -FIRING_LAUNCHER_OUTER_RADIUS_Q8;
	if (elapsed_ms < wave_ms) {
		launcher_head_q8 = (LAUNCHER_LEVEL_COUNT - 1U) * 256L +
				FIRING_LAUNCHER_OUTER_RADIUS_Q8 -
				(int32_t)(elapsed_ms * (uint32_t)launcher_travel_q8 / wave_ms);
	}
	for (uint16_t pixel = 0U; pixel < PA6_LED_NUM; pixel++) {
		int32_t distance_q8 = (int32_t)LED_LauncherPhysicalLevel(pixel) *
				256L - launcher_head_q8;
		if (distance_q8 < 0L) distance_q8 = -distance_q8;
		LED_BlendFiringLauncher(pixel, LED_FiringWaveStrength(distance_q8,
				FIRING_LAUNCHER_FULL_RADIUS_Q8,
				FIRING_LAUNCHER_OUTER_RADIUS_Q8));
	}

	const uint16_t max_distance = PA7_LED_NUM / 2U;
	const int32_t chassis_travel_q8 = (int32_t)max_distance * 256L +
			2L * FIRING_CHASSIS_OUTER_RADIUS_Q8;
	int32_t chassis_head_q8 = -FIRING_CHASSIS_OUTER_RADIUS_Q8;
	if (elapsed_ms < wave_ms) {
		chassis_head_q8 = -FIRING_CHASSIS_OUTER_RADIUS_Q8 +
				(int32_t)(elapsed_ms * (uint32_t)chassis_travel_q8 / wave_ms);
	}
	for (uint16_t pixel = 0U; pixel < PA7_LED_NUM; pixel++) {
		const uint16_t clockwise = (uint16_t)((pixel + PA7_LED_NUM -
				CHASSIS_FRONT_CENTER) % PA7_LED_NUM);
		const uint16_t counterclockwise = (uint16_t)(PA7_LED_NUM - clockwise);
		const uint16_t position = (clockwise < counterclockwise) ?
				clockwise : counterclockwise;
		int32_t distance_q8 = (int32_t)position * 256L - chassis_head_q8;
		if (distance_q8 < 0L) distance_q8 = -distance_q8;
		const uint16_t wave_strength = LED_FiringWaveStrength(distance_q8,
				FIRING_CHASSIS_FULL_RADIUS_Q8,
				FIRING_CHASSIS_OUTER_RADIUS_Q8);
		const uint16_t keep = 255U - wave_strength;
		const uint8_t bg_r = firing_chassis_background[pixel][0];
		const uint8_t bg_g = firing_chassis_background[pixel][1];
		const uint8_t bg_b = firing_chassis_background[pixel][2];
		setPixelPA7(pixel,
				(uint8_t)(((uint16_t)bg_r * keep +
						FIRING_COLOR_R * wave_strength) / 255U),
				(uint8_t)(((uint16_t)bg_g * keep +
						FIRING_COLOR_G * wave_strength) / 255U),
				(uint8_t)(((uint16_t)bg_b * keep +
						FIRING_COLOR_B * wave_strength) / 255U));
	}
}

static void LED_RenderAutoFiringRanges(uint32_t elapsed_ms,
		uint32_t wave_ms) {
	const int32_t travel_q8 =
			(LAUNCHER_LEVEL_COUNT - 1U) * 256L +
			2L * FIRING_LAUNCHER_OUTER_RADIUS_Q8;
	int32_t head_q8 = -FIRING_LAUNCHER_OUTER_RADIUS_Q8;
	if (elapsed_ms < wave_ms) {
		head_q8 = (LAUNCHER_LEVEL_COUNT - 1U) * 256L +
				FIRING_LAUNCHER_OUTER_RADIUS_Q8 -
				(int32_t)(elapsed_ms * (uint32_t)travel_q8 / wave_ms);
	}

	for (uint16_t offset = 0U; offset < GRID_CELL_COUNT; offset++) {
		const uint16_t pixels[2] = {
			(uint16_t)(20U + offset),
			(uint16_t)(36U + offset)
		};
		for (uint16_t side = 0U; side < 2U; side++) {
			int32_t distance_q8 =
					(int32_t)LED_LauncherPhysicalLevel(pixels[side]) *
					256L - head_q8;
			if (distance_q8 < 0L) distance_q8 = -distance_q8;
			LED_BlendFiringLauncher(pixels[side],
					LED_FiringWaveStrength(distance_q8,
							FIRING_LAUNCHER_FULL_RADIUS_Q8,
							FIRING_LAUNCHER_OUTER_RADIUS_Q8));
		}
	}
}

static uint32_t LED_AutoFiringPhaseMs(uint32_t elapsed_ms) {
	/* Linearly ramp frequency from 1/932 ms to 1/233 ms. Integrating the
	 * frequency keeps the white wave position continuous while it accelerates. */
	const uint32_t denominator = 2U * AUTO_FIRING_EFFECT_MS *
			AUTO_FIRING_INITIAL_WAVE_MS;
	const uint32_t phase_numerator =
			2U * AUTO_FIRING_EFFECT_MS * elapsed_ms +
			3U * elapsed_ms * elapsed_ms;
	return (phase_numerator % denominator) * FIRING_WAVE_MS / denominator;
}

/* Physical LED numbers in the wiring drawing are 1-based. Grid state indices
 * are row-major: top-left index 0 through bottom-right index 8. */
static const uint8_t grid_led_index[GRID_CELL_COUNT] = {
	2U, 45U, 17U,
	1U, 46U, 18U,
	0U, 47U, 19U
};

/* Non-automatic effects for added LEDs 46..48 continue to mirror physical
 * LEDs 3, 2 and 1; this mapping must not follow the automatic grid shift. */
static const uint8_t normal_added_led_source[3] = {2U, 1U, 0U};

static void LED_CopyNormalPatternToAddedGridLeds(void) {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	for (uint16_t row = 0U; row < 3U; row++) {
		getPixelPA6(normal_added_led_source[row], &red, &green, &blue);
		setPixelPA6(grid_led_index[row * 3U + 1U], red, green, blue);
	}
}

static void LED_TintRangeAutoGradient(uint16_t first, uint16_t count,
		bool reverse_gradient, uint8_t mode) {
	for (uint16_t offset = 0U; offset < count; offset++) {
		uint8_t source_red;
		uint8_t source_green;
		uint8_t source_blue;
		getPixelPA6((uint16_t)(first + offset),
				&source_red, &source_green, &source_blue);

		uint8_t intensity = source_red;
		if (mode == LED_MODE_FIRING) {
			/* The white wave adds red to the green preparation background.
			 * Red therefore isolates the moving white component cleanly. */
			intensity = source_red;
		} else {
			if (source_green > intensity) intensity = source_green;
			if (source_blue > intensity) intensity = source_blue;
			if ((mode == LED_MODE_LOADING) && (intensity < 32U)) {
				intensity = 32U;
			}
		}

		const uint16_t position = reverse_gradient ?
				(uint16_t)(count - 1U - offset) : offset;
		const uint16_t x = (count > 1U) ?
				(uint16_t)(position * 255U / (count - 1U)) : 0U;
		/* Smoothstep avoids visible color steps between adjacent LEDs. */
		const uint16_t blend = (uint16_t)((uint32_t)x * x *
				(765U - 2U * x) / (255U * 255U));
		uint16_t gradient_red = 85U + 115U * blend / 255U;
		uint16_t gradient_green = 18U + 22U * blend / 255U;
		uint16_t gradient_blue = 255U;
		if (mode == LED_MODE_LOADING) {
			/* Preparation: deep green to bright Emerald. */
			gradient_red = 0U;
			gradient_green = 150U + 105U * blend / 255U;
			gradient_blue = 80U - 50U * blend / 255U;
		} else if (mode == LED_MODE_FIRING) {
			/* Firing keeps the moving intensity gradient neutral white. */
			gradient_red = 255U;
			gradient_green = 255U;
			gradient_blue = 255U;
		}

		setPixelPA6((uint16_t)(first + offset),
				(uint8_t)(gradient_red * intensity / 255U),
				(uint8_t)(gradient_green * intensity / 255U),
				(uint8_t)(gradient_blue * intensity / 255U));
	}
}

static void LED_ApplyAutoRangeGradient(uint8_t mode) {
	/* Keep the existing wave position, brightness and belt lit count. The two
	 * physical ranges run in opposite directions for the same logical level. */
	LED_TintRangeAutoGradient(20U, GRID_CELL_COUNT, false, mode);
	LED_TintRangeAutoGradient(36U, GRID_CELL_COUNT, true, mode);
}

static void LED_OverlayGame2Grid(uint32_t grid_states, uint32_t now_ms,
		uint8_t mode) {
	const bool target_on =
			((now_ms / GRID_TARGET_BLINK_MS) & 1U) == 0U;
	for (uint16_t index = 0U; index < GRID_CELL_COUNT; index++) {
		const uint8_t state = (uint8_t)((grid_states >> (index * 2U)) & 0x03U);
		uint8_t red = 0U;
		uint8_t green = 180U;
		uint8_t blue = 255U;
		if (state == GRID_STATE_TARGET) {
			green = target_on ? 255U : 0U;
			blue = target_on ? 255U : 0U;
		} else if (state == GRID_STATE_FALLEN) {
			red = 255U;
			green = 90U;
			blue = 0U;
		} else if (state != GRID_STATE_STANDING) {
			red = 255U;
			green = 0U;
			blue = 255U;
		}
		setPixelPA6(grid_led_index[index], red, green, blue);
	}
	LED_ApplyAutoRangeGradient(mode);
}

static void LED_RenderGame2Searching(uint32_t now_ms, uint8_t status) {
	const uint16_t head = (uint16_t)((now_ms % 1200U) *
			PA7_LED_NUM / 1200U);
	for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
		const uint8_t glow = LED_GlowIntensity(LED_CircularDistance(i, head));
		setPixelPA7(i, 0U, glow, (uint8_t)(glow * 2U / 5U));
	}
	LED_RenderLauncherWave(now_ms, 1200U, true, 0U, 255U, 90U);

	/* Belt level 4: mark only the launcher tip row (folded-strip pair) red. */
	if ((status & LED_STATUS_BELT_MASK) == 4U) {
		setPixelPA6(9U, 255U, 0U, 0U);
		setPixelPA6(10U, 255U, 0U, 0U);
	}
}

/* Render modes 0..15 from LED_BIT_ASSIGNMENT.md. */
void LED_Effects_Init(void) {
	LED_Init();
	clear();
	show();
}

void LED_Effects_Render(uint8_t mode, uint8_t status, uint32_t grid_states) {
	const uint32_t now_ms = HAL_GetTick();
	static uint8_t previous_mode = 0xFFU;
	const uint8_t mode_before_change = previous_mode;
	const bool mode_changed = mode != previous_mode;

	if (mode_changed) {
		if ((mode == LED_MODE_FIRING) || (mode == LED_MODE_SLOW_FIRING)) {
			LED_CaptureFiringBackground(now_ms);
		}
		previous_mode = mode;
	}

	/* Game2 modes keep the normal manual animation; only grid LEDs are overlaid. */
	debug_led_game2_search_active =
			(mode == LED_MODE_GAME2_SEARCHING) ? 1U : 0U;

	switch (mode) {
	case LED_MODE_STARTUP:
	case LED_MODE_READY:
	{
		const uint8_t pulse = LED_SmoothPulse(now_ms, 2400U);
		const uint8_t green = (uint8_t)(145U + (uint16_t)pulse * 30U / 255U);
		const uint8_t blue = (uint8_t)(185U + (uint16_t)pulse * 35U / 255U);
		LED_SetChassisAll(0U, green, blue);
		LED_SetLauncherAll(0U, green, blue);
		break;
	}
	case LED_MODE_EMERGENCY_STOP:
		LED_RenderEmergency(now_ms);
		break;
	case LED_MODE_ARM_OPEN:
		LED_RenderChassisWaves(now_ms, 900U, 1U, false,
				0U, 170U, 255U, 0U);
		(void)LED_RenderArmOpenLauncher(now_ms, mode_changed, 1U);
		break;
	case LED_MODE_LOADING:
	{
		LED_RenderChassisWaves(now_ms, 1000U, 2U, true,
				0U, 255U, 90U, 0U);
		(void)LED_RenderArmOpenLauncher(now_ms,
				mode_changed && (mode_before_change != LED_MODE_ARM_OPEN), 9U);
		break;
	}
	case LED_MODE_FIRING:
	{
		const uint32_t firing_elapsed_ms = now_ms - firing_start_ms;
		if (firing_elapsed_ms >= FIRING_START_DELAY_MS) {
			const uint32_t wave_elapsed_ms =
					firing_elapsed_ms - FIRING_START_DELAY_MS;
			if ((status & LED_STATUS_GAME2_ENABLED) != 0U) {
				const uint32_t phase_ms =
						(wave_elapsed_ms < AUTO_FIRING_EFFECT_MS) ?
						LED_AutoFiringPhaseMs(wave_elapsed_ms) :
						FIRING_WAVE_MS;
				/* Automatic white wave is limited to PB4 LEDs 21..29 and
				 * 37..45; all other PB4/PB5 pixels keep their background. */
				LED_RenderAutoFiringRanges(phase_ms, FIRING_WAVE_MS);
			} else {
				LED_RenderFiring(wave_elapsed_ms, FIRING_WAVE_MS);
			}
		}
		break;
	}
	case LED_MODE_RETURNING:
		LED_RenderChassisWaves(now_ms, 900U, 1U, true,
				0U, 60U, 255U, 0U);
		LED_RenderLauncherWave(now_ms, 900U, false, 0U, 64U, 255U);
		break;
	case LED_MODE_GAME2_SEARCHING:
		LED_RenderGame2Searching(now_ms, status);
		break;
	case LED_MODE_GAME2_ALIGNING:
	{
		const uint8_t pulse = LED_TriangleWave(now_ms, 500U);
		for (uint16_t i = 0U; i < PA7_LED_NUM; i++) {
			const uint16_t distance = LED_CircularDistance(i, CHASSIS_FRONT_CENTER);
			const uint8_t intensity = (distance <= 3U) ?
					(uint8_t)((uint16_t)pulse * (4U - distance) / 4U) : 0U;
			setPixelPA7(i, 0U, intensity, intensity);
		}
		LED_SetLauncherAll(0U, pulse, pulse);
		break;
	}
	case LED_MODE_ERROR:
	default:
	{
		const uint8_t red = ((now_ms / 100U) & 1U) ? 255U : 0U;
		LED_SetChassisAll(red, 0U, 0U);
		LED_SetLauncherAll(red, 0U, 0U);
		break;
	}
	case LED_MODE_ARM_DRIBBLE:
	{
		const uint8_t pulse = LED_SmoothPulse(now_ms, 1800U);
		const uint8_t blue = (uint8_t)(110U + (uint16_t)pulse * 70U / 255U);
		LED_SetChassisAll(0U, (uint8_t)(blue / 12U), blue);
		LED_SetLauncherAll(0U, 255U, 90U);
		break;
	}
	case LED_MODE_SLOW_FIRING:
		LED_RenderFiring((now_ms - firing_start_ms) % 1200U,
				SLOW_FIRING_WAVE_MS);
		break;
	case LED_MODE_ARM_FEED:
		LED_RenderChassisWaves(now_ms, 1050U, 2U, true,
				0U, 255U, 90U, 6U);
		LED_RenderLauncherWave(now_ms, 650U, true, 0U, 255U, 90U);
		break;
	case LED_MODE_ARM_RECEIVE:
		LED_RenderChassisWaves(now_ms, 1050U, 2U, false,
				0U, 150U, 255U, 6U);
		LED_RenderLauncherWave(now_ms, 800U, false, 0U, 160U, 255U);
		break;
	case LED_MODE_ARM_HOME:
	{
		const uint8_t pulse = LED_SmoothPulse(now_ms, 2600U);
		const uint8_t green = (uint8_t)(55U + (uint16_t)pulse * 30U / 255U);
		const uint8_t blue = (uint8_t)(80U + (uint16_t)pulse * 35U / 255U);
		LED_SetChassisAll(0U, green, blue);
		LED_SetLauncherAll(0U, green, blue);
		break;
	}
	case LED_MODE_BELT_SPINUP:
	{
		const uint8_t pulse = LED_SmoothPulse(now_ms, 600U);
		LED_SetChassisAll(0U, pulse, (uint8_t)(pulse * 90U / 255U));
		LED_RenderLauncherWave(now_ms, 450U, true, 0U, 255U, 90U);
		break;
	}
	case LED_MODE_BELT_OFFSET_MINUS_3:
	case LED_MODE_BELT_OFFSET_MINUS_2:
	case LED_MODE_BELT_OFFSET_MINUS_1:
	case LED_MODE_BELT_OFFSET_ZERO:
	case LED_MODE_BELT_OFFSET_PLUS_1:
	case LED_MODE_BELT_OFFSET_PLUS_2:
	case LED_MODE_BELT_OFFSET_PLUS_3:
		/* Keep the active Emerald belt wave and add the offset indication. */
		LED_RenderBeltLauncher(status, now_ms);
		LED_OverlayBeltOffset(mode,
				(status & LED_STATUS_GAME2_ENABLED) != 0U);
		break;
	}

	/* Auxiliary states never override safety or firing/loading displays. */
	if ((mode != LED_MODE_STARTUP) &&
			(mode != LED_MODE_EMERGENCY_STOP) &&
			(mode != LED_MODE_LOADING) &&
			(mode != LED_MODE_FIRING) &&
			(mode != LED_MODE_ERROR) &&
			(mode != LED_MODE_SLOW_FIRING) && (mode <= LED_MODE_BELT_SPINUP)) {
		const bool dribble =
				(status & LED_STATUS_DRIBBLE_ENABLED) != 0U;
		const bool reversed =
				(status & LED_STATUS_DRIVE_REVERSED) != 0U;
		const bool roller_reverse =
				(status & LED_STATUS_ROLLER_REVERSE) != 0U;
		if (reversed) {
			if (dribble) {
				/* Reversed dribbling uses Gold gradients on both chains. */
				LED_RenderDriveLauncherGradient(now_ms, false, false);
				LED_RenderChassisWaves(now_ms, 1200U, 3U, true,
						255U, 215U, 0U, CHASSIS_BASE_INTENSITY);
			} else {
				/* Upper wave is opposite to normal; chassis stays solid Gold. */
				LED_RenderDriveLauncherGradient(now_ms, false, false);
				LED_SetChassisAll(255U, 215U, 0U);
			}
		} else {
			/* Normal drive keeps a blue gradient on the upper LED chain. */
			LED_RenderDriveLauncherGradient(now_ms, true, true);
			if (dribble) {
				LED_RenderChassisWaves(now_ms, 1200U, 3U,
						roller_reverse, 0U, 22U, 255U,
						CHASSIS_BASE_INTENSITY);
			} else {
				LED_SetChassisAll(0U, 22U, 255U);
			}
		}

		if (((mode == LED_MODE_READY) ||
				(mode == LED_MODE_ARM_DRIBBLE) ||
				(mode == LED_MODE_ARM_HOME))) {
			/* Active belt Emerald overrides the upper reversed Gold display. */
			LED_RenderBeltLauncher(status, now_ms);
		}
	}
	if (((status & LED_STATUS_GAME2_ENABLED) != 0U) &&
			(mode != LED_MODE_STARTUP) &&
			(mode != LED_MODE_EMERGENCY_STOP) && (mode != LED_MODE_ERROR)) {
		LED_OverlayGame2Grid(grid_states, now_ms, mode);
	} else if (((status & LED_STATUS_GAME2_ENABLED) == 0U) &&
			(mode != LED_MODE_STARTUP) &&
			(mode != LED_MODE_EMERGENCY_STOP) && (mode != LED_MODE_ERROR)) {
		LED_CopyNormalPatternToAddedGridLeds();
	}
	LED_ApplyImuDisconnectedWarning(now_ms);
	show();
}
