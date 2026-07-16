#include "LED_lite.h"
#include "can.h"
#include <stdbool.h>
#include <string.h>

/* main.c で定義されているTIM15のハンドルを外部参照 */
extern TIM_HandleTypeDef htim17;

//LEDテープ制御用ファイル
//sk6812用

//送信間の間隔保持用バッファサイズ
#define RESET_SLOTS 200

//bitの0,1を送るための時間管理のカウント
#define LED_LOW  19
#define LED_HIGH 38
#define LED_COMMAND_QUEUE_CAPACITY 32U

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} RGB_t;

typedef struct {
	uint8_t index;
	uint8_t r;
	uint8_t g;
	uint8_t b;
} LED_Command;

RGB_t ledBuffer[LED_NUM];
uint16_t pwmData[(LED_NUM * 24) + (RESET_SLOTS * 2)];
bool ledChanged = true;

//DMAが複数呼ばれたときに，競合しないように制御する変数
volatile bool dmaBusy = false;
static LED_Command ledCommandQueue[LED_COMMAND_QUEUE_CAPACITY];
static volatile uint8_t ledCommandReadIndex = 0U;
static volatile uint8_t ledCommandWriteIndex = 0U;
static volatile uint32_t ledCommandOverflowCount = 0U;

static void LED_RxCallback(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[]) {
	uint8_t writeIndex;
	uint8_t nextIndex;

	if ((rxHeader->DLC < 4U) || (rxData[0] >= LED_NUM))
		return;

	writeIndex = ledCommandWriteIndex;
	nextIndex = (uint8_t) ((writeIndex + 1U) % LED_COMMAND_QUEUE_CAPACITY);
	if (nextIndex == ledCommandReadIndex) {
		ledCommandOverflowCount++;
		return;
	}

	ledCommandQueue[writeIndex].index = rxData[0];
	ledCommandQueue[writeIndex].r = rxData[1];
	ledCommandQueue[writeIndex].g = rxData[2];
	ledCommandQueue[writeIndex].b = rxData[3];
	__DMB();
	ledCommandWriteIndex = nextIndex;
}

HAL_StatusTypeDef LED_CanInit(void) {
	return canAddRxCallback(0x210U, LED_RxCallback);
}

void LED_Process(void) {
	while (ledCommandReadIndex != ledCommandWriteIndex) {
		uint8_t readIndex = ledCommandReadIndex;
		LED_Command command = ledCommandQueue[readIndex];

		__DMB();
		ledCommandReadIndex = (uint8_t) ((readIndex + 1U)
				% LED_COMMAND_QUEUE_CAPACITY);
		setPixel(command.index, command.r, command.g, command.b);
	}

	show();
}

void appendByte(uint8_t value, uint32_t *index) {
	for (int i = 7; i >= 0; i--) {
		if (value & (1 << i)) {
			pwmData[(*index)++] = LED_HIGH;
		} else {
			pwmData[(*index)++] = LED_LOW;
		}
	}
}

void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
	if (index >= LED_NUM)
		return;
	// 既存値と比較
	if (ledBuffer[index].r == r && ledBuffer[index].g == g
			&& ledBuffer[index].b == b) { //阿多愛に変化がない場合は何もせずに終了
		return;
	} else {
		ledBuffer[index].r = r;
		ledBuffer[index].g = g;
		ledBuffer[index].b = b;
		ledChanged = true;
	}
}

RGB_t readPixel(uint16_t index) {
	return ledBuffer[index];
}

void show(void) {
	if (!ledChanged)
		return;
	if (dmaBusy)
		return;
	dmaBusy = true;

	memset(pwmData, 0, sizeof(pwmData));

	uint32_t pwmIndex = 0;

	for (uint16_t i = 0; i < RESET_SLOTS; i++) {
		pwmData[pwmIndex++] = 0;
	}

	for (uint16_t led = 0; led < LED_NUM; led++) {
		// GRBの順番
		appendByte(ledBuffer[led].g, &pwmIndex);
		appendByte(ledBuffer[led].r, &pwmIndex);
		appendByte(ledBuffer[led].b, &pwmIndex);
	}
	// Reset
	while (pwmIndex < (LED_NUM * 24U) + (RESET_SLOTS * 2U)) {
		pwmData[pwmIndex++] = 0;
	}

	ledChanged = false;
	if (HAL_TIM_PWM_Start_DMA(&htim17,
	TIM_CHANNEL_1, (uint32_t*) pwmData, pwmIndex) != HAL_OK) {
		dmaBusy = false;
		ledChanged = true;
	}
}

//LEDの色管理用配列の初期化
void clear(void) {
	for (uint16_t i = 0; i < LED_NUM; i++) {
		ledBuffer[i].r = 0;
		ledBuffer[i].g = 0;
		ledBuffer[i].b = 0;
	}
	ledChanged = true;
}

/**
 * @brief DMAの転送完了コールバック
 * 転送が終わったら自動的にPWM出力を停止して次の送信に備える
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM17) {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
		HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
		dmaBusy = false;
	}
}
