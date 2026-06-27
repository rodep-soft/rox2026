/*
 * limit_switch.h
 *
 *  Created on: 2026/06/24
 *      Author: aeijn
 */

#ifndef INC_LIMIT_SWITCH_H_
#define INC_LIMIT_SWITCH_H_

#include "main.h"

// リミットスイッチの状態を監視し、変化があればCANで送信する関数
void LimitSwitch_UpdateAndSend(CAN_HandleTypeDef *hcan);

#endif /* INC_LIMIT_SWITCH_H_ */
