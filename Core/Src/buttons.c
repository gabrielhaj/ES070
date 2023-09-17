/*
 * buttons.c
 *
 *  Created on: Sep 16, 2023
 *      Author: gabriel
 */

#include "main.h"
#include "buttons.h"

buttons xReadButtons() {
	buttons xButtons = {0};
	xButtons.leftBt = HAL_GPIO_ReadPin(LeftBt_GPIO_Port,LeftBt_Pin);
	xButtons.rightBt = HAL_GPIO_ReadPin(RightBt_GPIO_Port,RightBt_Pin);
	xButtons.upBt = HAL_GPIO_ReadPin(UpBt_GPIO_Port,UpBt_Pin);
	xButtons.downBt = HAL_GPIO_ReadPin(DownBt_GPIO_Port,DownBt_Pin);
	xButtons.enterBt = HAL_GPIO_ReadPin(EnterBt_GPIO_Port,EnterBt_Pin);
	return xButtons;
}
