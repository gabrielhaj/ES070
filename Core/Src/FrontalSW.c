/*
 * buttons.c
 *
 *  Created on: Sep 16, 2023
 *      Author: gabriel
 */

#include "main.h"
#include "FrontalSW.h"

GPIO_PinState SW = 0;

GPIO_PinState SWRead() {
	GPIO_PinState FrontalSW = 0;
	FrontalSW = HAL_GPIO_ReadPin(Frontal_SW_GPIO_Port,Frontal_SW_Pin);
	return FrontalSW;
}
