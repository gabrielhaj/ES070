/*
 * lineSensors.c
 *
 *  Created on: Sep 16, 2023
 *      Author: gabriel
 */

#include "lineSensors.h"
#include "main.h"

sensorsStateStruct xLineSensorsGetState() {
	sensorsStateStruct xS = {0};
	xS.mostLeftSensor = HAL_GPIO_ReadPin(LightSensor1_GPIO_Port,LightSensor1_Pin);
	xS.leftSensor = HAL_GPIO_ReadPin(LightSensor2_GPIO_Port,LightSensor2_Pin);
	xS.middleSensor = HAL_GPIO_ReadPin(LightSensor3_GPIO_Port,LightSensor3_Pin);
	xS.rightSensor = HAL_GPIO_ReadPin(LightSensor4_GPIO_Port,LightSensor4_Pin);
	xS.mostRightSensor = HAL_GPIO_ReadPin(LightSensor5_GPIO_Port,LightSensor5_Pin);
	return xS;
}
