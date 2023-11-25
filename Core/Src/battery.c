/*
 * battery.c
 *
 *  Created on: Nov 24, 2023
 *      Author: aluno
 */

#include "battery.h"
#include "adc.h"

uint32_t uiVoltage;

void vBatteryInit(ADC_HandleTypeDef *hadc2){
	HAL_ADC_Start_DMA(hadc2, &uiVoltage, 1);
}

float fBatteryGetVoltage(void) {
	return  3*uiVoltage/((2^12)-1);
}
