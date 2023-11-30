/*
 * battery.c
 *
 *  Created on: Nov 24, 2023
 *      Author: aluno
 */

#include "battery.h"
#include "adc.h"

uint16_t uiVoltageBuffer[BUFFERSIZE];
float fVoltageSum;
extern ADC_HandleTypeDef hadc2;

void vBatteryInit(ADC_HandleTypeDef *hadc2){
	HAL_ADC_Start(hadc2);
	HAL_ADC_Start_DMA(hadc2, (uint32_t *) uiVoltageBuffer, BUFFERSIZE);
}

float fBatteryGetMeanVoltage(){
	int i;
	fVoltageSum = 0;
	for (i = 0; i < BUFFERSIZE; i++) {
		fVoltageSum = fVoltageSum + uiVoltageBuffer[i];
	}
	return fVoltageSum/BUFFERSIZE;
}


float fBatteryGetVoltage(void) {
	return  3.3*fBatteryGetMeanVoltage()/((2^12)-1);
}
