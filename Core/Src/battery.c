/*
 * battery.c
 *
 *  Created on: Nov 24, 2023
 *      Author: aluno
 */

#include "battery.h"
#include "adc.h"

//static uint16_t uiVoltageBuffer[BUFFERSIZE];
extern uint16_t usTemperature;
float fVoltageSum;
//extern ADC_HandleTypeDef hadc2;
//extern DMA_HandleTypeDef hdma_adc2;

void vBatteryInit(ADC_HandleTypeDef *hadc){
	//HAL_ADC_Start(&hadc2);
	HAL_ADC_Start_DMA(&hadc2, &usTemperature, 1);
}

/*float fBatteryGetMeanVoltage(void){
	int i;
	fVoltageSum = 0;
	for (i = 0; i < BUFFERSIZE; i++) {
		fVoltageSum = fVoltageSum + uiVoltageBuffer[i];
	}
	return fVoltageSum/BUFFERSIZE;
}*/


float fBatteryGetVoltage(void) {
	//return  3.3*fBatteryGetMeanVoltage()/((2^12)-1);
	return 3.3*usTemperature/(2^12 -1);
}
