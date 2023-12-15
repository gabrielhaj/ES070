/*
 * battery.c
 *
 *  Created on: Dec 11, 2023
 *      Author: aluno
 */


#include "battery.h"
//#include "opamp.h"
#include "adc.h"
#include "dma.h"

extern uint16_t usBattery;
//extern ADC_HandleTypeDef hadc2;
//extern DMA_HandleTypeDef hdma_adc2;
//extern OPAMP_HandleTypeDef hopamp2;



void vBatteryInit(ADC_HandleTypeDef *hadc){

	HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
	//HAL_ADC_Start(hadc);
	HAL_ADC_Start_DMA(hadc, &usBattery, 1);
	//HAL_OPAMP_Start(&hopamp2);
}

float fBatteryGetPercentage(void){
	float fMin = 6.7;
	float fMax = 9.4;
	float fRange = fMax - fMin;
	return (fMax - fBatteryGetVoltage())/fRange;

}


float fBatteryGetVoltage(void) {
	//return  3.3*fBatteryGetMeanVoltage()/((2^12)-1);
	return (float)usBattery/209.0;
}
