/*
 * battery.h
 *
 *  Created on: Nov 24, 2023
 *      Author: aluno
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_
#include "adc.h"

//#define BUFFERSIZE 1



void vBatteryInit(ADC_HandleTypeDef *hadc);

float fBatteryGetMeanVoltage(void);

float fBatteryGetVoltage(void);


#endif /* INC_BATTERY_H_ */
