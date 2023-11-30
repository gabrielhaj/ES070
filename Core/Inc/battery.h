/*
 * battery.h
 *
 *  Created on: Nov 24, 2023
 *      Author: aluno
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#define BUFFERSIZE 10

#include "adc.h"

void vBatteryInit(ADC_HandleTypeDef *hadc2);

float fBatteryGetMeanVoltage();

float fBatteryGetVoltage(void);


#endif /* INC_BATTERY_H_ */
