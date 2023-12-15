/*
 * battery.h
 *
 *  Created on: Dec 11, 2023
 *      Author: aluno
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_
#include "adc.h"
//#include "opamp.h"
#include "dma.h"

void vBatteryInit(ADC_HandleTypeDef *hadc);

float fBatteryGetPercentage(void);

float fBatteryGetVoltage(void);


#endif /* INC_BATTERY_H_ */
