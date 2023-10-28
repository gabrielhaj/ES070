/*
 * ultraSonicSensor.h
 *
 *  Created on: 1 de out de 2023
 *      Author: Issao
 */

#ifndef INC_ULTRASONICSENSOR_H_
#define INC_ULTRASONICSENSOR_H_
#define VELOCIDADESOM 0.0343 // [cm/s]

#include "tim.h"
typedef struct {
	TIM_HandleTypeDef* htim;
	volatile uint32_t uiSendTime;
	volatile uint32_t uiReceiveTime;
	unsigned int uiOVC;
	float fDistance;
} ultraSonicSensorStruct;


#include "stm32g4xx_hal.h"

/* Inicializa o sensor ultrassônico */
void vUltrasonicSensorInit(TIM_HandleTypeDef *htim);

/* Envia um pulso */
void vUltrasonicSensorSendTriggerPulse(void);

/* Retorna a distância de um sensor */
float fUltrasonicSensorGetDistanceCm(ultraSonicSensorStruct xUltraSonicSensor);

/* Callback do tempo */
void vUltraSonicSensorCallback(TIM_HandleTypeDef *htim);

/*Callback de overclock*/
void vUltraSonicSensorOverclockCallback(TIM_HandleTypeDef *htim);
#endif /* INC_ULTRASONICSENSOR_H_ */
