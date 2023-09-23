/*
 * encoder.h
 *
 *  Created on: Sep 1, 2023
 *      Author: aluno
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "tim.h"
typedef struct {
	TIM_HandleTypeDef* htim;
	unsigned char ucState;
	volatile uint32_t uiT1;
	volatile uint32_t uiT2;
	volatile int32_t iTicks;
	unsigned int uiTIM_OVC;
	double dFreq;
	double dVel;
} encoderStruct;

void vInitEncoders(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2);

void vEncoderCallback(TIM_HandleTypeDef* htim);

void vEncoderOverflowCallback(TIM_HandleTypeDef* htim);


double dEncoderGetLeftWheelVelocity();

double dEncoderGetRightWheelVelocity();


#endif /* INC_ENCODER_H_ */
