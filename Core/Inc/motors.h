/*
 * motors.h
 *
 *  Created on: Sep 10, 2023
 *      Author: gabriel
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "tim.h"

void vMotorsInit(TIM_HandleTypeDef *htim);

void vMotorsStart();

void vMotorsStop();

void vMotorsLeftPower(float fLeftPower);

void vMotorsRightPower(float fRightPower);

void vMotorsLeftWheelFoward();

void vMotorsLeftWheelBackwards();

void vMotorsRightWheelFoward();

void vMotorsRightWheelBackwards();

void vMotorsBreak();

char cMotorsGetState();

void vMotorsSetState(char cNewState);

#endif /* INC_MOTORS_H_ */
