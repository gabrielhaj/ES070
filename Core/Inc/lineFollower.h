/*
 * LineFollower.h
 *
 *  Created on: Sep 11, 2023
 *      Author: gabriel
 */

#ifndef INC_LINEFOLLOWER_H_
#define INC_LINEFOLLOWER_H_

#include "lineSensors.h"
#include "tim.h"

void vLineFollowerInit(TIM_HandleTypeDef *htim);

void vLineFollowerTracker(lineSensorsStateStruct xS);

void vLineFollowerMoveFoward(float fV);

void vLineFollowerTurn(float fVleft, float fVright);

void vLineFollowerMoveFowardSlow();

void vLineFollowerTurnLeftSlow();

void vLineFollowerTurnRightSlow();

void vLineFollowerTurnLeft();

void vLineFollowerTurnRight();

void vLineFollowerSearchLeft();

void vLineFollowerSearchRight();

void vLineFollowerStop();

void vLineFollowerNewTracker();

#endif /* INC_LINEFOLLOWER_H_ */
