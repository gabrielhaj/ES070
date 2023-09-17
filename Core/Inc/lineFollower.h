/*
 * LineFollower.h
 *
 *  Created on: Sep 11, 2023
 *      Author: gabriel
 */

#ifndef INC_LINEFOLLOWER_H_
#define INC_LINEFOLLOWER_H_

#include "lineSensors.h"

void vLineFollowerTracker(sensorsStateStruct xS);

void vLineFollowerMoveFoward();

void vLineFollowerTurnLeftSlow();

void vLineFollowerTurnRightSlow();

void vLineFollowerTurnLeft();

void vLineFollowerTurnRight();

void vLineFollowerStop();

#endif /* INC_LINEFOLLOWER_H_ */
