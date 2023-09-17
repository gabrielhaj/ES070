/*
 * LineFollower.c
 *
 *  Created on: Sep 11, 2023
 *      Author: gabriel
 */

#include <lineFollower.h>
#include "motors.h"
#include "lineSensors.h"

float fVmax = 1;
float fVsoft = 0.75;
char cflag = 0;



void vLineFollowerTracker(sensorsStateStruct xS) {
	if(cflag && xS.mostRightSensor == white && xS.rightSensor == white && xS.middleSensor == white && xS.leftSensor == white && xS.mostLeftSensor == white) {
		vLineFollowerStop();
	} else if(!cflag && xS.mostRightSensor == white && xS.rightSensor == white && xS.middleSensor == white && xS.leftSensor == white && xS.mostLeftSensor == white) {
		//AndarEspessuraCruzamentoEChecarDenovo();
	} else if(xS.leftSensor == black && xS.rightSensor == black && xS.middleSensor == white) {
		//Manter linha reta
		vLineFollowerMoveFoward();
	} else if(xS.leftSensor == white && xS.rightSensor == black) {
		//vLineFollowerTurnLeft
		vLineFollowerTurnLeftSlow();
	} else if(xS.leftSensor == black && xS.rightSensor == white) {
		vLineFollowerTurnRightSlow();
	} else if(xS.mostLeftSensor == white) {
		vLineFollowerTurnLeft();
	} else if(xS.mostRightSensor == white) {
		vLineFollowerTurnRight();
	} else if(xS.mostRightSensor == black && xS.rightSensor == black && xS.middleSensor == black && xS.leftSensor == black && xS.mostLeftSensor == black) {
		vLineFollowerStop();
	}
}

void vLineFollowerMoveFoward() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVmax);
	vMotorsRightPower(fVmax);
}

void vLineFollowerTurnLeftSlow() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVsoft);
	vMotorsRightPower(fVmax);
}

void vLineFollowerTurnRightSlow() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVmax);
	vMotorsRightPower(fVsoft);
}

void vLineFollowerTurnLeft() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(0);
	vMotorsRightPower(fVmax);
}

void vLineFollowerTurnRight() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVmax);
	vMotorsRightPower(0);
}

void vLineFollowerStop() {
	vMotorsBreak();
}
