/*
 * manual.c
 *
 *  Created on: Dec 13, 2023
 *      Author: aluno
 */


#include "manual.h"
#include "pid.h"
#include "motors.h"

extern float fRightSetPoint;
extern float fLeftSetPoint;
extern float fVelSetPoint;

void vManualFoward(char cFlag) {
	if(cFlag == '0') {
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	  fRightSetPoint = 0;
	  fLeftSetPoint = 0;
	} else {
		fLeftSetPoint = fVelSetPoint;
		fRightSetPoint = fVelSetPoint;
	   vPIDMotorsOutput();
	}

}

void vManualBackward(char cFlag) {
//	if(cFlag == '0') {
//	  vMotorsLeftPower(0);
//	  vMotorsRightPower(0);
//	} else {
//	   vPIDLineFollowerOutput(0);
//	   vPIDMotorsOutput();
//	}
}

void vManualLeft(char cFlag) {
	if(cFlag == '0') {
	  fRightSetPoint = 0;
	  fLeftSetPoint = 0;
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	} else {
		fLeftSetPoint = 0;
		fRightSetPoint = fVelSetPoint;
	   vPIDMotorsOutput();
	}
}

void vManualRight(char cFlag) {
	if(cFlag == '0') {
	  fRightSetPoint = 0;
	  fLeftSetPoint = 0;
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	} else {
		fLeftSetPoint = fVelSetPoint;
		fRightSetPoint = 0;
	   vPIDMotorsOutput();
	}
}
