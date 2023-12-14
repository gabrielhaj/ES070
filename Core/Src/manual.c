/*
 * manual.c
 *
 *  Created on: Dec 13, 2023
 *      Author: aluno
 */


#include "manual.h"
#include "pid.h"
#include "motors.h"

void vManualFoward(char cFlag) {
	if(cFlag == '0') {
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	} else {
	   vPIDLineFollowerOutput(0);
	   vPIDMotorsOutput();
	}

}

void vManualBackward(char cFlag) {
	if(cFlag == '0') {
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	} else {
	   vPIDLineFollowerOutput(0);
	   vPIDMotorsOutput();
	}
}

void vManualLeft(char cFlag) {
	if(cFlag == '0') {
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	} else {
	   vPIDLineFollowerOutput(-1);
	   vPIDMotorsOutput();
	}
}

void vManualRight(char cFlag) {
	if(cFlag == '0') {
	  vMotorsLeftPower(0);
	  vMotorsRightPower(0);
	} else {
	   vPIDLineFollowerOutput(1);
	   vPIDMotorsOutput();
	}
}
