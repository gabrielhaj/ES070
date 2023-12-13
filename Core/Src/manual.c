/*
 * manual.c
 *
 *  Created on: Dec 13, 2023
 *      Author: aluno
 */


#include "manual.h"
#include "pid.h"

void vManualFoward() {
	vPIDLineFollowerOutput(0);
	vPIDMotorsOutput();
}

void vManualBackward() {
	vPIDLineFollowerOutput(0);
	vPIDMotorsOutput();
}

void vManualLeft() {
	vPIDLineFollowerOutput(-1);
	vPIDMotorsOutput();
}

void vManualRight() {
	vPIDLineFollowerOutput(1);
	vPIDMotorsOutput();
}
