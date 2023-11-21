 /*
 * LineFollower.c
 *
 *  Created on: Sep 11, 2023
 *      Author: gabriel
 */

#include <lineFollower.h>
#include "motors.h"
#include "lineSensors.h"

#define PI 3.1415
#define MAX_FOLLOWER_COUNTER 5
#define SENSORANGLE 30 //degrees

float fVmax = 1;
float fVsoft = 0.75;
char cflag = 0;
unsigned char ucFollowerState = 0;
unsigned char ucFollowerCounter = 0;
float fOrientationChange = 0;
extern float fVelSetPoint;
extern float fLeftSetPoint;
extern float fRightSetPoint;
TIM_HandleTypeDef *pLineFollowerTIM;



void vLineFollowerInit(TIM_HandleTypeDef *htim){
	pLineFollowerTIM = htim;
	HAL_TIM_Base_Start_IT(pLineFollowerTIM);
}
void vLineFollowerTracker(lineSensorsStateStruct xS) {
	if(cflag && xS.mostRightSensor == white && xS.rightSensor == white && xS.middleSensor == white && xS.leftSensor == white && xS.mostLeftSensor == white) {
		ucFollowerState = 6;
	} else if(!cflag && xS.mostRightSensor == white && xS.rightSensor == white && xS.middleSensor == white && xS.leftSensor == white && xS.mostLeftSensor == white) {
		//AndarEspessuraCruzamentoEChecarDenovo();
	} else if(xS.leftSensor == black && xS.rightSensor == black && xS.middleSensor == white) {
		//Manter linha reta
		ucFollowerState = 1;
		vLineFollowerMoveFoward(fVmax);
	} else if(xS.leftSensor == white && xS.rightSensor == black) {
		//vLineFollowerTurnLeft
		vLineFollowerTurnLeftSlow();
		ucFollowerState = 2;
	} else if(xS.leftSensor == black && xS.rightSensor == white) {
		vLineFollowerTurnRightSlow();
		ucFollowerState = 3;
	} else if(xS.mostLeftSensor == white) {
		vLineFollowerTurnLeft();
		ucFollowerState = 4;
	} else if(xS.mostRightSensor == white) {
		vLineFollowerTurnRight();
		ucFollowerState = 5;
	} else if(xS.mostRightSensor == black && xS.rightSensor == black && xS.middleSensor == black && xS.leftSensor == black && xS.mostLeftSensor == black) {
		ucFollowerCounter ++;
	} else {
		ucFollowerState = 0 ;
	}

	if(ucFollowerCounter > MAX_FOLLOWER_COUNTER) {
		ucFollowerCounter = 0;
		ucFollowerState = 0;
	}

	if(ucFollowerCounter > 0) {
		switch(ucFollowerState) {
		case 1:
			vLineFollowerMoveFowardSlow();
		case 2:
			vLineFollowerSearchLeft();
		case 3:
			vLineFollowerSearchRight();
		case 4:
			vLineFollowerSearchLeft();
		case 5:
			vLineFollowerSearchRight();
		case 6:
			vLineFollowerStop();
		}
	}
}

void vLineFollowerMoveFoward(float fV) {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fV);
	vMotorsRightPower(fV);
}


void vLineFollowerTurn(float fVleft, float fVright) {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVleft);
	vMotorsRightPower(fVright);
}

void vLineFollowerMoveFowardSlow() {
	vLineFollowerMoveFoward(fVsoft);
}

void vLineFollowerTurnLeftSlow() {
	vLineFollowerTurn(fVsoft,fVmax);
}

void vLineFollowerTurnRightSlow() {
	vLineFollowerTurn(fVmax,fVsoft);
}

void vLineFollowerTurnLeft() {
	vLineFollowerTurn(0,fVmax);
}

void vLineFollowerTurnRight() {
	vLineFollowerTurn(fVmax,0);
}

void vLineFollowerSearchLeft() {
	vLineFollowerTurn(0,fVsoft);
}

void vLineFollowerSearchRight() {
	vLineFollowerTurn(fVsoft,0);
}

void vLineFollowerStop() {
	vMotorsBreak();
}

void vLineFollowerNewTracker(lineSensorsStateStruct xS) {
	if(xS.leftSensor == white && xS.mostLeftSensor == white && xS.rightSensor == white && xS.mostRightSensor == white && xS.middleSensor == white ) {
		fOrientationChange = 0;
		fVelSetPoint = 0;
		vOdometryInverseKinematics(fOrientationChange, fVelSetPoint);
	} else if(xS.mostLeftSensor == white) {
		fOrientationChange = 2*(PI/180)*SENSORANGLE;
		vOdometryInverseKinematics(fOrientationChange, fVelSetPoint);
	} else if(xS.mostRightSensor == white) {
		fOrientationChange = -2*(PI/180)*SENSORANGLE;
		vOdometryInverseKinematics(fOrientationChange, fVelSetPoint);
	} else if(xS.leftSensor == white) {
		fOrientationChange = (PI/180)*SENSORANGLE;
		vOdometryInverseKinematics(fOrientationChange, fVelSetPoint);
	} else if(xS.rightSensor == white) {
		fOrientationChange = -1*(PI/180)*SENSORANGLE;
		vOdometryInverseKinematics(fOrientationChange, fVelSetPoint);
	} else if(xS.middleSensor == white) {
		fOrientationChange = 0;
		vOdometryInverseKinematics(fOrientationChange, fVelSetPoint);
	} else {
		//to-do
		//Aqui seria uma questão de subtrair da orientação o quanto que se variou nela entre uma interrupção e outra
	}
}

