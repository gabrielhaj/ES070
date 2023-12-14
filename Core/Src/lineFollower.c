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
#define SENSORANGLE 180 //degrees

float fVmax = 0.8;
float fVsoft = 0.75;
char cflag = 0;
unsigned char ucFollowerState = 0;
unsigned char ucFollowerCounter = 0;
char cCounter = 0;
double dOrientationChange = 0;
extern float fVelSetPoint;
extern float fLeftSetPoint;
extern float fRightSetPoint;
extern positionStruct xPosition;
double dActualOrientation = 0;
char cState = 0;
// double dActualXPosition = 0;
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
//		if(cCounter > ((4*0.01)/xPosition.dActualVelocity)/(0.01)) {//espessura da faixa e ciclo do clock
//			dOrientationChange = 0;
//			fVelSetPoint = 0;
//			vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
//		}
//		cCounter ++;
	} else if(xS.mostLeftSensor == white) {
		//dOrientationChange = 2*(PI/180)*SENSORANGLE;
		//vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
		vPIDLineFollowerOutput(-2);
	} else if(xS.mostRightSensor == white) {
		//dOrientationChange = -2*(PI/180)*SENSORANGLE;
		//vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
		vPIDLineFollowerOutput(2);
	} else if(xS.leftSensor == white) {
		//dOrientationChange = (PI/180)*SENSORANGLE;
		//vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
		vPIDLineFollowerOutput(-1);
	} else if(xS.rightSensor == white) {
		//dOrientationChange = -1*(PI/180)*SENSORANGLE;
		//vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
		vPIDLineFollowerOutput(1);
	} else if(xS.middleSensor == white) {
		//dOrientationChange = 0;
		//vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
		vPIDLineFollowerOutput(0);
		//to-do
	}
//	else {
//		dOrientationChange = dActualOrientation - xPosition.dThetaPosition;
//		vOdometryInverseKinematics(dOrientationChange, fVelSetPoint);
//		//to-do
//		//Aqui seria uma questão de subtrair da orientação o quanto que se variou nela entre uma interrupção e outra
//	}
//	dActualOrientation = xPosition.dThetaPosition;
	//dActualXPosition = xPosition.dXPostion;
}


char cLineFollowerGetState() {
	return cState;
}

void vLineFollowerSetState(char cNewState){
	if(cNewState) {
		cState = cNewState;
	} else if(!cNewState) {
		cState = cNewState;
	}
}

