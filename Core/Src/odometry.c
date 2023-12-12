/*
 * odometry.c
 *
 *  Created on: 22 de set de 2023
 *      Author: aluno
 */

#include "encoder.h"
#include "math.h"
#include "odometry.h"

#define DISTANCEBETWEENWHEELS (13*0.01)
#define PI 3.1415
#define WHEELRADIUS (32.5*0.001)
#define DISTANCETOSENSORS 7*0.01 //Definir a distância do centro do carro até os sensores, aproximadamente.
extern positionStruct xPosition;
extern float fLeftSetPoint;
extern float fRightSetPoint;
int iOdometryClockDivision = 100; //TIM6 is already divided by 17000. That results in a 10000 Hz freq.
//Thus, for a 1 Hz operation, we should divide by 10000;

TIM_HandleTypeDef *pOdometryTIM;

void vOdometryInit(TIM_HandleTypeDef* htim, int iClockDivision) {
	pOdometryTIM = htim;
	HAL_TIM_Base_Start_IT(pOdometryTIM);
	//pOdometryTIM->Instance->ARR = (uint32_t)iClockDivision;
}

void vOdometryUpdateCurrentStatus(){
	double dICCRadius = 0;
	float fAquisitionRate = 0.01; //seconds
	//por algum motivo o compilador não deixou isso ficar fora de uma função
	double dRightVel = dEncoderGetRightWheelVelocity();
	double dLeftVel = dEncoderGetLeftWheelVelocity();
	if((dRightVel-dLeftVel) != 0) {
		dICCRadius = (DISTANCEBETWEENWHEELS/2)*((dRightVel + dLeftVel)/(dRightVel-dLeftVel));
	}
	double dDTheta = fAquisitionRate*(dRightVel - dLeftVel)/DISTANCEBETWEENWHEELS;
	double dTravlledCenter = dICCRadius*dDTheta;
	xPosition.iTimeCounter ++;
	xPosition.dTravelledDistance += dTravlledCenter;
	xPosition.dXPostion += dTravlledCenter*(double)cos(dDTheta/2);
	xPosition.dYPostion += dTravlledCenter*(double)sin(dDTheta/2);
	xPosition.dThetaPosition +=  dDTheta;
	if(xPosition.dThetaPosition > 2*PI) {
		xPosition.dThetaPosition -= 2*PI;
	} else if(xPosition.dThetaPosition < 2*PI) {
		xPosition.dThetaPosition += 2*PI;
	}
	xPosition.dActualVelocity = (dRightVel+dLeftVel)/2;
	xPosition.dMeanVelocity = xPosition.dTravelledDistance/(xPosition.iTimeCounter*fAquisitionRate);
	//Velocidade média é uma questão: Velocidade média considerando que trecho? O total de todos os tempos?
	//Considerando um trecho específico? Média das velocidades?

}

void vOdometryInverseKinematics(double dOrientationChange, float fVelSetPoint) {
	fLeftSetPoint = fVelSetPoint*(1 - (DISTANCEBETWEENWHEELS/2)*dOrientationChange/DISTANCETOSENSORS);
	fRightSetPoint = fVelSetPoint*(1 + (DISTANCEBETWEENWHEELS/2)*dOrientationChange/DISTANCETOSENSORS);
}

double dOdometryGetTravelledDistance(void){
	return xPosition.dTravelledDistance;
}

double dOdometryGetActualVelocity(void) {
	return xPosition.dActualVelocity;
}
double dOdometryGetMeanVelocity(void) {
	return xPosition.dMeanVelocity;
}
double dOdometryGetXCoordinate(void) {
	return xPosition.dXPostion;
}
double dOdometryGetYCoordinate(void) {
	return xPosition.dYPostion;
}
double dOdometryGetThetaCoordinate(void) {
	return xPosition.dThetaPosition;
}


