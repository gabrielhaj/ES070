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
#define WHEELRADIUS (32.5*0.001)
positionStruct xPosition = {0};
int iOdometryClockDivision = 9999; //TIM6 is already divided by 17000. That results in a 10000 Hz freq.
//Thus, for a 1 Hz operation, we should divide by 10000;

TIM_HandleTypeDef *pOdometryTIM;

void vOdometryInit(TIM_HandleTypeDef* htim, int iClockDivision) {
	pOdometryTIM = htim;
	HAL_TIM_Base_Start_IT(pOdometryTIM);
	pOdometryTIM->Instance->ARR = (uint32_t)iClockDivision;
}

void vOdometryUpdateCurrentStatus(positionStruct xPosition){
	float fAquisitionRate = (iOdometryClockDivision+1)/10000; //seconds
	//por algum motivo o compilador não deixou isso ficar fora de uma função
	double dRightVel = WHEELRADIUS*dEncoderGetRightWheelVelocity();
	double dLeftVel = WHEELRADIUS*dEncoderGetLeftWheelVelocity();
	double dICCRadius = (DISTANCEBETWEENWHEELS/2)*((dRightVel + dLeftVel)/(dRightVel-dLeftVel));
	double dDTheta = fAquisitionRate*(dRightVel - dLeftVel)/DISTANCEBETWEENWHEELS;
	double dTravlledCenter = dICCRadius*dDTheta;
	xPosition.iTimeCounter ++;
	xPosition.dTravelledDistance += dTravlledCenter;
	xPosition.dXPostion += dTravlledCenter*(double)cos(dDTheta/2);
	xPosition.dYPostion += dTravlledCenter*(double)sin(dDTheta/2);
	xPosition.dThetaPosition +=  dDTheta;
	xPosition.dActualVelocity = (dRightVel+dLeftVel)/2;
	xPosition.dMeanVelocity = xPosition.dTravelledDistance/(xPosition.iTimeCounter*fAquisitionRate);
	//Velocidade média é uma questão: Velocidade média considerando que trecho? O total de todos os tempos?
	//Considerando um trecho específico? Média das velocidades?

}

