/*
 * odometry.h
 *
 *  Created on: 22 de set de 2023
 *      Author: aluno
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_


typedef struct {
	double dXPostion;
	double dYPostion;
	double dThetaPosition;
	double dTravelledDistance;
	double dActualVelocity;
	double dMeanVelocity;
	int iTimeCounter;
} positionStruct;

void vOdometryInit(TIM_HandleTypeDef* htim, int iClockDivision);
void vOdometryUpdateCurrentStatus();
void vOdometryInverseKinematics(double dOrientationChange, float fVelSetPoint);

#endif /* INC_ODOMETRY_H_ */
