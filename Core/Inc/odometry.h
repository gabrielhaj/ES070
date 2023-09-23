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
} positionStruct;

void vOdometryUpdateCurrentStatus(positionStruct xPosition);

#endif /* INC_ODOMETRY_H_ */
