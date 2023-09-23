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
float fAquisitionRate = 10*10^-3;

void vOdometryUpdateCurrentStatus(positionStruct xPosition){
	double dRightVel = WHEELRADIUS*dEncoderGetRightWheelVelocity();
	double dLeftVel = WHEELRADIUS*dEncoderGetLeftWheelVelocity();
	double dICCRadius = (DISTANCEBETWEENWHEELS/2)*((dRightVel + dLeftVel)/(dRightVel-dLeftVel));
	double dDTheta = fAquisitionRate*(dRightVel - dLeftVel)/DISTANCEBETWEENWHEELS;
	double dTravlledCenter = dICCRadius*dDTheta;
	xPosition.dXPostion += dTravlledCenter*(double)cos(dDTheta/2);
	xPosition.dYPostion += dTravlledCenter*(double)sin(dDTheta/2);
	xPosition.dThetaPosition +=  dDTheta;
	//todo mean Velocity, Actual Velocity and Travelled Distance

}

