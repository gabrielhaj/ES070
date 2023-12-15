/* ***************************************************************** */
/* File name:        pid.c                                           */
/* File description: This file has a couple of useful functions to   */
/*                   control the implemented PID controller          */
/* Author name:      julioalvesMS, IagoAF, rBacurau                  */
/* Creation date:    21jun2018                                       */
/* Revision date:    21mai2023                                       */
/* ***************************************************************** */

#include "pid.h"
#include "main.h"


// Struct used to store the PID configuration parameters
pid_data_type pidConfig[3] = {0};
// Counter used to control the integration error window
unsigned short usIntegratorCount[3] = {0,0,0};

// Buffer used to store the errors to generate the integral error
float fIntegratorBuffer[3][INTEGRATOR_MAX_SIZE]={0};

extern float fVelSetPoint;
float fLeftSetPoint = 0;
float fRightSetPoint = 0;
float fLeftActualPower = 0;
float fRightActualPower = 0;
float fUpdate = 0;
extern float fSetPointTemperature;
extern encoderStruct xRightEncoder;
extern encoderStruct xLeftEncoder;

/* ************************************************ */
/* Method name:        pid_init                     */
/* Method description: Initialize the PID controller*/
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void pid_init(float fKp, float fKi, float fKd, unsigned short usIntSizeMs, float fOutputUpperSaturation, float fOutputLowerSaturation )
{
	pidConfig[0].fKp = fKp;
	pidConfig[0].fKd = fKd;
	pidConfig[0].fKi = fKi;
	pidConfig[0].fError_previous = 0;
	pidConfig[0].fError_sum = 0.0;
	pidConfig[1].fKp = fKp;
	pidConfig[1].fKd = fKd;
	pidConfig[1].fKi = fKi;
	pidConfig[1].fError_previous = 0;
	pidConfig[1].fError_sum = 0.0;

	// Saturates Integrator size (up to 10 s)
	if((usIntSizeMs/UPDATE_RATE_MS)> INTEGRATOR_MAX_SIZE)
	  usIntSizeMs = INTEGRATOR_MAX_SIZE * UPDATE_RATE_MS;

	pidConfig[0].usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;

	pidConfig[0].fOutputUpperSaturation = fOutputUpperSaturation;
	pidConfig[0].fOutputLowerSaturation = fOutputLowerSaturation;

	pidConfig[1].usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;

	pidConfig[1].fOutputUpperSaturation = fOutputUpperSaturation;
	pidConfig[1].fOutputLowerSaturation = fOutputLowerSaturation;
}

void pid_init2(float fKp, float fKi, float fKd, unsigned short usIntSizeMs, float fOutputUpperSaturation, float fOutputLowerSaturation )
{
	pidConfig[2].fKp = fKp;
	pidConfig[2].fKd = fKd;
	pidConfig[2].fKi = fKi;
	pidConfig[2].fError_previous = 0;
	pidConfig[2].fError_sum = 0.0;

	// Saturates Integrator size (up to 10 s)
	if((usIntSizeMs/UPDATE_RATE_MS)> INTEGRATOR_MAX_SIZE)
	  usIntSizeMs = INTEGRATOR_MAX_SIZE * UPDATE_RATE_MS;

	pidConfig[2].usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;

	pidConfig[2].fOutputUpperSaturation = fOutputUpperSaturation;
	pidConfig[2].fOutputLowerSaturation = fOutputLowerSaturation;
}



/* ************************************************** */
/* Method name:        pid_setKp                      */
/* Method description: Set a new value for the PID    */
/*                     proportional constant          */
/* Input params:       fKp: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setKp(float fKp)
{
	pidConfig[0].fKp = fKp;
	pidConfig[1].fKp = fKp;
}


/* ************************************************** */
/* Method name:        pid_getKp                      */
/* Method description: Get the value from the PID     */
/*                     proportional constant          */
/* Input params:       n/a                            */
/* Output params:      float: Value                   */
/* ************************************************** */
float pid_getKp(void)
{
	return pidConfig[0].fKp;
}


/* ************************************************** */
/* Method name:        pid_setKi                      */
/* Method description: Set a new value for the PID    */
/*                     integrative constant           */
/* Input params:       fKi: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setKi(float fKi)
{
	pidConfig[0].fKi = fKi;
	pidConfig[1].fKi = fKi;
}


/* ************************************************** */
/* Method name:        pid_getKi                      */
/* Method description: Get the value from the PID     */
/*                     integrative constant           */
/* Input params:       n/a                            */
/* Output params:      float: Value                   */
/* ************************************************** */
float pid_getKi(void)
{
	return pidConfig[0].fKi;
}


/* ************************************************** */
/* Method name:        pid_setKd                      */
/* Method description: Set a new value for the PID    */
/*                     derivative constant            */
/* Input params:       fKd: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setKd(float fKd)
{
	pidConfig[0].fKd = fKd;
	pidConfig[1].fKd = fKd;
}


/* ************************************************** */
/* Method name:        pid_getKd                      */
/* Method description: Get the value from the PID     */
/*                     derivative constant            */
/* Input params:       n/a                            */
/* Output params:      float: Value                   */
/* ************************************************** */
float pid_getKd(void)
{
	return pidConfig[0].fKd;
}

/* ************************************************** */
/* Method name:        pid_setIntegratorWindow        */
/* Method description: Set a new value for the        */
/*                     integrator window (in ms)      */
/* Input params:       usIntSizeMs: New value (in ms) */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setIntegratorWindow (unsigned short usIntSizeMs)
{
	// Saturates Integrator size (10000 ms)
	if((usIntSizeMs/UPDATE_RATE_MS)> INTEGRATOR_MAX_SIZE)
	  usIntSizeMs = INTEGRATOR_MAX_SIZE * UPDATE_RATE_MS;

	pidConfig[0].usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;
}

/* ************************************************** */
/* Method name:        pid_getIntegratorWindow        */
/* Method description: Get the value from the         */
/*                     integrator window (in ms)      */
/* Input params:       n/a                            */
/* Output params:      usIntSizeMs: Value (in ms)     */
/* ************************************************** */
unsigned short pid_getIntegratorWindow (void)
{
	return (pidConfig[0].usIntegratorSize*UPDATE_RATE_MS);
}

/* ************************************************** */
/* Method name:        pid_updateData                 */
/* Method description: Update the control output      */
/*                     using the reference and sensor */
/*                     value                          */
/* Input params:       fSensorValue: Value read from  */
/*                     the sensor                     */
/*                     fReferenceValue: Value used as */
/*                     control reference              */
/* Output params:      float: New Control effort     */
/* ************************************************** */
float pidUpdateData(float fSensorValue, float fSetValue, int i)
{
	float fError, fDifference, fOut;

	// Proportional error
	fError = fSetValue - fSensorValue;


	//Ingtegral error
	pidConfig[i].fError_sum = pidConfig[i].fError_sum - fIntegratorBuffer[i][usIntegratorCount[i]] + fError;

	fIntegratorBuffer[i][usIntegratorCount[i]] = fError;

	if(++usIntegratorCount[i] >= pidConfig[i].usIntegratorSize)
		usIntegratorCount[i] = 0;

	// Differential error
	fDifference = (fError - pidConfig[i].fError_previous);

	fOut = pidConfig[i].fKp * fError
		 + pidConfig[i].fKi * pidConfig[i].fError_sum *UPDATE_RATE_MS
		 + pidConfig[i].fKd * fDifference /UPDATE_RATE_MS;

	pidConfig[i].fError_previous = fError;

	//fOut = -fOut;

	if(fOut > pidConfig[i].fOutputUpperSaturation)
	{
		fOut = pidConfig[i].fOutputUpperSaturation;
		pidConfig[i].fError_sum = pidConfig[i].fOutputUpperSaturation;

	}
	else if (fOut < -pidConfig[i].fOutputUpperSaturation)
	{
		fOut = -pidConfig[i].fOutputUpperSaturation;
		pidConfig[i].fError_sum = -pidConfig[i].fOutputUpperSaturation;

	}

	return fOut;
}

float pidUpdateData2(float fSensorValue, float fSetValue)
{
	float fError, fDifference, fOut;

	// Proportional error
	fError = fSetValue - fSensorValue;

	//Ingtegral error
	pidConfig[2].fError_sum = pidConfig[2].fError_sum - fIntegratorBuffer[2][usIntegratorCount[2]] + fError;

	fIntegratorBuffer[2][usIntegratorCount[2]] = fError;

	if(++usIntegratorCount[2] >= pidConfig[2].usIntegratorSize)
		usIntegratorCount[2] = 0;

	// Differential error
	fDifference = (fError - pidConfig[2].fError_previous);

	fOut = pidConfig[2].fKp * fError
		 + pidConfig[2].fKi * pidConfig[2].fError_sum *UPDATE_RATE_MS
		 + pidConfig[2].fKd * fDifference /UPDATE_RATE_MS;

	pidConfig[2].fError_previous = fError;

	//fOut = -fOut;

	if(fOut > pidConfig[2].fOutputUpperSaturation)
	{
		fOut = pidConfig[2].fOutputUpperSaturation;
		pidConfig[2].fError_sum = 0;
	}
	else if (fOut < -pidConfig[2].fOutputUpperSaturation)
	{
		fOut = -pidConfig[2].fOutputUpperSaturation;
		pidConfig[2].fError_sum = 0;
	}

	return fOut;
}

//void vPIDMotorsOutput() {
//	fLeftActualPower = fLeftActualPower + pidUpdateData(dEncoderGetLeftWheelVelocity(), fLeftSetPoint);
//	if(fLeftActualPower > 1) {
//		fLeftActualPower = 1;
//	} else if(fLeftActualPower < 0) {
//		fLeftActualPower = 0;
//	}
//	vMotorsLeftWheelFoward();
//	vMotorsLeftPower(fLeftActualPower);
//	fRightActualPower = fRightActualPower + pidUpdateData(dEncoderGetRightWheelVelocity(), fRightSetPoint);
//	if(fRightActualPower > 1) {
//		fRightActualPower = 1;
//	} else if(fRightActualPower < 0) {
//		fRightActualPower = 0;
//	}
//	vMotorsRightWheelFoward();
//	vMotorsRightPower(fRightActualPower);
//}


void vPIDMotorsOutput() {
  if(fLeftSetPoint == 0) {
	  fLeftActualPower = 0;
  } else {
	  fLeftActualPower =  pidUpdateData(dEncoderGetLeftWheelVelocity(), fLeftSetPoint, 0);
  }
  if(fLeftActualPower < 0) {
	  fLeftActualPower = 0;
  }
  vMotorsLeftPower(fLeftActualPower);
  if(fRightSetPoint == 0) {
	  fRightActualPower = 0;
  } else {
	  fRightActualPower = pidUpdateData(dEncoderGetRightWheelVelocity(), fRightSetPoint,1);
  }
  if(fRightActualPower < 0) {
	  fRightActualPower = 0;
  }
  vMotorsRightPower(fRightActualPower);
}

void vPIDLineFollowerOutput(float fDirection) {
	fUpdate = pidUpdateData2(fDirection,0);
	if (fUpdate < 0){
		fLeftSetPoint = fVelSetPoint;
		fRightSetPoint = fVelSetPoint*(1 + fUpdate);
	}
	else {
		fLeftSetPoint = fVelSetPoint*(1 - fUpdate);
		fRightSetPoint = fVelSetPoint;
	}

}

void vPIDIncreaseKp() {
	pidConfig[2].fKp +=  0.05;
}

void vPIDDecreaseKp(){
	pidConfig[2].fKp -= 0.05;
}

void vPIDIncreaseKd() {
	pidConfig[2].fKd ++;
}

void vPIDIncreaseKi() {
	pidConfig[2].fKi ++;
}

void vPID2SetKp(float fKp){
	pidConfig[2].fKp = fKp;
}
void vPID2SetKi(float fKi){
	pidConfig[2].fKi = fKi;
}
void vPID2SetKd(float fKd) {
	pidConfig[2].fKd = fKd;
}
float fPID2GetKp(){
	return pidConfig[2].fKp;
}
float fPID2GetKi(){
	return pidConfig[2].fKi;
}
float fPID2GetKd() {
	return pidConfig[2].fKd;
}

float fPIDGetVelSetPoint(){
	return fVelSetPoint;
}
__weak void vPIDPeriodicControlTask() {}
