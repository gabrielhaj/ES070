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
pid_data_type pidConfig, pidConfig2 ;
// Counter used to control the integration error window
unsigned short usIntegratorCount = 0;
// Buffer used to store the errors to generate the integral error
float fIntegratorBuffer[INTEGRATOR_MAX_SIZE]={0};

float fError, fDifference, fOut;

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
	pidConfig.fKp = fKp;
	pidConfig.fKd = fKd;
	pidConfig.fKi = fKi;
	pidConfig.fError_previous = 0;
	pidConfig.fError_sum = 0.0;

	// Saturates Integrator size (up to 10 s)
	if((usIntSizeMs/UPDATE_RATE_MS)> INTEGRATOR_MAX_SIZE)
	  usIntSizeMs = INTEGRATOR_MAX_SIZE * UPDATE_RATE_MS;

	pidConfig.usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;

	pidConfig.fOutputUpperSaturation = fOutputUpperSaturation;
	pidConfig.fOutputLowerSaturation = fOutputLowerSaturation;
}

void pid_init2(float fKp, float fKi, float fKd, unsigned short usIntSizeMs, float fOutputUpperSaturation, float fOutputLowerSaturation )
{
	pidConfig2.fKp = fKp;
	pidConfig2.fKd = fKd;
	pidConfig2.fKi = fKi;
	pidConfig2.fError_previous = 0;
	pidConfig2.fError_sum = 0.0;

	// Saturates Integrator size (up to 10 s)
	if((usIntSizeMs/UPDATE_RATE_MS)> INTEGRATOR_MAX_SIZE)
	  usIntSizeMs = INTEGRATOR_MAX_SIZE * UPDATE_RATE_MS;

	pidConfig2.usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;

	pidConfig2.fOutputUpperSaturation = fOutputUpperSaturation;
	pidConfig2.fOutputLowerSaturation = fOutputLowerSaturation;
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
	pidConfig.fKp = fKp;
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
	return pidConfig.fKp;
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
	pidConfig.fKi = fKi;
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
	return pidConfig.fKi;
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
	pidConfig.fKd = fKd;
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
	return pidConfig.fKd;
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

	pidConfig.usIntegratorSize = usIntSizeMs/UPDATE_RATE_MS;
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
	return (pidConfig.usIntegratorSize*UPDATE_RATE_MS);
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
float pidUpdateData(float fSensorValue, float fSetValue)
{
	//float fError, fDifference, fOut;

	// Proportional error
	fError = fSetValue - fSensorValue;

	//Ingtegral error
	pidConfig.fError_sum = pidConfig.fError_sum - fIntegratorBuffer[usIntegratorCount] + fError;

	fIntegratorBuffer[usIntegratorCount] = fError;

	if(++usIntegratorCount >= pidConfig.usIntegratorSize)
		usIntegratorCount = 0;

	// Differential error
	fDifference = (fError - pidConfig.fError_previous);

	fOut = pidConfig.fKp * fError
		 + pidConfig.fKi * pidConfig.fError_sum *UPDATE_RATE_MS
		 + pidConfig.fKd * fDifference /UPDATE_RATE_MS;

	pidConfig.fError_previous = fError;

	//fOut = -fOut;

	if(fOut > pidConfig.fOutputUpperSaturation)
	{
		fOut = pidConfig.fOutputUpperSaturation;

	}
	else if (fOut < -pidConfig.fOutputUpperSaturation)
	{
		fOut = -pidConfig.fOutputUpperSaturation;

	}

	return fOut;
}

float pidUpdateData2(float fSensorValue, float fSetValue)
{
	float fError, fDifference, fOut;

	// Proportional error
	fError = fSetValue - fSensorValue;

	//Ingtegral error
	pidConfig2.fError_sum = pidConfig2.fError_sum - fIntegratorBuffer[usIntegratorCount] + fError;

	fIntegratorBuffer[usIntegratorCount] = fError;

	if(++usIntegratorCount >= pidConfig2.usIntegratorSize)
		usIntegratorCount = 0;

	// Differential error
	fDifference = (fError - pidConfig2.fError_previous);

	fOut = pidConfig2.fKp * fError
		 + pidConfig2.fKi * pidConfig2.fError_sum *UPDATE_RATE_MS
		 + pidConfig2.fKd * fDifference /UPDATE_RATE_MS;

	pidConfig2.fError_previous = fError;

	//fOut = -fOut;

	if(fOut > pidConfig2.fOutputUpperSaturation)
	{
		fOut = pidConfig2.fOutputUpperSaturation;
		pidConfig2.fError_sum = 0;
	}
	else if (fOut < -pidConfig2.fOutputUpperSaturation)
	{
		fOut = -pidConfig2.fOutputUpperSaturation;
		pidConfig2.fError_sum = 0;
	}

	return fOut;
}

void vPIDMotorsOutput() {
	fLeftActualPower = fLeftActualPower + pidUpdateData(dEncoderGetLeftWheelVelocity(), fLeftSetPoint);
	if(fLeftActualPower > 1) {
		fLeftActualPower = 1;
	} else if(fLeftActualPower < 0) {
		fLeftActualPower = 0;
	}
	vMotorsLeftWheelFoward();
	vMotorsLeftPower(fLeftActualPower);
	fRightActualPower = fRightActualPower + pidUpdateData(dEncoderGetRightWheelVelocity(), fRightSetPoint);
	if(fRightActualPower > 1) {
		fRightActualPower = 1;
	} else if(fRightActualPower < 0) {
		fRightActualPower = 0;
	}
	vMotorsRightWheelFoward();
	vMotorsRightPower(fRightActualPower);
}


void vPIDMotorsOutput() {
  fLeftActualPower =  pidUpdateData(dEncoderGetLeftWheelVelocity(), fLeftSetPoint);
  if(fLeftActualPower > 1) {
    fLeftActualPower = 1;
  } else if(fLeftActualPower < 0) {
    fLeftActualPower = 0;
  }
  vMotorsLeftWheelFoward();
  vMotorsLeftPower(fLeftActualPower);
  fRightActualPower = pidUpdateData(dEncoderGetRightWheelVelocity(), fRightSetPoint);
  if(fRightActualPower > 1) {
    fRightActualPower = 1;
  } else if(fRightActualPower < 0) {
    fRightActualPower = 0;
  }
  vMotorsRightWheelFoward();
  vMotorsRightPower(fRightActualPower);
}

void vPIDLineFollowerOutput(float fDirection) {

	fUpdate = pidUpdateData2(fDirection,0);
	fLeftSetPoint = fVelSetPoint*(1- fUpdate);
	fRightSetPoint = fVelSetPoint*(1 + fUpdate);
}

void vPIDIncreaseKp() {
	pidConfig2.fKp +=  0.05;
}

void vPIDDecreaseKp(){
	pidConfig2.fKp -= 0.05;
}

void vPIDIncreaseKd() {
	pidConfig2.fKd ++;
}

void vPIDIncreaseKi() {
	pidConfig2.fKi ++;
}

void vPID2SetKp(float fKp){
	pidConfig2.fKp = fKp;
}
void vPID2SetKi(float fKi){
	pidConfig2.fKi = fKi;
}
void vPID2SetKd(float fKd) {
	pidConfig2.fKd = fKd;
}
__weak void vPIDPeriodicControlTask() {}
