/*
 * pid.c
 *
 *  Created on: Sep 29, 2023
 *      Author: aluno
 */
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
#include "lineSensors.h"
#include "motors.h"


// Struct used to store the PID configuration parameters
pid_data_type pidConfig;
// Counter used to control the integration error window
unsigned short usIntegratorCount = 0;
// Buffer used to store the errors to generate the integral error
float fIntegratorBuffer[INTEGRATOR_MAX_SIZE]={0};

float fError, fDifference, fOut;

extern float fSetPointTemperature;


/* ************************************************ */
/* Method name:        pid_init                     */
/* Method description: Initialize the PID controller*/
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void pid_init(float fKp, float fKi, float fKd, unsigned short usIntSizeMs, float fOutputSaturation)
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

	pidConfig.fOutputSaturation = fOutputSaturation;
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
float pidUpdateData()
{
	//float fError, fDifference, fOut, fPosition;

	// Proportional error
	/*
	if(iPIDGetLineSensor() != 10) {
		fError = iPIDGetLineSensor();
	}
	*/
	if(iPIDGetLineSensor() != 10){
		fError = iPIDGetLineSensor();
	}


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

	if(fOut > pidConfig.fOutputSaturation)
	{
		fOut = pidConfig.fOutputSaturation;
	}
	else if (fOut < -pidConfig.fOutputSaturation)
	{
		fOut = -pidConfig.fOutputSaturation;
	}
	//fError -= fOut;
	return fOut;
}

void vPIDActuatorSetValue(float fActuatorValue) {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsRightPower(0.6 - fActuatorValue);
	vMotorsLeftPower(0.6 + fActuatorValue);
}



int iPIDGetLineSensor() {
	lineSensorsStateStruct xS = {0};
	int iValue = 10;
	xS = xLineSensorsGetState();
	if(xS.mostLeftSensor) {
		iValue = -2;
	} else if(xS.leftSensor) {
		iValue = -1;
	} else if(xS.middleSensor) {
		iValue = 0;
	} else if(xS.rightSensor) {
		iValue = 1;
	} else if(xS.mostRightSensor) {
		iValue = 2;
	}
	return iValue;
}

__weak void vPIDPeriodicControlTask() {}


