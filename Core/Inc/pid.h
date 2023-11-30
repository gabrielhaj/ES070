/* ***************************************************************** */
/* File name:        pid.h                                           */
/* File description: Header file containing the functions/methods    */
/*                   interfaces for handling the PID                 */
/* Author name:      julioalvesMS, IagoAF, rBacurau                  */
/* Creation date:    21jun2018                                       */
/* Revision date:    21mai2023                                       */
/* ***************************************************************** */

#ifndef SOURCES_CONTROLLER_PID_H_
#define SOURCES_CONTROLLER_PID_H_

#define UPDATE_RATE_MS      1
#define INTEGRATOR_MAX_SIZE 100

typedef struct pid_data_type {
	float fKp, fKi, fKd;         		// PID gains
	float fError_previous;       		// used in the derivative
	float fError_sum;            		// integrator cumulative error
	unsigned short usIntegratorSize; 	//integrator window size
	float fOutputUpperSaturation;            // output saturation
	float fOutputLowerSaturation;
} pid_data_type;


/* ************************************************ */
/* Method name:        pid_init                     */
/* Method description: Initialize the PID controller*/
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void pid_init(float fKp, float fKi, float fKd, unsigned short usIntegratorSize, float fOutputUpperSaturation, float fOutputLowerSaturation);
void pid_init2(float fKp, float fKi, float fKd, unsigned short usIntegratorSize, float fOutputUpperSaturation, float fOutputLowerSaturation);


/* ************************************************** */
/* Method name:        pid_setKp                      */
/* Method description: Set a new value for the PID    */
/*                     proportional constant          */
/* Input params:       fKp: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setKp(float fKp);


/* ************************************************** */
/* Method name:        pid_getKp                      */
/* Method description: Get the value from the PID     */
/*                     proportional constant          */
/* Input params:       n/a                            */
/* Output params:      float: Value                   */
float pid_getKp(void);


/* ************************************************** */
/* Method name:        pid_setKi                      */
/* Method description: Set a new value for the PID    */
/*                     integrative constant           */
/* Input params:       fKi: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setKi(float fKi);


/* ************************************************** */
/* Method name:        pid_getKi                      */
/* Method description: Get the value from the PID     */
/*                     integrative constant           */
/* Input params:       n/a                            */
/* Output params:      float: Value                   */
/* ************************************************** */
float pid_getKi(void);


/* ************************************************** */
/* Method name:        pid_setKd                      */
/* Method description: Set a new value for the PID    */
/*                     derivative constant            */
/* Input params:       fKd: New value                 */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setKd(float fKd);


/* ************************************************** */
/* Method name:        pid_getKd                      */
/* Method description: Get the value from the PID     */
/*                     derivative constant            */
/* Input params:       n/a                            */
/* Output params:      float: Value                   */
/* ************************************************** */
float pid_getKd(void);

/* ************************************************** */
/* Method name:        pid_setIntegratorWindow        */
/* Method description: Set a new value for the        */
/*                     integrator window (in ms)      */
/* Input params:       usIntSizeMs: New value (in ms) */
/* Output params:      n/a                            */
/* ************************************************** */
void pid_setIntegratorWindow (unsigned short usIntSizeMs);

/* ************************************************** */
/* Method name:        pid_getIntegratorWindow        */
/* Method description: Get the value from the         */
/*                     integrator window (in ms)      */
/* Input params:       n/a                            */
/* Output params:      usIntSizeMs: Value (in ms)     */
/* ************************************************** */
unsigned short pid_getIntegratorWindow (void);

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
float pidUpdateData(float fSensorValue, float fReferenceValue);
float pidUpdateData2(float fSensorValue, float fReferenceValue);
void vPIDLineFollowerOutput(float fDirection);

void vPIDPeriodicControlTask();
float fPIDGetSetPointTemperature();
void vPIDMotorsOutput();

void vPIDIncreaseKp();
void vPIDDecreaseKp();

void vPIDIncreaseKd();

void vPIDIncreaseKi();

void vPID2SetKp(float fKp);
void vPID2SetKi(float fKi);
void vPID2SetKd(float fKd);



#endif /* SOURCES_CONTROLLER_PID_H_ */
