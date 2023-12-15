/*
 * encoder.c
 *
 *  Created on: Sep 1, 2023
 *      Author: aluno
 */
#include "communicationStateMachine.h"
#include "usart.h"
#include "stm32g4xx.h"               /* STM32G4xx Definitions              */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motors.h"
#include "odometry.h"
#include "lineFollower.h"
#include "manual.h"
//Possible states of the state machine
#define IDDLE '0'
#define READY '1'
#define GET '2'
#define SET '3'
#define PARAM '4'
#define VALUE '5'

#define PI 3.1415

//Maximum lenght of string of values ("100,00")
#define MAX_VALUE_LENGHT 6

//Current state of the state machine
unsigned char ucMachineState = IDDLE;

//Char sent via UART
unsigned char ucData;

//Variable to save the parameter received by the state machine
static unsigned char ucParam;

//Counter for positioning on the string
unsigned char ucValueCount;

//String to save the char values to be send to vSetParam
static char cValue[MAX_VALUE_LENGHT+1];

//The whole answer string sent from successful GET
char sMessage[MAX_VALUE_LENGHT + 5 + 2] = {0};

//The beggining of the answer string
char sData[5] = {"-A \0"};

//The end of the answer string
char sData2[2] = {"!\0"};

char cNextParam[16] = {"Dvmxytbpidgelzoh"};

//The whole answer string sent from successful GET (all parameters at once)
char sAllMessage[(MAX_VALUE_LENGHT+5+2)*6] = {0};



char cFlagAll = 0;

extern int iINextParam;


//PID config
extern pid_data_type pidConfig;
extern float fdummyData[3];
extern float a;
extern float fLeftSetPoint;
extern float fRightSetPoint;
extern float fVelSetPoint;

extern char cLigaBuzzer;


/* ************************************************************************************* */
/* Method name:        vCommunicationStateMachineProcessStateMachine                     */
/* Method description: The state machine logic and flow implementation                   */
/* Input params:       ucByte: byte received via UART                                    */
/* Output params:      n/a                                                               */
/* ************************************************************************************* */
void vCommunicationStateMachineProcessStateMachine(unsigned char ucByte) {
	if('-' == ucByte) {
		ucMachineState = READY;
	} else {
		if(IDDLE != ucMachineState) {
			switch(ucMachineState) {
				case READY:
					switch(ucByte) {
						case 'g' :
							ucMachineState = GET;
							break;
						case 's':
							ucMachineState = SET;
							break;
						default:
							ucMachineState = IDDLE;
					}
					break;
				case GET:
					if('D' == ucByte || 'v' == ucByte || 'm' == ucByte || 'x' == ucByte || 'y' == ucByte || 't' == ucByte || 'b' == ucByte
							|| 'p' == ucByte || 'i' == ucByte || 'd' == ucByte || 'g' == ucByte || 'e' == ucByte || 'l' == ucByte
							|| 'z' == ucByte || 'a' == ucByte || 'f' == ucByte || 'o' == ucByte || 'h' == ucByte) {
						ucParam = ucByte;
						ucMachineState = PARAM;
					} else {
						ucMachineState = IDDLE;
					}
					break;
				case SET:
					if('p' == ucByte || 'i' == ucByte || 'd' == ucByte || 'g' == ucByte || 'e' == ucByte || 'l' == ucByte
							|| 'z' == ucByte || 'o' == ucByte || 'f' == ucByte || 'h' == ucByte || 'F' == ucByte || 'B' == ucByte
							 || 'L' == ucByte || 'R' == ucByte) {
						ucParam = ucByte;
						ucValueCount = 0;
						ucMachineState = VALUE;
					} else {
						ucMachineState = IDDLE;
					}
					break;
				case PARAM:
					if('!' == ucByte) {
						vReturnParam(ucParam);
					}
					ucMachineState = IDDLE;
					break;
				case VALUE:
					if((ucByte >= '0' && ucByte <= '9') || ',' == ucByte || '.' == ucByte) {
						if(',' == ucByte) {
							ucByte = '.';
						}
						if(ucValueCount < MAX_VALUE_LENGHT){
							cValue[ucValueCount++] = ucByte;
						}
					}
					 else {
						if('!' == ucByte) {
							cValue[ucValueCount] = '\0';
							vSetParam(ucParam,cValue);
						}
						ucMachineState = IDDLE;
					}
					break;
			}
		}
	}
}




/* ************************************************************************************* */
/* Method name:        vReturnParam                                                      */
/* Method description: Transmit via UART the parameters requested by GET. Those can be:  */
/* 						'D' for travelled distance;                                      */
/* 						'v' for actual velocity;                                         */
/* 						'm' for mean velocity;                                           */
/* 						'x' for x coordinate;                                            */
/* 						'y' for y coordinate;                                            */
/* 						't' for theta coordinate;                                        */
/* 						'b' for current battery;                                         */
/* 						'p' for motor kp;                                                */
/* 						'i' for motor ki;                                                */
/* 						'd' for motor kd;                                                */
/* 						'g' for linefollower kp;                                         */
/* 						'e' for linefollower ki;                                         */
/* 						'l' for linefollower kd;                                         */
/* 						'z' for on/off motors;                                           */
/* 						'a' for on/off linefollower;                                     */
/* 						'f' for on/off buzzer;                                           */
/* 						'a' for all previous parameters.                                 */
/* Input params:       ucParamReturn: selector of which parameter is requested           */
/* Output params:      n/a                                                               */
/* ************************************************************************************* */
void vReturnParam(unsigned char ucParamReturn) {
	int iSize = 1;
	memset(sMessage,0,sizeof(sMessage));
	switch(ucParamReturn) {
		case 'D':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetTravelledDistance(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'v':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetActualVelocity()/100, 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'm':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetMeanVelocity()/100, 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'x':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetXCoordinate(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'y':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetYCoordinate(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 't':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetThetaCoordinate()*180/PI, 't'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'b':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"00,00");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'p':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(pid_getKp(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'i':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(pid_getKi(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'd':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(pid_getKd(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'g':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(fPID2GetKp(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'e':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(fPID2GetKi(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'l':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(fPID2GetKd(), 'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'z':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(cMotorsGetState(),'h'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'o':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(cLineFollowerGetState(),'h'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'h':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,vFtoa(fPIDGetVelSetPoint()*100,'o'));
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'a':
			char cAux;
			cFlagAll = 1;
			cAux = cNextParam[iINextParam];
			vReturnParam(cAux);
			break;
			}
			//break;
	}


/* ************************************************************************************* */
/* Method name:        vSetParam                                                         */
/* Method description: Set parameters of the system                                      */
/* Input params:       ucParamSet: parameter to be set. They can be:                     */
/* 								'p' for motor kp;                                     */
/* 								'i' for motor ki;                               */
/* 								'd' for motor kd;                               */
/* 								'g' for line follower kp;                                   */
/* 								'e' for line follower ki;                                   */
/* 								'l' for line follower kd.                                   */
/* 								'z' for motors on/off.                                   */
/* 								'a' for line follower on/off.                                   */
/* 								'f' for buzina on/off.                                   */
/* 								'h' for max speed.                                   */
/* 					   cValue: pointer to the new value for the parameter                */
/* Output params:      n/a                                                               */
/* ************************************************************************************* */
void vSetParam(unsigned char ucParamSet, char* cValue){

	switch(ucParamSet) {
		case 'p':
			pid_setKp(atof(cValue));
			break;
		case 'i':
			pid_setKi(atof(cValue));
			break;
		case 'd':
			pid_setKd(atof(cValue));
			break;
		case 'g':
			vPID2SetKp(atof(cValue));
			break;
		case 'e':
			vPID2SetKi(atof(cValue));
			break;
		case 'l':
			vPID2SetKd(atof(cValue));
			break;
		case 'h':
			fVelSetPoint = atof(cValue)/100;
			break;
		case 'z':
			vMotorsSetState(atof(cValue));
			break;
		case 'o':
			vLineFollowerSetState(atof(cValue));
			break;
		case 'f':
			if(cValue) {
				vBuzzerPlay();
			} else {
				vBuzzerStop();
			}

			break;
		case 'F':
			vManualBackward(*cValue);
			break;
		case 'B':
			vManualBackward(*cValue);
			break;
		case 'L':
			vManualLeft(*cValue);
			break;
		case 'R':
			vManualRight(*cValue);
			break;

	}
}



