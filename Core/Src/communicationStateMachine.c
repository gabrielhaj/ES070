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

//Possible states of the state machine
#define IDDLE '0'
#define READY '1'
#define GET '2'
#define SET '3'
#define PARAM '4'
#define VALUE '5'

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
char sData[5] = {"-a  \0"};

//The end of the answer string
char sData2[4] = {"!\n\r\0"};

char cNextParam[15] = {"Dvmxytbpidgelzo"};

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
							|| 'p' == ucByte || 'i' == ucByte || 'd' == ucByte || 'g' == ucByte || 'e' == ucByte || 'l' == ucByte || 'z' == ucByte || 'a' == ucByte || 'f' == ucByte || 'o' == ucByte) {
						ucParam = ucByte;
						ucMachineState = PARAM;
					} else {
						ucMachineState = IDDLE;
					}
					break;
				case SET:
					if('p' == ucByte || 'i' == ucByte || 'd' == ucByte || 'g' == ucByte || 'e' == ucByte || 'l' == ucByte || 'z' == ucByte || 'a' == ucByte || 'f' == ucByte || 'h' == ucByte) {
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
			//sData[2] = ucParamReturn;
			//strcat(sMessage,sData);
			strcat(sMessage,vFtoa(dOdometryGetTravelledDistance(), 'o'));
			strcat(sMessage,'\0');
			//strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'v':
			strcat(sMessage,vFtoa(dOdometryGetActualVelocity(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"001");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'm':
			strcat(sMessage,vFtoa(dOdometryGetMeanVelocity(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"002");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'x':
			strcat(sMessage,vFtoa(dOdometryGetXCoordinate(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"003");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'y':
			strcat(sMessage,vFtoa(dOdometryGetYCoordinate(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 't':
			strcat(sMessage,vFtoa(dOdometryGetThetaCoordinate(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"005");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'b':
			//strcat(sMessage,'100');
			//strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'p':
			strcat(sMessage,vFtoa(pid_getKp(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'i':
			strcat(sMessage,vFtoa(pid_getKi(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'd':
			strcat(sMessage,vFtoa(pid_getKd(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'g':
			strcat(sMessage,vFtoa(fPID2GetKp(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'e':
			strcat(sMessage,vFtoa(fPID2GetKi(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'l':
			strcat(sMessage,vFtoa(fPID2GetKd(), 'o'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'z':
			strcat(sMessage,vFtoa(cMotorsGetState(),'h'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'o':
			strcat(sMessage,vFtoa(cLineFollowerGetState(),'h'));
			strcat(sMessage,'\0');
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'f':
//			sData[2] = ucParamReturn;
//			strcat(sMessage,sData);
//			strcat(sMessage,"004");
//			strcat(sMessage,sData2);
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
			fVelSetPoint = atof(cValue);
			break;
		case 'z':
			vMotorsSetState(atof(cValue));
			break;
		case 'a':
			vLineFollowerSetState(atof(cValue));
			break;
		case 'f':
			vBuzzerPlay();
			break;

	}
}



