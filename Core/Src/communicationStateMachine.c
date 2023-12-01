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

//The whole answer string sent from successful GET (all parameters at once)
char sAllMessage[(MAX_VALUE_LENGHT+5+2)*6] = {0};

//PID config
extern pid_data_type pidConfig;
extern float fdummyData[3];
extern float a;
extern float fLeftSetPoint;
extern float fRightSetPoint;


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
					if('d' == ucByte || 'v' == ucByte || 'V' == ucByte || 'c' == ucByte || 'b' == ucByte || 'a' == ucByte) {
						ucParam = ucByte;
						ucMachineState = PARAM;
					} else {
						ucMachineState = IDDLE;
					}
					break;
				case SET:
					if('v' == ucByte ||'l' == ucByte || 'd' == ucByte || 'p' == ucByte || 'i' == ucByte || 'g' == ucByte|| 'b' == ucByte|| 'h' == ucByte|| 'r' == ucByte) {
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
/* 						't' for current temperature;                                     */
/* 						'h' for current heater duty cycle;                               */
/* 						'c' for current cooler duty cycle;                               */
/* 						'p' for current controller kp;                                   */
/* 						'i' for current controller ki;                                   */
/* 						'd' for current controller kd;                                   */
/* 						'a' for all previous parameters.                                 */
/* Input params:       ucParamReturn: selector of which parameter is requested           */
/* Output params:      n/a                                                               */
/* ************************************************************************************* */
void vReturnParam(unsigned char ucParamReturn) {
	int iSize = 1;
	memset(sMessage,0,sizeof(sMessage));
	switch(ucParamReturn) {
		case 'd':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"000");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'v':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"001");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'V':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"002");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'c':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"003");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'b':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"004");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
		case 'a':
			sData[2] = ucParamReturn;
			strcat(sMessage,sData);
			strcat(sMessage,"005");
			strcat(sMessage,sData2);
			while(sMessage[iSize] != '\0'){
				iSize ++;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sMessage, (uint16_t)iSize);
			break;
			}
			HAL_UART_Transmit_IT(&huart3, (uint8_t*)sAllMessage, (uint16_t)iSize);
			//break;
	}


/* ************************************************************************************* */
/* Method name:        vSetParam                                                         */
/* Method description: Set parameters of the system                                      */
/* Input params:       ucParamSet: parameter to be set. They can be:                     */
/* 								't' for temperature;                                     */
/* 								'h' for heater duty cycle;                               */
/* 								'c' for cooler duty cycle;                               */
/* 								'p' for controller kp;                                   */
/* 								'i' for controller ki;                                   */
/* 								'd' for controller kd.                                   */
/* 					   cValue: pointer to the new value for the parameter                */
/* Output params:      n/a                                                               */
/* ************************************************************************************* */
void vSetParam(unsigned char ucParamSet, char* cValue){

	switch(ucParamSet) {
		case 'v':
			vMotorsBreak();
			a = -1;
			break;
		case 'l':
			a = 0;
			vMotorsStart();
			vMotorsRightWheelFoward();
			vMotorsLeftWheelFoward();
			break;
		case 'p':
			pid_setKp(atof(cValue));
			break;
		case 'd':
			pid_setKd(atof(cValue));
			break;
		case 'g':
			vPID2SetKp(atof(cValue));
			break;
		case 'b':
			vPID2SetKi(atof(cValue));
			break;
		case 'h':
			vPID2SetKd(atof(cValue));
			break;
		case 'r':
			a = 1;
			vMotorsStart();
			vMotorsRightWheelFoward();
			vMotorsLeftWheelFoward();
			fLeftSetPoint = atof(cValue);
			fRightSetPoint = atof(cValue);

	}
}


