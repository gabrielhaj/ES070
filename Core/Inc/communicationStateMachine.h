/*
 * uart.h
 *
 *  Created on: Nov 23, 2023
 *      Author: eduar
 */

#ifndef INC_COMMUNICATIONSTATEMACHINE_H_
#define INC_COMMUNICATIONSTATEMACHINE_H_

#include <stm32g4xx.h>               /* STM32G4xx Definitions              */


void vCommunicationStateMachineProcessStateMachine(unsigned char ucByte);

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void vReturnParam(unsigned char ucParamReturn);
void vSetParam(unsigned char ucParamSet, char* cValue);
char* vFtoa(float fNum, unsigned char ucType);
/*extern void SER_Init      (void);
extern int  SER_GetChar   (void);
extern int SER_PutChar 		(int c);*/

#endif /* INC_COMMUNICATIONSTATEMACHINE_H_ */
