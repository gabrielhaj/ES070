/*
 * motors.c
 *
 *  Created on: Sep 10, 2023
 *      Author: gabriel
 */

#include "motors.h"
#include "main.h"

TIM_HandleTypeDef *pPWMMotors;
extern unsigned char ucRightMotorState;
extern unsigned char ucLeftMotorState;

void vMotorsInit(TIM_HandleTypeDef *htim) {
	pPWMMotors = htim;
}

void vMotorsStart() {
	HAL_TIM_PWM_Start(pPWMMotors,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pPWMMotors,TIM_CHANNEL_2);
	pPWMMotors->Instance->CCR1 = 0;
	pPWMMotors->Instance->CCR2 = 0;
	ucLeftMotorState = 1;
	ucRightMotorState = 1;
}

void vMotorsStop() {
	HAL_TIM_PWM_Stop(pPWMMotors,TIM_CHANNEL_1);
	ucLeftMotorState = 0;
	HAL_TIM_PWM_Stop(pPWMMotors,TIM_CHANNEL_2);
	ucRightMotorState = 0;
}

void vMotorsLeftPower(float fLeftPower){
	if(fLeftPower > 1) {
		fLeftPower = 1;
	} else if(fLeftPower < 0) {
		fLeftPower = 0;
	}
	pPWMMotors->Instance->CCR1 = (uint32_t)(fLeftPower*pPWMMotors->Instance->ARR);
}

void vMotorsRightPower(float fRightPower){
	if(fRightPower > 1) {
		fRightPower = 1;
	} else if(fRightPower < 0) {
		fRightPower = 0;
	}
	pPWMMotors->Instance->CCR2 = (uint32_t)(fRightPower*pPWMMotors->Instance->ARR);
}

void vMotorsLeftWheelFoward() {
	HAL_GPIO_WritePin(LeftMotorIn1_GPIO_Port, LeftMotorIn1_Pin, SET);
	HAL_GPIO_WritePin(LeftMotorIn2_GPIO_Port, LeftMotorIn2_Pin, RESET);
}

void vMotorsLeftWheelBackwards() {
	HAL_GPIO_WritePin(LeftMotorIn1_GPIO_Port, LeftMotorIn1_Pin, RESET);
	HAL_GPIO_WritePin(LeftMotorIn2_GPIO_Port, LeftMotorIn2_Pin, SET);
}

void vMotorsRightWheelFoward() {
	HAL_GPIO_WritePin(RightMotorIn3_GPIO_Port, RightMotorIn3_Pin, SET);
	HAL_GPIO_WritePin(RightMotorIn4_GPIO_Port, RightMotorIn4_Pin, RESET);
}


void vMotorsRightWheelBackwards() {
	HAL_GPIO_WritePin(RightMotorIn3_GPIO_Port, RightMotorIn3_Pin, RESET);
	HAL_GPIO_WritePin(RightMotorIn4_GPIO_Port, RightMotorIn4_Pin, SET);
}

void vMotorsBreak() {
	HAL_GPIO_WritePin(LeftMotorIn1_GPIO_Port, LeftMotorIn1_Pin, RESET);
	HAL_GPIO_WritePin(LeftMotorIn2_GPIO_Port, LeftMotorIn2_Pin, RESET);
	HAL_GPIO_WritePin(RightMotorIn3_GPIO_Port, RightMotorIn3_Pin, RESET);
	HAL_GPIO_WritePin(RightMotorIn4_GPIO_Port, RightMotorIn4_Pin, RESET);
}

char cMotorsGetState() {
	if(ucLeftMotorState && ucRightMotorState) {
		return 1;
	} else {
		return 0;
	}
}

void vMotorsSetState(char cNewState) {
	if(cNewState) {
		ucLeftMotorState = 1;
		ucRightMotorState = 1;
		vMotorsStart();
	}
	else if(!cNewState) {
		ucLeftMotorState = 0;
		ucRightMotorState = 0;
		vMotorsStop();
	}
}




