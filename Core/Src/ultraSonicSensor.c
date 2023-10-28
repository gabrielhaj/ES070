/*
 * ultraSonicSensor.c
 *
 *  Created on: 1 de out de 2023
 *      Author: Issao
 */


#include "ultraSonicSensor.h"

ultraSonicSensorStruct xUltraSonicSensor = {0};
char cFlag = 0;
TIM_HandleTypeDef *pUltraSonicTriggerCallback;

/* Inicializa o sensor ultrassônico */
void vUltrasonicSensorInit(TIM_HandleTypeDef *htim) {
	xUltraSonicSensor.htim = htim;
	xUltraSonicSensor.uiSendTime = 0;
	xUltraSonicSensor.uiReceiveTime = 0;
	xUltraSonicSensor.uiOVC = 0;
    HAL_TIM_IC_Start_IT(xUltraSonicSensor.htim, TIM_CHANNEL_2);
}

/* Envia um pulso */
void vUltrasonicSensorSendTriggerPulse(void) {
    if(cFlag == 1) {
    	HAL_GPIO_WritePin(GPIOB, Ultra_All_Trig_PWM, GPIO_PIN_RESET); // Volte o pino TRIGGER para nível baixo
    	cFlag = 0;
    	//To-do : Define timer to use on trigger with 10 microseconds width
    	HAL_TIM_Base_Stop_IT(htim);
    } else {
        HAL_GPIO_WritePin(GPIOB, Ultra_All_Trig_PWM, GPIO_PIN_SET); // Define o pino TRIGGER em nível alto
        HAL_TIM_Base_Start_IT(htim);
        cFlag = 1;
    }

}

/* Retorna a distância de um sensor (em cm) */
float fUltrasonicSensorGetDistanceCm(ultraSonicSensorStruct xUltraSonicSensor) {
	uint32_t uiEchoDuration = 0;
	float fDistanceCm = 0;
	uiEchoDuration = xUltraSonicSensor.uiReceiveTime + (xUltraSonicSensor.uiOVC*65536) - xUltraSonicSensor.uiSendTime;
	fDistanceCm = uiEchoDuration*VELOCIDADESOM/2;
	xUltraSonicSensor.uiReceiveTime = 0;
	xUltraSonicSensor.uiSendTime = 0;

	return fDistanceCm;
}

/* Callback do tempo */
void vUltraSonicSensorCallback(TIM_HandleTypeDef *htim) {
	if (xUltraSonicSensor.uiSendTime == 0) {
		//Utilizamos o CCR2 porque é o do sensor frontal
		xUltraSonicSensor.uiSendTime = xUltraSonicSensor.htim->Instance->CCR2; // tempo inicial
		xUltraSonicSensor.uiOVC = 0;
		vUltrasonicSensorSendTriggerPulse();
	} else {
		xUltraSonicSensor.uiReceiveTime = xUltraSonicSensor.htim->Instance->CCR2; // tempo final
		xUltraSonicSensor.fDistance = fUltrasonicSensorGetDistanceCm(xUltraSonicSensor);
		vUltrasonicSensorSendTriggerPulse();

	}

}

void vUltraSonicSensorOverclockCallback(TIM_HandleTypeDef *htim) {
	xUltraSonicSensor.uiOVC ++;
}

