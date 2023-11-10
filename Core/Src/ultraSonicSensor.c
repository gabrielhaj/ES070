/*
 * ultraSonicSensor.c
 *
 *  Created on: 1 de out de 2023
 *      Author: Issao
 */


#include "ultraSonicSensor.h"

ultraSonicSensorStruct xUltraSonicSensor = {0};
char cFlag = 0;
char cFlag1 = 0;
TIM_HandleTypeDef *pUltraSonicTriggerCallback;
TIM_HandleTypeDef *pPWMTrigger = &htim20;

/* Inicializa o sensor ultrassônico */
void vUltrasonicSensorInit(TIM_HandleTypeDef *htim) {
	xUltraSonicSensor.htim = htim;
	xUltraSonicSensor.uiSendTime = 0;
	xUltraSonicSensor.uiReceiveTime = 0;
	xUltraSonicSensor.uiOVC = 0;
	HAL_TIM_Base_Start_IT(xUltraSonicSensor.htim);
    HAL_TIM_IC_Start_IT(xUltraSonicSensor.htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_PWM_Start(pPWMTrigger,TIM_CHANNEL_1);
	pPWMTrigger->Instance->CCR1 = 10;
}

/* Envia um pulso */
void vUltrasonicSensorSendTriggerPulse(TIM_HandleTypeDef* htim) {
    if(cFlag1 == 1) {
    	HAL_GPIO_WritePin(GPIOB, Ultra_All_Trig_PWM_Pin, GPIO_PIN_RESET); // Volte o pino TRIGGER para nível baixo
    	cFlag1 = 0;
    	//To-do : Define timer to use on trigger with 10 microseconds width
    	HAL_TIM_Base_Stop_IT(htim);
    } else {
        HAL_GPIO_WritePin(GPIOB, Ultra_All_Trig_PWM_Pin, GPIO_PIN_SET); // Define o pino TRIGGER em nível alto
        HAL_TIM_Base_Start_IT(htim);
        cFlag1 = 1;
    }

}

/* Retorna a distância de um sensor (em cm) */
double fUltrasonicSensorGetDistanceCm(ultraSonicSensorStruct xUltraSonicSensor) {
	uint32_t uiEchoDuration = 0;
	double dDistanceCm = 0;
	uiEchoDuration = (xUltraSonicSensor.uiReceiveTime + (xUltraSonicSensor.uiOVC*65536) - xUltraSonicSensor.uiSendTime);

	dDistanceCm = uiEchoDuration*VELOCIDADESOM/2;
	xUltraSonicSensor.uiReceiveTime = 0;
	xUltraSonicSensor.uiSendTime = 0;

	return dDistanceCm;
}

/* Callback do tempo */
void vUltraSonicSensorCallback(TIM_HandleTypeDef *htim) {
	if (cFlag == 0) {
		//Utilizamos o CCR2 porque é o do sensor frontal
		xUltraSonicSensor.uiSendTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // tempo inicial
		xUltraSonicSensor.uiOVC = 0;
		cFlag = 1;
		//vUltrasonicSensorSendTriggerPulse(htim);
	} else {
		xUltraSonicSensor.uiReceiveTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // tempo final
		//TIM3->CNT = 0;
		xUltraSonicSensor.dDistance = fUltrasonicSensorGetDistanceCm(xUltraSonicSensor);
		cFlag = 0;
		//vUltrasonicSensorSendTriggerPulse(htim);

	}

}

void vUltraSonicSensorOverclockCallback(TIM_HandleTypeDef *htim) {
	xUltraSonicSensor.uiOVC ++;
}

