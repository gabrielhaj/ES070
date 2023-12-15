// ********************************************** //
// File name:                            buzzer.c //
// File description:    This file is the header   //
//                 of heater and cooler functions //
//                                                //
//                                                //
// Author names: Gabriel Haj and Luccas Yonei     //
// Creation date: 11/05/2023                      //
// Revision date: 11/05/2023					  //
// ********************************************** //

#include "buzzer.h"
extern TIM_HandleTypeDef htim20;
TIM_HandleTypeDef *pBuzzer;
TIM_HandleTypeDef *pCounterBuzzer;
unsigned short int usBuzzerPeriod;
unsigned short int usBuzzerFrequency;
extern unsigned int uiTimeCounter;


void vBuzzerConfig(unsigned short int usFrequency, unsigned short int usPeriod, TIM_HandleTypeDef *htim){
	pBuzzer = htim;
	//pCounterBuzzer = &htim5;
	usBuzzerFrequency = usFrequency;
	usBuzzerPeriod = usPeriod;
}

void vBuzzerPlay(void){ //If you press button Enter, enters in this function
//	//HAL_TIM_Base_Start_IT(pCounterBuzzer);
	HAL_TIM_PWM_Start(pBuzzer, TIM_CHANNEL_1);
	pBuzzer->Instance->CCR1 = (uint32_t)(0.5*1000);
	pBuzzer->Instance->ARR = (uint32_t)(usBuzzerFrequency);
}

void vBuzzerStop(void) {
  HAL_TIM_PWM_Stop(pBuzzer, TIM_CHANNEL_1);
}


