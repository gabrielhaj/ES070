// ********************************************** //
// File name:                   buzzer.h //
// File description:    This file is the header   //
//                 of heater and cooler functions //
//                                                //
//                                                //
// Author names: Gabriel Haj and Luccas Yonei     //
// Creation date: 11/05/2023                      //
// Revision date: 11/05/2023					  //
// ********************************************** //

#ifndef BUZZER_H
#define BUZZER_H
#include "buttons.h"
#include "tim.h"


void vBuzzerConfig(unsigned short int usFrequency, unsigned short int usPeriod, TIM_HandleTypeDef *htim);
void vBuzzerPlay(void);
void vBuzzerStop(void);

#endif
