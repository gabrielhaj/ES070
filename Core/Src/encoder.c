/*
 * encoder.c
 *
 *  Created on: Sep 1, 2023
 *      Author: aluno
 */

#include "tim.h"


#define IDLE   0
#define DONE   1
#define CLKFREQUENCY  1000000UL

unsigned char ucState = IDLE;
volatile uint32_t uiT1 = 0;
volatile uint32_t uiT2 = 0;
volatile int32_t iTicks = 0;
unsigned int uiTIM2_OVC = 0;
unsigned int uiFreq = 0;
unsigned int uiVel = 0;


void vInitEncoders() {
    //HAL_TIM_Base_Start_IT(&htim16);

	HAL_TIM_Base_Start_IT(&htim17);
    //HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
}


void vEncoderCallback(TIM_HandleTypeDef* htim)
{
    if(ucState == IDLE)
    {
        uiT1 = htim->Instance->CCR1;
        uiTIM2_OVC = 0;
        ucState = DONE;
    }
    else if(ucState == DONE)
    {
        uiT2 = htim->Instance->CCR1;
        iTicks = (uiT2 + (uiTIM2_OVC * 65536)) - uiT1;
        //Why >1 if? Because for some reason, the elapsedcallback is not called
        //(Overflowcallback) and iTicks becomes a negative number
        //also, Sometimes the inputcapture mechanis is triggered twice
        // and the difference between two ticks is 1 causing an interference
        //in the measurements. So this if makes those unstables behaviours
        //"filtered" by just keeping the old measurement
        //This is not a big problem because the measurements are quite
        //close between each other. But for sure this is not the "correct"
        //way to solve this problem
        if(iTicks > 1 ) {
        	uiFreq = (unsigned int)(CLKFREQUENCY/iTicks);
        }
        uiVel = uiFreq*60/20;
        ucState = IDLE;
    }
}

void vEncoderOverflowCallback(TIM_HandleTypeDef* htim)
{
    uiTIM2_OVC++;
}
