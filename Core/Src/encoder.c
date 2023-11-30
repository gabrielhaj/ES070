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
#define HOLESPERREVOLUTION 20
#define WHEELRADIUS (32.5*0.001)
#define PI 3.1415

encoderStruct xLeftEncoder = {0};
encoderStruct xRightEncoder = {0};
extern unsigned char ucLeftMotorState;
extern unsigned char ucRightMotorState;
extern int catchaD;
extern int catchaE;



void vInitEncoders(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2) {
	xLeftEncoder.htim = htim1;
	xRightEncoder.htim = htim2;
    HAL_TIM_Base_Start_IT(htim1);
	HAL_TIM_Base_Start_IT(htim2);
    HAL_TIM_IC_Start_IT(htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(htim2, TIM_CHANNEL_1);
}


void vEncoderCallback(TIM_HandleTypeDef* htim) {
    if(htim == xLeftEncoder.htim) {
    	if(xLeftEncoder.ucState == IDLE) {
    	        xLeftEncoder.uiT1 = htim->Instance->CCR1;
    	        xLeftEncoder.uiTIM_OVC = 0;
    	        xLeftEncoder.ucState = DONE;
    	    }
    	    else if(xLeftEncoder.ucState == DONE) {
    	    	xLeftEncoder.uiT2 = htim->Instance->CCR1;
    	    	xLeftEncoder.iTicks = (xLeftEncoder.uiT2 + (xLeftEncoder.uiTIM_OVC * 65536)) - xLeftEncoder.uiT1;
    	        //Why >1 if? Because for some reason, the elapsedcallback is not called
    	        //(Overflowcallback) and iTicks becomes a negative number
    	        //also, Sometimes the inputcapture mechanis is triggered twice
    	        // and the difference between two ticks is 1 causing an interference
    	        //in the measurements. So this if makes those unstables behaviours
    	        //"filtered" by just keeping the old measurement
    	        //This is not a big problem because the measurements are quite
    	        //close between each other. But for sure this is not the "correct"
    	        //way to solve this problem
    	    	xLeftEncoder.dFreq = (double)(CLKFREQUENCY/xLeftEncoder.iTicks);

    	        if(xLeftEncoder.dFreq*2*WHEELRADIUS*PI/HOLESPERREVOLUTION < 0.8 ) {
    	        	xLeftEncoder.dVel = xLeftEncoder.dFreq*2*WHEELRADIUS*PI/HOLESPERREVOLUTION;
    	        }

    	        xLeftEncoder.ucState = IDLE;
    	    }
    } else if (htim == xRightEncoder.htim) {
    	if(xRightEncoder.ucState == IDLE) {
    	        xRightEncoder.uiT1 = htim->Instance->CCR1;
    	        xRightEncoder.uiTIM_OVC = 0;
    	        xRightEncoder.ucState = DONE;
    	        catchaD ++;
    	    }
    	    else if(xRightEncoder.ucState == DONE) {
    	    	catchaD++;
    	    	xRightEncoder.uiT2 = htim->Instance->CCR1;
    	    	xRightEncoder.iTicks = (xRightEncoder.uiT2 + (xRightEncoder.uiTIM_OVC * 65536)) - xRightEncoder.uiT1;
    	        //Why >1 if? Because for some reason, the elapsedcallback is not called
    	        //(Overflowcallback) and iTicks becomes a negative number
    	        //also, Sometimes the inputcapture mechanis is triggered twice
    	        // and the difference between two ticks is 1 causing an interference
    	        //in the measurements. So this if makes those unstables behaviours
    	        //"filtered" by just keeping the old measurement
    	        //This is not a big problem because the measurements are quite
    	        //close between each other. But for sure this is not the "correct"
    	        //way to solve this problem
    	    	xRightEncoder.dFreq = (double)(CLKFREQUENCY/xRightEncoder.iTicks);

    	        if(xRightEncoder.dFreq*2*WHEELRADIUS*PI/HOLESPERREVOLUTION < 0.8 ) {
    	        	xRightEncoder.dVel = xRightEncoder.dFreq*2*WHEELRADIUS*PI/HOLESPERREVOLUTION;
    	        }

    	        xRightEncoder.ucState = IDLE;

    	    }
    }
}

void vEncoderOverflowCallback(TIM_HandleTypeDef* htim) {
	if(htim == xLeftEncoder.htim) {
		xLeftEncoder.uiTIM_OVC++;
	} else if(htim == xRightEncoder.htim) {
		xRightEncoder.uiTIM_OVC++;
	}
}

double dEncoderGetLeftWheelVelocity(){
	if(ucLeftMotorState == 0 || xLeftEncoder.uiTIM_OVC > 3) {
		xLeftEncoder.dVel = 0;
	}
	return xLeftEncoder.dVel;
}

double dEncoderGetRightWheelVelocity(){
	if(ucRightMotorState == 0 || xRightEncoder.uiTIM_OVC > 3) {
		xRightEncoder.dVel = 0;
	}
	return xRightEncoder.dVel;
}

