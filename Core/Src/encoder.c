/*
 * encoder.c
 *
 *  Created on: Sep 1, 2023
 *      Author: aluno
 */


void vInitEncoders() {
    HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim17);
    HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
}
