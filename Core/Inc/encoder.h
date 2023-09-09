/*
 * encoder.h
 *
 *  Created on: Sep 1, 2023
 *      Author: aluno
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

void vInitEncoders();

void vEncoderCallback(TIM_HandleTypeDef* htim);

void vEncoderOverflowCallback(TIM_HandleTypeDef* htim);

#endif /* INC_ENCODER_H_ */
