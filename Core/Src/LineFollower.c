/*
 * LineFollower.c
 *
 *  Created on: Sep 11, 2023
 *      Author: gabriel
 */

#include "motors.h"
#include "LineFollower.h"

NaPista SensorMaisEsquerda;
NaPista SensorEsquerda;
NaPista SensorMeio;
NaPista SensorDireita;
NaPista SensorMaisDireita;
float fVmax = 1;
float fVsoft = 0.75;
char flag = 0;



void LineTracker() {
	  SensorMaisEsquerda = HAL_GPIO_ReadPin(LightSensor1_GPIO_Port,LightSensor1_Pin);
	  SensorEsquerda = HAL_GPIO_ReadPin(LightSensor2_GPIO_Port,LightSensor2_Pin);
	  SensorMeio = HAL_GPIO_ReadPin(LightSensor3_GPIO_Port,LightSensor3_Pin);
	  SensorDireita = HAL_GPIO_ReadPin(LightSensor4_GPIO_Port,LightSensor4_Pin);
	  SensorMaisDireita = HAL_GPIO_ReadPin(LightSensor5_GPIO_Port,LightSensor5_Pin);
	if(flag && SensorMaisDireita == Branco && SensorDireita == Branco && SensorMeio == Branco && SensorEsquerda == Branco && SensorMaisEsquerda == Branco) {
		Parar();
	} else if(!flag && SensorMaisDireita == Branco && SensorDireita == Branco && SensorMeio == Branco && SensorEsquerda == Branco && SensorMaisEsquerda == Branco) {
		//AndarEspessuraCruzamentoEChecarDenovo();
	} else if(SensorEsquerda == Preto && SensorDireita == Preto && SensorMeio == Branco) {
		//Manter linha reta
		AndarReto();
	} else if(SensorEsquerda == Branco && SensorDireita == Preto) {
		//VirarEsquerda
		VirarEsquerdaSuave();
	} else if(SensorEsquerda == Preto && SensorDireita == Branco) {
		VirarDireitaSuave();
	} else if(SensorMaisEsquerda == Branco) {
		VirarEsquerda();
	} else if(SensorMaisDireita == Branco) {
		VirarDireita();
	} else if(SensorMaisDireita == Preto && SensorDireita == Preto && SensorMeio == Preto && SensorEsquerda == Preto && SensorMaisEsquerda == Preto) {
		Parar();
	}
}

void AndarReto() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVmax);
	vMotorsRightPower(fVmax);
}

void VirarEsquerdaSuave() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVsoft);
	vMotorsRightPower(fVmax);
}

void VirarDireitaSuave() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVmax);
	vMotorsRightPower(fVsoft);
}

void VirarEsquerda() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(0);
	vMotorsRightPower(fVmax);
}

void VirarDireita() {
	vMotorsLeftWheelFoward();
	vMotorsRightWheelFoward();
	vMotorsLeftPower(fVmax);
	vMotorsRightPower(0);
}

void Parar() {
	vMotorsBreak();
}
