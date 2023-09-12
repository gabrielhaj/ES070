/*
 * LineFollower.h
 *
 *  Created on: Sep 11, 2023
 *      Author: gabriel
 */

#ifndef INC_LINEFOLLOWER_H_
#define INC_LINEFOLLOWER_H_


typedef enum
{
  Preto = 0,
  Branco = 1
} NaPista;

void LineTracker();

void AndarReto();

void VirarEsquerdaSuave();

void VirarDireitaSuave();

void VirarEsquerda();

void VirarDireita();

void Parar();

#endif /* INC_LINEFOLLOWER_H_ */
