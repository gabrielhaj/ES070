/*
 * buttons.h
 *
 *  Created on: Sep 16, 2023
 *      Author: gabriel
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

typedef struct {
	GPIO_PinState leftBt;
	GPIO_PinState rightBt;
	GPIO_PinState upBt;
	GPIO_PinState downBt;
	GPIO_PinState enterBt;
} buttons;

buttons xReadButtons();

#endif /* INC_BUTTONS_H_ */
