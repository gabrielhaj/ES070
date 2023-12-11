/*
 * buttons.h
 *
 *  Created on: Sep 16, 2023
 *      Author: gabriel
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

typedef enum {
	off,
	on
} buttonState;


typedef struct {
	buttonState leftBt;
	buttonState rightBt;
	buttonState upBt;
	buttonState downBt;
	buttonState enterBt;
} buttons;

buttons xReadButtons();

#endif /* INC_BUTTONS_H_ */
