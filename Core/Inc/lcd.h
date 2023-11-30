/* ***************************************************************** */
/* File name:        lcd.h                                           */
/* File description: Header file containing the functions/methods    */
/*                   interfaces for handling the LCD hardware from   */
/*                   the hardware kit                                */
/* Author name:      dloubach, bacurau, Gabriel Haj, Luccas Yonei    */
/* Creation date:    16out2015                                       */
/* Revision date:    19/06/2023                                      */
/* ***************************************************************** */

#ifndef SOURCES_LCD_H_
#define SOURCES_LCD_H_

#include "stm32g4xx_hal.h"

/* lcd basic commands list */
#define CMD_INIT_CONFIG         0x28 /* data length 4, 2 lines, character 5x8 */
#define CMD_LCDON_CURSORON      0x0F /* display on, cursor on, blink on */
#define CMD_LCDOFF_CURSOROFF    0x08 /* display off, cursor off */
#define CMD_LCDON_CURSOROFF     0x0C /* display on, cursor off */
#define CMD_CLEAR               0x01 /* clear display */
#define CMD_RETURN_HOME         0x02 /* return cursor to top left */
#define CMD_DECREMENT_CURSOR    0x04 /* shift cursor to left */
#define CMD_INCREMENT_CURSOR    0x06 /* shift cursor to right */


/* RS and Enable definitions*/
#define LCD_RS_DATA				1U
#define LCD_RS_CMD   			0U
#define LCD_ENABLED				1U
#define LCD_DISABLED			0U


/* LCD Bits on the PCF8574 IC */
#define LCD_BIT_RS              01U
#define LCD_BIT_RW              0x02U
#define LCD_BIT_E               0x04U
#define LCD_BIT_BACKIGHT        0x08U

/*LCD Data Structure*/
typedef struct{
	I2C_HandleTypeDef *hi2c;
	char cAddress;
}Lcd;

/*LCD screens enumeration*/
typedef enum {screen1,screen2,screen3,screen4} screens;

/* ************************************************ */
/* Method name:        vLcdInitLcd                  */
/* Method description: Initialize the LCD function  */
/* Input params:       n/a                          */
/* Output params:      *hi2c=>pointer to the i2c    */
/*                     cAddress=>LCD I2C address    */
/* ************************************************ */
void vLcdInitLcd(I2C_HandleTypeDef *hi2c, char cAddress);


/* ************************************************ */
/* Method name:        vLcdWriteData                */
/* Method description: Write data to be displayed   */
/* Input params:       ucData => char to be written */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdWriteData(unsigned char ucData);


/* ************************************************ */
/* Method name:        vLcdSendCommand              */
/* Method description: Write command to LCD         */
/* Input params:       ucCmd=>command to be executed*/
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdSendCommand(unsigned char ucCmd);


/* ************************************************ */
/* Method name:        vLcdWriteString              */
/* Method description: Write string to be displayed */
/* Input params:       cBuffer => string to be      */
/*                     written in LCD               */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdWriteString(const char *cBuffer);


/* ************************************************ */
/* Method name:        vLcdSetCursor                */
/* Method description: Set cursor line and column   */
/* Input params:       cLine = LINE0..LINE1         */
/*                     cColumn = COLUMN0..MAX_COLUMN*/
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdSetCursor(unsigned char ucLine, unsigned char ucColumn);

/* ************************************************ */
/* Method name:        vLcdBackLightOn              */
/* Method description: Turns the back light On      */
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdBackLightOn(void);

/* ************************************************ */
/* Method name:        vLcdBackLightOff             */
/* Method description: Turns the back light Off     */
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdBackLightOff(void);

/* ************************************************ */
/* Method name:        vLcdUpdateScreen             */
/* Method description: Go to the next screen        */
/* Input params:       Actual screen                */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdUpdateScreen(screens screen);

void vLcdDummyText(void);

char* vFtoa(float fNum, unsigned char ucType);

#endif /* SOURCES_LCD_H_ */
