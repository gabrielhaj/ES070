/* ***************************************************************** */
/* File name:		 lcd.c							 			 */
/* File description: File dedicated to the hardware abstraction layer*/
/*                   related to the LCD HARDWARE based on the KS006U */
/*					 controller										 */
/* Author name:      dloubach, Gabriel Haj, Luccas Issao             */
/* Creation date:    16out2015										 */
/* Revision date:    19/06/2023                                      */
/* ***************************************************************** */

#include <lcd.h>
#include <main.h>
#include "string.h"
#include "i2c.h"
#include "odometry.h"



/* line and columns */
#define LINE0		0U
#define COLUMN0		0U

#define L0C0_BASE	0x80 /* line 0, column 0 */
#define L1C0_BASE	0xC0 /* line 1, column 0 */
#define MAX_COLUMN  15U

extern float fHeaterPWMDutyCycle;
extern float fCoolerPWMDutyCycle;
extern float fSetPointTemperature;
extern unsigned short int usCoolerSpeed;
extern pid_data_type pidConfig2;
Lcd xLcd = {0};
char cBackLight = 1;
screens xScreen = 0;
screens xActualScreen = 0;
char cStr[6] = {0};
extern positionStruct xPosition;



// Function with local scope
static void vLcdWrite2Lcd(unsigned char ucBuffer,  unsigned char cDataType);

/* ************************************************ */
/* Method name:        vLcdInitLcd                  */
/* Method description: Initialize the LCD function  */
/* Input params:       n/a                          */
/* Output params:      *hi2c=>pointer to the i2c    */
/*                     addrress=>LCD I2C address    */
/* ************************************************ */

void vLcdInitLcd(I2C_HandleTypeDef *hi2c, char cAddress)
{
	unsigned char ucTemp = 0;

	//TODO: register in a global variable the i2c handler and address
	xLcd.cAddress = cAddress;
	xLcd.hi2c = hi2c;
	// Time to the LCD's microcontroller start
	HAL_Delay(20);

	// set LCD to 4-bit mode
	ucTemp = 0x20 | LCD_BIT_E;
	HAL_I2C_Master_Transmit_IT(hi2c, cAddress<<1, &ucTemp, 1);
	HAL_Delay(1);
	ucTemp = 0x20;
	HAL_I2C_Master_Transmit_IT(hi2c, cAddress<<1, &ucTemp, 1);
	HAL_Delay(2);

	// turn-on LCD, with data length 4, 2 lines, character 5x8 */
	vLcdSendCommand(CMD_INIT_CONFIG);

	// init LCD
	vLcdSendCommand(CMD_LCDON_CURSOROFF);

	// clear LCD
	vLcdSendCommand(CMD_CLEAR);
}


/* ************************************************ */
/* Method name:        vLcdWriteData                */
/* Method description: Write data to be displayed   */
/* Input params:       ucData => char to be written */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdWriteData(unsigned char ucData)
{
	/* just a relay to send data */
	vLcdWrite2Lcd(ucData, LCD_RS_DATA);
}


/* ************************************************ */
/* Method name:        vLcdSendCommand              */
/* Method description: Write command to LCD         */
/* Input params:       ucCmd=>command to be executed*/
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdSendCommand(unsigned char ucCmd)
{
	/* just a relay to send command */
	vLcdWrite2Lcd(ucCmd, LCD_RS_CMD);
}


/* ************************************************ */
/* Method name:        vLcdSetCursor                */
/* Method description: Set cursor line and column   */
/* Input params:       cLine = LINE0..LINE1         */
/*                     cColumn = COLUMN0..MAX_COLUMN*/
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdSetCursor(unsigned char ucLine, unsigned char ucColumn)
{
	char cCommand;

	if(LINE0 == ucLine)
		/* line 0 */
		cCommand = L0C0_BASE;
	else
		/* line 1 */
		cCommand = L1C0_BASE;

	/* maximum MAX_COLUMN columns */
	cCommand += (ucColumn & MAX_COLUMN);

	// send the command to set the cursor
	vLcdSendCommand(cCommand);
}

/* ************************************************ */
/* Method name:        vLcdWriteString              */
/* Method description: Write string to be displayed */
/* Input params:       cBuffer => string to be      */
/*                     written in LCD               */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdWriteString(const char *cBuffer)
{
	while(*cBuffer)
	{
		vLcdWriteData(*cBuffer++);
	};
}


/* ************************************************ */
/* Method name:        vLCDDummyText                */
/* Method description: Write a dummy hard coded text*/
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdDummyText(void)
{
	// clear LCD
	vLcdSendCommand(CMD_CLEAR);

	// set the cursor line 0, column 1
	vLcdSetCursor(0,1);

	// send string
	vLcdWriteString("*** ES670 ***");

	// set the cursor line 1, column 0
	vLcdSetCursor(1,0);
	vLcdWriteString("Prj Sis Embarcad");
}

/* ************************************************ */
/* Method name: 	   vLcdWrite2Lcd         		*/
/* Method description: Send command or data to LCD  */
/* Input params:	   ucBuffer => byte to be send  */
/*                   cDataType => command LCD_RS_CMD*/
/*                     or data LCD_RS_DATA          */
/* Output params:	   n/a 							*/
/* ************************************************ */
static void vLcdWrite2Lcd(unsigned char ucBuffer,  unsigned char cDataType)
{
	// Vocês devem implementar essa função

	// cDataType indica se deve ser enviado um dado ou um comando ao display
	// Se cDataType for igual a LCD_RS_CMD, deve ser enviado um comando (Pino RS do display em nível baixo)
	// Se cDataType for igual a LCD_RS_DATA, deve ser enviado um dado (Pino RS do display em nível alto)

	// Vocês irão enviar o byte de dados ou comando em duas etapas
	// Na primeira etapa serão enviados os 4 bits mais significativos
	// Na segunda etapa serão enviados os 4 bits menos significativos
	// Para enviar cada bloco de 4 bits, os dados devem ser colocados nos pinos D4-D7, e deve ser dado
	// um pulso de duração de 1 ms no pino Enable do display, em seguida, o pino Enable deve voltar
	// para nível baixo. Após voltar o pino Enable para nível baixo aguarde 2 ms para enviar os próximos
	// 4 bits.
	// Para escrever um dado ou um comando o pino Write/Read do display deve estar em nível baixo

	// Na escrita vocês podem acender ou desligar o backlight do LCD (P3 do CI PCF8574). Registrem em uma
	// variável global se o backlight deve ficar aceso ou apagado. Essa variável global será atualizada nas
	// funções vLcdBacklighON() e vLcdBacklighOFF()

	unsigned char ucBufferMask = 15;
	unsigned char ucFirstHalf = ucBuffer & (ucBufferMask<<4);
	unsigned char ucSecondHalf = (ucBuffer & ucBufferMask)<<4;

	if(cDataType == LCD_RS_CMD) {
		CLEAR_BIT(ucFirstHalf,LCD_BIT_RS);
		CLEAR_BIT(ucSecondHalf,LCD_BIT_RS);
	} else if(cDataType == LCD_RS_DATA){
		SET_BIT(ucFirstHalf,LCD_BIT_RS);
		SET_BIT(ucSecondHalf,LCD_BIT_RS);
	}
	if(cBackLight) {
		SET_BIT(ucFirstHalf,LCD_BIT_BACKIGHT);
		SET_BIT(ucSecondHalf,LCD_BIT_BACKIGHT);
	}

	SET_BIT(ucFirstHalf,LCD_BIT_E);
	HAL_I2C_Master_Transmit_IT(xLcd.hi2c, xLcd.cAddress<<1, &ucFirstHalf, 1);
	HAL_Delay(1);
	CLEAR_BIT(ucFirstHalf,LCD_BIT_E);
	HAL_I2C_Master_Transmit_IT(xLcd.hi2c, xLcd.cAddress<<1, &ucFirstHalf, 1);
	HAL_Delay(2);

	SET_BIT(ucSecondHalf,LCD_BIT_E);
	HAL_I2C_Master_Transmit_IT(xLcd.hi2c, xLcd.cAddress<<1, &ucSecondHalf, 1);
	HAL_Delay(1);
	CLEAR_BIT(ucSecondHalf,LCD_BIT_E);
	HAL_I2C_Master_Transmit_IT(xLcd.hi2c, xLcd.cAddress<<1, &ucSecondHalf, 1);
	HAL_Delay(2);
}

/* ************************************************ */
/* Method name:        vLcdBackLightOn              */
/* Method description: Turns the back light On      */
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdBackLightOn(void) {
	cBackLight = 1;
}

/* ************************************************ */
/* Method name:        vLcdBackLightOff             */
/* Method description: Turns the back light Off     */
/* Input params:       n/a                          */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdBackLightOff(void) {
	cBackLight = 0;
}

char* vFtoa(float fNum, unsigned char ucType){ //Colocar ucParam como global na hora de mandar o final
	int iInt, iDec;
	if(ucType == 'h' || ucType == 'c') {
		iInt = (int)fNum;
		sprintf(cStr, "%d", iInt);
		return cStr;
	} else {
		iInt = (int)fNum;
		iDec = (int)((fNum - iInt)*100);
		sprintf(cStr, "%d,%001d", iInt, iDec);
		return cStr;
	}

}

/* ************************************************ */
/* Method name:        vLcdUpdateScreen             */
/* Method description: Go to the next screen,       */
/* 					   updating the values and      */
/* 					   building the strings to show */
/* 					   in LCD                       */
/* Input params:       Actual screen                */
/* Output params:      n/a                          */
/* ************************************************ */
void vLcdUpdateScreen(screens screen){
	char cLine1[16] = {0};
	char cLine2[16] = {0};
	char cAuxLine1[5] = {0};
	char cAuxLine2[5] = {0};
	//Clear screen, set cursor to lines
	//Build strings and send them to lcd
	switch(screen) {
	  case screen1:
		  vLcdSendCommand(CMD_CLEAR);
		  vLcdSetCursor(0,0);
		  strcat(cLine1,"Dist.:");
		  strcat(cAuxLine1,vFtoa(xPosition.dTravelledDistance,'0'));
		  strcat(cAuxLine1,"m");
		  strcat(cLine1,cAuxLine1);
		  vLcdWriteString(cLine1);
		  vLcdSetCursor(1,0);
		  strcat(cLine2,"V.Atual:");
		  strcat(cAuxLine2,vFtoa(xPosition.dMeanVelocity,'0'));
		  strcat(cAuxLine2,"v");
		  strcat(cLine2,cAuxLine2);
		  vLcdWriteString(cLine2);
		  break;
//	  case screen2:
//		  vLcdSendCommand(CMD_CLEAR);
//		  vLcdSetCursor(0,0);
//		  sprintf(cLine1,"DutyHeater:%d",(int)fHeaterPWMDutyCycle/10);
//		  strcat(cLine1,"%");
//		  vLcdWriteString(cLine1);
//		  vLcdSetCursor(1,0);
//		  sprintf(cLine2,"DutyCooler:%d",(int)fCoolerPWMDutyCycle);
//		  strcat(cLine2,"%");
//		  vLcdWriteString(cLine2);
//		  break;
//	  case screen3:
//		  vLcdSendCommand(CMD_CLEAR);
//		  vLcdSetCursor(0,0);
//		  strcat(cLine1,"Kp:");
//		  strcat(cLine1,vFtoa(pid_getKp(),'0'));
//		  vLcdWriteString(cLine1);
//		  vLcdSetCursor(1,0);
//		  strcat(cLine2,"Ki:");
//		  strcat(cLine2,vFtoa(pid_getKi(),'0'));
//		  vLcdWriteString(cLine2);
//		  break;
//	  case screen4:
//		  vLcdSendCommand(CMD_CLEAR);
//		  vLcdSetCursor(0,0);
//		  strcat(cLine1,"Cooler Speed:");
//		  vLcdWriteString(cLine1);
//		  vLcdSetCursor(1,0);
//		  sprintf(cLine2,"%d RPM",usCoolerSpeed);
//		  vLcdWriteString(cLine2);
//		  break;
 }
}

