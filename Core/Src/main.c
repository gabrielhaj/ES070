/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "ultraSonicSensor.h"
#include "lcd.h"
#include "FrontalSW.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern encoderStruct xRightEncoder;
extern encoderStruct xLeftEncoder;
extern buttons xBt;
extern GPIO_PinState SW;
lineSensorsStateStruct xS = {0};
positionStruct xPosition = {0};
extern TIM_HandleTypeDef *pOdometryTIM;
extern TIM_HandleTypeDef *pLineFollowerTIM;
extern int iOdometryClockDivision;
extern TIM_HandleTypeDef *pUltraSonicTriggerCallback;
extern ultraSonicSensorStruct xUltraSonicSensor;
extern unsigned char ucData;
float fdummyData[3] = {0, 0, 0};
unsigned char ucLcdAddress = 0x27;
unsigned char ucLeftMotorState = 0;
unsigned char ucRightMotorState = 0;
float fVelSetPoint = 0.3;
char cUpdateScreen = 0;
float a = -1;
float b = -1;
int catchaD = 0;
int catchaE = 0;
extern char cNextParam[17];
int iINextParam = 0;
extern char cFlagAll;
extern char cState;
int iCounter1s = 0;
char cBuzzer = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM20_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  vInitEncoders(&htim16,&htim17);
  vMotorsInit(&htim1);
  vLineFollowerInit(&htim7);
  vOdometryInit(&htim6, iOdometryClockDivision);
  HAL_UART_Receive_IT(&huart3, (uint8_t*)&ucData, 1);
  pid_init(3, 0.1 , 0.1 , 0, 1, 0.5);
  pid_init2(0.4, 0, 0, 0, 1, 1);
  vUltrasonicSensorInit(&htim3);
  vLcdInitLcd(&hi2c2,ucLcdAddress);
  vBuzzerConfig(100, 100, &htim8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(cUpdateScreen){
		  vLcdUpdateScreen();
		  cUpdateScreen = 0;

	  }




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if(htim == xRightEncoder.htim || htim == xLeftEncoder.htim){
		vEncoderCallback(htim);
	}
	else if(htim == xUltraSonicSensor.htim){
		vUltraSonicSensorCallback(htim);
		if(xUltraSonicSensor.dDistance < 20) {
		  //vBuzzerPlay();
		} else if (xUltraSonicSensor.dDistance < 15) {
			vMotorsStop();

		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim == pLineFollowerTIM) {
		//vLineFollowerTracker(xLineSensorsGetState());
		if(cState) {
			xS = xLineSensorsGetState();
			vLineFollowerNewTracker(xS);
			vPIDMotorsOutput();
		}
	} else if(htim == xLeftEncoder.htim || htim == xRightEncoder.htim) {
		vEncoderOverflowCallback(htim);
	} else if(htim == pOdometryTIM) {
		vOdometryUpdateCurrentStatus();
		iCounter1s ++;
		if(iCounter1s == 300) {
			iCounter1s = 0;
			cUpdateScreen = 1;
		} else if((!(iCounter1s % 50)) && cBuzzer) {
			vBuzzerPlay();
		} else if(!(iCounter1s % 110) && cBuzzer) {
			vBuzzerStop();
		}

	}else if(htim == xUltraSonicSensor.htim) {
		vUltraSonicSensorOverclockCallback(htim);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	GPIO_PinState SW = SWRead();
	xBt = xReadButtons();
	if (SW){

		vMotorsStop();
	}
	else if(xBt.enterBt) {
		cBuzzer = 1;
		//b = 0;
	} else if(xBt.downBt) {
		cBuzzer = 0;
		vPIDDecreaseKp();
		b = b - 0.1;
	} else if(xBt.upBt) {
		vPIDIncreaseKp();
		a = a+ 0.1;
		b = b + 0.1;
	} else if(xBt.leftBt) {
		vMotorsStop();
	} else if(xBt.rightBt) {
		vMotorsStart();
//		vMotorsRightWheelFoward();
//		vMotorsRightPower(a);
//		vMotorsLeftWheelFoward();
//		vMotorsLeftPower(b);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart3) {
		HAL_UART_Receive_IT(huart, (uint8_t*)&ucData, 1);
		vCommunicationStateMachineProcessStateMachine(ucData);
		//HAL_UART_Transmit_IT(huart, &ucData, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart3  && cFlagAll) {
		if(iINextParam < 15) {
			iINextParam ++;
			vReturnParam(cNextParam[iINextParam]);
			if(iINextParam == 15) {
				iINextParam = 0;
				cFlagAll = 0;
			}
		}

	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
