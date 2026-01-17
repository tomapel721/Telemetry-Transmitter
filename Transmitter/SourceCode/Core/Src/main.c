/* USER CODE BEGIN Header */
/*
 *  main.c
 *
 *  Created on: September 6, 2021
 *  Company: Polsl Racing
 *  Department: Electronics Team
 *  Author: Tomasz Pelan
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "fatfs.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "fatfs_sd.h"
#include "string.h"
#include <stdbool.h>
#include <NRF24/nRF24.h>
#include <NRF24/nRF24_Defs.h>
#include <Buffer/frame.h>
#include <SD/SD_helpers.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_DATA_LENGTH 8
#define FILENAME_BUFFER_LENGTH 35
#define SD_CARD_MOUNT_IMMEDIATELY 1
#define TIME_SECONDS_IN_HOUR 3600
#define TIME_MINUTES_IN_HOUR 60
#define TIME_MILISECONDS_IN_SECOND 1000000
#define SECOND_MILLENIUM 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void readDataToBuffer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* SD Card-related variables */
FATFS fs = {0};
FIL fil = {0};
FRESULT fresult = {0};
char filename[FILENAME_BUFFER_LENGTH] = {0};
UINT bw = 0;
uint8_t counterSD = 0;

/* CAN-related variables */
CAN_RxHeaderTypeDef rxHeader = {0};
uint8_t rxData[CAN_DATA_LENGTH] = {0};

/* CAN buffer */
Frame bufferingFrame = {0};

/* RTC-related variables */
RTC_TimeTypeDef currentTime = {0};
RTC_DateTypeDef currentDate = {0};
uint32_t miliseconds = 0;
uint32_t seconds = 0;

/* IRQ-related variables */
bool interruptFlag = false;

/* Radio module-related variables */
const uint8_t* rxAddress = (uint8_t*)"Nad";
const uint8_t* txAddress = (uint8_t*)"Odb";

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
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* CAN init */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Radio module init */
  nRF24_Init(&hspi3);
  nRF24_SetRXAddress(0, rxAddress);
  nRF24_SetTXAddress(txAddress);
  nRF24_TX_Mode();

  /* SD card mount */
   fresult = f_mount(&fs, "/", SD_CARD_MOUNT_IMMEDIATELY);

  /* Handle naming of file on SD card */
   HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
   HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
   miliseconds = ((currentTime.SecondFraction - currentTime.SubSeconds)/((float)currentTime.SecondFraction+1) * TIME_MILISECONDS_IN_SECOND);
   sprintf(filename, "LOG_%02d%02d%d_%02d%02d_%02d_%lu.txt", currentDate.Date,
		   currentDate.Month, currentDate.Year + SECOND_MILLENIUM, currentTime.Hours,
		   currentTime.Minutes, currentTime.Seconds, miliseconds);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Radio module interrupt checker */
	  nRF24_Event();

	  /* Handling of received packet */
	  if(interruptFlag)
	  {
		 readDataToBuffer();
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Radio module interrupt callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	if(GPIO_Pin == NRF24_IRQ_Pin)
	{
		nRF24_IRQ_Handler();
	}
}

/* CAN message received callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
	interruptFlag = true;
}

/* Function reads data, pack it into buffer and save data to SD card */
static void readDataToBuffer(void)
{
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
	bufferingFrame.year = currentDate.Year;
	bufferingFrame.month = currentDate.Month;
	bufferingFrame.day = currentDate.Date;
	bufferingFrame.hours = currentTime.Hours;
	bufferingFrame.minutes = currentTime.Minutes;
	bufferingFrame.seconds = currentTime.Seconds;
	miliseconds = ((currentTime.SecondFraction - currentTime.SubSeconds)/((float)currentTime.SecondFraction+1) * TIME_MILISECONDS_IN_SECOND);
	seconds = TIME_SECONDS_IN_HOUR * currentTime.Hours + TIME_MINUTES_IN_HOUR * currentTime.Minutes + currentTime.Seconds;
	memcpy(bufferingFrame.frameData, rxData, CAN_DATA_LENGTH);

	bufferingFrame.frameDLC = rxHeader.DLC;
	bufferingFrame.frameIDE = rxHeader.IDE;
	bufferingFrame.frameRTR = rxHeader.RTR;
	bufferingFrame.frameStdID = rxHeader.StdId;

	writeFrameToSDCard(&bufferingFrame);
	nRF24_SendData((uint8_t*)&bufferingFrame, sizeof(Frame));

	interruptFlag = false;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
