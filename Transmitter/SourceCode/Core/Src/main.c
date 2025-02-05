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
#include "nRF24/nRF24.h"
#include "nrF24/nRF24_Defs.h"
#include "Bufor/Frame.h"
#include "fatfs_sd.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MESSAGE_BUFF_LEN 32 // 32 payload + \0
#define CAN_DATA_LENGTH 8
#define CAN_STD_ID_BITS 8
#define ASCII_ZERO_SIGN_CODE 48
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t message[MESSAGE_BUFF_LEN];
uint8_t messageLength;

/* zmienne i funkcje do obsłgui karty SD */
FATFS fs;
FIL fil;
FRESULT fresult;
char filename[35];
UINT bw;
uint8_t counterSD; // licznik kiedy otworzyć/zamknąć plik - w domyśle co 10 ramek.

/* Zwraca wielkość bufora na kartę SD, potrzebny do pisania na karcie */
int bufsize(char* buf)
{
	int i = 0;
	while( *buf++ != '\0') i++;
	return i;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MX_NVIC_Init(void);

/* Deklaracja zmiennych do CAN */
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];


/* Deklaracja zmiennych do timestampów */
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
uint32_t miliseconds = 0;
uint32_t seconds = 0;

/* Deklaracja bufora dla CAN */
Frame bufferingFrame;
bool interruptFlag = false;


 /* Funkcja czytająca potrzebne dane, pakująca do bufora i kartę SD */
 void readDataToBuffer(void)
 {

	  	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	  	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
	  	bufferingFrame.year = currentDate.Year;
	  	bufferingFrame.month = currentDate.Month;
	  	bufferingFrame.day = currentDate.Date;
	  	bufferingFrame.hours = currentTime.Hours;
	  	bufferingFrame.minutes = currentTime.Minutes;
	  	bufferingFrame.seconds = currentTime.Seconds;
	  	miliseconds = ((currentTime.SecondFraction - currentTime.SubSeconds)/((float)currentTime.SecondFraction+1) * 1000000);
	  	seconds = 3600 * currentTime.Hours + 60 * currentTime.Minutes + currentTime.Seconds;
	  	memcpy(bufferingFrame.frameData, rxData, CAN_DATA_LENGTH);

	  	bufferingFrame.frameDLC = rxHeader.DLC;
	  	bufferingFrame.frameIDE = rxHeader.IDE;
	  	bufferingFrame.frameRTR = rxHeader.RTR;
	  	bufferingFrame.frameStdID = rxHeader.StdId;

	  	writeFrameToSDCard(bufferingFrame);
	  	nRF24_SendData(&bufferingFrame, sizeof(Frame)); //test ringBuffera

	  	interruptFlag = false;
 }

 /* Potrzebne do poprawnego użycia formatu Kvasera w funkcji writeFrameToSDCard */
 uint8_t numOfDigits(uint32_t number)
 {
 	uint8_t numberOfDigits = 0;
 	while (number > 0)
 	{
 		numberOfDigits++;
 		number /= 10;
 	}
 	return numberOfDigits;
 }

 /* Szybka konwersja dec -> hex */
 static inline char hex_nibble(uint8_t bin)
 {
 	return "0123456789ABCDEF"[bin];
 }

 /* Zapisywanie danych na kartę microSD w formacie Kvasera */
 void writeFrameToSDCard(Frame frame)
 {


	   char bufferSD[100];
	   if (counterSD % 10 == 0)
	   	{
		   fresult = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
	   	}
	    fresult = f_lseek(&fil,  f_size(&fil)); // do nadpisywania pliku, przesuwa wskaźnik na koniec
	   	uint8_t length = 0;
	   	bufferSD[length++] = ' ';
	   	bufferSD[length++] = '1';
	   	bufferSD[length++] = ' ';
	   	bufferSD[length++] = ' ';
	   	bufferSD[length++] = ' ';
	   	bufferSD[length++] = ' ';
	   	for (int i = 28; i >= 0; i -= 4)
	   	{
	   		bufferSD[length++] = hex_nibble(frame.frameStdID >> i & 0xF);
	   	}

	   	for (int i = 0; i < 9; i++)
	   	{
	   		bufferSD[length++] = ' ';
	   	}
	   	bufferSD[length++] = hex_nibble(frame.frameDLC & 0xF);
	   	bufferSD[length++] = ' ';
	   	bufferSD[length++] = ' ';

	   	for (uint8_t i = 0; i < frame.frameDLC; i++)
	   	{
	   		bufferSD[length++] = hex_nibble(frame.frameData[i] >> 4 & 0xF);
	   		bufferSD[length++] = hex_nibble(frame.frameData[i] & 0xF);
	   		bufferSD[length++] = ' ';
	   		bufferSD[length++] = ' ';
	   	}
	   	for (int i = 0; i < CAN_DATA_LENGTH - frame.frameDLC + 1; i++)
	   	{
	   		bufferSD[length++] = ' ';
	   		bufferSD[length++] = ' ';
	   		bufferSD[length++] = ' ';
	   		bufferSD[length++] = ' ';
	   	}

	   	uint8_t numOfSecondsDigits = numOfDigits(seconds);
	   	for (int i = 0; i < numOfSecondsDigits; i++)
	   	{
	   		uint32_t divider = pow(10, numOfSecondsDigits - 1 - i);
	   		uint8_t numberToWriteInBuffer = seconds / divider;
	   		seconds -= numberToWriteInBuffer * divider;
	   		bufferSD[length++] = numberToWriteInBuffer + ASCII_ZERO_SIGN_CODE; // 48 to stała ASCII między początkiem tablicy ASCII a cyfrą 0
	   	}
	   	bufferSD[length++] = '.';
	   	const uint8_t NUM_OF_MILISECONDS_DIGITS = 6;
	   	for (int i = 0; i < NUM_OF_MILISECONDS_DIGITS; i++)
	   	{
	   		uint32_t divider = pow(10, NUM_OF_MILISECONDS_DIGITS - 1 - i);
	   		uint8_t numberToWriteInBuffer = miliseconds / divider;
	   		miliseconds -= numberToWriteInBuffer * divider;
	   		bufferSD[length++] = numberToWriteInBuffer + ASCII_ZERO_SIGN_CODE; // 48 to stała ASCII między początkiem tablicy ASCII a cyfrą 0
	   	}
	   	bufferSD[length++] = ' ';
	   	bufferSD[length++] = 'R';
	   	bufferSD[length++] = '\n';
	   	bufferSD[length++] = '\0';
	   	fresult = f_write(&fil, bufferSD, bufsize(bufferSD), &bw);

	   	counterSD++;
	   	if (counterSD % 10 == 0)
	   	{
	   		f_close(&fil);
	   		counterSD = 0;
	   	}

 }
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

  /* Inicjalizacja CAN  */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Inicjalizacja modułu radiowego */
  nRF24_Init(&hspi3);
  nRF24_SetRXAddress(0, "Nad");
  nRF24_SetTXAddress("Odb");
  nRF24_TX_Mode();

  /* Montaż karty microSD */
   fresult = f_mount(&fs, "/", 1);

  /* Nazwanie pliku na karcie microSD */
   HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
   HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);
   miliseconds = ((currentTime.SecondFraction - currentTime.SubSeconds)/((float)currentTime.SecondFraction+1) * 1000000);
   sprintf(filename, "LOG_%02d%02d%d_%02d%02d_%02d_%d.txt", currentDate.Date,
		   currentDate.Month, currentDate.Year + 2000, currentTime.Hours,
		   currentTime.Minutes, currentTime.Seconds, miliseconds);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Obsługa przerwania */
	  nRF24_Event();

	  /* Wysyłka pakietu */
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

/* Przerwanie od modułu radiowego */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
	if(GPIO_Pin == NRF24_IRQ_Pin)
			{
				nRF24_IRQ_Handler();
			}
}



/* Przerwanie - przyszła ramka CAN */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
	interruptFlag = true;
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
