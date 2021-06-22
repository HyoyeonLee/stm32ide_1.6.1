/* USER CODE BEGIN Header */
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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pitches.h"
#include "alarmSongs.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
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
char rx_data;
char rx_buf[100]={'0'};
char tx_buf[200]={'0'};
char tokens[3][100]={'\0'};
int idx,idx_song;
float fArr,fDuration;
uint32_t uArr,uDuration;
int nNotes = sizeof(bike)/sizeof(bike[0]);
int count, end, count_interval,interval,end_interval;
char* AMPM[2]={"AM","PM"};
int isAlarmSet, isPlaying;
RTC_TimeTypeDef currentT ={0}, alarmT={0};
RTC_DateTypeDef currentD ={0};
int date_limit[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
int year,month,date,h,m,s;
char ampm;
int isValidCommand;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

//-----------------------------------------------------------------------------------------Command Error
void refresh()
{
	memset(tx_buf,0,sizeof(tx_buf));
	sprintf(tx_buf,"\r\n command not found.\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	memset(rx_buf,0,sizeof(rx_buf));
	idx = 0;
}

//-----------------------------------------------------------------------------------------response
void response(int time0_alarm1,int time0_date1_both2)
{
	char stampD[30]={'\0'};
	char stampT[30]={'\0'};
	sprintf(stampD,"20%02d-%02d-%02d",currentD.Year,currentD.Month,currentD.Date);
	if(time0_alarm1==0)
	{
		sprintf(stampT,"%s %02d:%02d:%02d",\
				AMPM[currentT.TimeFormat>>6],currentT.Hours,currentT.Minutes,currentT.Seconds);
	}
	else
	{
		sprintf(stampT,"[Alarm] %s %02d:%02d:%02d",\
				AMPM[alarmT.TimeFormat>>6],alarmT.Hours,alarmT.Minutes,alarmT.Seconds);
	}
	memset(tx_buf,0,sizeof(tx_buf));
	strcat(tx_buf,"\r\n< ");
	switch(time0_date1_both2)
	{
	case 0:
		strcat(tx_buf,stampD);
		break;
	case 1:
		strcat(tx_buf,stampT);
		break;
	case 2:
		strcat(tx_buf,stampD);
		strcat(tx_buf," ");
		strcat(tx_buf,stampT);
		break;
	}
	strcat(tx_buf,"\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	memset(rx_buf,0,sizeof(rx_buf));
	idx = 0;
}

//-----------------------------------------------------------------------------------------0)Get Date
void get_date()
{
	HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
	response(0,0);
}
//-----------------------------------------------------------------------------------------1)Get Time
void get_time()
{
	HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
	response(0,1);
}
//-----------------------------------------------------------------------------------------2)Get DateTime
void get_date_time()
{
	HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
	response(0,2);
}

//-----------------------------------------------------------------------------------------3)Set Date
void set_date()
{
	if(strlen(tokens[1])!=6)refresh();
	else
	{
		sscanf(tokens[1],"%02d%02d%02d",&year,&month,&date);
		if(month<1 || month>12)refresh();
		else if(date<1 && date>date_limit[month-1])refresh();
		else
		{
			currentD.Year  = year;
			currentD.Month = month;
			currentD.Date  = date;
			HAL_RTC_SetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
			response(0,0);
		}
	}
}
//-----------------------------------------------------------------------------------------4)Set Time
void set_time()
{
	if(strlen(tokens[1])!=7)refresh();
	else
	{
		sscanf(tokens[1],"%c%02d%02d%02d",&ampm,&h,&m,&s);
		if(!(ampm=='A' || ampm=='P'))refresh();
		else if(h<0 && h>12)refresh();
		else if(m<0 && m>59)refresh();
		else if(s<0 && s>59)refresh();
		else
		{
			if(ampm=='A')currentT.TimeFormat      &= ~(1<<6);
			else if(ampm=='P')currentT.TimeFormat |=  (1<<6);
			currentT.Hours=h;
			currentT.Minutes=m;
			currentT.Seconds=s;
			HAL_RTC_SetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
			response(0,1);
		}
	}
}

//-----------------------------------------------------------------------------------------5)Set DateTime
void set_date_time()
{
	if(!(strlen(tokens[1])==6 && strlen(tokens[2])==7))refresh();
	else
	{
		sscanf(tokens[1],"%02d%02d%02d",&year,&month,&date);
		sscanf(tokens[2],"%c%02d%02d%02d",&ampm,&h,&m,&s);
		if(month<1 || month>12)refresh();
		else if(date<1 && date>date_limit[month-1])refresh();
		else if(!(ampm=='A' || ampm=='P'))refresh();
		else if(h<0 && h>12)refresh();
		else if(m<0 && m>59)refresh();
		else if(s<0 && s>59)refresh();
		else
		{
			currentD.Year  = year;
			currentD.Month = month;
			currentD.Date  = date;
			if(ampm=='A')currentT.TimeFormat      &= ~(1<<6);
			else if(ampm=='P')currentT.TimeFormat |=  (1<<6);
			currentT.Hours=h;
			currentT.Minutes=m;
			currentT.Seconds=s;
			HAL_RTC_SetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
			HAL_RTC_SetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
			response(0,2);
		}
	}
}
//-----------------------------------------------------------------------------------------6)Set Alarm
void set_alarm_time()
{
	if(strlen(tokens[1])!=7)refresh();
	else
	{
		sscanf(tokens[1],"%c%02d%02d%02d",&ampm,&h,&m,&s);
		if(!(ampm=='A' || ampm=='P'))refresh();
		else if(h<0 && h>12)refresh();
		else if(m<0 && m>59)refresh();
		else if(s<0 && s>59)refresh();
		else
		{
			alarmT.Hours=h;
			alarmT.Minutes=m;
			alarmT.Seconds=s;
			if(ampm=='A')alarmT.TimeFormat      &= ~(1<<6);
			else if(ampm=='P')alarmT.TimeFormat |=  (1<<6);
			isAlarmSet=1;
			response(1,1);
		}
	}
}

//-----------------------------------------------------------------------------------------play a song
void play_a_song()
{
	get_time();
	isPlaying = 1;
	memset(tx_buf,0,sizeof(tx_buf));
	sprintf(tx_buf,"\r\n -----> Alarm Music Starts!!!\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
}

//-----------------------------------------------------------------------------------------clear
void clear()
{
	sprintf(tx_buf,"\r\n");
	for(int i=0;i<40;i++)
		strcat(tx_buf,"\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	strcat(tx_buf,"\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
}

//-----------------------------------------------------------------------------------------Menu array
void (*serve_the_menu[8])(void) = {\
		get_date,get_time,get_date_time,\
		set_date,set_time,set_date_time,\
		set_alarm_time,clear\
};

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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3,(uint8_t*)&rx_data,1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  sprintf(tx_buf,"> ");
  HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
  memset(tx_buf,0,sizeof(tx_buf));
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3,(uint8_t*)&rx_data,1);
		if(rx_data=='\b')
		{
			char backspace[] = "\b ";
			HAL_UART_Transmit(&huart3, (uint8_t*)& backspace, strlen(backspace), 1000);
			if (idx >= 1)
			{
				rx_buf[idx--] = '\0';
				rx_buf[idx]   = '\0';
			}
		}
		HAL_UART_Transmit(&huart3,(uint8_t*)&rx_data,1,1000);
		if(rx_data == '\r')
		{
			if(idx==0){}
			else
			{
				char* cmd[8] = {"GetDate","GetTime","GetDateTime",\
								"SetDate","SetTime","SetDateTime",\
								"SetAlarm","clear"};
				int valid_nstrings[8] = {1,1,1,2,2,3,2,1};
				char delimit[] = " ";
				char* temp;
				temp = strtok(rx_buf, delimit);
				int counting = 0;
				while (1)
				{
					if(temp == 0) break;
					strcpy(tokens[counting], temp);
					temp = strtok(NULL, delimit);
					counting++;
				}
				for(int i=0;i<8;i++)
				{
					if(strcmp(tokens[0],cmd[i])==0 && counting==valid_nstrings[i])
					{
						serve_the_menu[i]();
						isValidCommand=1;
					}
				}
				if(isValidCommand==0)refresh();
			}
			memset(rx_buf,0,sizeof(rx_buf));
			idx=0;
			for(int i=0;i<3;i++)memset(tokens[i],0,sizeof(tokens[i]));
			isValidCommand=0;
		}
		else if(rx_data!='\b'){rx_buf[idx++]=rx_data;}
	}
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(isPlaying==1)
		{
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
			if(count == 0)
			{
				if(bike[idx_song]==0)
				{
					fArr   = 1000;   							uArr = (uint32_t)round(fArr);
					fDuration = 2*1000/bike_tempos[idx_song]; 	uDuration = (uint32_t)round(fDuration);
					TIM3->ARR = uArr-1;
					TIM3->CCR3 = 1;
				}
				else
				{
					fArr   = pow(10,6)/bike[idx_song];   		uArr = (uint32_t)round(fArr);
					fDuration = 2*1000/bike_tempos[idx_song]; 	uDuration = (uint32_t)round(fDuration);
					TIM3->ARR = uArr-1;
					TIM3->CCR3 = (uint32_t)round((uArr-1)/2);
					end = round(fDuration*1000/fArr);
				}
				end = round(fDuration*1000/fArr);
				end_interval = round(100*1000/fArr);
				count++;
				memset(tx_buf,0,sizeof(tx_buf));
				sprintf(tx_buf,"\t\tnote[%d] = %lu Hz, arr = %lu \r\n",idx_song,bike[idx_song],TIM3->ARR);
				HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
				interval=1;
			}
			else if(count == end)
			{
				if(interval==1)
				{
					if(count_interval==0) {TIM3->CCR3=1;count_interval++;}
					else if(count_interval==end_interval) {count_interval=0;interval=0;}
					else {count_interval++;}
				}
				else
				{
					idx_song++;
					if (idx_song==nNotes-1)
					{
						isPlaying=0;
						get_time();
						sprintf(tx_buf,">");
						HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
						memset(tx_buf,0,sizeof(tx_buf));
					}
					count = 0;
				}
			}
			else {count++;}
		}
	}
	if(htim->Instance==TIM2)
	{
		HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
		HAL_RTC_GetTime(&hrtc, &currentT, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &currentD, RTC_FORMAT_BIN);
		if(isAlarmSet==1)
		{
			if(currentT.TimeFormat==alarmT.TimeFormat)
				if(currentT.Hours==alarmT.Hours)
					if(currentT.Minutes==alarmT.Minutes)
						if(currentT.Seconds==alarmT.Seconds) {isPlaying=1;play_a_song();}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
