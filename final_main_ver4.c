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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32_ds3231_at24c32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct{
	uint8_t name[32];
	uint8_t id[32];
	uint8_t pw[32];
}member;
typedef struct{
	uint8_t id[32];
	_RTC stamp;
}logInfo;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rx_data;
char rx_buf[100]={'0'};
char rx_buf2[100]={'0'};
char tx_buf[200]={'0'};
char tokens[10][32]={'\0'};
int idx;
uint32_t sysClock_Hz = 16000000;
uint32_t min_motor =544;
uint32_t max_motor = 2400;
int isAlarmSet, isPlaying;
int date_limit[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
int year,month,date,h,m,s;
int isValidCommand;
int delay_door_ms=2000;
int counting_tokens=0,counting_cr=0;
uint8_t nmember = 0,  nlog = 0;
const int max_member = 30;
const int max_log =30;
uint16_t addr_nmember = 4095;
uint16_t addr_nlog = 4094;
uint16_t addr_member_st = 0;
uint16_t addr_log_st = sizeof(member)*(1+max_member);//
uint16_t size_of_member  = sizeof(member);
uint16_t size_of_logInfo = sizeof(logInfo);
uint16_t size_of_uint16_t = sizeof(uint16_t);
int stop_receiving = 0;
int menu;
_RTC rtc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sign_change(uint8_t* to, char* from,int len)
{
	uint8_t c;
	int i;
	for(i=0;i<len;i++)
	{
		c = (uint8_t)from[i];
		to[i] = c;
	}
	to[i]='\0';
}
void sign_inverse_change(char* to, uint8_t* from,int len)
{
	char c;
	int i;
	for(i=0;i<len;i++)
	{
		c = (char)from[i];
		to[i] = c;
	}
	to[i]='\0';
}
void lf()
{
	memset(tx_buf,0,sizeof(tx_buf));
	sprintf(tx_buf,"\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
}
//-----------------------------------------------------------------------------------------HELP
void help()
{
	memset(tx_buf,0,sizeof(tx_buf));
	sprintf(tx_buf,"\r\n usage:\r\n");
	strcat(tx_buf,"   (0) Add or Delete a member\r\n");
	strcat(tx_buf,"\t> Add name id password\r\n");
	strcat(tx_buf,"\t> Delete id\r\n");
	strcat(tx_buf,"   (1) Get IDs \r\n");
	strcat(tx_buf,"\t> GetID\r\n");
	strcat(tx_buf,"   (2) Get or Delete Log \r\n");
	strcat(tx_buf,"\t> GetLog\r\n");
	strcat(tx_buf,"\t> DelLog id\r\n");
	strcat(tx_buf,"   (3) Get/Set Time/Date\r\n");
	strcat(tx_buf,"\t> GetTime\r\n");
	strcat(tx_buf,"\t> GetDate\r\n");
	strcat(tx_buf,"\t> SetTime 23:11:30\r\n");
	strcat(tx_buf,"\t> SetDate 210704\r\n");
	strcat(tx_buf,"   (4) Set Delay Time [sec] for the Door\r\n");
	strcat(tx_buf,"\t> SetDelay 5\r\n");
	strcat(tx_buf,"   (5) Input Password to Open the Door\r\n");
	strcat(tx_buf,"\t> Press Enter Key 3 times.\r\n");

	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
}


//-----------------------------------------------------------------------------------------Command Error
void refresh()
{
	memset(tx_buf,0,sizeof(tx_buf));
	sprintf(tx_buf,"\r\n Command not found. Try help for usage.\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	memset(rx_buf,0,sizeof(rx_buf));
	idx = 0;
}

//-----------------------------------------------------------------------------------------response
void response(int n)
{	memset(tx_buf,0,sizeof(tx_buf));
	switch(n)
	{
	case 0:
		sprintf(tx_buf,"\r\n > %02d:%02d:%02d",rtc.Hour,rtc.Min,rtc.Sec);
		break;
	case 1:
		sprintf(tx_buf,"\r\n > 20%02d-%02d-%02d",rtc.Year,rtc.Month,rtc.Date);
		break;
	case 2:
		sprintf(tx_buf,"\r\n > Only 30 members are allowed.\r\n");
		break;
	case 3:
		sprintf(tx_buf,"\r\n > Delete Complete");
		break;
	case 4:
		sprintf(tx_buf,"\r\n > The ID does not exist.\r\n");
		break;
	case 5:
		sprintf(tx_buf,"\r\n > Incorrect Password.\r\n");
		break;
	}
	strcat(tx_buf,"\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	memset(rx_buf,0,sizeof(rx_buf));
	idx = 0;
}

void enter_password()
{
	strcat(tx_buf," > Input Password : ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,sizeof(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	int count_pw = 0;
	while(1)
	{
		HAL_UART_Receive(&huart3,(uint8_t*)(rx_buf2+count_pw),1,1000);
		if(rx_buf2[count_pw]=='\r')break;
		count_pw++;
	}
}


//-----------------------------------------------------------------------------------------0-1)Add a memeber
void add_member()
{
	//HAL_UART_AbortReceive_IT(&huart3);
	member one;//={{0},{0},{0}};
	//sign_change(one.pw,tokens[counting_tokens-1],strlen(tokens[counting_tokens-1]));
	//sign_change(one.id,tokens[counting_tokens-2],strlen(tokens[counting_tokens-2]));
	//sign_change(one.name,tokens[1],strlen(tokens[1]));
	strcpy(one.name,tokens[1]);
	strcpy(one.id,  tokens[2]);
	strcpy(one.pw,  tokens[3]);

	/*
	int k=0;
	for(int i=1;i<counting_tokens-2;i++)
	{
		if(i!=1)one.name[k]=' ';
		k++;
		for(int j=0;j<strlen(tokens[i]);j++)
		{
			c=(uint8_t)tokens[i][j];
			one.name[k]=c;
			k++;
		}
	}
	*/
	if (nmember<= max_member - 1)
	{
		lf();
		uint8_t* ptr_one = (uint8_t*)&one;
		int f;
		//f=AT24C32_EEPROM_write(D2B((nmember-1)*sizeof(one)), ptr_one, size_of_member);
		f=AT24C32_EEPROM_write(D2B(0), ptr_one, size_of_member);
		sprintf(tx_buf,"write eeprom t or f: %d\r\n",f);
		HAL_UART_Transmit(&huart3,tx_buf,strlen(tx_buf),1000);
		memset(tx_buf,0,sizeof(tx_buf));
		nmember++;
		AT24C32_EEPROM_write(D2B(addr_nmember), &nmember, size_of_uint16_t);
		lf();
	}
	else if(nmember > 30)
	{
		response(2);
	}
	else
	{
		lf();lf();lf();lf();lf();lf();
	}
}

//-----------------------------------------------------------------------------------------0-2)Delete a memebr
void delete_member()
{
	for(int i=addr_member_st;i<=addr_member_st+(nmember-1)*sizeof(member);i+=sizeof(member))
	{
		member one;
		uint8_t* ptr_one = (uint8_t*)&one;
		AT24C32_EEPROM_read(D2B(i),ptr_one,size_of_member);
		char to1[32]={'\0'};
		sign_inverse_change(to1,one.id,32);
		if(strcmp(to1,tokens[1])==0)
		{
			enter_password();
			char to[32]={'\0'};
			sign_inverse_change(to,one.pw,32);
			if(strcmp(to,rx_buf2)==0)
			{
				member empty= {{0},{0},{0}};
				uint8_t* ptr_empty = (uint8_t*)&empty;

				AT24C32_EEPROM_write(D2B(i),ptr_empty,size_of_member);
				for (int j= i+sizeof(member);j<=addr_member_st+ (nmember)*sizeof(one);i+=sizeof(member))
				{
					AT24C32_EEPROM_read(D2B(j),ptr_one,size_of_member);
					AT24C32_EEPROM_write(D2B(j-sizeof(one)),ptr_one,size_of_member);
				}
				nmember--;
				AT24C32_EEPROM_write(D2B(addr_nmember),&nmember,size_of_uint16_t);

				response(3);
			}
			else
			{
				response(5);
			}
			break;
		}
	}
	response(4);
}
//-----------------------------------------------------------------------------------------1)Get IDs
void get_ids()
{
	member one;
	uint8_t* ptr_one = (uint8_t*)&one;
	for(int i=addr_member_st;i<=addr_member_st+(nmember-1)*sizeof(one);i+=sizeof(one))
	{
		AT24C32_EEPROM_read(D2B(i),ptr_one,size_of_member);
		sprintf(tx_buf,"%02d. %s\r\n",i+1,one.id);
		HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,sizeof(tx_buf),1000);
		memset(tx_buf,0,sizeof(tx_buf));
	}
	lf();
}
//-----------------------------------------------------------------------------------------2-1)Get Log
void get_log()
{
	logInfo log;
	uint8_t* ptr_log = (uint8_t*)&log;
	for(int i=addr_log_st;i<=(nlog-1)*sizeof(logInfo);i+=sizeof(logInfo))
	{
		AT24C32_EEPROM_read(D2B(i),ptr_log,size_of_logInfo);
		sprintf(tx_buf,"%02d. 20%02d-%02d-%02d %02d:%02d:%02d %s\r\n",\
				i, log.stamp.Year, log.stamp.Month, log.stamp.Date, log.stamp.Hour,log.stamp.Min,log.stamp.Sec,log.id);
		HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,sizeof(tx_buf),1000);
		memset(tx_buf,0,sizeof(tx_buf));
	}
	lf();
}

//-----------------------------------------------------------------------------------------2-2)Delete Log
void delete_log()
{
	logInfo log;//={{0},{0}};
	uint8_t* ptr_log = (uint8_t*)&log;
	for(int i=addr_log_st;i<=(nlog-1)*sizeof(logInfo);i+=sizeof(logInfo))
	{
		AT24C32_EEPROM_write(D2B(i),ptr_log,size_of_logInfo);
	}
	nlog=0;
	response(3);
}
//-----------------------------------------------------------------------------------------3-1)Get Time
void get_time()
{
	DS3231_GetTime(&rtc);
	response(0);
}
//-----------------------------------------------------------------------------------------3-2)Get Date
void get_date()
{
	DS3231_GetTime(&rtc);
	response(1);
}
//-----------------------------------------------------------------------------------------3-3)Set Time
void set_time()
{
	if(strlen(tokens[1])!=8)refresh(0);
	else
	{
		char colon;
		sscanf(tokens[1],"%02d%c%02d%c%02d",&h,&colon,&m,&colon,&s);
		     if(h<0 && h>23)refresh(0);
		else if(m<0 && m>59)refresh(0);
		else if(s<0 && s>59)refresh(0);
		else
		{
			DS3231_GetTime(&rtc);
			rtc.Hour=h;
			rtc.Min=m;
			rtc.Sec=s;
			DS3231_SetTime(&rtc);
			lf();
		}
	}
}
//-----------------------------------------------------------------------------------------3-4)Set Date
void set_date()
{
	if(strlen(tokens[1])!=6)refresh(0);
	else
	{
		sscanf(tokens[1],"%02d%02d%02d",&year,&month,&date);
		if(month<1 || month>12)refresh(0);
		else if(date<1 && date>date_limit[month-1])refresh(0);
		else
		{
			DS3231_GetTime(&rtc);
			rtc.Year  = year;
			rtc.Month = month;
			rtc.Date  = date;
			//rtc.DaysOfWeek = TUESDAY;
			DS3231_SetTime(&rtc);
			lf();
		}
	}
}
//-----------------------------------------------------------------------------------------4)Set Delay
void set_delay()
{
	int temp_delay;
	sscanf(tokens[1],"%d",&temp_delay);
	if(temp_delay<1 || temp_delay>HAL_MAX_DELAY)refresh(0);
	else {delay_door_ms = temp_delay*1000;}
	lf();
}

void buzzer(int correct1_incorrect0)
{
	int notes[4]={523,659,784,988};
	uint32_t prescaler = TIM4->PSC;
	uint32_t arr;
	int frequency_Hz;
	switch (correct1_incorrect0)
	{
	case 0:
		 frequency_Hz = notes[3];
		 arr = -1+sysClock_Hz/((prescaler+1)*frequency_Hz);
		 TIM4->ARR =arr;
		 TIM4->CCR3 = (arr+1)/2;
		 HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
		 HAL_Delay(400);
		 HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
		break;
	case 1:
		for(int i=0;i<3;i++)
		{
			frequency_Hz = notes[i];
			arr = -1+sysClock_Hz/((prescaler+1)*frequency_Hz);
			TIM4->ARR =arr;
			TIM4->CCR3 = (arr+1)/2;
			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
			HAL_Delay(400);
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
		}
		break;
	}
}

void open()
{
	int angle = 90;
	uint32_t ccr = (uint32_t)(min_motor+((max_motor-min_motor)/180.0)*angle);
	TIM3->CCR2 = ccr;
	HAL_Delay(delay_door_ms);
	angle=0;
	ccr = (uint32_t)(min_motor+((max_motor-min_motor)/180.0)*angle);
	TIM3->CCR2 = ccr;
}
//-----------------------------------------------------------------------------------------5)Input Password
void input_password()
{
	enter_password();
	for (int i=addr_member_st;i<=addr_member_st+(nmember-1);i+=sizeof(member))
	{
		member one;
		uint8_t* ptr_one = (uint8_t*)&one;
		AT24C32_EEPROM_read(D2B(i),ptr_one,sizeof(member));
		char to[32];
		sign_inverse_change(to,one.pw,32);
		if(strcmp(rx_buf2,to)==0)
		{
			buzzer(1);
			open();
			_RTC temp_stamp;
			DS3231_GetTime(&temp_stamp);
			logInfo log;
			uint8_t* ptr_log = (uint8_t*)&log;
			memset(to,0,sizeof(to));
			sign_inverse_change(to,one.id,32);
			sign_change(log.id,to,32);
			log.stamp.Hour 	= temp_stamp.Hour;
			log.stamp.Min 	= temp_stamp.Min;
			log.stamp.Sec 	= temp_stamp.Sec;
			log.stamp.Year 	= temp_stamp.Year;
			log.stamp.Month = temp_stamp.Month;
			log.stamp.Date	= temp_stamp.Date;
			log.stamp.DaysOfWeek = temp_stamp.DaysOfWeek;
			AT24C32_EEPROM_write(D2B(addr_log_st+ (nlog)*sizeof(log)),ptr_log,sizeof(log));
		}
	}
	buzzer(0);
	response(5);
}


//-----------------------------------------------------------------------------------------Menu array
void (*serve_the_menu[12])(void) =
	{
		add_member,delete_member,\
		get_ids,\
		get_log,delete_log,\
		get_time,get_date,set_time,set_date,\
		set_delay,\
		help,\
		input_password
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  rtc.Year  = 21;
  rtc.Month = 07;
  rtc.Date  = 04;
  rtc.DaysOfWeek = SUNDAY;
  rtc.Hour = 23;
  rtc.Min = 59;
  rtc.Sec = 59;
  DS3231_AT24C32_Init(&hi2c1);
  HAL_Delay(1000);
  DS3231_SetTime(&rtc);
  HAL_Delay(1000);
  DS3231_GetTime(&rtc);
  HAL_Delay(1000);
  _RTC rtc1;
  sprintf(tx_buf,"\r\n[%d%d] %d-%d-%d %d:%d:%d\r\n",\
		  DS3231_SetTime(&rtc),DS3231_GetTime(&rtc1),\
		  rtc1.Year,rtc1.Month,rtc1.Date,\
		  rtc1.Hour,rtc1.Min,rtc1.Sec);
  HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
  HAL_UART_Receive_IT(&huart3,(uint8_t*)&rx_data,1);
  lf();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(stop_receiving==1)
	  {
		  serve_the_menu[menu];
		  stop_receiving=0;
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{

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
			if(idx==0 && counting_cr==0){counting_cr=1;}
			else
			{
				char* cmd[12] = {"Add",	"Delete",\
								"GetID",\
								"GetLog","DelLog",\
								"GetTime","GetDate","SetTime","SetDate",\
								"SetDelay","help","\r\n\r\n"};
				int valid_nstrings[12] = {4,2, 1, 1,1, 1,1,2,2, 2,1,2};
				char delimit[] = " ";
				char* temp;
				temp = strtok(rx_buf, delimit);
				counting_tokens = 0;
				while (1)
				{
					if(temp == 0) break;
					strcpy(tokens[counting_tokens], temp);
					temp = strtok(NULL, delimit);
					counting_tokens++;
				}
				for(int i=0;i<=12;i++)
				{
					if(strcmp(tokens[0],cmd[i])==0)
					{
						if(counting_tokens==valid_nstrings[i])
						{
							//serve_the_menu[i]();
							menu=i;
							stop_receiving=1;
							isValidCommand=1;
						}
					}
				}
				if(isValidCommand==0)refresh(0);
			}
			memset(rx_buf,0,sizeof(rx_buf));
			idx=0;
			for(int i=0;i<10;i++)memset(tokens[i],0,sizeof(tokens[i]));
			isValidCommand=0;
		}
		else if(rx_data!='\b'){rx_buf[idx++]=rx_data;}
		HAL_UART_Receive_IT(&huart3,(uint8_t*)&rx_data,1);
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
