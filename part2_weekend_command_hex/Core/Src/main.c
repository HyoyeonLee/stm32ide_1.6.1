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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "stdlib.h"
#include "stdarg.h"
#define pi M_PI
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
uint8_t rx_data;
unsigned char rx_buf[100] = {'\0'};
unsigned char tx_buf[100] = {'\0'};
unsigned char* data;
unsigned char menu,ndata,checkSum;
unsigned char buf[100]={'\0'};
unsigned char buf4[100]={'\0'};
int idx=0;
uint32_t analog_reading[2];
uint32_t sysClock_Hz = 16*pow(10,6);
uint32_t min_motor = 544;
uint32_t max_motor = 2400;
int stopReceiving = 0;
unsigned char flag_ndata = 0, flag_checkSum=0;
int adc_channel;
int bitPosition[3]={0,7,14};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
//----------------------------------------------------------------------------------------------Response
void response(unsigned char ndata_tx, ...)
{
	int size = 2+1+ndata_tx+1+2;
	unsigned char checkSum_tx = menu+ndata_tx;
	memset(tx_buf,0,sizeof(tx_buf));
	tx_buf[0]=0xAA;
	tx_buf[1]=0xAA;
	tx_buf[2]=menu;
	tx_buf[3]=ndata_tx;
	va_list list;
	va_start(list,ndata_tx);
	int* temporary = (int*)malloc(ndata_tx*sizeof(int));
	for(int i=0;i<ndata_tx;i++)
	{
		temporary[i] = va_arg(list,int);
		checkSum_tx += temporary[i];
		tx_buf[4+i]=(unsigned char)temporary[i];
	}
	va_end(list);
	tx_buf[size-2]=checkSum_tx;
	tx_buf[size-1]=0x55;
	tx_buf[size-0]=0x55;
	HAL_UART_Transmit(&huart3,(unsigned char*)tx_buf,(size+1)*sizeof(unsigned char),1000);
	memset(tx_buf,0,sizeof(tx_buf));
	fflush(stdin);fflush(stdout);
	free(temporary);
}

//-----------------------------------------------------------------------------------------Command Error
void refresh(int n)
{
	buf[0]=0xff;
	buf[1]=0xff;
	buf[2]=n;
	for(int i=0;i<6;i++){buf[3+i] = rx_buf[i];}
	HAL_UART_Transmit(&huart3,(unsigned char*)buf,10*sizeof(unsigned char),1000);
	idx = 0;
	stopReceiving=0;
	memset(tx_buf,0,sizeof(tx_buf));
	memset(rx_buf,0,sizeof(rx_buf));
	memset(buf,0,sizeof(buf));
	menu=ndata=checkSum=flag_checkSum=flag_ndata=0;
	stopReceiving=0;
}

//----------------------------------------------------------------------------------------------1)LED
void read_led_state(unsigned char LDn)
{
	int led_state;
	led_state = ((GPIOB->ODR)>>bitPosition[LDn])&1;
	response(2,LDn,led_state);
}
void write_led_state(unsigned char LDn,unsigned char state)
{
	switch(state)
	{
	case 0:
		GPIOB->ODR &=~(1<<bitPosition[LDn]);
		break;
	case 1:
		GPIOB->ODR |= 1<<bitPosition[LDn];
		break;
	}
	response(1,LDn);
}
void LED(void)
{
	if(ndata==1)read_led_state(data[0]);
	else if(ndata==2)write_led_state(data[0],data[1]);
}

//---------------------------------------------------------------------------------------------2)PWM
void write_pwm_frequency(unsigned char frequency_kHz)
{
	uint16_t prescaler = TIM4->PSC;
	uint32_t arr =  -1 + sysClock_Hz/((prescaler+1)*(int)frequency_kHz*1000);
	uint32_t ccr1,ccr2,ccr3,ccr4,prearr = TIM4->ARR;
	TIM4->ARR = arr;
	ccr1=TIM4->CCR1;	ccr2=TIM4->CCR2;	ccr3=TIM4->CCR3;	ccr4=TIM4->CCR4;
	TIM4->CCR1 = round(ccr1*(float)(arr+1)/(prearr+1));
	TIM4->CCR2 = round(ccr2*(float)(arr+1)/(prearr+1));
	TIM4->CCR3 = round(ccr3*(float)(arr+1)/(prearr+1));
	TIM4->CCR4 = round(ccr4*(float)(arr+1)/(prearr+1));
	response(1,frequency_kHz);
}
void write_pwm_duty_mode(unsigned char ch,unsigned char duty,unsigned char mode)
{
	uint32_t ccr = round(((TIM4->ARR)+1)*((int)duty/100.0));
	switch((int)ch)
	{
	case 1:
		TIM4->CCR1 = ccr;
		if((int)mode==0)	TIM4->CCMR1 &=~(1<<4);
		else TIM4->CCMR1 |=(1<<4);
		break;
	case 2:
		TIM4->CCR2 = ccr;
		if((int)mode==0)	TIM4->CCMR1 &=~(1<<12);
		else TIM4->CCMR1 |=(1<<12);
		break;
	case 3:
		TIM4->CCR3 = ccr;
		if((int)mode==0)	TIM4->CCMR2 &=~(1<<4);
		else TIM4->CCMR2 |=(1<<4);
		break;
	case 4:
		TIM4->CCR4 = ccr;
		if((int)mode==0)	TIM4->CCMR2 &=~(1<<12);
		else TIM4->CCMR2 |=(1<<12);
		break;
	}
	response(1,ch);
}
void PWM(void)
{
	if(ndata==1)write_pwm_frequency(data[0]);
	else if(ndata==3)write_pwm_duty_mode(data[0],data[1],data[2]);
}

//---------------------------------------------------------------------------------------------3)SERVO
void write_servo_angle(unsigned char ch, unsigned char angle)
{
	uint32_t ccr = (uint32_t)(min_motor+((max_motor-min_motor)/180.0)*angle);
	switch(ch)
	{
	case 0:
		TIM2->CCR1 = ccr;
		break;
	case 1:
		TIM2->CCR2 = ccr;
		break;
	}
	response(1,ch);
}
void read_servo_angle(unsigned char ch)
{
	uint32_t ccr = (ch==0)?TIM2->CCR1:TIM2->CCR2;
	unsigned char angle = (uint32_t)(180.0*(ccr-min_motor)/(max_motor-min_motor));
	response(2,ch,angle);
}
void SERVO(void)
{
	if(ndata==1)read_servo_angle(data[0]);
	else if(ndata==2)write_servo_angle(data[0],data[1]);
}

//---------------------------------------------------------------------------------------------4)BUZZER

void write_buzzer_frequency(unsigned char frequency_Hz_MSB,unsigned char frequency_Hz_LSB)
{
	int frequency_Hz;
	frequency_Hz  = ((int)frequency_Hz_MSB)<<8;
	frequency_Hz |= (int)frequency_Hz_LSB;
	uint16_t prescaler = TIM3->PSC;
	uint32_t arr = -1 + sysClock_Hz/((prescaler+1)*frequency_Hz);
	TIM3->ARR = arr;
	TIM3->CCR1 = (arr+1)/2;
	response(2,frequency_Hz_MSB,frequency_Hz_LSB);
}
void BUZZER(void)
{
	write_buzzer_frequency(data[0],data[1]);
}
//--------------------------------------------------------------------------------------------5)ADC
void read_adc_ch1_or_temperature(unsigned char ch)
{
	HAL_ADC_Start_DMA(&hadc1,&analog_reading[0],2);
	double temperature, fraction, integer;
	switch(ch)
	{
	case 0:
		response(2,analog_reading[0]/16,analog_reading[0]%16);
		break;
	case 1:
		temperature = (((double)analog_reading[1]/4095.0)*3.3-0.76)*400+25;
		fraction = temperature-(int)temperature;
		integer = temperature-fraction;
		// slope = 2.5E-3[V/C] = (V-V_25)/(T-25[C])    where V_25 = 0.76[V]
		response(2,(unsigned char)(int)round(integer),(unsigned char)(int)round(100*fraction));
		break;
	}
	HAL_ADC_Stop_DMA(&hadc1);
}
void ANALOG(void)
{
	read_adc_ch1_or_temperature(data[0]);
}

//--------------------------------------------------------------------------------------------Menu array
void (*serve_the_menu[5])(void) = {LED,PWM,SERVO,BUZZER,ANALOG};



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//*************************************************
//*         function pointer array                *
//*************************************************

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3,(unsigned char*)&rx_data,sizeof(rx_data));
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//pwm1
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);//pwm2
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//pwm3
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//pwm4
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//servo0
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//servo1
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//buzzer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		//HAL_UART_Transmit(&huart3,(unsigned char*)&rx_data,sizeof(rx_data),1000);
		if(stopReceiving!=2)
		{
			rx_buf[idx++]=rx_data;
			if(rx_data==0x55 && stopReceiving==0){stopReceiving=1;}
			else if(rx_data==0x55 && stopReceiving==1)
			{
				stopReceiving=2;
				if(stopReceiving ==2)
				{
					if(idx==0){}
					else
					{
						unsigned char valid_ndata_values[5][2]={{1,2},{1,3},{1,2},{2,0},{1,0}};
						int valid_ndata_count[5]={2,2,2,1,1};
						menu=rx_buf[2];
						ndata=rx_buf[3];
						checkSum = rx_buf[idx-3];
						data = (unsigned char*)malloc(ndata*sizeof(unsigned char));
						flag_checkSum = rx_buf[2]+rx_buf[3];
						buf4[0]=menu;
						buf4[1]=ndata;
						buf4[2]=flag_checkSum;
						for(int i=0;i<ndata;i++)
						{
							data[i] = rx_buf[4+i];
							flag_checkSum+=rx_buf[4+i];
							buf4[i+3]=flag_checkSum;
						}
						for(int i=0;i<valid_ndata_count[menu-1];i++)
						{
							if(valid_ndata_values[menu-1][i]==ndata) {flag_ndata=1;}
						}
						//CHECK IF IT IS A VALID COMMAND
						     if(!(rx_buf[0]== 0xAA && rx_buf[1]==0xAA))	{refresh(1);}
						else if(menu<1 || menu>5)						{refresh(2);}
						else if(flag_ndata==0) 							{refresh(3);}
						else if(flag_checkSum!=checkSum)				{refresh(4);}
						else {serve_the_menu[menu-1]();}
						free(data);
					}
					memset(rx_buf,0,sizeof(rx_buf));
					idx=0;
					stopReceiving = 0;
					flag_checkSum=flag_ndata=0;
				}
			}
		}
		HAL_UART_Receive_IT(&huart3,(unsigned char*)&rx_data,sizeof(rx_data));
	}
}
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance ==ADC1)
	{
		if(is1st==1)
		{
			HAL_ADC_Start_IT(&hadc1);
			analog_reading[0] =HAL_ADC_GetValue(&hadc1);
			is1st=0;
		}
		else
		{
			analog_reading[1] =HAL_ADC_GetValue(&hadc1);
			char temp[30];
			sprintf(temp,"(%d,%d)\r\n",analog_reading[0],analog_reading[1]);
			HAL_UART_Transmit(&huart3,temp,strlen(temp),1000);
		}
	}
}
*/

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
