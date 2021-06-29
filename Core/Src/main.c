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

#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "arduino_related.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
typedef enum{INPUT,OUTPUT}pin_mode;
typedef enum{LOW,HIGH}pin_state;
*/
//D0~5: PG9,PC4,PF15,PF13,PF14,PE11
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LONG_CLICK_MIN 1100
#define LONG_CLICK_MAX 2000
#define DOUBLE_CLICK_MIN 10
#define DOUBLE_CLICK_MAX 200
#define NOISE_MAX 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern GPIO_TypeDef* arduino_pin_letter[20];
extern GPIO_TypeDef* arduino_pin_num[20];
extern int TIM_CHANNEL_n[14];
extern TIM_HandleTypeDef htim;

char rx_data;
char rx_buf[100];
char tx_buf[300];
int idx,menu;
char* low_or_high[2]={"LOW","HIGH"};
char str1[100];
char str[100];

int myPin,myState,is1st;
int32_t t0,t_1st_rising,t_1st_falling,t_2nd_rising,t_2nd_falling,dt;
int ninterrupt=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void print_menu()
{
	sprintf(str,"\r\n\r\n  **********************\r\n");
	strcat(tx_buf,str);
	sprintf(str,"  *        MENU        *\r\n");
	strcat(tx_buf,str);
	sprintf(str,"  **********************\r\n");
	strcat(tx_buf,str);
	sprintf(str,"    [1] test digitalWrite() : write high and low on D0~013 \r\n");
	strcat(tx_buf,str);
	sprintf(str,"    [2] test digitalRead() : read states of D0~D13\r\n");
	strcat(tx_buf,str);
	sprintf(str,"    [3] test analogWrite() : generate PWM on D{3,5,6,9,10,11}\r\n");
	strcat(tx_buf,str);
	sprintf(str,"    [4] test analogRead() : read value of analog channels A0~A5\r\n");
	strcat(tx_buf,str);
	sprintf(str,"    [5] test tone(), noTone() : make sounds of musical notes\r\n");
	strcat(tx_buf,str);
	sprintf(str,"----------> choose the menu from 1~5   : ");
	strcat(tx_buf,str);
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
}

void refresh()
{
		sprintf(str,"----------> choose the menu from 1~5   : ");
		strcat(tx_buf,str);
		HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
        memset(tx_buf,0,sizeof(tx_buf));
        memset(rx_buf,0,sizeof(rx_buf));
        idx = 0;
}
/**************************
* TEST1 : digitalWrite()  *
***************************/
void test_digitalWrite()
{
	  for(int i=0;i<14;i++)
	  {
		  pinMode(i,OUTPUT);
		  digitalWrite(i,HIGH);
		  HAL_Delay(200);
		  digitalWrite(i,LOW);
	  }
}
/**************************
* TEST2 : digitalRead()  *
**************************/
void test_digitalRead()
{
	   for(int i=0;i<14;i++)
	   {
		   pinMode(i,OUTPUT);
		   digitalWrite(i,HIGH);
		   pinMode(i,INPUT);
		   sprintf(tx_buf,"\r\nstate of pin%d = %s\r\n",i,low_or_high[digitalRead(8)]);
		   HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
		   memset(tx_buf,0,sizeof(tx_buf));
	   }
}
/***************************
 * TEST3 : analogWrite()   *
 ***************************/
void test_analogWrite()
{
	  int pins[6]={3,5,6,9,10,11};
	  for (int j=0;j<255;j++)
	  {
		  for(int i=0;i<6;i++)
		  {
			  analogWrite(pins[i], 2*j%255);
		  }
		  HAL_Delay(10);
	  }
	  for(int i=0;i<6;i++)
	  {
		  HAL_TIM_PWM_Stop(&htim,TIM_CHANNEL_n[11]);
	  }
}
/**************************
 * TEST4 : analogRead()   *
 **************************/
void test_analogRead()
{
	  sprintf(tx_buf,"\r\n [TEST : analogRead(A#) ]\r\n");
	  HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	  memset(tx_buf,0,sizeof(tx_buf));
	  for(int i=A0;i<=A5;i++)
	  {
		  pinMode(i,INPUT);
		  uint16_t val=analogRead(i);
		  sprintf(tx_buf,"(A%d : %d)",i-14,val);
		  HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
		  memset(tx_buf,0,sizeof(tx_buf));
		  HAL_GPIO_DeInit(arduino_pin_letter[i], arduino_pin_num[i]);
		  HAL_Delay(100);
	  }
}
/***************************
 * TEST5 : tone() noTone() *
 ***************************/
void test_tone_noTone()
{
	  int doremi[8]={523,587,659,698,784,880,988,1047};
	  for(int i=0;i<8;i++)
	  {
		  tone(11,doremi[i]);
		  HAL_Delay(500);
		  noTone(11);
		  HAL_Delay(1000);
	  }
	  HAL_TIM_PWM_Stop(&htim,TIM_CHANNEL_n[11]);
}
//-------------------------------------------
 void (*serve_the_menu[5])(void) = {\
 		test_digitalWrite, test_digitalRead, test_analogWrite, test_analogRead, test_tone_noTone\
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
  MX_USART3_UART_Init();
  MX_ADC3_Init();
 // MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  print_menu();
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
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState pin;
	if(GPIO_Pin == GPIO_PIN_13)
	{
		pin=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
//---------------------------------------------------------------1st rising
		if(ninterrupt==0 && pin==1)
		{
			t_1st_rising = HAL_GetTick();
			ninterrupt++;
		}
//---------------------------------------------------------------1st falling
		else if(ninterrupt==1 && pin==0)
		{
			t_1st_falling = HAL_GetTick();
			dt = t_1st_falling - t_1st_rising;
			if(dt>=LONG_CLICK_MIN && dt<=LONG_CLICK_MAX)
			{
				sprintf(str1,"LONG CLICK\r\n");
				ninterrupt=0;
			}
			else if(dt>0 && dt<=NOISE_MAX){	ninterrupt=0;}
			else if(dt>DOUBLE_CLICK_MAX+10 && dt<LONG_CLICK_MIN-10)
			{
				sprintf(str1, "Select Key. (interval = %d )\r\n",(int)dt);
				ninterrupt=0;
			}
			else if(dt>=DOUBLE_CLICK_MIN && dt<=DOUBLE_CLICK_MAX){ninterrupt++;}
		}
//---------------------------------------------------------------2nd rising
		else if(ninterrupt == 2 && pin==1)
		{
			t_2nd_rising = HAL_GetTick();
			dt = t_2nd_rising-t_1st_falling;
			if(dt>=DOUBLE_CLICK_MIN && dt<=DOUBLE_CLICK_MAX)
			{
				ninterrupt++;
			}
			else ninterrupt=0;
		}
//---------------------------------------------------------------2nd falling
		else if(ninterrupt==3 && pin==0)
		{
			t_2nd_falling = HAL_GetTick();
			dt = t_2nd_falling - t_2nd_rising;
			if(dt>=DOUBLE_CLICK_MIN && dt<=DOUBLE_CLICK_MAX)
			{
				sprintf(str1, "DOUBLE CLICK\r\n");
				ninterrupt=0;
			}
			else ninterrupt=0;
		}
		pin = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
		HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),1000);
		memset(str1,0,sizeof(str1));
		dt=0;
	}
}
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
        	//else if(idx>4)refresh();
            else
            {
            	int temp;
            	sscanf(rx_buf,"%d",&temp);
            	if(temp<1 || temp>5)refresh();
            	else
            	{
            		serve_the_menu[temp]();
            		print_menu();
            	}
            }
        	idx=0;
        	memset(rx_buf,0,sizeof(rx_buf));
        	memset(tx_buf,0,sizeof(tx_buf));
        	print_menu();
        }
        else if(rx_data!='\b'){rx_buf[idx++]=rx_data;}
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
