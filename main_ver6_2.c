
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
int narray=8;
typedef struct{
	//uint8_t x[16];
	uint8_t name[8];
	uint8_t id[8];
	uint8_t pw[8];
}member;
typedef struct{
	uint8_t id[16];
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
char tokens[10][16]={'\0'};
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

void lf()
{
	memset(tx_buf,0,sizeof(tx_buf));
	sprintf(tx_buf,"\r\n> ");
	HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	memset(tx_buf,0,sizeof(tx_buf));
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

  DS3231_AT24C32_Init(&hi2c1);

  _RTC rtc1;
  sprintf(tx_buf,"\r\n[%d%d] %d-%d-%d %d:%d:%d\r\n",\
		  DS3231_SetTime(&rtc),DS3231_GetTime(&rtc1),\
		  rtc1.Year,rtc1.Month,rtc1.Date,\
		  rtc1.Hour,rtc1.Min,rtc1.Sec);
  HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);


  //*****************************************************************************************************goto
  member me;
  uint8_t* ptr_me = (uint8_t*)&me;
  memset(me.name,0,sizeof(me.name));
  memset(me.id,0,sizeof(me.id));
  memset(me.pw,0,sizeof(me.pw));

  AT24C32_EEPROM_write((uint16_t)2000, ptr_me, size_of_member);


  char* tempname = (char*)calloc(narray,sizeof(char));
  char* tempid	= (char*)calloc(narray,sizeof(char));
  char* temppw	= (char*)calloc(narray,sizeof(char));


  sprintf(tempname,"%s","HYOYEON\0");
  sprintf(tempid,"%s","imango\0");
  sprintf(temppw,"%s","1234567\0");
  for(int i=0;i<narray;i++)
  {
	  //me.x[i]=D2B(tempx[i]);
	  me.name[i]=D2B(tempname[i]);
	  me.id[i]=D2B(tempid[i]);
	  me.pw[i]=D2B(temppw[i]);
  }

  AT24C32_EEPROM_write((uint16_t)0xbb8, ptr_me, size_of_member);
//***************************

  member one;
  		uint8_t* ptr_one = (uint8_t*)&one;
  		AT24C32_EEPROM_read((uint16_t)0xbb8,ptr_one,size_of_member);
  		//AT24C32_EEPROM_read(4070,ptr_one,size_of_member);

	  memset(tx_buf,0,sizeof(tx_buf));

	  for(int i=0;i<narray;i++)
	  {
	  one.name[i] = B2D(one.name[i]);
	  one.id[i]   = B2D(one.id[i]);
	  one.pw[i]   = B2D(one.pw[i]);
	  }

	 //strcpy(tx_buf," [temp] ");HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	//  HAL_UART_Transmit(&huart3,(uint8_t*)one.x,strlen(one.x),1000);

	  strcpy(tx_buf," [name] ");HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	  HAL_UART_Transmit(&huart3,(uint8_t*)one.name,sizeof(one.name),1000);

	  strcpy(tx_buf," [ID] ");HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	  HAL_UART_Transmit(&huart3,(uint8_t*)one.id,sizeof(one.id),1000);

	  strcpy(tx_buf," [PW] ");HAL_UART_Transmit(&huart3,(uint8_t*)tx_buf,strlen(tx_buf),1000);
	  HAL_UART_Transmit(&huart3,(uint8_t*)one.pw,sizeof(one.pw),1000);


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
							HAL_UART_AbortReceive_IT(&huart3);
							serve_the_menu[i]();
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
