/*
 * arduino_related.c
 *
 *  Created on: Jun 23, 2021
 *      Author: hylee
 */
#include "arduino_related.h"
#include "adc.h"
#include "stm32f4xx_hal_adc.h"

GPIO_TypeDef* arduino_pin_letter[20] =
		{
				GPIOG,	GPIOG,	GPIOF,	GPIOE,	GPIOF,			\
				GPIOE,	GPIOE,	GPIOF,	GPIOF,	GPIOD,			\
				GPIOD,	GPIOA,	GPIOA,	GPIOA, 					\
				GPIOA,	GPIOC,	GPIOC, 							\
				GPIOF,	GPIOF,	GPIOF 							\
		};
uint16_t arduino_pin_num[20] =
	{
		GPIO_PIN_9,		GPIO_PIN_14,	GPIO_PIN_15,	GPIO_PIN_13,	GPIO_PIN_14,			\
		GPIO_PIN_11,	GPIO_PIN_9,		GPIO_PIN_13,	GPIO_PIN_12,	GPIO_PIN_15,			\
		GPIO_PIN_14,	GPIO_PIN_7,		GPIO_PIN_6,		GPIO_PIN_5,				    			\
		GPIO_PIN_3,		GPIO_PIN_0,		GPIO_PIN_3,												\
		GPIO_PIN_3,		GPIO_PIN_5,  	GPIO_PIN_10												\
	};

uint32_t adc_channel[6] =
	{
			ADC_CHANNEL_3,		ADC_CHANNEL_10,		ADC_CHANNEL_13, \
			ADC_CHANNEL_9,		ADC_CHANNEL_15,		ADC_CHANNEL_8	\
	};


TIM_TypeDef* TIMx[14] =
	{
			0,				0,				0,				TIM1,			0,		\
			TIM1,			TIM1,			0,				0,				TIM4,	\
			TIM4,			TIM3,			0,				0						\
	};
uint32_t TIM_CHANNEL_n[14] =
	{
			0,				0,				0,				TIM_CHANNEL_3,	0,				\
			TIM_CHANNEL_2,	TIM_CHANNEL_1,	0,				0,				TIM_CHANNEL_4, 	\
			TIM_CHANNEL_3,	TIM_CHANNEL_2,	0,				0								\
	};
uint32_t prescaler[14]=
	{
			0,				0,				0,				320-1,			0,				\
			160-1,			160-1,			0,				0,				320-1,			\
			320-1,			320-1,			0,				0								\
	};
uint32_t alternate[14] =
	{
			0,				0,				0,				GPIO_AF1_TIM1,	0,				\
			GPIO_AF1_TIM1,	GPIO_AF1_TIM1,	0,				0,				GPIO_AF2_TIM4,	\
			GPIO_AF2_TIM4,	GPIO_AF2_TIM3,	0,				0								\
	};
TIM_HandleTypeDef htim;
void analogWrite_pre(int pin, int val)
{
	uint32_t ccr = (uint32_t)val*100.0/255;
	//TIM_HandleTypeDef htim;
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	htim.Instance = TIMx[pin];
	htim.Init.Prescaler = prescaler[pin];
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = 100-1;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim) != HAL_OK){Error_Handler();}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)			{Error_Handler();}
	if (HAL_TIM_PWM_Init(&htim) != HAL_OK){Error_Handler();}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)		{Error_Handler();}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = ccr;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_n[pin]) != HAL_OK)	{Error_Handler();}
	switch(pin)
	{
	case 11:
		__HAL_RCC_TIM3_CLK_ENABLE();
	//	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	//  HAL_NVIC_EnableIRQ(TIM3_IRQn);
		break;
	case 9:
	case 10:
		__HAL_RCC_TIM4_CLK_ENABLE();
		break;
	case 3:
	case 5:
	case 6:
		__HAL_RCC_TIM1_CLK_ENABLE();
		break;
	}
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	switch(pin)
	{
	case 3:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case 5:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case 6:
		__HAL_RCC_GPIOE_CLK_ENABLE();
		break;
	case 9:
		__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	case 10:
		__HAL_RCC_GPIOD_CLK_ENABLE();
		break;
	case 11:
		__HAL_RCC_GPIOA_CLK_ENABLE();
		break;
	}
	GPIO_InitStruct.Pin = arduino_pin_num[pin];
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = alternate[pin];
	HAL_GPIO_Init(arduino_pin_letter[pin], &GPIO_InitStruct);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_n[pin]);
	switch(pin)
	{
		case 3:
			TIM1->CCR3 = ccr;
			break;
		case 5:
			TIM1->CCR2 = ccr;
			break;
		case 6:
			TIM1->CCR1 = ccr;
			break;
		case 9:
			TIM4->CCR4 = ccr;
			break;
		case 10:
			TIM4->CCR3 = ccr;
			break;
		case 11:
			TIM3->CCR2 = ccr;
			break;
	}
}
void analogWrite(int pin,int val)
{
	analogWrite_pre(pin,val);
	analogWrite_pre(pin,val);
}
void tone(int pin, int frequency)
{
	int val = (int)255.0/2;
	analogWrite(pin,val);
	uint32_t pre = prescaler[pin];
	uint32_t arr = (uint32_t)( (int)(16000000.0/((pre+1)*frequency))-1);
	switch(pin)
	{
	case 3:
	case 5:
	case 6:
		TIM1->ARR = arr;
	case 9:
	case 10:
		TIM4->ARR = arr;
	case 11:
		TIM3->ARR = arr;
	}
}
void noTone(int pin)
{
	uint32_t ccr=1;
	switch(pin)
	{
	case 3:
		TIM1->CCR3 = ccr;
		break;
	case 5:
		TIM1->CCR2 = ccr;
		break;
	case 6:
		TIM1->CCR1 = ccr;
		break;
	case 9:
		TIM4->CCR4 = ccr;
		break;
	case 10:
		TIM4->CCR3 = ccr;
		break;
	case 11:
		TIM3->CCR2 = ccr;
		break;
	}
}
void digitalWrite(int pin,pin_state state)
{
	//if(state==LOW)GPIOG->ODR &= ~(1<<9);
	//else GPIOG->ODR |= 1<<9;
	HAL_GPIO_WritePin(arduino_pin_letter[pin], arduino_pin_num[pin], state);
}

int digitalRead(int pin)
{
	GPIO_PinState state= HAL_GPIO_ReadPin(arduino_pin_letter[pin], arduino_pin_num[pin]);
	if(state==GPIO_PIN_RESET)return 0;
	else return 1;
}
uint16_t analogRead(int pin)
{
	//to fix the reading 0 when first started
	HAL_ADC_Start_IT(&hadc3);
	HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
	//

	HAL_ADC_Start_IT(&hadc3);
	return HAL_ADC_GetValue(&hadc3);
	HAL_ADC_Stop(&hadc3);
}
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{

//}

void pinMode(int pin, pin_mode mode)
{
	if(pin<14)
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		/* GPIO Ports Clock Enable */
		if(pin>=11 && pin<=13)
			__HAL_RCC_GPIOA_CLK_ENABLE();
		//else if(pin==1)
		//	__HAL_RCC_GPIOC_CLK_ENABLE();
		else if(pin>=9 && pin<=10)
			__HAL_RCC_GPIOD_CLK_ENABLE();
		else if(pin==3 ||(pin>=5 && pin<=6))
			__HAL_RCC_GPIOE_CLK_ENABLE();
		else if(pin>=2 || pin==4||(pin>=7 && pin<=8))
			__HAL_RCC_GPIOF_CLK_ENABLE();
		else if(pin==0 || pin==1)
			__HAL_RCC_GPIOG_CLK_ENABLE();
		if(mode==INPUT)
		{
			GPIO_InitStruct.Pin = arduino_pin_num[pin];
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(arduino_pin_letter[pin], &GPIO_InitStruct);
		}
		else
		{
			GPIO_InitStruct.Pin = arduino_pin_num[pin];
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(arduino_pin_letter[pin], &GPIO_InitStruct);
		}
	}
	else
	{//-----------------------------------------------------------------------------
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		//if(adcHandle->Instance==ADC3)
		//{
		    __HAL_RCC_ADC3_CLK_ENABLE();
		    if(pin==A0)
			  __HAL_RCC_GPIOA_CLK_ENABLE();
		    else if(pin==A1 || pin==A2)
		  	  __HAL_RCC_GPIOC_CLK_ENABLE();
		    else if(pin>=3 && pin<=5)
		  	  __HAL_RCC_GPIOF_CLK_ENABLE();
		    GPIO_InitStruct.Pin = arduino_pin_num[pin];
		    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		    GPIO_InitStruct.Pull = GPIO_NOPULL;
		    HAL_GPIO_Init(arduino_pin_letter[pin], &GPIO_InitStruct);
		//  }
		  //-----------------------------------------------------------------------------
		ADC_ChannelConfTypeDef sConfig = {0};
		hadc3.Instance = ADC3;
		hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
		hadc3.Init.Resolution = ADC_RESOLUTION_12B;
		hadc3.Init.ScanConvMode = DISABLE;
		hadc3.Init.ContinuousConvMode = DISABLE;
		hadc3.Init.DiscontinuousConvMode = DISABLE;
		hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc3.Init.NbrOfConversion = 1;
		hadc3.Init.DMAContinuousRequests = DISABLE;
		hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		if (HAL_ADC_Init(&hadc3) != HAL_OK)
		{
		  Error_Handler();
		}
		sConfig.Channel = adc_channel[pin-14];
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
		{
		   Error_Handler();
		}

	}
}



































