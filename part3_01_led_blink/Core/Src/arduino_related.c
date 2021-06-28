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
				GPIOA,	GPIOC,	GPIOC,	GPIOF,	GPIOF,	GPIOF 	\
		};
uint16_t arduino_pin_num[20]=
	{
		GPIO_PIN_9,		GPIO_PIN_14,	GPIO_PIN_15,	GPIO_PIN_13,	GPIO_PIN_14,			\
		GPIO_PIN_11,	GPIO_PIN_9,		GPIO_PIN_13,	GPIO_PIN_12,	GPIO_PIN_15,			\
		GPIO_PIN_14,	GPIO_PIN_7,		GPIO_PIN_6,		GPIO_PIN_5,				    			\
		GPIO_PIN_3,		GPIO_PIN_0,		GPIO_PIN_3,		GPIO_PIN_3,		GPIO_PIN_5,  GPIO_PIN_10\
	};

uint32_t channel[6] =
	{
			ADC_CHANNEL_3,
			ADC_CHANNEL_10,
			ADC_CHANNEL_13,
			ADC_CHANNEL_9,
			ADC_CHANNEL_15,
			ADC_CHANNEL_8
	};


//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim4;
//TIM_HandleTypeDef htimx[6] 	= {htim1,htim1,htim1,htim4,htim4,htim3};
TIM_TypeDef* TIMx[6] 		= {TIM1,TIM1,TIM1,TIM4,TIM4,TIM3};
uint32_t TIM_CHANNEL_n[6] = {TIM_CHANNEL_3,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_4,TIM_CHANNEL_3,TIM_CHANNEL_2};

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
		sConfig.Channel = channel[pin-14];
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
		if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
		{
		   Error_Handler();
		}

	}
}
































