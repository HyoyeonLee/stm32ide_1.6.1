/*
 * arduino_gpio_api.c
 *
 *  Created on: Jun 23, 2021
 *      Author: ctc
 */
#include "main.h"

#include "arduino_gpio_api.h"
/*
 * D0 = PG9
 * D1 = PC4
 * D2 = PF15
 * D3 = PE13
 * D4 = PF14
 * D5 = PE11
 */
void pinMode(int pin, enum pin_state mode)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(pin ==0)
  {
	  if(mode == INPUT)
	  {
		  GPIO_InitStruct.Pin = GPIO_PIN_9;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	  }
	  else
	  {
		  GPIO_InitStruct.Pin = GPIO_PIN_9;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  }
  }
}
void ditalWrite(int pin, enum pin_level level)
{
  if(pin ==0)
  {
    if(level == LOW)
    {
    	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
    }
  }
}
