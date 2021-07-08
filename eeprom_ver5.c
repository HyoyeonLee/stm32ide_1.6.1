//#if 0
#include "stm32f4xx_hal.h"
#include "stm32_ds3231_at24c32.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#define ADDRESS_DS3231 (0x68<<1)
#define ADDRESS_AT24C32 (0x57<<1)


I2C_HandleTypeDef* i2c;



uint8_t B2D(uint8_t bcd) {  return (bcd >> 4) * 10 + (bcd & 0x0F);}
uint8_t D2B(uint8_t decimal) {  return (((decimal / 10) << 4) | (decimal % 10));}
void DS3231_AT24C32_Init(I2C_HandleTypeDef* handle){i2c=handle;}


//---------------------------------------------------------------------------------------------------AT24C32 -Mine
bool AT24C32_EEPROM_read(uint16_t start_address, uint8_t* read_buf,uint16_t buf_size)
{
	if(HAL_I2C_Mem_Read(i2c, 0x57<<1, start_address,  sizeof(uint16_t), read_buf, buf_size,  HAL_MAX_DELAY)!=HAL_OK){return false;}
	HAL_Delay(buf_size*10);
	return true;
}

bool AT24C32_EEPROM_write(uint16_t start_address, uint8_t* write_buf,uint16_t buf_size)
{
	if(HAL_I2C_Mem_Write(i2c,0x57<<1, start_address, sizeof(uint16_t), write_buf, buf_size,  HAL_MAX_DELAY)!=HAL_OK){return false;}
	HAL_Delay(buf_size*10);
	return true;
}
//--------------------------------------------------------------------------------------------------DS3231
bool DS3231_GetTime(_RTC *rtc)
{
  uint8_t startAddr = DS3231_REG_TIME;
  uint8_t buffer[7] = {0,};

  if(HAL_I2C_Master_Transmit(i2c,  ADDRESS_DS3231, &startAddr, 1, HAL_MAX_DELAY) != HAL_OK) return false;
  if(HAL_I2C_Master_Receive(i2c,  ADDRESS_DS3231, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;

  rtc->Sec = B2D(buffer[0] & 0x7F);		//0111(sec/10) 	1111(sec%10)
  rtc->Min = B2D(buffer[1] & 0x7F);		//0111(min/10) 	1111(min%10)
  rtc->Hour = B2D(buffer[2] & 0x3F);	//0011(hour/10)	1111(hour%10)
  rtc->DaysOfWeek = buffer[3] & 0x07;	//0000() 		0111(day)
  rtc->Date = B2D(buffer[4] & 0x3F);	//0011(date/10) 1111(date%10)
  rtc->Month = B2D(buffer[5] & 0x1F);	//0001(month/10)1111(month%10)
  rtc->Year = B2D(buffer[6]);			//1111(year/10) 1111(year%10)

  return true;
}

bool DS3231_SetTime(_RTC *rtc)
{
  uint8_t startAddr = DS3231_REG_TIME;
  uint8_t buffer[8] = {startAddr, D2B(rtc->Sec), D2B(rtc->Min), D2B(rtc->Hour), rtc->DaysOfWeek, D2B(rtc->Date), D2B(rtc->Month), D2B(rtc->Year)};
  if(HAL_I2C_Master_Transmit(i2c,  ADDRESS_DS3231, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK) return false;
  return true;
}

bool DS3231_Register_read(uint8_t start_address, uint8_t* read_buf)
{
	if (HAL_I2C_Master_Transmit(i2c, ADDRESS_DS3231, &start_address, sizeof(start_address), HAL_MAX_DELAY) != HAL_OK) return false;
	if (HAL_I2C_Master_Receive(i2c, ADDRESS_DS3231, read_buf, sizeof(read_buf), HAL_MAX_DELAY) != HAL_OK) return false;
	return true;
}

bool DS3231_Register_write(uint8_t start_address, uint8_t* write_buf)
{
	uint8_t* temp_buf;
	temp_buf = (uint8_t*)malloc(sizeof(write_buf)+sizeof(write_buf[0]));
	temp_buf[0]=start_address;
	for(int i=0;i<sizeof(write_buf)/sizeof(write_buf);i++)
	{
		temp_buf[i+1]=write_buf[i];
	}
	if (HAL_I2C_Master_Transmit(i2c, ADDRESS_DS3231, temp_buf, sizeof(temp_buf), HAL_MAX_DELAY) != HAL_OK)
	{
		free(temp_buf);
		return false;
	}

	free(temp_buf);
	return true;
}




bool DS3231_ReadTemperature(float* temp)
{
	uint8_t start_address;
	uint8_t* read_buf;

	start_address = DS3231_REG_TEMP;
	read_buf = (uint8_t*)calloc(2,sizeof(uint8_t));

	DS3231_Register_read(start_address,read_buf);
	*temp = ((read_buf[0]<<8 | read_buf[1])>>6)/4.0f;//read_buf[0]+(read_buf[1]>>6)*0.25
	free(read_buf);
	return true;
}

bool DS3231_SetAlarm1(uint8_t mode,uint8_t date, uint8_t hour, uint8_t min,uint8_t sec)
{
	uint8_t start_address;
	uint8_t* write_buf;
	uint8_t* read_buf;

	//--------------------------------------------------------------------------------Set the Alarm Time
	/*
	-(DT(DY)){A1M1,A1M2,A1M3,A1M4}
					=(X){1,1,1,1} PER SECOND		ALARM_MODE_ONCE_PER_SECOND
					=(X){1,1,1,0} SS				ALARM_MODE_SEC_MATCHED
					=(X){1,1,0,0} MM:SS				ALARM_MODE_MIN_SEC_MATCHED
					=(X){1,0,0,0} HH:MM:SS			ALARM_MODE_HOUR_MIN_SEC_MATCHED
					=(1){0,0,0,0} HH:MM:SS && DATE	ALARM_MODE_ALL_MATCHED
					=(0){0,0,0,0} HH:MM:SS && DAY
	*/
	typedef enum{ss=0,mm,hh,dd}index;
	start_address = DS3231_REG_ALARM1;
	write_buf = (uint8_t*)calloc(4,sizeof(uint8_t));;
	write_buf[ss]=D2B(sec);
	write_buf[mm]=D2B(min);
	write_buf[hh]=D2B(hour);
	write_buf[dd]=D2B(date);
	switch (mode)
	{
	case ALARM_MODE_ONCE_PER_SECOND:
		write_buf[ss] |= 1<<8;
		write_buf[mm] |= 1<<8;
		write_buf[hh] |= 1<<8;
		write_buf[dd] |= 1<<8;
		break;
	case ALARM_MODE_SEC_MATCHED:
	//	write_buf[ss] |= 1<<8;
		write_buf[mm] |= 1<<8;
		write_buf[hh] |= 1<<8;
		write_buf[dd] |= 1<<8;
		break;
	case ALARM_MODE_MIN_SEC_MATCHED:
	//	write_buf[ss] |= 1<<8;
	//	write_buf[mm] |= 1<<8;
		write_buf[hh] |= 1<<8;
		write_buf[dd] |= 1<<8;
		break;
	case ALARM_MODE_HOUR_MIN_SEC_MATCHED:
	//	write_buf[ss] |= 1<<8;
	//	write_buf[mm] |= 1<<8;
	//	write_buf[hh] |= 1<<8;
		write_buf[dd] |= 1<<8;
		break;
	case ALARM_MODE_ALL_MATCHED:
		break;
	default:break;
	}
	DS3231_Register_write(start_address,write_buf);
	free(write_buf);
	//--------------------------------------------------------------------------------Enable Alarm1 at Control Register
	// Ctrl	 [0X0E] |(EOSC)	|BBSQW	|CONV	|RS2	|RS1	|INTCN	|A2IE	|A1IE	|
	// INTCN [INTerrupt CoNtrol]					1(Square Wave is output on the (INT)/SQW pin
	// AnIE  [Alarm n Interrupt Enable]			1(permits AnF(alarm n flag))
	start_address = DS3231_REG_CONTROL;
	DS3231_Register_read(start_address,read_buf);

	*write_buf = *read_buf | (DS3231_CON_A1IE|DS3231_CON_INTCN);
	DS3231_Register_write(start_address,write_buf);

	free(write_buf);
	free(read_buf);
	return true;
}


bool DS3231_ClearAlarm1()
{
	uint8_t start_address;
	uint8_t* write_buf;
	uint8_t* read_buf;


	start_address = DS3231_REG_CONTROL;
	DS3231_Register_read(start_address,read_buf);
	*write_buf = *read_buf & (~DS3231_CON_A1IE);
	DS3231_Register_write(start_address,write_buf);

	start_address = DS3231_REG_STATUS;
	DS3231_Register_read(start_address,read_buf);
	*write_buf = *read_buf & (~DS3231_STA_A1F);
	DS3231_Register_write(start_address,write_buf);
}



























