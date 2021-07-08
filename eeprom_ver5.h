/*
 * stm32_ds3231_at24c32.h
 *
 *  Created on: Jul 4, 2021
 *      Author: yopla
 */

//#ifndef INC_STM32_DS3231_AT24C32_H_
//#define INC_STM32_DS3231_AT24C32_H_


//#if 0
#ifndef STM32_DS3231_AT24C32_H_
#define STM32_DS3231_AT24C32_H_

#include <stdlib.h>
#include <stdbool.h>
#include "stdint.h"

#define DS3231_REG_TIME 0X00
#define DS3231_REG_ALARM1 0X07
#define DS3231_REG_ALARM2 0X0B
#define DS3231_REG_CONTROL 0X0E
#define DS3231_REG_STATUS 0X0F
#define DS3231_REG_TEMP 0X11
#define DS3231_CON_EOSC 0X80
#define DS3231_CON_BBSQW 0X40
#define DS3231_CON_CONV 0X20
#define DS3231_CON_RS2 0X10
#define DS3231_CON_RS1 0X08
#define DS3231_CON_INTCN 0X04
#define DS3231_CON_A2IE 0X02
#define DS3231_CON_A1IE 0X01
#define DS3231_STA_OSF 0X80
#define DS3231_STA_32KHZ 0X08
#define DS3231_STA_BSY 0X04
#define DS3231_STA_A2F 0X02
#define DS3231_STA_A1F 0X01
/*
 *-------------------------------------------------------------------------------
 * 				|MSB	|		|		|		|		|		|		|LSB	|
 *-------------------------------------------------------------------------------
 TIME SEC[0X00] |0		|SEC/10					|SEC%10							|
 	  MIN[0X01] |0		|MIN/10					|MIN%10							|
 	 HOUR[0X02] |0		|12(24)	|P(AM)**|HOUR/10|HOUR%10						|
*--------------------------------------------------------------------------------
 	  DAY[0X03] |0		|0		|0		|0		|0		|DAY					|
 	 DATE[0X04] |0		|0		|DATE/10		|DATE%10						|
 	MONTH[0X05] |CENTURY|0		|0		|MON/10	|MON%10							|
 	 YEAR[0X06] |YEAR/10						|YEAR%10						|
*--------------------------------------------------------------------------------
 ALM1 SEC[0X07] |A1M1	|SEC/10					|SEC%10							|
 	  MIN[0X08] |A1M2	|MIN/10					|MIN%10							|
 	 HOUR[0X09] |A1M3	|12(24)	|P(AM)**|HOUR/10|HOUR%10						|
 	 DATE[0X0A] |A1M4	|DY(DT)*|DATE/10		|DAY OR DATE%10					|
*--------------------------------------------------------------------------------
 ALM2 MIN[0X0B] |A2M2	|MIN/10					|MIN%10							|
 	 HOUR[0X0C] |A2M3	|12(24)	|P(AM)**|HOUR/10|HOUR%10						|
 	 DATE[0X0D] |A2M4	|DY(DT)*|DATE/10		|DATE%10						|
*--------------------------------------------------------------------------------
 Ctrl	 [0X0E] |(EOSC)	|BBSQW	|CONV	|RS2	|RS1	|INTCN	|A2IE	|A1IE	|
 Status	 [0X0F] |OSF	|0		|0		|0		|EN32kHz|BSY	|A2F	|A1F	|
 offset  [0X10] |SIGN	|DATA	|DATA	|DATA	|DATA	|DATA	|DATA	|DATA	|
*--------------------------------------------------------------------------------
 T(MSB)	 [0X11] |SIGN	|DATA	|DATA	|DATA	|DATA	|DATA	|DATA	|DATA	|
 T(LSB)  [0X12] |DATA	|DATA	|0		|0		|0		|0		|0		|0		|
 *-------------------------------------------------------------------------------

-(**)or HOUR/20
-(*) DY(DAY OF THE WEEK) DT(DATE OF THE MONTH)
-(DT(DY)){A1M1,A1M2,A1M3,A1M4}
				=(X){1,1,1,1} PER SECOND		ALARM_MODE_ONCE_PER_SECOND
				=(X){1,1,1,0} SS				ALARM_MODE_SEC_MATCHED
				=(X){1,1,0,0} MM:SS				ALARM_MODE_MIN_SEC_MATCHED
				=(X){1,0,0,0} HH:MM:SS			ALARM_MODE_HOUR_MIN_SEC_MATCHED
				=(1){0,0,0,0} HH:MM:SS && DATE	ALARM_MODE_ALL_MATCHED
				=(0){0,0,0,0} HH:MM:SS && DAY
-(DT(DY)){A2M2,A2M3,A2M4}
				=(X){1,1,1} PER MINUTE (SS=00)
				=(X){1,1,0} MM
				=(X){1,0,0} HH:MM
				=(1){0,0,0} HH:MM && DATE
				=(0){0,0,0} HH:MM && DAY
- CONTROL :
	(EOSC)[Enable OSCillator]					0(OS started), 1(OS stopped)
	BBWQS [Battery-Backed Square-Wave Enable]	1(enable Square Wave with INTCN=1 and Vcc<Vpf)
	CONV  [Convert Temperature]					1(forces the T sensor to convert T into BCD and updates capacitance array to the OSC)
	RS2,1 [Rate Select]							Frequency of the Square-Wave
				={0,0} 	  1[Hz]
				={0,1} 2^10[Hz] (1024)
				={1,0} 2^12[Hz] (4096)
				={1,1} 2^13[Hz] (8192)
	INTCN [INTerrupt CoNtrol]					1(Square Wave is output on the (INT)/SQW pin
	AnIE  [Alarm n Interrupt Enable]			1(permits AnF(alarm n flag))
-STATUS  :
	OSF	  [Oscillator Stop Flag]             	1(oscillator is stopped or was stopped)
	EN32kHz [ENable 32kHz]						1(32kHz pin is enabled and outputs the square wave of that frequency)
	BSY	  [Busy]								1(T-conversion signal is asserted.)
	AnF	  [Alarm n Flag]
	Aging Offset
-TEMPERATURE : resolution = 0.25[degC]
	MSB(integer).LSB(fraction)
	(ex) 0b 0001 1001 01  [0]
	        |-------|----| : (16+8+1) + (1*0.25) = 25.25[degC]
 */

typedef enum{
	ALARM_MODE_ALL_MATCHED = 0,
	ALARM_MODE_HOUR_MIN_SEC_MATCHED,
	ALARM_MODE_MIN_SEC_MATCHED,
	ALARM_MODE_SEC_MATCHED,
	ALARM_MODE_ONCE_PER_SECOND
}AlarmMode;

typedef enum{SUNDAY=1,MONDAY,TUESDAY,WEDNESDAY,THURSDAY,FRIDAY,SATURDAY}DaysOfWeek;
typedef struct{uint8_t Year,Month,Date,DaysOfWeek,Hour,Min,Sec;}_RTC;

void DS3231_AT24C32_Init(I2C_HandleTypeDef* handle);
//bool DS3231_GetTime(_RTC* rtc);
bool DS3231_GetTime(_RTC* rtc);
bool DS3231_SetTime(_RTC* rtc);
bool DS3231_SetAlarm1(AlarmMode mode,uint8_t date,uint8_t hour,uint8_t min, uint8_t sec);
bool DS3231_ClearAlarm1();
bool DS3231_ReadTemerature(float* temp);

bool DS3231_Register_read(uint8_t start_address, uint8_t* read_buf);
bool DS3231_Register_write(uint8_t start_address, uint8_t* write_buf);

bool AT24C32_EEPROM_read(uint16_t start_address, uint8_t* read_buf,uint16_t ndata);
bool AT24C32_EEPROM_write(uint16_t start_address, uint8_t* write_buf,uint16_t ndata);
uint8_t B2D(uint8_t bcd);
uint8_t D2B(uint8_t decimal);

bool EEPROM_ReadByte(uint16_t addr, uint8_t *data);

bool EEPROM_WriteByte(uint16_t addr, uint8_t data);

#endif /*STM32_DS3231_AT24_C32_H_*/
//#endif


//#endif /* INC_STM32_DS3231_AT24C32_H_ */
