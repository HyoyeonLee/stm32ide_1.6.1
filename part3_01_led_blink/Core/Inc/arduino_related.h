/*
 * arduino_related.h
 *
 *  Created on: Jun 23, 2021
 *      Author: hylee
 */
#include "main.h"

#ifndef INC_ARDUINO_RELATED_H_
#define INC_ARDUINO_RELATED_H_

typedef enum{INPUT,OUTPUT}pin_mode;
typedef enum{LOW,HIGH}pin_state;

#define A0 ((int)14)
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19


void setup();
void loop();

void pinMode(int pin,pin_mode mode);
void digitalWrite(int pin, pin_state state);
int digitalRead(int pin);
uint16_t analogRead(int pin);

#endif /* INC_ARDUINO_RELATED_H_ */
