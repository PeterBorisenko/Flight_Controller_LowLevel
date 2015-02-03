/*
 * USART_mega328.h
 *
 * Created: 03.02.2015 0:35:54
 *  Author: Disgust
 */ 


#ifndef USART_MEGA328_H_
#define USART_MEGA328_H_

#include <stdint.h>
#include <avr/io.h>
#include "../Macro.h"

//#define BAUD_DOUBLE

#ifndef BAUD_DOUBLE
	#define BAUD_DIVIDER(X) ((F_CPU/(X)/16)-1) // x - baudrate
#else
	#define BAUD_DIVIDER(X) ((F_CPU/(X)/8)-1) // x - baudrate
#endif



void prepareUSART(uint16_t);

void sendChar(uint8_t);
void sendBuff(uint8_t *, uint8_t);
volatile void sendNext();
uint8_t receiveChar();

#endif /* USART_MEGA328_H_ */