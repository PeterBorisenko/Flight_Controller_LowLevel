/*
 * USART_mega328.c
 *
 * Created: 03.02.2015 0:58:39
 *  Author: Disgust
 */ 

#include "USART_mega328.h"

 void prepareUSART(uint16_t ubrr)
 {
	 //      DDRD|= (1 << PIND1);
	 //      DDRD&= ~(1 << PIND0);
	 UBRR0H= (uint8_t) (ubrr >> 8);
	 UBRR0L= (uint8_t) ubrr;
	 #ifdef BAUD_DOUBLE
	 BIT_set(UCSR0A, U2X0);
	 #endif
	 UCSR0C= (1 << USBS0)|(1 << UCSZ00)|(1 << UPM01); // Frame format: 8 data, 2 stop, parity even
	 UCSR0B= (1 << RXCIE0)|(1 << TXCIE0)|(1 << RXEN0)|(1 << TXEN0); // Enable RX, TX, TX and RX interrupts
 }
 
 void sendChar(uint8_t byteToSend)
 {
	 while (!(UCSR0A & (1 << UDRE0)));
	 UDR0= byteToSend;
 }

 uint8_t receiveChar()
 {
	 return UDR0;
 }