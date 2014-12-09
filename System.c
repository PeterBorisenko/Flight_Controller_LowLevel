/*
 * System.c
 *
 * Created: 12/7/2014 3:22:20 AM
 *  Author: Disgust
 */ 

#include "System.h"

void prepareSystem()
{
    WDTCSR|= (1 << WDE)|(1 << WDIE);
    //WDTCSR|=(0b111 << WDP0);
    setPowerReduction();
}

 void setPowerReduction() {
     PRR|= (1 << PRTIM1)|(1 << PRSPI)|(1 << PRADC);
 }

 void prepareTimer(uint8_t tmr, uint8_t mode, uint8_t prescaler)
 {
     switch (tmr)
     {
         case 0x00:
         TCCR0A= 0x00;
         TCCR0B= (0x00 | prescaler);
         OCR0A= 0x00;
         OCR0B= 0x00;
         TIMSK0= 0x07; //Enable all three timer interrupts
         TIFR0= 0x07; // Reset timer interrupts
         break;

         case 0x01:
         break;

         case 0x02:
         TCCR2A= 0x00;
         TCCR2B= (0x00 | prescaler);
         OCR2A= 0x00;
         OCR2B= 0x00;
         TIMSK2= 0x07;
         TIFR2= 0x07; // Reset timer interrupts
         break;
     }
 }

 void prepareUSART(unsigned int ubrr)
 {
     UBRR0H= (unsigned char) (ubrr >> 8);
     UBRR0L= (unsigned char) ubrr;
     UCSR0B= (1 << RXCIE0)|(1 << TXCIE0)|(1 << RXEN0)|(1 << TXEN0);
     UCSR0C= (1 << USBS0)|(1 << UCSZ00); // Frame format: 8 data, 2 stop
 }

 void prepareESC()
 {
     ESC_dir&= ~((1 << FL_pin)|(1 << FR_pin)|(1 << BL_pin)|(1 << BR_pin));
     ESC_dir|= (1 << FL_pin)|(1 << FR_pin)|(1 << BL_pin)|(1 << BR_pin); // ESC control pins are OUTs
 }
 
 void sendChar( uint8_t byteToSend)
 {
 	while (!(UCSR0A & (1 << UDRE0)))
 	{
        UDR0= byteToSend;
 	}
 }

uint8_t receiveChar()
{
	return UDR0;
}

 