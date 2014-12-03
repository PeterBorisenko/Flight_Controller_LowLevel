/*
 * ATmega328P_copter_HARD.c
 *
 * Created: 4/18/2014 8:05:30 PM
 *  Author: Disgust
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
//#include <tgmath.h> // test
#include <avr/delay.h>

#include "Macro.h"
#include "Proximity.h"
#include "Assign.h"

volatile static uint8_t required_vect_X= 0x00;
volatile static uint8_t required_vect_Y= 0x00;
volatile static uint8_t required_vect_Z= 0xFF;

volatile static uint8_t current_vect_X;
volatile static uint8_t current_vect_Y;
volatile static uint8_t current_vect_Z;

volatile static uint8_t IMU_DATA_READY;

void setPowerReduction() {
    PRR|= (1 << PRTIM1)|(1 << PRSPI)|(1 << PRADC);
}

void prepareTimer(uint8_t timer, uint8_t mode, uint8_t prescaler) 
{
    switch (timer)
    {
    default: case 0x00:
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

void prepareI2C() 
{
	//assert(!"The method or operation is not implemented.");
}

void prepareUSART() 
{
	//assert(!"The method or operation is not implemented.");
}

void prepareESC() 
{
    ESC_dir&= ~((1 << FL_pin)|(1 << FR_pin)|(1 << BL_pin)|(1 << BR_pin));
	ESC_dir|= (1 << FL_pin)|(1 << FR_pin)|(1 << BL_pin)|(1 << BR_pin); // ESC control pins are OUTs
}

inline volatile void setThrust(unsigned char * ESC_reg, uint8_t thrust) {
    *ESC_reg= thrust;
}

void getCurrentImuData() {
    
}

void prepareSystem() 
{
    WDTCSR|= (1 << WDE)|(1 << WDIE);
	setPowerReduction();
}

void test() 
{
	FL_reg= 0x01;
    FR_reg= 0x0F;
    BR_reg= 0x7F;
    BL_reg= 0xFF;
}

int main(void)
{
    prepareSystem();
    prepareTimer(0,0, PSC_0_8);
    prepareTimer(2,0, PSC_2_8);
    
    prepareI2C();
    prepareUSART();
    
    prepareESC();

    sei();

    while(1)
    {
        asm("wdr");
        test();
        _delay_ms(1);
    }
}

ISR(TIMER0_OVF_vect){
    ESC_port|= (1 << FL_pin)|(1 << FR_pin);
    if (BIT_read(TIFR0, OCR0A))
    {
        BIT_set(TIFR0, OCR0A);
        BIT_clear(ESC_port, FL_pin);
    } 
    else if (BIT_read(TIFR0, OCR0B))
    {
        BIT_set(TIFR0, OCR0B);
        BIT_clear(ESC_port, FR_pin);
    }
}

ISR(TIMER0_COMPA_vect){
    BIT_clear(ESC_port, FL_pin);
    if (BIT_read(TIFR0, OCR0B))
    {
        BIT_set(TIFR0, OCR0B);
        BIT_clear(ESC_port, FR_pin);
    } 
    else if (BIT_read(TIFR0, TOV0))
    {
        BIT_set(TIFR0, TOV0);
        ESC_port|= (1 << FL_pin)|(1 << FR_pin);
    }
}

ISR(TIMER0_COMPB_vect){
    BIT_clear(ESC_port, FR_pin);
    if (BIT_read(TIFR0, OCR0A))
    {
        BIT_set(TIFR0, OCR0A);
        BIT_clear(ESC_port, FL_pin);
    } 
    else if (BIT_read(TIFR0, TOV0))
    {
        BIT_set(TIFR0, TOV0);
        ESC_port|= (1 << FL_pin)|(1 << FR_pin);
    }
}

ISR(TIMER2_OVF_vect){
    ESC_port|= (1 << BL_pin)|(1 << BR_pin);
    if (BIT_read(TIFR2, OCR2A))
    {
        BIT_set(TIFR2, OCR2A);
        BIT_clear(ESC_port, BL_pin);
    }
    else if (BIT_read(TIFR2, OCR2B))
    {
        BIT_set(TIFR2, OCR2B);
        BIT_clear(ESC_port, BR_pin);
    }
}

ISR(TIMER2_COMPA_vect){
    BIT_clear(ESC_port, BL_pin);
    if (BIT_read(TIFR2, OCR2B))
    {
        BIT_set(TIFR2, OCR2B);
        BIT_clear(ESC_port, BR_pin);
    } 
    else if (BIT_read(TIFR2, TOV2))
    {
        BIT_set(TIFR2, TOV2);
        ESC_port|= (1 << BL_pin)|(1 << BR_pin);
    }

}

ISR(TIMER2_COMPB_vect){
    BIT_clear(ESC_port, BR_pin);
    if (BIT_read(TIFR2, OCR2A))
    {
        BIT_set(TIFR2, OCR2A);
        BIT_clear(ESC_port, BL_pin);
    } 
    else if (BIT_read(TIFR2, TOV2))
    {
        BIT_set(TIFR2, TOV2);
        ESC_port|= (1 << BL_pin)|(1 << BR_pin);
    }
}

ISR(TIMER1_OVF_vect){ // System TIMER
    
}

ISR(WDT_vect) {
    ESC_port&= ~((1 << BL_pin)|(1 << BR_pin)|(1 << FL_pin)|(1 << FR_pin)); // Shut down Engines If SYSTEM_FAULT
}