/*
 * ATmega328P_copter_HARD.c
 *
 * Created: 4/18/2014 8:05:30 PM
 *  Author: Disgust
 */ 


#include <avr/io.h>

#include "Macro.h"
#include "Proximity.h"
#include "Assign.h"

void prepareTimer(unsigned char timer, unsigned char mode) 
{
	assert(!"The method or operation is not implemented.");
}

void prepareI2C() 
{
	assert(!"The method or operation is not implemented.");
}

void prepareUSART() 
{
	assert(!"The method or operation is not implemented.");
}

void prepareESC() 
{
	assert(!"The method or operation is not implemented.");
}

void setThrust(unsigned char, unsigned char) {
    
}


int main(void)
{
    prepareTimer(0,2);
    prepareTimer(2,2);
    
    prepareI2C();
    prepareUSART();
    
    prepareESC();

    sei();
    
    while(1)
    {
        //TODO:: Please write your application code 
    }
}

ISR (TIMER0_OVF_vect){
    ESC_port|= (1 << FL_pin)|(1 << FR_pin);
//     BIT_set(ESC_port, FL_pin);
//     BIT_set(ESC_port, FR_pin);
}

ISR (TIMER0_COMPA_vect) {
    BIT_clear(ESC_port, FL_pin);
}

ISR (TIMER0_COMPB_vect) {
    BIT_clear(ESC_port, FR_pin);
}

ISR (TIMER2_OVF_vect){
    ESC_port|= (1 << BL_pin)|(1 << BR_pin);
//     BIT_set(ESC_port, BL_pin);
//     BIT_set(ESC_port, BR_pin);
}

ISR (TIMER2_COMPA_vect) {
    BIT_clear(ESC_port, BL_pin);
}

ISR (TIMER2_COMPB_vect) {
    BIT_clear(ESC_port, BR_pin);
}