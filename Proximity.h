/*
 * Proximity.h
 *
 * Created: 4/19/2014 12:24:07 AM
 *  Author: Disgust
 */ 


#ifndef PROXIMITY_H_
#define PROXIMITY_H_

#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t conversion( uint8_t ); /*ADC 8-bit, Vref 3.3V */
uint8_t LinearAPPROX( uint8_t );

#endif /* PROXIMITY_H_ */