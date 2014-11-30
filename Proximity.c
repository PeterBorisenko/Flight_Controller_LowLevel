/*
 * Proximity.c
 *
 * Created: 4/19/2014 12:26:02 AM
 *  Author: Disgust
 */ 
#include "Proximity.h"

uint8_t conversion( uint8_t _ADC ) /*ADC 8-bit, Vref 3.3V */
{
	uint8_t Half_DIV;
    uint16_t Result;
    
    if(_ADC<15) return 255;		// Очень далеко. Или ошибка датчика.
    
    Half_DIV = _ADC-8;
    Half_DIV >>=1;			// Делим пополам сдвигом.
    
    Result = 1000+(uint16_t)Half_DIV;
    
    return (uint8_t)(Result/(_ADC-8));
}

uint8_t LinearAPPROX(uint8_t input)
{
    // Too far
    if (input<21) return 255;
    
    // Line1 X=63+255-4Y
    if(input<56)
    {
        input <<=2;         		//4*Y
        return (63+(255-input));
    }
    
    // Line2 X=150-Y
    if (input<118)
    {
        return (150-input);
    }
    
    // Line3 X=63-0.25Y
    if(input<250)
    {
        input >>=2;         		//0.25*Y
        return (63-input);
    }
    
    // Too close
    return 0;
}