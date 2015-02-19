/*
 * ACS758.h
 *
 * Created: 03.02.2015 1:30:28
 *  Author: Disgust
 */ 


#ifndef ACS758_H_
#define ACS758_H_

#include <stdint.h>
#include "../Macro.h"
#include "../ADC_mega328.h"

#define ADC_RES	1024
#define CURRENT_ADC_OFFSET(x) ((x - 510)) // Calculates pure ADC value
#define CURRENT_ADC_TO_CURRENT(y) ((y*0.122)-0.04) // Calculates current value. Y is pure ADC value

void startCurrentMeasure(uint8_t, volatile adc_t *);

#endif /* ACS758_H_ */