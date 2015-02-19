#include "ACS758.h"


void startCurrentMeasure(uint8_t chMsk, volatile adc_t * valueContanier)
{
	adcSelectChannel(chMsk);
	adcStart(valueContanier);
}