#include "ACS758.h"


void startCurrentMeasure(uint8_t chMsk, adc_t * valueContanier)
{
	adcSelectChannel(chMsk);
	adcStart(valueContanier);
}