#include "ADC_mega328.h"


void adcInit(uint8_t adcPrescaler, uint8_t reference, uint8_t leftAlign, uint8_t sleepMode)
{
	ADCSRA= 0; // Used for proper reinit being possible
	if (adcPrescaler <= 0x07)
	{
		ADCSRA|= (adcPrescaler << ADPS0);
	}
	if (reference <= 0x03)
	{
		ADMUX|= (reference << REFS0);
	}
	if (leftAlign)
	{
		BIT_set(ADMUX, ADLAR);
	}
	BIT_set(ADCSRA, ADEN); // Enable adc unit
	BIT_set(ADCSRA, ADIE); // ADC conversion complete interrupt enable
	if (sleepMode)
	{
		SMCR|= (0b001 << SM0);
	} 
	else
	{
		SMCR&= ~(0b001 << SM0);
	}
}

void adcDigInDisable(uint8_t didb) {
	DIDR0|= (1 << didb);
}

void adcStart(volatile adc_t * dat)
{
	dat->state= 0;
	BIT_set(ADCSRA, ADSC); // Start conversion
}

void adcStartAndSleep(adc_t * dat)
 {
	 BIT_set(SMCR, SE);
	 adcStart(dat);
	 asm("sleep");
 }

void adcGetDataAndWakeUp(adc_t * result)
{
	BIT_clear(SMCR, SE);
	adcGetData(result);
}

void adcSelectChannel(uint8_t adc_msk)
{
	ADMUX&= 0xF0; // Clear previous channel settings
	ADMUX&= adc_msk;
}

void adcGetData(volatile adc_t * result)
{
	if(BIT_read(ADMUX, ADLAR)) {
		result->value= (ADC >> 6);
	}
	else {
		result->value= ADC;
	}
	result->state= 1;
}

uint8_t adcRead()
{
	return ADC;
}
