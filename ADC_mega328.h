#pragma once

#include <stdint.h>
#include <avr/io.h>

#define ADC_REFER_AREF	0x00
#define ADC_REFER_AVCC	0x01
#define ADC_REFER_1V1	0x03

#define ADC_INPUT_CH_1V1	0x0E
#define ADC_INPUT_CH_GND	0x0F // This is not masks!

#define ADC_PSC_1			0x00
#define ADC_PSC_2			0x01
#define ADC_PSC_4			0x02
#define ADC_PSC_8			0x03
#define ADC_PSC_16			0x04
#define ADC_PSC_32			0x05
#define ADC_PSC_64			0x06
#define ADC_PSC_128			0x07

typedef struct {
	uint8_t state;
	uint16_t value;
} adc_t;

adcInit(uint8_t, uint8_t, uint8_t, uint8_t);
adcSelectChannel(uint8_t);
adcStart(adc_t *);
adcGetData(adc_t *);
adcStartAndSleep(adc_t *);
adcGetDataAndWakeUp(adc_t *);
adcDigInDisable(uint8_t);