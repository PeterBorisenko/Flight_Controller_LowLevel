/*
 * ATmega328P_copter_HARD.c
 *
 * Created: 4/18/2014 8:05:30 PM
 *  Author: Disgust
 */ 
#define WITHOUT_CHECKS

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
/*#include <avr/delay.h>*/


#include "Macro.h"
#include "Assign.h"
#include "System.h"
#include "Platform.h"
#include "protocol.h"
#include "User.h"

#define wGyro           5 // wGyro is a factor of trust Gyroscope. Test it in a range: 5...20

typedef struct
{
	// Stored thrust
	uint8_t FL;
	uint8_t FR;
	uint8_t BL;
	uint8_t BR;
	int8_t Rot;
} Thrust_t;

// Variables
int8_t Rotation; // Rotation: x < 0 - CCW, x > 0 - CW, x == 0 - No rotation;
volatile static vect_float_t Required;
volatile static vect_float_t A_filtered;
volatile static vect_float_t Norm;
volatile static vect_t A_measured;
volatile static vect_t G_measured;
volatile static vect_t Current;
volatile static vect_float_t Previous;

Thrust_t Thrust;
adc_t CS_measured;

// Pointers
volatile static vect_float_t * pA_filtered= &A_filtered;// Accelerometer filtered vars
volatile static vect_t * pG_measured= &G_measured;// Gyroscope measure vars
volatile static vect_float_t * pNorm= &Norm;		// Full Calculation results (Gyro-compensated Accelerometer) 
volatile static vect_t * pCurrent= &Current;	// Full Calculation results (Gyro-compensated Accelerometer)
volatile static vect_float_t * pPrevious= &Previous;	// Previous measured (calculated) vector values
volatile static vect_t * pA_measured= &A_measured;	// Accelerometer measure vars
Thrust_t * pThrust= &Thrust;
volatile static adc_t * pCS_measured= &CS_measured;

// Data filter
volatile static uint16_t CSfiltered; // Needed to prevent false reaction and improve noise immunity
uint32_t CSFilterBuffer;
uint8_t	CSFilterIndex= CS_FILTER_COUNT;

void readGyro() {
	L3G4200D_GetAngRateRaw(pG_measured);
}

void readAccelerometer() {
	ADXL345_GetXyz(pA_measured);
}

void measure() {
        while(BIT_read(FLAGS, CALCULATING));
        BIT_clear(FLAGS, IMU_DATA_READY);
        // Prepare Recent Cycle Data
        pPrevious->X= pA_filtered->X; // TODO: Replace wth memcpy()
        pPrevious->Y= pA_filtered->Y;
        pPrevious->Z= pA_filtered->Z;

        readAccelerometer();
        pA_filtered->X= filtr(pA_measured->X, pPrevious->X, wGyro);
        pA_filtered->Y= filtr(pA_measured->Y, pPrevious->Y, wGyro);
        pA_filtered->Z= filtr(pA_measured->Z, pPrevious->Z, wGyro);

        BIT_set(FLAGS, IMU_DATA_READY);
}

void calculate()
{
    while(!BIT_read(FLAGS, IMU_DATA_READY));
    BIT_set(FLAGS, CALCULATING);
    float scalar= hypo3(pA_filtered->X, pA_filtered->Y, pA_filtered->Z);
    pNorm->X= pA_filtered->X/scalar;
    pNorm->Y= pA_filtered->Y/scalar;
    pNorm->Z= pA_filtered->Z/scalar;
    BIT_clear(FLAGS, CALCULATING);
}

static inline void setThrust(volatile uint8_t * ESC_reg, uint8_t thrust) {
    *ESC_reg= thrust;
}

void thrustOut() {
	pThrust->BL= 0;
	pThrust->BR= 0;
	pThrust->FL= 0;
	pThrust->FR= 0;
}

void makeDecision() { //TODO: vector comps must be in float?
    if (pNorm->X > pRequired->X) {
        setThrust(&FL_reg, CONSTRAIN(++(pThrust->FL), 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++pThrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--pThrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--pThrust->BL, 0, 255));
    }
    else if (pNorm->X < pRequired->X) {
        setThrust(&FL_reg, CONSTRAIN(--pThrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--pThrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++pThrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(++pThrust->BL, 0, 255));
    }
    if (pNorm->Y > pRequired->Y) {
        setThrust(&FL_reg, CONSTRAIN(--pThrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++pThrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++pThrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--pThrust->BL, 0, 255));
    }
    else if (pNorm->Y < pRequired->Y) {
        setThrust(&FL_reg, CONSTRAIN(++pThrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--pThrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--pThrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(++pThrust->BL, 0, 255));
    }
    if (pNorm->Z > pRequired->Z) {
        setThrust(&FL_reg, CONSTRAIN(--pThrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--pThrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--pThrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--pThrust->BL, 0, 255));
    }
    else if (pNorm->Z < pRequired->Z) {
        setThrust(&FL_reg, CONSTRAIN(++pThrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++pThrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++pThrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(++pThrust->BL, 0, 255));
    }
	if(Rotation != 0) {
		setThrust(&FL_reg, CONSTRAIN((pThrust->FL + pThrust->Rot - *pRotation), 0, 255));
		setThrust(&FR_reg, CONSTRAIN((pThrust->FR - pThrust->Rot + *pRotation), 0, 255));
		setThrust(&BR_reg, CONSTRAIN((pThrust->BR - pThrust->Rot + *pRotation), 0, 255));
		setThrust(&BL_reg, CONSTRAIN((pThrust->BL + pThrust->Rot - *pRotation), 0, 255));
		pThrust->Rot= *pRotation;
	}
}

void CSMeasure() {
	startCurrentMeasure(CS_ADCmask, pCS_measured);
}

void CSFilter() {
	if (pCS_measured->state)
	{
		CSFilterBuffer+= pCS_measured->value;
	}
	if(!CSFilterIndex--) {
		CSfiltered= CSFilterBuffer/CS_FILTER_COUNT;
		CSFilterIndex= CS_FILTER_COUNT;
	}
}

void error( uint8_t affectedModule )
{
	ERROR_CODE|= affectedModule;
	DEVICE_STATUS= FAULT;
}

int main(void)
{
	pRotation= &Rotation;
	pRequired= &Required;	// Stored vector instructions
    prepareSystem();
    prepareTimer(0,0, PSC_0_64);
    prepareTimer(2,0, PSC_2_64);
    
    prepareUSART(BAUD_DIVIDER(BAUD));
    sei();

    while(1)
    {
		switch (DEVICE_STATUS)
		{
			case ON_GO:
				asm("wdr");
				measure();
				asm("wdr");
				calculate();
				asm("wdr");
				makeDecision();
			break;
		} 
    }
}

// Software PWM
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
    // TODO: Call Task Manager from here

}

// USART Message Protocol
ISR (USART_RX_vect) {
	protoRxHandler();
}

ISR(WDT_vect) {
    ESC_port&= ~((1 << BL_pin)|(1 << BR_pin)|(1 << FL_pin)|(1 << FR_pin)); // Shut down Engines If SYSTEM_FAULT =)
}

ISR(ADC_vect) {
	adcGetData(pCS_measured);
}