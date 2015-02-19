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
#include "ADC_mega328.h"
#include "System.h"

volatile uint8_t receiveByteCount= DATA_WIDTH;

#define wGyro           5 // wGyro is a factor of trust Gyroscope. Test it in a range: 5...20

volatile static uint8_t FLAGS= 0x00;

#define IMU_DATA_READY  0
#define CALCULATING     1

volatile static uint8_t USART_STATE= USART_IDLE;

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
volatile static int8_t Rotation; // Rotation: x < 0 - CCW, x > 0 - CW, x == 0 - No rotation;
volatile static vect_t Received;
volatile static vect_t Required;
volatile static vect_t A_filtered;
volatile static vect_t G_measured;
volatile static vect_t Norm;
volatile static vect_t Current;
volatile static vect_t Previous;
volatile static vect_t A_measured;
Thrust_t Thrust;
adc_t CS_measured;

// Pointers
volatile static vect_t * pReceived= &Received;	// Vector instructions from control system
volatile static vect_t * pRequired= &Required;	// Stored vector instructions
volatile static vect_t * pA_filtered= &A_filtered;// Accelerometer filtered vars
volatile static vect_t * pG_measured= &G_measured;// Gyroscope measure vars
volatile static vect_t * pNorm= &Norm;		// Full Calculation results (Gyro-compensated Accelerometer) 
volatile static vect_t * pCurrent= &Current;	// Full Calculation results (Gyro-compensated Accelerometer)
volatile static vect_t * pPrevious= &Previous;	// Previous measured (calculated) vector values
volatile static vect_t * pA_measured= &A_measured;	// Accelerometer measure vars
Thrust_t * pThrust= &Thrust;
volatile static adc_t * pCS_measured= &CS_measured;

volatile static uint16_t CSfiltered; // Needed to prevent false reaction and improve noise immunity
uint32_t CSFilterBuffer;
uint8_t	CSFilterIndex= CS_FILTER_COUNT;


int16_t filtr(int16_t curr_val, int16_t prev_val, uint8_t mod) {
	return ((curr_val + prev_val * mod)/(1 + mod));
}

int16_t hypo3(int16_t a, int16_t b, int16_t c )
{
	return sqrt(a*a + b*b + c*c);
}

void error( uint8_t affectedModule )
{
    //TODO: Error processing
}

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
        pPrevious->X= pA_filtered->X;
        pPrevious->Y= pA_filtered->Y;
        pPrevious->Z= pA_filtered->Z;

        readAccelerometer();
        pA_filtered->X= filtr(pA_measured->X, pPrevious->X, wGyro);
        pA_filtered->Y= filtr(pA_measured->Y, pPrevious->Y, wGyro);
        pA_filtered->Z= filtr(pA_measured->Z, pPrevious->Z, wGyro);

        BIT_set(FLAGS, IMU_DATA_READY);
}

void calculate()  //TODO: vector calcs must be in float
{
    while(!BIT_read(FLAGS, IMU_DATA_READY));
    BIT_set(FLAGS, CALCULATING);
    int16_t scalar= hypo3(pA_filtered->X, pA_filtered->Y, pA_filtered->Z); 
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
		setThrust(&FL_reg, CONSTRAIN((pThrust->FL + pThrust->Rot - Rotation), 0, 255));
		setThrust(&FR_reg, CONSTRAIN((pThrust->FR - pThrust->Rot + Rotation), 0, 255));
		setThrust(&BR_reg, CONSTRAIN((pThrust->BR - pThrust->Rot + Rotation), 0, 255));
		setThrust(&BL_reg, CONSTRAIN((pThrust->BL + pThrust->Rot - Rotation), 0, 255));
		pThrust->Rot= Rotation;
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

int main(void)
{
    prepareSystem();
    prepareTimer(0,0, PSC_0_64);
    prepareTimer(2,0, PSC_2_64);
    
    prepareUSART(BAUD_DIVIDER(BAUD));
    sei();

    while(1)
    {
        asm("wdr");
        measure();
        asm("wdr");
        calculate();
        asm("wdr");
        makeDecision();
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
    switch (USART_STATE)
    {
    case USART_IDLE:
        if (((HEADER >> 8)&0xFF) == receiveChar())
        {
            USART_STATE= USART_REQ;
        }
        else {
            sendChar(NACK);
        }
    	break;
    case USART_REQ:
        if ((unsigned char)HEADER == receiveChar())
        {
            USART_STATE= HEADER_OK;
        }
        else {
            sendChar(NACK);
        }
    	break;
    case HEADER_OK:
        USART_STATE= RECEIVE_X;
    case RECEIVE_X:
        if (receiveByteCount > 0) {
            pReceived->X= (pReceived->X << 8)|receiveChar();
            receiveByteCount--;
        }
        if (receiveByteCount == 0) {
            receiveByteCount= DATA_WIDTH;
			sendChar(ACK);
            USART_STATE= RECEIVE_Y;
        }
    	break;
    case RECEIVE_Y:
        if (receiveByteCount > 0) {
            pReceived->Y= (pReceived->Y << 8)|receiveChar();
            receiveByteCount--;
        }
        if (receiveByteCount == 0) {
            receiveByteCount= DATA_WIDTH;
			sendChar(ACK);
            USART_STATE= RECEIVE_Z;
        }
    	break;
    case RECEIVE_Z:
        if (receiveByteCount > 0) {
            pReceived->Z= (pReceived->Z << 8)|receiveChar();
            receiveByteCount--;
        }
        if (receiveByteCount == 0) {
            receiveByteCount= DATA_WIDTH;
            pRequired->X= pReceived->X;
            pRequired->Y= pReceived->Y;
            pRequired->Z= pReceived->Z;
            sendChar(ACK);
            USART_STATE= RECEIVE_ROT;
        }
    	break;
	case RECEIVE_ROT:
		Rotation= (int8_t)receiveChar();
		USART_STATE= USART_IDLE;
	break;
    }
	
	
}

ISR(WDT_vect) {
    ESC_port&= ~((1 << BL_pin)|(1 << BR_pin)|(1 << FL_pin)|(1 << FR_pin)); // Shut down Engines If SYSTEM_FAULT =)
}

ISR(ADC_vect) {
	adcGetData(pCS_measured);
}