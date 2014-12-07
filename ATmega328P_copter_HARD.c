/*
 * ATmega328P_copter_HARD.c
 *
 * Created: 4/18/2014 8:05:30 PM
 *  Author: Disgust
 */ 
 #define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <avr/delay.h>


#include "Macro.h"
#include "Proximity.h"
#include "Assign.h"
#include "System.h"
#include "TWI.h"

#define ACCEL_ADDR      0x00
#define A_DATAXH        0x32
#define A_DATAXL        0x33
#define A_DATAYH        0x34
#define A_DATAYL        0x35
#define A_DATAZH        0x36
#define A_DATAZL        0x37

#define GYRO_ADDR       0x01

#define wGyro           5 // wGyro is a factor of trust Gyroscope. Test it in range: 5...20

volatile static uint8_t FLAGS= 0x00;

#define IMU_DATA_READY  0
#define CALCULATING     1
#define FIRST_CALC      2

// Vector instructions from control system
volatile static vect_t required_vect_X= 0x00;
volatile static vect_t required_vect_Y= 0x00;
volatile static vect_t required_vect_Z= 0x7F;

// Accellerometer measure vars
volatile static vect_t A_measured_vect_X;
volatile static vect_t A_measured_vect_Y;
volatile static vect_t A_measured_vect_Z;

// Gyroscope measure vars
volatile static vect_t G_measured_vect_X;
volatile static vect_t G_measured_vect_Y;
volatile static vect_t G_measured_vect_Z; // Is this can be measured?

// Vector normalie
volatile static vect_t norm_vect_X;
volatile static vect_t norm_vect_Y;
volatile static vect_t norm_vect_Z;

// Full Calculation results (Gyro-compensated Accellerometer)
volatile static vect_t current_vect_X;
volatile static vect_t current_vect_Y;
volatile static vect_t current_vect_Z;

// Previous measured (calculated) vector values
volatile static vect_t prev_vect_X;
volatile static vect_t prev_vect_Y;
volatile static vect_t prev_vect_Z;


// Store thrust
uint8_t FL= 0;
uint8_t FR= 0;
uint8_t BL= 0;
uint8_t BR= 0;

void error( uint8_t affectedModule )
{
    //assert(!"The method or operation is not implemented.");
}

void getCurrentImuData() {
    
}

void prepareAccellerometer() {
    
}

void prepareTWI() {
    DDRC&= ~((1 << PINC4)|(1 << PINC5));    // Configure TWI Pins as inputs
    PORTC|= (1 << PINC4)|(1 << PINC5);      // Connect internal PULL-UPs
}

void readAccellerometer() {
    startTWI();
    slaveWriteTWI(ACCEL_ADDR);
    byteWriteTWI(A_DATAXH);
    startTWI();
    slaveReadTWI(ACCEL_ADDR);
    uint8_t temp= byteReadTWI();
    A_measured_vect_X= (temp << 8)|(byteReadTWI());
    temp= byteReadTWI();
    A_measured_vect_Y= (temp << 8)|(byteReadTWI());
    temp= byteReadTWI();
    A_measured_vect_Z= (temp << 8)|(byteReadTWI());
    stopTWI();
}

void prepareGyro() {

}

volatile vect_t filtr(vect_t curr_val, vect_t prev_val) {
    return ((curr_val + prev_val * wGyro)/(1 + wGyro));
}

volatile vect_t hypo3( volatile vect_t a, volatile vect_t b, volatile vect_t c ) 
{
	return sqrt(a*a + b*b + c*c);
}

void measure() {
        while(BIT_read(FLAGS, CALCULATING));
        BIT_clear(FLAGS, IMU_DATA_READY);
        // Prepare Recent Cycle Data
        prev_vect_X= A_measured_vect_X;
        prev_vect_Y= A_measured_vect_Y;
        prev_vect_Z= A_measured_vect_Z;

        readAccellerometer();
        A_measured_vect_X= filtr(A_measured_vect_X, prev_vect_X);
        A_measured_vect_Y= filtr(A_measured_vect_Y, prev_vect_Y);
        A_measured_vect_Z= filtr(A_measured_vect_Z, prev_vect_Z);

        BIT_set(FLAGS, IMU_DATA_READY);
}

void calculate() 
{
    while(!BIT_read(FLAGS, IMU_DATA_READY));
    BIT_set(FLAGS, CALCULATING);
    vect_t scalar= hypo3(A_measured_vect_X, A_measured_vect_Y, A_measured_vect_Z);
    norm_vect_X= A_measured_vect_X/scalar;
    norm_vect_Y= A_measured_vect_Y/scalar;
    norm_vect_Z= A_measured_vect_Z/scalar;
    BIT_clear(FLAGS, CALCULATING);
}

volatile static inline void setThrust(unsigned char * ESC_reg, uint8_t thrust) {
    *ESC_reg= thrust;
}

void makeDecision() {
    if (norm_vect_X>required_vect_X) {
        setThrust(&FL_reg, CONSTRAIN(++FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--BL, 0, 255));
    }
    else if (norm_vect_X<required_vect_X) {
        setThrust(&FL_reg, CONSTRAIN(--FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++BR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++BL, 0, 255));
    }
    if (norm_vect_Y>required_vect_Y) {
        setThrust(&FL_reg, CONSTRAIN(--FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--BL, 0, 255));
    }
    else if (norm_vect_Y<required_vect_Y) {
        setThrust(&FL_reg, CONSTRAIN(++FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--BR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++BL, 0, 255));
    }
    if (norm_vect_Z>required_vect_Z) {
        setThrust(&FL_reg, CONSTRAIN(--FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--BL, 0, 255));
    }
    else if (norm_vect_Z<required_vect_Z) {
        setThrust(&FL_reg, CONSTRAIN(++FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++BR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++BL, 0, 255));
    }
}

int main(void)
{
    prepareSystem();
    prepareTimer(0,0, PSC_0_64);
    prepareTimer(2,0, PSC_2_64);
    
    prepareUSART();
    
    prepareESC();

    sei();

    while(1)
    {
        asm("wdr");
        measure();
        calculate();
        makeDecision();
    }
}

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

ISR(WDT_vect) {
    ESC_port&= ~((1 << BL_pin)|(1 << BR_pin)|(1 << FL_pin)|(1 << FR_pin)); // Shut down Engines If SYSTEM_FAULT
}