/*
 * ATmega328P_copter_HARD.c
 *
 * Created: 4/18/2014 8:05:30 PM
 *  Author: Disgust
 */ 

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
#include "Communication.h"
#include "Accellerometer/ADXL345.h"
#include "Gyroscope/L3G4200D.h"

volatile uint8_t receiveByteCount= DATA_WIDTH;

#define GYRO_ADDR       0x01

#define wGyro           5 // wGyro is a factor of trust Gyroscope. Test it in a range: 5...20

volatile static uint8_t FLAGS= 0x00;

#define IMU_DATA_READY  0
#define CALCULATING     1

volatile static uint8_t USART_STATE= USART_IDLE;

typedef struct
{
	// Vector instructions from control system
	vect_t X;
	vect_t Y;
	vect_t Z;	
} Received_t;


typedef struct
{
	// Stored vector instructions
	vect_t X;
	vect_t Y;
	vect_t Z;
} Required_t;


typedef struct
{
	// Accellerometer measure vars
	vect_t X;
	vect_t Y;
	vect_t Z;
} A_measured_t;

typedef struct
{
	// Accellerometer filtered vars
	vect_t X;
	vect_t Y;
	vect_t Z;
} A_filtered_t;

typedef struct
{
	// Gyroscope measure vars
	vect_t X;
	vect_t Y;
	vect_t Z;
} G_measured_t;

typedef struct
{
	// Vector normalised
	vect_t X;
	vect_t Y;
	vect_t Z;
} Norm_t;

typedef struct  
{
	// Full Calculation results (Gyro-compensated Accellerometer)
	vect_t X;
	vect_t Y;
	vect_t Z;
} Current_t;

typedef struct
{
	// Previous measured (calculated) vector values
	vect_t X;
	vect_t Y;
	vect_t Z;
} Previous_t;


typedef struct
{
	// Stored thrust
	uint8_t FL;
	uint8_t FR;
	uint8_t BL;
	uint8_t BR;
} Thrust_t;

volatile static Received_t * Received;
volatile static Required_t * Required;
volatile static A_filtered_t * A_filtered;
volatile static G_measured_t * G_measured;
volatile static Norm_t * Norm;
volatile static Current_t * Current;
volatile static Previous_t * Previous;
static A_measured_t * A_measured;
Thrust_t * Thrust;

void error( uint8_t affectedModule )
{
    //TODO: Error processing
}

void prepareAccellerometer() {
    uint8_t status= ADXL345_Init();
	if (!status) {
		//module fault or module not exist
	}
	ADXL345_SetPowerMode(0x01);
}

void prepareGyro() {
	//set ODR (turn ON device)
	L3G4200D_SetODR(L3G4200D_ODR_95Hz_BW_25);
	//set PowerMode
	L3G4200D_SetMode(L3G4200D_NORMAL);
	//set Fullscale
	L3G4200D_SetFullScale(L3G4200D_FULLSCALE_250);
	//set axis Enable
	L3G4200D_SetAxis(L3G4200D_X_ENABLE | L3G4200D_Y_ENABLE | L3G4200D_Z_ENABLE);
}



void readGyro() {
	L3G4200D_GetAngRateRaw(&G_measured->X, &G_measured->Y, &G_measured->Z);
}

void readAccellerometer() {
	
	ADXL345_GetXyz(&A_measured->X, &A_measured->Y, &A_measured->Z);
	
//     TWIstart();
//     TWIslaveWrite(ACCEL_ADDR);
//     TWIbyteWrite(A_DATAXH);
//     TWIstart();
//     TWIslaveRead(ACCEL_ADDR);
//     uint8_t temp= TWIbyteRead();
//     A_measured_vect_X= (temp << 8)|(TWIbyteRead());
//     temp= TWIbyteRead();
//     A_measured_vect_Y= (temp << 8)|(TWIbyteRead());
//     temp= TWIbyteRead();
//     A_measured_vect_Z= (temp << 8)|(TWIbyteRead());
//     TWIstop();
}

void getCurrentImuData() {
	
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
        Previous->X= A_filtered->X;
        Previous->Y= A_filtered->Y;
        Previous->Z= A_filtered->Z;

        readAccellerometer();
        A_filtered->X= filtr(A_measured->X, Previous->X);
        A_filtered->Y= filtr(A_measured->Y, Previous->Y);
        A_filtered->Z= filtr(A_measured->Z, Previous->Z);

        BIT_set(FLAGS, IMU_DATA_READY);
}

void calculate() 
{
    while(!BIT_read(FLAGS, IMU_DATA_READY));
    BIT_set(FLAGS, CALCULATING);
    vect_t scalar= hypo3(A_filtered->X, A_filtered->Y, A_filtered->Z);
    Norm->X= A_filtered->X/scalar;
    Norm->Y= A_filtered->Y/scalar;
    Norm->Z= A_filtered->Z/scalar;
    BIT_clear(FLAGS, CALCULATING);
}

static inline void setThrust(uint8_t * ESC_reg, uint8_t thrust) {
    *ESC_reg= thrust;
}

void thrustOut() {
	Thrust->BL= 0;
	Thrust->BR= 0;
	Thrust->FL= 0;
	Thrust->FR= 0;
}

void makeDecision() {
    if (Norm->X > Required->X) {
        setThrust(&FL_reg, CONSTRAIN(++(Thrust->FL), 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++Thrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--Thrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--Thrust->BL, 0, 255));
    }
    else if (Norm->X < Required->X) {
        setThrust(&FL_reg, CONSTRAIN(--Thrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--Thrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++Thrust->BR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++Thrust->BL, 0, 255));
    }
    if (Norm->Y > Required->Y) {
        setThrust(&FL_reg, CONSTRAIN(--Thrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++Thrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++Thrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--Thrust->BL, 0, 255));
    }
    else if (Norm->Y < Required->Y) {
        setThrust(&FL_reg, CONSTRAIN(++Thrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--Thrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--Thrust->BR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++Thrust->BL, 0, 255));
    }
    if (Norm->Z > Required->Z) {
        setThrust(&FL_reg, CONSTRAIN(--Thrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(--Thrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(--Thrust->BR, 0, 255));
        setThrust(&BL_reg, CONSTRAIN(--Thrust->BL, 0, 255));
    }
    else if (Norm->Z < Required->Z) {
        setThrust(&FL_reg, CONSTRAIN(++Thrust->FL, 0, 255));
        setThrust(&FR_reg, CONSTRAIN(++Thrust->FR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++Thrust->BR, 0, 255));
        setThrust(&BR_reg, CONSTRAIN(++Thrust->BL, 0, 255));
    }
}

int main(void)
{
    prepareSystem();
    prepareTimer(0,0, PSC_0_64);
    prepareTimer(2,0, PSC_2_64);
    
    prepareUSART(BAUD_DIVIDER(BAUD));
    
    prepareESC();
	prepareAccellerometer();
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
            Received->X= (Received->X << 8)|receiveChar();
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
            Received->Y= (Received->Y << 8)|receiveChar();
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
            Received->Z= (Received->Z << 8)|receiveChar();
            receiveByteCount--;
        }
        if (receiveByteCount == 0) {
            receiveByteCount= DATA_WIDTH;
            Required->X= Received->X;
            Required->Y= Received->Y;
            Required->Z= Received->Z;
            sendChar(ACK);
            USART_STATE= USART_IDLE;
        }
    	break;
    }
	
	
}

ISR(WDT_vect) {
    ESC_port&= ~((1 << BL_pin)|(1 << BR_pin)|(1 << FL_pin)|(1 << FR_pin)); // Shut down Engines If SYSTEM_FAULT =)
}