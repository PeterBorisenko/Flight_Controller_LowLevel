/*
 * Assign.h
 *
 * Created: 11/26/2014 1:10:41 AM
 *  Author: Disgust
 */ 


#ifndef ASSIGN_H_
#define ASSIGN_H_

#include <avr/io.h>

#define F_CPU 16000000UL


// Data format
typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
} vect_t;

typedef struct {
	float X;
	float Y;
	float Z;
} vect_float_t;

// Timer Prescaler
#define TMR_OFF         0b000
#define PSC_1           0b001
#define PSC_0_8         0b010
#define PSC_0_64        0b011
#define PSC_0_256       0b100
#define PSC_0_1024      0b101
#define EXT_0_FALLING   0b110
#define EXT_0_RISING    0b111
#define PSC_2_8         0b010
#define PSC_2_32        0b011
#define PSC_2_64        0b100
#define PSC_2_128       0b101
#define PSC_2_256       0b110
#define PSC_2_1024      0b111

// Motor (ESC) Control Pins/Ports
    // Needed Software PWM !
#define ESC_port PORTD
#define ESC_dir DDRD
#define FL_pin PIND3
#define FR_pin PIND4
#define BL_pin PIND5
#define BR_pin PIND6
#define FL_reg OCR0A
#define FR_reg OCR0B
#define BL_reg OCR2A
#define BR_reg OCR2B

// Current Sensor Pins/Ports
#define CS_port			PORTC
#define CS_dir			DDRC
#define CS_pin			PINC0
#define CS_ADCmask		0xFF
#define CS_DID			ADC0D
#define CS_FILTER_COUNT	0x05;

// General
volatile static uint8_t FLAGS= 0x00;
//Flag bits
#define IMU_DATA_READY  0
#define CALCULATING     1
//#define ON_GO			2

#endif /* ASSIGN_H_ */