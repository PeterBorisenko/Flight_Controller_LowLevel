/*
 * Protocol.h
 *
 * Created: 20.02.2015 20:47:50
 *  Author: Disgust
 */ 


#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <string.h>

#include "Macro.h"
#include "Platform.h"
#include "Assign.h"

#define BAUD 1200
#define DATA_WIDTH 4 // bytes, because we sending float

// USART States
#define USART_IDLE  0x00
#define USART_REQ   0x01
#define HEADER_OK   0x02
#define RECEIVE_X   0x03
#define RECEIVE_Y   0x04
#define RECEIVE_Z   0x05
#define RECEIVE_ROT	0x06
#define RECEIVE_RSP	0x07
#define USART_ASKED	0x10
#define USART_ASKS	0x11
#define SEND_STAT	0x12

// Message parts
#define HEADER      0x1010
#define ASK_STATUS	0x75
#define SET_ONGO	0xA6
#define UNSET_ONGO	0xB7
#define SET_REINIT	0xCA
#define DIAG		0xDB

// Responses
#define ACK         0x20
#define NACK        0x31
#define WAIT		0x42
#define ARMED		0x53
#define FAULT		0x64
#define ON_GO		0xEC
#define LANDING		0xFD


// Globals
//
#define XYZDataLength 12

typedef union {
	vect_float_t XYZ;
	uint8_t byteReceived[XYZDataLength];
} ResInstruction_t;

volatile static uint8_t USART_STATE= USART_IDLE;
volatile static uint8_t DEVICE_STATUS= WAIT;
volatile static uint8_t ERROR_CODE;

volatile static vect_float_t * pRequired;	// Stored vector instructions
volatile static int8_t * pRotation;

// Methods
void protoRxHandler();

#endif /* PROTOCOL_H_ */
