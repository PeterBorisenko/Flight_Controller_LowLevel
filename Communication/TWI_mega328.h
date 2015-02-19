#ifndef TWI_MEGA328_H
#define TWI_MEGA328_H

#include <stdint.h>
#include <avr/io.h>
#include "../Macro.h"

// TWI States
#define TWI_BUS_ERR         0x00
#define TWI_START           0x08
#define TWI_REPEAT_START    0x10        
#define TWI_MT_SLA_ACK      0x18
#define TWI_MT_SLA_NACK     0x20
#define TWI_MT_DATA_ACK     0x28
#define TWI_MT_DATA_NACK    0x30
#define TWI_ARB_LOST        0x38
#define TWI_MR_SLA_ACK      0x40
#define TWI_MR_SLA_NACK     0x48
#define TWI_MR_DATA_ACK     0x50
#define TWI_MR_DATA_NACK    0x58
#define TWI_IDLE        0xF8

#define TWI_READ            0x01
#define TWI_WRITE           0x00

// Functions

uint8_t TWI_Init(uint32_t, uint32_t);

void TWIslaveRead(uint8_t);

void TWIbyteWrite(uint8_t);

uint8_t TWIbyteRead();

void TWIslaveWrite(uint8_t);

void TWIstop();

void TWIstart();

void TWIread(uint8_t, uint8_t *, uint8_t);

void TWIwrite(uint8_t, uint8_t *, uint8_t);

#endif /*TWI_MEGA328_H*/