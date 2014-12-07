#pragma once

 #define WITHOUT_CHECKS 1

// TWI States
#define BUS_ERR         0x00
#define START           0x08
#define REPEAT_START    0x10        
#define MT_SLA_ACK      0x18
#define MT_SLA_NACK     0x20
#define MT_DATA_ACK     0x28
#define MT_DATA_NACK    0x30
#define ARB_LOST        0x38
#define MR_SLA_ACK      0x40
#define MR_SLA_NACK     0x48
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define IDLE            0xF8

#define READ            0x01
#define WRITE           0x00

// Functions

inline void slaveReadTWI(uint8_t);

inline void byteWriteTWI(uint8_t);

inline uint8_t byteReadTWI();

inline void slaveWriteTWI(uint8_t);

inline void stopTWI();

inline void startTWI();

void readTWI(uint8_t, uint8_t *, uint8_t);

void writeTWI(uint8_t, uint8_t *, uint8_t);