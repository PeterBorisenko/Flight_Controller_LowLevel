/*
 * System.h
 *
 * Created: 12/7/2014 3:19:32 AM
 *  Author: Disgust
 */ 
#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>
#include "Assign.h"

void setPowerReduction();
void prepareTimer(uint8_t, uint8_t, uint8_t);
void prepareESC();
void prepareSystem();
void prepareUSART(unsigned int);

void sendChar(uint8_t);
uint8_t receiveChar();

#endif /*SYSTEM_H*/