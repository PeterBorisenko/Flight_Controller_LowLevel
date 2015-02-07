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
#include "Communication.h"
#include "Accelerometer/ADXL345.h"
#include "Gyroscope/L3G4200D.h"
#include "Current/ACS758.h"

void setPowerReduction();
void prepareTimer(uint8_t, uint8_t, uint8_t);
void prepareESC();
void prepareSystem();
void prepareUSART(unsigned int);


void sendChar(uint8_t);
uint8_t receiveChar();
void prepareAccelerometer();
void prepareGyro();
void prepareCurrentSens();
#endif /*SYSTEM_H*/