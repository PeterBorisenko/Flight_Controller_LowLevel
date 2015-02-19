/*
 * System.c
 *
 * Created: 12/7/2014 3:22:20 AM
 *  Author: Disgust
 */ 

#include "System.h"

void prepareSystem()
{
    //WDTCSR|= (1 << WDE)|(1 << WDIE);
    //WDTCSR|=(0b111 << WDP0);
    setPowerReduction();
	TWI_Init(400000, F_CPU);
    prepareAccelerometer();
	prepareGyro();
	prepareESC();
}

 void setPowerReduction() {
     PRR|= (1 << PRTIM1)|(1 << PRSPI);
 }

 void prepareTimer(uint8_t tmr, uint8_t mode, uint8_t prescaler)
 {
     switch (tmr)
     {
         case 0x00:
         TCCR0A= 0x00;
         TCCR0B= (0x00 | prescaler);
         OCR0A= 0x00;
         OCR0B= 0x00;
         TIMSK0= 0x07; //Enable all three timer interrupts
         TIFR0= 0x07; // Reset timer interrupts
         break;

         case 0x01:
         break;

         case 0x02:
         TCCR2A= 0x00;
         TCCR2B= (0x00 | prescaler);
         OCR2A= 0x00;
         OCR2B= 0x00;
         TIMSK2= 0x07;
         TIFR2= 0x07; // Reset timer interrupts
         break;
     }
 }
 
void prepareESC()
{
	ESC_dir&= ~((1 << FL_pin)|(1 << FR_pin)|(1 << BL_pin)|(1 << BR_pin));
	ESC_dir|= (1 << FL_pin)|(1 << FR_pin)|(1 << BL_pin)|(1 << BR_pin); // ESC control pins are OUTs
}

void prepareAccelerometer() {
	uint8_t status= ADXL345_Init();
	if (!status) { // TODO: do such checks in every TWI device
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

void prepareCurrentSens() {
	adcInit(ADC_PSC_8, ADC_REFER_AVCC, 1, 0);
	adcDigInDisable(CS_DID);
}