/*
 * Platform.h
 *
 * Created: 08.02.2015 21:39:28
 *  Author: Peter Borisenko : AWSMTEK.COM
 */ 


#ifndef PLATFORM_H_
#define PLATFORM_H_

// Platform dependent
#ifdef __XC8

#elif defined _AVR_IO_H_
	#ifdef _AVR_IOM328P_H_
		#include "ADC_mega328.h"
		//#include "PCINT_mega328.h"
		//#include "Timer0_mega328.h"
		//#include "Timer1_mega328.h"
		#include "Communication_mega328.h"	
	#endif
	
#endif



#endif /* PLATFORM_H_ */