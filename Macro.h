/*
 * Defines.h
 *
 * Created: 4/25/2014 10:17:40 PM
 *  Author: Peter
 */ 


#ifndef MACRO_H_
#define MACRO_H_

///�������///

#define BIT_set(x,y) (x|=(1<<y))
#define BIT_clear(x,y) (x&=~(1<<y))
#define BIT_read(x,y) (((x)>>(y))&0x01)
#define BIT_tgl(x,y) (x^=(1<<y))
#define BIT_write(x,y,z) ((z)?(BIT_set(x,y)):(BIT_clear(x,y)))

#define HI(x) (x>>8)
#define LO(x) (x^0xFF)
#define BIT(x)	(x&0x01)

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#define CONSTRAIN(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define CIRCLE(amt, low, high) ((amt)<(low)?(high):((amt)>(high)?(low):(amt)))
#define ROUND(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

#define DEC_TO_ASCII(x)	(x+0x30)
#define HEX_TO_ASCII(x)	((x<0x0F)?(DEC_TO_ASCII(x)):(x+0x37))

// constants
#define PI		3.14159265359

// Platform dependent
#ifdef __XC8

#elif defined IO_H
 #define DDR(x) ((x)-1)    // address of data direction register of port x
 #define PIN(x) ((x)-2)    // address of input register of port x
#endif

#endif /* MACRO_H_ */