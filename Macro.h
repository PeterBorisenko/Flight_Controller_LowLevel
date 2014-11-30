/*
 * Defines.h
 *
 * Created: 4/25/2014 10:17:40 PM
 *  Author: Peter
 */ 


#ifndef DEFINES_H_
#define DEFINES_H_

///�������///

#define BIT_set(x,y) (x|=(1<<y))
#define BIT_clear(x,y) (x&=~(1<<y))
#define BIT_read(x,y) (((x)>>(y))&0x01)
#define BIT_write(x,y,z) ((z)?(BIT_set(x,y)):(BIT_clear(x,y)))

#define HI(x) (x>>8)
#define LO(x) (x^0xFF)

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MAP(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#define CONSTRAIN(x, a, b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#define CIRCLE(amt, low, high) ((amt)<(low)?(high):((amt)>(high)?(low):(amt)))
#define ROUND(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

#ifdef __XC8

#elif defined IO_H
 #define BAUD_DIVIDER(X) (( F_CPU /((X) * 16 ) ) - 1) // x - baudrate
#endif

#endif /* DEFINES_H_ */