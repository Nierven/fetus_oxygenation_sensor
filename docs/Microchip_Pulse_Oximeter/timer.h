/* 
 * File:   timer.h
 * Author: Brandon Mitchell
 *
 * Created on March 6, 2015, 3:38 PM
 *
 * 10 ms resolution timer
 */

#ifndef TIMER_H
#define	TIMER_H

#ifdef	__cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS

// Modify this for the timer used in your application
#define TIMER_ONE_INIT_COUNT 65226 // 10 ms - Timer 1 interrupt will occur when TMR1 overflows, at approx ( 2^16 - 65226 ) / 31000 Hz = 10 ms

////////////////////////////////////////////////////////////////////////////////
// MACROS
    
// Modify these for the MCU in your application
//#define	DisableInterrupts()	(GIE = 0)
//#define	EnableInterrupts()	(GIE = 1)


enum {
    systemTick_ms = 10
};

////////////////////////////////////////////////////////////////////////////////
// Public Data

extern unsigned long systemTime_ms;

////////////////////////////////////////////////////////////////////////////////
// TYPES

typedef struct timer_define {
    volatile unsigned long startTime_ms;
    volatile unsigned long delayTime_ms;
} timer_t;

////////////////////////////////////////////////////////////////////////////////
// Interface Function Declarations

void          InitTimer( void );
void          TimerDelay_ms( const unsigned long delayTime_ms );
void          TimerStart( timer_t * timer, const unsigned long delayTime_ms );
unsigned char TimerExpired( const timer_t * timer );


#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_H */

