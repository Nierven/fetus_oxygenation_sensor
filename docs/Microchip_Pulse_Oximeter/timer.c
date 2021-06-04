/*
 * File:   timer.c
 * Author: Brandon Mitchell
 *
 * Created on March 6, 2015, 3:38 PM
 *
 * 10 ms resolution timer, interrupt driven
 */

#include "timer.h"
#include <xc.h>

////////////////////////////////////////////////////////////////////////////////
// Public Data

unsigned long systemTime_ms = 0;

////////////////////////////////////////////////////////////////////////////////
// Local Function Declarations

unsigned long GetSystemTime_ms( void );

////////////////////////////////////////////////////////////////////////////////
// Interface Function Definitions

// Sets up 10 ms TIMER 1 interrupt
// Modify this for your MCU and hardware configuration
void InitTimer( void )
{
	T1CON = 0x0020;			// Stop 16-bit Timer1, 1:64 Prescale, Internal clock (Fosc/2)
	TMR1 = 0x00;			// Clear timer register
	PR1 = 3125;             // Load the period value, OCxRS <= PRx, 10ms period = (1/(Fosc/2))*1000*64*PR2 = (1/(40000000/2))*1000*64*3125
    IPC0bits.T1IP = 2;		// Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0;		// clear interrupt flag
    IEC0bits.T1IE = 1;      // Enable Timer1 interrupt
    T1CONbits.TON = 1;      // Starts Timer1
}

// returns after delayTime_ms milliseconds
void TimerDelay_ms( const unsigned long delayTime_ms ) {

    timer_t timer;
    TimerStart( & timer, delayTime_ms );
    while( !TimerExpired( & timer ) );
}

// sets a timer to expire after delayTime_ms milliseconds
void TimerStart( timer_t * timer, const unsigned long delayTime_ms ) {

    timer->startTime_ms = GetSystemTime_ms();
    timer->delayTime_ms = delayTime_ms;
}

// returns 1 if timer has expired, 0 otherwise
unsigned char TimerExpired( const timer_t * timer ) {

    unsigned char expired;
//    DisableInterrupts();
    expired = ( systemTime_ms - timer->startTime_ms ) >= timer->delayTime_ms;
//    EnableInterrupts();
    return expired;
}


////////////////////////////////////////////////////////////////////////////////
//Local Function Definitions

//returns current system time
unsigned long GetSystemTime_ms( void ) {

    unsigned long currentTime_ms;
//    di();
    currentTime_ms = systemTime_ms;
//    ei();
    return currentTime_ms;
}