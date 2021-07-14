#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "msp.h"

typedef enum LED_t
{
    Board,
    Red,
    IR
} LED;

void initInterrupts(void);

void initGPIO(void);
void initTimers(void);
void initADC(void);
void initUART(void);

uint8_t readGPIO(LED led);
void writeGPIO(LED led, uint8_t on);

void T1_Start(void);
void T1_Stop(void);
void T1_SetDelay(uint32_t delayUs);
void sleep(void);

void UART_TransmitData(uint8_t *data, uint16_t length);

#endif // PERIPHERALS_H
