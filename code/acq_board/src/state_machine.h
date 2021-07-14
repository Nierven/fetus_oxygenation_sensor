#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "peripherals.h"

#define STOPPED_PERIOD   1000
#define INIT_DELAY       10000
#define RED_ON_TIME      200
#define RED_TO_IR_TIME   50
#define IR_ON_TIME       200
#define ADC_DELAY        100
#define PROCESS_DELAY    50

void stateMachineLoop(void);
void processCommandByte(uint8_t b);

#endif // STATE_MACHINE_H
