#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "peripherals.h"

#define STOPPED_PERIOD   1000
#define INIT_DELAY       10000
#define RED_ON_TIME      220
#define RED_TO_IR_TIME   320
#define IR_ON_TIME       220
#define ADC_DELAY        110
#define TOTAL_PERIOD     2000
#define PROCESS_DELAY    50
#define PROCESS_TIME     100      // TOTAL_PERIOD - IR_ON_TIME - RED_TO_IR_TIME - RED_ON_TIME - PROCESS_DELAY

void stateMachineLoop(void);
void processCommandByte(uint8_t b);

#endif // STATE_MACHINE_H
