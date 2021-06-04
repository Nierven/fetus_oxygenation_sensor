#ifndef FILTER_H
#define FILTER_H

#include "peripherals.h"

void FIRInit(void);
int16_t FIRRedOutput(int16_t x);
int16_t FIRIROutput(int16_t x);

#endif // FILTER_H
