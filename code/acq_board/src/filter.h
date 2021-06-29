#ifndef FILTER_H
#define FILTER_H

#include "peripherals.h"

void FiltersInit(void);
void updateValues(int16_t red, int16_t ir);

int16_t FIRRedOutput(void);
int16_t FIRIROutput(void);
int16_t movingRedAvg(void);
int16_t movingIRAvg(void);

#endif // FILTER_H
