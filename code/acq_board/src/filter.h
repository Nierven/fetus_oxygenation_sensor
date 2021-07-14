#ifndef FILTER_H
#define FILTER_H

#include "peripherals.h"
#include "circular_buffer.h"

#define FILTER_LENGTH 325
#define FILTER_AMP 100000
#define AVERAGE_LENGTH CB_BUFFER_SIZE / 2

typedef enum FilterType_t
{
    LP_5Hz,
    LP_15Hz,
    BP_3Hz,
    BP_5Hz,
    BP_15Hz
} FilterType;

void FiltersInit(void);
void setFilterFunction(FilterType filtertype);
void updateValues(int16_t red, int16_t ir);

int16_t FIRRedOutput(void);
int16_t FIRIROutput(void);
int16_t movingRedAvg(void);
int16_t movingIRAvg(void);
int16_t movingIRApprox(void);

extern CircularBuffer x_red;
extern CircularBuffer x_ir;
extern CircularBuffer x_ir_filtered;
extern CircularBuffer x_red_filtered;

#endif // FILTER_H
