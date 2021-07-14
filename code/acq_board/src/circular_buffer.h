#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "stdint.h"

#define CB_BUFFER_SIZE 500

typedef struct
{
    int16_t Buffer[CB_BUFFER_SIZE];
    int writeIndex;
} CircularBuffer;

void CB_Init(CircularBuffer *buffer);
void CB_Add(CircularBuffer *buffer, int16_t value);
int16_t CB_Get(CircularBuffer *buffer, int index);

#endif // CIRCULAR_BUFFER_H
