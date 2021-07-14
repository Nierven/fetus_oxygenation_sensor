#include "circular_buffer.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

void CB_Init(CircularBuffer *buffer)
{
    memset(buffer->Buffer, 0, CB_BUFFER_SIZE);
    buffer->writeIndex = 0;
}

void CB_Add(CircularBuffer *buffer, int16_t value) {
    buffer->Buffer[buffer->writeIndex] = value;

    if (buffer->writeIndex < CB_BUFFER_SIZE - 1)
        buffer->writeIndex++;
    else
        buffer->writeIndex = 0;
}

int16_t CB_Get(CircularBuffer *buffer, int index) {
    int newIndex = buffer->writeIndex - index;
    if (newIndex >= 0)
        return buffer->Buffer[newIndex];
    else
        return buffer->Buffer[CB_BUFFER_SIZE + newIndex];
}
