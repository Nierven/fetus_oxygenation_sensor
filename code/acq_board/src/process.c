#include "process.h"

#include "string.h"
#include "stdio.h"
#include "filter.h"

//#define ARDUINO

static float computeSpO2(int16_t red_dc, int16_t red_vpp, int16_t ir_dc, int16_t ir_vpp);
static int16_t computeHeartRate(void);
static uint8_t findMinMax(int16_t ir);
static int16_t min(CircularBuffer *values, int16_t start, int16_t end);
static int16_t max(CircularBuffer *values, int16_t start, int16_t end);

static int16_t red_min, red_max, red_min_2, red_max_2;
static int16_t ir_min, ir_max, ir_min_2, ir_max_2;
static int16_t min_t = 0, max_t = 0, min_t_2 = FILTER_LENGTH / 2, max_t_2 = 0;

void processData(int16_t red, int16_t ir)
{
    volatile unsigned int i;
    writeGPIO(Board, 1);

    // 1.1. Update data on history
    updateValues(red, ir);

    // 1.2. Use FIR filter to get AC values
    int16_t red_ac_filtered = FIRRedOutput();
    int16_t ir_ac_filtered = FIRIROutput();

    // 1.3. Get the mean of DC values
    int16_t red_dc = movingRedAvg();
    int16_t ir_dc = movingIRAvg();

    int16_t ir_mean = movingIRApprox();

    // 2. Send data via UART
#ifndef ARDUINO
    uint8_t values[16];
    values[0] = 0xFE;
    values[1] = 0;
    values[2] = 12;
    values[3] = red >> 8;
    values[4] = red;
    values[5] = red_ac_filtered >> 8;
    values[6] = red_ac_filtered;
    values[7] = red_dc >> 8;
    values[8] = red_dc;
    values[9] = ir >> 8;
    values[10] = ir;
    values[11] = ir_ac_filtered >> 8;
    values[12] = ir_ac_filtered;
    values[13] = ir_dc >> 8;
    values[14] = ir_dc;
    values[15] = 0;
    for (i = 0; i < 15; i++)
        values[15] ^= values[i];

    UART_TransmitData(values, 16);
#else
    uint8_t values[8];
    values[0] = 0xFE;
    values[1] = 0;
    values[2] = 4;
    values[3] = red_ac_filtered >> 8;
    values[4] = red_ac_filtered;
    values[5] = ir_ac_filtered >> 8;
    values[6] = ir_ac_filtered;
    values[7] = 0;
    for (i = 0; i < 7; i++)
        values[7] ^= values[i];

    UART_TransmitData(values, 8);
#endif

    // 3. Find min & max of AC values
    uint8_t done = findMinMax(ir_mean);
    if (done)
    {
        // 4. Compute RMS values
        int16_t red_vpp = (abs(red_max - red_min) + abs(red_max_2 - red_min_2)) / 2;
        int16_t ir_vpp = (abs(ir_max - ir_min) + abs(ir_max_2 - ir_min_2)) / 2;

        // 5. Compute SpO2 & Heartrate
        int16_t spo2 = (int16_t) computeSpO2(red_dc, red_vpp, ir_dc, ir_vpp);
        int16_t heartrate = computeHeartRate();

        // 6. Send data via UART
#ifndef ARDUINO
        uint8_t result[8];
        result[0] = 0xFE;
        result[1] = 1;
        result[2] = 4;
        result[3] = spo2 >> 8;
        result[4] = spo2;
        result[5] = heartrate >> 8;
        result[6] = heartrate;
        result[7] = 0;
        for (i = 0; i < 7; i++)
            result[7] ^= result[i];

        UART_TransmitData(result, 8);
#else
        uint8_t result[6];
        result[0] = 0xFE;
        result[1] = 1;
        result[2] = 2;
        result[3] = spo2 >> 8;
        result[4] = spo2;
        result[5] = 0;
        for (i = 0; i < 5; i++)
            result[5] ^= result[i];

        UART_TransmitData(result, 6);
#endif
    }
    writeGPIO(Board, 0);
}

static float computeSpO2(int16_t red_dc, int16_t red_vpp, int16_t ir_dc, int16_t ir_vpp)
{
    float ratio = ((float) red_vpp / (float) red_dc) / ((float) ir_vpp / (float) ir_dc);
    float spo2 = 114 - 32.5 * ratio;

#ifndef ARDUINO
    return spo2;
#else
    return ratio * 1000;
#endif
}

static int16_t computeHeartRate(void)
{
    return (int16_t) (200.0 / ((min_t + max_t) / 2.0) * 60.0); // mean / 200Hz * 60
}

#define INACTIVITY_COUNTER_THRESHOLD   500
#define Sample_Window_Interval_down    20     // For detecting max 4Hz(250ms,240bpm) pulse rate, 250ms*0.5sample/ms(@2msPeriod)=125samples, 50,25
#define Sample_Window_Interval_up      20     // For detecting max 5Hz(200ms,300bpm) pulse rate, 200ms*0.25sample/ms(@4msPeriod)=50samples, 16,20

static uint8_t findMinMax(int16_t ir)
{
    static int16_t _ir_min = 0, _ir_max = 0;

    static unsigned int Sample_Window_Counter;
    static unsigned int Inactivity_Counter = 0;

    static uint8_t Find_MaxMin_State = 0;

    min_t++;
    max_t++;
    min_t_2++;
    max_t_2++;

    switch (Find_MaxMin_State)
    {
        case 0:
            _ir_max = ir;
            Find_MaxMin_State = 1;
            break;

        case 1:
            if (ir >= _ir_max)
            {
                _ir_max = ir;
                Find_MaxMin_State = 2;
            }
            else
            {
                Find_MaxMin_State = 0;
            }
            break;

        case 2:     // Finding Max
            if (ir >= _ir_max)
            {
                _ir_max = ir;
                Sample_Window_Counter = Sample_Window_Interval_down;        // restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going down
                if (Sample_Window_Counter == 0)     //no more max peaks detected in the sampling window interval, go to detect min
                {
                    Find_MaxMin_State = 3;          //go to next state - Finding Min state

                    _ir_min = ir;

                    Inactivity_Counter = 0;
                    max_t = Sample_Window_Interval_down + 1;
                }
            }

            break;

        case 3:     // Finding min
            if (ir <= _ir_min)
            {
                _ir_min = ir;
                Sample_Window_Counter = Sample_Window_Interval_up;      // restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going up
                if (Sample_Window_Counter == 0)     //no more min peaks detected in the sampling window interval, go to detect max
                {
                    Find_MaxMin_State = 4;          //go to next state

                    _ir_max = ir;

                    Inactivity_Counter = 0;
                    min_t = Sample_Window_Interval_up + 1;
                }
            }

            break;

        case 4:     //Finding Second Max
            if (ir >= _ir_max)
            {
                _ir_max = ir;
                Sample_Window_Counter = Sample_Window_Interval_down;        //restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going down
                if (Sample_Window_Counter == 0)     //no more max peaks detected in the sampling window interval, go to detect min
                {
                    Find_MaxMin_State = 5;          //go to next state - Finding Min state

                    _ir_min = ir;

                    Inactivity_Counter = 0;
                    max_t_2 = Sample_Window_Interval_down + 1;
                }
            }

            break;

        case 5:     //Finding Second Min
            if (ir <= _ir_min)
            {
                _ir_min = ir;
                Sample_Window_Counter = Sample_Window_Interval_up;      //restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going up
                if (Sample_Window_Counter == 0)     //no more min peaks detected in the sampling window interval, go to detect max
                {
                    Find_MaxMin_State = 1;          //go to next state

                    _ir_max = ir;

                    // All times found, analyze the waves
                    ir_max = max(&x_ir_filtered, max_t, min_t_2);
                    red_max = max(&x_red_filtered, max_t, min_t_2);
                    ir_max_2 = max(&x_ir_filtered, max_t_2, min_t);
                    red_max_2 = max(&x_red_filtered, max_t_2, min_t);

                    Inactivity_Counter = 0;
                    min_t_2 = Sample_Window_Interval_up + 1;

                    ir_min = min(&x_ir_filtered, min_t, max_t);
                    red_min = min(&x_red_filtered, min_t, max_t);
                    ir_min_2 = min(&x_ir_filtered, min_t_2, max_t_2);
                    red_min_2 = min(&x_red_filtered, min_t_2, max_t_2);

                    return 1;
                }
            }
    }

    Inactivity_Counter++;
    if (Inactivity_Counter > INACTIVITY_COUNTER_THRESHOLD)
    {
        Find_MaxMin_State = 0;
        Inactivity_Counter = 0;
    }

    return 0;
}

static int16_t min(CircularBuffer *values, int16_t start, int16_t end)
{
    volatile unsigned int i;

    int16_t m = 20000;
    int index = values->writeIndex - start + 1;
    if (index < 0) index = CB_BUFFER_SIZE + index;

    for (i = 0; i < end - start; i++)
    {
        if (values->Buffer[--index] < m)
            m = values->Buffer[index];
        if (index == 0) index = CB_BUFFER_SIZE;
    }

    return m;
}

static int16_t max(CircularBuffer *values, int16_t start, int16_t end)
{
    volatile unsigned int i;

    int16_t m = 0;
    int index = values->writeIndex - start + 1;
    if (index < 0) index = CB_BUFFER_SIZE + index;

    for (i = 0; i < end - start; i++)
    {
        if (values->Buffer[--index] > m)
            m = values->Buffer[index];
        if (index == 0) index = CB_BUFFER_SIZE;
    }

    return m;
}
