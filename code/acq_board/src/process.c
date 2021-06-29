#include "process.h"

#include "string.h"
#include "stdio.h"
#include "filter.h"

static uint8_t findMinMax(int16_t red, int16_t ir);
static float computeSpO2(int16_t red_dc, int16_t red_rms, int16_t ir_dc, int16_t ir_rms);
static int16_t computeHeartRate(void);

static int16_t red_min, red_max, red_min_2, red_max_2;
static int16_t ir_min, ir_max, ir_min_2, ir_max_2;

void processData(int16_t red, int16_t ir)
{
    static unsigned int sentCnt = 0;
    volatile unsigned int i;

    // 1.1. Update data on history
    updateValues(red, ir);

    // 1.2. Use FIR filter to get AC values
    int16_t red_ac_filtered = FIRRedOutput();
    int16_t ir_ac_filtered = FIRIROutput();

    // 1.3. Get the mean of DC values
    int16_t red_dc = red;//movingRedAvg();
    int16_t ir_dc = ir;//movingIRAvg();

    // 2. Send data via UART & SPI
    if (sentCnt++ == 0)
    {
        uint8_t values[14];
        values[0] = 0xFE;
        values[1] = 0;
        values[2] = 0;
        values[3] = 0;
        values[4] = 8;
        values[5] = red_ac_filtered >> 8;
        values[6] = red_ac_filtered;
        values[7] = red_dc >> 8;
        values[8] = red_dc;
        values[9] = ir_ac_filtered >> 8;
        values[10] = ir_ac_filtered;
        values[11] = ir_dc >> 8;
        values[12] = ir_dc;
        values[13] = 0;
        for (i = 0; i < 13; i++)
            values[13] ^= values[i];

        UART_TransmitData(values, 14);
        sentCnt = 0;
    }

    // 3. Find min & max of AC values
//    uint8_t done = findMinMax(red_ac_filtered, ir_ac_filtered);
//    if (done)
//    {
//        // 4. Compute RMS values
//        int16_t red_vpp = (abs(red_max - red_min) + abs(red_max_2 - red_min_2)) / 2;
//        int16_t ir_vpp = (abs(ir_max - ir_min) + abs(ir_max_2 - ir_min_2)) / 2;
//
//        float ir_rms = ir_vpp / 2.828427; // sqrt(8)
//        float red_rms = red_vpp / 2.828427; // sqrt(8)
//
//        // 5. Compute SpO2 & Heartrate
//        int16_t spo2 = (int16_t) computeSpO2(red_dc, red_rms, ir_dc, ir_rms);
//        int16_t heartrate = computeHeartRate();
//
//        // 6. Send data via UART & SPI
//        uint8_t result[10];
//        result[0] = 0xFE;
//        result[1] = 0;
//        result[2] = 1;
//        result[3] = 0;
//        result[4] = 4;
//        result[5] = spo2 >> 8;
//        result[6] = spo2;
//        result[7] = heartrate >> 8;
//        result[8] = heartrate;
//        result[9] = 0;
//        for (i = 0; i < 9; i++)
//            result[9] ^= result[i];
//
//        UART_TransmitData(result, 10);
//    }
}

static float computeSpO2(int16_t red_dc, int16_t red_rms, int16_t ir_dc, int16_t ir_rms)
{
    float ratio = (red_rms / red_dc) / (ir_rms / ir_dc);
    float spo2 = 114 - 32.5 * ratio;

    return spo2;
}

static int16_t computeHeartRate(void)
{
    return 0;
}

#define Sample_Window_Interval_down     50      // For detecting max 4Hz(250ms,240bpm) pulse rate, 250ms*0.5sample/ms(@2msPeriod)=125samples, 50,25
#define Sample_Window_Interval_up       30      // For detecting max 5Hz(200ms,300bpm) pulse rate, 200ms*0.25sample/ms(@4msPeriod)=50samples, 16,20

static uint8_t findMinMax(int16_t red, int16_t ir)
{
    static int16_t _red_min = 0, _red_max = 0, _ir_min = 0, _ir_max = 0;
    static unsigned int Sample_Window_Counter;

    static uint8_t Find_MaxMin_State = 0;
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
                _red_max = red;
                Sample_Window_Counter = Sample_Window_Interval_down;        // restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going down
                if (Sample_Window_Counter == 0)     //no more max peaks detected in the sampling window interval, go to detect min
                {
                    Find_MaxMin_State = 3;          //go to next state - Finding Min state

                    _ir_min = ir;
                    _red_min = red;
                    ir_max = _ir_max;
                    red_max = _red_max;
                }
            }

            break;

        case 3:     // Finding min
            if (ir <= _ir_min)
            {
                _ir_min = ir;
                _red_min = red;
                Sample_Window_Counter = Sample_Window_Interval_up;      // restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going up
                if (Sample_Window_Counter == 0)     //no more min peaks detected in the sampling window interval, go to detect max
                {
                    Find_MaxMin_State = 4;          //go to next state

                    _ir_max = ir;
                    _red_max = red;
                    ir_min = _ir_min;
                    red_min = _red_min;
                }
            }

            break;

        case 4:     //Finding Second Max
            if (ir >= _ir_max)
            {
                _ir_max = ir;
                _red_max = red;
                Sample_Window_Counter = Sample_Window_Interval_down;        //restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going down
                if (Sample_Window_Counter == 0)     //no more max peaks detected in the sampling window interval, go to detect min
                {
                    Find_MaxMin_State = 5;          //go to next state - Finding Min state

                    _ir_min = ir;
                    _red_min = red;
                    ir_max_2 = _ir_max;
                    red_max_2 = _red_max;
                }
            }

            break;

        case 5:     //Finding Second Min
            if (ir <= _ir_min)
            {
                _ir_min = ir;
                _red_min = red;
                Sample_Window_Counter = Sample_Window_Interval_up;      //restart counter
            }
            else
            {
                Sample_Window_Counter--;            //signal is now going up
                if (Sample_Window_Counter == 0)     //no more min peaks detected in the sampling window interval, go to detect max
                {
                    Find_MaxMin_State = 1;          //go to next state

                    _ir_max = ir;
                    _red_max = red;
                    ir_min_2 = _ir_min;
                    red_min_2 = _red_min;

                    return 1;
                }
            }
    }

    return 0;
}
