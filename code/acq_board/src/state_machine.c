#include "state_machine.h"
#include "process.h"
#include "filter.h"

#define SAMPLE_MEAN 25

typedef enum MachineState_t
{
    Stopped,
    Init,
    Red_ON,
    Red_ADC,
    Red_OFF,
    IR_ON,
    IR_ADC,
    IR_OFF,
    Process
} MachineState;

static MachineState state = Init;
void stateMachineLoop(void)
{
    volatile unsigned int i;
    static int16_t red, ir;
    switch (state)
    {
        case Stopped:
        {
            T1_SetDelay(STOPPED_PERIOD);
            break;
        }
        case Init:
        {
            // Wait a bit before starting
            T1_SetDelay(INIT_DELAY);
            state = Red_ON;
            break;
        }
        case Red_ON:
        {
            // Turn ON the Red LED
            writeGPIO(Red, 1);

            T1_SetDelay(ADC_DELAY);
            state = Red_ADC;
            break;
        }
        case Red_ADC:
        {
            // Read the Red value
            int32_t result = 0;
            for (i = 0; i < SAMPLE_MEAN; i++)
            {
                ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; // Start sampling/conversion
                while (!(ADC14->CTL0 & ADC14_CTL0_BUSY));      // Wait until the conversion is done
                result += ADC14->MEM[0];
            }

            red = result / SAMPLE_MEAN;

            T1_SetDelay(RED_ON_TIME - ADC_DELAY);
            state = Red_OFF;
            break;
        }
        case Red_OFF:
        {
            // Turn OFF the Red LED
            writeGPIO(Red, 0);

            T1_SetDelay(RED_TO_IR_TIME);
            state = IR_ON;
            break;
        }
        case IR_ON:
        {
            // Turn ON the IR LED
            writeGPIO(IR, 1);

            T1_SetDelay(ADC_DELAY);
            state = IR_ADC;
            break;
        }
        case IR_ADC:
        {
            // Read the IR value
            int32_t result = 0;
            for (i = 0; i < SAMPLE_MEAN; i++)
            {
                ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; // Start sampling/conversion
                while (!(ADC14->CTL0 & ADC14_CTL0_BUSY));      // Wait until the conversion is done
                result += ADC14->MEM[0];
            }

            ir = result / SAMPLE_MEAN;

            T1_SetDelay(IR_ON_TIME - ADC_DELAY);
            state = IR_OFF;
            break;
        }
        case IR_OFF:
        {
            // Turn OFF the IR LED
            writeGPIO(IR, 0);

            T1_SetDelay(PROCESS_DELAY);
            state = Process;
            break;
        }
        case Process:
        {
            // Process the data
            processData(red, ir);

            T1_SetDelay(PROCESS_DELAY);
            T1_Stop();

            state = Red_ON;
            break;
        }
    }
}

void processCommandByte(uint8_t b)
{
    switch (b)
    {
        case 't':
        {
            state = Stopped;
            break;
        }
        case 's':
        {
            state = Init;
            break;
        }
        case '0':
        {
            setFilterFunction(LP_5Hz);
            break;
        }
        case '1':
        {
            setFilterFunction(LP_15Hz);
            break;
        }
        case '4':
        {
            setFilterFunction(BP_3Hz);
            break;
        }
        case '2':
        {
            setFilterFunction(BP_5Hz);
            break;
        }
        case '3':
        {
            setFilterFunction(BP_15Hz);
            break;
        }
    }
}
