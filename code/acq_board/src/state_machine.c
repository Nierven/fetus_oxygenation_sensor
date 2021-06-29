#include "state_machine.h"
#include "process.h"

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
    static int16_t red, ir;
    switch (state)
    {
        case Stopped:
        {
            T32_SetDelay(STOPPED_PERIOD);
            break;
        }
        case Init:
        {
            // Wait a bit before starting
            T32_SetDelay(INIT_DELAY);
            state = Red_ON;
            break;
        }
        case Red_ON:
        {
//            writeGPIO(Board, !readGPIO(Board));
            // Turn ON the Red LED
            writeGPIO(Red, 1);

            T32_SetDelay(ADC_DELAY);
            state = Red_ADC;
            break;
        }
        case Red_ADC:
        {
            // Read the Red AC & DC values
            ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; // Start sampling/conversion
            while (!(ADC14->CTL0 & ADC14_CTL0_BUSY));      // Wait until the conversion is done

//            red_ac = ADC14->MEM[0];
            red = ADC14->MEM[1];

            T32_SetDelay(RED_ON_TIME - ADC_DELAY);
            state = Red_OFF;
            break;
        }
        case Red_OFF:
        {
            // Turn OFF the Red LED
            writeGPIO(Red, 0);

            T32_SetDelay(RED_TO_IR_TIME);
            state = IR_ON;
            break;
        }
        case IR_ON:
        {
            // Turn ON the IR LED
            writeGPIO(IR, 1);

            T32_SetDelay(ADC_DELAY);
            state = IR_ADC;
            break;
        }
        case IR_ADC:
        {
            // Read the Red AC & DC values
            ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC; // Start sampling/conversion
            while (!(ADC14->CTL0 & ADC14_CTL0_BUSY));      // Wait until the conversion is done

//            ir_ac = ADC14->MEM[0];
            ir = ADC14->MEM[1];

            T32_SetDelay(IR_ON_TIME - ADC_DELAY);
            state = IR_OFF;
            break;
        }
        case IR_OFF:
        {
            // Turn OFF the IR LED
            writeGPIO(IR, 0);

            T32_SetDelay(PROCESS_DELAY);
            state = Process;
            break;
        }
        case Process:
        {
            // Process the data
            processData(red, ir);

            T32_SetDelay(PROCESS_TIME);
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
    }
}
