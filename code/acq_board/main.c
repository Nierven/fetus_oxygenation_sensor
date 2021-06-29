#include "peripherals.h"
#include "filter.h"

int main(void)
{
    initInterrupts();
    initGPIO();
    initTimers();
    initADC();
    initUART();

    FiltersInit();

    // Wait a bit before starting
    T32_SetDelay(1000);

    // Enter sleep mode
    sleep();
}
