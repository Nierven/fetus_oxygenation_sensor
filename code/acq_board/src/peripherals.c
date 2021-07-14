#include "peripherals.h"
#include "state_machine.h"

void initInterrupts(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;    // Stop watchdog timer

    __enable_irq();                                // Enable global interrupt
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;           // Sleep on exit from ISR
//    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;          // Wake up on exit from ISR
}

void initGPIO(void)
{
    P1->DIR |= BIT0;                // Set P1.0/LED to output
    P1->OUT &= ~BIT0;               // Clear LED to start

    P6->DIR |= BIT1 | BIT0;         // Set P6.0_1/LED to output
    P6->OUT &= ~(BIT1 | BIT0);      // Clear LED to start
}

void initTimers(void)
{
    // ----------- TIMER 1 ----------- //

    // Timer32 set up in periodic mode, 32-bit, no pre-scale
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE |
            TIMER32_CONTROL_MODE;

    // Load Timer32 counter with period = 0xFFFFFFFF
    TIMER32_1->LOAD = 0xFFFFFFFF;

    // Enable the Timer32 interrupt in NVIC
    NVIC->ISER[0] = 1 << ((T32_INT1_IRQn) & 31);

    // Stop Timer32 with interrupt enabled
    TIMER32_1->CONTROL &= ~TIMER32_CONTROL_ENABLE;
    TIMER32_1->CONTROL |= TIMER32_CONTROL_IE;

    // ----------- TIMER 2 ----------- //

    // Timer32 set up in periodic mode, 32-bit, no pre-scale
    TIMER32_2->CONTROL = TIMER32_CONTROL_SIZE |
            TIMER32_CONTROL_MODE;

    // Load Timer32 counter with period = 5ms
    TIMER32_2->LOAD = 60000;

    // Enable the Timer32 interrupt in NVIC
    NVIC->ISER[0] = 1 << ((T32_INT2_IRQn) & 31);

    // Start Timer32 with interrupt enabled
    TIMER32_2->CONTROL |= TIMER32_CONTROL_ENABLE |
            TIMER32_CONTROL_IE;
}

void initADC(void)
{
    // GPIO Setup
    P5->SEL1 |= BIT5;                                         // Configure P5.5 (A0) for ADC
    P5->SEL0 |= BIT5;                                         // Configure P5.5 (A0) for ADC

    // Configure ADC14
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0;                      // ref+=AVcc, channel = A0

    ADC14->CTL0 = ADC14_CTL0_ON |                             // Turn ADC14 on
                  ADC14_CTL0_SHT0__16 |                       // 16 cycles for sample & conversion
                  ADC14_CTL0_SHP;                             // sample & hold mode
}

void initUART(void)
{
    CS->KEY = CS_KEY_VAL;                             // Unlock CS module for register access
    CS->CTL0 = 0;                                     // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;                     // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |                       // Select ACLK = REFO
               CS_CTL1_SELS_3 |                       // SMCLK = DCO
               CS_CTL1_SELM_3;                        // MCLK = DCO
    CS->KEY = 0;                                      // Lock CS module from unintended accesses

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);       // Enable eUSCIA0 interrupt in NVIC module

    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                          // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;           // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST |           // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;                // Configure eUSCI clock source for SMCLK

    // See table 24.5
    EUSCI_A0->BRW = 6;                               // 12000000/16/115200
    EUSCI_A0->MCTLW = (8 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;          // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;              // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                  // Enable USCI_A0 RX interrupt
}

uint8_t readGPIO(LED led)
{
    switch (led)
    {
        case Board:
        {
            return P1->OUT & BIT0;;
        }
        case Red:
        {
            return P6->OUT & BIT1;
        }
        case IR:
        {
            return P6->OUT & BIT0;
        }
    }

    return 0;
}

void writeGPIO(LED led, uint8_t on)
{
    switch (led)
    {
        case Board:
        {
            P1->OUT = on ? P1->OUT | BIT0 : P1->OUT & ~BIT0;
            break;
        }
        case Red:
        {
            P6->OUT = on ? P6->OUT | BIT1 : P6->OUT & ~BIT1;
            break;
        }
        case IR:
        {
            P6->OUT = on ? P6->OUT | BIT0 : P6->OUT & ~BIT0;
            break;
        }
    }
}

void T1_Start(void)
{
    TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;
}

void T1_Stop(void)
{
    TIMER32_1->CONTROL &= ~TIMER32_CONTROL_ENABLE;
}

void T1_SetDelay(uint32_t delayUs)
{
    // Load Timer32 counter with period "delayUs" in µs
    TIMER32_1->LOAD = 12 * delayUs; // 12 MHz
}

void sleep(void)
{
    __DSB();           // Ensures SLEEPONEXIT occurs immediately
    __sleep();         // Enter LPM0;
    __no_operation();  // For debug only
}

void UART_TransmitData(uint8_t *data, uint16_t length)
{
    volatile int i;
    for (i = 0; i < length; i++)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        EUSCI_A0->TXBUF = data[i];
    }
}

void T32_INT1_IRQHandler(void)
{
    stateMachineLoop();
    TIMER32_1->INTCLR |= BIT0;              // Clear Timer32 interrupt flag
}

void T32_INT2_IRQHandler(void)
{
    T1_Start();
    TIMER32_2->INTCLR |= BIT0;              // Clear Timer32 interrupt flag
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
        processCommandByte(EUSCI_A0->RXBUF);
}
