/*******************************************************************

              Microchip Pulse Oximeter Demo Main Code

********************************************************************
 FileName:        Main.c
 Description:     Pulse Oximeter Demo Main Code
 Processor:       dsPIC33FJ128GP802
 Compiler:        Microchip MPLAB C30 Compiler v3.31
 Company:         Microchip Technology Inc.
 Author:          Zhang Feng, Medical Products Group

********************************************************************
 SOFTWARE COPYRIGHT NOTICE:

 © [2013] Microchip Technology Inc. and its subsidiaries.
 You may use this software and any derivatives exclusively with Microchip products. 

 THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS,
 COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 

 IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
 EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

 MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.

********************************************************************
 MEDICAL DEMO WARNINGS, RESTRICTIONS AND DISCLAIMER:

 This demo is intended solely for evaluation and development purposes.
 It is not intended for medical diagnostic use.

********************************************************************
 Change History:

 Author       Rev   Date         Description
 Zhang Feng   1.0   10/05/2012   Initial Release, FIR Lowpass Filter, Schematics Rev 1.5.
 Zhang Feng   2.0   12/11/2012   - Changed FIR Lowpass Filter to FIR Bandpass Filter (in firmware) for removing basedline wander.
                                 - Added a RC Highpass Filter (in hardware) in between 1st stage amplifer and 2nd stage amplifer to reduce background light interference, Schematics revised to Rev 1.6.
 Zhang Feng   2.1   05/15/2013   - Changed Bandpass Filter to Kaiser window with filter length of 513.
                                 - Modified the Ratio-SpO2 lookup table.
 Zhang Feng   2.2   07/03/2013   - Display pulse rate and ratio readings in real time or gradually_display mode thru #define.
                                 - Added a RC Lowpass Filter (in hardware) at output of 2nd stage amplifer, Schematics revised to Rev 1.70.
                                 - Take 5 ADC samples from each light instead only one, and then everage them.
 Zhang Feng   2.3   12/18/2013   - Changed both PWMs' period from 2ms to 4ms. Therefore, the ADC sample rate became 250Hz (from 500Hz),
                                 - but the Sampling Frequency for FIR bandpass filter is still kept at 500Hz.
                                 - Added a RC Lowpass Filter (in hardware) right before AN0 in Schematics Rev 1.70.
                                 - Added max/min_found indicators in waveform display.
                                 - Modified Find_MaxMin() routine to detect two Maxs and two Mins in two consecutive pulses.
 Zhang Feng   2.4   01/29/2014   - Added "Reset_Counter" to reset the Find_MaxMin_State when the code gets stuck somewhere in the Switch loop of Find_MaxMin().
 Zhang Feng   2.5   07/29/2014   - Converted the project to MPLAB X project.
                                 - Added the WiFi code for the HHH demo.
 Zhang Feng   2.6   06/12/2015   - Added code for the RN4020 BTLE HUB/Gateway demo.
 Zhang Feng   2.7   07/14/2015   - Added code for the RN4020 BTLE to Android App.



********************************************************************
 Additional Note:

 See app note AN1525 "Pulse Oximeter Design Using Microchip’s dsPIC® Digital Signal Controllers (DSCs) and Analog Devices" for reference.
 
 - 20MIPS Internal Clock
 - FIR Bandpass Filter

*******************************************************************/


//*****************************************************************************
// Include and Header files
//*****************************************************************************
#include "p33Fxxxx.h"
#include "dsp.h"	//Don't #include "FIR_Filter.h" or "IIRT_Filter.h" if #include "dsp.h"
#include "math.h"
#include "GenericTypeDefs.h"
#include "lcd.h"
#include "I2C_MCP4728.h"
#include "delay.h"
#include "string.h"
#include <stdint.h>
#include "version.h"
#include "btle.h"

#ifdef HUB_POX
#include "timer.h"
#include "peripheral_comm.h"
#endif

#if defined(WiFi_Enable)||defined(WiFi_Config)
#include "wifi_command.h"
#endif

//*****************************************************************************
// PIC Configuration Bits
//*****************************************************************************
_FOSCSEL(FNOSC_FRC)					// Internal FRC Oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE)		// Clock Switching is enable and Fail Safe Clock Monitor is disabled, OSC2 is digital IO, Primary Oscillator Mode: Disabled
_FWDT(FWDTEN_OFF)					// Watchdog timer enabled/disabled by user software
_FICD(ICS_PGD2 & JTAGEN_OFF)		// Communicate on PGC1/EMUC1 and PGD1/EMUD1, Disable JTAG
//_FGS(GSS_STD & GCP_ON)				// User program memory is code-protected
_FGS(GSS_OFF & GCP_OFF)

//*****************************************************************************
// Global Definitions
//*****************************************************************************

#define Sleep_Enabled

//#define NellcorDS100A_9pin
#define NellcorDS100A_7pin
//#define ProSim8_NellcorDS100A_7pin
#define gradually_display

#define Sample_Window_Interval_down		50		// For detecting max 4Hz(250ms,240bpm) pulse rate, 250ms*0.5sample/ms(@2msPeriod)=125samples, 50,25
#define Sample_Window_Interval_up		30		// For detecting max 5Hz(200ms,300bpm) pulse rate, 200ms*0.25sample/ms(@4msPeriod)=50samples, 16,20
#define Filter_Length					1792
#define DCVppHigh						200		// 2000, 200
#define DCVppLow						100		// 1800, 100
#define dc_offset						440		// 2.048V/4096*400=0.2V, 440=0.22V, 500=0.25V
#define duty_cycle						70		// 30=200uSD/C@10MIPS, 70=450uSD/C@40MIPS, 70=220usD/C@20MIPS
#define oversampling_number				5		// Take 5 ADC samples from each light instead only one, and then everage them.

#ifdef NellcorDS100A_9pin
  #define Finger_Present_Threshold		620		// 2482=2V/(3.3V/4096), 4000, 620=Nellcor DS100A (9pin), 500=7pin probe
  #define initial_DAC_IRLed_intensity	2000	// 2000
  #define initial_DAC_RedLed_intensity	2000	// 2000
#elif defined ProSim8_NellcorDS100A_7pin || defined NellcorDS100A_7pin
  #define Finger_Present_Threshold		620		// 500=0.4V, 620=0.5V, 1200~=1V
  #define initial_DAC_IRLed_intensity	1700
  #define initial_DAC_RedLed_intensity	1700
#endif

#if defined(WiFi_Enable) && defined(WiFi_Config)
	#error Select either WiFi_Enable or WiFi_Config
#endif
#if defined(U1_waveform) && defined(WiFi_Config)
	#error Select either U1_waveform or WiFi_Config
#endif

//*****************************************************************************
// Global Variables
//*****************************************************************************
//extern FIRStruct BandpassIRFilter, BandpassRedFilter, BandpassIR0Filter, BandpassRed0Filter;
extern FIRStruct BandpassIRFilter, BandpassRedFilter, Bandpass250IRFilter, Bandpass250RedFilter;
signed int FIR_output_IR[1];
signed int FIR_output_Red[1];
signed int FIR_input_IR[1];
signed int FIR_input_Red[1];

unsigned int Vref;
unsigned int DAC_IRLed_intensity, DAC_RedLed_intensity;

signed int CH0_ADRES_IR, CH0_ADRES_Red, CH1_ADRES_IR, CH1_ADRES_Red;
signed int CH0_ADRES_IR_sum, CH0_ADRES_Red_sum, CH1_ADRES_IR_sum, CH1_ADRES_Red_sum;
signed int CH1_ADRES_IR_max, CH1_ADRES_IR_min;
signed int CH1_ADRES_Red_max, CH1_ADRES_Red_min;
signed int IR_Max, IR_Min, Red_Max, Red_Min;
signed int IR_Max2, IR_Min2, Red_Max2, Red_Min2, IR_Max_display, IR_Min_display;
signed int IR_Vpp, Red_Vpp;
signed int IR_Vpp1, Red_Vpp1, IR_Vpp2, Red_Vpp2;
double IR_Vrms, Red_Vrms;
double Ratio;
signed int CH1_ADRES_IR_temp;

double Ratio_temp[8];
double Ratio_previous;

signed int Baseline_Upper_Limit, Baseline_Lower_Limit, Baseline_ambient;

unsigned char Read_ADC_IR, Read_ADC_Red;
unsigned int Sample_Window_Counter;
unsigned char Find_MaxMin_State=0;
unsigned char Meter_State=0;
unsigned int Samples_Between_Pulses=0, Samples_Between_MinPulses=0;
unsigned char Pulse_Rate, Pulse_Rate_previous, Pulse_Rate_max, Pulse_Rate_min;
unsigned char SpO2, SpO2_previous;
unsigned int maxCounter=0, minCounter=0;
double SpO2_temp;

unsigned char PR_cnt, PR_ready, Ratio_cnt, Ratio_ready;
unsigned int Pulse_Rate_evg;
double Ratio_evg;

unsigned char Detection_Done=0;
unsigned char first_reading=0;
unsigned char RedReady=0, IRReady=0;
unsigned char goto_sleep=0;

//unsigned char SpO2_buffer[4] = {0};
//unsigned char PR_buffer[4] = {0};
//unsigned char avg_counter, avg_counter2;

const char mytext0[] =  "Microchip  Pulse";
const char mytext1[] = "Oximeter Demo";
const char mytext2[] = " SPO2(%): ";
const char mytext3[] = " PR(bpm): ";
const char mytext4[] = "Check SpO2 Probe";
const char mytext5[] = "Goto Sleep...";
const char mytext6[] = "Ratio:";
const char mytext7[] = "%SPO2=";
const char mytext8[] = "Measuring...";
const char mytext9[] = " PR:";
const char mytext10[] = Firmware_Version;

const char mytext11[] = "Measures your";
const char mytext12[] = "heart rate and";
const char mytext13[] = "blood oxygen";
const char mytext14[] = "saturation level";
const char mytext15[] = "Great function";
const char mytext16[] = "for wearables";
const char mytext17[] = "Press MCLR Buttn";
const char mytext18[] = "To Start Measure";

extern volatile unsigned char hunds;
extern volatile unsigned char tens;
extern volatile unsigned char ones;

unsigned char FirstByte;
unsigned char SecondByte_SingleWrite_A;
unsigned char SecondByte_MultiWrite_A;
unsigned char ThirdByte_A;
unsigned char FourthByte_A;
unsigned char SecondByte_SingleWrite_B;
unsigned char SecondByte_MultiWrite_B;
unsigned char ThirdByte_B;
unsigned char FourthByte_B;
unsigned char SecondByte_SingleWrite_C;
unsigned char SecondByte_MultiWrite_C;
unsigned char ThirdByte_C;
unsigned char FourthByte_C;
unsigned char SecondByte_SingleWrite_D;
unsigned char SecondByte_MultiWrite_D;
unsigned char ThirdByte_D;
unsigned char FourthByte_D;

unsigned int wf;
unsigned int delay0, delay_counter1;

unsigned char max_found=0, min_found=0;
unsigned int Reset_Counter=0;

//*****************************************************************************
// Local Function Prototypes
//*****************************************************************************
void Init(void);
void CH0_process(void);
void CH1_process(void);
void U1_write1byte(unsigned char value);
void send2U1(unsigned char* Uart_Output, unsigned int sz);
void sendChar2U1(unsigned char ch);
void send2U2(const char *Uart_Output, unsigned char sz);
void sendChar2U2(unsigned char ch);
void Find_MaxMin(void);
void SpO2_Calculation(void);
void Pulse_Rate_Calculation(void);
void LCD_display(void);
void LCD_display_Ratio(void);
extern void hexdec( unsigned char count );
unsigned char Calibrate_IR(void);
unsigned char Calibrate_Red(void);
void str_Int(uint16_t i);
void uitoa(WORD Value, BYTE* Buffer);

#ifdef HUB_POX
#define RESCAN_TIME_ms              20000   //10000
#define TIME_BETWEEN_CONNECTS_ms    10000   //5000
#define ENTER_COMMAND_MODE_DELAY_ms 1500
timer_t rescanBtle;
timer_t connectBtle;
void HUB_POX_TASK(void);
#endif

#ifdef Android_App
void Android_TASK(void);
#endif

/******************************************************************************
* Function:         Init (void) 
*
* Overview:         This function is called when the system first
*                   powers up and is responsible for initializing the
*                   variables, registers, and modules of the system.
*
* Note:             None
******************************************************************************/
void Init(void)
{
    //*****************************************************************************
	// Configure Oscillator to operate the device at 40MIPS where M=43,N1=N2=2
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	// Fosc= 7.37M*43/(2*2)=79.22Mhz for ~40MIPS input clock
	//
	// Configure Oscillator to operate the device at 10MIPS where M=11,N1=N2=2
	// Fosc= 7.37M*11/(2*2)=20.2675Mhz for ~10MIPS input clock
	//
	// Configure Oscillator to operate the device at 20MIPS where M=22,N1=N2=2
	// Fosc= 7.37M*22/(2*2)=20.2675Mhz for ~20MIPS input clock
    //*****************************************************************************
	PLLFBD = 20;							// 41=M=43@40MIPS, 9=1001->M=11@10MIPS, 20=10100->M=22@20MIPS
	CLKDIVbits.PLLPOST = 0;					// N1=2
	CLKDIVbits.PLLPRE = 0;					// N2=2
	OSCTUN = 0;								// Tune FRC oscillator, if FRC is used
	RCONbits.SWDTEN = 0;					// Disable Watch Dog Timer

	// Clock switch to incorporate PLL
	__builtin_write_OSCCONH(0x01);			// Initiate Clock Switch to FRC with PLL (NOSC=0b001)
	__builtin_write_OSCCONL(0x01);			// Start clock switching

	while (OSCCONbits.COSC != 0b001);		// Wait for Clock switch to occur
	while (OSCCONbits.LOCK != 1) {};		// Wait for PLL to lock

    //***************************
    // Initialize Ports
    //***************************
    PORTA = 0x0000;
    PORTB = 0x0000;
	LATA  = 0x0000;
	LATB  = 0x0000;
	TRISA = 0x0000;
	TRISB = 0x0000;
	AD1PCFGL = 0xffff;				// Set all AN ports digtial

    //*******************************************************************************************************************************************************
    // Initialize 12-bit ADC module, (AN0 = DC signal at 1st stage amplifier & AN1 = AC signal at 2nd stage amplifier)
    //*******************************************************************************************************************************************************
	TRISAbits.TRISA0 = 1;			// Set AN0 to input pin
	TRISAbits.TRISA1 = 1;			// Set AN1 to input pin
	AD1PCFGLbits.PCFG0 = 0;			// configure AN0/RA0 as Analog pin
	AD1PCFGLbits.PCFG1 = 0;			// configure AN1/RA1 as Analog pin
	AD1CON1 = 0x04e0;				// Select 12-bit 1-channel ADC operation, unsigned integer, internal counter auto-convert, Sampling begins when SAMP bit is set
	AD1CON2 = 0x0000;				// ADREF+ = AVdd, ADREF- = AVss
	AD1CON3 = 0x1f03;				// ADC clock derived from system clock
									// ADC Conversion clock Tad=Tcy*(ADCS+1) = (1/40M)*4 = 100ns
									// Auto sample time SAMC = 31*Tad = 31*100 = 3.1us
									// ADC Conversion Time for 12-bit Tc=14*Tad = 14*100 = 1.4us
	AD1CHS0 = 0x0001;				// initial positive input for Sample A is AN1, negative input is Vref-
	IPC3bits.AD1IP = 1;				// ADC1 conversion complete interrupt priority
	IFS0bits.AD1IF = 0;				// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE = 0;				// Disable A/D interrupt
	AD1CON1bits.ADON = 1;			// Turn on the A/D converter

    //***************************
    // Initialize LCD Display
    //***************************
	Init_LCD();

    //*********************************************************************************************************
    // Initialize Timer 2 - IR light
    //*********************************************************************************************************
	T2CON = 0x0020;					// Stop 16-bit Timer2, 1:64(40MhzFosc) Prescale, Internal clock (Fosc/2)
	TMR2 = 0x00;					// Clear timer register
	PR2 = 1250;						// Load the period value, OCxRS <= PRx, 4ms period = (1/(Fosc/2))*1000*64*PR2 = (1/(40000000/2))*1000*64*1250
	IPC1bits.T2IP = 2;				// Set Timer2 Interrupt Priority Level
	IFS0bits.T2IF = 0;				// Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 1;				// Enable Timer2 Interrupt

    //*********************************************************************************************************
    // Initialize Timer 3 - Red light
    //*********************************************************************************************************
	T3CON = 0x0020;					// Stop 16-bit Timer3, 1:64(40MhzFosc) Prescale, Internal clock (Fosc/2)
	TMR3 = 0x00;					// Clear timer register
	PR3 = 1250;						// Load the period value, OCxRS <= PRx, 4ms period = (1/(Fosc/2))*1000*64*PR2 = (1/(40000000/2))*1000*64*1250
	IPC2bits.T3IP = 2;				// Set Timer3 Interrupt Priority Level
	IFS0bits.T3IF = 0;				// Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 1;				// Enable Timer3 Interrupt

    //*********************************************************************************************************
    // Initialize Output Compare 1 module in Continuous Pulse mode, OC1 controls IR LED switch
    //*********************************************************************************************************
	RPOR6bits.RP13R = 0b10010;		// RP13/RB13 tied to OC1 (IR)
	OC1CONbits.OCM = 0b000; 		// Disable Output Compare 1 Module
	OC1R = 0; 						// Write the duty cycle for the first PWM pulse, 24=8MHzFosc(50us), 30=40MHzFosc(50us), 600=40MHzFosc(1ms)
	OC1RS = duty_cycle; 			// Write the duty cycle for the second PWM pulse, OCxRS <= PRx, 499=8MHzFosc(1ms), 623=40MHzFosc(1ms), 1246=40MHzFoc,2msPeriod, 4984=40MHzFoc,8msPeriod, 280=450us D/C@40MHzFoc,2msPeriod,switch
	OC1CONbits.OCTSEL = 0; 			// Select Timer 2 as output compare time base

    //*********************************************************************************************************
    // Initialize Output Compare 2 module in Continuous Pulse mode, OC2 controls Red LED switch
    //*********************************************************************************************************
	RPOR6bits.RP12R = 0b10011;		// RP12/RB12 tied to OC2 (Red)
	OC2CONbits.OCM = 0b000; 		// Disable Output Compare 2 Module
	OC2R = 0; 						// Write the duty cycle for the first PWM pulse, 24=8MHzFosc, 30=40MHzFosc, 600=40MHzFosc(1ms)
	OC2RS = duty_cycle; 			// Write the duty cycle for the second PWM pulse, OCxRS <= PRx, 499=8MHzFosc(1ms), 623=40MHzFosc(1ms), 1246=40MHzFoc,2msPeriod, 4984=40MHzFoc,8msPeriod, 280=450us D/C@40MHzFoc,2msPeriod,switch
	OC2CONbits.OCTSEL = 1; 			// Select Timer 3 as output compare time base

#ifdef HUB_POX
    //Initial Hub_Pedometer related functions
    InitBtleUart();
    InitTimer();
    InitPeripheral();
    ClearRxFifo();
#else
    //*****************************************************************************************
    // Initialize UART1 module for outputing serial data or displaying waveform, 115200/8-N-1
    //*****************************************************************************************
#ifndef WiFi_Config
#if defined(U1_waveform) || defined(Android_App)
	RPOR7bits.RP15R = 0b00011;		// RP15/RB15/Pin26 tied to U1TX
	U1MODEbits.STSEL = 0;			// 1 Stop bit
	U1MODEbits.PDSEL = 0;			// No Parity, 8 data bits
	U1MODEbits.ABAUD = 0;			// Auto-Baud Disabled
	U1MODEbits.BRGH = 1;			// High Speed mode
	U1BRG = 42;						// BAUD Rate Setting for 115200 BRGH=1 UxBRG=((Fcy/Baudrate)/4)-1=86, for 19200, BRGH=0 UxBRG = ((Fcy/Baudrate)/16)-1 = 40000000/19200/16-1 = 129, (UxBRG=259 for 9600), Fcy=40MHz, 21=10MIPS, 42=20MIPS
	U1STAbits.UTXISEL0 = 1;			// UTXISEL<1:0> = 01 = Interrupt when all transmit operations are completed.
	U1STAbits.UTXISEL1 = 0;
	IPC3bits.U1TXIP = 2;			// UART Transmitter interrupt priority
	IPC2bits.U1RXIP = 0;			// UART Receiver interrupt prioirty
	IFS0bits.U1TXIF = 0;			// Clear UART Transmitter interrupt flag
	IFS0bits.U1RXIF = 0;			// Clear UART Receiver interrupt flag
	IEC0bits.U1TXIE = 0;			// Disable UART Transmitter interrupt
	IEC0bits.U1RXIE = 0;			// Disable UART Receiver interrupt
	U1MODEbits.UARTEN = 1;			// Enable UART
	U1STAbits.UTXEN = 1;			// Enable UART TX, wait at least 104 usec (1/9600) before sending first char
#endif
#endif

#if defined(WiFi_Enable) || defined(WiFi_Config)
    InitWiFiUart();

    //*********************************************************************************************************
    // Initialize Timer 1 - Used for other stuff
    //*********************************************************************************************************
	T1CON = 0x0030;				// Stop 16-bit Timer2, 1:256 Prescale, Internal clock (Fosc/2)
	TMR1 = 0x00;				// Clear timer register
	PR1 = 4984;				// Load the period value, OCxRS <= PRx, 499=8MHzFosc(1ms), 623=40MHzFosc(1ms), 1246=40MHzFoc,2msPeriod, 2492=4msPeriod, 4984=40MHzFoc,8msPeriod, 312=2msPeriod@10MIPS
#endif
#endif
    //*********************************************************************************
    // Initialize I2C for communicating with external DAC MCP4728
    //*********************************************************************************
	Init_I2C();

	//*****************************************************************************
	// Initialize MCP4728 DAC Output values
	//*****************************************************************************
	DAC_IRLed_intensity = initial_DAC_IRLed_intensity;		// (DAC-A) DAC output voltage for controlling IR LED current, 2000=1.0V (@1.5V IR)
	DAC_RedLed_intensity = initial_DAC_RedLed_intensity;	// (DAC-B) DAC output voltage for controlling Red LED current, 1500=0.75V (@1.5V Red)

	//Address Byte for MCP4728: 1100 000 0, DeviceCode=1100, AddressBits=000, R/nW=0(write)
	FirstByte = 0xc0;

	//Single Write Command (bit7-3) for DAC Input Register and EEPROM of selected channel (bit2,1 / DAC1,2)
	//2nd Byte format:  0     1    0    1   1    DAC1  DAC0  nUDAC
	//3rd Byte format:  Vref  PD1  PD0  Gx  D11  D10   D9    D8
	//4th Byte format:  D7    D6   D5   D4  D3   D2    D1    D0
	//DAC input register = 0x80 = internal Vref 2.048V

	SecondByte_SingleWrite_A = 0b01011000;	//Channel A
	SecondByte_SingleWrite_B = 0b01011010;	//Channel B
	SecondByte_SingleWrite_C = 0b01011100;	//Channel C
	SecondByte_SingleWrite_D = 0b01011110;	//Channel D

	ThirdByte_A = 0x80 + (DAC_IRLed_intensity>>8 & 0x0f);
	FourthByte_A = DAC_IRLed_intensity & 0x00ff;

	ThirdByte_B = 0x80 + (DAC_RedLed_intensity>>8 & 0x0f);
	FourthByte_B = DAC_RedLed_intensity & 0x00ff;

	ThirdByte_C = 0x80 + (dc_offset>>8 & 0x0f);				// (DAC-C) DC offset value for the second stage amplier
	FourthByte_C = dc_offset & 0x00ff;

	ThirdByte_D = 0x40;										// (DAC-D) Channel D is disabled
	FourthByte_D = 0x00;

	//Multi-Write Command for DAC Input Registier of selected channel only, EEPROM not affected.
	SecondByte_MultiWrite_A = 0b01000000;	//Channel A
	SecondByte_MultiWrite_B = 0b01000010;	//Channel B
	SecondByte_MultiWrite_C = 0b01000100;	//Channel C
	SecondByte_MultiWrite_D = 0b01000110;	//Channel D

	//Write Vref bits of each channel in this format: 1 0 0 X VrefA VrefB VrefC VrefD
	//SecondByte_WriteVref = 0x8e;			//Select Internal Vref 2.048V for channel A,B,C and Vdd for channel D (Vref=0 for Vdd, Vref=1 for Internal Ref)

	//Write Gain bits of each channel in this format: 1 1 0 X GxA GxB GxC GxD
	//SecondByte_WriteGain = 0xc0;			//Select Gain of 1 for all channels (Gx=0 for Gain 1, Gx=1 for Gain 2)

	//Write Power-Down bits of each channel in this format: 1 0 1 X PD1A PD0A PD1B PD0B, PD1C PD0C PD1D PD0D X X X X
	//SecondByte_WritePD = 0xa0;			//Select Normal Mode for channel A,B
	//ThirdByte_WritePD = 0x20;				//Select Normal Mode for channel C and Power-Down for channel D

	//Single Write Mode requires to connect and monitor the RDY/nBSY pin for EEPROM write time.
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_A, ThirdByte_A, FourthByte_A);
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_B, ThirdByte_B, FourthByte_B);
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_C, ThirdByte_C, FourthByte_C);
//	I2C_MCP4728_Write(FirstByte, SecondByte_SingleWrite_D, ThirdByte_D, FourthByte_D);

	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_A, ThirdByte_A, FourthByte_A);
	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_B, ThirdByte_B, FourthByte_B);
	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_C, ThirdByte_C, FourthByte_C);
	I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_D, ThirdByte_D, FourthByte_D);

    //**********************************************************************************************************************
    // Initialize FIR Bandpass Digital Filter (Generated by dsPIC Filter Design Software)
	//   Sampling Frequency(Hz) = 500, Passband Frequency(Hz) = 1 & 5, Stopband Frequency(Hz) = 0.05 & 25,
	//   Passband Ripple(-dB) = 0.1, Stopband Ripple(-dB) = 50, Kaiser Window, Filter Length = 513.
    //**********************************************************************************************************************
	FIRDelayInit(&BandpassIRFilter);
	FIRDelayInit(&BandpassRedFilter);

	//**********************************************************************************************************************
    // Initialize FIR Bandpass Digital Filter (Generated by dsPIC Filter Design Software)
	//   Sampling Frequency(Hz) = 500, Passband Frequency(Hz) = 1 & 5, Stopband Frequency(Hz) = 0.05 & 25,
	//   Passband Ripple(-dB) = 0.1, Stopband Ripple(-dB) = 50, Kaiser Window, Filter Length = 201.
    //**********************************************************************************************************************
//	FIRDelayInit(&BandpassIR0Filter);
//	FIRDelayInit(&BandpassRed0Filter);

    //**********************************************************************************************************************
    // Initialize FIR Bandpass Digital Filter (Generated by dsPIC Filter Design Software)
	//   Sampling Frequency(Hz) = 250, Passband Frequency(Hz) = 1 & 5, Stopband Frequency(Hz) = 0.05 & 25,
	//   Passband Ripple(-dB) = 0.1, Stopband Ripple(-dB) = 50, Kaiser Window, Filter Length = 513.
    //**********************************************************************************************************************
//	FIRDelayInit(&Bandpass250IRFilter);
//	FIRDelayInit(&Bandpass250RedFilter);

}

/****************************************************************************
*
* MAIN FUNCTION
*
*****************************************************************************/
int main(void)
{
	unsigned int delay;
    
#ifdef HUB_POX
    TimerStart( & rescanBtle, 0 );
    TimerStart( & connectBtle, TIME_BETWEEN_CONNECTS_ms );
#endif
    
	//********** Initialization **********
	Init();

	//***** LCD Welcome message *****
	home_clr();
	puts_lcd( (unsigned char*) &mytext0[0], sizeof(mytext0) -1 );
	line_2();
	puts_lcd( (unsigned char*) &mytext1[0], sizeof(mytext1) -1 );
	puts_lcd( (unsigned char*) &mytext10[0], sizeof(mytext10) -1 );

	for (delay_counter1=0; delay_counter1<100; delay_counter1++)
	{
		for (delay0=0; delay0<30000; delay0++);
	}
    
    home_clr();
	puts_lcd( (unsigned char*) &mytext8[0], sizeof(mytext8) -1 );

#if defined(WiFi_Config)			//Configure WiFi module
	WiFi_Module_Config();
#endif
#if defined(WiFi_Config) || defined(WiFi_Enable)			//Configure WiFi module
   //while(WiFi_Join_AP());
   for (delay2=0; delay2<30000; delay2++);
   //while(WiFi_Open_Socket());
#endif

	//********** Enable OC1 & OC2 ouputs for IR & Red LED's on/off switch **********
	OC2CONbits.OCM = 0b101;				// Select the Output Compare 2 mode, Turn on Red LED
	T3CONbits.TON = 1;					// Start Timer3

	for (delay=0; delay<2200; delay++);

	OC1CONbits.OCM = 0b101;				// Select the Output Compare 1 mode, Turn on IR LED
	T2CONbits.TON = 1;					// Start Timer2

	goto_sleep = 0;
	first_reading = 0;
    
	//********** Main Loop **********
	while (1)
	{
		if (goto_sleep)
		{
			home_clr();
			puts_lcd( (unsigned char*) &mytext4[0], sizeof(mytext4) -1 );	//display "check spo2 probe" text on LCD
			line_2();
			puts_lcd( (unsigned char*) &mytext5[0], sizeof(mytext5) -1 );	//display "Goto Sleep..." text on LCD
			for (delay_counter1=0; delay_counter1<100; delay_counter1++)	//delay for displaying text on LCD
			{
				for (delay0=0; delay0<30000; delay0++);
			}

			I2C_MCP4728_Write(FirstByte, 0xa0, 0x40, 0x00);		//2nd_Byte = 0b10100000(power-down ch_A), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
			I2C_MCP4728_Write(FirstByte, 0xa2, 0x40, 0x00);		//2nd_Byte = 0b10100010(power-down ch_B), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
			I2C_MCP4728_Write(FirstByte, 0xa4, 0x40, 0x00);		//2nd_Byte = 0b10100100(power-down ch_C), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
			I2C_MCP4728_Write(FirstByte, 0xa6, 0x40, 0x00);		//2nd_Byte = 0b10100110(power-down ch_D), 3rd_Byte = 0b01000000(Vref=VDD 100kOhm)
/*
			#ifdef WiFi_Enable
			send2U2 (close_TCP, 6);		//disconnect a TCP connection
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			for (delay2=0; delay2<30000; delay2++);
			//JM add sleep
//			send2U2 (WF_sleep, 6);		//put the WiFi module to sleep mode
			#endif
*/
			home_clr();
			PMD1 = 0xffff;
			PMD2 = 0xffff;
			PMD3 = 0xffff;
			Nop();

//			LATBbits.LATB14 = 0;		//Disable MCP1640C
			Nop();
			Nop();

			while (1)
			{
                /*
                //----- Keep displaying following text for Show Mode -----
                puts_lcd( (unsigned char*) &mytext0[0], sizeof(mytext0) -1 );
                line_2();
                puts_lcd( (unsigned char*) &mytext1[0], sizeof(mytext1) -1 );
                for (delay_counter1=0; delay_counter1<100; delay_counter1++)
                {
                    for (delay0=0; delay0<60000; delay0++);
                }
                home_clr();
                puts_lcd( (unsigned char*) &mytext11[0], sizeof(mytext11) -1 );
                line_2();
                puts_lcd( (unsigned char*) &mytext12[0], sizeof(mytext12) -1 );
                for (delay_counter1=0; delay_counter1<100; delay_counter1++)
                {
                    for (delay0=0; delay0<60000; delay0++);
                }
                home_clr();
                puts_lcd( (unsigned char*) &mytext13[0], sizeof(mytext13) -1 );
                line_2();
                puts_lcd( (unsigned char*) &mytext14[0], sizeof(mytext14) -1 );
                for (delay_counter1=0; delay_counter1<100; delay_counter1++)
                {
                    for (delay0=0; delay0<60000; delay0++);
                }
                home_clr();
                puts_lcd( (unsigned char*) &mytext15[0], sizeof(mytext15) -1 );
                line_2();
                puts_lcd( (unsigned char*) &mytext16[0], sizeof(mytext16) -1 );
                for (delay_counter1=0; delay_counter1<100; delay_counter1++)
                {
                    for (delay0=0; delay0<60000; delay0++);
                }
                home_clr();
                puts_lcd( (unsigned char*) &mytext17[0], sizeof(mytext17) -1 );
                line_2();
                puts_lcd( (unsigned char*) &mytext18[0], sizeof(mytext18) -1 );
                for (delay_counter1=0; delay_counter1<100; delay_counter1++)
                {
                    for (delay0=0; delay0<60000; delay0++);
                }
                home_clr();
                //-------------------------------------------------------------
                */
				Sleep();					// Put MCU into sleep
				Nop();
			}
		}

		//--------- Main State Machine starts here ---------
		if (RedReady && IRReady)
		{
			RedReady = 0;
			IRReady = 0;

//			LATBbits.LATB14 = 1;			//for debugging

			FIR(1, &FIR_output_IR[0], &FIR_input_IR[0], &BandpassIRFilter);
			FIR(1, &FIR_output_Red[0], &FIR_input_Red[0], &BandpassRedFilter);

			CH1_ADRES_IR = FIR_output_IR[0];
			CH1_ADRES_Red = FIR_output_Red[0];

//			FIR(1, &FIR_output_IR[0], &FIR_input_IR[0], &Bandpass250IRFilter);
//			CH1_ADRES_IR = FIR_output_IR[0];
//			FIR(1, &FIR_output_Red[0], &FIR_input_Red[0], &Bandpass250RedFilter);
//			CH1_ADRES_Red = FIR_output_Red[0];

			Reset_Counter++;

			Find_MaxMin();

			#ifdef U1_waveform			//using mTouch Sensing Solutions GUI
			str_Int(0x0f);				//Turn on LED1, 3=Turn on LED1(RED=Red) & LED2(IR=Blue)
			str_Int(CH1_ADRES_Red);			//convert unsigned int to ascii - RED, output one light at a time
			str_Int(CH1_ADRES_IR);			//convert unsigned int to ascii
                        if (max_found)
			{
                            str_Int(IR_Max_display);
                            max_found = 0;
			}
			else { str_Int(400); }
			if (min_found)
			{
                            str_Int(IR_Min_display);
                            min_found = 0;
			}
			else { str_Int(400); }
			U1_write1byte(0x0d);			//footer1
			U1_write1byte(0x0a);			//footer2
			#endif

			if (maxCounter == 1)
				Samples_Between_Pulses++;			//Counting samples between two continuous pulses' peak for PR
			if (minCounter == 1)
				Samples_Between_MinPulses++;		//Counting samples between two continuous pulses' min/bottom for PR

			if (Detection_Done)
			{
				//Max & Min are all found. Calculate SpO2 & Pulse Rate
				SpO2_Calculation();				//calculate SpO2
				Pulse_Rate_Calculation();		//calculate pulse rate

				#ifdef gradually_display	//gradually dispaly results' different
				if (first_reading==1)
				{
					SpO2_previous = SpO2;
					Pulse_Rate_previous = Pulse_Rate;
					first_reading = 2;
				}
				else if (first_reading==2)
				{
					if (SpO2 > SpO2_previous)
					{
						SpO2 = SpO2_previous + 1;
						SpO2_previous = SpO2;
					}
					else if (SpO2 < SpO2_previous)
					{
						SpO2 = SpO2_previous - 1;
						SpO2_previous = SpO2;
					}
					if (Pulse_Rate > Pulse_Rate_previous)
					{
						Pulse_Rate = Pulse_Rate_previous + 1;
						Pulse_Rate_previous = Pulse_Rate;
					}
					else if (Pulse_Rate < Pulse_Rate_previous)
					{
						Pulse_Rate = Pulse_Rate_previous - 1;
						Pulse_Rate_previous = Pulse_Rate;
					}
				}
				#endif

				if (first_reading > 0)			//First reading is no good, not display.
				{
                    LCD_display();			//Send the result to LCD display
				    //LCD_display_Ratio();	//for debugging
                    
                #if defined (HUB_POX)
                    HUB_POX_TASK();
                #elif defined (Android_App)
                    Android_TASK();
                #elif defined (WiFi_Enable)
                    WiFi_TASK();
                #endif
                }

                if (first_reading != 2)
                    first_reading = 1;

				Detection_Done = 0;
				maxCounter = 0;
				minCounter = 0;
				Samples_Between_Pulses = 0;
				Samples_Between_MinPulses = 0;
				Reset_Counter = 0;
			}

			//If no Detection_Done occured every 0x7ff counts, the code is probably stuck somewhere in the switch loop in Find_MaxMin(). Then reset Find_MaxMin_State.
			if (Reset_Counter >= 0x7ff)
			{
				Reset_Counter = 0;
				Find_MaxMin_State = 0;
			}
//			LATBbits.LATB14 = 0;			//for debugging
		}
	}
}

/*******************************************************************************
 * Function Name: Find_MaxMin()
 * Specification: Detecte the max & min value of the IR & Red AC signals at AN1
 *******************************************************************************/
void Find_MaxMin(void)
{
	switch (Find_MaxMin_State)
	{
		case 0:
			CH1_ADRES_IR_max = CH1_ADRES_IR;
			Find_MaxMin_State = 1;
			break;

		case 1:
			if (CH1_ADRES_IR >= CH1_ADRES_IR_max)
			{
				CH1_ADRES_IR_max = CH1_ADRES_IR;
				Find_MaxMin_State = 2;
			}
			else
			{
				Find_MaxMin_State = 0;
			}
			break;

		case 2:		//Finding Max
			if (CH1_ADRES_IR >= CH1_ADRES_IR_max)
			{
				CH1_ADRES_IR_max = CH1_ADRES_IR;
				CH1_ADRES_Red_max = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval_down;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going down
				if (Sample_Window_Counter == 0)		//no more max peaks detected in the sampling window interval, go to detect min
				{
					Find_MaxMin_State = 3;			//go to next state - Finding Min state
					maxCounter++;
//					CH1_ADRES_IR_min = CH1_ADRES_IR_max;
//					CH1_ADRES_Red_min = CH1_ADRES_Red_max;
					CH1_ADRES_IR_min = CH1_ADRES_IR;
					CH1_ADRES_Red_min = CH1_ADRES_Red;
					IR_Max = CH1_ADRES_IR_max;
					Red_Max = CH1_ADRES_Red_max;

					IR_Max_display = IR_Max;
					max_found = 1;
				}
			}
			break;

		case 3:		//Finding Min
			if (CH1_ADRES_IR <= CH1_ADRES_IR_min)
			{
				CH1_ADRES_IR_min = CH1_ADRES_IR;
				CH1_ADRES_Red_min = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval_up;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going up
				if (Sample_Window_Counter == 0)		//no more min peaks detected in the sampling window interval, go to detect max
				{
					Find_MaxMin_State = 4;			//go to next state
					minCounter++;
//					CH1_ADRES_IR_max = CH1_ADRES_IR_min;
//					CH1_ADRES_Red_max = CH1_ADRES_Red_min;
					CH1_ADRES_IR_max = CH1_ADRES_IR;
					CH1_ADRES_Red_max = CH1_ADRES_Red;
					IR_Min = CH1_ADRES_IR_min;
					Red_Min = CH1_ADRES_Red_min;

					IR_Min_display = IR_Min;
					min_found = 1;
				}
			}
			break;

		case 4:		//Finding Second Max
			if (CH1_ADRES_IR >= CH1_ADRES_IR_max)
			{
				CH1_ADRES_IR_max = CH1_ADRES_IR;
				CH1_ADRES_Red_max = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval_down;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going down
				if (Sample_Window_Counter == 0)		//no more max peaks detected in the sampling window interval, go to detect min
				{
					Find_MaxMin_State = 5;			//go to next state - Finding Min state
					maxCounter++;

//					CH1_ADRES_IR_min = CH1_ADRES_IR_max;
//					CH1_ADRES_Red_min = CH1_ADRES_Red_max;
					CH1_ADRES_IR_min = CH1_ADRES_IR;
					CH1_ADRES_Red_min = CH1_ADRES_Red;
					IR_Max2 = CH1_ADRES_IR_max;
					Red_Max2 = CH1_ADRES_Red_max;

//					Detection_Done = 1;				//Found Max & Min

					IR_Max_display = IR_Max2;
					max_found = 1;
				}
			}
			break;

		case 5:		//Finding Second Min
			if (CH1_ADRES_IR <= CH1_ADRES_IR_min)
			{
				CH1_ADRES_IR_min = CH1_ADRES_IR;
				CH1_ADRES_Red_min = CH1_ADRES_Red;
				Sample_Window_Counter = Sample_Window_Interval_up;		//restart counter
			}
			else
			{
				Sample_Window_Counter--;			//signal is now going up
				if (Sample_Window_Counter == 0)		//no more min peaks detected in the sampling window interval, go to detect max
				{
					Find_MaxMin_State = 1;			//go to next state
					minCounter++;
//					CH1_ADRES_IR_max = CH1_ADRES_IR_min;
//					CH1_ADRES_Red_max = CH1_ADRES_Red_min;
					CH1_ADRES_IR_max = CH1_ADRES_IR;
					CH1_ADRES_Red_max = CH1_ADRES_Red;
					IR_Min2 = CH1_ADRES_IR_min;
					Red_Min2 = CH1_ADRES_Red_min;

					IR_Min_display = IR_Min2;
					min_found = 1;

					Detection_Done = 1;				//Found Max & Min
				}
			}
			break;
	}
}

/*****************************************************************************
 * Function Name: SpO2_Calculation()
 * Specification: Calculate the %SpO2
 *****************************************************************************/
void SpO2_Calculation (void)
{
	double Ratio_temp;

	IR_Vpp1 = fabs(IR_Max - IR_Min);
	Red_Vpp1 = fabs(Red_Max - Red_Min);
	IR_Vpp2 = fabs(IR_Max2 - IR_Min2);
	Red_Vpp2 = fabs(Red_Max2 - Red_Min2);

	IR_Vpp = (IR_Vpp1 + IR_Vpp2) / 2;
	Red_Vpp = (Red_Vpp1 + Red_Vpp2) / 2;

	IR_Vrms = IR_Vpp / sqrt(8);
	Red_Vrms = Red_Vpp / sqrt(8);

//	SpO2 = log10(Red_Vrms) / log10(IR_Vrms) * 100;
//	if (SpO2 > 100)
//	{
//		SpO2 = 100;
//	}

	// Using lookup table to calculate SpO2
	Ratio = (Red_Vrms/CH0_ADRES_Red) / (IR_Vrms/CH0_ADRES_IR);
//	Ratio = Red_Vrms / IR_Vrms;

//	Ratio = (Red_Vrms*2/Red_Vpp) / (IR_Vrms*2/IR_Vpp);
//	Ratio_temp = (Red_Vrms/CH0_ADRES_Red) / (IR_Vrms/CH0_ADRES_IR);
/*
	if (Ratio_cnt < 2)		// Take 3 Ratio value and everage to calculate SpO2
	{
		Ratio_evg = Ratio_temp + Ratio_evg;
		Ratio_cnt++;
	}
	else
	{
		Ratio = Ratio_evg / 2;
		Ratio_evg = 0;
		Ratio_cnt = 0;
		Ratio_ready = 1;
	}
*/
//	Ratio_ready = 1;

#ifdef NellcorDS100A_7pin

	if (Ratio == 0)
		Nop();
	else if (Ratio < 0.55)	//0.53
		SpO2 = 99;
	else if (Ratio < 0.60)	//0.57
		SpO2 = 98;
	else if (Ratio < 0.65)	//0.61
		SpO2 = 97;
	else if (Ratio < 0.67)	//0.63
		SpO2 = 96;
	else if (Ratio < 0.69)	//0.66
		SpO2 = 95;
	else if (Ratio < 0.70)	//0.69
		SpO2 = 94;
	else if (Ratio < 0.72)
		SpO2 = 93;
	else if (Ratio < 0.75)
		SpO2 = 92;
	else if (Ratio < 0.78)
		SpO2 = 91;
	else if (Ratio < 0.81)
		SpO2 = 90;
	else if (Ratio < 0.84)
		SpO2 = 89;
	else if (Ratio < 0.87)
		SpO2 = 88;
	else if (Ratio < 0.90)
		SpO2 = 87;
	else if (Ratio < 0.92)
		SpO2 = 86;
	else if (Ratio < 0.95)
		SpO2 = 85;
	else if (Ratio < 0.98)
		SpO2 = 84;
	else if (Ratio < 1.00)
		SpO2 = 83;
	else if (Ratio < 1.03)
		SpO2 = 82;
	else if (Ratio < 1.05)
		SpO2 = 81;
	else if (Ratio < 1.08)
		SpO2 = 80;
	else if (Ratio < 1.11)
		SpO2 = 79;
	else if (Ratio < 1.13)
		SpO2 = 78;
	else if (Ratio < 1.16)
		SpO2 = 77;
	else if (Ratio < 1.18)
		SpO2 = 76;
	else if (Ratio < 1.21)
		SpO2 = 75;
	else if (Ratio < 1.24)
		SpO2 = 74;
	else if (Ratio < 1.26)
		SpO2 = 73;
	else if (Ratio < 1.29)
		SpO2 = 72;
	else if (Ratio < 1.31)
		SpO2 = 71;
	else if (Ratio < 1.33)
		SpO2 = 70;
	else if (Ratio < 1.36)
		SpO2 = 69;
	else if (Ratio < 1.38)
		SpO2 = 68;
	else if (Ratio < 1.40)
		SpO2 = 67;
	else if (Ratio < 1.42)
		SpO2 = 66;
	else if (Ratio < 1.45)
		SpO2 = 65;
	else if (Ratio < 1.47)
		SpO2 = 64;
	else if (Ratio < 1.49)
		SpO2 = 63;
	else if (Ratio < 1.51)
		SpO2 = 62;
	else if (Ratio < 1.54)
		SpO2 = 61;
	else if (Ratio < 1.56)
		SpO2 = 60;
	else if (Ratio < 1.58)
		SpO2 = 59;
	else if (Ratio < 1.60)
		SpO2 = 58;
	else if (Ratio < 1.63)
		SpO2 = 57;
	else if (Ratio < 1.66)
		SpO2 = 56;
	else if (Ratio < 1.69)
		SpO2 = 55;
	else if (Ratio < 1.72)
		SpO2 = 54;
	else if (Ratio < 1.75)
		SpO2 = 53;
	else if (Ratio < 1.78)
		SpO2 = 52;
	else if (Ratio < 1.81)
		SpO2 = 51;
	else if (Ratio < 1.84)
		SpO2 = 50;
	else if (Ratio < 1.87)
		SpO2 = 49;
	else if (Ratio < 1.90)
		SpO2 = 48;
	else if (Ratio < 1.94)
		SpO2 = 47;
	else if (Ratio < 1.97)
		SpO2 = 46;
	else if (Ratio < 2.00)
		SpO2 = 45;
	else if (Ratio < 2.04)
		SpO2 = 44;
	else if (Ratio < 2.08)
		SpO2 = 43;
	else if (Ratio < 2.11)
		SpO2 = 42;
	else if (Ratio < 2.15)
		SpO2 = 41;
	else if (Ratio < 2.19)
		SpO2 = 40;
	else if (Ratio < 2.22)
		SpO2 = 39;
	else if (Ratio < 2.27)
		SpO2 = 38;
	else if (Ratio < 2.31)
		SpO2 = 37;
	else if (Ratio < 2.34)
		SpO2 = 36;
	else
		SpO2 = 35;

#elif defined ProSim8_NellcorDS100A_7pin

	if (Ratio == 0)
		Nop();
	else if (Ratio < 0.43)
		SpO2 = 99;
	else if (Ratio < 0.47)
		SpO2 = 98;
	else if (Ratio < 0.53)
		SpO2 = 97;
	else if (Ratio < 0.59)
		SpO2 = 96;
	else if (Ratio < 0.61)
		SpO2 = 95;
	else if (Ratio < 0.63)
		SpO2 = 94;
	else if (Ratio < 0.65)
		SpO2 = 93;
	else if (Ratio < 0.70)
		SpO2 = 92;
	else if (Ratio < 0.74)
		SpO2 = 91;
	else if (Ratio < 0.79)
		SpO2 = 90;
	else if (Ratio < 0.84)
		SpO2 = 89;
	else if (Ratio < 0.87)
		SpO2 = 88;
	else if (Ratio < 0.90)
		SpO2 = 87;
	else if (Ratio < 0.92)
		SpO2 = 86;
	else if (Ratio < 0.95)
		SpO2 = 85;
	else if (Ratio < 0.98)
		SpO2 = 84;
	else if (Ratio < 1.00)
		SpO2 = 83;
	else if (Ratio < 1.03)
		SpO2 = 82;
	else if (Ratio < 1.05)
		SpO2 = 81;
	else if (Ratio < 1.08)
		SpO2 = 80;
	else if (Ratio < 1.11)
		SpO2 = 79;
	else if (Ratio < 1.13)
		SpO2 = 78;
	else if (Ratio < 1.16)
		SpO2 = 77;
	else if (Ratio < 1.18)
		SpO2 = 76;
	else if (Ratio < 1.21)
		SpO2 = 75;
	else if (Ratio < 1.24)
		SpO2 = 74;
	else if (Ratio < 1.26)
		SpO2 = 73;
	else if (Ratio < 1.29)
		SpO2 = 72;
	else if (Ratio < 1.31)
		SpO2 = 71;
	else if (Ratio < 1.33)
		SpO2 = 70;
	else if (Ratio < 1.36)
		SpO2 = 69;
	else if (Ratio < 1.38)
		SpO2 = 68;
	else if (Ratio < 1.40)
		SpO2 = 67;
	else if (Ratio < 1.42)
		SpO2 = 66;
	else if (Ratio < 1.45)
		SpO2 = 65;
	else if (Ratio < 1.47)
		SpO2 = 64;
	else if (Ratio < 1.49)
		SpO2 = 63;
	else if (Ratio < 1.51)
		SpO2 = 62;
	else if (Ratio < 1.54)
		SpO2 = 61;
	else if (Ratio < 1.56)
		SpO2 = 60;
	else if (Ratio < 1.58)
		SpO2 = 59;
	else if (Ratio < 1.60)
		SpO2 = 58;
	else if (Ratio < 1.63)
		SpO2 = 57;
	else if (Ratio < 1.66)
		SpO2 = 56;
	else if (Ratio < 1.69)
		SpO2 = 55;
	else if (Ratio < 1.72)
		SpO2 = 54;
	else if (Ratio < 1.75)
		SpO2 = 53;
	else if (Ratio < 1.78)
		SpO2 = 52;
	else if (Ratio < 1.81)
		SpO2 = 51;
	else if (Ratio < 1.84)
		SpO2 = 50;
	else if (Ratio < 1.87)
		SpO2 = 49;
	else if (Ratio < 1.90)
		SpO2 = 48;
	else if (Ratio < 1.94)
		SpO2 = 47;
	else if (Ratio < 1.97)
		SpO2 = 46;
	else if (Ratio < 2.00)
		SpO2 = 45;
	else if (Ratio < 2.04)
		SpO2 = 44;
	else if (Ratio < 2.08)
		SpO2 = 43;
	else if (Ratio < 2.11)
		SpO2 = 42;
	else if (Ratio < 2.15)
		SpO2 = 41;
	else if (Ratio < 2.19)
		SpO2 = 40;
	else if (Ratio < 2.22)
		SpO2 = 39;
	else if (Ratio < 2.27)
		SpO2 = 38;
	else if (Ratio < 2.31)
		SpO2 = 37;
	else if (Ratio < 2.34)
		SpO2 = 36;
	else
		SpO2 = 35;

#endif
}

/*****************************************************************************
 * Function Name: Pulse_Rate_Calculation()
 * Specification: Calculate the pulse rate (Beats Per Minute, bpm)
 *****************************************************************************/
void Pulse_Rate_Calculation (void)
{
//	Pulse_Rate = 30000 / Samples_Between_Pulses;	//PR(bpm) = 500sps x 60s / (Samples Count)
//	Pulse_Rate = 15000 / Samples_Between_Pulses;	//PR = 250sps x 60s where PWM's period=4ms
//	Pulse_Rate = 7500 / Samples_Between_Pulses;		//PR = 125sps x 60s where PWM's period=8ms

	Pulse_Rate_max = 15000 / Samples_Between_Pulses;
	Pulse_Rate_min = 15000 / Samples_Between_MinPulses;
	Pulse_Rate = (Pulse_Rate_max + Pulse_Rate_min) / 2;

//	Samples_Between_Pulses = 0;		//reset the counter
//	PR_ready = 1;
/*
	if (PR_cnt < 2)		// Take 2 pulse reading and everage to calculate pulse rate
	{
		Pulse_Rate_evg = 30000 / Samples_Between_Pulses + Pulse_Rate_evg;
		Samples_Between_Pulses = 0;
		PR_cnt++;
	}
	else
	{
		Pulse_Rate = Pulse_Rate_evg / 2;
		Pulse_Rate_evg = 0;
		PR_cnt = 0;
		PR_ready = 1;
	}
*/
}

/*****************************************************************************
 * Function Name: U1_write1byte(unsigned char value)
 * Specification: Write one byte to UART serial port
 *****************************************************************************/
void U1_write1byte (unsigned char value)
{
	while (!U1STAbits.TRMT);		//wait until Transmit Shift Register is empty
	U1TXREG = value;				//write 1 byte to serial port
}

/*****************************************************************************
 * Function Name: sendChar2U1(unsigned char ch)
 * Specification: Write one byte to UART serial port
 *****************************************************************************/
void sendChar2U1(unsigned char ch)
{
	while (U1STAbits.UTXBF); // wait when TX buffer full
	U1TXREG = ch;
}

/*****************************************************************************
 * Function Name: send2U1(unsigned char* Uart_Output, unsigned int sz)
 * Specification: Write a string to UART serial port
 *****************************************************************************/
void send2U1(unsigned char* Uart_Output, unsigned int sz)  // str is "\0" terminated string
{
	while (sz) {
		while (U1STAbits.UTXBF);	// wait when TX buffer full
		U1TXREG = *Uart_Output++;
		sz--;
		Nop(); Nop();		
	}
}

/*****************************************************************************
 * Function Name: sendChar2U2(unsigned char ch)
 * Specification: Write one byte to UART serial port
 *****************************************************************************/
void sendChar2U2(unsigned char ch)
{
	while (U2STAbits.UTXBF); // wait when TX buffer full
	U2TXREG = ch;
}

/*****************************************************************************
 * Function Name: send2U2(unsigned char* Uart_Output, unsigned int sz)
 * Specification: Write a string to UART serial port
 *****************************************************************************/
void send2U2(const char *Uart_Output, unsigned char sz)  // str is "\0" terminated string
{
	while (sz) {
		while (U2STAbits.UTXBF);	// wait when TX buffer full
		U2TXREG = *Uart_Output++;
		sz--;
		Nop(); Nop();
	}
}

/*****************************************************************************
 * Function Name: LCD_display(void)
 * Specification: Display SPO2 & Pulse Rate results on LCD
 *****************************************************************************/
void LCD_display (void)
{
	hexdec(SpO2);
	home_clr();
	puts_lcd( (unsigned char*) &mytext2[0], sizeof(mytext2) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);

	hexdec(Pulse_Rate);
	line_2();
	puts_lcd( (unsigned char*) &mytext3[0], sizeof(mytext3) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);
}

/*****************************************************************************
 * Function Name: LCD_display_Ratio(void)
 * Specification: Display Ratio results on LCD for debugging
 *****************************************************************************/
void LCD_display_Ratio (void)
{
	hexdec(SpO2);
	home_clr();
	puts_lcd( (unsigned char*) &mytext2[0], sizeof(mytext2) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);

	Ratio = Ratio * 100;
	hexdec(Ratio);
	line_2();
	puts_lcd( (unsigned char*) &mytext6[0], sizeof(mytext6) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);

	hexdec(Pulse_Rate);
	puts_lcd( (unsigned char*) &mytext9[0], sizeof(mytext9) -1 );
	lcd_data(hunds + 0x30);
	lcd_data(tens + 0x30);
	lcd_data(ones + 0x30);
}

/*****************************************************************************
 * Function Name: unsigned char Calibrate_IR (void)
 * Specification: Calibrate IR LED intensity.
 *****************************************************************************/
unsigned char Calibrate_IR (void)
{
	unsigned char MeterState;

	MeterState = Meter_State;

	if (CH0_ADRES_IR > Baseline_Upper_Limit)
	{
		if (DAC_IRLed_intensity > 0)
		{
			DAC_IRLed_intensity--;
			ThirdByte_A = 0x80 + (DAC_IRLed_intensity>>8 & 0x0f);
			FourthByte_A = DAC_IRLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_A, ThirdByte_A, FourthByte_A);
		}
		IRReady = 0;
	}
	else if (CH0_ADRES_IR < Baseline_Lower_Limit)
	{
		if (DAC_IRLed_intensity < 4095)
		{
			DAC_IRLed_intensity++;
			ThirdByte_A = 0x80 + (DAC_IRLed_intensity>>8 & 0x0f);
			FourthByte_A = DAC_IRLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_A, ThirdByte_A, FourthByte_A);
		}
		IRReady = 0;
	}
	else
	{
		IRReady = 1;
	}

	return(MeterState);
}

/*****************************************************************************
 * Function Name: unsigned char Calibrate_Red (void)
 * Specification: Calibrate Red LED intensity.
 *****************************************************************************/
unsigned char Calibrate_Red (void)
{
	unsigned char MeterState;

	MeterState = Meter_State;

	if (CH0_ADRES_Red > Baseline_Upper_Limit)
	{
		if (DAC_RedLed_intensity > 0)
		{
			DAC_RedLed_intensity--;
			ThirdByte_B = 0x80 + (DAC_RedLed_intensity>>8 & 0x0f);
			FourthByte_B = DAC_RedLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_B, ThirdByte_B, FourthByte_B);
		}
		RedReady = 0;
	}
	else if (CH0_ADRES_Red < Baseline_Lower_Limit)
	{
		if (DAC_RedLed_intensity < 4095)
		{
			DAC_RedLed_intensity++;
			ThirdByte_B = 0x80 + (DAC_RedLed_intensity>>8 & 0x0f);
			FourthByte_B = DAC_RedLed_intensity & 0x00ff;
			I2C_MCP4728_Write(FirstByte, SecondByte_MultiWrite_B, ThirdByte_B, FourthByte_B);
		}
		RedReady = 0;
	}
	else
	{
		RedReady = 1;
	}

	return(MeterState);
}

/*****************************************************************************
  Function:
	void uitoa(WORD Value, BYTE* Buffer)
  Summary:
	Converts an unsigned integer to a decimal string.
  Description:
	Converts a 16-bit unsigned integer to a null-terminated decimal string.
  Precondition:
	None
  Parameters:
	Value	- The number to be converted
	Buffer	- Pointer in which to store the converted string
  Returns:
  	None
  ***************************************************************************/
void uitoa(WORD Value, BYTE* Buffer)
{
	BYTE i;
	WORD Digit;
	WORD Divisor;
	BOOL Printed = FALSE;

	if(Value)
	{
		for(i = 0, Divisor = 10000; i < 5u; i++)
		{
			Digit = Value/Divisor;
			if(Digit || Printed)
			{
				*Buffer++ = '0' + Digit;
				Value -= Digit*Divisor;
				Printed = TRUE;
			}
			Divisor /= 10;
		}
	}
	else
	{
		*Buffer++ = '0';
	}

	*Buffer = '\0';
}

/*****************************************************************************
 * Function Name: void str_Int(uint16_t i)
 * Specification: Converts integer to ASCII.
 *****************************************************************************/
void str_Int(uint16_t i)
{       uint8_t ctr = 0;

        // Decimal Output
        while (i >= 10000)  { i -= 10000; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=  1000)  { i -=  1000; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=   100)  { i -=   100; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=    10)  { i -=    10; ctr++; } U1_write1byte(ctr + 0x30); ctr = 0;
        while (i >=     1)  { i -=     1; ctr++; } U1_write1byte(ctr + 0x30);

         U1_write1byte(0x3b);
}


#ifdef WiFi_Enable
/********************************************************************
 * Function:        void WiFi_TASK(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is to send the data to the WiFi module.
 *
 * Note:            None
 *******************************************************************/
void WiFi_TASK(void)
{
    if (wifi_counter==0 || wifi_counter==wificnts)     //Send result to wifi module on every 5th result
    {
        uitoa(SpO2, SPO2_String);
        uitoa(Pulse_Rate, PR_String);
        /*
        for(i=0;i<4;i++)
        {
                if(SPO2_String[i] == 0x00)
                {
                        SPO2_String[i] = 0x20;
                }
        }
        for(i=0;i<4;i++)
        {
                if(PR_String[i] == 0x00)
                {
                        PR_String[i] = 0x20;
                }
        }
        for (i=0;i<4;i++)
        {
                while(U2STAbits.UTXBF); //TX buffer is full
                U2TXREG = SPO2_String[i];
        }
        //for (delay2=0; delay2<3000; delay2++);
        for (i=0;i<4;i++)
        {
                while(U2STAbits.UTXBF); //TX buffer is full
                U2TXREG = PR_String[i];
        }
        */
        for (sl=0; sl<4; sl++) {
            sendChar2U2(SPO2_String[sl]);
        }
        for (sl=0; sl<4; sl++) {
            sendChar2U2(PR_String[sl]);
        }
    }
    wifi_counter++;
    if (wifi_counter > wificnts)
        wifi_counter = 1;
}
#endif

#ifdef HUB_POX
/********************************************************************
 * Function:        void HUB_POX_TASK(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is to send the data to the HUB.
 *
 * Note:            None
 *******************************************************************/
void HUB_POX_TASK(void)
{
    if( pList.valid ) {

        //bail before for loop if timer has expired
//        if( TimerExpired( & rescanBtle ) ) {
//            return;
//        }

        if( TimerExpired( & connectBtle ) ) {
            int i;
            //step through hub list - usually will only connect to one hub, but if more are available, it will connect with them.
            for( i = 0; i <= pList.highestIndex ; ++i ) {

                //pList can become invalid inside the for loop
                if( pList.valid ) {

                    //find next valid index
                    if( pList.periph[i].type != NONE ) {
                        //Connect and get data
                        if( ConnectToHub( i ) == EXIT_SUCCESS ) {

                            //wait for hub to send notification command ("CUWC")
                            TimerDelay_ms( 300 );
                            SendData( pList.periph[i].type, SpO2, Pulse_Rate );
                            HandleBtleCommand( DISCONNECT );
                            TimerDelay_ms( 200 );
                            ClearRxFifo();
                        } else if( DeleteHubFromList( i ) == EXIT_SUCCESS ) {
                            --i;
                        }
                    } else if( DeleteHubFromList( i ) == EXIT_SUCCESS ) {
                        --i;
                    }
                }
            }
            TimerStart( & connectBtle, TIME_BETWEEN_CONNECTS_ms );
        }
    }

    //bug workaround for RN4020 showing connected after peripheral turns off
//    if( BTLE_CONNECTED ) {
//        HandleBtleCommand( STOP_CONNECTION );
//        TimerDelay_ms( 200 );
//    }
    ClearRxFifo();

    //(re)build list of peripheral MAC addresses and types
    if( TimerExpired( & rescanBtle ) ) {
        pList.highestIndex = -1;
        pList.valid = 0;
        if( FindHub() == EXIT_SUCCESS ) {
        } else {
        }
        TimerStart( & rescanBtle, RESCAN_TIME_ms );
        ClearRxFifo();
    }
}
#endif

#ifdef Android_App
/********************************************************************
 * Function:        void Android_TASK(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is to send the data to the Android App.
 *
 * Note:            None
 *******************************************************************/
void Android_TASK(void)
{
    //********** Iuliana Albu ******************//
    str_Int(0x0f);          //header
    str_Int(SpO2);
    str_Int(Pulse_Rate);
    U1_write1byte(0x0d);	//footer1
    U1_write1byte(0x0a);	//footer2
}
#endif


//=============== Interrupt Service Routine (ISR) ========================================

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)		//Read IR DC & AC signals from AN0 & AN1
{
	int delay;
	unsigned char i;

	Read_ADC_IR = 1;
	CH0_ADRES_IR_sum = 0;
	CH1_ADRES_IR_sum = 0;

	for (delay=0; delay<200; delay++);	//2000=delayed 256us before read ADC, 200, 500

//	LATBbits.LATB14 = 1;			// for debugging

	for (i=0; i<oversampling_number; i++)
	{
		//Acquires IR-DC from Channel0 (AN0)
		AD1CHS0bits.CH0SA = 0x00;		// Select AN0
		AD1CON1bits.SAMP = 1;			// Begin sampling
		while(!AD1CON1bits.DONE);		// Waiting for ADC completed
		AD1CON1bits.DONE = 0;			// Clear conversion done status bit
		CH0_ADRES_IR_sum = CH0_ADRES_IR_sum + ADC1BUF0;		// Read ADC result

		//Acquires IR-AC from Channel1 (AN1)
		AD1CHS0bits.CH0SA = 0x01;		// Select AN1
		AD1CON1bits.SAMP = 1;			// Begin sampling
		while(!AD1CON1bits.DONE);		// Waiting for ADC completed
		AD1CON1bits.DONE = 0;			// Clear conversion done status bit
		CH1_ADRES_IR_sum = CH1_ADRES_IR_sum + ADC1BUF0;		// Read ADC result
	}

	CH0_ADRES_IR = CH0_ADRES_IR_sum / oversampling_number;
	FIR_input_IR[0] = CH1_ADRES_IR_sum / oversampling_number;

//	FIR(1, &FIR_output_IR[0], &FIR_input_IR[0], &BandpassIRFilter);
//	FIR(1, &FIR_output_IR[0], &FIR_input_IR[0], &Bandpass250IRFilter);
//	CH1_ADRES_IR = FIR_output_IR[0];

	Meter_State = Calibrate_IR();

//	LATBbits.LATB14 = 0;			// for debugging

	OC1RS = duty_cycle; 			// Write Duty Cycle value for next PWM cycle
	IFS0bits.T2IF = 0;				// Clear Timer2 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)		//Read Red DC & AC signals from AN0 & AN1
{
	int delay;
	unsigned char i;

	Read_ADC_Red = 1;
	CH0_ADRES_Red_sum = 0;
	CH1_ADRES_Red_sum = 0;

	for (delay=0; delay<200; delay++);	//2000=delayed 256us before read ADC

//	LATBbits.LATB14 = 1;			// for debugging

	for (i=0; i<oversampling_number; i++)
	{
		//Acquires Red-DC from Channel0 (AN0)
		AD1CHS0bits.CH0SA = 0x00;		// Select AN0
		AD1CON1bits.SAMP = 1;			// Begin sampling
		while(!AD1CON1bits.DONE);		// Waiting for ADC completed
		AD1CON1bits.DONE = 0;			// Clear conversion done status bit
		CH0_ADRES_Red_sum = CH0_ADRES_Red_sum + ADC1BUF0;	// Read ADC result

		//Acquires Red-AC from Channel1 (AN1)
		AD1CHS0bits.CH0SA = 0x01;		// Select AN1
		AD1CON1bits.SAMP = 1;			// Begin sampling
		while(!AD1CON1bits.DONE);		// Waiting for ADC completed
		AD1CON1bits.DONE = 0;			// Clear conversion done status bit
		CH1_ADRES_Red_sum = CH1_ADRES_Red_sum + ADC1BUF0;	// Read ADC result
	}

	CH0_ADRES_Red = CH0_ADRES_Red_sum / oversampling_number;
	FIR_input_Red[0] = CH1_ADRES_Red_sum / oversampling_number;

//	FIR(1, &FIR_output_Red[0], &FIR_input_Red[0], &BandpassRedFilter);
//	FIR(1, &FIR_output_Red[0], &FIR_input_Red[0], &Bandpass250RedFilter);
//	CH1_ADRES_Red = FIR_output_Red[0];

#ifdef Sleep_Enabled
	if (CH0_ADRES_Red<=74 && CH1_ADRES_Red>=4000)	//if spo2 probe is not connected, 74=60mV, 4000=3.2V
	{
		goto_sleep = 1;
	}
	else if (CH0_ADRES_Red > Finger_Present_Threshold)	//if no finger present then goto sleep
	{
		goto_sleep = 1;
	}
	else
#endif
	{
//		LATBbits.LATB14 = 0;			// for debugging
		for (delay=0; delay<500; delay++);	//1000=delayed 256us before read ADC
//		LATBbits.LATB14 = 1;			// for debugging

		//Acquires Red-DC baseline from Channel0 (AN0)
		AD1CHS0bits.CH0SA = 0x00;		// Select AN0
		AD1CON1bits.SAMP = 1;			// Begin sampling
		while(!AD1CON1bits.DONE);		// Waiting for ADC completed
		AD1CON1bits.DONE = 0;			// Clear conversion done status bit
		Baseline_ambient = ADC1BUF0;

		Baseline_Upper_Limit = Baseline_ambient + DCVppHigh;
		Baseline_Lower_Limit = Baseline_ambient + DCVppLow;

		Meter_State = Calibrate_Red();
	}

//	LATBbits.LATB14 = 0;			// for debugging

	OC2RS = duty_cycle;				// Write Duty Cycle value for next PWM cycle
	IFS0bits.T3IF = 0;				// Clear Timer3 Interrupt Flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)		//UART1 TX Interrupt
{
	IFS0bits.U1TXIF = 0;			// clear interrupt flag
}

#ifdef HUB_POX
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)		//UART1 RX Interrupt
{
    RxCharUart();
	IFS0bits.U1RXIF = 0;			// clear interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)     	//Timer1 Interrupt
{
    //timer 1 interrupts every 10 ms
    TMR1 = 0;
    systemTime_ms += systemTick_ms;
	IFS0bits.T1IF = 0;			// clear interrupt flag
}
#endif
