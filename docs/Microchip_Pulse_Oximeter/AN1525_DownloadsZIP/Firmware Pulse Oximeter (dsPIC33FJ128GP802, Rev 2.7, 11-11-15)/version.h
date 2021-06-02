/* 
 * File:   version.h
 * Author: Brandon Mitchell
 *
 * Created on February 9, 2015, 3:11 PM
 *
 * Firmware version file
 */

#ifndef VERSION_H
#define	VERSION_H

#ifdef	__cplusplus
extern "C" {
#endif

#define Firmware_Version	"2.7"	// Define Firmware Version

//Only one application can be defined. "HUB_POX" or "Android_App" or "WiFi_Enable".
//If none of above application are defined, U1_waveform will be defined for stand-alone pulse ox application.
//#define HUB_POX         //Uncomment for HUB_POX demo.
//#define Android_App   //Uncomment for Android_App.

#if !defined (HUB_POX) && !defined (Android_App)
#define U1_waveform		//waveform sent to UART1  !!! U1_waveform & HUB_POX share UART1 so only one can be defined !!!
#endif
//#define WiFi_Enable       //WiFi sent to UART2
//#define WiFi_Config

//COMMENT OUT FOR PERIPHERAL COMPILE
//#define HUB_RN4020

#if defined(WiFi_Enable) && defined(WiFi_Config)
	#error Select either WiFi_Enable or WiFi_Config
#endif
#if defined(HUB_POX) && defined(Android_App)
	#error Select either HUB_POX or Android_App
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* VERSION_H */
