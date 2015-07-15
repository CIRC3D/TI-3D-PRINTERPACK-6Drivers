#ifndef PINS_h
#define PINS_h

#include <Energia.h>

#define ALARM_PIN          -1
/* Modified by CIRC3D for Texas Instruments/UTD
        Original project is Tonokip, repository: https://github.com/tonokip/Tonokip-Firmware
	Works with Energia for MSP430F5529 now, not Arduino
	June 2015
        This code is licensed as GPLv2
   For use with the CIRC3D BoosterPack 
*/ // Pinning below changed as per schematic...
/****************************************************************************************
* Energia MSP-EXP430F5529LP pin assignment V.1.3
*         	 J1-J3                              J4-J1					
*     VCC  | 1  21 |  5V             HEATER0  | 40 20 |  GND
*  THERM0  | 2  22 |  GND              E1-EN  | 39 19 |  X-STEP
* HEATER2  | 3  23 |  Z-STEP             LED  | 38 18 |  X-DIR
* E2-STEP  | 4  24 |  THERM2            Z-MIN  | 37 17 |  SLEEP
* HEATER1  | 5  25 |  Z-EN PWM+          1.3  | 36 16 |  RST
*  THERM1  | 6  26 |  E0-STEP          Y-MIN  | 35 15 |  SIMO
*    SCLK  | 7  27 |  E0-DIR            X-EN  | 34 14 |  SOMI
*     2.7  | 8  28 |  E0-EN           Y-STEP  | 33 13 |  Z-DIR
*  E2-DIR  | 9  29 |  Y-DIR          E1-STEP  | 32 12 |  X-MIN
*   E2-EN  | 10 30 |  Y-EN            E1-DIR  | 31 11 |  SD_CS
*           +---+                              +----+
****************************************************************************************/
#if MOTHERBOARD == 10
#define RAMPS_V_1_4		//was _3
#endif
#if MOTHERBOARD == 10	//was 3
#define KNOWN_BOARD 1

//////////////////FIX THIS//////////////
/*#ifndef __LaunchPad_w/_msp430f5529_(16MHz)__ 		//was __AVR_ATmega1280__
 #ifndef __LaunchPad_w/_msp430f5529_(16MHz)__		//was __AVR_ATmega2560__
 #error Oops!  Make sure you have 'LaunchPad w/ msp430f5529' selected from the 'Tools -> Boards' menu.
 #endif
#endif*/

// uncomment one of the following lines for RAMPS v1.3 or v1.0, comment both for v1.2 or 1.1
// #define RAMPS_V_1_3
// #define RAMPS_V_1_0

#ifdef RAMPS_V_1_4

#define X_STEP_PIN         19
#define X_DIR_PIN          18
#define X_ENABLE_PIN       34
#define X_MIN_PIN          12
#define X_MAX_PIN          -1 //Max endstops default to disabled "-1", set to commented value to enable.

#define Y_STEP_PIN         33
#define Y_DIR_PIN          29
#define Y_ENABLE_PIN       30
#define Y_MIN_PIN          35
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         23
#define Z_DIR_PIN          13 //Updated to move Therm2 to analog side
#define Z_ENABLE_PIN       25
#define Z_MIN_PIN          37
#define Z_MAX_PIN          -1

#define E_STEP_PIN         26//E0
#define E_DIR_PIN          27//E0
#define E_ENABLE_PIN       28//E0

#define E_1_STEP_PIN       -1//E0
#define E_1_DIR_PIN        -1//E0
#define E_1_ENABLE_PIN     -1//E0

#define Q_STEP_PIN         32//E1
#define Q_DIR_PIN          31//E1
#define Q_ENABLE_PIN       39//E1

#define Q_1_STEP_PIN       -1//E1
#define Q_1_DIR_PIN        -1//E1
#define Q_1_ENABLE_PIN     -1//E1

#define B_STEP_PIN         4//E2
#define B_DIR_PIN          9//E2
#define B_ENABLE_PIN       10//E2

#define B_1_STEP_PIN       -1//E2
#define B_1_DIR_PIN        -1//E2
#define B_1_ENABLE_PIN     -1//E2

#define SDPOWER            -1
#define SDSS               11//SD_CS
#define SS_PIN             11//Same definition
#define LED_PIN            38
#define FAN_PIN            -1
#define PS_ON_PIN          -1
#define KILL_PIN           -1//-1 Pending
#define ALARM_PIN          -1//-1

#define HEATER_0_PIN       40
#define HEATER_1_PIN       5
#define HEATER_2_PIN       3
#define TEMP_0_PIN         2
#define TEMP_1_PIN         6
#define TEMP_2_PIN         24 // was 13
#endif


// SPI for Max6675 Thermocouple

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define SCK_PIN          7//SCLK
  #define MISO_PIN         14//SOMI
  #define MOSI_PIN         15//SIMO
  #define MAX6675_SS       -1
#else
  #define MAX6675_SS       -1
#endif

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif


//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
const int sensitive_pins[] = {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN,
                              Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN,
                              Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN,
                              E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN,
                              Q_STEP_PIN, Q_DIR_PIN, Q_ENABLE_PIN,
                              B_STEP_PIN, B_DIR_PIN, B_ENABLE_PIN,
                              LED_PIN, PS_ON_PIN, HEATER_0_PIN, HEATER_1_PIN, HEATER_2_PIN, TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN};

#endif


#endif
