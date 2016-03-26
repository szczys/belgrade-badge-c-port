/**
  @Generated MPLAB® Code Configurator Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    HaD_Badge.h

  @Summary:
    This is the header file with defines and globals for HaD european sueprconference 2016 badge

  
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#include <stdint.h>

extern uint8_t KeyUpPress;
extern uint8_t KeyMiddlePress;
extern uint8_t KeyLeftPress;
extern uint8_t KeyRightPress;

extern uint8_t Buffer[16];

/**** Pin Definitions ********************/
#define LEDtestlat      LATC
#define LEDtestbit      (1<<2)

#define PeriphPwrlat    LATB
#define PeriphPwrbit    (1<<5)
#define ShiftDatalat    LATB
#define ShiftDatabit    (1<<3)
#define ShiftClklat     LATB
#define ShiftClkbit     (1<<1)
#define ShiftLatchlat   LATB
#define ShiftLatchbit   (1<<1)

#define Anode0lat       LATA
#define Anode0bit       (1<<0)
#define Anode1lat       LATA
#define Anode1bit       (1<<1)
#define Anode2lat       LATA
#define Anode2bit       (1<<2)
#define Anode3lat       LATA
#define Anode3bit       (1<<3)
#define Anode4lat       LATA
#define Anode4bit       (1<<4)
#define Anode5lat       LATA
#define Anode5bit       (1<<5)
#define Anode6lat       LATA
#define Anode6bit       (1<<6)
#define Anode7lat       LATA
#define Anode7bit       (1<<7)
/**** End Pin Definitions ****************/


#define TRUE 1
#define FALSE 0

// button bounce duration when in increments of 100ms
#define BOUNCE_LIMIT 4

// hardware specific to match logic for buttons on badge
#define PRESSED 0
#define UNPRESSED 1

// VUSB definitions when checking if USB power is there
#define CONNECTED 1
#define DISCONNECTED 0

// Peripheral power control
#define PeripheralsOn()     PeriphPwrlat &= ~PeriphPwrbit
#define PeripheralsOff()    PeriphPwrlat |= PeriphPwrbit

// Shift register control
#define ClkDelay()          __delay_ms(4)   //Needed for driver to see clk strobe
#define DataHigh()          ShiftDatalat |= ShiftDatabit
#define DataLow()           ShiftDatalat &= ~ShiftDatabit
#define ClkHigh()           do { LATB5 = 1; } while(0) //ShiftClklat |= ShiftClkbit
#define ClkLow()            do { LATB5 = 0; } while(0) //ShiftClklat &= ~ShiftClkbit
#define ClkStrobe()         { ClkLow(); ClkDelay(); ClkHigh(); }

/**** Anode control function macros ***********/
#define Anode0on()      Anode0lat &= ~(Anode0bit)
#define Anode0off()     Anode0lat |= Anode0bit
#define Anode1on()      Anode1lat &= ~(Anode1bit)
#define Anode1off()     Anode1lat |= Anode1bit
#define Anode2on()      Anode2lat &= ~(Anode2bit)
#define Anode2off()     Anode2lat |= Anode2bit
#define Anode3on()      Anode3lat &= ~(Anode3bit)
#define Anode3off()     Anode3lat |= Anode3bit
#define Anode4on()      Anode4lat &= ~(Anode4bit)
#define Anode4off()     Anode4lat |= Anode4bit
#define Anode5on()      Anode5lat &= ~(Anode5bit)
#define Anode5off()     Anode5lat |= Anode5bit
#define Anode6on()      Anode6lat &= ~(Anode6bit)
#define Anode6off()     Anode6lat |= Anode6bit
#define Anode7on()      Anode7lat &= ~(Anode7bit)
#define Anode7off()     Anode7lat |= Anode7bit
/**** End anode control functions macros*******/

#define TOTPIXELX       8
#define TOTPIXELY       16

//LED Control definitions
#define OFF     0x00
#define ON      0x0F

//Directions of travel
#define UP      0
#define DOWN    1
#define LEFT    2
#define RIGHT   3
//Miscellaneous
#define ESCAPE  4
#define NOINPUT 5
#define BUTTON  6

//Time tracking global
uint32_t ticks;

//button definitions
#define KEY_PIN		PORTB
#define KEY0		3	// Left
#define KEY1		1	// Right
#define KEY2		0	// Up
#define KEY3		2   // Down

//Debounce
#define REPEAT_MASK   (1<<KEY0 | 1<<KEY1 | 1<<KEY2 | 1<<KEY3)   // repeat: key1, key2 
#define REPEAT_START   50      // after 500ms 
#define REPEAT_NEXT   20      // every 200ms
volatile unsigned char key_press;
volatile unsigned char key_state;
volatile unsigned char key_rpt;


/*---- Display Prototypes ----*/
void initDisplay(void);             //Turn on display and set all LEDs off
void displayClear(void);            //Turn all LEDs off
void displayPixel(uint8_t x, uint8_t y, uint8_t state); //Set LED to state (ON|OFF)
void displayClose(void);            //Close the display (used for SDL2 emulator window)
void displayLatch(void);            //Make display changes visible (can be used for a framebuffer)
/*--------------------*/



/*---- Control Prototypes ----*/
void initControl(void);             //Setup button input
uint8_t getControl(void);           //Return last pressed button
void initTime(void);                //Initialize timekeeping hardware
uint32_t getTime(void);             //Return milliseconds (upcounting)
void controlDelayMs(uint16_t ms);   //Delay milliseconds (blocking)