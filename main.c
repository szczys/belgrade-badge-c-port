/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC18F24K50
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
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

#include "mcc_generated_files/mcc.h"
#include "HaD_Badge.h"
#include <stdint.h>

uint8_t KeyUpPress;
uint8_t KeyMiddlePress;
uint8_t KeyLeftPress;
uint8_t KeyRightPress;

uint8_t Buffer[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


void delayMS(uint16_t milliseconds) {
    milliseconds = milliseconds/10;
    while (milliseconds > 0) {
        __delay_ms(10);
        --milliseconds;
    }
}

void POST(void) {
    //Test all anode drivers
    LEDtestlat &= ~LEDtestbit;  //Sink to test LEDs
    Anode0on();
    delayMS(500);
    Anode0off();
    Anode1on();
    delayMS(500);
    Anode1off();
    Anode2on();
    delayMS(500);
    Anode2off();
    Anode3on();
    delayMS(500);
    Anode3off();
    Anode4on();
    delayMS(500);
    Anode4off();
    Anode5on();
    delayMS(500);
    Anode5off();
    Anode6on();
    delayMS(500);
    Anode6off();
    Anode7on();
    delayMS(500);
    Anode7off();
    LEDtestlat |= LEDtestbit;
    //End Anode driver test
    
    
}
/*
                         Main application
 */
void main(void) {
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    // INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    // INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    //POST();
    
    for (uint8_t i=0; i<8; i++) {
        Buffer[i] = 0xFF;
    }
    PeripheralsOn();
    delayMS(100);

    /*
    while(1) {
        static uint8_t BitMask = 0b10000000;
        
        Anode0off();
        Anode1off();
        Anode2off();
        Anode3off();
        Anode4off();
        Anode5off();
        Anode6off();
        Anode7off();

        LatchLow();
        for (uint8_t i=0; i<16; i++) {
            if (Buffer[ShiftSeq[i]] & BitMask) { DataHigh(); }
            else { DataLow(); }


            ClkHigh();
            //ClkDelay();
            ClkLow();
            //delayMS(100);
            //ClkStrobe();
        }

        LatchHigh();
        switch (BitMask) {
            case(0b10000000):
                Anode0on();
                break;
            case(0b01000000):
                Anode1on();
                break;
            case(0b00100000):
                Anode2on();
                break;
            case(0b00010000):
                Anode3on();
                break;
            case(0b00001000):
                Anode4on();
                break;
            case(0b00000100):
                Anode5on();
                break;
            case(0b00000010):
                Anode6on();
                break;
            case(0b00000001):
                Anode7on();
                break;
        }
        
        BitMask >>= 1;
        if (BitMask == 0) { BitMask = 0b10000000; }
        //LATB &= ~(1<<7);
        delayMS(5);
        //LEDtestlat &= ~LEDtestbit;  //Sink to test LEDs
    }
    */
    while (1) {
                // Add your application code
        if (KeyUpPress == TRUE) {
            Buffer[8] = 0b10000000; // light top mid led
        }
        if (KeyMiddlePress == TRUE) {
            Buffer[7] = 0b00000001; // light bottom mid led
        }
        if (KeyLeftPress == TRUE) {
            Buffer[0] = 0b00000001; // light bottom left led
        }
        if (KeyRightPress == TRUE) {
            Buffer[15] = 0b00000001; // light bottom right led
        }     
    }
}
/**
 End of File
 */