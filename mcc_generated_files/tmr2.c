/**
  TMR2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated driver implementation file for the TMR2 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for TMR2.
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

/**
  Section: Included Files
 */

#include <xc.h>
#include "tmr2.h"
#include "pin_manager.h"
#include "..\HaD_Badge.h"



/**
  Section: TMR2 APIs
 */

void TMR2_Initialize(void) {
    // Set TMR2 to the options selected in the User Interface

    // TMR2ON off; T2OUTPS 1:2; T2CKPS 1:16; 
    T2CON = 0x0A;

    // PR2 128; 
    PR2 = 0x80;

    // TMR2 0x0; 
    TMR2 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR2IF = 0;

    // Enabling TMR2 interrupt.
    PIE1bits.TMR2IE = 1;

    // Start TMR2
    TMR2_StartTimer();
}

void TMR2_StartTimer(void) {
    // Start the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 1;
}

void TMR2_StopTimer(void) {
    // Stop the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 0;
}

uint8_t TMR2_ReadTimer(void) {
    uint8_t readVal;

    readVal = TMR2;

    return readVal;
}

void TMR2_WriteTimer(uint8_t timerVal) {
    // Write to the Timer2 register
    TMR2 = timerVal;
}

void TMR2_LoadPeriodRegister(uint8_t periodVal) {
    PR2 = periodVal;
}



void TMR2_ISR(void) {

    const uint8_t ShiftSeq[]            = {10, 13, 15, 14, 12, 1, 3, 0, 4, 6, 7, 5, 2, 9, 11, 8}; // order 
    static uint16_t BitMask             = 0b10000000;
    static uint8_t KeyUpDebounceCnt     = 0;
    static uint8_t KeyMiddleDebounceCnt = 0;
    static uint8_t KeyLeftDebounceCnt   = 0;
    static uint8_t KeyRightDebounceCnt  = 0;
    
    
    // clear the TMR2 interrupt flag
    PIR1bits.TMR2IF = 0;


    
    // add your TMR2 interrupt custom code
    
    // Clears all anodes 
    // could do writes to whole ports but less simple to rewire
    Anode1_SetHigh();
    Anode2_SetHigh();    
    Anode3_SetHigh();
    Anode4_SetHigh();
    Anode5_SetHigh();
    Anode6_SetHigh();
    Anode7_SetHigh();
    Anode8_SetHigh();    
    
// Executes macro "shift1" total of 16 times in the hardware dependent order 	(112T)

    for (uint8_t i=0; i<16; i++) {    
            // if bit is position set then light then clock high into shift reg
        if ((BitMask && Buffer[ShiftSeq[i]]) == '0') {
            SDO_SetLow();       
        }
        else {
            SDO_SetHigh();
        }
        
        
        CLK_SetHigh();          // create a clock pulse to clock it into shift register
        CLK_SetLow();   
    }

    
    // generates one pulse on "latch" pin 											(2T)
    Latch_SetLow();
    Latch_SetHigh();    //	latch serial shift reg ---> paralel reg

    switch(BitMask) {
        case 0b10000000:
            Anode8_SetLow();
// unsure what flag is used for            Flag2 = 1;
            break;
        case 0b01000000:
            Anode7_SetLow();
            break;
        case 0b00100000:
            Anode6_SetLow();
            break;
        case 0b00010000:
            Anode5_SetLow();
            break;
        case 0b00001000:
            Anode4_SetLow();
            break;            
        case 0b00000100:
            Anode3_SetLow();
            break;              
        case 0b00000010:
            Anode2_SetLow();
            break;   
        case 0b00000001:
            Anode1_SetLow();
            break;  
        default :
            break;
    }
            
// Tests keys in every 8th pass (100 Hz rate)								(3 or 31T)
    if (BitMask == 0b00000001) {
        
        KeyUpPress                      = 0;            // set to no pressed by default
        if(KeyUp_GetValue() == PRESSED)                   // check if button pressed
        {
            KeyUpDebounceCnt++;                         // if pressed then increment count
            if (KeyUpDebounceCnt > BOUNCE_LIMIT){                  // if pressed after define number of tesets
                KeyUpPress              = 1;            // set flag that button has transitioned to pressed
                KeyUpDebounceCnt        = BOUNCE_LIMIT; // lock count to max to prevent rollover    
            }            
        }
        else
        {
            KeyUpDebounceCnt = 0;
        }

        KeyLeftPress                      = 0;            // set to no pressed by default
        if(KeyLeft_GetValue() == PRESSED)                   // check if button pressed
        {
            KeyLeftDebounceCnt++;                         // if pressed then increment count
            if (KeyLeftDebounceCnt > BOUNCE_LIMIT){       // if pressed after define number of tesets
                KeyLeftPress              = 1;            // set flag that button has transitioned to pressed
                KeyLeftDebounceCnt        = BOUNCE_LIMIT; // lock count to max to prevent rollover    
            }            
        }
        else
        {
            KeyLeftDebounceCnt = 0;
        }
        
        
        KeyMiddlePress                      = 0;            // set to no pressed by default
        if(KeyMiddle_GetValue() == PRESSED)                   // check if button pressed
        {
            KeyMiddleDebounceCnt++;                         // if pressed then increment count
            if (KeyMiddleDebounceCnt > BOUNCE_LIMIT){       // if pressed after define number of tesets
                KeyMiddlePress              = 1;            // set flag that button has transitioned to pressed
                KeyMiddleDebounceCnt        = BOUNCE_LIMIT; // lock count to max to prevent rollover    
            }            
        }
        else
        {
            KeyMiddleDebounceCnt = 0;
        }     
        
        KeyRightPress                      = 0;            // set to no pressed by default
        if(KeyRight_GetValue() == PRESSED)                   // check if button pressed
        {
            KeyRightDebounceCnt++;                         // if pressed then increment count
            if (KeyRightDebounceCnt > BOUNCE_LIMIT){       // if pressed after define number of tesets
                KeyRightPress              = 1;            // set flag that button has transitioned to pressed
                KeyRightDebounceCnt        = BOUNCE_LIMIT; // lock count to max to prevent rollover    
            }            
        }
        else
        {
            KeyRightDebounceCnt = 0;
        }          
        
        // check if left & right are pressed and USB is connected
        // this causes bootloader entry
        // no debouncing in this case
        if((KeyLeft_GetValue() == PRESSED) & (KeyRight_GetValue() == PRESSED) & (USBConnected_GetValue() == CONNECTED)){
            Reset();
        }       
        
        BitMask = BitMask >> 1; // walk along bitmask to next bit
        if(BitMask == 0) {
            BitMask = 0b10000000;
        }
        
        
        
    }


												

// unsure function of flag handshake    Flag_handshake = 1;

    
    
    

}

/**
  End of File
 */