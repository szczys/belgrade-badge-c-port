/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB� Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC18F24K50
        Version           :  1.01
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set Anode6 aliases
#define Anode6_TRIS               TRISA0
#define Anode6_LAT                LATA0
#define Anode6_PORT               PORTAbits.RA0
#define Anode6_ANS                ANSA0
#define Anode6_SetHigh()    do { LATA0 = 1; } while(0)
#define Anode6_SetLow()   do { LATA0 = 0; } while(0)
#define Anode6_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define Anode6_GetValue()         PORTAbits.RA0
#define Anode6_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define Anode6_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define Anode6_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define Anode6_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set Anode1 aliases
#define Anode1_TRIS               TRISA1
#define Anode1_LAT                LATA1
#define Anode1_PORT               PORTAbits.RA1
#define Anode1_ANS                ANSA1
#define Anode1_SetHigh()    do { LATA1 = 1; } while(0)
#define Anode1_SetLow()   do { LATA1 = 0; } while(0)
#define Anode1_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define Anode1_GetValue()         PORTAbits.RA1
#define Anode1_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define Anode1_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define Anode1_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define Anode1_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set USBConnected aliases
#define USBConnected_TRIS               TRISA2
#define USBConnected_LAT                LATA2
#define USBConnected_PORT               PORTAbits.RA2
#define USBConnected_ANS                ANSA2
#define USBConnected_SetHigh()    do { LATA2 = 1; } while(0)
#define USBConnected_SetLow()   do { LATA2 = 0; } while(0)
#define USBConnected_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define USBConnected_GetValue()         PORTAbits.RA2
#define USBConnected_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define USBConnected_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define USBConnected_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define USBConnected_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set SDO aliases
#define SDO_TRIS               TRISA3
#define SDO_LAT                LATA3
#define SDO_PORT               PORTAbits.RA3
#define SDO_ANS                ANSA3
#define SDO_SetHigh()    do { LATA3 = 1; } while(0)
#define SDO_SetLow()   do { LATA3 = 0; } while(0)
#define SDO_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define SDO_GetValue()         PORTAbits.RA3
#define SDO_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define SDO_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define SDO_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define SDO_SetDigitalMode()   do { ANSA3 = 0; } while(0)
// get/set LEDtest aliases
#define LEDtest_TRIS               TRISA4
#define LEDtest_LAT                LATA4
#define LEDtest_PORT               PORTAbits.RA4
#define LEDtest_SetHigh()    do { LATA4 = 1; } while(0)
#define LEDtest_SetLow()   do { LATA4 = 0; } while(0)
#define LEDtest_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define LEDtest_GetValue()         PORTAbits.RA4
#define LEDtest_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define LEDtest_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

// get/set Latch aliases
#define Latch_TRIS               TRISA5
#define Latch_LAT                LATA5
#define Latch_PORT               PORTAbits.RA5
#define Latch_ANS                ANSA5
#define Latch_SetHigh()    do { LATA5 = 1; } while(0)
#define Latch_SetLow()   do { LATA5 = 0; } while(0)
#define Latch_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define Latch_GetValue()         PORTAbits.RA5
#define Latch_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define Latch_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define Latch_SetAnalogMode()   do { ANSA5 = 1; } while(0)
#define Latch_SetDigitalMode()   do { ANSA5 = 0; } while(0)
// get/set Anode8 aliases
#define Anode8_TRIS               TRISA6
#define Anode8_LAT                LATA6
#define Anode8_PORT               PORTAbits.RA6
#define Anode8_SetHigh()    do { LATA6 = 1; } while(0)
#define Anode8_SetLow()   do { LATA6 = 0; } while(0)
#define Anode8_Toggle()   do { LATA6 = ~LATA6; } while(0)
#define Anode8_GetValue()         PORTAbits.RA6
#define Anode8_SetDigitalInput()    do { TRISA6 = 1; } while(0)
#define Anode8_SetDigitalOutput()   do { TRISA6 = 0; } while(0)

// get/set Anode7 aliases
#define Anode7_TRIS               TRISA7
#define Anode7_LAT                LATA7
#define Anode7_PORT               PORTAbits.RA7
#define Anode7_SetHigh()    do { LATA7 = 1; } while(0)
#define Anode7_SetLow()   do { LATA7 = 0; } while(0)
#define Anode7_Toggle()   do { LATA7 = ~LATA7; } while(0)
#define Anode7_GetValue()         PORTAbits.RA7
#define Anode7_SetDigitalInput()    do { TRISA7 = 1; } while(0)
#define Anode7_SetDigitalOutput()   do { TRISA7 = 0; } while(0)

// get/set KeyUp aliases
#define KeyUp_TRIS               TRISB0
#define KeyUp_LAT                LATB0
#define KeyUp_PORT               PORTBbits.RB0
#define KeyUp_WPU                WPUB0
#define KeyUp_ANS                ANSB0
#define KeyUp_SetHigh()    do { LATB0 = 1; } while(0)
#define KeyUp_SetLow()   do { LATB0 = 0; } while(0)
#define KeyUp_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define KeyUp_GetValue()         PORTBbits.RB0
#define KeyUp_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define KeyUp_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define KeyUp_SetPullup()    do { WPUB0 = 1; } while(0)
#define KeyUp_ResetPullup()   do { WPUB0 = 0; } while(0)
#define KeyUp_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define KeyUp_SetDigitalMode()   do { ANSB0 = 0; } while(0)
// get/set KeyRight aliases
#define KeyRight_TRIS               TRISB1
#define KeyRight_LAT                LATB1
#define KeyRight_PORT               PORTBbits.RB1
#define KeyRight_WPU                WPUB1
#define KeyRight_ANS                ANSB1
#define KeyRight_SetHigh()    do { LATB1 = 1; } while(0)
#define KeyRight_SetLow()   do { LATB1 = 0; } while(0)
#define KeyRight_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define KeyRight_GetValue()         PORTBbits.RB1
#define KeyRight_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define KeyRight_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define KeyRight_SetPullup()    do { WPUB1 = 1; } while(0)
#define KeyRight_ResetPullup()   do { WPUB1 = 0; } while(0)
#define KeyRight_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define KeyRight_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set KeyMiddle aliases
#define KeyMiddle_TRIS               TRISB2
#define KeyMiddle_LAT                LATB2
#define KeyMiddle_PORT               PORTBbits.RB2
#define KeyMiddle_WPU                WPUB2
#define KeyMiddle_ANS                ANSB2
#define KeyMiddle_SetHigh()    do { LATB2 = 1; } while(0)
#define KeyMiddle_SetLow()   do { LATB2 = 0; } while(0)
#define KeyMiddle_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define KeyMiddle_GetValue()         PORTBbits.RB2
#define KeyMiddle_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define KeyMiddle_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define KeyMiddle_SetPullup()    do { WPUB2 = 1; } while(0)
#define KeyMiddle_ResetPullup()   do { WPUB2 = 0; } while(0)
#define KeyMiddle_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define KeyMiddle_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set KeyLeft aliases
#define KeyLeft_TRIS               TRISB3
#define KeyLeft_LAT                LATB3
#define KeyLeft_PORT               PORTBbits.RB3
#define KeyLeft_WPU                WPUB3
#define KeyLeft_ANS                ANSB3
#define KeyLeft_SetHigh()    do { LATB3 = 1; } while(0)
#define KeyLeft_SetLow()   do { LATB3 = 0; } while(0)
#define KeyLeft_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define KeyLeft_GetValue()         PORTBbits.RB3
#define KeyLeft_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define KeyLeft_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define KeyLeft_SetPullup()    do { WPUB3 = 1; } while(0)
#define KeyLeft_ResetPullup()   do { WPUB3 = 0; } while(0)
#define KeyLeft_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define KeyLeft_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set ShiftRegPwr aliases
#define ShiftRegPwr_TRIS               TRISB4
#define ShiftRegPwr_LAT                LATB4
#define ShiftRegPwr_PORT               PORTBbits.RB4
#define ShiftRegPwr_WPU                WPUB4
#define ShiftRegPwr_ANS                ANSB4
#define ShiftRegPwr_SetHigh()    do { LATB4 = 1; } while(0)
#define ShiftRegPwr_SetLow()   do { LATB4 = 0; } while(0)
#define ShiftRegPwr_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define ShiftRegPwr_GetValue()         PORTBbits.RB4
#define ShiftRegPwr_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define ShiftRegPwr_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define ShiftRegPwr_SetPullup()    do { WPUB4 = 1; } while(0)
#define ShiftRegPwr_ResetPullup()   do { WPUB4 = 0; } while(0)
#define ShiftRegPwr_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define ShiftRegPwr_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set CLK aliases
#define CLK_TRIS               TRISB5
#define CLK_LAT                LATB5
#define CLK_PORT               PORTBbits.RB5
#define CLK_WPU                WPUB5
#define CLK_ANS                ANSB5
#define CLK_SetHigh()    do { LATB5 = 1; } while(0)
#define CLK_SetLow()   do { LATB5 = 0; } while(0)
#define CLK_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define CLK_GetValue()         PORTBbits.RB5
#define CLK_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define CLK_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define CLK_SetPullup()    do { WPUB5 = 1; } while(0)
#define CLK_ResetPullup()   do { WPUB5 = 0; } while(0)
#define CLK_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define CLK_SetDigitalMode()   do { ANSB5 = 0; } while(0)
// get/set OE aliases
#define OE_TRIS               TRISB7
#define OE_LAT                LATB7
#define OE_PORT               PORTBbits.RB7
#define OE_WPU                WPUB7
#define OE_SetHigh()    do { LATB7 = 1; } while(0)
#define OE_SetLow()   do { LATB7 = 0; } while(0)
#define OE_Toggle()   do { LATB7 = ~LATB7; } while(0)
#define OE_GetValue()         PORTBbits.RB7
#define OE_SetDigitalInput()    do { TRISB7 = 1; } while(0)
#define OE_SetDigitalOutput()   do { TRISB7 = 0; } while(0)

#define OE_SetPullup()    do { WPUB7 = 1; } while(0)
#define OE_ResetPullup()   do { WPUB7 = 0; } while(0)
// get/set Anode3 aliases
#define Anode3_TRIS               TRISC0
#define Anode3_LAT                LATC0
#define Anode3_PORT               PORTCbits.RC0
#define Anode3_SetHigh()    do { LATC0 = 1; } while(0)
#define Anode3_SetLow()   do { LATC0 = 0; } while(0)
#define Anode3_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define Anode3_GetValue()         PORTCbits.RC0
#define Anode3_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define Anode3_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

// get/set Anode2 aliases
#define Anode2_TRIS               TRISC1
#define Anode2_LAT                LATC1
#define Anode2_PORT               PORTCbits.RC1
#define Anode2_SetHigh()    do { LATC1 = 1; } while(0)
#define Anode2_SetLow()   do { LATC1 = 0; } while(0)
#define Anode2_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define Anode2_GetValue()         PORTCbits.RC1
#define Anode2_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define Anode2_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

// get/set IRLed aliases
#define IRLed_TRIS               TRISC2
#define IRLed_LAT                LATC2
#define IRLed_PORT               PORTCbits.RC2
#define IRLed_ANS                ANSC2
#define IRLed_SetHigh()    do { LATC2 = 1; } while(0)
#define IRLed_SetLow()   do { LATC2 = 0; } while(0)
#define IRLed_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IRLed_GetValue()         PORTCbits.RC2
#define IRLed_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IRLed_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define IRLed_SetAnalogMode()   do { ANSC2 = 1; } while(0)
#define IRLed_SetDigitalMode()   do { ANSC2 = 0; } while(0)
// get/set Anode5 aliases
#define Anode5_TRIS               TRISC6
#define Anode5_LAT                LATC6
#define Anode5_PORT               PORTCbits.RC6
#define Anode5_ANS                ANSC6
#define Anode5_SetHigh()    do { LATC6 = 1; } while(0)
#define Anode5_SetLow()   do { LATC6 = 0; } while(0)
#define Anode5_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define Anode5_GetValue()         PORTCbits.RC6
#define Anode5_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define Anode5_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

#define Anode5_SetAnalogMode()   do { ANSC6 = 1; } while(0)
#define Anode5_SetDigitalMode()   do { ANSC6 = 0; } while(0)
// get/set Anode4 aliases
#define Anode4_TRIS               TRISC7
#define Anode4_LAT                LATC7
#define Anode4_PORT               PORTCbits.RC7
#define Anode4_ANS                ANSC7
#define Anode4_SetHigh()    do { LATC7 = 1; } while(0)
#define Anode4_SetLow()   do { LATC7 = 0; } while(0)
#define Anode4_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define Anode4_GetValue()         PORTCbits.RC7
#define Anode4_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define Anode4_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define Anode4_SetAnalogMode()   do { ANSC7 = 1; } while(0)
#define Anode4_SetDigitalMode()   do { ANSC7 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */