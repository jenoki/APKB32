/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

uint32_t previous_scan = 0;
uint32_t previous_key = 0;
uint32_t current_key = 0;
KEYSCAN current_scan;


uint8_t     lenc_prev;
bool        LRotSwitch = false;

extern APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
 
void __ISR(_CHANGE_NOTICE_VECTOR, ipl1AUTO) _IntHandlerChangeNotification(void)
{
    /* TODO: Add code to process interrupt here */
    uint8_t r;
    // Lower(Main) Rotator
    r = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_A) & 0x03;

    if (r == 0) {
        if(lenc_prev == 2){ //CW
            appData.keystat.nb.lrot = (LRotSwitch)? ROT_MOVE_PUSH_CW:ROT_MOVE_CW;
            appData.isKeyPressed = true;
        }else if(lenc_prev == 1){ //CCW
            appData.keystat.nb.lrot = (LRotSwitch)? ROT_MOVE_PUSH_CCW:ROT_MOVE_CCW;
            appData.isKeyPressed = true;
        }       
    }else if(r == 3){
		appData.keystat.nb.lrot = ROT_NOT_MOVE;
		appData.isKeyPressed = true;
    }
    lenc_prev = r;

    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_CHANGE_NOTICE_A);
}    
 


void __ISR(_TIMER_1_VECTOR, ipl1AUTO) IntHandlerDrvTmrInstance0(void)
{
    uint8_t r;
    //keyboad matrix
    switch(appData.keyline){
        case 0:
            r=PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_C) & 0x0F; 
            current_scan.nb.n0 = (r);          
            appData.keyline++;
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_4,0);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_5,1);
            break;
        case 1:
            r=PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_C) & L12MASK;
            current_scan.nb.n1 = (r);
            appData.keyline++;
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_5,0);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_6,1);
            break;
        case 2:
            r=PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_C) & L12MASK;
            current_scan.nb.n2 = (r);
            appData.keyline++;
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_6,0);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_7,1);
            break;
        case 3:
            r=PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_C) & 0x0F; 
            current_scan.nb.n3 = (r);
            r=PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_B,PORTS_BIT_POS_15);
            current_scan.nb.side = (r)? 0x0:0x1;//side sw
            r=PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_B,PORTS_BIT_POS_2);
            current_scan.nb.encsw = (r)? 0x0:0x1;//encoder sw

            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_7,0);
            PLIB_PORTS_PinWrite(PORTS_ID_0,PORT_CHANNEL_C,PORTS_BIT_POS_4,1);
            appData.keyline = 0;

            //Key detection
            if (previous_scan==current_scan.wd){
                current_key = current_scan.wd;
                uint32_t dif = current_key ^ previous_key;
                appData.keystat.wd = current_key;
                if (dif != 0){
                    previous_key = current_key;
                    appData.isKeyPressed = true;
                }
        	}
            previous_scan = current_scan.wd;
            break;
	}
     
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_1);
}            
void __ISR(_TIMER_2_VECTOR, ipl0AUTO) IntHandlerDrvTmrInstance1(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
	
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

/*******************************************************************************
 End of File
*/
