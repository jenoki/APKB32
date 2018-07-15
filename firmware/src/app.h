/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "led.h"
#include "bitcount.h"

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef union {
    uint32_t wd;
    struct{
        unsigned char n0:4;
        unsigned char n1:4;
        unsigned char n2:4;
		unsigned char n3:4;
		unsigned char lrot:4;
		unsigned char urot:4;
        unsigned char side:1;
        unsigned char encsw:1;
        unsigned char uu:6;
    } nb;
} KEYSCAN;

typedef struct {
    uint8_t modifier;
    uint8_t keycode;
} KEY_DEFINE;

typedef struct {
    uint8_t index;
    uint8_t movement;
} MOUSE_DEFINE;

typedef struct {
	uint8_t element[4];
} KEY_MODIFIER;

//Moved From Harmony keyboard.h
typedef struct
{
    uint8_t data[5];

}KEYBOARD_INPUT_REPORT;

//Moved From Harmony keyboard.h
typedef union
{
    struct
    {
        unsigned int numLock      :1;
        unsigned int capsLock     :1;
        unsigned int scrollLock   :1;
        unsigned int compose      :1;
        unsigned int kana         :1;
        unsigned int constant     :3;

    }ledState;

    uint8_t data[64];

}KEYBOARD_OUTPUT_REPORT;

//Moved from keyboard.h
typedef enum
{
    /* This is the LED OFF state */
    KEYBOARD_LED_STATE_OFF = 0,

    /* This is the LED ON state */
    KEYBOARD_LED_STATE_ON = 1

}KEYBOARD_LED_STATE;

//Moved from mouse.h
typedef struct
{
	uint8_t data[5];
}
MOUSE_REPORT;

// *****************************************************************************
// *****************************************************************************
// Section: Constant Definitions
// *****************************************************************************
// *****************************************************************************

//APKB definitions
#define USB_PID 0xABBA
#define APKB32_REVISION 0x0110

#define HID_HINSTANCE_KEYBOARD  0
#define HID_DRVIER_INDEX_KEYBOARD   HID_HINSTANCE_KEYBOARD
#define HID_INTERFACE_INDEX_KEYBOARD   HID_HINSTANCE_KEYBOARD

#define HID_HINSTANCE_MOUSE 1
#define HID_DRVIER_INDEX_MOUSE   HID_HINSTANCE_MOUSE
#define HID_INTERFACE_INDEX_MOUSE   HID_HINSTANCE_MOUSE

#define KEY_COUNT 16
#define	SIDE_SW_KEYBOARD	0
#define	SIDE_SW_MOUSE       1

#define ROT_NOT_MOVE        0
#define ROT_MOVE_CW         1
#define ROT_MOVE_PUSH_CW    2
#define ROT_MOVE_CCW        4         
#define ROT_MOVE_PUSH_CCW   8

/* Keyboard bit Layouts for LRKB Board 1.0
B00	B01	B02	B03
B04	B05	B06	B07
B08	B09	B10	B11
B12	B13	B14	B15

B16 SIDE SW
B17 Lower Rotator SW
B18 Uppper Rotator SW
*/
#define MOD_SW_BIT_MASK    0x03ffffff

#define WHEEL_UP	0x01
#define WHEEL_DOWN	0xFF
#define WHEEL_RIGHT	0x01
#define WHEEL_LEFT	0xFF
#define MOUSE_MOVE_LEFT	0xE0
#define MOUSE_MOVE_UP	0xE0
#define MOUSE_MOVE_RIGHT	0x20
#define MOUSE_MOVE_DOWN	0x20

#define MOUSE_BUTTON_LEFT   0x1
#define MOUSE_BUTTON_CENTER 0x2
#define MOUSE_BUTTON_RIGHT  0x4

//Indexes in mouse report
#define MOUSE_RPT_IDX_BTN       0
#define MOUSE_RPT_IDX_MOV_X     1
#define MOUSE_RPT_IDX_MOV_Y     2
#define MOUSE_RPT_IDX_ROTATE    3
#define MOUSE_RPT_IDX_TILT      4
//Indexes in keyboard report
#define KBD_RPT_IDX_MODIFIER    0
#define KBD_RPT_IDX_KEYCODE     2

#ifndef APKB32ENC 
    #define L12MASK 0x0F;
#else
    #define L12MASK 0x09;
#endif
	
//HID key codes HID 1.11
//Not pressed
#define HID_NONE    0x00
//Alphabet
#define HID_a   0x04
#define HID_b   0x05
#define HID_c   0x06
#define HID_d   0x07
#define HID_e   0x08
#define HID_f   0x09
#define HID_g   0x0A
#define HID_h   0x0B
#define HID_i   0x0C
#define HID_j   0x0D
#define HID_k   0x0E
#define HID_l   0x0F
#define HID_m   0x10
#define HID_n   0x11
#define HID_o   0x12
#define HID_p   0x13
#define HID_q   0x14
#define HID_r   0x15
#define HID_s   0x16
#define HID_t   0x17
#define HID_u   0x18
#define HID_v   0x19
#define HID_w   0x1A
#define HID_x   0x1B
#define HID_y   0x1C
#define HID_z   0x1D
//Numeric
#define HID_1   0x1E
#define HID_2   0x1F
#define HID_3   0x20
#define HID_4   0x21
#define HID_5   0x22
#define HID_6   0x23
#define HID_7   0x24
#define HID_8   0x25
#define HID_9   0x26
#define HID_0   0x27
//Control
#define HIDBS   0x2A
#define HIDTAB  0x2B
#define HID_MI  0x2D    //Minus '-'
#define HID_EQ  0x2E    //Equal '='
#define HID_AT  0x2F
#define HID_LA  0x36    //Left Allow '<'
#define HID_CM  0x36    //Comma ','
#define HID_RA  0x37    //Right Allow '>'
#define HID_DT  0x37    //Period '.'
#define HID_QS  0x38
#define HID_UA  0x2E    //Up Arrow '^'
#define HID_RB  0x32    //Right brace ']'
#define HID_LB  0x30    //Left brace '['
#define HID_SL  0x38    //Slash '/'
//Cursor
#define HID_UP      0x52
#define HID_DOWN    0x51
#define HID_LEFT    0x50
#define HID_RIGHT   0x4F
//Function keys
#define HID_F01 0x3A
#define HID_F02 0x3B
#define HID_F03 0x3C
#define HID_F04 0x3D
#define HID_F05 0x3E
#define HID_F06 0x3F
#define HID_F07 0x40
#define HID_F08 0x41
#define HID_F09 0x42
#define HID_F10 0x43
#define HID_F11 0x44
#define HID_F12 0x45
//Misc.
#define HID_PGUP    0x4B
#define HID_PGDOWN  0x4E

//Modifier codes
#define HID_NONM 0x00
#define HID_GUIM 0x08
#define HID_ALTM 0x04
#define HID_SFTM 0x02
#define HID_CTLM 0x01

#define USB_HID_NOEVENT USB_HID_KEYBOARD_KEYPAD_RESERVED_NO_EVENT_INDICATED

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,

	/* Application waits for configuration in this state */
    APP_STATE_WAIT_FOR_CONFIGURATION,

    /* Application checks if an output report is available */
    APP_STATE_CHECK_FOR_OUTPUT_REPORT,

    /* Application checks if it is still configured*/
    APP_STATE_CHECK_IF_CONFIGURED,

    /* Application emulates keyboard */
    APP_STATE_EMULATE_KEYBOARD,

    /* Application error state */
    APP_STATE_ERROR

} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;
    /* Handle to the device layer */
    USB_DEVICE_HANDLE deviceHandle;
    /* Application HID instance */
    USB_DEVICE_HID_INDEX hidInstance;
	/* USB HID current Idle */
	uint8_t idleRate;
	/* Receive transfer handle */
	USB_DEVICE_HID_TRANSFER_HANDLE receiveTransferHandle;
	/* Send transfer handle */
	USB_DEVICE_HID_TRANSFER_HANDLE sendTransferHandle;
	/* USB HID active Protocol */
	USB_HID_PROTOCOL_CODE activeProtocol;
    /* Is device configured */
    bool isConfigured;
	/* Track the send report status */
	bool isReportSentComplete;
	/* Track if a report was received */
	bool isReportReceived;
    /* Tracks switch press*/
    bool isKeyPressed;
	
	// added for APKB
    KEYSCAN     keystat;
    uint8_t     keymouse;
	uint8_t		keyline;
	//(for mouse) MS_APP_DATA members
	/* Is device configured */
	/* HID instance associated with this app object*/
	USB_DEVICE_HID_INDEX hidInstanceMouse;
	/* Transfer handle */
	USB_DEVICE_HID_TRANSFER_HANDLE sendTransferHandleMouse;
	/* USB HID active Protocol */
	uint8_t activeProtocolMouse;
	/* USB HID current Idle */
	uint8_t idleRateMouse;
	/* SET IDLE timer */
	uint16_t setIdleTimerMouse;
	/* Track the send report status */
	bool isReportSentCompleteMouse;
} APP_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks ( void );


#endif /* _APP_H */
/*******************************************************************************
 End of File
 */
