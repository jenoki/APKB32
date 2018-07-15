/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits, 
    and allocates any necessary global system resources, such as the 
    sysObj structure that contains the object handles to all the MPLAB Harmony 
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config PWP =        OFF
#pragma config BWP =        OFF
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      PRIPLL
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FPBDIV =     DIV_8
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     OFF
#pragma config FWDTWINSZ =  WINSZ_50
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_4
#pragma config FPLLMUL =    MUL_20
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLIDIV =   DIV_4
#pragma config UPLLEN =     ON
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON
#pragma config FVBUSONIO =  ON
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USB Initialization Data">
/******************************************************
 * USB Driver Initialization
 ******************************************************/
/****************************************************
 * Endpoint Table needed by the Device Layer.
 ****************************************************/
uint8_t __attribute__((aligned(512))) endPointTable[DRV_USBFS_ENDPOINTS_NUMBER * 32];
const DRV_USBFS_INIT drvUSBFSInit =
{
    /* Assign the endpoint table */
    .endpointTable= endPointTable,

    /* Interrupt Source for USB module */
    .interruptSource = INT_SOURCE_USB_1,
    
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Operation Mode */
    .operationMode = DRV_USBFS_OPMODE_DEVICE,
    
    .operationSpeed = USB_SPEED_FULL,
    
    /* Stop in idle */
    .stopInIdle = false,

    /* Suspend in sleep */
    .suspendInSleep = false,

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USB_ID_1,
    
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="USB Stack Initialization Data">


/****************************************************
 * Class specific descriptor - HID Report descriptor
 ****************************************************/
const uint8_t hid_rpt0[] =
{
    0x05, 0x01,	// USAGE_PAGE (Generic Desktop)
	0x09, 0x06,	// USAGE (Keyboard)
	0xa1, 0x01,	// COLLECTION (Application)
	0x05, 0x07,	//   USAGE_PAGE (Keyboard)
	0x19, 0xe0,	//   USAGE_MINIMUM (Keyboard LeftControl)
	0x29, 0xe7,	//   USAGE_MAXIMUM (Keyboard Right GUI)
	0x15, 0x00,	//   LOGICAL_MINIMUM (0)
	0x25, 0x01,	//   LOGICAL_MAXIMUM (1)
	0x75, 0x01,	//   REPORT_SIZE (1)
	0x95, 0x08,	//   REPORT_COUNT (8)
	0x81, 0x02,	//   INPUT (Data,Var,Abs)
	0x95, 0x01,	//   REPORT_COUNT (1)
	0x75, 0x08,	//   REPORT_SIZE (8)
	0x81, 0x01,	//   INPUT (Cnst,Var,Abs)
	0x95, 0x05,	//   REPORT_COUNT (5)
	0x75, 0x01,	//   REPORT_SIZE (1)
	0x05, 0x08,	//   USAGE_PAGE (LEDs)
	0x19, 0x01,	//   USAGE_MINIMUM (Num Lock)
	0x29, 0x05,	//   USAGE_MAXIMUM (Kana)
	0x91, 0x02,	//   OUTPUT (Data,Var,Abs)
	0x95, 0x01,	//   REPORT_COUNT (1)
	0x75, 0x03,	//   REPORT_SIZE (3)
	0x91, 0x01,	//   OUTPUT (Cnst,Var,Abs)
	0x95, 0x03,	//   REPORT_COUNT (6)
	0x75, 0x08,	//   REPORT_SIZE (8)
	0x15, 0x00,	//   LOGICAL_MINIMUM (0)
	0x25, 0x65,	//   LOGICAL_MAXIMUM (101)
	0x05, 0x07,	//   USAGE_PAGE (Keyboard)
	0x19, 0x00,	//   USAGE_MINIMUM (Reserved (no event indicated))
	0x29, 0x65,	//   USAGE_MAXIMUM (Keyboard Application)
	0x81, 0x00,	//   INPUT (Data,Ary,Abs)
	0xc0        // End Collection
};

const uint8_t hid_rpt1[] =	//for mouse
{
    0x05, 0x01, /*	Usage Page (Generic Desktop)	*/
//Generic Mouse with Virtcal Wheel
	0x09, 0x02, /*	Usage (Mouse)			*/
	0xA1, 0x01, /*	Collection (Application)	*/
	0x09, 0x01, /*	Usage (Pointer)			*/
	0xA1, 0x00, /*	Collection (Physical)		*/
	0x05, 0x09, /*	Usage Page (Buttons)		*/
	0x19, 0x01, /*	Usage Minimum (01)		*/
	0x29, 0x05, /*	Usage Maximum (03)		*/
	0x15, 0x00, /*	Logical Minimum (0)		*/
	0x25, 0x01, /*	Logical Maximum (1)		*/
	0x95, 0x05, /*	Report Count (3)		*/
	0x75, 0x01, /*	Report Size (1)			*/
	0x81, 0x02, /*	Input (Data, Variable, Absolute)*/
	0x95, 0x01, /*	Report Count (1)		*/
	0x75, 0x03, /*	Report Size (5)			*/
	0x81, 0x01, /*	Input (Constant);5 bit padding	*/
	0x05, 0x01, /*	Usage Page (Generic Desktop)	*/
	0x09, 0x30, /*	Usage (X)			*/
	0x09, 0x31, /*	Usage (Y)			*/
	0x09, 0x38, /*	Usage (Wheel)			*/
	0x15, 0x81, /*	Logical Minimum (-127)		*/
	0x25, 0x7F, /*	Logical Maximum (127)           */
	0x75, 0x08, /*	Report Size (8)			*/
	0x95, 0x03, /*	Report Count (4)		*/
	0x81, 0x06, /*	Input (Data, Variable, Relative)*/
//Additional Horizontal Wheel
	0x05,0x0c,	//	usage-page,g,1,"consumer devices"
	0x0a,0x38,0x02,	//	usage,l,2b,"AC Pan"
	0x15,0x81,	//	logical-minimum,g,1b,0x81
	0x25,0x7f,	//	logical-maximum,g,1b,0x7f
	0x75,0x08,	//	report-size,g,1b,"8bit"
	0x95,0x01,	//	repoer-count,g,1b,1
	0x81,0x06,	//	input,m,1b,"valiable|relative"
	0xC0,	/*End of collection*/
	0xC0		/*End of collection*/
};
/**************************************************
 * USB Device Function Driver Init Data
 **************************************************/
    const USB_DEVICE_HID_INIT hidInit0 =
    {
        .hidReportDescriptorSize = sizeof(hid_rpt0),
        .hidReportDescriptor = &hid_rpt0,
        .queueSizeReportReceive = 1,
        .queueSizeReportSend = 1
    };
    const USB_DEVICE_HID_INIT hidInit1 = //for mouse
    {
        .hidReportDescriptorSize = sizeof(hid_rpt1),
        .hidReportDescriptor = &hid_rpt1,
        .queueSizeReportReceive = 1,
        .queueSizeReportSend = 1
    };
/**************************************************
 * USB Device Layer Function Driver Registration 
 * Table
 **************************************************/
const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[2] =
{
    /* Function 1 */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 0,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .numberOfInterfaces = 1,    /* Number of interfaces */
        .funcDriverIndex = HID_DRVIER_INDEX_KEYBOARD,  /* Index of HID Function Driver */
        .driver = (void*)USB_DEVICE_HID_FUNCTION_DRIVER,    /* USB HID function data exposed to device layer */
        .funcDriverInit = (void*)&hidInit0,    /* Function driver init data*/
    },
    /* Function 2 (mouse) */
    { 
        .configurationValue = 1,    /* Configuration value */ 
        .interfaceNumber = 1,       /* First interfaceNumber of this function */ 
        .speed = USB_SPEED_FULL,    /* Function Speed */ 
        .numberOfInterfaces = 1,    /* Number of interfaces */
        .funcDriverIndex = HID_DRVIER_INDEX_MOUSE,  /* Index of HID Function Driver */
        .driver = (void*)USB_DEVICE_HID_FUNCTION_DRIVER,    /* USB HID function data exposed to device layer */
        .funcDriverInit = (void*)&hidInit1,    /* Function driver init data */
    },
};

/*******************************************
 * USB Device Layer Descriptors
 *******************************************/
/*******************************************
 *  USB Device Descriptor 
 *******************************************/
const USB_DEVICE_DESCRIPTOR deviceDescriptor =
{
    0x12,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_DEVICE,          // DEVICE descriptor type
    0x0200,                         // USB Spec Release Number in BCD format
    0x00,                           // Class Code
    0x00,                           // Subclass code
    0x00,                           // Protocol code
    USB_DEVICE_EP0_BUFFER_SIZE,     // Max packet size for EP0, see system_config.h
    0x04D8,                         // Vendor ID
    USB_PID,                        // Product ID
    APKB32_REVISION,                // Device release number in BCD format
    0x01,                           // Manufacturer string index
    0x02,                           // Product string index
    0x05,                           // Device serial number string index
    0x01                            // Number of possible configurations
};


/*******************************************
 *  USB Full Speed Configuration Descriptor
 *******************************************/
const uint8_t fullSpeedConfigurationDescriptor[]=
{
    /* Configuration Descriptor */

    0x09,                                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_CONFIGURATION,                       // Descriptor Type
    73,0,                //(73 Bytes)Size of the Config descriptor.e
    2,                                               	// Number of interfaces in this cfg
    0x01,                                               // Index value of this configuration
    0x00,                                               // Configuration string index
    USB_ATTRIBUTE_DEFAULT | USB_ATTRIBUTE_SELF_POWERED | USB_ATTRIBUTE_REMOTE_WAKEUP, // Attributes
    50,                                                 // Max power consumption (2X mA)
    /* Descriptor for Function 1 - HID (for Keyboard) */ 

	/* Interface Descriptor */

    0x09,                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,           // INTERFACE descriptor type
    HID_INTERFACE_INDEX_KEYBOARD,       // Interface Number
    0,                                  // Alternate Setting Number
	2,                                  // Number of endpoints in this interface
    USB_HID_CLASS_CODE,                 // Class code
    USB_HID_SUBCLASS_CODE_BOOT_INTERFACE_SUBCLASS , // Subclass code
    USB_HID_PROTOCOL_CODE_KEYBOARD,         // Keyboard Protocol
    3,                                  // Interface string index

    /* HID Class-Specific Descriptor */

    0x09,                           // Size of this descriptor in bytes
    USB_HID_DESCRIPTOR_TYPES_HID,   // HID descriptor type
    0x11,0x01,                      // HID Spec Release Number in BCD format (1.11)
    USB_HID_COUNTRY_CODE_NOT_SUPPORTED,// Country Code (0x00 for Not supported)
    1,                              // Number of class descriptors, see usbcfg.h
    USB_HID_DESCRIPTOR_TYPES_REPORT,// Report descriptor type
    USB_DEVICE_16bitTo8bitArrange(sizeof(hid_rpt0)),   // Size of the report descriptor

    /* Endpoint Descriptor */

    0x07,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    1 | USB_EP_DIRECTION_IN,        // EndpointAddress ( EP1 IN )
    USB_TRANSFER_TYPE_INTERRUPT,    // Attributes
    0x40,0x00,                      // Size
    0x10,                           // Interval

	/* Endpoint Descriptor */

	0x07,                           // Size of this descriptor in bytes
	USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    1 | USB_EP_DIRECTION_OUT,       // EndpointAddress ( EP1 OUT )
	USB_TRANSFER_TYPE_INTERRUPT,    // Attributes
	0x40,0x00,                      // size
	0x01,                           // Interval



    /* Descriptor for Function 2 - HID (for mouse)     */

    /* Interface Descriptor */

    0x09,                               // Size of this descriptor in bytes
    USB_DESCRIPTOR_INTERFACE,           // INTERFACE descriptor type
    HID_INTERFACE_INDEX_MOUSE,                                  // Interface Number
    0,                                  // Alternate Setting Number
    1,                                  // Number of endpoints in this interface
    USB_HID_CLASS_CODE,                 // Class code
    USB_HID_SUBCLASS_CODE_BOOT_INTERFACE_SUBCLASS , // Subclass code
    USB_HID_PROTOCOL_CODE_MOUSE,         // Mouse Protocol
    4,                                  // Interface string index

    /* HID Class-Specific Descriptor */

    0x09,                           // Size of this descriptor in bytes
    USB_HID_DESCRIPTOR_TYPES_HID,   // HID descriptor type
    0x11,0x01,                      // HID Spec Release Number in BCD format (1.11)
    0x00,                           // Country Code (0x00 for Not supported)
    1,                              // Number of class descriptors, see usbcfg.h
    USB_HID_DESCRIPTOR_TYPES_REPORT,// Report descriptor type
    USB_DEVICE_16bitTo8bitArrange(sizeof(hid_rpt1)),   // Size of the report descriptor

    /* Endpoint Descriptor */

    0x07,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    2 | USB_EP_DIRECTION_IN,      // EndpointAddress
    USB_TRANSFER_TYPE_INTERRUPT,    // Attributes
    0x40,0x00,                      // Size
    0x10,                           // Interval

    /* Endpoint Descriptor */

    0x07,                           // Size of this descriptor in bytes
    USB_DESCRIPTOR_ENDPOINT,        // Endpoint Descriptor
    1 | USB_EP_DIRECTION_OUT,     // EndpointAddress
    USB_TRANSFER_TYPE_INTERRUPT,    // Attributes
    0x40,0x00,                      // size
    0x01,                           // Interval
    
    

    
};

/*******************************************
 * Array of Full speed config descriptors
 *******************************************/
USB_DEVICE_CONFIGURATION_DESCRIPTORS_TABLE fullSpeedConfigDescSet[1] =
{
    fullSpeedConfigurationDescriptor
};


/**************************************
 *  String descriptors.
 *************************************/

 /*******************************************
 *  Language code string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;
        uint8_t bDscType;
        uint16_t string[1];
    }
    sd000 =
    {
        sizeof(sd000),          // Size of this descriptor in bytes
        USB_DESCRIPTOR_STRING,  // STRING descriptor type
        {0x0409}                // Language ID
    };
/*******************************************
 *  Manufacturer string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[12];    // String
    }
    sd001 =
    {
        sizeof(sd001),
        USB_DESCRIPTOR_STRING,
        {'R','u','f','f','l','e','s',' ','I','n','c','.'}
		
    };

/*******************************************
 *  Product string descriptor
 *******************************************/
    const struct
    {
        uint8_t bLength;        // Size of this descriptor in bytes
        uint8_t bDscType;       // STRING descriptor type
        uint16_t string[6];    // String
    }
    sd002 =
    {
        sizeof(sd002),
        USB_DESCRIPTOR_STRING,
		{'A','P','K','B','3','2'}
    }; 

/*******************************************
 *  Function string descriptor (for Keyboard)
 *******************************************/
const struct
{
	uint8_t bLength;        // Size of this descriptor in bytes
	uint8_t bDscType;       // STRING descriptor type
	uint16_t string[15];    // String
}
sd003 =
{
	sizeof(sd003),
	USB_DESCRIPTOR_STRING,
	{'A','P','K','B','3','2',' ','K','e','y','b','o','a','r','d'}
}; 

/*******************************************
 *  Function string descriptor (for Mouse)
 *******************************************/
const struct
{
	uint8_t bLength;        // Size of this descriptor in bytes
	uint8_t bDscType;       // STRING descriptor type
	uint16_t string[12];    // String
}
sd004 =
{
	sizeof(sd004),
	USB_DESCRIPTOR_STRING,
	{'A','P','K','B','3','2',' ','M','o','u','s','e'}
}; 
/*******************************************
 *  Serial Number string descriptor
 *******************************************/
const struct
{
	uint8_t bLength;        // Size of this descriptor in bytes
	uint8_t bDscType;       // STRING descriptor type
	uint16_t string[9];    // String
}
sd005 =
{
	sizeof(sd005),
	USB_DESCRIPTOR_STRING,
	{'P','r','o','t','o','t','y','p','e'}
}; 

/***************************************
 * Array of string descriptors
 ***************************************/
USB_DEVICE_STRING_DESCRIPTORS_TABLE stringDescriptors[6]=
{
    (const uint8_t *const)&sd000,
    (const uint8_t *const)&sd001,
    (const uint8_t *const)&sd002,
	(const uint8_t *const)&sd003,
	(const uint8_t *const)&sd004,
	(const uint8_t *const)&sd005,
};

/*******************************************
 * USB Device Layer Master Descriptor Table 
 *******************************************/
const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor =
{
    &deviceDescriptor,          /* Full speed descriptor */
    1,                          /* Total number of full speed configurations available */
    fullSpeedConfigDescSet,     /* Pointer to array of full speed configurations descriptors*/
    NULL, 
    0, 
    NULL, 
    6,                          // Total number of string descriptors available.
    stringDescriptors,          // Pointer to array of string descriptors.
    NULL, 
    NULL
};


/****************************************************
 * USB Device Layer Initialization Data
 ****************************************************/
const USB_DEVICE_INIT usbDevInitData =
{
    /* System module initialization */
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    
    /* Number of function drivers registered to this instance of the
       USB device layer */
    .registeredFuncCount = 2,

    /* Function driver table registered to this instance of the USB device layer*/
    .registeredFunctions = (USB_DEVICE_FUNCTION_REGISTRATION_TABLE*)funcRegistrationTable,

    /* Pointer to USB Descriptor structure */
    .usbMasterDescriptor = (USB_DEVICE_MASTER_DESCRIPTOR*)&usbMasterDescriptor,

    /* USB Device Speed */
    .deviceSpeed = USB_SPEED_FULL,
    
    /* Index of the USB Driver to be used by this Device Layer Instance */
    .driverIndex = DRV_USBFS_INDEX_0,

    /* Pointer to the USB Driver Functions. */
    .usbDriverInterface = DRV_USBFS_DEVICE_INTERFACE,
    
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();

    /* Initialize Drivers */
    /* Initialize the OC Driver */
    DRV_OC0_Initialize();
    DRV_OC1_Initialize();
    DRV_OC2_Initialize();
    /*Initialize TMR0 */
    DRV_TMR0_Initialize();
    /*Initialize TMR1 */
    DRV_TMR1_Initialize();
 
     /* Initialize USB Driver */ 
    sysObj.drvUSBObject = DRV_USBFS_Initialize(DRV_USBFS_INDEX_0, (SYS_MODULE_INIT *) &drvUSBFSInit);

    /* Set priority of USB interrupt source */
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);

    /* Set Sub-priority of USB interrupt source */
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);

    /* Initialize System Services */

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /* Initialize Middleware */
    /* Initialize the USB device layer */
    sysObj.usbDevObject0 = USB_DEVICE_Initialize (USB_DEVICE_INDEX_0 , ( SYS_MODULE_INIT* ) & usbDevInitData);

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_4); //initialize L0 line

    DRV_TMR0_Start();
    DRV_TMR1_Start();
    
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_1);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
    //Initialize Logic
    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

