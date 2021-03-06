/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version 2.04
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

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/
#include "bsp/bsp.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
/*** Ports System Service Configuration ***/


#define SYS_PORT_A_DIR          0x00000001
#define SYS_PORT_A_LAT          0x00000000
#define SYS_PORT_A_OD           0x00000000
#define SYS_PORT_A_PU           0x00000200
#define SYS_PORT_A_PD           0x00000000
#define SYS_PORT_A_PER          0xFFFFFFFF
#define SYS_PORT_A_ABCD1        0x00000000
#define SYS_PORT_A_ABCD2        0x00000000
#define SYS_PORT_A_INT          0x00000000
#define SYS_PORT_A_INT_TYPE     0x00000000
#define SYS_PORT_A_INT_EDGE     0x00000000
#define SYS_PORT_A_INT_RE_HL    0x00000000
#define SYS_PORT_A_NUM_INT_PINS 0

#define SYS_PORT_B_DIR          0x00000004
#define SYS_PORT_B_LAT          0x00000000
#define SYS_PORT_B_OD           0x00000000
#define SYS_PORT_B_PU           0x00000000
#define SYS_PORT_B_PD           0x00000000
#define SYS_PORT_B_PER          0xFFFFFFFF
#define SYS_PORT_B_ABCD1        0x00000000
#define SYS_PORT_B_ABCD2        0x00000000
#define SYS_PORT_B_INT          0x00000000
#define SYS_PORT_B_INT_TYPE     0x00000000
#define SYS_PORT_B_INT_EDGE     0x00000000
#define SYS_PORT_B_INT_RE_HL    0x00000000
#define SYS_PORT_B_NUM_INT_PINS 0

#define SYS_PORT_C_DIR          0x40000000
#define SYS_PORT_C_LAT          0x00000000
#define SYS_PORT_C_OD           0x00000000
#define SYS_PORT_C_PU           0x00000200
#define SYS_PORT_C_PD           0x00000000
#define SYS_PORT_C_PER          0xFFFFFFFF
#define SYS_PORT_C_ABCD1        0x00000000
#define SYS_PORT_C_ABCD2        0x00000000
#define SYS_PORT_C_INT          0x00000000
#define SYS_PORT_C_INT_TYPE     0x00000000
#define SYS_PORT_C_INT_EDGE     0x00000000
#define SYS_PORT_C_INT_RE_HL    0x00000000
#define SYS_PORT_C_NUM_INT_PINS 0

#define SYS_PORT_D_DIR          0x00000000
#define SYS_PORT_D_LAT          0x00000000
#define SYS_PORT_D_OD           0x00000000
#define SYS_PORT_D_PU           0x00000000
#define SYS_PORT_D_PD           0x00000000
#define SYS_PORT_D_PER          0xFFFFFFFF
#define SYS_PORT_D_ABCD1        0x00000000
#define SYS_PORT_D_ABCD2        0x00000000
#define SYS_PORT_D_INT          0x00000000
#define SYS_PORT_D_INT_TYPE     0x00000000
#define SYS_PORT_D_INT_EDGE     0x00000000
#define SYS_PORT_D_INT_RE_HL    0x00000000
#define SYS_PORT_D_NUM_INT_PINS 0

#define SYS_PORT_E_DIR          0x00000000
#define SYS_PORT_E_LAT          0x00000000
#define SYS_PORT_E_OD           0x00000000
#define SYS_PORT_E_PU           0x00000000
#define SYS_PORT_E_PD           0x00000000
#define SYS_PORT_E_PER          0xFFFFFFFF
#define SYS_PORT_E_ABCD1        0x00000000
#define SYS_PORT_E_ABCD2        0x00000000
#define SYS_PORT_E_INT          0x00000000
#define SYS_PORT_E_INT_TYPE     0x00000000
#define SYS_PORT_E_INT_EDGE     0x00000000
#define SYS_PORT_E_INT_RE_HL    0x00000000
#define SYS_PORT_E_NUM_INT_PINS 0

#define SYS_PORT_ANALOG0        0x00000000
#define SYS_PORT_ANALOG1        0x00000000
#define SYS_PORT_DACC           0x00000000

/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************

/*** USB Driver Configuration ***/


/* Enables Device Support */
#define DRV_USBHSV1_DEVICE_SUPPORT    true

/* Disable Host Support */
#define DRV_USBHSV1_HOST_SUPPORT      false

/* Maximum USB driver instances */
#define DRV_USBHSV1_INSTANCES_NUMBER  1

/* Interrupt mode enabled */
#define DRV_USBHSV1_INTERRUPT_MODE    true


/* Number of Endpoints used */
#define DRV_USBHSV1_ENDPOINTS_NUMBER  7


/*** USB Device Stack Configuration ***/










/* The USB Device Layer will not initialize the USB Driver */
#define USB_DEVICE_DRIVER_INITIALIZE_EXPLICIT

/* Maximum device layer instances */
#define USB_DEVICE_INSTANCES_NUMBER     1

/* EP0 size in bytes */
#define USB_DEVICE_EP0_BUFFER_SIZE      64










/* Maximum instances of CDC function driver */
#define USB_DEVICE_CDC_INSTANCES_NUMBER     2










/* CDC Transfer Queue Size for both read and
   write. Applicable to all instances of the
   function driver */
#define USB_DEVICE_CDC_QUEUE_DEPTH_COMBINED 6


// *****************************************************************************
/* BSP Configuration Options
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************
/*** Application Defined Pins ***/

/*** Functions for BSP_LED_2 pin ***/
#define BSP_LED_2Toggle() SYS_PORTS_PinToggle(PORTS_ID_0, PORTS_CHANNEL_C, PORTS_BIT_POS_30)
#define BSP_LED_2On() SYS_PORTS_PinClear(PORTS_ID_0, PORTS_CHANNEL_C, PORTS_BIT_POS_30)
#define BSP_LED_2Off() SYS_PORTS_PinSet(PORTS_ID_0, PORTS_CHANNEL_C, PORTS_BIT_POS_30)
#define BSP_LED_2StateGet() !(SYS_PORTS_PinRead(PORTS_ID_0, PORTS_CHANNEL_C, PORTS_BIT_POS_30))

/*** Functions for BSP_LED_3 pin ***/
#define BSP_LED_3Toggle() SYS_PORTS_PinToggle(PORTS_ID_0, PORTS_CHANNEL_B, PORTS_BIT_POS_2)
#define BSP_LED_3On() SYS_PORTS_PinClear(PORTS_ID_0, PORTS_CHANNEL_B, PORTS_BIT_POS_2)
#define BSP_LED_3Off() SYS_PORTS_PinSet(PORTS_ID_0, PORTS_CHANNEL_B, PORTS_BIT_POS_2)
#define BSP_LED_3StateGet() !(SYS_PORTS_PinRead(PORTS_ID_0, PORTS_CHANNEL_B, PORTS_BIT_POS_2))

/*** Functions for BSP_LED_1 pin ***/
#define BSP_LED_1Toggle() SYS_PORTS_PinToggle(PORTS_ID_0, PORTS_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_LED_1On() SYS_PORTS_PinClear(PORTS_ID_0, PORTS_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_LED_1Off() SYS_PORTS_PinSet(PORTS_ID_0, PORTS_CHANNEL_A, PORTS_BIT_POS_0)
#define BSP_LED_1StateGet() !(SYS_PORTS_PinRead(PORTS_ID_0, PORTS_CHANNEL_A, PORTS_BIT_POS_0))

/*** Functions for BSP_SWITCH_0 pin ***/
#define BSP_SWITCH_0StateGet() SYS_PORTS_PinRead(PORTS_ID_0, PORTS_CHANNEL_A, PORTS_BIT_POS_9)

/*** Functions for BSP_USB_VBUS_IN pin ***/
#define BSP_USB_VBUS_INStateGet() SYS_PORTS_PinRead(PORTS_ID_0, PORTS_CHANNEL_C, PORTS_BIT_POS_9)


/*** Application Instance 0 Configuration ***/
    
/* Application USB Device CDC Read Buffer Size. This should be a multiple of
 * the CDC Bulk Endpoint size */

#define APP_READ_BUFFER_SIZE 512

/* Macro defines USB internal DMA Buffer criteria*/

#define APP_MAKE_BUFFER_DMA_READY  __attribute__((aligned(16)))

/* Macros defines board specific led */

#define APP_USB_LED_1    BSP_LED_1

/* Macros defines board specific led */

#define APP_USB_LED_2    BSP_LED_2

/* Macros defines board specific led */

#define APP_USB_LED_3    BSP_LED_3

/* Macros defines board specific switch */

#define APP_USB_SWITCH_1    BSP_SWITCH_0

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/
