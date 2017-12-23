/*******************************************************************************
  SYS PORTS Static Functions for PORTS System Service

  Company:
    Microchip Technology Inc.

  File Name:
    sys_ports_static_pic32c.c

  Summary:
    SYS PORTS static function implementations for the Ports System Service.

  Description:
    The Ports System Service provides a simple interface to manage the ports
    on Microchip microcontrollers. This file defines the static implementation for the
    Ports System Service.

  Remarks:
    Static functions incorporate all system ports configuration settings as
    determined by the user via the Microchip Harmony Configurator GUI.  It provides
    static version of the routines, eliminating the need for an object ID or
    object handle.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#include "system/ports/sys_ports.h"

typedef struct {

    /* Bit position of the target pin */
    PORTS_BIT_POS          bitPos;

    /* Callback for event on target pin*/
    SYS_PORTS_PIN_CALLBACK callback;

    /* Callback Context */
    uintptr_t              context;

} SYS_PORTS_PIN_CALLBACK_OBJ;


/******************************************************************************
  Function:
    SYS_PORTS_Initialize ( void )

  Summary:
    Initializes Ports System Service

  Description:
    This function initializes different port pins/channels to the desired state.
    It also remaps the pins to the desired specific function.

  Remarks:
    None.
*/
void SYS_PORTS_Initialize ( void )
{
    /* Initialize the Debug pins as GPIO pins */
    _MATRIX_REGS->CCFG_SYSIO.w |= 0x00000000;

    /********** PORT A Initialization **********/
    /* PORTA ABCD 1 */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_ABCDSR[0].w = SYS_PORT_A_ABCD1;
    /* PORTA ABCD 2 */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_ABCDSR[1].w = SYS_PORT_A_ABCD2;
    /* PORTA Pio enable */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_PDR.w = ~SYS_PORT_A_PER;
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_PER.w = SYS_PORT_A_PER;
    /* PORTA Active state */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_CODR.w = ~SYS_PORT_A_LAT;
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_SODR.w = SYS_PORT_A_LAT;
    /* PORTA Open drain state */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_MDDR.w = ~SYS_PORT_A_OD;
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_MDER.w = SYS_PORT_A_OD;
    /* PORTA Pull Up */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_PUDR.w = ~SYS_PORT_A_PU;
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_PUER.w = SYS_PORT_A_PU;
    /* PORTA Pull Down */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_PPDDR.w = ~SYS_PORT_A_PD;
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_PPDER.w = SYS_PORT_A_PD;
    /* PORTA Direction */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_ODR.w = ~SYS_PORT_A_DIR;
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_OER.w = SYS_PORT_A_DIR;
    /* PORTA module level Interrupt disable */
    VCAST(port_registers_t, PORTS_CHANNEL_A)->PORT_IDR.w = ~SYS_PORT_A_INT;

    /********** PORT B Initialization **********/
    /* PORTB ABCD 1 */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_ABCDSR[0].w = SYS_PORT_B_ABCD1;
    /* PORTB ABCD 2 */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_ABCDSR[1].w = SYS_PORT_B_ABCD2;
    /* PORTB Pio enable */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_PDR.w = ~SYS_PORT_B_PER;
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_PER.w = SYS_PORT_B_PER;
    /* PORTB Active state */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_CODR.w = ~SYS_PORT_B_LAT;
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_SODR.w = SYS_PORT_B_LAT;
    /* PORTB Open drain state */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_MDDR.w = ~SYS_PORT_B_OD;
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_MDER.w = SYS_PORT_B_OD;
    /* PORTB Pull Up */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_PUDR.w = ~SYS_PORT_B_PU;
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_PUER.w = SYS_PORT_B_PU;
    /* PORTB Pull Down */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_PPDDR.w = ~SYS_PORT_B_PD;
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_PPDER.w = SYS_PORT_B_PD;
    /* PORTB Direction */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_ODR.w = ~SYS_PORT_B_DIR;
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_OER.w = SYS_PORT_B_DIR;
    /* PORTB module level Interrupt disable */
    VCAST(port_registers_t, PORTS_CHANNEL_B)->PORT_IDR.w = ~SYS_PORT_B_INT;

    /********** PORT C Initialization **********/
    /* PORTC ABCD 1 */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_ABCDSR[0].w = SYS_PORT_C_ABCD1;
    /* PORTC ABCD 2 */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_ABCDSR[1].w = SYS_PORT_C_ABCD2;
    /* PORTC Pio enable */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_PDR.w = ~SYS_PORT_C_PER;
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_PER.w = SYS_PORT_C_PER;
    /* PORTC Active state */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_CODR.w = ~SYS_PORT_C_LAT;
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_SODR.w = SYS_PORT_C_LAT;
    /* PORTC Open drain state */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_MDDR.w = ~SYS_PORT_C_OD;
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_MDER.w = SYS_PORT_C_OD;
    /* PORTC Pull Up */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_PUDR.w = ~SYS_PORT_C_PU;
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_PUER.w = SYS_PORT_C_PU;
    /* PORTC Pull Down */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_PPDDR.w = ~SYS_PORT_C_PD;
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_PPDER.w = SYS_PORT_C_PD;
    /* PORTC Direction */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_ODR.w = ~SYS_PORT_C_DIR;
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_OER.w = SYS_PORT_C_DIR;
    /* PORTC module level Interrupt disable */
    VCAST(port_registers_t, PORTS_CHANNEL_C)->PORT_IDR.w = ~SYS_PORT_C_INT;

    /********** PORT D Initialization **********/
    /* PORTD ABCD 1 */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_ABCDSR[0].w = SYS_PORT_D_ABCD1;
    /* PORTD ABCD 2 */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_ABCDSR[1].w = SYS_PORT_D_ABCD2;
    /* PORTD Pio enable */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_PDR.w = ~SYS_PORT_D_PER;
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_PER.w = SYS_PORT_D_PER;
    /* PORTD Active state */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_CODR.w = ~SYS_PORT_D_LAT;
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_SODR.w = SYS_PORT_D_LAT;
    /* PORTD Open drain state */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_MDDR.w = ~SYS_PORT_D_OD;
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_MDER.w = SYS_PORT_D_OD;
    /* PORTD Pull Up */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_PUDR.w = ~SYS_PORT_D_PU;
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_PUER.w = SYS_PORT_D_PU;
    /* PORTD Pull Down */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_PPDDR.w = ~SYS_PORT_D_PD;
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_PPDER.w = SYS_PORT_D_PD;
    /* PORTD Direction */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_ODR.w = ~SYS_PORT_D_DIR;
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_OER.w = SYS_PORT_D_DIR;
    /* PORTD module level Interrupt disable */
    VCAST(port_registers_t, PORTS_CHANNEL_D)->PORT_IDR.w = ~SYS_PORT_D_INT;

    /********** PORT E Initialization **********/
    /* PORTE ABCD 1 */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_ABCDSR[0].w = SYS_PORT_E_ABCD1;
    /* PORTE ABCD 2 */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_ABCDSR[1].w = SYS_PORT_E_ABCD2;
    /* PORTE Pio enable */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_PDR.w = ~SYS_PORT_E_PER;
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_PER.w = SYS_PORT_E_PER;
    /* PORTE Active state */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_CODR.w = ~SYS_PORT_E_LAT;
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_SODR.w = SYS_PORT_E_LAT;
    /* PORTE Open drain state */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_MDDR.w = ~SYS_PORT_E_OD;
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_MDER.w = SYS_PORT_E_OD;
    /* PORTE Pull Up */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_PUDR.w = ~SYS_PORT_E_PU;
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_PUER.w = SYS_PORT_E_PU;
    /* PORTE Pull Down */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_PPDDR.w = ~SYS_PORT_E_PD;
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_PPDER.w = SYS_PORT_E_PD;
    /* PORTE Direction */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_ODR.w = ~SYS_PORT_E_DIR;
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_OER.w = SYS_PORT_E_DIR;
    /* PORTE module level Interrupt disable */
    VCAST(port_registers_t, PORTS_CHANNEL_E)->PORT_IDR.w = ~SYS_PORT_E_INT;

    /* Analog pins Initialization */
    _AFEC0_REGS->AFEC_CHER.w = SYS_PORT_ANALOG0;
    _AFEC0_REGS->AFEC_CHER.w = SYS_PORT_ANALOG1;
    _DACC_REGS->DACC_CHER.w = SYS_PORT_DACC;
}

/******************************************************************************
  Function:
    PORTS_DATA_TYPE SYS_PORTS_Read ( PORTS_MODULE_ID index, PORTS_CHANNEL channel )

  Summary:
    Reads the data from the I/O port.

  Description:
    This function reads the data from the I/O port.

  Remarks:
    None.
*/

PORTS_DATA_TYPE SYS_PORTS_Read ( PORTS_MODULE_ID index, PORTS_CHANNEL channel )
{
    return VCAST(port_registers_t, channel)->PORT_PDSR.w;
}


/******************************************************************************
  Function:
    void SYS_PORTS_Write ( PORTS_MODULE_ID index,
                           PORTS_CHANNEL channel,
                           PORTS_DATA_TYPE value )

  Summary:
    Writes the data from the I/O port.

  Description:
    This function writes the data to the I/O port.

  Remarks:
    None.
*/

void SYS_PORTS_Write ( PORTS_MODULE_ID index,
                       PORTS_CHANNEL channel,
                       PORTS_DATA_TYPE value )
{
    /* Enable write to the selected port */
    VCAST(port_registers_t, channel)->PORT_OWER.w = PORT_OWER_Msk;

    /* Write the desired value */
    VCAST(port_registers_t, channel)->PORT_ODSR.w = value;
}

/******************************************************************************
  Function:
    PORTS_DATA_TYPE SYS_PORTS_LatchedGet ( PORTS_MODULE_ID index, PORTS_CHANNEL channel )

  Summary:
    Reads the data driven on the I/O port.

  Description:
    This function reads the data driven on the I/O port.

  Remarks:
    None.
*/

PORTS_DATA_TYPE SYS_PORTS_LatchedGet ( PORTS_MODULE_ID index, PORTS_CHANNEL channel )
{
    return VCAST(port_registers_t, channel)->PORT_ODSR.w;
}

/******************************************************************************
  Function:
    void SYS_PORTS_Set ( PORTS_MODULE_ID index,
                         PORTS_CHANNEL channel,
                         PORTS_DATA_TYPE value,
                         PORTS_DATA_MASK mask )

  Summary:
    Sets the selected digital port/latch based on the mask.

  Description:
    This function sets the selected digital port/latch relative to the mask.

  Remarks:
    None.
*/

void SYS_PORTS_Set ( PORTS_MODULE_ID index,
                     PORTS_CHANNEL channel,
                     PORTS_DATA_TYPE value,
                     PORTS_DATA_MASK mask )
{
    VCAST(port_registers_t, channel)->PORT_SODR.w = (value & mask);
}


/******************************************************************************
  Function:
    void SYS_PORTS_Clear ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                           PORTS_DATA_MASK clearMask )

  Summary:
    Clears the selected digital port.

  Description:
    This function clears the selected digital port.

  Remarks:
    None.
*/

void SYS_PORTS_Clear ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                       PORTS_DATA_MASK clearMask )
{
    VCAST(port_registers_t, channel)->PORT_CODR.w = clearMask;
}


/******************************************************************************
  Function:
    void SYS_PORTS_DirectionSelect ( PORTS_MODULE_ID index,
                                     SYS_PORTS_PIN_DIRECTION pinDir,
                                     PORTS_CHANNEL channel,
                                     PORTS_DATA_MASK mask )
  Summary:
    Enables the direction for the selected port.

  Description:
    This function enables the direction for the selected port.

  Remarks:
    None.
*/

void SYS_PORTS_DirectionSelect ( PORTS_MODULE_ID index,
                                 SYS_PORTS_PIN_DIRECTION pinDir,
                                 PORTS_CHANNEL channel,
                                 PORTS_DATA_MASK mask )
{
    if (pinDir == SYS_PORTS_DIRECTION_INPUT)
    {
        VCAST(port_registers_t, channel)->PORT_ODR.w = mask;
    }
    else
    {
        VCAST(port_registers_t, channel)->PORT_OER.w = mask;
    }
}


/******************************************************************************
  Function:
    PORTS_DATA_MASK SYS_PORTS_DirectionGet ( PORTS_MODULE_ID index,
                                             PORTS_CHANNEL channel )

  Summary:
    Reads the port direction for the selected port.

  Description:
    This function reads the port direction for the selected port.

  Remarks:
    None.
*/

PORTS_DATA_MASK SYS_PORTS_DirectionGet ( PORTS_MODULE_ID index,
                                         PORTS_CHANNEL channel )
{
    return VCAST(port_registers_t, channel)->PORT_OSR.w;
}


/******************************************************************************
  Function:
    void SYS_PORTS_Toggle ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                            PORTS_DATA_MASK toggleMask )

  Summary:
    Toggles the selected digital port pins.

  Description:
    This function toggles the selected digital port pins.

  Remarks:
    None.
*/

void SYS_PORTS_Toggle ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                        PORTS_DATA_MASK toggleMask )
{
    uint32_t statusReg = 0;

    statusReg = VCAST(port_registers_t, channel)->PORT_ODSR.w;

    /* Write into Clr and Set registers */
    VCAST(port_registers_t, channel)->PORT_CODR.w = ( toggleMask & (statusReg));;
    VCAST(port_registers_t, channel)->PORT_SODR.w = ( toggleMask & (~statusReg));;
}


/******************************************************************************
  Function:
    void SYS_PORTS_OpenDrainEnable ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                                     PORTS_DATA_MASK mask )

  Summary:
    Enables the open drain functionality for the selected port.

  Description:
    This function enables the open drain functionality for the selected port.

  Remarks:
    None.
*/

void SYS_PORTS_OpenDrainEnable ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                                 PORTS_DATA_MASK mask )
{
    VCAST(port_registers_t, channel)->PORT_MDER.w = mask;
}


/******************************************************************************
  Function:
    void SYS_PORTS_OpenDrainDisable ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                                      PORTS_DATA_MASK mask )

  Summary:
    Disables the open drain functionality for the selected port.

  Description:
    This function disables the open drain functionality for the selected port.

  Remarks:
    None.
*/

void SYS_PORTS_OpenDrainDisable ( PORTS_MODULE_ID index, PORTS_CHANNEL channel,
                                  PORTS_DATA_MASK mask )
{
    VCAST(port_registers_t, channel)->PORT_MDDR.w = mask;
}

// *****************************************************************************
/* Function:
    PORTS_DATA_TYPE SYS_PORTS_InterruptStatusGet ( PORTS_MODULE_ID index,
                                                   PORTS_CHANNEL channel )

  Summary:
    Reads the data from the I/O port.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function reads the data from the I/O port.

*/

PORTS_DATA_TYPE SYS_PORTS_InterruptStatusGet ( PORTS_MODULE_ID index,
                                               PORTS_CHANNEL channel )
{
    return VCAST(port_registers_t, channel)->PORT_ISR.w;
}

// *****************************************************************************
// *****************************************************************************
// Section: SYS Change Notification Pins Routines
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    void SYS_PORTS_ChangeNotificationGlobalEnable( PORTS_MODULE_ID index )

  Summary:
    Globally enables the change notification.

  Description:
    This function globally enables the change notification.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationGlobalEnable ( PORTS_MODULE_ID index )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}

/******************************************************************************
  Function:
    void SYS_PORTS_ChangeNotificationGlobalDisable( PORTS_MODULE_ID index )

  Summary:
    Globally disables the change notification.

  Description:
    This function globally disables the change notification.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationGlobalDisable ( PORTS_MODULE_ID index )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}

/******************************************************************************
  Function:
    void SYS_PORTS_GlobalChangeNotificationDisable( PORTS_MODULE_ID index )

  Summary:
    Globally disables the change notification for the selected port.

  Description:
    This function globally disables the change notification for the selected port.

  Remarks:
    None.
*/

void SYS_PORTS_GlobalChangeNotificationDisable ( PORTS_MODULE_ID index )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}


/******************************************************************************
  Function:
    void SYS_PORTS_ChangeNotificationEnable ( PORTS_MODULE_ID index,
                                              PORTS_CHANGE_NOTICE_PIN pinNum,
                                              SYS_PORTS_PULLUP_PULLDOWN_STATUS value )

  Summary:
    Enables the change notification for the selected port.

  Description:
    This function enables the change notification for the selected port.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationEnable ( PORTS_MODULE_ID index,
                                          PORTS_CHANGE_NOTICE_PIN pinNum,
                                          SYS_PORTS_PULLUP_PULLDOWN_STATUS value )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}


/******************************************************************************
  Function:
    void SYS_PORTS_ChangeNotificationDisable ( PORTS_MODULE_ID index,
                                               PORTS_CHANGE_NOTICE_PIN pinNum )

  Summary:
    Disables the change notification for the selected port.

  Description:
    This function disables the change notification for the selected port.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationDisable ( PORTS_MODULE_ID index,
                                           PORTS_CHANGE_NOTICE_PIN pinNum )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}


/******************************************************************************
  Function:
    void SYS_PORTS_ChangeNotificationInIdleModeEnable ( PORTS_MODULE_ID index )

  Summary:
    Enables the change notification for the selected port in Sleep or Idle mode.

  Description:
    This function enables the change notification for the selected port in Sleep
    or Idle mode.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationInIdleModeEnable ( PORTS_MODULE_ID index )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_ChangeNotificationInIdleModeDisable ( PORTS_MODULE_ID index )

  Summary:
    Disables the change notification for the selected port in Sleep or Idle mode.

  Description:
    This function disables the change notification for the selected port in Sleep
    or Idle mode.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationInIdleModeDisable ( PORTS_MODULE_ID index )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_ChangeNotificationPullUpEnable ( PORTS_MODULE_ID index,
                                                    PORTS_CHANGE_NOTICE_PIN pinNum )

  Summary:
    Enables weak pull-up on change notification pin.

  Description:
    This function enables weak pull-up on change notification pin.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationPullUpEnable ( PORTS_MODULE_ID index,
                                                PORTS_CHANGE_NOTICE_PIN pinNum )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_ChangeNotificationPullUpDisable ( PORTS_MODULE_ID index,
                                                    PORTS_CHANGE_NOTICE_PIN pinNum )

  Summary:
    Disables pull-up on input change.

  Description:
    This function disables pull-up on input change.

  Remarks:
    None.
*/

void SYS_PORTS_ChangeNotificationPullUpDisable ( PORTS_MODULE_ID index,
                                                 PORTS_CHANGE_NOTICE_PIN pinNum )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: SYS PORT PINS Control Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void SYS_PORTS_PinModeSelect ( PORTS_MODULE_ID index, PORTS_ANALOG_PIN pin,
                                   PORTS_PIN_MODE mode)

  Summary:
    Enables the selected pin as analog or digital.

  Description:
    This function enables the selected pin as analog or digital.

 Remarks:
    None.
*/

void SYS_PORTS_PinModeSelect ( PORTS_MODULE_ID index, PORTS_ANALOG_PIN pin,
                               PORTS_PIN_MODE mode)
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}

// *****************************************************************************
/* Function:
    void SYS_PORTS_PinWrite ( PORTS_MODULE_ID index,
                              PORTS_CHANNEL channel,
                              PORTS_BIT_POS bitPos
                              bool value )
  Summary:
    Writes the selected digital pin.

  Description:
    This function writes the selected digital pin.

  Remarks:
    None.
*/

void SYS_PORTS_PinWrite ( PORTS_MODULE_ID index,
                          PORTS_CHANNEL channel,
                          PORTS_BIT_POS bitPos,
                          bool value )
{
    if (value == false)
    {
        VCAST(port_registers_t, channel)->PORT_CODR.w = 1 << bitPos;
    }
    else
    {
        VCAST(port_registers_t, channel)->PORT_SODR.w = 1 << bitPos;
    }
}


// *****************************************************************************
/* Function:
    bool SYS_PORTS_PinLatchedGet ( PORTS_MODULE_ID index,
                                   PORTS_CHANNEL channel,
                                   PORTS_BIT_POS bitPos )

  Summary:
    Reads the data driven on selected digital pin.

  Description:
    This function reads the driven data on selected digital pin.

  Remarks:
    None.
*/

bool SYS_PORTS_PinLatchedGet ( PORTS_MODULE_ID index,
                               PORTS_CHANNEL channel,
                               PORTS_BIT_POS bitPos )
{
    return (bool)((VCAST(port_registers_t, channel)->PORT_ODSR.w >> bitPos) & 0x00000001);
}


// *****************************************************************************
/* Function:
    bool SYS_PORTS_PinRead ( PORTS_MODULE_ID index,
                             PORTS_CHANNEL channel,
                             PORTS_BIT_POS bitPos )

  Summary:
    Reads the selected digital pin.

  Description:
    This function reads the selected digital pin.

  Remarks:
    None.
*/

bool SYS_PORTS_PinRead ( PORTS_MODULE_ID index,
                         PORTS_CHANNEL channel,
                         PORTS_BIT_POS bitPos )
{
    return (bool)((VCAST(port_registers_t, channel)->PORT_PDSR.w >> bitPos) & 0x00000001);
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinToggle ( PORTS_MODULE_ID index,
                               PORTS_CHANNEL channel,
                               PORTS_BIT_POS bitPos )

  Summary:
    Toggles the selected digital pin.

  Description:
    This function toggles the selected digital pin.

  Remarks:
    None.
*/

void SYS_PORTS_PinToggle ( PORTS_MODULE_ID index,
                           PORTS_CHANNEL channel,
                           PORTS_BIT_POS bitPos )
{
    if ( ((VCAST(port_registers_t, channel)->PORT_ODSR.w >> bitPos) & 1) == 1 )
    {
        VCAST(port_registers_t, channel)->PORT_CODR.w = 1 << bitPos;
    }
    else
    {
        VCAST(port_registers_t, channel)->PORT_SODR.w = 1 << bitPos;
    }
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinSet ( PORTS_MODULE_ID index,
                            PORTS_CHANNEL channel,
                            PORTS_BIT_POS bitPos )

  Summary:
    Sets the selected digital pin/latch.

  Description:
    This function sets the selected digital pin/latch.

  Remarks:
    None.
*/

void SYS_PORTS_PinSet ( PORTS_MODULE_ID index,
                        PORTS_CHANNEL channel,
                        PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_SODR.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinClear ( PORTS_MODULE_ID index,
                              PORTS_CHANNEL channel,
                              PORTS_BIT_POS bitPos )

  Summary:
    Clears the selected digital pin.

  Description:
    This function clears the selected digital pin.

  Remarks:
    None.
*/

void SYS_PORTS_PinClear ( PORTS_MODULE_ID index,
                          PORTS_CHANNEL channel,
                          PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_CODR.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinDirectionSelect ( PORTS_MODULE_ID index,
                                        SYS_PORTS_PIN_DIRECTION pinDir,
                                        PORTS_CHANNEL channel,
                                        PORTS_BIT_POS bitPos )
  Summary:
    Enables the direction for the selected pin.

  Description:
    This function enables the direction for the selected pin.

  Remarks:
    None.
*/

void SYS_PORTS_PinDirectionSelect ( PORTS_MODULE_ID index,
                                    SYS_PORTS_PIN_DIRECTION pinDir,
                                    PORTS_CHANNEL channel,
                                    PORTS_BIT_POS bitPos )
{
    if (pinDir == SYS_PORTS_DIRECTION_OUTPUT)
    {
        VCAST(port_registers_t, channel)->PORT_OER.w = 1 << bitPos;
    }
    else
    {
        VCAST(port_registers_t, channel)->PORT_ODR.w = 1 << bitPos;
    }
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinOpenDrainEnable ( PORTS_MODULE_ID index,
                                        PORTS_CHANNEL channel,
                                        PORTS_BIT_POS bitPos )

  Summary:
    Enables the open-drain functionality for the selected pin.

  Description:
    This function enables the open-drain functionality for the selected pin.

  Remarks:
    None.
*/

void SYS_PORTS_PinOpenDrainEnable ( PORTS_MODULE_ID index,
                                    PORTS_CHANNEL channel,
                                    PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_MDER.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinOpenDrainDisable ( PORTS_MODULE_ID index,
                                         PORTS_CHANNEL channel,
                                         PORTS_BIT_POS bitPos )

  Summary:
    Disables the open-drain functionality for the selected pin.

  Description:
    This function disables the open-drain functionality for the selected pin.

  Remarks:
    None.
*/

void SYS_PORTS_PinOpenDrainDisable ( PORTS_MODULE_ID index,
                                     PORTS_CHANNEL channel,
                                     PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_MDDR.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinPullUpEnable ( PORTS_MODULE_ID index,
                                        PORTS_CHANNEL channel,
                                        PORTS_BIT_POS bitPos )

  Summary:
    Enables the pull-up functionality for the selected pin.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables the pull-up functionality for the selected pin.

  Remarks:
    Not all features are available on all devices. Refer to the specific device
    data sheet for availability.
*/

void SYS_PORTS_PinPullUpEnable ( PORTS_MODULE_ID index,
                                    PORTS_CHANNEL channel,
                                    PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_PUER.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinPullUpDisable ( PORTS_MODULE_ID index,
                                         PORTS_CHANNEL channel,
                                         PORTS_BIT_POS bitPos )

  Summary:
    Disables the pull-up functionality for the selected pin.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables the pull-up functionality for the selected pin.

  Remarks:
    Not all features are available on all devices. Refer to the specific device
    data sheet for availability.
*/

void SYS_PORTS_PinPullUpDisable ( PORTS_MODULE_ID index,
                                     PORTS_CHANNEL channel,
                                     PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_PUDR.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinPullDownEnable ( PORTS_MODULE_ID index,
                                        PORTS_CHANNEL channel,
                                        PORTS_BIT_POS bitPos )

  Summary:
    Enables the pull-down functionality for the selected pin.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables the pull-down functionality for the selected pin.

  Remarks:
    Not all features are available on all devices. Refer to the specific device
    data sheet for availability.
*/

void SYS_PORTS_PinPullDownEnable ( PORTS_MODULE_ID index,
                                    PORTS_CHANNEL channel,
                                    PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_PPDER.w = 1 << bitPos;
}


// *****************************************************************************
/* Function:
    void SYS_PORTS_PinPullDownDisable ( PORTS_MODULE_ID index,
                                        PORTS_CHANNEL channel,
                                        PORTS_BIT_POS bitPos )

  Summary:
    Disables the pull-down functionality for the selected pin.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function disables the pull-down functionality for the selected pin.

  Remarks:
    Not all features are available on all devices. Refer to the specific device
    data sheet for availability.
*/

void SYS_PORTS_PinPullDownDisable ( PORTS_MODULE_ID index,
                                    PORTS_CHANNEL channel,
                                    PORTS_BIT_POS bitPos )
{
    VCAST(port_registers_t, channel)->PORT_PPDDR.w = 1 << bitPos;
}


/******************************************************************************
  Function:
    void SYS_PORTS_InterruptEnable
    (
        PORTS_MODULE_ID index,
        PORTS_CHANNEL channel,
        PORTS_BIT_POS bitPos,
        PORTS_PIN_INTERRUPT_TYPE pinInterruptType
    )

  Summary:
    Enables the change notification interrupt for the selected port pin.

  Description:
    This function enables the change notification interrupt of selected type
    for the selected port pin.

  Remarks:
    None.
*/

void SYS_PORTS_InterruptEnable ( PORTS_MODULE_ID index,
                                 PORTS_CHANNEL channel,
                                 PORTS_BIT_POS bitPos,
                                 PORTS_PIN_INTERRUPT_TYPE pinInterruptType )
{
    switch (pinInterruptType)
    {
        case PORTS_PIN_INTERRUPT_RISING_EDGE:
            VCAST(port_registers_t, channel)->PORT_ESR.w = 1 << bitPos;
            VCAST(port_registers_t, channel)->PORT_REHLSR.w = 1 << bitPos;
        break;

        case PORTS_PIN_INTERRUPT_FALLING_EDGE:
            VCAST(port_registers_t, channel)->PORT_ESR.w = 1 << bitPos;
            VCAST(port_registers_t, channel)->PORT_FELLSR.w = 1 << bitPos;
        break;

        case PORTS_PIN_INTERRUPT_HIGH_LEVEL:
            VCAST(port_registers_t, channel)->PORT_LSR.w = 1 << bitPos;
            VCAST(port_registers_t, channel)->PORT_REHLSR.w = 1 << bitPos;
        break;

        case PORTS_PIN_INTERRUPT_LOW_LEVEL:
            VCAST(port_registers_t, channel)->PORT_LSR.w = 1 << bitPos;
            VCAST(port_registers_t, channel)->PORT_FELLSR.w = 1 << bitPos;
        break;

        default:
        break;
    }

    if (pinInterruptType == PORTS_PIN_INTERRUPT_NONE)
    {
        /* Do nothing */
    }
    else
    {
        if (pinInterruptType == PORTS_PIN_INTERRUPT_BOTH_EDGE)
        {
           VCAST(port_registers_t, channel)->PORT_IER.w = 1 << bitPos;
        }
        else
        {
            /* For other interrupts, we have to enable additional interrupt */
            VCAST(port_registers_t, channel)->PORT_AIMER.w = 1 << bitPos;
            VCAST(port_registers_t, channel)->PORT_IER.w = 1 << bitPos;
        }
    }

    return;
}

// *****************************************************************************
/* Function:
    void SYS_PORTS_RemapInput ( PORTS_MODULE_ID index,
                                PORTS_REMAP_INPUT_FUNCTION function,
                                PORTS_REMAP_INPUT_PIN remapPin )

  Summary:
    Input/Output (I/O) function remapping.

  Description:
    This function controls the I/O function remapping.

  Precondition:
    None.
*/
void SYS_PORTS_RemapInput ( PORTS_MODULE_ID index,
                            PORTS_REMAP_INPUT_FUNCTION function,
                            PORTS_REMAP_INPUT_PIN remapPin )
{
    SYS_ASSERT(false, "This API is not supported on this device");
    return;
}

// *****************************************************************************
/* Function:
    void SYS_PORTS_RemapOutput ( PORTS_MODULE_ID index,
                                 PORTS_REMAP_OUTPUT_FUNCTION function,
                                 PORTS_REMAP_OUTPUT_PIN remapPin )

  Summary:
    Input/Output (I/O) function remapping.

  Description:
    This function controls the I/O function remapping.

  Precondition:
    None.
*/
void SYS_PORTS_RemapOutput ( PORTS_MODULE_ID index,
                             PORTS_REMAP_OUTPUT_FUNCTION function,
                             PORTS_REMAP_OUTPUT_PIN remapPin )
{
    SYS_ASSERT(false, "This API is not supported on this device");
}

//******************************************************************************
/* Function:
    void SYS_PORTS_CallbackSet
    (
        PORTS_MODULE_ID index,
        PORTS_CHANNEL channel,
        PORTS_BIT_POS bitPos,
        SYS_PORTS_PIN_CALLBACK callback,
        uintptr_t context
    )

  Summary:
    Sets the event callback function for the selected port pin.

  Description:
    This function Sets the event callback function for the selected port pin.

  Preconditions:
    Interrupt for the pin to be enabled from the PIO Interrupt tab of Pin Settings in MHC.
*/

void SYS_PORTS_PinCallbackRegister( PORTS_CHANNEL channel,
                                    PORTS_BIT_POS bitPos,
                                    SYS_PORTS_PIN_CALLBACK callback,
                                    uintptr_t context )
{
    SYS_ASSERT(false, "Interrupt for the pin to be enabled from the PIO Interrupt tab of Pin Settings in MHC.");
}

//******************************************************************************
/* Function:
    void SYS_PORTS_Tasks ( PORTS_CHANNEL channel )

  Summary:
    Implements the ISR for the given channel.

  Description:
    This function implements the ISR for the given channel.

  Preconditions:
    Interrupt for the pin to be enabled from the PIO Interrupt tab of Pin Settings in MHC.
*/

void SYS_PORTS_Tasks ( PORTS_CHANNEL channel )
{
    ;
}

/*******************************************************************************
 End of File
*/
