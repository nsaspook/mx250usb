 /*******************************************************************************
  Watchdog Timer System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_wdt.c

  Summary:
    Watchdog Timer (WDT) System Service implementation.

  Description:
    The WDT System Service provides a simple interface to manage the
    Watchdog Timer module on Microchip microcontrollers.  This file implements
    the core interface routines for the WDT System Service. While building the
    system service from source, ALWAYS include this file in the build.
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system/wdt/sys_wdt.h"
#include "system/common/sys_common.h"
#include "system/int/sys_int.h"
#include "system/clk/sys_clk.h"

static void _SYS_WDT_Slowclock_Delay ( void );

static SYS_WDT_MODULE_INSTANCE instance;

void SYS_WDT_Enable ( bool windowModeEnable )
{
    /* Not possible on PIC32C device (WDT config set during initialization) */
    return ;
}

void SYS_WDT_Disable ( void )
{
    /* Not possible on PIC32C device (WDT config set during initialization) */
    return ;
}

void SYS_WDT_TimerClear ( void )
{
    /* Perform WDT Software restart */
    _WDT_REGS->WDT_CR.w = (WDT_CR_KEY_PASSWD|WDT_CR_WDRSTT_Msk);
    /* Wait 3 slow clock cycles (see WDT_CR register description from product datasheet) */
    _SYS_WDT_Slowclock_Delay( );
}

void SYS_WDT_Initialize (void)
{
  /* - Initialize system watchdog (WDT) */
  instance.timeoutPeriod = 0xFFF;
  instance.interruptMode = false;
  instance.resetEnable   = false;
  instance.interruptSource = WDT_IRQn;
  _WDT_REGS->WDT_MR.w = \
                WDT_MR_WDV(0xFFF) \
                | WDT_MR_WDD(0xFFF) \
  ;
    /* - Disable system reinforced safety watchdog (WDT) */
  _RSWDT_REGS->RSWDT_MR.WDDIS = true;
}
void SYS_WDT_RegisterCallback(SYS_WDT_TIMEOUT_CALLBACK callback, 
    uintptr_t context )
{
	instance.callback = callback;
	instance.context  = context;
}

void SYS_WDT_ProcessEvents(void)
{
	(instance.callback)(instance.context);
}

static void _SYS_WDT_Slowclock_Delay ( void )
{
    volatile uint32_t wdt_synchro_delay;
    wdt_synchro_delay = 3 * (SYS_CLK_SystemFrequencyGet()/32768)  ;
    for(;wdt_synchro_delay;wdt_synchro_delay--)
    {
        //Do nothing
    }
}

/*******************************************************************************
End of File
*/
