/**
 * Copyright (c) 2016-2017 Microchip Technology Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arch/arm/devices_pic32c.h" /* for vectors structure and default handler names and CMSIS API */
#include "arch/arch.h" /* for ARCH_CORE_MPU_Initialize() defintion */
#include "system_definitions.h" /* for potential custom handler names */

/* Symbols exported from linker script */
extern uint32_t __etext ;
extern uint32_t __ram_vectors__;
extern uint32_t __data_start__ ;
extern uint32_t __data_end__ ;
extern uint32_t __bss_start__ ;
extern uint32_t __bss_end__ ;
extern uint32_t __StackTop;

extern int main( void ) ;
/* symbols from libc */
extern void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Declaration of Reset handler (may be custom) */
void Reset_Handler(void);

/* Cortex-M7 core handlers */
/*
 * Warn about debug specific handlers usage:
 * - DEBUG=1 may be set by user on toolchain command line
 * - __DEBUG is set by MPLAB X while Building Application for Debug
 */
#if (defined DEBUG && (DEBUG == 1)) || (defined __DEBUG)
#pragma message "DEBUG handlers activated"

/*
 * These handlers are unitary implemented as weak (coming from ARM CMSIS-Core definitions) to let user defined its own.
 */
__attribute__((weak)) void NonMaskableInt_Handler(void)
{
  __BKPT(14);
  while (1);
}

__attribute__((weak)) void HardFault_Handler(void)
{
  __BKPT(13);
  while (1);
}

__attribute__((weak)) void MemoryManagement_Handler(void)
{
  __BKPT(12);
  while (1);
}

__attribute__((weak)) void BusFault_Handler(void)
{
  __BKPT(11);
  while (1);
}

__attribute__((weak)) void UsageFault_Handler(void)
{
  __BKPT(10);
  while (1);
}

__attribute__((weak)) void SVCall_Handler(void)
{
  __BKPT(5);
  while (1);
}

__attribute__((weak)) void DebugMonitor_Handler(void)
{
  __BKPT(4);
  while (1);
}

__attribute__((weak)) void PendSV_Handler(void)
{
  __BKPT(2);
  while (1);
}

__attribute__((weak)) void SysTick_Handler(void)
{
  __BKPT(1);
  while (1);
}

#else

void NonMaskableInt_Handler   ( void ) __attribute__((weak, alias("Dummy_Handler")));
void HardFault_Handler        ( void ) __attribute__((weak, alias("Dummy_Handler")));
void MemoryManagement_Handler ( void ) __attribute__((weak, alias("Dummy_Handler")));
void BusFault_Handler         ( void ) __attribute__((weak, alias("Dummy_Handler")));
void UsageFault_Handler       ( void ) __attribute__((weak, alias("Dummy_Handler")));
void SVCall_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));
void DebugMonitor_Handler     ( void ) __attribute__((weak, alias("Dummy_Handler")));
void PendSV_Handler           ( void ) __attribute__((weak, alias("Dummy_Handler")));

#endif // DEBUG=1

/* Exception Table */
__attribute__ ((section(".flash_vectors")))
const DeviceVectors exception_table=
{
  /* Configure Initial Stack Pointer, using linker-generated symbols */
  .pvStack = (void*) (&__StackTop),

  .pfnReset_Handler              = (void*) Reset_Handler,
  .pfnNonMaskableInt_Handler     = (void*) NonMaskableInt_Handler,
  .pfnHardFault_Handler          = (void*) HardFault_Handler,
  .pfnMemoryManagement_Handler   = (void*) MemoryManagement_Handler,
  .pfnBusFault_Handler           = (void*) BusFault_Handler,
  .pfnUsageFault_Handler         = (void*) UsageFault_Handler,
  .pfnDebugMonitor_Handler       = (void*) DebugMonitor_Handler,
  .pvReservedC9                  = (void*) (0UL), /* Reserved */
  .pvReservedC8                  = (void*) (0UL), /* Reserved */
  .pvReservedC7                  = (void*) (0UL), /* Reserved */
  .pvReservedC6                  = (void*) (0UL), /* Reserved */
  .pfnSVCall_Handler             = (void*) SVCall_Handler,
  .pvReservedC3                  = (void*) (0UL), /* Reserved */
  .pfnPendSV_Handler             = (void*) PendSV_Handler,

  .pfnTWI0_Handler               = (void*) Dummy_Handler,
  .pfnTWI1_Handler               = (void*) Dummy_Handler,
  .pfnTWI2_Handler               = (void*) Dummy_Handler,
  .pfnEFC_Handler                = (void*) Dummy_Handler,
  .pfnHSMCI_Handler              = (void*) Dummy_Handler,
  .pfnSPI0_Handler               = (void*) Dummy_Handler,
  .pfnSPI1_Handler               = (void*) Dummy_Handler,
  .pfnQSPI_Handler               = (void*) Dummy_Handler,
  .pfnTC0_CH0_Handler            = (void*) Dummy_Handler,
  .pfnTC0_CH1_Handler            = (void*) Dummy_Handler,
  .pfnTC0_CH2_Handler            = (void*) Dummy_Handler,
  .pfnTC1_CH0_Handler            = (void*) Dummy_Handler,
  .pfnTC1_CH1_Handler            = (void*) Dummy_Handler,
  .pfnTC1_CH2_Handler            = (void*) Dummy_Handler,
  .pfnTC2_CH0_Handler            = (void*) Dummy_Handler,
  .pfnTC2_CH1_Handler            = (void*) Dummy_Handler,
  .pfnTC2_CH2_Handler            = (void*) Dummy_Handler,
  .pfnTC3_CH0_Handler            = (void*) Dummy_Handler,
  .pfnTC3_CH1_Handler            = (void*) Dummy_Handler,
  .pfnTC3_CH2_Handler            = (void*) Dummy_Handler,
  .pfnUSART0_Handler             = (void*) Dummy_Handler,
  .pfnUSART1_Handler             = (void*) Dummy_Handler,
  .pfnUSART2_Handler             = (void*) Dummy_Handler,
  .pfnPORTA_Handler              = (void*) Dummy_Handler,
  .pfnPORTB_Handler              = (void*) Dummy_Handler,
  .pfnPORTC_Handler              = (void*) Dummy_Handler,
  .pfnPORTD_Handler              = (void*) Dummy_Handler,
  .pfnPORTE_Handler              = (void*) Dummy_Handler,
  .pfnXDMAC_Handler              = (void*) Dummy_Handler,
  .pfnWDT_Handler                = (void*) Dummy_Handler,
  .pfnRSWDT_Handler              = (void*) Dummy_Handler,
  .pfnUSBHS_Handler              = (void*) USBHS_Handler,
};

/**
 * This is the code that gets called on processor reset.
 * - Initializes the data sections
 * - Optionally calls CMSIS SystemInit()
 * - Initializes the (nano-)newlib libc
 * - Then call main()
 */
void Reset_Handler(void)
{
  uint32_t *pSrc, *pDest;

  /* call present for MIPS, not sure it is needed here */
//  _on_reset();



  /* enable FPU */
  SCB->CPACR |= (0x3u << 20) | (0x3u << 22);
  __DSB();
  __ISB();

  /* Initialize the initialized data section */
  pSrc = &__etext;
  pDest = &__data_start__;

  if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
  {
    for (; pDest < &__data_end__ ; pDest++, pSrc++ )
    {
      *pDest = *pSrc ;
    }
  }

  /* Clear the zero section */
  if ( &__bss_start__ != &__bss_end__ )
  {
    for ( pDest = &__bss_start__ ; pDest < &__bss_end__ ; pDest++ )
    {
      *pDest = 0ul ;
    }
  }

  /* Call SYS_MEMORY init */
  /* todo() */


#if !defined DONT_USE_CMSIS_INIT
  /* Initialize the system */
  SystemInit() ;
#endif /* DONT_USE_CMSIS_INIT */

  /* calls _init() functions, C++ constructors included */
  __libc_init_array();

  /* Call the "on bootstrap" procedure */
//    _on_bootstrap();

  /* Branch to main function */
  main() ;

  /* Infinite loop */
#if (defined DEBUG && (DEBUG == 1)) || (defined __DEBUG)
  __BKPT(15);
#endif

  while (1)
  {
  }
}

/**
 * Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
  /* Software breakpoint */
#if (defined DEBUG && (DEBUG == 1)) || (defined __DEBUG)
  __BKPT(0);
#endif

	/* halt CPU */
  while (1)
  {
  }
}
