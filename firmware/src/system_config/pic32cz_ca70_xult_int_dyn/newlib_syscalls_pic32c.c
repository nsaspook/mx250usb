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

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "arch/arm/devices_pic32c.h" /* for ARM CMSIS __BKPT() */

#ifdef __cplusplus
extern "C" {
#endif

#undef errno
extern int errno;
extern int __HeapStart;
extern int __ram_end__;

extern caddr_t _sbrk(int incr);
extern void _exit(int status);


extern caddr_t _sbrk(int incr)
{
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;
    int ramend = (int)&__ram_end__;

    if (heap == NULL)
    {
        heap = (unsigned char *)&__HeapStart;
    }
    prev_heap = heap;

    if (((int)prev_heap + incr) > ramend)
    {
        return (caddr_t) -1;
    }

    heap += incr;

    return (caddr_t) prev_heap;
}

extern void _exit(int status)
{
    /* Software breakpoint */
#ifdef DEBUG
//    asm("bkpt #0");
    __BKPT(0);
#endif

    /* halt CPU */
    while (1)
    {
    }
}


#ifdef __cplusplus
}
#endif
