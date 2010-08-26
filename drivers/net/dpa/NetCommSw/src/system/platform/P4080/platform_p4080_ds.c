/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/version.h>

#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#define MODVERSIONS
#endif
#ifdef MODVERSIONS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#include <linux/modversions.h>
#else
#include <config/modversions.h>
#endif    /* LINUX_VERSION_CODE */
#endif /* MODVERSIONS */

#include <linux/module.h>
#include <linux/kernel.h>

#include <sysdev/fsl_soc.h>

#include <asm/io.h>

#include "std_ext.h"
#include "error_ext.h"
#include "part_ext.h"
#include "string_ext.h"
#include "xx_ext.h"
#include "sys_io_ext.h"
#include "platform_p4080_ds_ext.h"


#define __ERR_MODULE__      MODULE_UNKNOWN


typedef struct t_Platform
{
    /* Platform-owned module handles */
    t_Handle                h_Part;
    /* Clocks-related variables */
    uint32_t                clockInFreqHz;
    uint32_t                localBusFreqHz;
} t_Platform;

#if 0
uint64_t ram_virt_addr;
uint64_t ram_phys_addr;
uint64_t ram_size;
#endif

t_Handle PLATFORM_Init(t_PlatformParam *p_PlatformParam)
{
    t_Platform *p_Platform = NULL;

    SANITY_CHECK_RETURN_VALUE(p_PlatformParam, E_NULL_POINTER, NULL);

    /* Allocate the platform's control structure */
    p_Platform = XX_Malloc(sizeof(t_Platform));
    if (!p_Platform)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Platform structure"));
        return NULL;
    }
    memset(p_Platform, 0, sizeof(t_Platform));

    /* Are these ok for all platforms? For ram, let's base on the system mapping. */
#if 0
    ram_virt_addr = (uintptr_t)phys_to_virt(0);
    ram_phys_addr = 0x0;
    ram_size      = 0x20000000;
    if (SYS_RegisterIoMap((uint64_t)ram_virt_addr, (uint64_t)ram_phys_addr, ram_size) != E_OK)
    {
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("RAM memory map"));
        return NULL;
    }
#endif
    return p_Platform;
}

t_Error PLATFORM_Free(t_Handle h_Platform)
{
    t_Platform *p_Platform = (t_Platform *)h_Platform;

    if (!p_Platform)
        RETURN_ERROR(MINOR, E_INVALID_HANDLE, NO_MSG);
#if 0
    SYS_UnregisterIoMap((uint64_t)ram_virt_addr);
#endif
    XX_Free(p_Platform);

    return E_OK;
}

