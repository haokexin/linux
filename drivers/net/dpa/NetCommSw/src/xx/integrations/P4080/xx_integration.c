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

/**************************************************************************//**
 @File          xx_integration.c

 @Description   XX routines implementation for Linux.
*//***************************************************************************/
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
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/smp_lock.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#include <linux/tqueue.h>
#else
#include <linux/workqueue.h>
#endif    /* LINUX_VERSION_CODE */

#ifdef BIGPHYSAREA_ENABLE
#include <linux/bigphysarea.h>
#endif /* BIGPHYSAREA_ENABLE */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <asm/of_device.h>
#include <sysdev/fsl_soc.h>
#endif    /* LINUX_VERSION_CODE */
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/string.h>
#include <asm/byteorder.h>
#include <asm/page.h>

#include "error_ext.h"
#include "std_ext.h"
//#include "mem_ext.h"
#include "list_ext.h"
#include "mm_ext.h"
#include "sys_io_ext.h"
#include "platform_ext.h"

#include "xx.h"


#define __ERR_MODULE__      MODULE_UNKNOWN

/*Forced to introduce due to PRINT_FMT_PARAMS define*/
uint32_t E500_GetId(void)
{
    return smp_processor_id();
}

void * XX_GetMemPartitionBase(int memPartitionId)
{
    switch(memPartitionId)
    {
        case(0):
        case(e_MEM_1ST_DDR_CACHEABLE):
            return phys_to_virt(0);
            break;
        default:
            REPORT_ERROR(MINOR, E_INVALID_VALUE, ("Memory type!"));
            return UINT_TO_PTR(ILLEGAL_BASE);
    }
    return UINT_TO_PTR(ILLEGAL_BASE);
}

char * GetDeviceName(int irq)
{
    switch (irq)
    {
//        case e_DUART1_INTR:
//            return "NCSW_DUART1";
        default:
            return NULL;
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
int GetDeviceIrqNum(int irq)
{
    struct device_node  *iPar;
    struct irq_host     *irqHost;
    uint32_t            hwIrq;

    /* Get the interrupt controller */
    iPar = of_find_node_by_name(NULL, "mpic");
    hwIrq = 0;

    ASSERT_COND(iPar != NULL);
    /* Get the irq host */
    irqHost = irq_find_host(iPar);
    of_node_put(iPar);

    /* Create irq mapping */
    return irq_create_mapping(irqHost, hwIrq);
}
#else
#error "kernel not supported!!!"
#endif    /* LINUX_VERSION_CODE */


void * XX_PhysToVirt(physAddress_t addr)
{
    return UINT_TO_PTR(SYS_PhysToVirt((uint64_t)addr));
}

physAddress_t XX_VirtToPhys(void * addr)
{
    return (physAddress_t)SYS_VirtToPhys(PTR_TO_UINT(addr));
}

/*****************************************************************************/
void * xx_MallocSmart(uint32_t size, int memPartitionId, uint32_t alignment)
{
    uintptr_t	*returnCode, tmp;

    switch(memPartitionId) {
       case(0):
       case(e_MEM_1ST_DDR_CACHEABLE):
            if (alignment < sizeof(uintptr_t))
                alignment = sizeof(uintptr_t);
            size += alignment + sizeof(returnCode);
            tmp = (uintptr_t)xx_Malloc(size);
            if (tmp == 0)
                return NULL;
            returnCode = (uintptr_t*)((tmp + alignment + sizeof(returnCode)) & ~((uintptr_t)alignment - 1));
            *(returnCode - 1) = tmp;
            break;
        default:
            XX_Print("XX_MallocSmart:Mem type not supported\r\n");
            return NULL;
    }
    return (void*)returnCode;
}

void xx_FreeSmart(void *p)
{
    xx_Free((void*)(*((uintptr_t *)(p) - 1)));
}


