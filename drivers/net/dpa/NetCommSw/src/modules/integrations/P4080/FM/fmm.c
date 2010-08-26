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
 @File          fmm.c

 @Description   FM module
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
#endif /* LINUX_VERSION_CODE */
#endif /* MODVERSIONS */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/smp_lock.h>
#include <linux/uts.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <asm/pgtable.h>

#include "error_ext.h"
#include "std_ext.h"
#include "xx_ext.h"
#include "sys_ext.h"
#include "platform_p4080_ds_ext.h"
#include "lnxwrp_fm_ext.h"


#define __ERR_MODULE__      MODULE_P4080


#define DRIVER_AUTHOR   "Shlomi Gridish"
#define DRIVER_DESC     "Frame Manager module"
#define DRIVER_INFO     DRIVER_DESC


MODULE_DESCRIPTION (DRIVER_INFO);
MODULE_AUTHOR (DRIVER_AUTHOR);
MODULE_LICENSE ("Dual BSD/GPL");


typedef struct {
    t_Handle                    h_Pltfrm;
    t_Handle                    h_FmLnxWrp;
#ifdef CONFIG_FSL_FMAN_TEST
    t_Handle                    h_FmTestLnxWrp;
#endif /* CONFIG_FSL_FMAN_TEST */
} t_NcswModule;


#ifdef MODULE
extern int  __init __cold port_wrapper_load(void);
extern void __exit __cold port_wrapper_unload(void);
extern int  __init __cold mac_load(void);
extern void __exit __cold mac_unload(void);
extern int  __init __cold dpa_load(void);
extern void __exit __cold dpa_unload(void);
#endif /* MODULE */


static t_NcswModule mod;


#if 0
static t_Handle ModGetObject (t_Handle h_App, e_SysModule mod, ...)
{
    t_NcswModule    *p_Mod = (t_NcswModule *)h_App;
    t_Handle        obj = NULL;
    va_list         args;
    int             index;

    va_start(args, mod);
    index = va_arg(args, int);

    switch (mod)
    {
        case e_SYS_MODULE_PLATFORM:
            if (index == 0)
                obj = p_Mod->sys.p_Descriptors[e_DESC_INTEG];
            break;
        default:
            REPORT_ERROR(MINOR, E_INVALID_SELECTION, ("device-type!!!"));
            break;
    }
    va_end(args);

    if (!obj)
        REPORT_ERROR(MINOR, E_INVALID_HANDLE, ("device [%d,%d]!!!", mod,index));
    return obj;
}
#endif /* 0 */

static void FreeInitResources(t_NcswModule *p_Mod)
{
#ifdef CONFIG_FSL_FMAN_TEST
    if (p_Mod->h_FmTestLnxWrp)
        LNXWRP_FM_TEST_Free(p_Mod->h_FmTestLnxWrp);
#endif /* CONFIG_FSL_FMAN_TEST */

    if (p_Mod->h_FmLnxWrp)
        LNXWRP_FM_Free(p_Mod->h_FmLnxWrp);

    PLATFORM_Free(&p_Mod->h_Pltfrm);
}

/*-------------------------------------------------------------------------*/
static int __init __cold fm_load (void)
{
    t_PlatformParam     platformParam;

    memset(&platformParam, 0, sizeof(platformParam));
    if ((mod.h_Pltfrm = PLATFORM_Init(&platformParam)) == NULL)
    {
        printk("Failed to init general modules!\n");
        return -ENODEV;
    }

    if ((mod.h_FmLnxWrp = LNXWRP_FM_Init()) == NULL)
    {
        printk("Failed to init FM wrapper!\n");
        FreeInitResources(&mod);
        return -ENODEV;
    }

#ifdef MODULE
    if (port_wrapper_load() || mac_load() || dpa_load())
    {
        printk("Failed to init port-wrapper or MAC or DPA driver!\n");
        FreeInitResources(&mod);
        return -ENODEV;
    }
#endif /* MODULE */

#ifdef CONFIG_FSL_FMAN_TEST
    if ((mod.h_FmTestLnxWrp = LNXWRP_FM_TEST_Init()) == NULL)
    {
        printk("Failed to init FM-test wrapper!\n");
        FreeInitResources(&mod);
        return -ENODEV;
    }
#endif /* CONFIG_FSL_FMAN_TEST */

    printk (KERN_CRIT "Freescale FM module ("__DATE__ ":"__TIME__")\n");

    return 0;
}

static void __exit __cold fm_unload (void)
{
#ifdef MODULE
    dpa_unload();
    mac_unload();
    port_wrapper_unload();
#endif /* MODULE */

    FreeInitResources(&mod);
}

module_init (fm_load);
module_exit (fm_unload);
