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
 @File          lnxwrp_fm.c

 @Author        Shlomi Gridish

 @Description   FM Linux wrapper functions.
*//***************************************************************************/

/* Linux Headers ------------------- */
#include <linux/version.h>

#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#define MODVERSIONS
#endif
#ifdef MODVERSIONS
#include <config/modversions.h>
#endif /* MODVERSIONS */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/qe.h>        /* For struct qe_firmware */
#include <sysdev/fsl_soc.h>
#include <linux/stat.h>	   /* For file access mask */

/* NetCommSw Headers --------------- */
#include "std_ext.h"
#include "error_ext.h"
#include "sprint_ext.h"
#include "debug_ext.h"
#include "sys_io_ext.h"
#include "procbuff_ext.h"

#include "fm_ioctls.h"

#include "lnxwrp_fm.h"
#include "fm_port_ext.h"

#define PROC_PRINT(args...) offset += sprintf(buf+offset,args)

#define ADD_ADV_CONFIG_NO_RET(_func, _param)    \
    do {                                        \
        if (i<max){                             \
            p_Entry = &p_Entrys[i];             \
            p_Entry->p_Function = _func;        \
            _param                              \
            i++;                                \
        }                                       \
        else                                    \
            REPORT_ERROR(MAJOR, E_INVALID_VALUE,\
                         ("Number of advanced-configuration entries exceeded"));\
    } while (0)


static t_LnxWrpFm   lnxWrpFm;

typedef enum e_FmDmaMatchStatistics {
    e_FM_DMA_COUNTERS_CMQ_NOT_EMPTY,
    e_FM_DMA_COUNTERS_BUS_ERROR,
    e_FM_DMA_COUNTERS_READ_BUF_ECC_ERROR,
    e_FM_DMA_COUNTERS_WRITE_BUF_ECC_SYS_ERROR,
    e_FM_DMA_COUNTERS_WRITE_BUF_ECC_FM_ERROR
}e_FmDmaMatchStatistics;

/* FM PORT STATISTICS */
typedef struct{
    const char *statisticName;
    uint8_t statisticCounter;
}t_SysfsStats;

static const t_SysfsStats fmSysfsStats[] = {
    /* FM statistics */
    {
        .statisticName = "enq_total_frame",
        .statisticCounter = e_FM_COUNTERS_ENQ_TOTAL_FRAME,
    },
    {
        .statisticName = "deq_total_frame",
        .statisticCounter = e_FM_COUNTERS_DEQ_TOTAL_FRAME,
    },
    {
        .statisticName = "deq_0",
        .statisticCounter = e_FM_COUNTERS_DEQ_0,
    },
    {
        .statisticName = "deq_1",
        .statisticCounter = e_FM_COUNTERS_DEQ_1,
    },
    {
        .statisticName = "deq_2",
        .statisticCounter = e_FM_COUNTERS_DEQ_2,
    },
    {
        .statisticName = "deq_from_default",
        .statisticCounter = e_FM_COUNTERS_DEQ_FROM_DEFAULT,
    },
    {
        .statisticName = "deq_from_context",
        .statisticCounter = e_FM_COUNTERS_DEQ_FROM_CONTEXT,
    },
    {
        .statisticName = "deq_from_fd",
        .statisticCounter = e_FM_COUNTERS_DEQ_FROM_FD,
    },
    {
        .statisticName = "deq_confirm",
        .statisticCounter = e_FM_COUNTERS_DEQ_CONFIRM,
    },
    /* FM:DMA  statistics */
    {
        .statisticName = "cmq_not_empty",
        .statisticCounter = e_FM_DMA_COUNTERS_CMQ_NOT_EMPTY,
    },
    {
        .statisticName = "bus_error",
        .statisticCounter = e_FM_DMA_COUNTERS_BUS_ERROR,
    },
    {
        .statisticName = "read_buf_ecc_error",
        .statisticCounter = e_FM_DMA_COUNTERS_READ_BUF_ECC_ERROR,
    },
    {
        .statisticName = "write_buf_ecc_sys_error",
        .statisticCounter = e_FM_DMA_COUNTERS_WRITE_BUF_ECC_SYS_ERROR,
    },
    {
        .statisticName = "write_buf_ecc_fm_error",
        .statisticCounter = e_FM_DMA_COUNTERS_WRITE_BUF_ECC_FM_ERROR,
    },
    /* FM:PCD  statistics */
    {
        .statisticName = "pcd_enq_total_frame",
        .statisticCounter = e_FM_COUNTERS_ENQ_TOTAL_FRAME,
    },
    {
        .statisticName = "pcd_kg_total",
        .statisticCounter = e_FM_PCD_KG_COUNTERS_TOTAL,
    },
    {
        .statisticName = "pcd_plcr_yellow",
        .statisticCounter = e_FM_PCD_PLCR_COUNTERS_YELLOW,
    },
    {
        .statisticName = "pcd_plcr_red",
        .statisticCounter = e_FM_PCD_PLCR_COUNTERS_RED,
    },
    {
        .statisticName = "pcd_plcr_recolored_to_red",
        .statisticCounter = e_FM_PCD_PLCR_COUNTERS_RECOLORED_TO_RED,
    },
    {
        .statisticName = "pcd_plcr_recolored_to_yellow",
        .statisticCounter = e_FM_PCD_PLCR_COUNTERS_RECOLORED_TO_YELLOW,
    },
    {
        .statisticName = "pcd_plcr_total",
        .statisticCounter = e_FM_PCD_PLCR_COUNTERS_TOTAL,
    },
    {
        .statisticName = "pcd_plcr_length_mismatch",
        .statisticCounter = e_FM_PCD_PLCR_COUNTERS_LENGTH_MISMATCH,
    },
    {
        .statisticName = "pcd_prs_parse_dispatch",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_PARSE_DISPATCH,
    },
    {
        .statisticName = "pcd_prs_l2_parse_result_returned",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_L2_PARSE_RESULT_RETURNED,
    },
    {
        .statisticName = "pcd_prs_l3_parse_result_returned",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_L3_PARSE_RESULT_RETURNED,
    },
    {
        .statisticName = "pcd_prs_l4_parse_result_returned",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_L4_PARSE_RESULT_RETURNED,
    },
    {
        .statisticName = "pcd_prs_shim_parse_result_returned",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_SHIM_PARSE_RESULT_RETURNED,
    },
    {
        .statisticName = "pcd_prs_l2_parse_result_returned_with_err",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_L2_PARSE_RESULT_RETURNED_WITH_ERR,
    },
    {
        .statisticName = "pcd_prs_l3_parse_result_returned_with_err",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_L3_PARSE_RESULT_RETURNED_WITH_ERR,
    },
    {
        .statisticName = "pcd_prs_l4_parse_result_returned_with_err",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_L4_PARSE_RESULT_RETURNED_WITH_ERR,
    },
    {
        .statisticName = "pcd_prs_shim_parse_result_returned_with_err",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_SHIM_PARSE_RESULT_RETURNED_WITH_ERR,
    },
    {
        .statisticName = "pcd_prs_soft_prs_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_SOFT_PRS_CYCLES,
    },
    {
        .statisticName = "pcd_prs_soft_prs_stall_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_SOFT_PRS_STALL_CYCLES,
    },
    {
        .statisticName = "pcd_prs_hard_prs_cycle_incl_stall_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_HARD_PRS_CYCLE_INCL_STALL_CYCLES,
    },
    {
        .statisticName = "pcd_prs_muram_read_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_MURAM_READ_CYCLES,
    },
    {
        .statisticName = "pcd_prs_muram_read_stall_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_MURAM_READ_STALL_CYCLES,
    },
    {
        .statisticName = "pcd_prs_muram_write_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_MURAM_WRITE_CYCLES,
    },
    {
        .statisticName = "pcd_prs_muram_write_stall_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_MURAM_WRITE_STALL_CYCLES,
    },
    {
        .statisticName = "pcd_prs_fpm_command_stall_cycles",
        .statisticCounter = e_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES,
    },
    {}
};

static const t_SysfsStats portSysfsStats[] = {
    /* RX/TX/OH common statistics */
    {
        .statisticName = "port_frame",
        .statisticCounter = e_FM_PORT_COUNTERS_FRAME,
    },
    {
        .statisticName = "port_discard_frame",
        .statisticCounter = e_FM_PORT_COUNTERS_DISCARD_FRAME,
    },
    {
        .statisticName = "port_dealloc_buf",
        .statisticCounter = e_FM_PORT_COUNTERS_DEALLOC_BUF,
    },
    {
        .statisticName = "port_enq_total",
        .statisticCounter = e_FM_PORT_COUNTERS_ENQ_TOTAL,
    },
    /* TX/OH */
    {
        .statisticName = "port_length_err",
        .statisticCounter = e_FM_PORT_COUNTERS_LENGTH_ERR,
    },
    {
        .statisticName = "port_unsupprted_format",
        .statisticCounter = e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT,
    },
    {
        .statisticName = "port_deq_total",
        .statisticCounter = e_FM_PORT_COUNTERS_DEQ_TOTAL,
    },
    {
        .statisticName = "port_deq_from_default",
        .statisticCounter = e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT,
    },
    {
        .statisticName = "port_deq_confirm",
        .statisticCounter = e_FM_PORT_COUNTERS_DEQ_CONFIRM,
    },
    /* RX/OH */
    {
        .statisticName = "port_rx_bad_frame",
        .statisticCounter = e_FM_PORT_COUNTERS_RX_BAD_FRAME,
    },
    {
        .statisticName = "port_rx_large_frame",
        .statisticCounter = e_FM_PORT_COUNTERS_RX_LARGE_FRAME ,
    },
    {
        .statisticName = "port_rx_out_of_buffers_discard",
        .statisticCounter = e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD,
    },
    {
        .statisticName = "port_rx_filter_frame",
        .statisticCounter = e_FM_PORT_COUNTERS_RX_FILTER_FRAME,
    },
    /* TODO: Particular statistics for OH ports */
    {}
};

static uint8_t find_fm_statistic_counter_by_name(const char *attr_name, t_SysfsStats *sysfs_stats, uint8_t *offset)
{
    int i = 0;

    while(sysfs_stats[i].statisticName != NULL)
    {
        if(strcmp(sysfs_stats[i].statisticName, attr_name) == 0){
            if(offset != NULL)
                *offset = i;
            return sysfs_stats[i].statisticCounter;
        }

        i++;
    }
    /* Should never get here! */
    BUG();
    return 0;
}

static ssize_t show_fm_port_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
    t_LnxWrpFmPortDev           *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)dev_get_drvdata(dev);
    t_LnxWrpFmDev               *p_LnxWrpFmDev     = NULL;
    unsigned long               flags;
    int                         n = 0;
    uint8_t                     counter = 0;

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

    if (p_LnxWrpFmPortDev == NULL)
        BUG();

    p_LnxWrpFmDev = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
    if (p_LnxWrpFmDev == NULL)
        BUG();

    if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_Dev)
        return -EIO;

    if (!p_LnxWrpFmPortDev->h_Dev)
    {
        n = snprintf( buf, PAGE_SIZE,
            "\tFM Port not configured... \n");
        return n;
    }

    counter = find_fm_statistic_counter_by_name(attr->attr.name, (t_SysfsStats *)&portSysfsStats[0], NULL);

    if(counter == e_FM_PORT_COUNTERS_RX_LIST_DMA_ERR){
        uint32_t fmRev = 0;
        fmRev = (uint32_t)(*((volatile uint32_t *)UINT_TO_PTR(p_LnxWrpFmDev->fmBaseAddr+0x000c30c4))& 0xffff);
        local_irq_save(flags);

        if (fmRev == 0x0100)
            n = snprintf( buf, PAGE_SIZE, "counter not available for revision 1\n");

        local_irq_restore(flags);
        return n;
    }

    local_irq_save(flags);
    n = snprintf( buf, PAGE_SIZE, "\tFM %d Port %d counter: %d\n",
            p_LnxWrpFmDev->id,
            p_LnxWrpFmPortDev->id,
            FM_PORT_GetCounter(p_LnxWrpFmPortDev->h_Dev, (e_FmPortCounters)counter));
    local_irq_restore(flags);

    return n;
}

/* FM PORT RX/TX/OH statistics */
static DEVICE_ATTR(port_frame                       , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_discard_frame               , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_dealloc_buf                 , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_enq_total                   , S_IRUGO, show_fm_port_stats, NULL);
/* FM PORT TX/OH statistics */
static DEVICE_ATTR(port_length_err                  , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_unsupprted_format           , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_deq_total                   , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_deq_from_default            , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_deq_confirm                 , S_IRUGO, show_fm_port_stats, NULL);
/* FM PORT RX/OH statistics */
static DEVICE_ATTR(port_rx_bad_frame                , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_rx_large_frame              , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_rx_out_of_buffers_discard   , S_IRUGO, show_fm_port_stats, NULL);
static DEVICE_ATTR(port_rx_filter_frame             , S_IRUGO, show_fm_port_stats, NULL);

/* FM PORT TX statistics */
static struct attribute *fm_tx_port_dev_stats_attributes[] = {
    &dev_attr_port_frame.attr,
    &dev_attr_port_discard_frame.attr,
    &dev_attr_port_dealloc_buf.attr,
    &dev_attr_port_enq_total.attr,
    &dev_attr_port_length_err.attr,
    &dev_attr_port_unsupprted_format.attr,
    &dev_attr_port_deq_total.attr,
    &dev_attr_port_deq_from_default.attr,
    &dev_attr_port_deq_confirm.attr,
    NULL
};

static const struct attribute_group fm_tx_port_dev_stats_attr_grp = {
    .name = "statistics",
    .attrs = fm_tx_port_dev_stats_attributes
};

/* FM PORT RX statistics */
static struct attribute *fm_rx_port_dev_stats_attributes[] = {
    &dev_attr_port_frame.attr,
    &dev_attr_port_discard_frame.attr,
    &dev_attr_port_dealloc_buf.attr,
    &dev_attr_port_enq_total.attr,
    &dev_attr_port_rx_bad_frame.attr,
    &dev_attr_port_rx_large_frame.attr,
    &dev_attr_port_rx_out_of_buffers_discard.attr,
    &dev_attr_port_rx_filter_frame.attr,
    NULL
};

static const struct attribute_group fm_rx_port_dev_stats_attr_grp = {
    .name = "statistics",
    .attrs = fm_rx_port_dev_stats_attributes
};

/* TODO: add particular OH ports statistics */
static struct attribute *fm_oh_port_dev_stats_attributes[] = {
    &dev_attr_port_frame.attr,
    &dev_attr_port_discard_frame.attr,
    &dev_attr_port_dealloc_buf.attr,
    &dev_attr_port_enq_total.attr,
    /*TX*/
    &dev_attr_port_length_err.attr,
    &dev_attr_port_unsupprted_format.attr,
    &dev_attr_port_deq_total.attr,
    &dev_attr_port_deq_from_default.attr,
    &dev_attr_port_deq_confirm.attr,
    /*RX*/
    &dev_attr_port_rx_bad_frame.attr,
    &dev_attr_port_rx_large_frame.attr,
    &dev_attr_port_rx_out_of_buffers_discard.attr,
    /*&dev_attr_port_rx_filter_frame.attr,*/
    NULL
};

static const struct attribute_group fm_oh_port_dev_stats_attr_grp = {
    .name = "statistics",
    .attrs = fm_oh_port_dev_stats_attributes
};

static ssize_t show_fm_port_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long   flags;
    unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)dev_get_drvdata(dev);
#endif

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    local_irq_save(flags);

    if (!p_LnxWrpFmPortDev->h_Dev){
        n = snprintf( buf, PAGE_SIZE,
            "\tFM Port not configured... \n");
        return n;
    }
    else{
        n = snprintf(buf, PAGE_SIZE, "FM port driver registers dump.\n");
        FM_PORT_DumpRegs(p_LnxWrpFmPortDev->h_Dev);
    }

    local_irq_restore(flags);

    return n;
#else

    local_irq_save(flags);
    n = snprintf(buf, PAGE_SIZE, "Debug level is too low to dump registers!!!\n");
    local_irq_restore(flags);

    return n;
#endif
}

static DEVICE_ATTR(fm_port_regs,0x644,show_fm_port_regs,NULL);

static int fm_port_sysfs_create(struct device *dev)
{
    t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = NULL;

    if(dev == NULL)
        return -EINVAL;

    p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmPortDev == NULL)
        BUG();

    /* store to remove them when module si disabled */
    p_LnxWrpFmPortDev->dev_attr_regs = &dev_attr_fm_port_regs;

    /* Registers dump entry - in future will be moved to debugfs */
    if(device_create_file(dev,&dev_attr_fm_port_regs) !=0 )
         return -EIO;

    /* FM Ports statistics */
    switch(p_LnxWrpFmPortDev->settings.param.portType){
        case e_FM_PORT_TYPE_TX:
        case e_FM_PORT_TYPE_TX_10G:
            if (sysfs_create_group(&dev->kobj, &fm_tx_port_dev_stats_attr_grp) !=0)
                return -EIO;
        break;
        case e_FM_PORT_TYPE_RX:
        case e_FM_PORT_TYPE_RX_10G :
            if (sysfs_create_group(&dev->kobj, &fm_rx_port_dev_stats_attr_grp) !=0)
                return -EIO;
        break;
        case e_FM_PORT_TYPE_OH_OFFLINE_PARSING :
        case e_FM_PORT_TYPE_OH_HOST_COMMAND :
            if (sysfs_create_group(&dev->kobj, &fm_oh_port_dev_stats_attr_grp) !=0)
                return -EIO;
        break;
        case e_FM_PORT_TYPE_DUMMY:
        default:
            BUG();
        break;
    };

    return 0;
}

static void fm_port_sysfs_destroy(struct device *dev)
{
    t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = NULL;

    /* this function has never been tested !!! */

    if(dev == NULL)
        BUG();

    p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmPortDev == NULL)
        BUG();

    /* The name attribute will be freed also by these 2 functions? Hell knows... */
    switch(p_LnxWrpFmPortDev->settings.param.portType){
        case e_FM_PORT_TYPE_TX:
        case e_FM_PORT_TYPE_TX_10G:
            sysfs_remove_group(&dev->kobj, &fm_tx_port_dev_stats_attr_grp);
        break;
        case e_FM_PORT_TYPE_RX:
        case e_FM_PORT_TYPE_RX_10G :
            sysfs_remove_group(&dev->kobj, &fm_rx_port_dev_stats_attr_grp);
        break;
        case e_FM_PORT_TYPE_OH_OFFLINE_PARSING :
        case e_FM_PORT_TYPE_OH_HOST_COMMAND :
            /* Not supported yet... */
            printk( KERN_WARNING "Statistics for port type not supported... \n");
        break;
        case e_FM_PORT_TYPE_DUMMY :
        default :
            BUG();
        break;
    };

    device_remove_file(dev, p_LnxWrpFmPortDev->dev_attr_regs);
}

/* Fm stats and regs dumps via sysfs */
static ssize_t show_fm_dma_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
    t_LnxWrpFmDev *p_LnxWrpFmDev = NULL;
    t_FmDmaStatus  fmDmaStatus;
    unsigned long  flags = 0;
    unsigned n = 0;
    uint8_t counter_value = 0, counter = 0;

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmDev == NULL)
        BUG();

    if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_Dev)
        return -EIO;

    counter = find_fm_statistic_counter_by_name(attr->attr.name, (t_SysfsStats *)&fmSysfsStats[0], NULL);

    local_irq_save(flags);

    memset(&fmDmaStatus, 0, sizeof(fmDmaStatus));
    FM_GetDmaStatus(p_LnxWrpFmDev->h_Dev, &fmDmaStatus);

    switch(counter){
        case e_FM_DMA_COUNTERS_CMQ_NOT_EMPTY:
            counter_value = fmDmaStatus.cmqNotEmpty;
        break;
        case e_FM_DMA_COUNTERS_BUS_ERROR:
            counter_value = fmDmaStatus.busError;
        break;
        case e_FM_DMA_COUNTERS_READ_BUF_ECC_ERROR:
            counter_value = fmDmaStatus.readBufEccError;
        break;
        case e_FM_DMA_COUNTERS_WRITE_BUF_ECC_SYS_ERROR:
            counter_value = fmDmaStatus.writeBufEccSysError;
        break;
        case e_FM_DMA_COUNTERS_WRITE_BUF_ECC_FM_ERROR:
            counter_value = fmDmaStatus.writeBufEccFmError;
        break;
        default:
            BUG();
        break;
    };

    n = snprintf( buf, PAGE_SIZE, "\tFM %u counter: %c\n",
            p_LnxWrpFmDev->id,
            counter_value?'T':'F' );

    local_irq_restore(flags);

    return n;
}

static ssize_t show_fm_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
    t_LnxWrpFmDev *p_LnxWrpFmDev = NULL;
    unsigned long  flags = 0;
    unsigned n = 0, counter = 0;

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmDev == NULL)
        BUG();

    if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_Dev)
        return -EIO;

    counter = find_fm_statistic_counter_by_name(attr->attr.name, (t_SysfsStats *)&fmSysfsStats[0], NULL);

    local_irq_save(flags);

    n = snprintf( buf, PAGE_SIZE, "\tFM %d counter: %d\n",
            p_LnxWrpFmDev->id,
            FM_GetCounter(p_LnxWrpFmDev->h_Dev, (e_FmCounters)counter));

    local_irq_restore(flags);

    return n;
}

static ssize_t show_fm_pcd_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
    t_LnxWrpFmDev *p_LnxWrpFmDev = NULL;
    unsigned long  flags = 0;
    unsigned n = 0, counter = 0;

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmDev == NULL)
        BUG();

    if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_Dev)
        return -EIO;

    counter = find_fm_statistic_counter_by_name(attr->attr.name, (t_SysfsStats *)&fmSysfsStats[0], NULL);

    local_irq_save(flags);

    n = snprintf( buf, PAGE_SIZE, "\tFM %d counter: %d\n",
            p_LnxWrpFmDev->id,
            FM_PCD_GetCounter(p_LnxWrpFmDev->h_PcdDev, (e_FmPcdCounters)counter));

    local_irq_restore(flags);

    return n;
}

/* FM */
static DEVICE_ATTR(enq_total_frame              , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_total_frame              , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_0                        , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_1                        , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_2                        , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_from_default             , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_from_context             , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_from_fd                  , S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_confirm                  , S_IRUGO, show_fm_stats, NULL);
/* FM:DMA */
static DEVICE_ATTR(cmq_not_empty            , S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(bus_error                , S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(read_buf_ecc_error       , S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(write_buf_ecc_sys_error  , S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(write_buf_ecc_fm_error   , S_IRUGO, show_fm_dma_stats, NULL);
/* FM:PCD */
static DEVICE_ATTR(pcd_enq_total_frame                          , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_kg_total                                 , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_yellow                              , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_red                                 , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_recolored_to_red                    , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_recolored_to_yellow                 , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_total                               , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_length_mismatch                     , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_parse_dispatch                       , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l2_parse_result_returned             , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l3_parse_result_returned             , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l4_parse_result_returned             , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_shim_parse_result_returned           , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l2_parse_result_returned_with_err    , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l3_parse_result_returned_with_err    , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l4_parse_result_returned_with_err    , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_shim_parse_result_returned_with_err  , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_soft_prs_cycles                      , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_soft_prs_stall_cycles                , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_hard_prs_cycle_incl_stall_cycles     , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_muram_read_cycles                    , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_muram_read_stall_cycles              , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_muram_write_cycles                   , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_muram_write_stall_cycles             , S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_fpm_command_stall_cycles             , S_IRUGO, show_fm_pcd_stats, NULL);

static struct attribute *fm_dev_stats_attributes[] = {
    &dev_attr_enq_total_frame.attr,
    &dev_attr_deq_total_frame.attr,
    &dev_attr_deq_0.attr,
    &dev_attr_deq_1.attr,
    &dev_attr_deq_2.attr,
    &dev_attr_deq_from_default.attr,
    &dev_attr_deq_from_context.attr,
    &dev_attr_deq_from_fd.attr,
    &dev_attr_deq_confirm.attr,
    &dev_attr_cmq_not_empty.attr,
    &dev_attr_bus_error.attr,
    &dev_attr_read_buf_ecc_error.attr,
    &dev_attr_write_buf_ecc_sys_error.attr,
    &dev_attr_write_buf_ecc_fm_error.attr,
    &dev_attr_pcd_enq_total_frame.attr,
    &dev_attr_pcd_kg_total.attr,
    &dev_attr_pcd_plcr_yellow.attr,
    &dev_attr_pcd_plcr_red.attr,
    &dev_attr_pcd_plcr_recolored_to_red.attr,
    &dev_attr_pcd_plcr_recolored_to_yellow.attr,
    &dev_attr_pcd_plcr_total.attr,
    &dev_attr_pcd_plcr_length_mismatch.attr,
    &dev_attr_pcd_prs_parse_dispatch.attr,
    &dev_attr_pcd_prs_l2_parse_result_returned.attr,
    &dev_attr_pcd_prs_l3_parse_result_returned.attr,
    &dev_attr_pcd_prs_l4_parse_result_returned.attr,
    &dev_attr_pcd_prs_shim_parse_result_returned.attr,
    &dev_attr_pcd_prs_l2_parse_result_returned_with_err.attr,
    &dev_attr_pcd_prs_l3_parse_result_returned_with_err.attr,
    &dev_attr_pcd_prs_l4_parse_result_returned_with_err.attr,
    &dev_attr_pcd_prs_shim_parse_result_returned_with_err.attr,
    &dev_attr_pcd_prs_soft_prs_cycles.attr,
    &dev_attr_pcd_prs_soft_prs_stall_cycles.attr,
    &dev_attr_pcd_prs_hard_prs_cycle_incl_stall_cycles.attr,
    &dev_attr_pcd_prs_muram_read_cycles.attr,
    &dev_attr_pcd_prs_muram_read_stall_cycles.attr,
    &dev_attr_pcd_prs_muram_write_cycles.attr,
    &dev_attr_pcd_prs_muram_write_stall_cycles.attr,
    &dev_attr_pcd_prs_fpm_command_stall_cycles.attr,
    NULL
};

static const struct attribute_group fm_dev_stats_attr_grp = {
    .name = "statistics",
    .attrs = fm_dev_stats_attributes
};

static ssize_t show_fm_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long   flags;
    unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    t_LnxWrpFmDev   *p_LnxWrpFmDev = NULL;
#endif

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmDev == NULL)
        BUG();

    local_irq_save(flags);

    n = snprintf(buf, PAGE_SIZE, "FM driver registers dump.\n");

    if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_Dev)
        return -EIO;
    else
        FM_DumpRegs(p_LnxWrpFmDev->h_Dev);

    local_irq_restore(flags);
#else

    local_irq_save(flags);
    n = snprintf(buf, PAGE_SIZE, "Debug level is too low to dump registers!!!\n");
    local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

    return n;
}

static ssize_t show_pcd_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long   flags;
    unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    t_LnxWrpFmDev   *p_LnxWrpFmDev = NULL;
#endif

    if (attr == NULL || buf == NULL || dev == NULL)
        return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmDev == NULL)
        BUG();

    local_irq_save(flags);
    n = snprintf(buf, PAGE_SIZE, "FM driver registers dump.\n");

    if (!p_LnxWrpFmDev->active || !p_LnxWrpFmDev->h_PcdDev)
        return -EIO;
    else
        FM_PCD_DumpRegs(p_LnxWrpFmDev->h_PcdDev);

    local_irq_restore(flags);
#else

    local_irq_save(flags);
    n = snprintf(buf, PAGE_SIZE,  "Debug level is too low to dump registers!!!\n");
    local_irq_restore(flags);

#endif /* (defined(DEBUG_ERRORS) && ... */

    return n;
}

static DEVICE_ATTR(fm_regs, S_IRUGO, show_fm_regs, NULL);
static DEVICE_ATTR(fm_pcd_regs, S_IRUGO, show_pcd_regs, NULL);

static int fm_sysfs_create(struct device *dev)
{
    t_LnxWrpFmDev *p_LnxWrpFmDev = NULL;

    if(dev == NULL)
        return -EIO;

    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);

    /* store to remove them when module si disabled */
    p_LnxWrpFmDev->dev_attr_regs = &dev_attr_fm_regs;
    p_LnxWrpFmDev->dev_pcd_attr_regs = &dev_attr_fm_pcd_regs;

    /* Create sysfs statistics group for FM module */
    if (sysfs_create_group(&dev->kobj, &fm_dev_stats_attr_grp) !=0)
        return -EIO;

    /* Registers dump entry - in future will be moved to debugfs */
    if (device_create_file(dev,&dev_attr_fm_regs) != 0 ||
        device_create_file(dev,&dev_attr_fm_pcd_regs) != 0)
        return -EIO;

    return 0;
}

static void fm_sysfs_distroy(struct device *dev)
{
    t_LnxWrpFmDev *p_LnxWrpFmDev = NULL;

    if(dev == NULL)
        BUG();

    p_LnxWrpFmDev = (t_LnxWrpFmDev*)dev_get_drvdata(dev);
    if(p_LnxWrpFmDev == NULL)
        BUG();

    /* this function has never been tested !!! */
    sysfs_remove_group(&dev->kobj, &fm_dev_stats_attr_grp);
    device_remove_file(dev,p_LnxWrpFmDev->dev_attr_regs);
}

static irqreturn_t fm_irq(int irq, void *_dev)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)_dev;

    if (!p_LnxWrpFmDev || !p_LnxWrpFmDev->h_Dev)
        return IRQ_NONE;

    FM_EventIsr(p_LnxWrpFmDev->h_Dev);

    return IRQ_HANDLED;
}

static irqreturn_t fm_err_irq(int irq, void *_dev)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)_dev;

    if (!p_LnxWrpFmDev || !p_LnxWrpFmDev->h_Dev)
        return IRQ_NONE;

    if (FM_ErrorIsr(p_LnxWrpFmDev->h_Dev) == E_OK)
        return IRQ_HANDLED;

    return IRQ_NONE;
}

static volatile int   hcFrmRcv = 0;
static spinlock_t     lock;
/* used to protect FMD/LLD from concurent calls in functions fm_mutex_lock / fm_mutex_unlock */
static struct mutex   lnxwrp_mutex;

static enum qman_cb_dqrr_result qm_tx_conf_dqrr_cb(struct qman_portal          *portal,
                                                   struct qman_fq              *fq,
                                                   const struct qm_dqrr_entry  *dq)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = ((t_FmTestFq *)fq)->h_Arg;
    unsigned long flags;

    FM_PCD_HcTxConf(p_LnxWrpFmDev->h_PcdDev, (t_DpaaFD *)&dq->fd);
    spin_lock_irqsave(&lock, flags);
    hcFrmRcv--;
    spin_unlock_irqrestore(&lock, flags);

    return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result qm_tx_dqrr_cb(struct qman_portal          *portal,
                                              struct qman_fq              *fq,
                                              const struct qm_dqrr_entry  *dq)
{
    BUG();
    return qman_cb_dqrr_consume;
}

static void qm_err_cb(struct qman_portal       *portal,
                       struct qman_fq           *fq,
                       const struct qm_mr_entry *msg)
{
    BUG();
}

static struct qman_fq * FqAlloc(t_LnxWrpFmDev   *p_LnxWrpFmDev,
                                uint32_t        fqid,
                                uint32_t        flags,
                                uint16_t        channel,
                                uint8_t         wq)
{
    int                     _errno;
    struct qman_fq          *fq = NULL;
    t_FmTestFq              *p_FmtFq;
    struct qm_mcc_initfq    initfq;

    p_FmtFq = (t_FmTestFq *)XX_Malloc(sizeof(t_FmTestFq));
    if (!p_FmtFq) {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FQ obj!!!"));
        return NULL;
    }

    p_FmtFq->fq_base.cb.dqrr = (QMAN_FQ_FLAG_NO_ENQUEUE ? qm_tx_conf_dqrr_cb : qm_tx_dqrr_cb);
    p_FmtFq->fq_base.cb.ern = p_FmtFq->fq_base.cb.dc_ern = p_FmtFq->fq_base.cb.fqs = qm_err_cb;
    p_FmtFq->h_Arg = (t_Handle)p_LnxWrpFmDev;
    if (fqid == 0) {
        flags |= QMAN_FQ_FLAG_DYNAMIC_FQID;
        flags &= ~QMAN_FQ_FLAG_NO_MODIFY;
    } else {
        flags &= ~QMAN_FQ_FLAG_DYNAMIC_FQID;
    }

    if (qman_create_fq(fqid, flags, &p_FmtFq->fq_base)) {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FQ obj - qman_new_fq!!!"));
        XX_Free(p_FmtFq);
        return NULL;
    }
    fq = &p_FmtFq->fq_base;

    if (!(flags & QMAN_FQ_FLAG_NO_MODIFY)) {
        initfq.we_mask            = QM_INITFQ_WE_DESTWQ;
        initfq.fqd.dest.channel   = channel;
        initfq.fqd.dest.wq        = wq;

        _errno = qman_init_fq(fq, QMAN_INITFQ_FLAG_SCHED, &initfq);
        if (unlikely(_errno < 0)) {
            REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FQ obj - qman_init_fq!!!"));
            qman_destroy_fq(fq, 0);
            XX_Free(p_FmtFq);
            return NULL;
        }
    }

    DBG(TRACE, ("fqid %d, flags 0x%08x, channel %d, wq %d",qman_fq_fqid(fq),flags,channel,wq));

    return fq;
}

static t_Error QmEnqueueCB (t_Handle h_Arg, void *p_Fd)
{
    t_LnxWrpFmDev   *p_LnxWrpFmDev = (t_LnxWrpFmDev*)h_Arg;
    int             _errno, timeout=1000000;
    unsigned long flags;

    ASSERT_COND(p_LnxWrpFmDev);

    spin_lock_irqsave(&lock, flags);
    hcFrmRcv++;
    spin_unlock_irqrestore(&lock, flags);
//MemDisp((uint8_t*)p_Fd,sizeof(t_DpaaFD));
    _errno = qman_enqueue(p_LnxWrpFmDev->hc_tx_fq, (struct qm_fd*)p_Fd, 0);
    if (_errno)
        RETURN_ERROR(MINOR, E_INVALID_STATE, ("qman_enqueue() failed"));

    while (hcFrmRcv && --timeout)
    {
        udelay(1);
        cpu_relax();
    }
    if(timeout == 0)
    {
        dump_stack();
        RETURN_ERROR(MINOR, E_WRITE_FAILED, ("timeout waiting for Tx confirmation"));
        return E_WRITE_FAILED;
    }

    return E_OK;
}

static t_LnxWrpFmDev * CreateFmDev(uint8_t  id)
{
    t_LnxWrpFmDev   *p_LnxWrpFmDev;
    int             j;

    p_LnxWrpFmDev = (t_LnxWrpFmDev *)XX_Malloc(sizeof(t_LnxWrpFmDev));
    if (!p_LnxWrpFmDev)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, NO_MSG);
        return NULL;
    }

    memset(p_LnxWrpFmDev, 0, sizeof(t_LnxWrpFmDev));
    p_LnxWrpFmDev->fmDevSettings.advConfig = (t_SysObjectAdvConfigEntry*)XX_Malloc(FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry));
    memset(p_LnxWrpFmDev->fmDevSettings.advConfig, 0, (FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry)));
    p_LnxWrpFmDev->fmPcdDevSettings.advConfig = (t_SysObjectAdvConfigEntry*)XX_Malloc(FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry));
    memset(p_LnxWrpFmDev->fmPcdDevSettings.advConfig, 0, (FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry)));
    p_LnxWrpFmDev->hcPort.settings.advConfig = (t_SysObjectAdvConfigEntry*)XX_Malloc(FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry));
    memset(p_LnxWrpFmDev->hcPort.settings.advConfig, 0, (FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry)));
    for (j=0; j<FM_MAX_NUM_OF_RX_PORTS; j++)
    {
        p_LnxWrpFmDev->rxPorts[j].settings.advConfig = (t_SysObjectAdvConfigEntry*)XX_Malloc(FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry));
        memset(p_LnxWrpFmDev->rxPorts[j].settings.advConfig, 0, (FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry)));
    }
    for (j=0; j<FM_MAX_NUM_OF_TX_PORTS; j++)
    {
        p_LnxWrpFmDev->txPorts[j].settings.advConfig = (t_SysObjectAdvConfigEntry*)XX_Malloc(FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry));
        memset(p_LnxWrpFmDev->txPorts[j].settings.advConfig, 0, (FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry)));
    }
    for (j=0; j<FM_MAX_NUM_OF_OH_PORTS-1; j++)
    {
        p_LnxWrpFmDev->opPorts[j].settings.advConfig = (t_SysObjectAdvConfigEntry*)XX_Malloc(FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry));
        memset(p_LnxWrpFmDev->opPorts[j].settings.advConfig, 0, (FM_MAX_NUM_OF_ADV_SETTINGS*sizeof(t_SysObjectAdvConfigEntry)));
    }

    return p_LnxWrpFmDev;
}

static void DistroyFmDev(t_LnxWrpFmDev *p_LnxWrpFmDev)
{
    int             j;

    for (j=0; j<FM_MAX_NUM_OF_OH_PORTS-1; j++)
        if (p_LnxWrpFmDev->opPorts[j].settings.advConfig)
            XX_Free(p_LnxWrpFmDev->opPorts[j].settings.advConfig);
    for (j=0; j<FM_MAX_NUM_OF_TX_PORTS; j++)
        if (p_LnxWrpFmDev->txPorts[j].settings.advConfig)
            XX_Free(p_LnxWrpFmDev->txPorts[j].settings.advConfig);
    for (j=0; j<FM_MAX_NUM_OF_RX_PORTS; j++)
        if (p_LnxWrpFmDev->rxPorts[j].settings.advConfig)
            XX_Free(p_LnxWrpFmDev->rxPorts[j].settings.advConfig);
    if (p_LnxWrpFmDev->hcPort.settings.advConfig)
        XX_Free(p_LnxWrpFmDev->hcPort.settings.advConfig);
    if (p_LnxWrpFmDev->fmPcdDevSettings.advConfig)
        XX_Free(p_LnxWrpFmDev->fmPcdDevSettings.advConfig);
    if (p_LnxWrpFmDev->fmDevSettings.advConfig)
        XX_Free(p_LnxWrpFmDev->fmDevSettings.advConfig);

#ifdef NO_OF_SUPPORT
    memset(p_LnxWrpFmDev, 0, sizeof(t_LnxWrpFmDev));
#else
    XX_Free(p_LnxWrpFmDev);
#endif /* NO_OF_SUPPORT */
}

static t_Error FillRestFmInfo(t_LnxWrpFmDev *p_LnxWrpFmDev)
{
#define FM_BMI_PPIDS_OFFSET                 0x00080304
#define FM_DMA_PLR_OFFSET                   0x000c2060
#define FM_FPM_IP_REV_1_OFFSET              0x000c30c4
#define DMA_HIGH_LIODN_MASK                 0x0FFF0000
#define DMA_LOW_LIODN_MASK                  0x00000FFF
#define DMA_LIODN_SHIFT                     16

typedef _Packed struct {
    uint32_t    plr[32];
} _PackedType t_Plr;

typedef _Packed struct {
   volatile uint32_t   fmbm_ppid[63];
} _PackedType t_Ppids;

    t_Plr       *p_Plr;
    t_Ppids     *p_Ppids;
    int         i,j;
    uint32_t    fmRev;

    static const uint8_t     phys1GRxPortId[] = {0x8,0x9,0xa,0xb,0xc};
    static const uint8_t     phys10GRxPortId[] = {0x10};
    static const uint8_t     physOhPortId[] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7};
    static const uint8_t     phys1GTxPortId[] = {0x28,0x29,0x2a,0x2b,0x2c};
    static const uint8_t     phys10GTxPortId[] = {0x30};

    fmRev = (uint32_t)(*((volatile uint32_t *)UINT_TO_PTR(p_LnxWrpFmDev->fmBaseAddr+FM_FPM_IP_REV_1_OFFSET)));
    fmRev &= 0xffff;

    p_Plr = (t_Plr *)UINT_TO_PTR(p_LnxWrpFmDev->fmBaseAddr+FM_DMA_PLR_OFFSET);
#ifdef MODULE
    for (i=0;i<FM_MAX_NUM_OF_PARTITIONS/2;i++)
        p_Plr->plr[i] = 0;
#endif /* MODULE */

    for (i=0; i<FM_MAX_NUM_OF_PARTITIONS; i++)
    {
        uint16_t liodnBase = (uint16_t)((i%2) ?
                       (p_Plr->plr[i/2] & DMA_LOW_LIODN_MASK) :
                       ((p_Plr->plr[i/2] & DMA_HIGH_LIODN_MASK) >> DMA_LIODN_SHIFT));
#ifdef FM_PARTITION_ARRAY
        /* TODO: this was .liodnPerPartition[i] = liodnBase; is the index meaning the same? */
        p_LnxWrpFmDev->fmDevSettings.param.liodnBasePerPort[i] = liodnBase;
#endif /* FM_PARTITION_ARRAY */

        if ((i >= phys1GRxPortId[0]) &&
             (i <= phys1GRxPortId[FM_MAX_NUM_OF_1G_RX_PORTS-1]))
        {
            for (j=0; j<ARRAY_SIZE(phys1GRxPortId); j++)
                if (phys1GRxPortId[j] == i)
                    break;
            ASSERT_COND(j<ARRAY_SIZE(phys1GRxPortId));
            p_LnxWrpFmDev->rxPorts[j].settings.param.liodnBase = liodnBase;
        }
        else if (FM_MAX_NUM_OF_10G_RX_PORTS &&
                 (i >= phys10GRxPortId[0]) &&
                 (i <= phys10GRxPortId[FM_MAX_NUM_OF_10G_RX_PORTS-1]))
        {
            for (j=0; j<ARRAY_SIZE(phys10GRxPortId); j++)
                if (phys10GRxPortId[j] == i)
                    break;
            ASSERT_COND(j<ARRAY_SIZE(phys10GRxPortId));
            p_LnxWrpFmDev->rxPorts[FM_MAX_NUM_OF_1G_RX_PORTS+j].settings.param.liodnBase = liodnBase;
        }
        else if ((i >= physOhPortId[0]) &&
                 (i <= physOhPortId[FM_MAX_NUM_OF_OH_PORTS-1]))
        {
            for (j=0; j<ARRAY_SIZE(physOhPortId); j++)
                if (physOhPortId[j] == i)
                    break;
            ASSERT_COND(j<ARRAY_SIZE(physOhPortId));
            if (j == 0)
                p_LnxWrpFmDev->hcPort.settings.param.liodnBase = liodnBase;
            else
                p_LnxWrpFmDev->opPorts[j - 1].settings.param.liodnBase = liodnBase;
        }
        else if ((i >= phys1GTxPortId[0]) &&
                  (i <= phys1GTxPortId[FM_MAX_NUM_OF_1G_TX_PORTS-1]))
        {
            for (j=0; j<ARRAY_SIZE(phys1GTxPortId); j++)
                if (phys1GTxPortId[j] == i)
                    break;
            ASSERT_COND(j<ARRAY_SIZE(phys1GTxPortId));
            p_LnxWrpFmDev->txPorts[j].settings.param.liodnBase = liodnBase;
        }
        else if (FM_MAX_NUM_OF_10G_TX_PORTS &&
                 (i >= phys10GTxPortId[0]) &&
                 (i <= phys10GTxPortId[FM_MAX_NUM_OF_10G_TX_PORTS-1]))
        {
            for (j=0; j<ARRAY_SIZE(phys10GTxPortId); j++)
                if (phys10GTxPortId[j] == i)
                    break;
            ASSERT_COND(j<ARRAY_SIZE(phys10GTxPortId));
            p_LnxWrpFmDev->txPorts[FM_MAX_NUM_OF_1G_TX_PORTS+j].settings.param.liodnBase = liodnBase;
        }
    }

    p_Ppids = (t_Ppids *)UINT_TO_PTR(p_LnxWrpFmDev->fmBaseAddr+FM_BMI_PPIDS_OFFSET);

    for (i=0; i<FM_MAX_NUM_OF_1G_RX_PORTS; i++)
        /* TODO: this was .rxPartitionId = ; liodnOffset seems to have the same meaning */
        p_LnxWrpFmDev->rxPorts[i].settings.param.specificParams.rxParams.liodnOffset =
                p_Ppids->fmbm_ppid[phys1GRxPortId[i]-1];

    for (i=0; i<FM_MAX_NUM_OF_10G_RX_PORTS; i++)
            p_LnxWrpFmDev->rxPorts[FM_MAX_NUM_OF_1G_RX_PORTS+i].settings.param.specificParams.rxParams.liodnOffset =
                p_Ppids->fmbm_ppid[phys10GRxPortId[i]-1];

#ifdef FM_OP_PARTITION_ERRATA_FMANx8
    for (i=0; i<FM_MAX_NUM_OF_OH_PORTS; i++)
    {
        /* OH port #0 is host-command, don't need this workaround */
        if (i == 0)
            continue;
        if (fmRev == 0x0100)
            /* TODO: this was opPartitionId = ; opLiodnOffset seems to have the same meaning */
            p_LnxWrpFmDev->opPorts[i-1].settings.param.specificParams.nonRxParams.opLiodnOffset =
                p_Ppids->fmbm_ppid[physOhPortId[i]-1];
    }
#endif  /* FM_OP_PARTITION_ERRATA_FMANx8 */

    return E_OK;
}

#ifndef NO_OF_SUPPORT
/* The default address for the Fman microcode in flash. Having a default
 * allows older systems to continue functioning.  0xEF000000 is the address
 * where the firmware is normally on a P4080DS.
 */
#ifdef CONFIG_PHYS_64BIT
static phys_addr_t P4080_UCAddr = 0xfef000000ull;
#else
static phys_addr_t P4080_UCAddr = 0xef000000;
#endif


/**
 * FmanUcodeAddrParam - process the fman_ucode kernel command-line parameter
 *
 * This function is called when the kernel encounters a fman_ucode command-
 * line parameter.  This parameter contains the address of the Fman microcode
 * in flash.
 */
static int FmanUcodeAddrParam(char *str)
{
    unsigned long long l;
    int ret;

    ret = strict_strtoull(str, 0, &l);
    if (!ret)
        P4080_UCAddr = (phys_addr_t) l;

    return ret;
}
__setup("fman_ucode=", FmanUcodeAddrParam);

/**
 * FindFmanMicrocode - find the Fman micrcode in memory
 *
 * This function returns a pointer to the QE Firmware blob that holds
 * the Fman microcode.  We use the QE Firmware structure because Fman microcode
 * is similar to QE microcode, so there's no point in defining a new layout.
 *
 * Current versions of U-Boot embed the Fman firmware into the device tree,
 * so we check for that first.  Each Fman node in the device tree contains a
 * node or a pointer to node that holds the firmware.  Technically, we should
 * be fetching the firmware node for the current Fman, but we don't have that
 * information any more, so we assume that there is only one firmware node in
 * the device tree, and that all Fmen use the same firmware.
 *
 * If we have an older U-Boot, then we assume that the firmware is located in
 * flash at physical address 'P4080_UCAddr'
 */
static const struct qe_firmware *FindFmanMicrocode(void)
{
    static const struct qe_firmware *P4080_UCPatch;
    struct device_node *np;
#ifdef FMAN_READ_MICROCODE_FROM_NOR_FLASH
    unsigned long P4080_UCSize;
    const struct qe_header *hdr;
#endif

    if (P4080_UCPatch)
	    return P4080_UCPatch;

    /* The firmware should be inside the device tree. */
    np = of_find_compatible_node(NULL, NULL, "fsl,fman-firmware");
    if (np) {
	    P4080_UCPatch = of_get_property(np, "fsl,firmware", NULL);
	    if (P4080_UCPatch)
		    return P4080_UCPatch;
	    else
		    REPORT_ERROR(WARNING, E_NOT_FOUND, ("firmware node is incomplete"));
    }

#ifdef FMAN_READ_MICROCODE_FROM_NOR_FLASH
    /* If not, then we have a legacy U-Boot.  The firmware is in flash. */
    /* Only map enough to the get the core structure */
    P4080_UCPatch = ioremap(P4080_UCAddr, sizeof(struct qe_firmware));
    if (!P4080_UCPatch) {
        REPORT_ERROR(MAJOR, E_NULL_POINTER, ("ioremap(%llx) returned NULL", (u64) P4080_UCAddr));
        return NULL;
    }
    /* Make sure it really is a QE Firmware blob */
    hdr = &P4080_UCPatch->header;
    if (!hdr ||
        (hdr->magic[0] != 'Q') || (hdr->magic[1] != 'E') ||
        (hdr->magic[2] != 'F')) {
        REPORT_ERROR(MAJOR, E_NOT_FOUND, ("data at %llx is not a Fman microcode", (u64) P4080_UCAddr));
        return NULL;
    }

    /* Now we call ioremap again, this time to pick up the whole blob. We never
     * iounmap() the memory because we might reset the Fman at any time.
     */
    /* TODO: ionumap() should be performed when unloading the driver */
    P4080_UCSize = sizeof(u32) * P4080_UCPatch->microcode[0].count;
    iounmap((void *)P4080_UCPatch);
    P4080_UCPatch = ioremap(P4080_UCAddr, P4080_UCSize);
    if (!P4080_UCPatch) {
        REPORT_ERROR(MAJOR, E_NULL_POINTER, ("ioremap(%llx) returned NULL", (u64) P4080_UCAddr));
        return NULL;
    }
#else
    /* Returning NULL here forces the reuse of the IRAM content */
    P4080_UCPatch = NULL;
#endif /* FMAN_READ_MICROCODE_FROM_NOR_FLASH */
    return P4080_UCPatch;
}

static t_LnxWrpFmDev * ReadFmDevTreeNode (struct of_device *of_dev)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    struct device_node  *fm_node, *dev_node, *dpa_node;
    struct of_device_id name;
    struct resource     res;
    const uint32_t      *uint32_prop;
    int                 _errno=0, lenp;

    fm_node = of_dev->node;

    uint32_prop = (uint32_t *)of_get_property(fm_node, "cell-index", &lenp);
    if (unlikely(uint32_prop == NULL)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, cell-index) failed", fm_node->full_name));
        return NULL;
    }
    BUG_ON(lenp != sizeof(uint32_t));
    if (*uint32_prop > INTG_MAX_NUM_OF_FM) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("fm id!"));
        return NULL;
    }
    p_LnxWrpFmDev = CreateFmDev(*uint32_prop);
    if (!p_LnxWrpFmDev) {
        REPORT_ERROR(MAJOR, E_NULL_POINTER, NO_MSG);
        return NULL;
    }
    p_LnxWrpFmDev->dev = &of_dev->dev;
    p_LnxWrpFmDev->id = *uint32_prop;

    /* Get the FM interrupt */
    p_LnxWrpFmDev->irq = of_irq_to_resource(fm_node, 0, NULL);
    if (unlikely(p_LnxWrpFmDev->irq == /*NO_IRQ*/0)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_irq_to_resource() = %d", NO_IRQ));
        return NULL;
    }

    /* Get the FM error interrupt */
    p_LnxWrpFmDev->err_irq = of_irq_to_resource(fm_node, 1, NULL);
    /* TODO - un-comment it once there will be err_irq in the DTS */
#if 0
    if (unlikely(p_LnxWrpFmDev->err_irq == /*NO_IRQ*/0)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_irq_to_resource() = %d", NO_IRQ));
        return NULL;
    }
#endif /* 0 */

    /* Get the FM address */
    _errno = of_address_to_resource(fm_node, 0, &res);
    if (unlikely(_errno < 0)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_address_to_resource() = %d", _errno));
        return NULL;
    }

    p_LnxWrpFmDev->fmBaseAddr = res.start;
    p_LnxWrpFmDev->fmMemSize = res.end + 1 - res.start;

    uint32_prop = (uint32_t *)of_get_property(fm_node, "clock-frequency", &lenp);
    if (unlikely(uint32_prop == NULL)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, clock-frequency) failed", fm_node->full_name));
        return NULL;
    }
    BUG_ON(lenp != sizeof(uint32_t));
    p_LnxWrpFmDev->fmDevSettings.param.fmClkFreq = (*uint32_prop + 500000)/1000000; /* In MHz, rounded */

    /* Get the MURAM base address and size */
    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("muram") >= sizeof(name.name));
    strcpy(name.name, "muram");
    BUG_ON(strlen("fsl,fman-muram") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman-muram");
    for_each_child_of_node(fm_node, dev_node) {
        if (likely(of_match_node(&name, dev_node) != NULL)) {
            _errno = of_address_to_resource(dev_node, 0, &res);
            if (unlikely(_errno < 0)) {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_address_to_resource() = %d", _errno));
                return NULL;
            }

            p_LnxWrpFmDev->fmMuramBaseAddr = res.start;
            p_LnxWrpFmDev->fmMuramMemSize = res.end + 1 - res.start;
        }
    }

    /* Get the RTC base address and size */
    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("rtc") >= sizeof(name.name));
    strcpy(name.name, "rtc");
    BUG_ON(strlen("fsl,fman-rtc") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman-rtc");
    for_each_child_of_node(fm_node, dev_node) {
        if (likely(of_match_node(&name, dev_node) != NULL)) {
            _errno = of_address_to_resource(dev_node, 0, &res);
            if (unlikely(_errno < 0)) {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_address_to_resource() = %d", _errno));
                return NULL;
            }

            p_LnxWrpFmDev->fmRtcBaseAddr = res.start;
            p_LnxWrpFmDev->fmRtcMemSize = res.end + 1 - res.start;
        }
    }

    /* Get all PCD nodes */
    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("parser") >= sizeof(name.name));
    strcpy(name.name, "parser");
    BUG_ON(strlen("fsl,fman-parser") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman-parser");
    for_each_child_of_node(fm_node, dev_node)
        if (likely(of_match_node(&name, dev_node) != NULL))
            p_LnxWrpFmDev->prsActive = TRUE;

    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("keygen") >= sizeof(name.name));
    strcpy(name.name, "keygen");
    BUG_ON(strlen("fsl,fman-keygen") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman-keygen");
    for_each_child_of_node(fm_node, dev_node)
        if (likely(of_match_node(&name, dev_node) != NULL))
            p_LnxWrpFmDev->kgActive = TRUE;

    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("cc") >= sizeof(name.name));
    strcpy(name.name, "cc");
    BUG_ON(strlen("fsl,fman-cc") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman-cc");
    for_each_child_of_node(fm_node, dev_node)
        if (likely(of_match_node(&name, dev_node) != NULL))
            p_LnxWrpFmDev->ccActive = TRUE;

    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("policer") >= sizeof(name.name));
    strcpy(name.name, "policer");
    BUG_ON(strlen("fsl,fman-policer") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman-policer");
    for_each_child_of_node(fm_node, dev_node)
        if (likely(of_match_node(&name, dev_node) != NULL))
            p_LnxWrpFmDev->plcrActive = TRUE;

    if (p_LnxWrpFmDev->prsActive || p_LnxWrpFmDev->kgActive ||
        p_LnxWrpFmDev->ccActive || p_LnxWrpFmDev->plcrActive)
        p_LnxWrpFmDev->pcdActive = TRUE;

    if (p_LnxWrpFmDev->pcdActive)
    {
        const char *str_prop = (char *)of_get_property(fm_node, "fsl,default-pcd", &lenp);
        if (str_prop) {
            if (strncmp(str_prop, "3-tuple", strlen("3-tuple")) == 0)
                p_LnxWrpFmDev->defPcd = e_FM_PCD_3_TUPLE;
        }
        else
            p_LnxWrpFmDev->defPcd = e_NO_PCD;
    }

    of_node_put(fm_node);

    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("fsl,dpa-ethernet") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,dpa-ethernet");
    for_each_matching_node(dpa_node, &name) {
        struct device_node  *mac_node;
        const phandle       *phandle_prop;

        phandle_prop = (typeof(phandle_prop))of_get_property(dpa_node, "fsl,fman-mac", &lenp);
        if (phandle_prop == NULL)
            continue;

        BUG_ON(lenp != sizeof(phandle));

        mac_node = of_find_node_by_phandle(*phandle_prop);
        if (unlikely(mac_node == NULL)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_find_node_by_phandle() failed"));
            return NULL;
        }

        fm_node = of_get_parent(mac_node);
        if (unlikely(fm_node == NULL)) {
            REPORT_ERROR(MAJOR, E_NO_DEVICE, ("of_get_parent() = %d", _errno));
            return NULL;
        }
        of_node_put(mac_node);

        uint32_prop = (uint32_t *)of_get_property(fm_node, "cell-index", &lenp);
        if (unlikely(uint32_prop == NULL)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, cell-index) failed", fm_node->full_name));
            return NULL;
        }
        BUG_ON(lenp != sizeof(uint32_t));
        of_node_put(fm_node);

        if (*uint32_prop == p_LnxWrpFmDev->id) {
            phandle_prop = (typeof(phandle_prop))of_get_property(dpa_node, "fsl,qman-channel", &lenp);
            if (unlikely(phandle_prop == NULL)) {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, fsl,qman-channel) failed", dpa_node->full_name));
                return NULL;
            }
            BUG_ON(lenp != sizeof(phandle));

            dev_node = of_find_node_by_phandle(*phandle_prop);
            if (unlikely(dev_node == NULL)) {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_find_node_by_phandle() failed"));
                return NULL;
            }

            uint32_prop = (typeof(uint32_prop))of_get_property(dev_node, "fsl,qman-channel-id", &lenp);
            if (unlikely(uint32_prop == NULL)) {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, fsl,qman-channel-id) failed", dev_node->full_name));
                of_node_put(dev_node);
                return NULL;
            }
            of_node_put(dev_node);
            BUG_ON(lenp != sizeof(uint32_t));
            p_LnxWrpFmDev->hcCh = *uint32_prop;
            break;
        }
    }

    p_LnxWrpFmDev->active = TRUE;

    return p_LnxWrpFmDev;
}

static t_LnxWrpFmPortDev * ReadFmPortDevTreeNode (struct of_device *of_dev)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev;
    struct device_node  *fm_node, *port_node;
    struct resource     res;
    const uint32_t      *uint32_prop;
    int                 _errno=0, lenp;
#ifdef CONFIG_FMAN_P1023
    static unsigned char have_oh_port = 0;
#endif

    port_node = of_dev->node;

    /* Get the FM node */
    fm_node = of_get_parent(port_node);
    if (unlikely(fm_node == NULL)) {
        REPORT_ERROR(MAJOR, E_NO_DEVICE, ("of_get_parent() = %d", _errno));
        return NULL;
    }

    p_LnxWrpFmDev = dev_get_drvdata(&of_find_device_by_node(fm_node)->dev);
    of_node_put(fm_node);

    /* if fm_probe() failed, no point in going further with port probing */
    if (p_LnxWrpFmDev == NULL)
        return NULL;

    uint32_prop = (uint32_t *)of_get_property(port_node, "cell-index", &lenp);
    if (unlikely(uint32_prop == NULL)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, cell-index) failed", port_node->full_name));
        return NULL;
    }
    BUG_ON(lenp != sizeof(uint32_t));
    if (of_device_is_compatible(port_node, "fsl,fman-port-oh")) {
        if (unlikely(*uint32_prop >= FM_MAX_NUM_OF_OH_PORTS)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, cell-index) failed", port_node->full_name));
            return NULL;
        }

#ifdef CONFIG_FMAN_P1023
        /* Beware, this can be done when there is only one FMan to be initialized */
        if (!have_oh_port) {
            have_oh_port = 1; /* first OP/HC port is used for host command */
#else
        /* Here it is hardcoded the use of the OH port 1 (with cell-index 0) */
        if (*uint32_prop == 0) {
#endif
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->hcPort;
            p_LnxWrpFmPortDev->id = 0;
            //p_LnxWrpFmPortDev->id = *uint32_prop-1;
            //p_LnxWrpFmPortDev->id = *uint32_prop;
            p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_OH_HOST_COMMAND;
        }
        else {
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->opPorts[*uint32_prop-1];
            p_LnxWrpFmPortDev->id = *uint32_prop-1;
            p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_OH_OFFLINE_PARSING;
        }
        p_LnxWrpFmPortDev->settings.param.portId = *uint32_prop;

        uint32_prop = (uint32_t *)of_get_property(port_node, "fsl,qman-channel-id", &lenp);
        if (uint32_prop == NULL) {
//            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("missing fsl,qman-channel-id"));
            XX_Print("FM warning: missing fsl,qman-channel-id for OH port.\n");
            return NULL;
        }
        BUG_ON(lenp != sizeof(uint32_t));
        p_LnxWrpFmPortDev->txCh = *uint32_prop;

        p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.qmChannel = p_LnxWrpFmPortDev->txCh;
    }
    else if (of_device_is_compatible(port_node, "fsl,fman-port-1g-tx") ||
             of_device_is_compatible(port_node, "fsl,fman-port-10g-tx")) {
        if (unlikely(*uint32_prop >= FM_MAX_NUM_OF_TX_PORTS)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, cell-index) failed", port_node->full_name));
            return NULL;
        }
        if (of_device_is_compatible(port_node, "fsl,fman-port-10g-tx"))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[*uint32_prop+FM_MAX_NUM_OF_1G_TX_PORTS];
        else
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[*uint32_prop];

        p_LnxWrpFmPortDev->id = *uint32_prop;
        p_LnxWrpFmPortDev->settings.param.portId = p_LnxWrpFmPortDev->id;
        if (of_device_is_compatible(port_node, "fsl,fman-port-10g-tx"))
            p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_TX_10G;
        else
            p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_TX;

        uint32_prop = (uint32_t *)of_get_property(port_node, "fsl,qman-channel-id", &lenp);
        if (uint32_prop == NULL) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("missing fsl,qman-channel-id"));
            return NULL;
        }
        BUG_ON(lenp != sizeof(uint32_t));
        p_LnxWrpFmPortDev->txCh = *uint32_prop;
        p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.qmChannel = p_LnxWrpFmPortDev->txCh;
    }
    else if (of_device_is_compatible(port_node, "fsl,fman-port-1g-rx") ||
             of_device_is_compatible(port_node, "fsl,fman-port-10g-rx")) {
        if (unlikely(*uint32_prop >= FM_MAX_NUM_OF_RX_PORTS)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_get_property(%s, cell-index) failed", port_node->full_name));
            return NULL;
        }
        if (of_device_is_compatible(port_node, "fsl,fman-port-10g-rx"))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[*uint32_prop+FM_MAX_NUM_OF_1G_RX_PORTS];
        else
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[*uint32_prop];

        p_LnxWrpFmPortDev->id = *uint32_prop;
        p_LnxWrpFmPortDev->settings.param.portId = p_LnxWrpFmPortDev->id;
        if (of_device_is_compatible(port_node, "fsl,fman-port-10g-rx"))
            p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_RX_10G;
        else
            p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_RX;

        if (p_LnxWrpFmDev->pcdActive)
            p_LnxWrpFmPortDev->defPcd = p_LnxWrpFmDev->defPcd;
    }
    else {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("Illegal port type"));
        return NULL;
    }

    _errno = of_address_to_resource(port_node, 0, &res);
    if (unlikely(_errno < 0)) {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("of_address_to_resource() = %d", _errno));
        return NULL;
    }

    p_LnxWrpFmPortDev->dev = &of_dev->dev;
    p_LnxWrpFmPortDev->baseAddr = res.start;
    p_LnxWrpFmPortDev->memSize = res.end + 1 - res.start;
    p_LnxWrpFmPortDev->settings.param.h_Fm = p_LnxWrpFmDev->h_Dev;
    p_LnxWrpFmPortDev->h_LnxWrpFmDev = (t_Handle)p_LnxWrpFmDev;

    of_node_put(port_node);

    p_LnxWrpFmPortDev->active = TRUE;

    return p_LnxWrpFmPortDev;
}
#endif /* !NO_OF_SUPPORT */

static void LnxwrpFmDevExceptionsCb(t_Handle h_App, e_FmExceptions exception)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)h_App;

    ASSERT_COND(p_LnxWrpFmDev);

    DBG(INFO, ("got fm exception %d", exception));

    /* do nothing */
    UNUSED(exception);
}

static void LnxwrpFmDevBusErrorCb(t_Handle        h_App,
                                  e_FmPortType    portType,
                                  uint8_t         portId,
                                  uint64_t        addr,
                                  uint8_t         tnum,
                                  uint16_t        liodn)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)h_App;

    ASSERT_COND(p_LnxWrpFmDev);

    /* do nothing */
    UNUSED(portType);UNUSED(portId);UNUSED(addr);UNUSED(tnum);UNUSED(liodn);
}

static void LnxwrpFmPcdDevExceptionsCb( t_Handle h_App, e_FmPcdExceptions exception)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)h_App;

    ASSERT_COND(p_LnxWrpFmDev);

    DBG(INFO, ("got fm-pcd exception %d", exception));

    /* do nothing */
    UNUSED(exception);
}

static void LnxwrpFmPcdDevIndexedExceptionsCb(t_Handle          h_App,
                                              e_FmPcdExceptions exception,
                                              uint16_t          index)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)h_App;

    ASSERT_COND(p_LnxWrpFmDev);

    DBG(INFO, ("got fm-pcd-indexed exception %d, indx %d", exception, index));

    /* do nothing */
    UNUSED(exception);UNUSED(index);
}

static t_Error ConfigureFmDev(t_LnxWrpFmDev  *p_LnxWrpFmDev)
{
    struct resource     *dev_res;
    int                 _errno;
    uint64_t            fmPhysAddr, muramPhysAddr;

    if (!p_LnxWrpFmDev->active)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM not configured!!!"));

    _errno = can_request_irq(p_LnxWrpFmDev->irq, 0);
    if (unlikely(_errno < 0))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("can_request_irq() = %d", _errno));
    _errno = devm_request_irq(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->irq, fm_irq, 0, "fman", p_LnxWrpFmDev);
    if (unlikely(_errno < 0))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("request_irq(%d) = %d", p_LnxWrpFmDev->irq, _errno));

    if (p_LnxWrpFmDev->err_irq != 0) {
        _errno = can_request_irq(p_LnxWrpFmDev->err_irq, 0);
        if (unlikely(_errno < 0))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("can_request_irq() = %d", _errno));
        _errno = devm_request_irq(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->err_irq, fm_err_irq, IRQF_SHARED, "fman-err", p_LnxWrpFmDev);
        if (unlikely(_errno < 0))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("request_irq(%d) = %d", p_LnxWrpFmDev->err_irq, _errno));
    }

    fmPhysAddr = p_LnxWrpFmDev->fmBaseAddr;
    p_LnxWrpFmDev->res = devm_request_mem_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->fmBaseAddr, p_LnxWrpFmDev->fmMemSize, "fman");
    if (unlikely(p_LnxWrpFmDev->res == NULL))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("request_mem_region() failed"));

    p_LnxWrpFmDev->fmBaseAddr = PTR_TO_UINT(devm_ioremap(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->fmBaseAddr, p_LnxWrpFmDev->fmMemSize));
    if (unlikely(p_LnxWrpFmDev->fmBaseAddr == 0))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("devm_ioremap() failed"));

    muramPhysAddr = p_LnxWrpFmDev->fmMuramBaseAddr;
    dev_res = __devm_request_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res, p_LnxWrpFmDev->fmMuramBaseAddr, p_LnxWrpFmDev->fmMuramMemSize, "fman-muram");
    if (unlikely(dev_res == NULL))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("__devm_request_region() failed"));

    p_LnxWrpFmDev->fmMuramBaseAddr = PTR_TO_UINT(devm_ioremap(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->fmMuramBaseAddr, p_LnxWrpFmDev->fmMuramMemSize));
    if (unlikely(p_LnxWrpFmDev->fmMuramBaseAddr == 0))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("devm_ioremap() failed"));

    if (SYS_RegisterIoMap((uint64_t)p_LnxWrpFmDev->fmMuramBaseAddr, (uint64_t)muramPhysAddr, p_LnxWrpFmDev->fmMuramMemSize) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM MURAM memory map"));

    if (p_LnxWrpFmDev->fmRtcBaseAddr)
    {
        uint64_t rtcPhysAddr = p_LnxWrpFmDev->fmRtcBaseAddr;
        dev_res = __devm_request_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res, p_LnxWrpFmDev->fmRtcBaseAddr, p_LnxWrpFmDev->fmRtcMemSize, "fman-rtc");
        if (unlikely(dev_res == NULL))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("__devm_request_region() failed"));

        p_LnxWrpFmDev->fmRtcBaseAddr = PTR_TO_UINT(devm_ioremap(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->fmRtcBaseAddr, p_LnxWrpFmDev->fmRtcMemSize));
        if (unlikely(p_LnxWrpFmDev->fmRtcBaseAddr == 0))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("devm_ioremap() failed"));

        if (SYS_RegisterIoMap((uint64_t)p_LnxWrpFmDev->fmRtcBaseAddr, (uint64_t)rtcPhysAddr, p_LnxWrpFmDev->fmRtcMemSize) != E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM-RTC memory map"));
    }

    if (SYS_RegisterIoMap((uint64_t)p_LnxWrpFmDev->fmBaseAddr, (uint64_t)fmPhysAddr, p_LnxWrpFmDev->fmMemSize) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM memory map"));

    p_LnxWrpFmDev->fmDevSettings.param.baseAddr     = p_LnxWrpFmDev->fmBaseAddr;
    p_LnxWrpFmDev->fmDevSettings.param.fmId         = p_LnxWrpFmDev->id;
    p_LnxWrpFmDev->fmDevSettings.param.irq          = NO_IRQ;
    p_LnxWrpFmDev->fmDevSettings.param.errIrq       = NO_IRQ;
    p_LnxWrpFmDev->fmDevSettings.param.f_Exception  = LnxwrpFmDevExceptionsCb;
    p_LnxWrpFmDev->fmDevSettings.param.f_BusError   = LnxwrpFmDevBusErrorCb;
    p_LnxWrpFmDev->fmDevSettings.param.h_App        = p_LnxWrpFmDev;

    return FillRestFmInfo(p_LnxWrpFmDev);
}

static t_Error ConfigureFmPortDev(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
    struct resource     *dev_res;

    if (!p_LnxWrpFmPortDev->active)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM port not configured!!!"));

    dev_res = __devm_request_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res, p_LnxWrpFmPortDev->baseAddr, p_LnxWrpFmPortDev->memSize, "fman-port-hc");
    if (unlikely(dev_res == NULL))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("__devm_request_region() failed"));
    p_LnxWrpFmPortDev->baseAddr = PTR_TO_UINT(devm_ioremap(p_LnxWrpFmDev->dev, p_LnxWrpFmPortDev->baseAddr, p_LnxWrpFmPortDev->memSize));
    if (unlikely(p_LnxWrpFmPortDev->baseAddr == 0))
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("devm_ioremap() failed"));

    p_LnxWrpFmPortDev->settings.param.baseAddr = p_LnxWrpFmPortDev->baseAddr;

    return E_OK;
}

static t_Error InitFmPcdDev(t_LnxWrpFmDev  *p_LnxWrpFmDev)
{
    if (p_LnxWrpFmDev->pcdActive)
    {
        t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = &p_LnxWrpFmDev->hcPort;
        t_FmPcdParams       fmPcdParams;
        t_Error             err;

        memset(&fmPcdParams, 0, sizeof(fmPcdParams));
        fmPcdParams.h_Fm        = p_LnxWrpFmDev->h_Dev;
        fmPcdParams.prsSupport  = p_LnxWrpFmDev->prsActive;
        fmPcdParams.kgSupport   = p_LnxWrpFmDev->kgActive;
        fmPcdParams.plcrSupport = p_LnxWrpFmDev->plcrActive;
        fmPcdParams.ccSupport   = p_LnxWrpFmDev->ccActive;
        fmPcdParams.numOfSchemes = FM_PCD_KG_NUM_OF_SCHEMES;

#ifndef CONFIG_GUEST_PARTITION
        fmPcdParams.f_Exception   = LnxwrpFmPcdDevExceptionsCb;
        if (fmPcdParams.kgSupport)
            fmPcdParams.f_ExceptionId  = LnxwrpFmPcdDevIndexedExceptionsCb;
        fmPcdParams.h_App              = p_LnxWrpFmDev;
#endif /* !CONFIG_GUEST_PARTITION */

#ifdef CONFIG_MULTI_PARTITION_SUPPORT
        fmPcdParams.numOfSchemes = 0;
        fmPcdParams.numOfClsPlanEntries = 0;
        fmPcdParams.partitionId = 0;
#endif  /* CONFIG_MULTI_PARTITION_SUPPORT */
        fmPcdParams.useHostCommand = TRUE;

        p_LnxWrpFmDev->hc_tx_fq  =
            FqAlloc(p_LnxWrpFmDev,
                    0,
                    QMAN_FQ_FLAG_TO_DCPORTAL,
                    p_LnxWrpFmPortDev->txCh,
                    0);
        if(!p_LnxWrpFmDev->hc_tx_fq)
            RETURN_ERROR(MAJOR, E_NULL_POINTER, ("Frame queue allocation failed..."));

        p_LnxWrpFmDev->hc_tx_conf_fq  =
            FqAlloc(p_LnxWrpFmDev,
                    0,
                    QMAN_FQ_FLAG_NO_ENQUEUE,
                    p_LnxWrpFmDev->hcCh,
                    7);
        if(!p_LnxWrpFmDev->hc_tx_conf_fq)
            RETURN_ERROR(MAJOR, E_NULL_POINTER, ("Frame queue allocation failed..."));

        p_LnxWrpFmDev->hc_tx_err_fq  =
            FqAlloc(p_LnxWrpFmDev,
                    0,
                    QMAN_FQ_FLAG_NO_ENQUEUE,
                    p_LnxWrpFmDev->hcCh,
                    7);
        if(!p_LnxWrpFmDev->hc_tx_err_fq)
            RETURN_ERROR(MAJOR, E_NULL_POINTER, ("Frame queue allocation failed..."));

        fmPcdParams.hc.portBaseAddr = p_LnxWrpFmPortDev->baseAddr;
        fmPcdParams.hc.portId = p_LnxWrpFmPortDev->settings.param.portId;
        fmPcdParams.hc.liodnBase = p_LnxWrpFmPortDev->settings.param.liodnBase;
        fmPcdParams.hc.errFqid = qman_fq_fqid(p_LnxWrpFmDev->hc_tx_err_fq);
        fmPcdParams.hc.confFqid = qman_fq_fqid(p_LnxWrpFmDev->hc_tx_conf_fq);
        fmPcdParams.hc.qmChannel = p_LnxWrpFmPortDev->txCh;
        fmPcdParams.hc.f_QmEnqueue = QmEnqueueCB;
        fmPcdParams.hc.h_QmArg = (t_Handle)p_LnxWrpFmDev;

        p_LnxWrpFmDev->h_PcdDev = FM_PCD_Config(&fmPcdParams);
        if(!p_LnxWrpFmDev->h_PcdDev)
            RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM PCD!"));

        if((err = FM_PCD_ConfigPlcrNumOfSharedProfiles(p_LnxWrpFmDev->h_PcdDev,
                                                       LNXWRP_FM_NUM_OF_SHARED_PROFILES))!= E_OK)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        if((err = FM_PCD_Init(p_LnxWrpFmDev->h_PcdDev))!= E_OK)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        if (p_LnxWrpFmDev->err_irq == 0) {
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_KG_EXCEPTION_DOUBLE_ECC,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_KG_EXCEPTION_KEYSIZE_OVERFLOW,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_PLCR_EXCEPTION_INIT_ENTRY_ERROR,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_PLCR_EXCEPTION_DOUBLE_ECC,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_PRS_EXCEPTION_DOUBLE_ECC,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_PLCR_EXCEPTION_PRAM_SELF_INIT_COMPLETE,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_PLCR_EXCEPTION_ATOMIC_ACTION_COMPLETE,FALSE);
            FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,e_FM_PCD_PRS_EXCEPTION_SINGLE_ECC,FALSE);
        }
    }

    return E_OK;
}

static t_Error InitFmDev(t_LnxWrpFmDev  *p_LnxWrpFmDev)
{
    const struct qe_firmware *fw;

    if (!p_LnxWrpFmDev->active)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM not configured!!!"));

    if ((p_LnxWrpFmDev->h_MuramDev = FM_MURAM_ConfigAndInit(p_LnxWrpFmDev->fmMuramBaseAddr, p_LnxWrpFmDev->fmMuramMemSize)) == NULL)
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM-MURAM!"));

    /* Loading the fman-controller code */
    fw = FindFmanMicrocode();

    if (!fw) {
#ifdef FMAN_READ_MICROCODE_FROM_NOR_FLASH
        /* We already reported an error, so just return NULL*/
        return ERROR_CODE(E_NULL_POINTER);
#else
        /* this forces the reuse of the current IRAM content */
        p_LnxWrpFmDev->fmDevSettings.param.firmware.size = 0;
        p_LnxWrpFmDev->fmDevSettings.param.firmware.p_Code = NULL;
#endif
    } else {
        p_LnxWrpFmDev->fmDevSettings.param.firmware.p_Code =
            (void *) fw + fw->microcode[0].code_offset;
        p_LnxWrpFmDev->fmDevSettings.param.firmware.size =
            sizeof(u32) * fw->microcode[0].count;
        DBG(INFO, ("Loading fman-controller code version %d.%d.%d",
                   fw->microcode[0].major,
                   fw->microcode[0].minor,
                   fw->microcode[0].revision));
    }

    p_LnxWrpFmDev->fmDevSettings.param.h_FmMuram = p_LnxWrpFmDev->h_MuramDev;

    if ((p_LnxWrpFmDev->h_Dev = FM_Config(&p_LnxWrpFmDev->fmDevSettings.param)) == NULL)
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM"));

    if (FM_ConfigMaxNumOfOpenDmas(p_LnxWrpFmDev->h_Dev,BMI_MAX_NUM_OF_DMAS) != E_OK)
         RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM"));

    if (FM_ConfigResetOnInit(p_LnxWrpFmDev->h_Dev, TRUE) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM"));

#ifdef CONFIG_FMAN_P1023
    if (FM_ConfigDmaAidOverride(p_LnxWrpFmDev->h_Dev, TRUE) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM"));
#endif

    if (FM_Init(p_LnxWrpFmDev->h_Dev) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM"));

    if (p_LnxWrpFmDev->err_irq == 0) {
        FM_SetException(p_LnxWrpFmDev->h_Dev, e_FM_EX_DMA_BUS_ERROR,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_DMA_READ_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_DMA_SYSTEM_WRITE_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_DMA_FM_WRITE_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_FPM_STALL_ON_TASKS , FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_FPM_DOUBLE_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_IRAM_ECC,FALSE);
        /* TODO: FmDisableRamsEcc assert for ramsEccOwners.
         * FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_MURAM_ECC,FALSE);*/
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_QMI_DOUBLE_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_BMI_LIST_RAM_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_BMI_PIPELINE_ECC,FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_BMI_STATISTICS_RAM_ECC, FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_FPM_SINGLE_ECC, FALSE);
        FM_SetException(p_LnxWrpFmDev->h_Dev,e_FM_EX_QMI_SINGLE_ECC, FALSE);
    }

    if (p_LnxWrpFmDev->fmRtcBaseAddr)
    {
        t_FmRtcParams   fmRtcParam;

        memset(&fmRtcParam, 0, sizeof(fmRtcParam));
        fmRtcParam.h_App = p_LnxWrpFmDev;
        fmRtcParam.h_Fm = p_LnxWrpFmDev->h_Dev;
        fmRtcParam.baseAddress = p_LnxWrpFmDev->fmRtcBaseAddr;

        if(!(p_LnxWrpFmDev->h_RtcDev = FM_RTC_Config(&fmRtcParam)))
            RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM-RTC"));

	if (FM_RTC_ConfigPeriod(p_LnxWrpFmDev->h_RtcDev, 10) != E_OK)
	    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM-RTC"));

        if (FM_RTC_Init(p_LnxWrpFmDev->h_RtcDev) != E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("FM-RTC"));
    }

//    return InitFmPcdDev(p_LnxWrpFmDev);
    return E_OK;
}

static t_Error InitFmPort3TupleDefPcd(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
    t_LnxWrpFmDev                       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
    t_FmPcdNetEnvParams                 *p_netEnvParam = NULL;
    t_FmPcdKgSchemeParams               *p_schemeParam = NULL;
    t_FmPortPcdParams                   pcdParam;
    t_FmPortPcdPrsParams                prsParam;
    t_FmPortPcdKgParams                 kgParam;
    uint8_t                             i, j;

    if (!p_LnxWrpFmDev->kgActive)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("keygen must be enabled for 3-tuple PCD!"));

    if (!p_LnxWrpFmDev->prsActive)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("parser must be enabled for 3-tuple PCD!"));

    if (p_LnxWrpFmPortDev->pcdNumOfQs < 9)
        RETURN_ERROR(MINOR, E_INVALID_VALUE, ("Need to save at least 18 queues for 3-tuple PCD!!!"));

    p_LnxWrpFmPortDev->totalNumOfSchemes = p_LnxWrpFmPortDev->numOfSchemesUsed = 2;

    if (AllocSchemesForPort(p_LnxWrpFmDev, p_LnxWrpFmPortDev->totalNumOfSchemes, &p_LnxWrpFmPortDev->schemesBase) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("No schemes for Rx or OP port for 3-tuple PCD!!!"));

    p_netEnvParam = kzalloc(sizeof(*p_netEnvParam), GFP_KERNEL);
    if (!p_netEnvParam) {
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Failed to allocate p_netEnvParam"));
    }
    /* set netEnv */
    p_netEnvParam->numOfDistinctionUnits = 2;
    p_netEnvParam->units[0].hdrs[0].hdr = HEADER_TYPE_IPv4; /* no special options */
    p_netEnvParam->units[1].hdrs[0].hdr = HEADER_TYPE_ETH;
    p_LnxWrpFmPortDev->h_DefNetEnv = FM_PCD_SetNetEnvCharacteristics(p_LnxWrpFmDev->h_PcdDev, p_netEnvParam);
    kfree(p_netEnvParam);
    if(!p_LnxWrpFmPortDev->h_DefNetEnv)
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM PCD!"));

    p_schemeParam = kmalloc(sizeof(*p_schemeParam), GFP_KERNEL);
    if (!p_schemeParam) {
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Failed to allocate p_schemeParam"));
    }
    for(i=0; i<p_LnxWrpFmPortDev->numOfSchemesUsed; i++)
    {
        memset(p_schemeParam, 0, sizeof(*p_schemeParam));
        p_schemeParam->modify = FALSE;
        p_schemeParam->id.relativeSchemeId = i+p_LnxWrpFmPortDev->schemesBase;
        p_schemeParam->alwaysDirect = FALSE;
        p_schemeParam->netEnvParams.h_NetEnv = p_LnxWrpFmPortDev->h_DefNetEnv;
        p_schemeParam->schemeCounter.update = TRUE;
        p_schemeParam->schemeCounter.value = 0;

        switch (i)
        {
            case (0): /* catch IPv4 */
                p_schemeParam->netEnvParams.numOfDistinctionUnits = 1;
                p_schemeParam->netEnvParams.unitIds[0] = 0;
                p_schemeParam->baseFqid = p_LnxWrpFmPortDev->pcdBaseQ;
                p_schemeParam->nextEngine = e_FM_PCD_DONE;
                p_schemeParam->numOfUsedExtractedOrs = 0;
                p_schemeParam->useHash = TRUE;
                p_schemeParam->keyExtractAndHashParams.numOfUsedExtracts = 3;
                for(j=0; j<p_schemeParam->keyExtractAndHashParams.numOfUsedExtracts; j++)
                {
                    p_schemeParam->keyExtractAndHashParams.extractArray[j].type = e_FM_PCD_EXTRACT_BY_HDR;
                    p_schemeParam->keyExtractAndHashParams.extractArray[j].extractByHdr.hdr = HEADER_TYPE_IPv4;
                    p_schemeParam->keyExtractAndHashParams.extractArray[j].extractByHdr.ignoreProtocolValidation = FALSE;
                    p_schemeParam->keyExtractAndHashParams.extractArray[j].extractByHdr.type = e_FM_PCD_EXTRACT_FULL_FIELD;
                }
                p_schemeParam->keyExtractAndHashParams.extractArray[0].extractByHdr.extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_PROTO;
                p_schemeParam->keyExtractAndHashParams.extractArray[1].extractByHdr.extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_SRC_IP;
                p_schemeParam->keyExtractAndHashParams.extractArray[2].extractByHdr.extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_DST_IP;

                if(p_schemeParam->useHash)
                {
                    p_schemeParam->keyExtractAndHashParams.privateDflt0 = 0x01020304;
                    p_schemeParam->keyExtractAndHashParams.privateDflt1 = 0x11121314;
                    p_schemeParam->keyExtractAndHashParams.numOfUsedDflts = FM_PCD_KG_NUM_OF_DEFAULT_GROUPS;
                    for(j=0; j<FM_PCD_KG_NUM_OF_DEFAULT_GROUPS; j++)
                    {
                        p_schemeParam->keyExtractAndHashParams.dflts[j].type = (e_FmPcdKgKnownFieldsDfltTypes)j; /* all types */
                        p_schemeParam->keyExtractAndHashParams.dflts[j].dfltSelect = e_FM_PCD_KG_DFLT_GBL_0;
                    }
                    p_schemeParam->keyExtractAndHashParams.numOfUsedMasks = 0;
                    p_schemeParam->keyExtractAndHashParams.hashShift = 0;
                    p_schemeParam->keyExtractAndHashParams.hashDistributionNumOfFqids = 8;
                }
                break;

            case (1): /* Garbage collector */
                p_schemeParam->netEnvParams.numOfDistinctionUnits = 0;
                p_schemeParam->baseFqid = p_LnxWrpFmPortDev->pcdBaseQ+8;
                break;

            default:
                break;
        }

        p_LnxWrpFmPortDev->h_Schemes[i] = FM_PCD_KgSetScheme(p_LnxWrpFmDev->h_PcdDev, p_schemeParam);
        if(!p_LnxWrpFmPortDev->h_Schemes[i]) {
            kfree(p_schemeParam);
            RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM_PCD_KgSetScheme failed"));
        }
    }
    kfree(p_schemeParam);

    /* initialize PCD parameters */
    memset(&pcdParam, 0, sizeof( t_FmPortPcdParams));
    pcdParam.h_NetEnv   = p_LnxWrpFmPortDev->h_DefNetEnv;
    pcdParam.pcdSupport = e_FM_PORT_PCD_SUPPORT_PRS_AND_KG;

    /* initialize Keygen parameters */
    memset(&prsParam, 0, sizeof( t_FmPortPcdPrsParams));

    prsParam.parsingOffset = 0;
    prsParam.firstPrsHdr = HEADER_TYPE_ETH;
    pcdParam.p_PrsParams = &prsParam;

    /* initialize Parser parameters */
    memset(&kgParam, 0, sizeof( t_FmPortPcdKgParams));
    kgParam.numOfSchemes = p_LnxWrpFmPortDev->numOfSchemesUsed;
    for(i=0;i<kgParam.numOfSchemes;i++)
        kgParam.h_Schemes[i] = p_LnxWrpFmPortDev->h_Schemes[i];

    pcdParam.p_KgParams = &kgParam;

    return FM_PORT_SetPCD(p_LnxWrpFmPortDev->h_Dev, &pcdParam);
}

static t_Error InitFmPortDev(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
#define MY_ADV_CONFIG_CHECK_END                                 \
            RETURN_ERROR(MAJOR, E_INVALID_SELECTION,            \
                         ("Advanced configuration routine"));   \
        if (errCode != E_OK)                                    \
            RETURN_ERROR(MAJOR, errCode, NO_MSG);               \
    }

    int                 i = 0;

    if (!p_LnxWrpFmPortDev->active || p_LnxWrpFmPortDev->h_Dev)
        return E_INVALID_STATE;

    if ((p_LnxWrpFmPortDev->h_Dev = FM_PORT_Config(&p_LnxWrpFmPortDev->settings.param)) == NULL)
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM-port"));

    if ((p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX_10G) ||
        (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX))
    {
        t_Error errCode = E_OK;
        if ((errCode = FM_PORT_ConfigDeqHighPriority(p_LnxWrpFmPortDev->h_Dev, TRUE)) != E_OK)
             RETURN_ERROR(MAJOR, errCode, NO_MSG);
#ifdef FM_QMI_DEQ_OPTIONS_SUPPORT
        if ((errCode = FM_PORT_ConfigDeqPrefetchOption(p_LnxWrpFmPortDev->h_Dev, e_FM_PORT_DEQ_FULL_PREFETCH)) != E_OK)
             RETURN_ERROR(MAJOR, errCode, NO_MSG);
#endif /* FM_QMI_DEQ_OPTIONS_SUPPORT */
    }

    /* Call the driver's advanced configuration routines, if requested:
       Compare the function pointer of each entry to the available routines,
       and invoke the matching routine with proper casting of arguments. */
    while (p_LnxWrpFmPortDev->settings.advConfig[i].p_Function && (i < FM_MAX_NUM_OF_ADV_SETTINGS))
    {
        ADV_CONFIG_CHECK_START(&(p_LnxWrpFmPortDev->settings.advConfig[i]))

        ADV_CONFIG_CHECK(p_LnxWrpFmPortDev->h_Dev, FM_PORT_ConfigBufferPrefixContent,   PARAMS(1, (t_FmPortBufferPrefixContent*)))

        MY_ADV_CONFIG_CHECK_END

        /* Advance to next advanced configuration entry */
        i++;
    }

    if (FM_PORT_Init(p_LnxWrpFmPortDev->h_Dev) != E_OK)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    /* Set for 10G ports for performance purposes... */
    if ((p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX_10G) ||
        (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX_10G))
    {
        t_Error      errCode = E_OK;
        t_FmPortRsrc numOfOpenDmas;
        t_FmPortRsrc numOfTasks;

        numOfOpenDmas.num = (p_LnxWrpFmPortDev->settings.param.portType ==
                        e_FM_PORT_TYPE_TX_10G) ? 12 : 8; 
        numOfOpenDmas.extra = 0;

        if ((errCode = FM_PORT_SetNumOfOpenDmas(p_LnxWrpFmPortDev->h_Dev, &numOfOpenDmas)) != E_OK)
             RETURN_ERROR(MAJOR, errCode, NO_MSG);

        numOfTasks.num = 22;  /* performance requirements impose these values */
        numOfTasks.extra = 8;

        if ((errCode = FM_PORT_SetNumOfTasks(p_LnxWrpFmPortDev->h_Dev,
                                &numOfTasks)) != E_OK)
                RETURN_ERROR(MAJOR, errCode, NO_MSG);
    }

    /*Do we still need this one?*/
#if 0
    {
        t_FmPortRsrc    portRsrc;
        t_Error         errCode;

        if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX_10G)
//              portRsrc.num = (6 + 3 + 7) * 256; /* on P4080, and for MAXFRM=1522 */
            portRsrc.num = 21*1024;
        else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX_10G)
//              portRsrc.num = (7 + 7) * 256; /* on P4080, for bpool size=1588 */
            portRsrc.num = 22*1024 + 512;
        else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX)
//              portRsrc.num = (6 + 3 + 1) * 256; /* on both P4080 and P1023, for MAXFRM=1522 */
            portRsrc.num = 10*1024 + 768;
        else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX)
//              portRsrc.num = (7 + 7) * 256; /* on P4080, for bpool size=1588 */
            portRsrc.num = 10*1024;
        else
//              portRsrc.num = (2 + 4) * 256; /* on P4080 (>> ?BMI brings only the frame header? <<) */
            portRsrc.num = 1536; /* OH ports */

       /* Using the following formulae (*), under a set of simplifying assumptions (.):
        *  . all ports are configured in Normal Mode (rather than Independent Mode)
        *  . the DPAA Eth driver allocates buffers of size:
        *      . MAXFRM + NET_IP_ALIGN + DPA_PRIV_DATA_SIZE + DPA_PARSE_RESULTS_SIZE + DPA_HASH_RESULTS_SIZE, i.e.:
        *        MAXFRM + 2 + 16 + sizeof(t_FmPrsResult) + 16, i.e.:
        *        MAXFRM + 66
        *  . excessive buffer pools not accounted for
        *
        *  * for Rx ports on P4080:
        *      . IFSZ = ceil(max(FMBM_EBMPI[PBS]) / 256) * 256 + 7 * 256
        *      . no internal frame offset (FMBM_RIM[FOF] == 0) - otherwise, add up to 256 to the above
        *
        *  * for Rx ports on P1023:
        *      . IFSZ = ceil(second_largest(FMBM_EBMPI[PBS] / 256)) * 256 + 7 * 256, if at least 2 bpools are configured
        *      . IFSZ = 8 * 256, if only a single bpool is configured
        *
        *  * for Tx ports:
        *      . IFSZ = ceil(frame_size / 256) * 256 + 3 * 256 + FMBM_TFP[DPDE] * 256, i.e.:
        *        IFSZ = ceil(MAXFRM / 256) * 256 + 3 x 256 + FMBM_TFP[DPDE] * 256
        *
        *  * for OH ports on P4080:
        *      . IFSZ = ceil(frame_size / 256) * 256 + 1 * 256 + FMBM_PP[MXT] * 256
        *  * for OH ports on P1023:
        *      . IFSZ = ceil(frame_size / 256) * 256 + 3 * 256 + FMBM_TFP[DPDE] * 256
        *  * for both P4080 and P1023:
        *      . (conservative decisions, assuming that BMI must bring the entire frame, not only the frame header)
        *      . no internal frame offset (FMBM_OIM[FOF] == 0) - otherwise, add up to 256 to the above
        *
        *  . for P4080/P5020/P3041/P2040, DPDE is:
        *              > 0 or 1, for 1Gb ports, HW default: 0
        *              > 2..7 (recommended: 3..7) for 10Gb ports, HW default: 3
        *  . for P1023, DPDE should be 1
        *
        *  . for P1023, MXT is in range (0..31)
        *  . for P4080, MXT is in range (0..63)
        *
        */

        portRsrc.extra = 0;
        if ((errCode = FM_PORT_SetSizeOfFifo(p_LnxWrpFmPortDev->h_Dev, &portRsrc)) != E_OK)
             RETURN_ERROR(MAJOR, errCode, NO_MSG);
    }
#endif
/* TODO: Implement f_FM_MAC_SetException...
    if (((t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev)->err_irq == 0) {
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_MDIO_SCAN_EVENTMDIO,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_MDIO_CMD_CMPL,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_REM_FAULT,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_LOC_FAULT,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_1TX_ECC_ER,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_TX_FIFO_UNFL,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_TX_FIFO_OVFL,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_TX_ER,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_RX_FIFO_OVFL,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_RX_ECC_ER,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_RX_JAB_FRM,FALSE);
        FM_MAC_SetException(p_LnxWrpFmPortDev->h_Dev, e_FM_MAC_EX_10G_RX_OVRSZ_FRM,FALSE);
    }
*/

    if ((p_LnxWrpFmPortDev->defPcd != e_NO_PCD) &&
        (InitFmPort3TupleDefPcd(p_LnxWrpFmPortDev) != E_OK))
        RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    return E_OK;
}

static void FreeFmDev(t_LnxWrpFmDev  *p_LnxWrpFmDev)
{
    if (!p_LnxWrpFmDev->active)
        return;

    if (p_LnxWrpFmDev->h_PcdDev)
        FM_PCD_Free(p_LnxWrpFmDev->h_PcdDev);

    if (p_LnxWrpFmDev->h_Dev)
        FM_Free(p_LnxWrpFmDev->h_Dev);

    if (p_LnxWrpFmDev->h_MuramDev)
        FM_MURAM_Free(p_LnxWrpFmDev->h_MuramDev);

    if (p_LnxWrpFmDev->fmRtcBaseAddr)
    {
        SYS_UnregisterIoMap(p_LnxWrpFmDev->fmRtcBaseAddr);
        devm_iounmap(p_LnxWrpFmDev->dev, UINT_TO_PTR(p_LnxWrpFmDev->fmRtcBaseAddr));
        __devm_release_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res, p_LnxWrpFmDev->fmRtcBaseAddr, p_LnxWrpFmDev->fmRtcMemSize);
    }
    SYS_UnregisterIoMap(p_LnxWrpFmDev->fmMuramBaseAddr);
    devm_iounmap(p_LnxWrpFmDev->dev, UINT_TO_PTR(p_LnxWrpFmDev->fmMuramBaseAddr));
    __devm_release_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res, p_LnxWrpFmDev->fmMuramBaseAddr, p_LnxWrpFmDev->fmMuramMemSize);
    SYS_UnregisterIoMap(p_LnxWrpFmDev->fmBaseAddr);
    devm_iounmap(p_LnxWrpFmDev->dev, UINT_TO_PTR(p_LnxWrpFmDev->fmBaseAddr));
    release_mem_region(p_LnxWrpFmDev->fmBaseAddr, p_LnxWrpFmDev->fmMemSize);
//    devm_release_mem_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->fmBaseAddr, p_LnxWrpFmDev->fmMemSize);
}

static void FreeFmPortDev(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;

    if (!p_LnxWrpFmPortDev->active)
        return;

    if (p_LnxWrpFmPortDev->h_Dev)
        FM_PORT_Free(p_LnxWrpFmPortDev->h_Dev);
    devm_iounmap(p_LnxWrpFmDev->dev, UINT_TO_PTR(p_LnxWrpFmPortDev->baseAddr));
    __devm_release_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res, p_LnxWrpFmPortDev->baseAddr, p_LnxWrpFmPortDev->memSize);
}


/*****************************************************************************/
/*               API routines for the FM Linux Device                        */
/*****************************************************************************/

static int fm_open(struct inode *inode, struct file *file)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = NULL;
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = NULL;
    unsigned int        major = imajor(inode);
    unsigned int        minor = iminor(inode);

    DBG(TRACE, ("Opening minor - %d - ", minor));

    if (file->private_data != NULL)
        return 0;

#ifdef NO_OF_SUPPORT
{
    int                 i;
    for (i=0; i<INTG_MAX_NUM_OF_FM; i++)
        if (lnxWrpFm.p_FmDevs[i]->major == major)
            p_LnxWrpFmDev = lnxWrpFm.p_FmDevs[i];
}
#else
{
    struct of_device_id name;
    struct device_node  *fm_node;

    /* Get all the FM nodes */
    memset(&name, 0, sizeof(struct of_device_id));
    BUG_ON(strlen("fsl,fman") >= sizeof(name.compatible));
    strcpy(name.compatible, "fsl,fman");
    for_each_matching_node(fm_node, &name) {
        struct of_device    *of_dev;

        of_dev = of_find_device_by_node(fm_node);
        if (unlikely(of_dev == NULL)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("fm id!"));
            return -ENXIO;
        }

        p_LnxWrpFmDev = (t_LnxWrpFmDev *)fm_bind(&of_dev->dev);
        if (p_LnxWrpFmDev->major == major)
            break;
        fm_unbind((struct fm *)p_LnxWrpFmDev);
        p_LnxWrpFmDev = NULL;
    }
}
#endif /* NO_OF_SUPPORT */

    if (!p_LnxWrpFmDev)
        return -ENODEV;

    if (minor == DEV_FM_MINOR_BASE)
        file->private_data = p_LnxWrpFmDev;
    else if (minor == DEV_FM_PCD_MINOR_BASE)
        file->private_data = p_LnxWrpFmDev;
    else {
        if (minor == DEV_FM_OH_PORTS_MINOR_BASE)
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->hcPort;
        else if ((minor > DEV_FM_OH_PORTS_MINOR_BASE) && (minor < DEV_FM_RX_PORTS_MINOR_BASE))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->opPorts[minor-DEV_FM_OH_PORTS_MINOR_BASE-1];
        else if ((minor >= DEV_FM_RX_PORTS_MINOR_BASE) && (minor < DEV_FM_TX_PORTS_MINOR_BASE))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[minor-DEV_FM_RX_PORTS_MINOR_BASE];
        else if ((minor >= DEV_FM_TX_PORTS_MINOR_BASE) && (minor < DEV_FM_MAX_MINORS))
            p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[minor-DEV_FM_TX_PORTS_MINOR_BASE];
        else
            return -EINVAL;

        /* if trying to open port, check if it initialized */
        if (!p_LnxWrpFmPortDev->h_Dev)
            return -ENODEV;

        p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *)fm_port_bind(p_LnxWrpFmPortDev->dev);
        file->private_data = p_LnxWrpFmPortDev;
        fm_unbind((struct fm *)p_LnxWrpFmDev);
    }

    if (file->private_data == NULL)
         return -ENXIO;

    return 0;
}

static int fm_close(struct inode *inode, struct file *file)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev;
    unsigned int        minor = iminor(inode);
    int                 err = 0;

    DBG(TRACE, ("Closing minor - %d - ", minor));

    if ((minor == DEV_FM_MINOR_BASE) ||
        (minor == DEV_FM_PCD_MINOR_BASE))
    {
        p_LnxWrpFmDev = (t_LnxWrpFmDev*)file->private_data;
        if (!p_LnxWrpFmDev)
            return -ENODEV;
        fm_unbind((struct fm *)p_LnxWrpFmDev);
    }
    else if (((minor >= DEV_FM_OH_PORTS_MINOR_BASE) && (minor < DEV_FM_RX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_RX_PORTS_MINOR_BASE) && (minor < DEV_FM_TX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_TX_PORTS_MINOR_BASE) && (minor < DEV_FM_MAX_MINORS)))
    {
        p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)file->private_data;
        if (!p_LnxWrpFmPortDev)
            return -ENODEV;
        fm_port_unbind((struct fm_port *)p_LnxWrpFmPortDev);
    }

    return err;
}

static int fm_ioctls(unsigned int minor, struct file *file, unsigned int cmd, unsigned long arg, bool compat)
{
    DBG(TRACE, ("IOCTL minor - %u, cmd - 0x%08x, arg - 0x%08lx \n", minor, cmd, arg));

    if ((minor == DEV_FM_MINOR_BASE) ||
        (minor == DEV_FM_PCD_MINOR_BASE))
    {
        t_LnxWrpFmDev *p_LnxWrpFmDev = ((t_LnxWrpFmDev*)file->private_data);
        if (!p_LnxWrpFmDev)
            return -ENODEV;
        if (LnxwrpFmIOCTL(p_LnxWrpFmDev, cmd, arg, compat))
            return -EFAULT;
    }
    else if (((minor >= DEV_FM_OH_PORTS_MINOR_BASE) && (minor < DEV_FM_RX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_RX_PORTS_MINOR_BASE) && (minor < DEV_FM_TX_PORTS_MINOR_BASE)) ||
             ((minor >= DEV_FM_TX_PORTS_MINOR_BASE) && (minor < DEV_FM_MAX_MINORS)))
    {
        t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = ((t_LnxWrpFmPortDev*)file->private_data);
        if (!p_LnxWrpFmPortDev)
            return -ENODEV;
        if (LnxwrpFmPortIOCTL(p_LnxWrpFmPortDev, cmd, arg, compat))
            return -EFAULT;
    }
    else
    {
        REPORT_ERROR(MINOR, E_INVALID_VALUE, ("minor"));
        return -ENODEV;
    }

    return 0;
}

#ifdef CONFIG_COMPAT
static long fm_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned int minor = iminor(file->f_path.dentry->d_inode);
    long res;

    fm_mutex_lock();
    res = fm_ioctls(minor, file, cmd, arg, true);
    fm_mutex_unlock();

    return res;
}
#endif

static long fm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned int minor = iminor(file->f_path.dentry->d_inode);
    long res;

    fm_mutex_lock();
    res = fm_ioctls(minor, file, cmd, arg, false);
    fm_mutex_unlock();

    return res;
}

/* Globals for FM character device */
static struct file_operations fm_fops =
{
    .owner =            THIS_MODULE,
    .unlocked_ioctl =   fm_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl =     fm_compat_ioctl,
#endif
    .open =             fm_open,
    .release =          fm_close,
};

#ifndef NO_OF_SUPPORT
static int /*__devinit*/ fm_probe(struct of_device *of_dev, const struct of_device_id *match)
{
    t_LnxWrpFmDev   *p_LnxWrpFmDev;

    if ((p_LnxWrpFmDev = ReadFmDevTreeNode(of_dev)) == NULL)
        return -EIO;
    if (ConfigureFmDev(p_LnxWrpFmDev) != E_OK)
        return -EIO;
    if (InitFmDev(p_LnxWrpFmDev) != E_OK)
        return -EIO;

    Sprint (p_LnxWrpFmDev->name, "%s%d", DEV_FM_NAME, p_LnxWrpFmDev->id);

    /* Register to the /dev for IOCTL API */
    /* Register dynamically a new major number for the character device: */
    if ((p_LnxWrpFmDev->major = register_chrdev(0, p_LnxWrpFmDev->name, &fm_fops)) <= 0) {
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Failed to allocate a major number for device \"%s\"", p_LnxWrpFmDev->name));
        return -EIO;
    }

    /* Creating classes for FM */
    DBG(TRACE ,("class_create fm_class"));
    p_LnxWrpFmDev->fm_class = class_create(THIS_MODULE, p_LnxWrpFmDev->name);
    if (IS_ERR(p_LnxWrpFmDev->fm_class)) {
        unregister_chrdev(p_LnxWrpFmDev->major, p_LnxWrpFmDev->name);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("class_create error fm_class"));
        return -EIO;
    }

    device_create(p_LnxWrpFmDev->fm_class, NULL, MKDEV(p_LnxWrpFmDev->major, DEV_FM_MINOR_BASE), NULL,
                  "fm%d", p_LnxWrpFmDev->id);
    device_create(p_LnxWrpFmDev->fm_class, NULL, MKDEV(p_LnxWrpFmDev->major, DEV_FM_PCD_MINOR_BASE), NULL,
                  "fm%d-pcd", p_LnxWrpFmDev->id);
    dev_set_drvdata(p_LnxWrpFmDev->dev, p_LnxWrpFmDev);

   /* create sysfs entries for stats and regs */
    if ( fm_sysfs_create(p_LnxWrpFmDev->dev) !=0 )
    {
        FreeFmDev(p_LnxWrpFmDev);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Unable to sysfs entry - fm!!!"));
        return -EIO;
    }

    DBG(TRACE, ("FM%d probed", p_LnxWrpFmDev->id));

    return 0;
}

static int __devexit fm_remove(struct of_device *of_dev)
{
    t_LnxWrpFmDev   *p_LnxWrpFmDev;
    struct device   *dev;

    dev = &of_dev->dev;
    p_LnxWrpFmDev = dev_get_drvdata(dev);

    fm_sysfs_distroy(dev);

    DBG(TRACE, ("destroy fm_class"));
    device_destroy(p_LnxWrpFmDev->fm_class, MKDEV(p_LnxWrpFmDev->major, DEV_FM_MINOR_BASE));
    device_destroy(p_LnxWrpFmDev->fm_class, MKDEV(p_LnxWrpFmDev->major, DEV_FM_PCD_MINOR_BASE));
    class_destroy(p_LnxWrpFmDev->fm_class);

    /* Destroy chardev */
    unregister_chrdev(p_LnxWrpFmDev->major, p_LnxWrpFmDev->name);

    FreeFmDev(p_LnxWrpFmDev);

    DistroyFmDev(p_LnxWrpFmDev);

    dev_set_drvdata(dev, NULL);

    return 0;
}

static const struct of_device_id fm_match[] __devinitconst = {
    {
        .compatible    = "fsl,fman"
    },
    {}
};
#ifndef MODULE
MODULE_DEVICE_TABLE(of, fm_match);
#endif /* !MODULE */

static struct of_platform_driver fm_driver = {
    .name           = "fsl-fman",
    .match_table    = fm_match,
    .owner          = THIS_MODULE,
    .probe          = fm_probe,
    .remove         = __devexit_p(fm_remove)
};

static int /*__devinit*/ fm_port_probe(struct of_device *of_dev, const struct of_device_id *match)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev;
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    struct device       *dev;

    dev = &of_dev->dev;

    if ((p_LnxWrpFmPortDev = ReadFmPortDevTreeNode(of_dev)) == NULL)
        return -EIO;
    if (ConfigureFmPortDev(p_LnxWrpFmPortDev) != E_OK)
        return -EIO;

#if 0
    if ((p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_RX_10G) &&
        (p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_RX) &&
        (p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_TX_10G) &&
        (p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_TX) &&
        (InitFmPortDev(p_LnxWrpFmPortDev) != E_OK))
        return -EIO;
#endif /* 0 */

    dev_set_drvdata(dev, p_LnxWrpFmPortDev);

    if ((p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_OH_HOST_COMMAND) &&
        (InitFmPcdDev((t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev) != E_OK))
        return -EIO;

    p_LnxWrpFmDev = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;

    if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX)
    {
        Sprint (p_LnxWrpFmPortDev->name, "%s-port-rx%d", p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id);
        p_LnxWrpFmPortDev->minor = p_LnxWrpFmPortDev->id + DEV_FM_RX_PORTS_MINOR_BASE;
    }
    else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX_10G)
    {
        Sprint (p_LnxWrpFmPortDev->name, "%s-port-rx%d", p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id+FM_MAX_NUM_OF_1G_RX_PORTS);
        p_LnxWrpFmPortDev->minor = p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_RX_PORTS + DEV_FM_RX_PORTS_MINOR_BASE;
    }
    else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX)
    {
        Sprint (p_LnxWrpFmPortDev->name, "%s-port-tx%d", p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id);
        p_LnxWrpFmPortDev->minor = p_LnxWrpFmPortDev->id + DEV_FM_TX_PORTS_MINOR_BASE;
    }
    else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX_10G)
    {
        Sprint (p_LnxWrpFmPortDev->name, "%s-port-tx%d", p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id+FM_MAX_NUM_OF_1G_TX_PORTS);
        p_LnxWrpFmPortDev->minor = p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_TX_PORTS + DEV_FM_TX_PORTS_MINOR_BASE;
    }
    else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_OH_HOST_COMMAND)
    {
        Sprint (p_LnxWrpFmPortDev->name, "%s-port-oh%d", p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id);
        p_LnxWrpFmPortDev->minor = p_LnxWrpFmPortDev->id + DEV_FM_OH_PORTS_MINOR_BASE;
    }
    else if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
    {
        Sprint (p_LnxWrpFmPortDev->name, "%s-port-oh%d", p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id+1);
        p_LnxWrpFmPortDev->minor = p_LnxWrpFmPortDev->id + 1 + DEV_FM_OH_PORTS_MINOR_BASE;
    }

    device_create(p_LnxWrpFmDev->fm_class, NULL, MKDEV(p_LnxWrpFmDev->major, p_LnxWrpFmPortDev->minor), NULL, p_LnxWrpFmPortDev->name);

    /* create sysfs entries for stats and regs */

    if (fm_port_sysfs_create(dev) !=0 )
    {
        FreeFmDev(p_LnxWrpFmDev);
        REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Unable to sys entry - fm port!!!"));
        return -EIO;
    }

#ifdef FM_TX_INVALID_ECC_ERRATA_10GMAC_A009
    FM_DisableRamsEcc(p_LnxWrpFmDev->h_Dev);
#endif /* FM_TX_INVALID_ECC_ERRATA_10GMAC_A009 */

    DBG(TRACE, ("%s probed", p_LnxWrpFmPortDev->name));

    return 0;
}

static int __devexit fm_port_remove(struct of_device *of_dev)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev;
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    struct device       *dev;

    dev = &of_dev->dev;
    p_LnxWrpFmPortDev = dev_get_drvdata(dev);

    fm_port_sysfs_destroy(dev);

    p_LnxWrpFmDev = (t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev;
    device_destroy(p_LnxWrpFmDev->fm_class, MKDEV(p_LnxWrpFmDev->major, p_LnxWrpFmPortDev->minor));

    FreeFmPortDev(p_LnxWrpFmPortDev);

    dev_set_drvdata(dev, NULL);

    return 0;
}

static const struct of_device_id fm_port_match[] __devinitconst = {
    {
        .compatible    = "fsl,fman-port-oh"
    },
    {
        .compatible    = "fsl,fman-port-1g-rx"
    },
    {
        .compatible    = "fsl,fman-port-10g-rx"
    },
    {
        .compatible    = "fsl,fman-port-1g-tx"
    },
    {
        .compatible    = "fsl,fman-port-10g-tx"
    },
    {}
};
#ifndef MODULE
MODULE_DEVICE_TABLE(of, fm_port_match);
#endif /* !MODULE */

static struct of_platform_driver fm_port_driver = {
    .name           = "fsl-fman-port",
    .match_table    = fm_port_match,
    .owner          = THIS_MODULE,
    .probe          = fm_port_probe,
    .remove         = __devexit_p(fm_port_remove)
};
#endif /* !NO_OF_SUPPORT */


t_Handle LNXWRP_FM_Init(void)
{
#ifdef NO_OF_SUPPORT
    t_LnxWrpFmDev   *p_LnxWrpFmDev;
    int             i, j;
#endif /* NO_OF_SUPPORT */

    memset(&lnxWrpFm, 0, sizeof(lnxWrpFm));
    spin_lock_init(&lock);
    mutex_init(&lnxwrp_mutex);

#ifdef NO_OF_SUPPORT
    for (i=0; i<INTG_MAX_NUM_OF_FM; i++)
    {
        p_LnxWrpFmDev = CreateFmDev(i);
        if (!p_LnxWrpFmDev)
        {
            REPORT_ERROR(MAJOR, E_INVALID_HANDLE, ("FM dev obj!"));
            return NULL;
        }
        lnxWrpFm.p_FmDevs[i] = p_LnxWrpFmDev;

        if (i==1)
        {
            ConfigureFmDev(p_LnxWrpFmDev);
            if (InitFmDev(p_LnxWrpFmDev) != E_OK)
                return NULL;

            Sprint (p_LnxWrpFmDev->name, "%s%d", DEV_FM_NAME, p_LnxWrpFmDev->id);

            /* Register to the /dev for IOCTL API */
            /* Register dynamically a new major number for the character device: */
            if ((p_LnxWrpFmDev->major = register_chrdev(0, p_LnxWrpFmDev->name, &fm_fops)) <= 0)
            {
                REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Failed to allocate a major number for device \"%s\"", p_LnxWrpFmDev->name));
                return NULL;
            }

            /* Register to the /proc for debug and statistics API */
            if (((p_LnxWrpFmDev->proc_fm = proc_mkdir(p_LnxWrpFmDev->name, NULL)) == NULL) ||
                ((p_LnxWrpFmDev->proc_fm_regs = create_proc_read_entry("regs", 0, p_LnxWrpFmDev->proc_fm, fm_proc_dump_regs, p_LnxWrpFmDev)) == NULL) ||
                ((p_LnxWrpFmDev->proc_fm_stats = create_proc_read_entry("stats", 0, p_LnxWrpFmDev->proc_fm, fm_proc_dump_stats, p_LnxWrpFmDev)) == NULL))
            {
                REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Unable to create proc entry - fm!!!"));
                return NULL;
            }
        }

//        lnxWrpFm.h_Mod = p_TdmLnxWrpParam->h_Mod;
//        lnxWrpFm.f_GetObject = p_TdmLnxWrpParam->f_GetObject;
    }

#else
    /* Register to the DTB for basic FM API */
    of_register_platform_driver(&fm_driver);
    /* Register to the DTB for basic FM port API */
    of_register_platform_driver(&fm_port_driver);
#endif /* !NO_OF_SUPPORT */

#ifdef CONFIG_FSL_FMAN_TEST
    /* Seed the QMan allocator so we'll have enough queues to run PCD with
       dinamically fqid-range allocation */
    qman_release_fqid_range(0x100, 0x400);
#endif /* CONFIG_FSL_FMAN_TEST */

    return &lnxWrpFm;
}

t_Error LNXWRP_FM_Free(t_Handle h_LnxWrpFm)
{
#ifdef NO_OF_SUPPORT
    t_LnxWrpFm          *p_LnxWrpFm = (t_LnxWrpFm *)h_LnxWrpFm;
    t_LnxWrpFmDev       *p_LnxWrpFmDev;
    int                 i, j;

    for (i=0; i<INTG_MAX_NUM_OF_FM; i++)
    {
        p_LnxWrpFmDev = p_LnxWrpFm->p_FmDevs[i];

        remove_proc_entry("stats", p_LnxWrpFmDev->proc_fm);
        remove_proc_entry("regs", p_LnxWrpFmDev->proc_fm);
        remove_proc_entry(p_LnxWrpFmDev->name, NULL);

        /* Destroy chardev */
        unregister_chrdev(p_LnxWrpFmDev->major, p_LnxWrpFmDev->name);

        FreeFmDev(p_LnxWrpFmDev);

        DistroyFmDev(p_LnxWrpFmDev);
    }

#else
        of_unregister_platform_driver(&fm_port_driver);
        of_unregister_platform_driver(&fm_driver);
#endif /* NO_OF_SUPPORT */
    mutex_destroy(&lnxwrp_mutex);

    return E_OK;
}


struct fm * fm_bind (struct device *fm_dev)
{
    return (struct fm *)(dev_get_drvdata(get_device(fm_dev)));
}
EXPORT_SYMBOL(fm_bind);

void fm_unbind(struct fm *fm)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev*)fm;

    put_device(p_LnxWrpFmDev->dev);
}
EXPORT_SYMBOL(fm_unbind);

struct resource * fm_get_mem_region(struct fm *fm)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev*)fm;

    return p_LnxWrpFmDev->res;
}
EXPORT_SYMBOL(fm_get_mem_region);

void * fm_get_handle(struct fm *fm)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev*)fm;

    return (void *)p_LnxWrpFmDev->h_Dev;
}
EXPORT_SYMBOL(fm_get_handle);

void * fm_get_rtc_handle(struct fm *fm)
{
    t_LnxWrpFmDev       *p_LnxWrpFmDev = (t_LnxWrpFmDev*)fm;

    return (void *)p_LnxWrpFmDev->h_RtcDev;
}
EXPORT_SYMBOL(fm_get_rtc_handle);

struct fm_port * fm_port_bind (struct device *fm_port_dev)
{
    return (struct fm_port *)(dev_get_drvdata(get_device(fm_port_dev)));
}
EXPORT_SYMBOL(fm_port_bind);

void fm_port_unbind(struct fm_port *port)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    put_device(p_LnxWrpFmPortDev->dev);
}
EXPORT_SYMBOL(fm_port_unbind);

void * fm_port_get_handle(struct fm_port *port)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    return (void *)p_LnxWrpFmPortDev->h_Dev;
}
EXPORT_SYMBOL(fm_port_get_handle);

void fm_port_get_base_addr(const struct fm_port *port, uint64_t *base_addr)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *)port;

    *base_addr = p_LnxWrpFmPortDev->settings.param.baseAddr;
}
EXPORT_SYMBOL(fm_port_get_base_addr);

void fm_set_rx_port_params(struct fm_port *port, struct fm_port_rx_params *params)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;
    int                 i;

    p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.errFqid  = params->errq;
    p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.dfltFqid = params->defq;
    p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.extBufPools.numOfPoolsUsed = params->num_pools;
    for (i=0; i<params->num_pools; i++)
    {
        p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.extBufPools.extBufPool[i].id =
            params->pool_param[i].id;
        p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.extBufPools.extBufPool[i].size =
            params->pool_param[i].size;
    }

    p_LnxWrpFmPortDev->buffPrefixContent.privDataSize     = params->priv_data_size;
    p_LnxWrpFmPortDev->buffPrefixContent.passPrsResult    = params->parse_results;
    p_LnxWrpFmPortDev->buffPrefixContent.passHashResult   = params->hash_results;
    p_LnxWrpFmPortDev->buffPrefixContent.passTimeStamp    = params->time_stamp;

    ADD_ADV_CONFIG_START(p_LnxWrpFmPortDev->settings.advConfig, FM_MAX_NUM_OF_ADV_SETTINGS)

    ADD_ADV_CONFIG_NO_RET(FM_PORT_ConfigBufferPrefixContent,   ARGS(1, (&p_LnxWrpFmPortDev->buffPrefixContent)));

    ADD_ADV_CONFIG_END

    InitFmPortDev(p_LnxWrpFmPortDev);
}
EXPORT_SYMBOL(fm_set_rx_port_params);

void fm_port_pcd_bind (struct fm_port *port, struct fm_port_pcd_param *params)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    p_LnxWrpFmPortDev->pcd_owner_params.cb      = params->cb;
    p_LnxWrpFmPortDev->pcd_owner_params.dev     = params->dev;
}
EXPORT_SYMBOL(fm_port_pcd_bind);

void fm_set_tx_port_params(struct fm_port *port, struct fm_port_non_rx_params *params)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.errFqid  = params->errq;
    p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.dfltFqid = params->defq;

    p_LnxWrpFmPortDev->buffPrefixContent.privDataSize     = params->priv_data_size;
    p_LnxWrpFmPortDev->buffPrefixContent.passPrsResult    = params->parse_results;
    p_LnxWrpFmPortDev->buffPrefixContent.passHashResult   = params->hash_results;
    p_LnxWrpFmPortDev->buffPrefixContent.passTimeStamp    = params->time_stamp;

    ADD_ADV_CONFIG_START(p_LnxWrpFmPortDev->settings.advConfig, FM_MAX_NUM_OF_ADV_SETTINGS)

    ADD_ADV_CONFIG_NO_RET(FM_PORT_ConfigBufferPrefixContent,   ARGS(1, (&p_LnxWrpFmPortDev->buffPrefixContent)));

    ADD_ADV_CONFIG_END

    InitFmPortDev(p_LnxWrpFmPortDev);
}
EXPORT_SYMBOL(fm_set_tx_port_params);

int fm_get_tx_port_channel(struct fm_port *port)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    return p_LnxWrpFmPortDev->txCh;
}
EXPORT_SYMBOL(fm_get_tx_port_channel);

int fm_port_enable (struct fm_port *port)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    FM_PORT_Enable(p_LnxWrpFmPortDev->h_Dev);

    return 0;
}
EXPORT_SYMBOL(fm_port_enable);

void fm_port_disable(struct fm_port *port)
{
    t_LnxWrpFmPortDev   *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev*)port;

    FM_PORT_Disable(p_LnxWrpFmPortDev->h_Dev);
}
EXPORT_SYMBOL(fm_port_disable);

void fm_mutex_lock(void)
{
    mutex_lock(&lnxwrp_mutex);
}
EXPORT_SYMBOL(fm_mutex_lock);

void fm_mutex_unlock(void)
{
    mutex_unlock(&lnxwrp_mutex);
}
EXPORT_SYMBOL(fm_mutex_unlock);

