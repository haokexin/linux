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
 @File          lnxwrp_fm.h

 @Author        Shlomi Gridish

 @Description   FM Linux wrapper functions.
*//***************************************************************************/
#ifndef __LNXWRP_FM_H__
#define __LNXWRP_FM_H__

#include <linux/fsl_qman.h>    /* struct qman_fq */

#include "std_ext.h"
#include "error_ext.h"
#include "list_ext.h"
#include "procbuff_ext.h"

#include "lnxwrp_fm_ext.h"


#define __ERR_MODULE__      MODULE_FM


#define FM_MAX_NUM_OF_ADV_SETTINGS          10

#define LNXWRP_FM_NUM_OF_SHARED_PROFILES    16

#define FM_PORT_10G_NUM_OF_OPEN_DMA             12   /* Use only for 10G ports - RX and TX */

typedef enum {
    e_NO_PCD = 0,
    e_FM_PCD_3_TUPLE
} e_LnxWrpFmPortPcdDefUseCase;


typedef struct t_FmTestFq {
    struct qman_fq      fq_base;
    t_Handle            h_Arg;
} t_FmTestFq;

typedef struct {
    uint8_t                     id; /* sw port id, see SW_PORT_ID_TO_HW_PORT_ID() in fm_common.h */
    int                         minor;
    char                        name[20];
    bool                        active;
    uint64_t                    baseAddr;               /* Port's *virtual* address */
    uint32_t                    memSize;
    t_WrpFmPortDevSettings      settings;
    uint8_t                     totalNumOfSchemes;
    uint8_t                     schemesBase;
    uint8_t                     numOfSchemesUsed;
    uint32_t                    pcdBaseQ;
    uint16_t                    pcdNumOfQs;
    struct fm_port_pcd_param    pcd_owner_params;
    e_LnxWrpFmPortPcdDefUseCase defPcd;
    t_Handle                    h_DefNetEnv;
    t_Handle                    h_Schemes[FM_PCD_KG_NUM_OF_SCHEMES];
    t_FmPortBufferPrefixContent buffPrefixContent;
    t_Handle                    h_Dev;
    t_Handle                    h_LnxWrpFmDev;
    uint16_t                    txCh;
    struct device               *dev;
    struct device_attribute     *dev_attr_stats;
    struct device_attribute     *dev_attr_regs;
} t_LnxWrpFmPortDev;

typedef struct {
    uint8_t                     id;
    bool                        active;
    uint64_t                    baseAddr;
    uint32_t                    memSize;
    t_WrpFmMacDevSettings       settings;
    t_Handle                    h_Dev;
    t_Handle                    h_LnxWrpFmDev;
} t_LnxWrpFmMacDev;

typedef struct {
    uint8_t                     id;
    char                        name[10];
    bool                        active;
    bool                        pcdActive;
    bool                        prsActive;
    bool                        kgActive;
    bool                        ccActive;
    bool                        plcrActive;
    e_LnxWrpFmPortPcdDefUseCase defPcd;
    uint32_t                    usedSchemes;
    uint8_t                     totalNumOfSharedSchemes;
    uint8_t                     sharedSchemesBase;
    uint8_t                     numOfSchemesUsed;
    uint8_t                     defNetEnvId;
    uint64_t                    fmBaseAddr;
    uint32_t                    fmMemSize;
    uint64_t                    fmMuramBaseAddr;
    uint32_t                    fmMuramMemSize;
    uint64_t                    fmRtcBaseAddr;
    uint32_t                    fmRtcMemSize;
    int                         irq;
    int                         err_irq;
    t_WrpFmDevSettings          fmDevSettings;
    t_WrpFmPcdDevSettings       fmPcdDevSettings;
    t_Handle                    h_Dev;
    uint16_t                    hcCh;

    t_Handle                    h_MuramDev;
    t_Handle                    h_PcdDev;
    t_Handle                    h_RtcDev;

    t_LnxWrpFmPortDev           hcPort;
    t_LnxWrpFmPortDev           opPorts[FM_MAX_NUM_OF_OH_PORTS-1];
    t_LnxWrpFmPortDev           rxPorts[FM_MAX_NUM_OF_RX_PORTS];
    t_LnxWrpFmPortDev           txPorts[FM_MAX_NUM_OF_TX_PORTS];
    t_LnxWrpFmMacDev            macs[FM_MAX_NUM_OF_MACS];

    struct device               *dev;
    struct resource             *res;
    int                         major;
    struct class                *fm_class;
    struct device_attribute     *dev_attr_stats;
    struct device_attribute     *dev_attr_regs;

    struct device_attribute     *dev_pcd_attr_stats;
    struct device_attribute     *dev_pcd_attr_regs;

    struct qman_fq              *hc_tx_conf_fq, *hc_tx_err_fq, *hc_tx_fq;
} t_LnxWrpFmDev;

typedef struct {
//    t_Handle h_Mod;
//    t_Handle (*f_GetObject) (t_Handle h_Mod, e_SysModule devType, ...);

    t_LnxWrpFmDev   *p_FmDevs[INTG_MAX_NUM_OF_FM];
} t_LnxWrpFm;
#define LNXWRP_FM_OBJECT(ptr)   LIST_OBJECT(ptr, t_LnxWrpFm, fms[((t_LnxWrpFmDev *)ptr)->id])


t_Error  LnxwrpFmIOCTL(t_LnxWrpFmDev *p_LnxWrpFmDev, unsigned int cmd, unsigned long arg, bool compat);
t_Error  LnxwrpFmPortIOCTL(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev, unsigned int cmd, unsigned long arg, bool compat);


static __inline__ t_Error AllocSchemesForPort(t_LnxWrpFmDev *p_LnxWrpFmDev, uint8_t numSchemes, uint8_t *p_BaseSchemeNum)
{
    uint32_t    schemeMask;
    uint8_t     i;

    if (!numSchemes)
        RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

    schemeMask = 0x80000000;
    *p_BaseSchemeNum = 0xff;

    for (i=0; schemeMask && numSchemes; schemeMask>>=1, i++)
        if ((p_LnxWrpFmDev->usedSchemes & schemeMask) == 0)
        {
            p_LnxWrpFmDev->usedSchemes |= schemeMask;
            numSchemes--;
            if (*p_BaseSchemeNum==0xff)
                *p_BaseSchemeNum = i;
        }
        else if (*p_BaseSchemeNum!=0xff)
            RETURN_ERROR(MINOR, E_INVALID_STATE, ("Fragmentation on schemes array!!!"));

    if (numSchemes)
        RETURN_ERROR(MINOR, E_FULL, ("schemes!!!"));
    return E_OK;
}


#endif /* __LNXWRP_FM_H__ */
