/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
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


/******************************************************************************
 @File          memac.c

 @Description   FM 10G MAC ...
*//***************************************************************************/

#include "std_ext.h"
#include "string_ext.h"
#include "error_ext.h"
#include "xx_ext.h"
#include "endian_ext.h"
#include "crc_mac_addr_ext.h"
#include "debug_ext.h"

#include "fm_common.h"
#include "memac.h"


/*****************************************************************************/
/*                      Internal routines                                    */
/*****************************************************************************/

static void SetupSgmiiInternalPhy(t_Memac *p_Memac, uint8_t phyAddr)
{
    uint16_t    tmpReg16;

    /* SGMII mode + AN enable */
    tmpReg16 = PHY_SGMII_IF_MODE_AN | PHY_SGMII_IF_MODE_SGMII;
    MEMAC_MII_WritePhyReg(p_Memac, phyAddr, 0x14, tmpReg16);

    /* Dev ability according to SGMII specification */
    tmpReg16 = PHY_SGMII_DEV_ABILITY_SGMII;
    MEMAC_MII_WritePhyReg(p_Memac, phyAddr, 0x4, tmpReg16);

    /* Adjust link timer for SGMII  -
       1.6 ms in units of 8 ns = 2 * 10^5 = 0x30d40 */
    MEMAC_MII_WritePhyReg(p_Memac, phyAddr, 0x13, 0x0003);
    MEMAC_MII_WritePhyReg(p_Memac, phyAddr, 0x12, 0x0d40);

    /* Restart AN */
    tmpReg16 = PHY_SGMII_CR_DEF_VAL | PHY_SGMII_CR_RESET_AN;
    MEMAC_MII_WritePhyReg(p_Memac, phyAddr, 0x0, tmpReg16);
}

static t_Error CheckInitParameters(t_Memac *p_Memac)
{
    e_FmMacType     portType;

    portType = ((ENET_SPEED_FROM_MODE(p_Memac->enetMode) < e_ENET_SPEED_10000) ? e_FM_MAC_1G : e_FM_MAC_10G);

#if (FM_MAX_NUM_OF_10G_MACS > 0)
    if((portType == e_FM_MAC_10G) && (p_Memac->macId >= FM_MAX_NUM_OF_10G_MACS))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("10G MAC ID must be less than %d", FM_MAX_NUM_OF_10G_MACS));
#endif /* (FM_MAX_NUM_OF_10G_MACS > 0) */

    if ((portType == e_FM_MAC_1G) && (p_Memac->macId >= FM_MAX_NUM_OF_1G_MACS))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("1G MAC ID must be less than %d", FM_MAX_NUM_OF_1G_MACS));
    if (p_Memac->addr == 0)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Ethernet MAC must have a valid MAC address"));
    if (!p_Memac->f_Exception)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Uninitialized f_Exception"));
    if (!p_Memac->f_Event)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Uninitialized f_Event"));
    return E_OK;

#ifdef FM_LEN_CHECK_ERRATA_FMAN_SW002
    if (!p_Memac->p_MemacDriverParam->noLengthCheckEnable)
       RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("LengthCheck!"));
#endif /* FM_LEN_CHECK_ERRATA_FMAN_SW002 */
}

/* .............................................................................. */

static void SetDefaultParam(t_MemacDriverParam *p_MemacDriverParam)
{
    p_MemacDriverParam->wanModeEnable            = DEFAULT_wanModeEnable;
    p_MemacDriverParam->promiscuousModeEnable    = DEFAULT_promiscuousEnable;
    p_MemacDriverParam->pauseForwardEnable       = DEFAULT_pauseForwardEnable;
    p_MemacDriverParam->pauseIgnore              = DEFAULT_rxIgnorePause;
    p_MemacDriverParam->txAddrInsEnable          = DEFAULT_txAddrInsEnable;

    p_MemacDriverParam->loopbackEnable           = DEFAULT_loopback;
    p_MemacDriverParam->cmdFrameEnable           = DEFAULT_cmdFrameEnable;
    p_MemacDriverParam->rxErrorDiscard           = DEFAULT_rxErrorDiscard;
    p_MemacDriverParam->phyTxenaOn               = DEFAULT_phyTxenaOn;
    p_MemacDriverParam->sendIdleEnable           = DEFAULT_sendIdleEnable;

    p_MemacDriverParam->noLengthCheckEnable      = !DEFAULT_lengthCheckEnable;

    p_MemacDriverParam->lgthCheckNostdr          = DEFAULT_lgthCheckNostdr;
    p_MemacDriverParam->timeStampEnable          = DEFAULT_timeStampEnable;
    p_MemacDriverParam->padAndCrcEnable          = DEFAULT_padAndCrcEnable;
    p_MemacDriverParam->rxSfdAny                 = DEFAULT_rxSfdAny;
    p_MemacDriverParam->rxPblFwd                 = DEFAULT_rxPblFwd;
    p_MemacDriverParam->txPblFwd                 = DEFAULT_txPblFwd;

    p_MemacDriverParam->txIpgLength              = DEFAULT_txIpgLength;
    p_MemacDriverParam->maxFrameLength           = DEFAULT_maxFrameLength;

    p_MemacDriverParam->debugMode                = DEFAULT_debugMode;

    p_MemacDriverParam->pauseTime                = DEFAULT_pauseTime;
    p_MemacDriverParam->resetOnInit              = DEFAULT_resetOnInit;
}

/* ........................................................................... */

static void MemacErrException(t_Handle h_Memac)
{
    t_Memac             *p_Memac = (t_Memac *)h_Memac;
    uint32_t            event;
    t_MemacMemMap        *p_MemacMemMap = p_Memac->p_MemMap;

    event = GET_UINT32(p_MemacMemMap->ievent);
    /* do not handle MDIO events */
    //event &= ~(IMASK_MDIO_SCAN_EVENTMDIO | IMASK_MDIO_CMD_CMPL);

    event &= GET_UINT32(p_MemacMemMap->imask);

    WRITE_UINT32(p_MemacMemMap->ievent, event);

}


static void FreeInitResources(t_Memac *p_Memac)
{
    e_FmMacType             portType;

    portType =
        ((ENET_SPEED_FROM_MODE(p_Memac->enetMode) < e_ENET_SPEED_10000) ? e_FM_MAC_1G : e_FM_MAC_10G);

    if (portType == e_FM_MAC_10G)
        FmUnregisterIntr(p_Memac->fmMacControllerDriver.h_Fm, e_FM_MOD_10G_MAC, p_Memac->macId, e_FM_INTR_TYPE_ERR);
    else
        FmUnregisterIntr(p_Memac->fmMacControllerDriver.h_Fm, e_FM_MOD_1G_MAC, p_Memac->macId, e_FM_INTR_TYPE_ERR);

    /* release the driver's group hash table */
    FreeHashTable(p_Memac->p_MulticastAddrHash);
    p_Memac->p_MulticastAddrHash =   NULL;

    /* release the driver's individual hash table */
    FreeHashTable(p_Memac->p_UnicastAddrHash);
    p_Memac->p_UnicastAddrHash =     NULL;
}

/* .............................................................................. */

static void HardwareClearAddrInPaddr(t_Memac *p_Memac, uint8_t paddrNum)
{
    WRITE_UINT32(p_Memac->p_MemMap->mac_addr[paddrNum].mac_addr_l, 0x0);
    WRITE_UINT32(p_Memac->p_MemMap->mac_addr[paddrNum].mac_addr_u, 0x0);
}

/* ........................................................................... */

static void HardwareAddAddrInPaddr(t_Memac *p_Memac, uint64_t *p_Addr, uint8_t paddrNum)
{
    uint32_t        tmpReg32 = 0;
    uint64_t        addr = *p_Addr;
    t_MemacMemMap   *p_MemacMemMap = p_Memac->p_MemMap;

    tmpReg32 = (uint32_t)(addr>>16);
    SwapUint32P(&tmpReg32);
    WRITE_UINT32(p_MemacMemMap->mac_addr[paddrNum].mac_addr_l, tmpReg32);

    tmpReg32 = (uint32_t)(addr);
    SwapUint32P(&tmpReg32);
    tmpReg32 >>= 16;
    WRITE_UINT32(p_MemacMemMap->mac_addr[paddrNum].mac_addr_u, tmpReg32);
}

/*****************************************************************************/
/*                     10G MAC API routines                                  */
/*****************************************************************************/

/* .............................................................................. */

static t_Error MemacEnable(t_Handle h_Memac,  e_CommMode mode)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap       *p_MemMap ;
    uint32_t            tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_HANDLE);

    p_MemMap= (t_MemacMemMap*)(p_Memac->p_MemMap);

    tmpReg32 = GET_UINT32(p_MemMap->command_config);

    switch (mode)
    {
        case e_COMM_MODE_NONE:
            tmpReg32 &= ~(CMD_CFG_TX_EN | CMD_CFG_RX_EN);
            break;
        case e_COMM_MODE_RX :
            tmpReg32 |= CMD_CFG_RX_EN ;
            break;
        case e_COMM_MODE_TX :
            tmpReg32 |= CMD_CFG_TX_EN ;
            break;
        case e_COMM_MODE_RX_AND_TX:
            tmpReg32 |= (CMD_CFG_TX_EN | CMD_CFG_RX_EN);
            break;
    }

    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacDisable (t_Handle h_Memac, e_CommMode mode)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap       *p_MemMap ;
    uint32_t            tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_HANDLE);

    p_MemMap= (t_MemacMemMap*)(p_Memac->p_MemMap);

    tmpReg32 = GET_UINT32(p_MemMap->command_config);
    switch (mode)
    {
        case e_COMM_MODE_RX:
            tmpReg32 &= ~CMD_CFG_RX_EN;
            break;
        case e_COMM_MODE_TX:
            tmpReg32 &= ~CMD_CFG_TX_EN;
            break;
        case e_COMM_MODE_RX_AND_TX:
            tmpReg32 &= ~(CMD_CFG_TX_EN | CMD_CFG_RX_EN);
        break;
        default:
            RETURN_ERROR(MINOR, E_INVALID_SELECTION, NO_MSG);
    }
    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacSetPromiscuous(t_Handle h_Memac, bool newVal)
{
    t_Memac       *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap *p_MemacMemMap;
    uint32_t     tmpReg32;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Memac->p_MemacDriverParam, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_NULL_POINTER);

    p_MemacMemMap = p_Memac->p_MemMap;

    tmpReg32 = GET_UINT32(p_MemacMemMap->command_config);

    if (newVal)
        tmpReg32 |= CMD_CFG_PROMIS_EN;
    else
        tmpReg32 &= ~CMD_CFG_PROMIS_EN;

    WRITE_UINT32(p_MemacMemMap->command_config, tmpReg32);

    return E_OK;
}


/*****************************************************************************/
/*                      Memac Configs modification functions                 */
/*****************************************************************************/

/* .............................................................................. */

static t_Error MemacConfigLoopback(t_Handle h_Memac, bool newVal)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);

    p_Memac->p_MemacDriverParam->loopbackEnable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacConfigWan(t_Handle h_Memac, bool newVal)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);

    p_Memac->p_MemacDriverParam->wanModeEnable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacConfigMaxFrameLength(t_Handle h_Memac, uint16_t newVal)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);

    p_Memac->p_MemacDriverParam->maxFrameLength = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacConfigPadAndCrc(t_Handle h_Memac, bool newVal)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);

    p_Memac->p_MemacDriverParam->padAndCrcEnable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacConfigLengthCheck(t_Handle h_Memac, bool newVal)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);

    p_Memac->p_MemacDriverParam->noLengthCheckEnable = !newVal;

    return E_OK;
}


/* .............................................................................. */

static t_Error MemacConfigResetOnInit(t_Handle h_Memac, bool enable)
{
    t_Memac *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);

    p_Memac->p_MemacDriverParam->resetOnInit = enable;

    return E_OK;
}


/*****************************************************************************/
/*                      Memac Run Time API functions                         */
/*****************************************************************************/

static t_Error MemacSetTxPauseFrames(t_Handle h_Memac,
                                     uint8_t  priority,
                                     uint16_t pauseTime,
                                     uint16_t threshTime)
{
    t_Memac          *p_Memac = (t_Memac *)h_Memac;
    uint32_t         tmpReg32 = 0;
    t_MemacMemMap    *p_MemMap;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Memac->p_MemacDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_STATE);

    p_MemMap = (t_MemacMemMap*)(p_Memac->p_MemMap);

    tmpReg32 = GET_UINT32(p_MemMap->command_config);
    if (priority == FM_MAC_NO_PFC)
    {
        tmpReg32 &= ~CMD_CFG_PFC_MODE;
        priority = 0;
    }
    else
        tmpReg32 |= CMD_CFG_PFC_MODE;

    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    tmpReg32 =  GET_UINT32(p_MemMap->pause_quanta[priority/2]);
    tmpReg32 &= (~0xFFFF<<(16*(priority%2)));
    tmpReg32 |= ((uint32_t)pauseTime<<(16*(priority%2)));
    WRITE_UINT32(p_MemMap->pause_quanta[priority/2], tmpReg32);

    tmpReg32 =  GET_UINT32(p_MemMap->pause_thresh[priority/2]);
    tmpReg32 &= (~0xFFFF<<(16*(priority%2)));
    tmpReg32 |= ((uint32_t)threshTime<<(16*(priority%2)));
    WRITE_UINT32(p_MemMap->pause_thresh[priority/2], tmpReg32);

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacSetTxAutoPauseFrames(t_Handle h_Memac,
                                         uint16_t pauseTime)
{
    return MemacSetTxPauseFrames(h_Memac, FM_MAC_NO_PFC, pauseTime, 0);
}

/* .............................................................................. */

static t_Error MemacSetRxIgnorePauseFrames(t_Handle h_Memac, bool en)
{
    t_Memac          *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap    *p_MemMap;
    uint32_t        tmpReg32;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Memac->p_MemacDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_STATE);

    p_MemMap = (t_MemacMemMap*)(p_Memac->p_MemMap);
    tmpReg32 = GET_UINT32(p_MemMap->command_config);
    if (en)
        tmpReg32 |= CMD_CFG_PAUSE_IGNORE;
    else
        tmpReg32 &= ~CMD_CFG_PAUSE_IGNORE;
    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    return E_OK;
}

/* Counters handling */
/* .............................................................................. */

static t_Error MemacGetStatistics(t_Handle h_Memac, t_FmMacStatistics *p_Statistics)
{
    t_Memac          *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap    *p_MemacMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Statistics, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_NULL_POINTER);

    p_MemacMemMap = p_Memac->p_MemMap;

    p_Statistics->eStatPkts64           = (((uint64_t)GET_UINT32(p_MemacMemMap->r64_u)<<32)|GET_UINT32(p_MemacMemMap->r64_l));
    p_Statistics->eStatPkts65to127      = (((uint64_t)GET_UINT32(p_MemacMemMap->r127_u)<<32)|GET_UINT32(p_MemacMemMap->r127_l));
    p_Statistics->eStatPkts128to255     = (((uint64_t)GET_UINT32(p_MemacMemMap->r255_u)<<32)|GET_UINT32(p_MemacMemMap->r255_l));
    p_Statistics->eStatPkts256to511     = (((uint64_t)GET_UINT32(p_MemacMemMap->r511_u)<<32)|GET_UINT32(p_MemacMemMap->r511_l));
    p_Statistics->eStatPkts512to1023    = (((uint64_t)GET_UINT32(p_MemacMemMap->r1023_u)<<32)|GET_UINT32(p_MemacMemMap->r1023_l));
    p_Statistics->eStatPkts1024to1518   = (((uint64_t)GET_UINT32(p_MemacMemMap->r1518_u)<<32)|GET_UINT32(p_MemacMemMap->r1518_l));
    p_Statistics->eStatPkts1519to1522   = (((uint64_t)GET_UINT32(p_MemacMemMap->r1519x_u)<<32)|GET_UINT32(p_MemacMemMap->r1519x_l));
/* */
    p_Statistics->eStatFragments        = (((uint64_t)GET_UINT32(p_MemacMemMap->rfrg_u)<<32)|GET_UINT32(p_MemacMemMap->rfrg_l));
    p_Statistics->eStatJabbers          = (((uint64_t)GET_UINT32(p_MemacMemMap->rjbr_u)<<32)|GET_UINT32(p_MemacMemMap->rjbr_l));

    p_Statistics->eStatsDropEvents      = (((uint64_t)GET_UINT32(p_MemacMemMap->rdrp_u)<<32)|GET_UINT32(p_MemacMemMap->rdrp_l));
    p_Statistics->eStatCRCAlignErrors   = (((uint64_t)GET_UINT32(p_MemacMemMap->raln_u)<<32)|GET_UINT32(p_MemacMemMap->raln_l));

    p_Statistics->eStatUndersizePkts    = (((uint64_t)GET_UINT32(p_MemacMemMap->tund_u)<<32)|GET_UINT32(p_MemacMemMap->tund_l));
    p_Statistics->eStatOversizePkts     = (((uint64_t)GET_UINT32(p_MemacMemMap->rovr_u)<<32)|GET_UINT32(p_MemacMemMap->rovr_l));
/* Pause */
    p_Statistics->reStatPause           = (((uint64_t)GET_UINT32(p_MemacMemMap->rxpf_u)<<32)|GET_UINT32(p_MemacMemMap->rxpf_l));
    p_Statistics->teStatPause           = (((uint64_t)GET_UINT32(p_MemacMemMap->txpf_u)<<32)|GET_UINT32(p_MemacMemMap->txpf_l));

/* MIB II */
    p_Statistics->ifInOctets            = (((uint64_t)GET_UINT32(p_MemacMemMap->roct_u)<<32)|GET_UINT32(p_MemacMemMap->roct_l));
    p_Statistics->ifInMcastPkts         = (((uint64_t)GET_UINT32(p_MemacMemMap->rmca_u)<<32)|GET_UINT32(p_MemacMemMap->rmca_l));
    p_Statistics->ifInBcastPkts         = (((uint64_t)GET_UINT32(p_MemacMemMap->rbca_u)<<32)|GET_UINT32(p_MemacMemMap->rbca_l));
    p_Statistics->ifInPkts              = (((uint64_t)GET_UINT32(p_MemacMemMap->ruca_u)<<32)|GET_UINT32(p_MemacMemMap->ruca_l))
                                        + p_Statistics->ifInMcastPkts
                                        + p_Statistics->ifInBcastPkts;
    p_Statistics->ifInDiscards          = 0;
    p_Statistics->ifInErrors            = (((uint64_t)GET_UINT32(p_MemacMemMap->rerr_u)<<32)|GET_UINT32(p_MemacMemMap->rerr_l));

    p_Statistics->ifOutOctets           = (((uint64_t)GET_UINT32(p_MemacMemMap->toct_u)<<32)|GET_UINT32(p_MemacMemMap->toct_l));
    p_Statistics->ifOutMcastPkts        = (((uint64_t)GET_UINT32(p_MemacMemMap->tmca_u)<<32)|GET_UINT32(p_MemacMemMap->tmca_l));
    p_Statistics->ifOutBcastPkts        = (((uint64_t)GET_UINT32(p_MemacMemMap->tbca_u)<<32)|GET_UINT32(p_MemacMemMap->tbca_l));
    p_Statistics->ifOutPkts             = (((uint64_t)GET_UINT32(p_MemacMemMap->tuca_u)<<32)|GET_UINT32(p_MemacMemMap->tuca_l))
                                            + p_Statistics->ifOutMcastPkts
                                            + p_Statistics->ifOutBcastPkts;
    p_Statistics->ifOutDiscards         = 0;
    p_Statistics->ifOutErrors           = (((uint64_t)GET_UINT32(p_MemacMemMap->terr_u)<<32)|GET_UINT32(p_MemacMemMap->terr_l));

    return E_OK;
}


/* .............................................................................. */

static t_Error MemacModifyMacAddress (t_Handle h_Memac, t_EnetAddr *p_EnetAddr)
{
    t_Memac              *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap        *p_MemacMemMap;
    uint32_t            tmpReg32 = 0;
    uint64_t            addr;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_NULL_POINTER);

    p_MemacMemMap = p_Memac->p_MemMap;

    addr = ((*(uint64_t *)p_EnetAddr) >> 16);
    p_Memac->addr = addr;

    tmpReg32 = (uint32_t)(addr>>16);
    SwapUint32P(&tmpReg32);
    WRITE_UINT32(p_MemacMemMap->mac_addr0.mac_addr_l, tmpReg32);

    tmpReg32 = (uint32_t)(addr);
    SwapUint32P(&tmpReg32);
    tmpReg32 >>= 16;
    WRITE_UINT32(p_MemacMemMap->mac_addr0.mac_addr_u, tmpReg32);

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacResetCounters (t_Handle h_Memac)
{
    t_Memac         *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap   *p_MemMap;
    uint32_t        tmpReg32;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_HANDLE);

    p_MemMap = (t_MemacMemMap*)(p_Memac->p_MemMap);

    tmpReg32 = GET_UINT32(p_MemMap->statn_config);

    tmpReg32 |= STATS_CFG_CLR;

    WRITE_UINT32(p_MemMap->statn_config, tmpReg32);

    while (GET_UINT32(p_MemMap->statn_config) & STATS_CFG_CLR) ;

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacAddExactMatchMacAddress(t_Handle h_Memac, t_EnetAddr *p_EthAddr)
{
    t_Memac   *p_Memac = (t_Memac *) h_Memac;
    uint64_t  ethAddr;
    uint8_t   paddrNum;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);

    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    if (ethAddr & GROUP_ADDRESS)
        /* Multicast address has no effect in PADDR */
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Multicast address"));

    /* Make sure no PADDR contains this address */
    for (paddrNum = 0; paddrNum < MEMAC_NUM_OF_PADDRS; paddrNum++)
        if (p_Memac->indAddrRegUsed[paddrNum])
            if (p_Memac->paddr[paddrNum] == ethAddr)
                RETURN_ERROR(MAJOR, E_ALREADY_EXISTS, NO_MSG);

    /* Find first unused PADDR */
    for (paddrNum = 0; paddrNum < MEMAC_NUM_OF_PADDRS; paddrNum++)
        if (!(p_Memac->indAddrRegUsed[paddrNum]))
        {
            /* mark this PADDR as used */
            p_Memac->indAddrRegUsed[paddrNum] = TRUE;
            /* store address */
            p_Memac->paddr[paddrNum] = ethAddr;

            /* put in hardware */
            HardwareAddAddrInPaddr(p_Memac, &ethAddr, paddrNum);
            p_Memac->numOfIndAddrInRegs++;

            return E_OK;
        }

    /* No free PADDR */
    RETURN_ERROR(MAJOR, E_FULL, NO_MSG);
}

/* .............................................................................. */

static t_Error MemacDelExactMatchMacAddress(t_Handle h_Memac, t_EnetAddr *p_EthAddr)
{
    t_Memac   *p_Memac = (t_Memac *) h_Memac;
    uint64_t  ethAddr;
    uint8_t   paddrNum;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_HANDLE);

    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    /* Find used PADDR containing this address */
    for (paddrNum = 0; paddrNum < MEMAC_NUM_OF_PADDRS; paddrNum++)
    {
        if ((p_Memac->indAddrRegUsed[paddrNum]) &&
            (p_Memac->paddr[paddrNum] == ethAddr))
        {
            /* mark this PADDR as not used */
            p_Memac->indAddrRegUsed[paddrNum] = FALSE;
            /* clear in hardware */
            HardwareClearAddrInPaddr(p_Memac, paddrNum);
            p_Memac->numOfIndAddrInRegs--;

            return E_OK;
        }
    }

    RETURN_ERROR(MAJOR, E_NOT_FOUND, NO_MSG);
}

/* .............................................................................. */

static t_Error MemacAddHashMacAddress(t_Handle h_Memac, t_EnetAddr *p_EthAddr)
{
    t_Memac          *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap    *p_MemacMemMap;
    t_EthHashEntry  *p_HashEntry;
    uint32_t        crc;
    uint32_t        hash;
    uint64_t        ethAddr;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_NULL_POINTER);

    p_MemacMemMap = p_Memac->p_MemMap;
    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    if (!(ethAddr & GROUP_ADDRESS))
        /* Unicast addresses not supported in hash */
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unicast Address"));

    /* CRC calculation */
    GET_MAC_ADDR_CRC(ethAddr, crc);
    crc = MIRROR_32(crc);

    hash = (crc >> HASH_CTRL_MCAST_SHIFT) & HASH_CTRL_ADDR_MASK;        /* Take 6 MSB bits */

    /* Create element to be added to the driver hash table */
    p_HashEntry = (t_EthHashEntry *)XX_Malloc(sizeof(t_EthHashEntry));
    p_HashEntry->addr = ethAddr;
    INIT_LIST(&p_HashEntry->node);

    LIST_AddToTail(&(p_HashEntry->node), &(p_Memac->p_MulticastAddrHash->p_Lsts[hash]));
    WRITE_UINT32(p_MemacMemMap->hashtable_ctrl, (hash | HASH_CTRL_MCAST_EN));

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacDelHashMacAddress(t_Handle h_Memac, t_EnetAddr *p_EthAddr)
{
    t_Memac          *p_Memac = (t_Memac *)h_Memac;
    t_MemacMemMap    *p_MemacMemMap;
    t_EthHashEntry  *p_HashEntry = NULL;
    t_List          *p_Pos;
    uint32_t        crc;
    uint32_t        hash;
    uint64_t        ethAddr;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_NULL_POINTER);

    p_MemacMemMap = p_Memac->p_MemMap;
    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    /* CRC calculation */
    GET_MAC_ADDR_CRC(ethAddr, crc);
    crc = MIRROR_32(crc);

    hash = (crc >> HASH_CTRL_MCAST_SHIFT) & HASH_CTRL_ADDR_MASK;        /* Take 6 MSB bits */

    LIST_FOR_EACH(p_Pos, &(p_Memac->p_MulticastAddrHash->p_Lsts[hash]))
    {

        p_HashEntry = ETH_HASH_ENTRY_OBJ(p_Pos);
        if(p_HashEntry->addr == ethAddr)
        {
            LIST_DelAndInit(&p_HashEntry->node);
            XX_Free(p_HashEntry);
            break;
        }
    }
    if(LIST_IsEmpty(&p_Memac->p_MulticastAddrHash->p_Lsts[hash]))
        WRITE_UINT32(p_MemacMemMap->hashtable_ctrl, (hash & ~HASH_CTRL_MCAST_EN));

    return E_OK;
}

/* .............................................................................. */

/* .............................................................................. */

static uint16_t MemacGetMaxFrameLength(t_Handle h_Memac)
{
    t_Memac              *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_VALUE(p_Memac, E_INVALID_HANDLE, 0);

    return (uint16_t)GET_UINT32(p_Memac->p_MemMap->maxfrm);
}

/* .............................................................................. */

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
static t_Error MemacDumpRegs(t_Handle h_Memac)
{
    return E_OK;
}
#endif /* (defined(DEBUG_ERRORS) && ... */


/*****************************************************************************/
/*                      FM Init & Free API                                   */
/*****************************************************************************/

/* .............................................................................. */

static t_Error MemacInit(t_Handle h_Memac)
{
    t_Memac                 *p_Memac = (t_Memac *)h_Memac;
    t_MemacDriverParam      *p_MemacDriverParam;
    t_MemacMemMap           *p_MemMap;
    uint64_t                addr;
    uint32_t                tmpReg32;
    uint8_t                 i, phyAddr;
    e_FmMacType             portType;
    t_Error                 err;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemacDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->p_MemMap, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Memac->fmMacControllerDriver.h_Fm, E_INVALID_HANDLE);

    /* not needed! */
    /*FM_GetRevision(p_Memac->fmMacControllerDriver.h_Fm, &p_Memac->fmMacControllerDriver.fmRevInfo);*/

    CHECK_INIT_PARAMETERS(p_Memac, CheckInitParameters);

    p_MemacDriverParam = p_Memac->p_MemacDriverParam;
    p_MemMap = p_Memac->p_MemMap;

    portType =
        ((ENET_SPEED_FROM_MODE(p_Memac->enetMode) < e_ENET_SPEED_10000) ? e_FM_MAC_1G : e_FM_MAC_10G);

    /* First, reset the MAC if desired. */
    if (p_MemacDriverParam->resetOnInit)
    {
        tmpReg32 = GET_UINT32(p_MemMap->command_config);
        WRITE_UINT32(p_MemMap->command_config, tmpReg32 | CMD_CFG_SW_RESET);
        XX_UDelay(10);
        do
        {
            tmpReg32 = GET_UINT32(p_MemMap->command_config);
        }
        while (tmpReg32 & CMD_CFG_SW_RESET);
    }

    /* MAC Address */
    addr = p_Memac->addr;
    tmpReg32 = (uint32_t)(addr>>16);
    SwapUint32P(&tmpReg32);
    WRITE_UINT32(p_MemMap->mac_addr0.mac_addr_l, tmpReg32);

    tmpReg32 = (uint32_t)(addr);
    SwapUint32P(&tmpReg32);
    tmpReg32 >>= 16;
    WRITE_UINT32(p_MemMap->mac_addr0.mac_addr_u, tmpReg32);

    /* Config */
    tmpReg32 = 0;
    if (p_MemacDriverParam->wanModeEnable)
        tmpReg32 |= CMD_CFG_WAN_MODE;
    if (p_MemacDriverParam->promiscuousModeEnable)
        tmpReg32 |= CMD_CFG_PROMIS_EN;
    if (p_MemacDriverParam->pauseForwardEnable)
        tmpReg32 |= CMD_CFG_PAUSE_FWD;
    if (p_MemacDriverParam->pauseIgnore)
        tmpReg32 |= CMD_CFG_PAUSE_IGNORE;
    if (p_MemacDriverParam->txAddrInsEnable)
        tmpReg32 |= CMD_CFG_TX_ADDR_INS;
    if (p_MemacDriverParam->loopbackEnable)
        tmpReg32 |= CMD_CFG_LOOPBACK_EN;
    if (p_MemacDriverParam->cmdFrameEnable)
        tmpReg32 |= CMD_CFG_CNT_FRM_EN;
    if (p_MemacDriverParam->sendIdleEnable)
        tmpReg32 |= CMD_CFG_SEND_IDLE;
    if (p_MemacDriverParam->noLengthCheckEnable)
        tmpReg32 |= CMD_CFG_NO_LEN_CHK;
    if (p_MemacDriverParam->rxSfdAny)
        tmpReg32 |= CMD_CFG_SFD_ANY;
    if (p_MemacDriverParam->padAndCrcEnable)
        tmpReg32 |= CMD_CFG_TX_PAD_EN;
    tmpReg32 |= CMD_CFG_CRC_FWD;

    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    /* Set up interface bits */
    tmpReg32 = GET_UINT32(p_MemMap->if_mode) & ~IF_MODE_MASK;
    if (portType == e_FM_MAC_10G)
        WRITE_UINT32(p_MemMap->if_mode, tmpReg32 | IF_MODE_XGMII);
    else
        WRITE_UINT32(p_MemMap->if_mode, tmpReg32 | IF_MODE_GMII);

    if (ENET_INTERFACE_FROM_MODE(p_Memac->enetMode) == e_ENET_IF_SGMII)
    {
        /* Configure internal SGMII phy */
        SetupSgmiiInternalPhy(p_Memac, PHY_MDIO_ADDR);
    }
    else if (ENET_INTERFACE_FROM_MODE(p_Memac->enetMode) == e_ENET_IF_QSGMII)
    {
        /* Configure 4 internal SGMII phys */
        for (i = 0; i < 4; i++)
        {
            /* QSGMII phy address occupies 3 upper bits of 5-bit
               phyAddress; the lower 2 bits are used to extend
               register address space and access each one of 4
               ports inside QSGMII. */
            phyAddr = (uint8_t)((PHY_MDIO_ADDR << 2) | i);
            SetupSgmiiInternalPhy(p_Memac, phyAddr);
        }
    }

    /* Max Frame Length */
    WRITE_UINT32(p_MemMap->maxfrm, (uint32_t)p_MemacDriverParam->maxFrameLength);
    err = FmSetMacMaxFrame(p_Memac->fmMacControllerDriver.h_Fm,
                           portType,
                           p_Memac->fmMacControllerDriver.macId,
                           p_MemacDriverParam->maxFrameLength);

    /* Pause Time */
    WRITE_UINT32(p_MemMap->pause_quanta[0], p_MemacDriverParam->pauseTime);
    WRITE_UINT32(p_MemMap->pause_thresh[0], 0);

    p_Memac->p_MulticastAddrHash = AllocHashTable(HASH_TABLE_SIZE);
    if (!p_Memac->p_MulticastAddrHash)
    {
        FreeInitResources(p_Memac);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("allocation hash table is FAILED"));
    }

    p_Memac->p_UnicastAddrHash = AllocHashTable(HASH_TABLE_SIZE);
    if (!p_Memac->p_UnicastAddrHash)
    {
        FreeInitResources(p_Memac);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("allocation hash table is FAILED"));
    }

    /* interrupts */
    WRITE_UINT32(p_MemMap->ievent, EVENTS_MASK);
    WRITE_UINT32(p_MemMap->imask, p_Memac->exceptions);

    if (portType == e_FM_MAC_10G)
        FmRegisterIntr(p_Memac->fmMacControllerDriver.h_Fm,
                       e_FM_MOD_10G_MAC,
                       p_Memac->macId,
                       e_FM_INTR_TYPE_ERR,
                       MemacErrException,
                       p_Memac);
    else
        FmRegisterIntr(p_Memac->fmMacControllerDriver.h_Fm,
                       e_FM_MOD_1G_MAC,
                       p_Memac->macId,
                       e_FM_INTR_TYPE_ERR,
                       MemacErrException,
                       p_Memac);
    /*if ((p_Memac->mdioIrq != 0) && (p_Memac->mdioIrq != NO_IRQ))
    {
        XX_SetIntr(p_Memac->mdioIrq, MemacException, p_Memac);
        XX_EnableIntr(p_Memac->mdioIrq);
    }
    else if (p_Memac->mdioIrq == 0)
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED, (NO_MSG));*/

    XX_Free(p_MemacDriverParam);
    p_Memac->p_MemacDriverParam = NULL;

    return E_OK;
}

/* .............................................................................. */

static t_Error MemacFree(t_Handle h_Memac)
{
    t_Memac       *p_Memac = (t_Memac *)h_Memac;

    SANITY_CHECK_RETURN_ERROR(p_Memac, E_INVALID_HANDLE);

    FreeInitResources(p_Memac);

    if (p_Memac->p_MemacDriverParam)
    {
        XX_Free(p_Memac->p_MemacDriverParam);
        p_Memac->p_MemacDriverParam = NULL;
    }
    XX_Free (p_Memac);

    return E_OK;
}

/* .............................................................................. */

static void InitFmMacControllerDriver(t_FmMacControllerDriver *p_FmMacControllerDriver)
{
    p_FmMacControllerDriver->f_FM_MAC_Init                      = MemacInit;
    p_FmMacControllerDriver->f_FM_MAC_Free                      = MemacFree;

    p_FmMacControllerDriver->f_FM_MAC_SetStatistics             = NULL;
    p_FmMacControllerDriver->f_FM_MAC_ConfigLoopback            = MemacConfigLoopback;
    p_FmMacControllerDriver->f_FM_MAC_ConfigMaxFrameLength      = MemacConfigMaxFrameLength;

    p_FmMacControllerDriver->f_FM_MAC_ConfigWan                 = MemacConfigWan;

    p_FmMacControllerDriver->f_FM_MAC_ConfigPadAndCrc           = MemacConfigPadAndCrc;
    p_FmMacControllerDriver->f_FM_MAC_ConfigHalfDuplex          = NULL; /* half-duplex is not supported in xgec */
    p_FmMacControllerDriver->f_FM_MAC_ConfigLengthCheck         = MemacConfigLengthCheck;
    p_FmMacControllerDriver->f_FM_MAC_ConfigException           = NULL; //MemacConfigException;
    p_FmMacControllerDriver->f_FM_MAC_ConfigResetOnInit         = NULL; //MemacConfigResetOnInit;

    p_FmMacControllerDriver->f_FM_MAC_SetException              = NULL; //MemacSetExcpetion;

    p_FmMacControllerDriver->f_FM_MAC_Enable1588TimeStamp       = NULL; //MemacEnable1588TimeStamp;
    p_FmMacControllerDriver->f_FM_MAC_Disable1588TimeStamp      = NULL; //MemacDisable1588TimeStamp;

    p_FmMacControllerDriver->f_FM_MAC_SetPromiscuous            = MemacSetPromiscuous;
    p_FmMacControllerDriver->f_FM_MAC_AdjustLink                = NULL;
    p_FmMacControllerDriver->f_FM_MAC_RestartAutoneg            = NULL;

    p_FmMacControllerDriver->f_FM_MAC_Enable                    = MemacEnable;
    p_FmMacControllerDriver->f_FM_MAC_Disable                   = MemacDisable;

    p_FmMacControllerDriver->f_FM_MAC_SetTxAutoPauseFrames      = MemacSetTxAutoPauseFrames;
    p_FmMacControllerDriver->f_FM_MAC_SetTxPauseFrames          = MemacSetTxPauseFrames;
    p_FmMacControllerDriver->f_FM_MAC_SetRxIgnorePauseFrames    = MemacSetRxIgnorePauseFrames;

    p_FmMacControllerDriver->f_FM_MAC_ResetCounters             = MemacResetCounters;
    p_FmMacControllerDriver->f_FM_MAC_GetStatistics             = MemacGetStatistics;

    p_FmMacControllerDriver->f_FM_MAC_ModifyMacAddr             = MemacModifyMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_AddHashMacAddr            = MemacAddHashMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_RemoveHashMacAddr         = MemacDelHashMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_AddExactMatchMacAddr      = MemacAddExactMatchMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_RemovelExactMatchMacAddr  = MemacDelExactMatchMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_GetId                     = NULL;
    p_FmMacControllerDriver->f_FM_MAC_GetVersion                = NULL;
    p_FmMacControllerDriver->f_FM_MAC_GetMaxFrameLength         = MemacGetMaxFrameLength;

    p_FmMacControllerDriver->f_FM_MAC_MII_WritePhyReg           = MEMAC_MII_WritePhyReg;
    p_FmMacControllerDriver->f_FM_MAC_MII_ReadPhyReg            = MEMAC_MII_ReadPhyReg;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    p_FmMacControllerDriver->f_FM_MAC_DumpRegs                  = MemacDumpRegs;
#endif /* (defined(DEBUG_ERRORS) && ... */
}


/*****************************************************************************/
/*                      Memac Config  Main Entry                             */
/*****************************************************************************/

/* .............................................................................. */

t_Handle MEMAC_Config(t_FmMacParams *p_FmMacParam)
{
    t_Memac                 *p_Memac;
    t_MemacDriverParam      *p_MemacDriverParam;
    uintptr_t               baseAddr;
    uint8_t                 i;

    SANITY_CHECK_RETURN_VALUE(p_FmMacParam, E_NULL_POINTER, NULL);

    baseAddr = p_FmMacParam->baseAddr;

    /* Allocate memory for the mEMAC data structure */
    p_Memac = (t_Memac *) XX_Malloc(sizeof(t_Memac));
    if (!p_Memac)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("mEMAC driver structure"));
        return NULL;
    }
    /* Zero out *p_Memac */
    memset(p_Memac, 0, sizeof(t_Memac));
    InitFmMacControllerDriver(&p_Memac->fmMacControllerDriver);

    /* Allocate memory for the mEMAC driver parameters data structure */
    p_MemacDriverParam = (t_MemacDriverParam *) XX_Malloc(sizeof(t_MemacDriverParam));
    if (!p_MemacDriverParam)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("mEMAC driver parameters"));
        MemacFree(p_Memac);
        return NULL;
    }
    /* Zero out */
    memset(p_MemacDriverParam, 0, sizeof(t_MemacDriverParam));

    /* Plant parameter structure pointer */
    p_Memac->p_MemacDriverParam = p_MemacDriverParam;

    SetDefaultParam(p_MemacDriverParam);

    for (i=0; i < sizeof(p_FmMacParam->addr); i++)
        p_Memac->addr |= ((uint64_t)p_FmMacParam->addr[i] << ((5-i) * 8));

    p_Memac->p_MemMap        = (t_MemacMemMap *)UINT_TO_PTR(baseAddr);
    p_Memac->p_MiiMemMap     = (t_MemacMiiAccessMemMap *)UINT_TO_PTR(baseAddr + MEMAC_TO_MII_OFFSET);
    p_Memac->enetMode        = p_FmMacParam->enetMode;
    p_Memac->macId           = p_FmMacParam->macId;
    p_Memac->exceptions      = DEFAULT_exceptions;
    p_Memac->f_Exception     = p_FmMacParam->f_Exception;
    p_Memac->f_Event         = p_FmMacParam->f_Event;
    p_Memac->h_App           = p_FmMacParam->h_App;

    return p_Memac;
}
