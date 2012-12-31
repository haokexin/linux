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
 @File          tgec.c

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
#include "tgec.h"


/*****************************************************************************/
/*                      Internal routines                                    */
/*****************************************************************************/

static t_Error CheckInitParameters(t_Tgec    *p_Tgec)
{
    if (ENET_SPEED_FROM_MODE(p_Tgec->enetMode) < e_ENET_SPEED_10000)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Ethernet 10G MAC driver only support 10G speed"));
#if (FM_MAX_NUM_OF_10G_MACS > 0)
    if (p_Tgec->macId >= FM_MAX_NUM_OF_10G_MACS)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("macId of 10G can not be greater than 0"));
#endif /* (FM_MAX_NUM_OF_10G_MACS > 0) */

    if (p_Tgec->addr == 0)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Ethernet 10G MAC Must have a valid MAC Address"));
    if (!p_Tgec->f_Exception)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("uninitialized f_Exception"));
    if (!p_Tgec->f_Event)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("uninitialized f_Event"));
#ifdef FM_LEN_CHECK_ERRATA_FMAN_SW002
    if (!p_Tgec->p_TgecDriverParam->noLengthCheckEnable)
       RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("LengthCheck!"));
#endif /* FM_LEN_CHECK_ERRATA_FMAN_SW002 */

    return E_OK;
}

/* .............................................................................. */

static void SetDefaultParam(t_TgecDriverParam *p_TgecDriverParam)
{
    p_TgecDriverParam->wanModeEnable            = DEFAULT_wanModeEnable;
    p_TgecDriverParam->promiscuousModeEnable    = DEFAULT_promiscuousEnable;
    p_TgecDriverParam->pauseForwardEnable       = DEFAULT_pauseForwardEnable;
    p_TgecDriverParam->pauseIgnore              = DEFAULT_rxIgnorePause;
    p_TgecDriverParam->txAddrInsEnable          = DEFAULT_txAddrInsEnable;

    p_TgecDriverParam->loopbackEnable           = DEFAULT_loopback;
    p_TgecDriverParam->cmdFrameEnable           = DEFAULT_cmdFrameEnable;
    p_TgecDriverParam->rxErrorDiscard           = DEFAULT_rxErrorDiscard;
    p_TgecDriverParam->phyTxenaOn               = DEFAULT_phyTxenaOn;
    p_TgecDriverParam->sendIdleEnable           = DEFAULT_sendIdleEnable;

    p_TgecDriverParam->noLengthCheckEnable      = !DEFAULT_lengthCheckEnable;

    p_TgecDriverParam->lgthCheckNostdr          = DEFAULT_lgthCheckNostdr;
    p_TgecDriverParam->timeStampEnable          = DEFAULT_timeStampEnable;
    p_TgecDriverParam->rxSfdAny                 = DEFAULT_rxSfdAny;
    p_TgecDriverParam->rxPblFwd                 = DEFAULT_rxPblFwd;
    p_TgecDriverParam->txPblFwd                 = DEFAULT_txPblFwd;

    p_TgecDriverParam->txIpgLength              = DEFAULT_txIpgLength;
    p_TgecDriverParam->maxFrameLength           = DEFAULT_maxFrameLength;

    p_TgecDriverParam->debugMode                = DEFAULT_debugMode;

    p_TgecDriverParam->pauseTime                = DEFAULT_pauseTime;

#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
    p_TgecDriverParam->skipFman11Workaround     = DEFAULT_skipFman11Workaround;
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */
}

/* ........................................................................... */

static void TgecErrException(t_Handle h_Tgec)
{
    t_Tgec             *p_Tgec = (t_Tgec *)h_Tgec;
    uint32_t            event;
    t_TgecMemMap        *p_TgecMemMap = p_Tgec->p_MemMap;

    event = GET_UINT32(p_TgecMemMap->ievent);
    /* do not handle MDIO events */
    event &= ~(IMASK_MDIO_SCAN_EVENTMDIO | IMASK_MDIO_CMD_CMPL);

    event &= GET_UINT32(p_TgecMemMap->imask);

    WRITE_UINT32(p_TgecMemMap->ievent, event);

    if (event & IMASK_REM_FAULT)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_REM_FAULT);
    if (event & IMASK_LOC_FAULT)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_LOC_FAULT);
    if (event & IMASK_1TX_ECC_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_1TX_ECC_ER);
    if (event & IMASK_TX_FIFO_UNFL)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_TX_FIFO_UNFL);
    if (event & IMASK_TX_FIFO_OVFL)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_TX_FIFO_OVFL);
    if (event & IMASK_TX_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_TX_ER);
    if (event & IMASK_RX_FIFO_OVFL)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_FIFO_OVFL);
    if (event & IMASK_RX_ECC_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_ECC_ER);
    if (event & IMASK_RX_JAB_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_JAB_FRM);
    if (event & IMASK_RX_OVRSZ_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_OVRSZ_FRM);
    if (event & IMASK_RX_RUNT_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_RUNT_FRM);
    if (event & IMASK_RX_FRAG_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_FRAG_FRM);
    if (event & IMASK_RX_LEN_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_LEN_ER);
    if (event & IMASK_RX_CRC_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_CRC_ER);
    if (event & IMASK_RX_ALIGN_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_ALIGN_ER);
}

static void TgecException(t_Handle h_Tgec)
{
     t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
     uint32_t            event;
     t_TgecMemMap        *p_TgecMemMap = p_Tgec->p_MemMap;

     event = GET_UINT32(p_TgecMemMap->ievent);
     /* handle only MDIO events */
     event &= (IMASK_MDIO_SCAN_EVENTMDIO | IMASK_MDIO_CMD_CMPL);
     event &= GET_UINT32(p_TgecMemMap->imask);

     WRITE_UINT32(p_TgecMemMap->ievent, event);

     if(event & IMASK_MDIO_SCAN_EVENTMDIO)
         p_Tgec->f_Event(p_Tgec->h_App, e_FM_MAC_EX_10G_MDIO_SCAN_EVENTMDIO);
     if(event & IMASK_MDIO_CMD_CMPL)
         p_Tgec->f_Event(p_Tgec->h_App, e_FM_MAC_EX_10G_MDIO_CMD_CMPL);
}

static void FreeInitResources(t_Tgec *p_Tgec)
{
    if ((p_Tgec->mdioIrq != 0) && (p_Tgec->mdioIrq != NO_IRQ))
    {
        XX_DisableIntr(p_Tgec->mdioIrq);
        XX_FreeIntr(p_Tgec->mdioIrq);
    }
    else if (p_Tgec->mdioIrq == 0)
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED, (NO_MSG));
    FmUnregisterIntr(p_Tgec->fmMacControllerDriver.h_Fm, e_FM_MOD_10G_MAC, p_Tgec->macId, e_FM_INTR_TYPE_ERR);

    /* release the driver's group hash table */
    FreeHashTable(p_Tgec->p_MulticastAddrHash);
    p_Tgec->p_MulticastAddrHash =   NULL;

    /* release the driver's individual hash table */
    FreeHashTable(p_Tgec->p_UnicastAddrHash);
    p_Tgec->p_UnicastAddrHash =     NULL;
}

/* .............................................................................. */

static void HardwareClearAddrInPaddr(t_Tgec   *p_Tgec, uint8_t paddrNum)
{
    if (paddrNum != 0)
        return;             /* At this time MAC has only one address */

    WRITE_UINT32(p_Tgec->p_MemMap->mac_addr_2, 0x0);
    WRITE_UINT32(p_Tgec->p_MemMap->mac_addr_3, 0x0);
}

/* ........................................................................... */

static void HardwareAddAddrInPaddr(t_Tgec   *p_Tgec, uint64_t *p_Addr, uint8_t paddrNum)
{
    uint32_t        tmpReg32 = 0;
    uint64_t        addr = *p_Addr;
    t_TgecMemMap    *p_TgecMemMap = p_Tgec->p_MemMap;

    if (paddrNum != 0)
        return;             /* At this time MAC has only one address */

    tmpReg32 = (uint32_t)(addr>>16);
    SwapUint32P(&tmpReg32);
    WRITE_UINT32(p_TgecMemMap->mac_addr_2, tmpReg32);

    tmpReg32 = (uint32_t)(addr);
    SwapUint32P(&tmpReg32);
    tmpReg32 >>= 16;
    WRITE_UINT32(p_TgecMemMap->mac_addr_3, tmpReg32);
}

/*****************************************************************************/
/*                     10G MAC API routines                                  */
/*****************************************************************************/

/* .............................................................................. */

static t_Error TgecEnable(t_Handle h_Tgec,  e_CommMode mode)
{
    t_Tgec *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap       *p_MemMap ;
    uint32_t            tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_HANDLE);

    p_MemMap= (t_TgecMemMap*)(p_Tgec->p_MemMap);

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

static t_Error TgecDisable (t_Handle h_Tgec, e_CommMode mode)
{
    t_Tgec *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap       *p_MemMap ;
    uint32_t            tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_HANDLE);

    p_MemMap= (t_TgecMemMap*)(p_Tgec->p_MemMap);

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

static t_Error TgecSetPromiscuous(t_Handle h_Tgec, bool newVal)
{
    t_Tgec       *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap *p_TgecMemMap;
    uint32_t     tmpReg32;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;

    tmpReg32 = GET_UINT32(p_TgecMemMap->command_config);

    if (newVal)
        tmpReg32 |= CMD_CFG_PROMIS_EN;
    else
        tmpReg32 &= ~CMD_CFG_PROMIS_EN;

    WRITE_UINT32(p_TgecMemMap->command_config, tmpReg32);

    return E_OK;
}


/*****************************************************************************/
/*                      Tgec Configs modification functions                 */
/*****************************************************************************/

/* .............................................................................. */

static t_Error TgecConfigLoopback(t_Handle h_Tgec, bool newVal)
{
    t_Tgec *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->loopbackEnable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigWan(t_Handle h_Tgec, bool newVal)
{
    t_Tgec *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->wanModeEnable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigMaxFrameLength(t_Handle h_Tgec, uint16_t newVal)
{
    t_Tgec *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->maxFrameLength = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigLengthCheck(t_Handle h_Tgec, bool newVal)
{
    t_Tgec *p_Tgec = (t_Tgec *)h_Tgec;

    UNUSED(newVal);

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->noLengthCheckEnable = !newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigException(t_Handle h_Tgec, e_FmMacExceptions exception, bool enable)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;
    uint32_t    bitMask = 0;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    GET_EXCEPTION_FLAG(bitMask, exception);
    if(bitMask)
    {
        if (enable)
            p_Tgec->exceptions |= bitMask;
        else
            p_Tgec->exceptions &= ~bitMask;
    }
    else
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Undefined exception"));

    return E_OK;
}

#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
/* .............................................................................. */

static t_Error TgecConfigSkipFman11Workaround(t_Handle h_Tgec)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->skipFman11Workaround     = TRUE;

    return E_OK;
}
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */


/*****************************************************************************/
/*                      Tgec Run Time API functions                         */
/*****************************************************************************/

/* .............................................................................. */

static t_Error TgecSetTxPauseFrames(t_Handle h_Tgec,
                                    uint8_t  priority,
                                    uint16_t pauseTime,
                                    uint16_t threshTime)
{
    t_Tgec          *p_Tgec = (t_Tgec *)h_Tgec;
    uint32_t        ptv = 0;
    t_TgecMemMap    *p_MemMap;

UNUSED(priority);UNUSED(threshTime);

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_STATE);

    p_MemMap = (t_TgecMemMap*)(p_Tgec->p_MemMap);

    ptv = (uint32_t)pauseTime;

    WRITE_UINT32(p_MemMap->pause_quant, ptv);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecSetTxAutoPauseFrames(t_Handle h_Tgec,
                                        uint16_t pauseTime)
{
    return TgecSetTxPauseFrames(h_Tgec, 0, pauseTime, 0);
}

/* .............................................................................. */

static t_Error TgecRxIgnoreMacPause(t_Handle h_Tgec, bool en)
{
    t_Tgec          *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap    *p_MemMap;
    uint32_t        tmpReg32;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_STATE);

    p_MemMap = (t_TgecMemMap*)(p_Tgec->p_MemMap);
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

static t_Error TgecGetStatistics(t_Handle h_Tgec, t_FmMacStatistics *p_Statistics)
{
    t_Tgec          *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap    *p_TgecMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Statistics, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;

    p_Statistics->eStatPkts64           = (((uint64_t)GET_UINT32(p_TgecMemMap->r64_u)<<32)|GET_UINT32(p_TgecMemMap->r64_l));
    p_Statistics->eStatPkts65to127      = (((uint64_t)GET_UINT32(p_TgecMemMap->r127_u)<<32)|GET_UINT32(p_TgecMemMap->r127_l));
    p_Statistics->eStatPkts128to255     = (((uint64_t)GET_UINT32(p_TgecMemMap->r255_u)<<32)|GET_UINT32(p_TgecMemMap->r255_l));
    p_Statistics->eStatPkts256to511     = (((uint64_t)GET_UINT32(p_TgecMemMap->r511_u)<<32)|GET_UINT32(p_TgecMemMap->r511_l));
    p_Statistics->eStatPkts512to1023    = (((uint64_t)GET_UINT32(p_TgecMemMap->r1023_u)<<32)|GET_UINT32(p_TgecMemMap->r1023_l));
    p_Statistics->eStatPkts1024to1518   = (((uint64_t)GET_UINT32(p_TgecMemMap->r1518_u)<<32)|GET_UINT32(p_TgecMemMap->r1518_l));
    p_Statistics->eStatPkts1519to1522   = (((uint64_t)GET_UINT32(p_TgecMemMap->r1519x_u)<<32)|GET_UINT32(p_TgecMemMap->r1519x_l));
/* */
    p_Statistics->eStatFragments        = (((uint64_t)GET_UINT32(p_TgecMemMap->trfrg_u)<<32)|GET_UINT32(p_TgecMemMap->trfrg_l));
    p_Statistics->eStatJabbers          = (((uint64_t)GET_UINT32(p_TgecMemMap->trjbr_u)<<32)|GET_UINT32(p_TgecMemMap->trjbr_l));

    p_Statistics->eStatsDropEvents      = (((uint64_t)GET_UINT32(p_TgecMemMap->rdrp_u)<<32)|GET_UINT32(p_TgecMemMap->rdrp_l));
    p_Statistics->eStatCRCAlignErrors   = (((uint64_t)GET_UINT32(p_TgecMemMap->raln_u)<<32)|GET_UINT32(p_TgecMemMap->raln_l));

    p_Statistics->eStatUndersizePkts    = (((uint64_t)GET_UINT32(p_TgecMemMap->trund_u)<<32)|GET_UINT32(p_TgecMemMap->trund_l));
    p_Statistics->eStatOversizePkts     = (((uint64_t)GET_UINT32(p_TgecMemMap->trovr_u)<<32)|GET_UINT32(p_TgecMemMap->trovr_l));
/* Pause */
    p_Statistics->reStatPause           = (((uint64_t)GET_UINT32(p_TgecMemMap->rxpf_u)<<32)|GET_UINT32(p_TgecMemMap->rxpf_l));
    p_Statistics->teStatPause           = (((uint64_t)GET_UINT32(p_TgecMemMap->txpf_u)<<32)|GET_UINT32(p_TgecMemMap->txpf_l));

/* MIB II */
    p_Statistics->ifInOctets            = (((uint64_t)GET_UINT32(p_TgecMemMap->roct_u)<<32)|GET_UINT32(p_TgecMemMap->roct_l));
    p_Statistics->ifInMcastPkts         = (((uint64_t)GET_UINT32(p_TgecMemMap->rmca_u)<<32)|GET_UINT32(p_TgecMemMap->rmca_l));
    p_Statistics->ifInBcastPkts         = (((uint64_t)GET_UINT32(p_TgecMemMap->rbca_u)<<32)|GET_UINT32(p_TgecMemMap->rbca_l));
    p_Statistics->ifInPkts              = (((uint64_t)GET_UINT32(p_TgecMemMap->ruca_u)<<32)|GET_UINT32(p_TgecMemMap->ruca_l))
                                        + p_Statistics->ifInMcastPkts
                                        + p_Statistics->ifInBcastPkts;
    p_Statistics->ifInDiscards          = 0;
    p_Statistics->ifInErrors            = (((uint64_t)GET_UINT32(p_TgecMemMap->rerr_u)<<32)|GET_UINT32(p_TgecMemMap->rerr_l));

    p_Statistics->ifOutOctets           = (((uint64_t)GET_UINT32(p_TgecMemMap->toct_u)<<32)|GET_UINT32(p_TgecMemMap->toct_l));
    p_Statistics->ifOutMcastPkts        = (((uint64_t)GET_UINT32(p_TgecMemMap->tmca_u)<<32)|GET_UINT32(p_TgecMemMap->tmca_l));
    p_Statistics->ifOutBcastPkts        = (((uint64_t)GET_UINT32(p_TgecMemMap->tbca_u)<<32)|GET_UINT32(p_TgecMemMap->tbca_l));
    p_Statistics->ifOutPkts             = (((uint64_t)GET_UINT32(p_TgecMemMap->tuca_u)<<32)|GET_UINT32(p_TgecMemMap->tuca_l))
                                            + p_Statistics->ifOutMcastPkts
                                            + p_Statistics->ifOutBcastPkts;
    p_Statistics->ifOutDiscards         = 0;
    p_Statistics->ifOutErrors           = (((uint64_t)GET_UINT32(p_TgecMemMap->terr_u)<<32)|GET_UINT32(p_TgecMemMap->terr_l));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecEnable1588TimeStamp(t_Handle h_Tgec)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap        *p_TgecMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_TgecMemMap = p_Tgec->p_MemMap;
    SANITY_CHECK_RETURN_ERROR(p_TgecMemMap, E_INVALID_HANDLE);

    WRITE_UINT32(p_TgecMemMap->command_config, GET_UINT32(p_TgecMemMap->command_config) | CMD_CFG_EN_TIMESTAMP);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecDisable1588TimeStamp(t_Handle h_Tgec)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap        *p_TgecMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_TgecMemMap = p_Tgec->p_MemMap;
    SANITY_CHECK_RETURN_ERROR(p_TgecMemMap, E_INVALID_HANDLE);

    WRITE_UINT32(p_TgecMemMap->command_config, GET_UINT32(p_TgecMemMap->command_config) & ~CMD_CFG_EN_TIMESTAMP);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecModifyMacAddress (t_Handle h_Tgec, t_EnetAddr *p_EnetAddr)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap        *p_TgecMemMap;
    uint32_t            tmpReg32 = 0;
    uint64_t            addr;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;

    /*  Initialize MAC Station Address registers (1 & 2)    */
    /*  Station address have to be swapped (big endian to little endian */

    addr = ((*(uint64_t *)p_EnetAddr) >> 16);
    p_Tgec->addr = addr;

    tmpReg32 = (uint32_t)(addr>>16);
    SwapUint32P(&tmpReg32);
    WRITE_UINT32(p_TgecMemMap->mac_addr_0, tmpReg32);

    tmpReg32 = (uint32_t)(addr);
    SwapUint32P(&tmpReg32);
    tmpReg32 >>= 16;
    WRITE_UINT32(p_TgecMemMap->mac_addr_1, tmpReg32);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecResetCounters (t_Handle h_Tgec)
{
    t_Tgec          *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap    *p_MemMap;
    uint32_t        tmpReg32;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_HANDLE);

    p_MemMap = (t_TgecMemMap*)(p_Tgec->p_MemMap);

    tmpReg32 = GET_UINT32(p_MemMap->command_config);

    tmpReg32 |= CMD_CFG_STAT_CLR;

    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    while (GET_UINT32(p_MemMap->command_config) & CMD_CFG_STAT_CLR) ;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecAddExactMatchMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec   *p_Tgec = (t_Tgec *) h_Tgec;
    uint64_t  ethAddr;
    uint8_t   paddrNum;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);

    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    if (ethAddr & GROUP_ADDRESS)
        /* Multicast address has no effect in PADDR */
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Multicast address"));

    /* Make sure no PADDR contains this address */
    for (paddrNum = 0; paddrNum < TGEC_NUM_OF_PADDRS; paddrNum++)
        if (p_Tgec->indAddrRegUsed[paddrNum])
            if (p_Tgec->paddr[paddrNum] == ethAddr)
                RETURN_ERROR(MAJOR, E_ALREADY_EXISTS, NO_MSG);

    /* Find first unused PADDR */
    for (paddrNum = 0; paddrNum < TGEC_NUM_OF_PADDRS; paddrNum++)
        if (!(p_Tgec->indAddrRegUsed[paddrNum]))
        {
            /* mark this PADDR as used */
            p_Tgec->indAddrRegUsed[paddrNum] = TRUE;
            /* store address */
            p_Tgec->paddr[paddrNum] = ethAddr;

            /* put in hardware */
            HardwareAddAddrInPaddr(p_Tgec, &ethAddr, paddrNum);
            p_Tgec->numOfIndAddrInRegs++;

            return E_OK;
        }

    /* No free PADDR */
    RETURN_ERROR(MAJOR, E_FULL, NO_MSG);
}

/* .............................................................................. */

static t_Error TgecDelExactMatchMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec   *p_Tgec = (t_Tgec *) h_Tgec;
    uint64_t  ethAddr;
    uint8_t   paddrNum;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_HANDLE);

    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    /* Find used PADDR containing this address */
    for (paddrNum = 0; paddrNum < TGEC_NUM_OF_PADDRS; paddrNum++)
    {
        if ((p_Tgec->indAddrRegUsed[paddrNum]) &&
            (p_Tgec->paddr[paddrNum] == ethAddr))
        {
            /* mark this PADDR as not used */
            p_Tgec->indAddrRegUsed[paddrNum] = FALSE;
            /* clear in hardware */
            HardwareClearAddrInPaddr(p_Tgec, paddrNum);
            p_Tgec->numOfIndAddrInRegs--;

            return E_OK;
        }
    }

    RETURN_ERROR(MAJOR, E_NOT_FOUND, NO_MSG);
}

/* .............................................................................. */

static t_Error TgecAddHashMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec          *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap    *p_TgecMemMap;
    t_EthHashEntry  *p_HashEntry;
    uint32_t        crc;
    uint32_t        hash;
    uint64_t        ethAddr;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;
    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    if (!(ethAddr & GROUP_ADDRESS))
        /* Unicast addresses not supported in hash */
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unicast Address"));

    /* CRC calculation */
    GET_MAC_ADDR_CRC(ethAddr, crc);
    crc = MIRROR_32(crc);

    hash = (crc >> HASH_CTRL_MCAST_SHIFT) & HASH_ADDR_MASK;        /* Take 9 MSB bits */

    /* Create element to be added to the driver hash table */
    p_HashEntry = (t_EthHashEntry *)XX_Malloc(sizeof(t_EthHashEntry));
    p_HashEntry->addr = ethAddr;
    INIT_LIST(&p_HashEntry->node);

    LIST_AddToTail(&(p_HashEntry->node), &(p_Tgec->p_MulticastAddrHash->p_Lsts[hash]));
    WRITE_UINT32(p_TgecMemMap->hashtable_ctrl, (hash | HASH_CTRL_MCAST_EN));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecDelHashMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec          *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap    *p_TgecMemMap;
    t_EthHashEntry  *p_HashEntry = NULL;
    t_List          *p_Pos;
    uint32_t        crc;
    uint32_t        hash;
    uint64_t        ethAddr;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;
    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    /* CRC calculation */
    GET_MAC_ADDR_CRC(ethAddr, crc);
    crc = MIRROR_32(crc);

    hash = (crc >> HASH_CTRL_MCAST_SHIFT) & HASH_ADDR_MASK;        /* Take 9 MSB bits */

    LIST_FOR_EACH(p_Pos, &(p_Tgec->p_MulticastAddrHash->p_Lsts[hash]))
    {

        p_HashEntry = ETH_HASH_ENTRY_OBJ(p_Pos);
        if(p_HashEntry->addr == ethAddr)
        {
            LIST_DelAndInit(&p_HashEntry->node);
            XX_Free(p_HashEntry);
            break;
        }
    }
    if(LIST_IsEmpty(&p_Tgec->p_MulticastAddrHash->p_Lsts[hash]))
        WRITE_UINT32(p_TgecMemMap->hashtable_ctrl, (hash & ~HASH_CTRL_MCAST_EN));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecGetId(t_Handle h_Tgec, uint32_t *macId)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_NULL_POINTER);

    UNUSED(p_Tgec);
    UNUSED(macId);
    RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("TgecGetId Not Supported"));
}

/* .............................................................................. */

static t_Error TgecGetVersion(t_Handle h_Tgec, uint32_t *macVersion)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecMemMap        *p_TgecMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;
    *macVersion = GET_UINT32(p_TgecMemMap->tgec_id);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecSetExcpetion(t_Handle h_Tgec, e_FmMacExceptions exception, bool enable)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    uint32_t            bitMask = 0, tmpReg;
    t_TgecMemMap        *p_TgecMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;

    GET_EXCEPTION_FLAG(bitMask, exception);
    if(bitMask)
    {
        if (enable)
            p_Tgec->exceptions |= bitMask;
        else
            p_Tgec->exceptions &= ~bitMask;
   }
    else
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Undefined exception"));

    tmpReg = GET_UINT32(p_TgecMemMap->imask);
    if(enable)
        tmpReg |= bitMask;
    else
        tmpReg &= ~bitMask;
    WRITE_UINT32(p_TgecMemMap->imask, tmpReg);
    return E_OK;
}

/* .............................................................................. */

static uint16_t TgecGetMaxFrameLength(t_Handle h_Tgec)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_VALUE(p_Tgec, E_INVALID_HANDLE, 0);

    return (uint16_t)GET_UINT32(p_Tgec->p_MemMap->maxfrm);
}

/* .............................................................................. */

#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
static t_Error TgecTxEccWorkaround(t_Tgec *p_Tgec)
{
    t_Error err;

#if defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0)
    XX_Print("Applying 10G TX ECC workaround (10GMAC-A004) ...");
#endif /* (DEBUG_ERRORS > 0) */
    /* enable and set promiscuous */
    WRITE_UINT32(p_Tgec->p_MemMap->command_config, CMD_CFG_PROMIS_EN | CMD_CFG_TX_EN | CMD_CFG_RX_EN);
    err = Fm10GTxEccWorkaround(p_Tgec->fmMacControllerDriver.h_Fm, p_Tgec->macId);
    /* disable */
    WRITE_UINT32(p_Tgec->p_MemMap->command_config, 0);
#if defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0)
    if (err)
        XX_Print("FAILED!\n");
    else
        XX_Print("done.\n");
#endif /* (DEBUG_ERRORS > 0) */
    TgecResetCounters (p_Tgec);

    return err;
}
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */

/* .............................................................................. */

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
static t_Error TgecDumpRegs(t_Handle h_Tgec)
{
    t_Tgec    *p_Tgec = (t_Tgec *)h_Tgec;

    DECLARE_DUMP;

    if (p_Tgec->p_MemMap)
    {
        DUMP_TITLE(p_Tgec->p_MemMap, ("10G MAC %d: ", p_Tgec->macId));
        DUMP_VAR(p_Tgec->p_MemMap, tgec_id);
        DUMP_VAR(p_Tgec->p_MemMap, scratch);
        DUMP_VAR(p_Tgec->p_MemMap, command_config);
        DUMP_VAR(p_Tgec->p_MemMap, mac_addr_0);
        DUMP_VAR(p_Tgec->p_MemMap, mac_addr_1);
        DUMP_VAR(p_Tgec->p_MemMap, maxfrm);
        DUMP_VAR(p_Tgec->p_MemMap, pause_quant);
        DUMP_VAR(p_Tgec->p_MemMap, rx_fifo_sections);
        DUMP_VAR(p_Tgec->p_MemMap, tx_fifo_sections);
        DUMP_VAR(p_Tgec->p_MemMap, rx_fifo_almost_f_e);
        DUMP_VAR(p_Tgec->p_MemMap, tx_fifo_almost_f_e);
        DUMP_VAR(p_Tgec->p_MemMap, hashtable_ctrl);
        DUMP_VAR(p_Tgec->p_MemMap, mdio_cfg_status);
        DUMP_VAR(p_Tgec->p_MemMap, mdio_command);
        DUMP_VAR(p_Tgec->p_MemMap, mdio_data);
        DUMP_VAR(p_Tgec->p_MemMap, mdio_regaddr);
        DUMP_VAR(p_Tgec->p_MemMap, status);
        DUMP_VAR(p_Tgec->p_MemMap, tx_ipg_len);
        DUMP_VAR(p_Tgec->p_MemMap, mac_addr_2);
        DUMP_VAR(p_Tgec->p_MemMap, mac_addr_3);
        DUMP_VAR(p_Tgec->p_MemMap, rx_fifo_ptr_rd);
        DUMP_VAR(p_Tgec->p_MemMap, rx_fifo_ptr_wr);
        DUMP_VAR(p_Tgec->p_MemMap, tx_fifo_ptr_rd);
        DUMP_VAR(p_Tgec->p_MemMap, tx_fifo_ptr_wr);
        DUMP_VAR(p_Tgec->p_MemMap, imask);
        DUMP_VAR(p_Tgec->p_MemMap, ievent);
        DUMP_VAR(p_Tgec->p_MemMap, udp_port);
        DUMP_VAR(p_Tgec->p_MemMap, type_1588v2);
    }

    return E_OK;
}
#endif /* (defined(DEBUG_ERRORS) && ... */


/*****************************************************************************/
/*                      FM Init & Free API                                   */
/*****************************************************************************/

/* .............................................................................. */

static t_Error TgecInit(t_Handle h_Tgec)
{
    t_Tgec                  *p_Tgec = (t_Tgec *)h_Tgec;
    t_TgecDriverParam       *p_TgecDriverParam;
    t_TgecMemMap            *p_MemMap;
    uint64_t                addr;
    uint32_t                tmpReg32;
    t_Error                 err;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_MemMap, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->fmMacControllerDriver.h_Fm, E_INVALID_HANDLE);

    FM_GetRevision(p_Tgec->fmMacControllerDriver.h_Fm, &p_Tgec->fmMacControllerDriver.fmRevInfo);

    CHECK_INIT_PARAMETERS(p_Tgec, CheckInitParameters);

#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
    if (p_Tgec->fmMacControllerDriver.fmRevInfo.majorRev <= 6 /*fixed for rev3 */)
    {
        if (!p_Tgec->p_TgecDriverParam->skipFman11Workaround &&
            ((err = TgecTxEccWorkaround(p_Tgec)) != E_OK))
        {
            FreeInitResources(p_Tgec);
            RETURN_ERROR(MAJOR, err, ("TgecTxEccWorkaround FAILED"));
        }
    }
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */

    p_TgecDriverParam = p_Tgec->p_TgecDriverParam;
    p_MemMap = p_Tgec->p_MemMap;

    /* MAC Address */
    addr = p_Tgec->addr;
    tmpReg32 = (uint32_t)(addr>>16);
    SwapUint32P(&tmpReg32);
    WRITE_UINT32(p_MemMap->mac_addr_0, tmpReg32);

    tmpReg32 = (uint32_t)(addr);
    SwapUint32P(&tmpReg32);
    tmpReg32 >>= 16;
    WRITE_UINT32(p_MemMap->mac_addr_1, tmpReg32);

    /* Config */
    tmpReg32 = 0;
    if (p_TgecDriverParam->wanModeEnable)
        tmpReg32 |= CMD_CFG_WAN_MODE;
    if (p_TgecDriverParam->promiscuousModeEnable)
        tmpReg32 |= CMD_CFG_PROMIS_EN;
    if (p_TgecDriverParam->pauseForwardEnable)
        tmpReg32 |= CMD_CFG_PAUSE_FWD;
    if (p_TgecDriverParam->pauseIgnore)
        tmpReg32 |= CMD_CFG_PAUSE_IGNORE;
    if (p_TgecDriverParam->txAddrInsEnable)
        tmpReg32 |= CMD_CFG_TX_ADDR_INS;
    if (p_TgecDriverParam->loopbackEnable)
        tmpReg32 |= CMD_CFG_LOOPBACK_EN;
    if (p_TgecDriverParam->cmdFrameEnable)
        tmpReg32 |= CMD_CFG_CMD_FRM_EN;
    if (p_TgecDriverParam->rxErrorDiscard)
        tmpReg32 |= CMD_CFG_RX_ER_DISC;
    if (p_TgecDriverParam->phyTxenaOn)
        tmpReg32 |= CMD_CFG_PHY_TX_EN;
    if (p_TgecDriverParam->sendIdleEnable)
        tmpReg32 |= CMD_CFG_SEND_IDLE;
    if (p_TgecDriverParam->noLengthCheckEnable)
        tmpReg32 |= CMD_CFG_NO_LEN_CHK;
    if (p_TgecDriverParam->lgthCheckNostdr)
        tmpReg32 |= CMD_CFG_LEN_CHK_NOSTDR;
    if (p_TgecDriverParam->timeStampEnable)
        tmpReg32 |= CMD_CFG_EN_TIMESTAMP;
    if (p_TgecDriverParam->rxSfdAny)
        tmpReg32 |= RX_SFD_ANY;
    if (p_TgecDriverParam->rxPblFwd)
        tmpReg32 |= CMD_CFG_RX_PBL_FWD;
    if (p_TgecDriverParam->txPblFwd)
        tmpReg32 |= CMD_CFG_TX_PBL_FWD;
    tmpReg32 |= 0x40;
    WRITE_UINT32(p_MemMap->command_config, tmpReg32);

    /* Max Frame Length */
    WRITE_UINT32(p_MemMap->maxfrm, (uint32_t)p_TgecDriverParam->maxFrameLength);
    err = FmSetMacMaxFrame(p_Tgec->fmMacControllerDriver.h_Fm,
                           e_FM_MAC_10G,
                           p_Tgec->fmMacControllerDriver.macId,
                           p_TgecDriverParam->maxFrameLength);

    /* Pause Time */
    WRITE_UINT32(p_MemMap->pause_quant, p_TgecDriverParam->pauseTime);

#ifdef FM_TX_FIFO_CORRUPTION_ERRATA_10GMAC_A007
    if (p_Tgec->fmMacControllerDriver.fmRevInfo.majorRev == 2)
    {
        WRITE_UINT32(p_Tgec->p_MemMap->tx_ipg_len,
            (GET_UINT32(p_Tgec->p_MemMap->tx_ipg_len) & ~TX_IPG_LENGTH_MASK) | DEFAULT_txIpgLength);
    }
#endif /* FM_TX_FIFO_CORRUPTION_ERRATA_10GMAC_A007 */

    p_Tgec->p_MulticastAddrHash = AllocHashTable(HASH_TABLE_SIZE);
    if(!p_Tgec->p_MulticastAddrHash)
    {
        FreeInitResources(p_Tgec);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("allocation hash table is FAILED"));
    }

    p_Tgec->p_UnicastAddrHash = AllocHashTable(HASH_TABLE_SIZE);
    if(!p_Tgec->p_UnicastAddrHash)
    {
        FreeInitResources(p_Tgec);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("allocation hash table is FAILED"));
    }

    /* interrupts */
#ifdef FM_10G_REM_N_LCL_FLT_EX_10GMAC_ERRATA_SW005
    if (p_Tgec->fmMacControllerDriver.fmRevInfo.majorRev <=2)
        p_Tgec->exceptions &= ~(IMASK_REM_FAULT | IMASK_LOC_FAULT);
#endif /* FM_10G_REM_N_LCL_FLT_EX_10GMAC_ERRATA_SW005 */
    WRITE_UINT32(p_MemMap->ievent, EVENTS_MASK);
    WRITE_UINT32(p_MemMap->imask, p_Tgec->exceptions);

    FmRegisterIntr(p_Tgec->fmMacControllerDriver.h_Fm,
                   e_FM_MOD_10G_MAC,
                   p_Tgec->macId,
                   e_FM_INTR_TYPE_ERR,
                   TgecErrException,
                   p_Tgec);
    if ((p_Tgec->mdioIrq != 0) && (p_Tgec->mdioIrq != NO_IRQ))
    {
        XX_SetIntr(p_Tgec->mdioIrq, TgecException, p_Tgec);
        XX_EnableIntr(p_Tgec->mdioIrq);
    }
    else if (p_Tgec->mdioIrq == 0)
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED, (NO_MSG));

    XX_Free(p_TgecDriverParam);
    p_Tgec->p_TgecDriverParam = NULL;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecFree(t_Handle h_Tgec)
{
    t_Tgec       *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);

    FreeInitResources(p_Tgec);

    if (p_Tgec->p_TgecDriverParam)
    {
        XX_Free(p_Tgec->p_TgecDriverParam);
        p_Tgec->p_TgecDriverParam = NULL;
    }
    XX_Free (p_Tgec);

    return E_OK;
}

/* .............................................................................. */

static void InitFmMacControllerDriver(t_FmMacControllerDriver *p_FmMacControllerDriver)
{
    p_FmMacControllerDriver->f_FM_MAC_Init                      = TgecInit;
    p_FmMacControllerDriver->f_FM_MAC_Free                      = TgecFree;

    p_FmMacControllerDriver->f_FM_MAC_SetStatistics             = NULL;
    p_FmMacControllerDriver->f_FM_MAC_ConfigLoopback            = TgecConfigLoopback;
    p_FmMacControllerDriver->f_FM_MAC_ConfigMaxFrameLength      = TgecConfigMaxFrameLength;

    p_FmMacControllerDriver->f_FM_MAC_ConfigWan                 = TgecConfigWan;

    p_FmMacControllerDriver->f_FM_MAC_ConfigPadAndCrc           = NULL; /* TGEC always works with pad+crc */
    p_FmMacControllerDriver->f_FM_MAC_ConfigHalfDuplex          = NULL; /* half-duplex is not supported in xgec */
    p_FmMacControllerDriver->f_FM_MAC_ConfigLengthCheck         = TgecConfigLengthCheck;
    p_FmMacControllerDriver->f_FM_MAC_ConfigException           = TgecConfigException;
    p_FmMacControllerDriver->f_FM_MAC_ConfigResetOnInit         = NULL;

#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
    p_FmMacControllerDriver->f_FM_MAC_ConfigSkipFman11Workaround= TgecConfigSkipFman11Workaround;
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */

    p_FmMacControllerDriver->f_FM_MAC_SetException              = TgecSetExcpetion;

    p_FmMacControllerDriver->f_FM_MAC_Enable1588TimeStamp       = TgecEnable1588TimeStamp;
    p_FmMacControllerDriver->f_FM_MAC_Disable1588TimeStamp      = TgecDisable1588TimeStamp;

    p_FmMacControllerDriver->f_FM_MAC_SetPromiscuous            = TgecSetPromiscuous;
    p_FmMacControllerDriver->f_FM_MAC_AdjustLink                = NULL;
    p_FmMacControllerDriver->f_FM_MAC_RestartAutoneg            = NULL;

    p_FmMacControllerDriver->f_FM_MAC_Enable                    = TgecEnable;
    p_FmMacControllerDriver->f_FM_MAC_Disable                   = TgecDisable;

    p_FmMacControllerDriver->f_FM_MAC_SetTxAutoPauseFrames      = TgecSetTxAutoPauseFrames;
    p_FmMacControllerDriver->f_FM_MAC_SetTxPauseFrames          = TgecSetTxPauseFrames;
    p_FmMacControllerDriver->f_FM_MAC_SetRxIgnorePauseFrames    = TgecRxIgnoreMacPause;

    p_FmMacControllerDriver->f_FM_MAC_ResetCounters             = TgecResetCounters;
    p_FmMacControllerDriver->f_FM_MAC_GetStatistics             = TgecGetStatistics;

    p_FmMacControllerDriver->f_FM_MAC_ModifyMacAddr             = TgecModifyMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_AddHashMacAddr            = TgecAddHashMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_RemoveHashMacAddr         = TgecDelHashMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_AddExactMatchMacAddr      = TgecAddExactMatchMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_RemovelExactMatchMacAddr  = TgecDelExactMatchMacAddress;
    p_FmMacControllerDriver->f_FM_MAC_GetId                     = TgecGetId;
    p_FmMacControllerDriver->f_FM_MAC_GetVersion                = TgecGetVersion;
    p_FmMacControllerDriver->f_FM_MAC_GetMaxFrameLength         = TgecGetMaxFrameLength;

    p_FmMacControllerDriver->f_FM_MAC_MII_WritePhyReg           = TGEC_MII_WritePhyReg;
    p_FmMacControllerDriver->f_FM_MAC_MII_ReadPhyReg            = TGEC_MII_ReadPhyReg;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
    p_FmMacControllerDriver->f_FM_MAC_DumpRegs                  = TgecDumpRegs;
#endif /* (defined(DEBUG_ERRORS) && ... */
}


/*****************************************************************************/
/*                      Tgec Config  Main Entry                             */
/*****************************************************************************/

/* .............................................................................. */

t_Handle TGEC_Config(t_FmMacParams *p_FmMacParam)
{
    t_Tgec                  *p_Tgec;
    t_TgecDriverParam       *p_TgecDriverParam;
    uintptr_t               baseAddr;
    uint8_t                 i;

    SANITY_CHECK_RETURN_VALUE(p_FmMacParam, E_NULL_POINTER, NULL);

    baseAddr = p_FmMacParam->baseAddr;
    /* allocate memory for the UCC GETH data structure. */
    p_Tgec = (t_Tgec *) XX_Malloc(sizeof(t_Tgec));
    if (!p_Tgec)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("10G MAC driver structure"));
        return NULL;
    }
    /* Zero out * p_Tgec */
    memset(p_Tgec, 0, sizeof(t_Tgec));
    InitFmMacControllerDriver(&p_Tgec->fmMacControllerDriver);

    /* allocate memory for the 10G MAC driver parameters data structure. */
    p_TgecDriverParam = (t_TgecDriverParam *) XX_Malloc(sizeof(t_TgecDriverParam));
    if (!p_TgecDriverParam)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("10G MAC driver parameters"));
        TgecFree(p_Tgec);
        return NULL;
    }
    /* Zero out */
    memset(p_TgecDriverParam, 0, sizeof(t_TgecDriverParam));

    /* Plant parameter structure pointer */
    p_Tgec->p_TgecDriverParam = p_TgecDriverParam;

    SetDefaultParam(p_TgecDriverParam);

    for (i=0; i < sizeof(p_FmMacParam->addr); i++)
        p_Tgec->addr |= ((uint64_t)p_FmMacParam->addr[i] << ((5-i) * 8));

    p_Tgec->p_MemMap        = (t_TgecMemMap *)UINT_TO_PTR(baseAddr);
    p_Tgec->p_MiiMemMap     = (t_TgecMiiAccessMemMap *)UINT_TO_PTR(baseAddr + TGEC_TO_MII_OFFSET);
    p_Tgec->enetMode        = p_FmMacParam->enetMode;
    p_Tgec->macId           = p_FmMacParam->macId;
    p_Tgec->exceptions      = DEFAULT_exceptions;
    p_Tgec->mdioIrq         = p_FmMacParam->mdioIrq;
    p_Tgec->f_Exception     = p_FmMacParam->f_Exception;
    p_Tgec->f_Event         = p_FmMacParam->f_Event;
    p_Tgec->h_App           = p_FmMacParam->h_App;

    return p_Tgec;
}
