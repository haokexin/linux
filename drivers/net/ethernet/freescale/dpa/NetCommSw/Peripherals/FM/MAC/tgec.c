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
#include "debug_ext.h"

#include "fm_common.h"
#include "fsl_fman_tgec.h"
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
    if (!p_Tgec->p_TgecDriverParam->no_length_check_enable)
       RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("LengthCheck!"));
#endif /* FM_LEN_CHECK_ERRATA_FMAN_SW002 */
    return E_OK;
}

/* .............................................................................. */

static void TgecErrException(t_Handle h_Tgec)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    uint32_t            event;
    struct tgec_regs    *p_TgecMemMap = p_Tgec->p_MemMap;

    /* do not handle MDIO events */
    event = tgec_get_event(p_TgecMemMap, ~(TGEC_IMASK_MDIO_SCAN_EVENT | TGEC_IMASK_MDIO_CMD_CMPL));
    event &= tgec_get_interrupt_mask(p_TgecMemMap);

    tgec_ack_event(p_TgecMemMap, event);

    if (event & TGEC_IMASK_REM_FAULT)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_REM_FAULT);
    if (event & TGEC_IMASK_LOC_FAULT)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_LOC_FAULT);
    if (event & TGEC_IMASK_TX_ECC_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_1TX_ECC_ER);
    if (event & TGEC_IMASK_TX_FIFO_UNFL)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_TX_FIFO_UNFL);
    if (event & TGEC_IMASK_TX_FIFO_OVFL)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_TX_FIFO_OVFL);
    if (event & TGEC_IMASK_TX_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_TX_ER);
    if (event & TGEC_IMASK_RX_FIFO_OVFL)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_FIFO_OVFL);
    if (event & TGEC_IMASK_RX_ECC_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_ECC_ER);
    if (event & TGEC_IMASK_RX_JAB_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_JAB_FRM);
    if (event & TGEC_IMASK_RX_OVRSZ_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_OVRSZ_FRM);
    if (event & TGEC_IMASK_RX_RUNT_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_RUNT_FRM);
    if (event & TGEC_IMASK_RX_FRAG_FRM)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_FRAG_FRM);
    if (event & TGEC_IMASK_RX_LEN_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_LEN_ER);
    if (event & TGEC_IMASK_RX_CRC_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_CRC_ER);
    if (event & TGEC_IMASK_RX_ALIGN_ER)
        p_Tgec->f_Exception(p_Tgec->h_App, e_FM_MAC_EX_10G_RX_ALIGN_ER);
}

static void TgecException(t_Handle h_Tgec)
{
     t_Tgec             *p_Tgec = (t_Tgec *)h_Tgec;
     uint32_t           event;
     struct tgec_regs   *p_TgecMemMap = p_Tgec->p_MemMap;

     /* handle only MDIO events */
     event = tgec_get_event(p_TgecMemMap, (TGEC_IMASK_MDIO_SCAN_EVENT | TGEC_IMASK_MDIO_CMD_CMPL));
     event &= tgec_get_interrupt_mask(p_TgecMemMap);

     tgec_ack_event(p_TgecMemMap, event);

     if (event & TGEC_IMASK_MDIO_SCAN_EVENT)
         p_Tgec->f_Event(p_Tgec->h_App, e_FM_MAC_EX_10G_MDIO_SCAN_EVENTMDIO);
     if (event & TGEC_IMASK_MDIO_CMD_CMPL)
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
        REPORT_ERROR(MINOR, E_NOT_SUPPORTED, NO_MSG);
    FmUnregisterIntr(p_Tgec->fmMacControllerDriver.h_Fm, e_FM_MOD_10G_MAC, p_Tgec->macId, e_FM_INTR_TYPE_ERR);

    /* release the driver's group hash table */
    FreeHashTable(p_Tgec->p_MulticastAddrHash);
    p_Tgec->p_MulticastAddrHash =   NULL;

    /* release the driver's individual hash table */
    FreeHashTable(p_Tgec->p_UnicastAddrHash);
    p_Tgec->p_UnicastAddrHash =     NULL;
}

/* ........................................................................... */


/*****************************************************************************/
/*                     10G MAC API routines                                  */
/*****************************************************************************/

/* .............................................................................. */

static t_Error TgecEnable(t_Handle h_Tgec,  e_CommMode mode)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_enable(p_Tgec->p_MemMap, (mode & e_COMM_MODE_RX), (mode & e_COMM_MODE_TX));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecDisable (t_Handle h_Tgec, e_CommMode mode)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_disable(p_Tgec->p_MemMap, (mode & e_COMM_MODE_RX), (mode & e_COMM_MODE_TX));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecSetPromiscuous(t_Handle h_Tgec, bool newVal)
{
    t_Tgec       *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_set_promiscuous(p_Tgec->p_MemMap, newVal);

    return E_OK;
}


/*****************************************************************************/
/*                      Tgec Configs modification functions                 */
/*****************************************************************************/

/* .............................................................................. */

static t_Error TgecConfigLoopback(t_Handle h_Tgec, bool newVal)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->loopback_enable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigWan(t_Handle h_Tgec, bool newVal)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->wan_mode_enable = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigMaxFrameLength(t_Handle h_Tgec, uint16_t newVal)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->max_frame_length = newVal;

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecConfigLengthCheck(t_Handle h_Tgec, bool newVal)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    UNUSED(newVal);

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->p_TgecDriverParam->no_length_check_enable = !newVal;

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
    if (bitMask)
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

    p_Tgec->p_TgecDriverParam->skip_fman11_workaround = TRUE;

    return E_OK;
}
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */


/*****************************************************************************/
/*                      Tgec Run Time API functions                         */
/*****************************************************************************/

/* .............................................................................. */
/* backward compatibility. will be removed in the future. */
static t_Error TgecTxMacPause(t_Handle h_Tgec, uint16_t pauseTime)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_tx_mac_pause(p_Tgec->p_MemMap, pauseTime);

    return E_OK;
}

static t_Error TgecSetTxPauseFrames(t_Handle h_Tgec,
                                    uint8_t  priority,
                                    uint16_t pauseTime,
                                    uint16_t threshTime)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    UNUSED(priority); UNUSED(threshTime);

    tgec_tx_mac_pause(p_Tgec->p_MemMap, pauseTime);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecRxIgnoreMacPause(t_Handle h_Tgec, bool en)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_rx_ignore_mac_pause(p_Tgec->p_MemMap, en);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecGetStatistics(t_Handle h_Tgec, t_FmMacStatistics *p_Statistics)
{
    t_Tgec              *p_Tgec = (t_Tgec *)h_Tgec;
    struct tgec_regs    *p_TgecMemMap;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Statistics, E_NULL_POINTER);

    p_TgecMemMap = p_Tgec->p_MemMap;

    p_Statistics->eStatPkts64           = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R64);
    p_Statistics->eStatPkts65to127      = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R127);
    p_Statistics->eStatPkts128to255     = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R255);
    p_Statistics->eStatPkts256to511     = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R511);
    p_Statistics->eStatPkts512to1023    = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R1023);
    p_Statistics->eStatPkts1024to1518   = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R1518);
    p_Statistics->eStatPkts1519to1522   = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_R1519X);
/* */
    p_Statistics->eStatFragments        = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TRFRG);
    p_Statistics->eStatJabbers          = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TRJBR);

    p_Statistics->eStatsDropEvents      = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RDRP);
    p_Statistics->eStatCRCAlignErrors   = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RALN);

    p_Statistics->eStatUndersizePkts    = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TRUND);
    p_Statistics->eStatOversizePkts     = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TROVR);
/* Pause */
    p_Statistics->reStatPause           = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RXPF);
    p_Statistics->teStatPause           = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TXPF);

/* MIB II */
    p_Statistics->ifInOctets            = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_ROCT);
    p_Statistics->ifInMcastPkts         = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RMCA);
    p_Statistics->ifInBcastPkts         = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RBCA);
    p_Statistics->ifInPkts              = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RUCA)
                                        + p_Statistics->ifInMcastPkts
                                        + p_Statistics->ifInBcastPkts;
    p_Statistics->ifInDiscards          = 0;
    p_Statistics->ifInErrors            = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_RERR);

    p_Statistics->ifOutOctets           = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TOCT);
    p_Statistics->ifOutMcastPkts        = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TMCA);
    p_Statistics->ifOutBcastPkts        = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TBCA);
    p_Statistics->ifOutPkts             = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TUCA)
                                            + p_Statistics->ifOutMcastPkts
                                            + p_Statistics->ifOutBcastPkts;
    p_Statistics->ifOutDiscards         = 0;
    p_Statistics->ifOutErrors           = tgec_get_counter(p_TgecMemMap, E_TGEC_COUNTER_TERR);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecEnable1588TimeStamp(t_Handle h_Tgec)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_enable_1588_time_stamp(p_Tgec->p_MemMap, 1);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecDisable1588TimeStamp(t_Handle h_Tgec)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_enable_1588_time_stamp(p_Tgec->p_MemMap, 0);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecModifyMacAddress (t_Handle h_Tgec, t_EnetAddr *p_EnetAddr)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    p_Tgec->addr = ENET_ADDR_TO_UINT64(*p_EnetAddr);
    tgec_set_mac_address(p_Tgec->p_MemMap, (uint8_t *)(*p_EnetAddr));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecResetCounters (t_Handle h_Tgec)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    tgec_reset_stat(p_Tgec->p_MemMap);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecAddExactMatchMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec      *p_Tgec = (t_Tgec *) h_Tgec;
    uint64_t    ethAddr;
    uint8_t     paddrNum;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    ethAddr = ENET_ADDR_TO_UINT64(*p_EthAddr);

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
    {
        if (!(p_Tgec->indAddrRegUsed[paddrNum]))
        {
            /* mark this PADDR as used */
            p_Tgec->indAddrRegUsed[paddrNum] = TRUE;
            /* store address */
            p_Tgec->paddr[paddrNum] = ethAddr;

            /* put in hardware */
            tgec_add_addr_in_paddr(p_Tgec->p_MemMap, (uint8_t*)(*p_EthAddr)/* , paddrNum */);
            p_Tgec->numOfIndAddrInRegs++;

            return E_OK;
        }
    }

    /* No free PADDR */
    RETURN_ERROR(MAJOR, E_FULL, NO_MSG);
}

/* .............................................................................. */

static t_Error TgecDelExactMatchMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec      *p_Tgec = (t_Tgec *) h_Tgec;
    uint64_t    ethAddr;
    uint8_t     paddrNum;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    ethAddr = ENET_ADDR_TO_UINT64(*p_EthAddr);

    /* Find used PADDR containing this address */
    for (paddrNum = 0; paddrNum < TGEC_NUM_OF_PADDRS; paddrNum++)
    {
        if ((p_Tgec->indAddrRegUsed[paddrNum]) &&
            (p_Tgec->paddr[paddrNum] == ethAddr))
        {
            /* mark this PADDR as not used */
            p_Tgec->indAddrRegUsed[paddrNum] = FALSE;
            /* clear in hardware */
            tgec_clear_addr_in_paddr(p_Tgec->p_MemMap /*, paddrNum */);
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
    t_EthHashEntry  *p_HashEntry;
    uint32_t        crc;
    uint32_t        hash;
    uint64_t        ethAddr;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    ethAddr = ENET_ADDR_TO_UINT64(*p_EthAddr);

    if (!(ethAddr & GROUP_ADDRESS))
        /* Unicast addresses not supported in hash */
        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Unicast Address"));

    /* CRC calculation */
    crc = get_mac_addr_crc(ethAddr);

    hash = (crc >> TGEC_HASH_MCAST_SHIFT) & TGEC_HASH_ADR_MSK;        /* Take 9 MSB bits */

    /* Create element to be added to the driver hash table */
    p_HashEntry = (t_EthHashEntry *)XX_Malloc(sizeof(t_EthHashEntry));
    p_HashEntry->addr = ethAddr;
    INIT_LIST(&p_HashEntry->node);

    LIST_AddToTail(&(p_HashEntry->node), &(p_Tgec->p_MulticastAddrHash->p_Lsts[hash]));
    tgec_set_hash_table(p_Tgec->p_MemMap, (hash | TGEC_HASH_MCAST_EN));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecDelHashMacAddress(t_Handle h_Tgec, t_EnetAddr *p_EthAddr)
{
    t_Tgec           *p_Tgec = (t_Tgec *)h_Tgec;
    t_EthHashEntry   *p_HashEntry = NULL;
    t_List           *p_Pos;
    uint32_t         crc;
    uint32_t         hash;
    uint64_t         ethAddr;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    ethAddr = ((*(uint64_t *)p_EthAddr) >> 16);

    /* CRC calculation */
    crc = get_mac_addr_crc(ethAddr);

    hash = (crc >> TGEC_HASH_MCAST_SHIFT) & TGEC_HASH_ADR_MSK;        /* Take 9 MSB bits */

    LIST_FOR_EACH(p_Pos, &(p_Tgec->p_MulticastAddrHash->p_Lsts[hash]))
    {
        p_HashEntry = ETH_HASH_ENTRY_OBJ(p_Pos);
        if (p_HashEntry->addr == ethAddr)
        {
            LIST_DelAndInit(&p_HashEntry->node);
            XX_Free(p_HashEntry);
            break;
        }
    }
    if (LIST_IsEmpty(&p_Tgec->p_MulticastAddrHash->p_Lsts[hash]))
        tgec_set_hash_table(p_Tgec->p_MemMap, (hash & ~TGEC_HASH_MCAST_EN));

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecGetId(t_Handle h_Tgec, uint32_t *macId)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    UNUSED(p_Tgec);
    UNUSED(macId);
    RETURN_ERROR(MINOR, E_NOT_SUPPORTED, ("TgecGetId Not Supported"));
}

/* .............................................................................. */

static t_Error TgecGetVersion(t_Handle h_Tgec, uint32_t *macVersion)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    *macVersion = tgec_get_revision(p_Tgec->p_MemMap);

    return E_OK;
}

/* .............................................................................. */

static t_Error TgecSetExcpetion(t_Handle h_Tgec, e_FmMacExceptions exception, bool enable)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;
    uint32_t    bitMask = 0;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE);

    GET_EXCEPTION_FLAG(bitMask, exception);
    if (bitMask)
    {
        if (enable)
            p_Tgec->exceptions |= bitMask;
        else
            p_Tgec->exceptions &= ~bitMask;
   }
    else
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Undefined exception"));

    if (enable)
        tgec_enable_interrupt(p_Tgec->p_MemMap, bitMask);
    else
        tgec_disable_interrupt(p_Tgec->p_MemMap, bitMask);

    return E_OK;
}

/* .............................................................................. */

static uint16_t TgecGetMaxFrameLength(t_Handle h_Tgec)
{
    t_Tgec      *p_Tgec = (t_Tgec *)h_Tgec;

    SANITY_CHECK_RETURN_VALUE(p_Tgec, E_INVALID_HANDLE, 0);
    SANITY_CHECK_RETURN_VALUE(!p_Tgec->p_TgecDriverParam, E_INVALID_STATE, 0);

    return tgec_get_max_frame_len(p_Tgec->p_MemMap);
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
    tgec_enable(p_Tgec->p_MemMap, TRUE, TRUE);
    tgec_set_promiscuous(p_Tgec->p_MemMap, TRUE);
    err = Fm10GTxEccWorkaround(p_Tgec->fmMacControllerDriver.h_Fm, p_Tgec->macId);
    /* disable */
    tgec_set_promiscuous(p_Tgec->p_MemMap, FALSE);
    tgec_enable(p_Tgec->p_MemMap, FALSE, FALSE);
#if defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0)
    if (err)
        XX_Print("FAILED!\n");
    else
        XX_Print("done.\n");
#endif /* (DEBUG_ERRORS > 0) */
    tgec_reset_stat(p_Tgec->p_MemMap);

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
    struct tgec_cfg         *p_TgecDriverParam;
    t_EnetAddr              ethAddr;
    t_Error                 err;

    SANITY_CHECK_RETURN_ERROR(p_Tgec, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->p_TgecDriverParam, E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(p_Tgec->fmMacControllerDriver.h_Fm, E_INVALID_HANDLE);

    FM_GetRevision(p_Tgec->fmMacControllerDriver.h_Fm, &p_Tgec->fmMacControllerDriver.fmRevInfo);
    CHECK_INIT_PARAMETERS(p_Tgec, CheckInitParameters);

    p_TgecDriverParam = p_Tgec->p_TgecDriverParam;

    MAKE_ENET_ADDR_FROM_UINT64(p_Tgec->addr, ethAddr);
    tgec_set_mac_address(p_Tgec->p_MemMap, (uint8_t *)ethAddr);

    /* interrupts */
#ifdef FM_10G_REM_N_LCL_FLT_EX_10GMAC_ERRATA_SW005
    {
        if (p_Tgec->fmMacControllerDriver.fmRevInfo.majorRev <=2)
            p_Tgec->exceptions &= ~(TGEC_IMASK_REM_FAULT | TGEC_IMASK_LOC_FAULT);
    }
#endif /* FM_10G_REM_N_LCL_FLT_EX_10GMAC_ERRATA_SW005 */

    err = tgec_init(p_Tgec->p_MemMap, p_TgecDriverParam, p_Tgec->exceptions);
    if (err) {
        FreeInitResources(p_Tgec);
        RETURN_ERROR(MAJOR, err, ("This TGEC version does not support the required i/f mode"));
    }

    /* Max Frame Length */
    err = FmSetMacMaxFrame(p_Tgec->fmMacControllerDriver.h_Fm,
                           e_FM_MAC_10G,
                           p_Tgec->fmMacControllerDriver.macId,
                           p_TgecDriverParam->max_frame_length);
/* we consider having no IPC a non crasher... */
/*    if (err)
    {
        FreeInitResources(p_Tgec);
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }
*/
#ifdef FM_TX_FIFO_CORRUPTION_ERRATA_10GMAC_A007
    if (p_Tgec->fmMacControllerDriver.fmRevInfo.majorRev == 2)
        tgec_fm_tx_fifo_corruption_errata_10gmac_a007(p_Tgec->p_MemMap);
#endif /* FM_TX_FIFO_CORRUPTION_ERRATA_10GMAC_A007 */

    p_Tgec->p_MulticastAddrHash = AllocHashTable(HASH_TABLE_SIZE);
    if (!p_Tgec->p_MulticastAddrHash)
    {
        FreeInitResources(p_Tgec);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("allocation hash table is FAILED"));
    }

    p_Tgec->p_UnicastAddrHash = AllocHashTable(HASH_TABLE_SIZE);
    if (!p_Tgec->p_UnicastAddrHash)
    {
        FreeInitResources(p_Tgec);
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("allocation hash table is FAILED"));
    }

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

#ifdef FM_TX_ECC_FRMS_ERRATA_10GMAC_A004
    if (p_Tgec->fmMacControllerDriver.fmRevInfo.majorRev <= 6 /*fixed for rev3 */)
    {
        if (!p_Tgec->p_TgecDriverParam->skip_fman11_workaround &&
            ((err = TgecTxEccWorkaround(p_Tgec)) != E_OK))
        {
            FreeInitResources(p_Tgec);
            RETURN_ERROR(MAJOR, err, ("TgecTxEccWorkaround FAILED"));
        }
    }
#endif /* FM_TX_ECC_FRMS_ERRATA_10GMAC_A004 */

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

    p_FmMacControllerDriver->f_FM_MAC_SetTxAutoPauseFrames      = TgecTxMacPause;
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
    t_Tgec              *p_Tgec;
    struct tgec_cfg     *p_TgecDriverParam;
    uintptr_t           baseAddr;

    SANITY_CHECK_RETURN_VALUE(p_FmMacParam, E_NULL_POINTER, NULL);

    baseAddr = p_FmMacParam->baseAddr;
    /* allocate memory for the UCC GETH data structure. */
    p_Tgec = (t_Tgec *)XX_Malloc(sizeof(t_Tgec));
    if (!p_Tgec)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("10G MAC driver structure"));
        return NULL;
    }
    memset(p_Tgec, 0, sizeof(t_Tgec));
    InitFmMacControllerDriver(&p_Tgec->fmMacControllerDriver);

    /* allocate memory for the 10G MAC driver parameters data structure. */
    p_TgecDriverParam = (struct tgec_cfg *) XX_Malloc(sizeof(struct tgec_cfg));
    if (!p_TgecDriverParam)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("10G MAC driver parameters"));
        TgecFree(p_Tgec);
        return NULL;
    }
    memset(p_TgecDriverParam, 0, sizeof(struct tgec_cfg));

    /* Plant parameter structure pointer */
    p_Tgec->p_TgecDriverParam = p_TgecDriverParam;

    tgec_defconfig(p_TgecDriverParam);

    p_Tgec->p_MemMap        = (struct tgec_regs *)UINT_TO_PTR(baseAddr);
    p_Tgec->p_MiiMemMap     = (t_TgecMiiAccessMemMap *)UINT_TO_PTR(baseAddr + TGEC_TO_MII_OFFSET);
    p_Tgec->addr            = ENET_ADDR_TO_UINT64(p_FmMacParam->addr);
    p_Tgec->enetMode        = p_FmMacParam->enetMode;
    p_Tgec->macId           = p_FmMacParam->macId;
    p_Tgec->exceptions      = DEFAULT_exceptions;
    p_Tgec->mdioIrq         = p_FmMacParam->mdioIrq;
    p_Tgec->f_Exception     = p_FmMacParam->f_Exception;
    p_Tgec->f_Event         = p_FmMacParam->f_Event;
    p_Tgec->h_App           = p_FmMacParam->h_App;

    return p_Tgec;
}
