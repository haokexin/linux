/* Copyright (c) 2008-2012 Freescale Semiconductor, Inc.
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

/******************************************************************************
 @File          fm_manip.c

 @Description   FM PCD manip ...
*//***************************************************************************/
#include "std_ext.h"
#include "error_ext.h"
#include "string_ext.h"
#include "debug_ext.h"
#include "fm_pcd_ext.h"
#include "fm_port_ext.h"
#include "fm_muram_ext.h"
#include "memcpy_ext.h"

#include "fm_common.h"
#include "fm_hc.h"
#include "fm_manip.h"


#ifdef CORE_8BIT_ACCESS_ERRATA
#undef WRITE_UINT16
#undef GET_UINT16
#undef WRITE_UINT8
#undef GET_UINT8

#define WRITE_UINT16(addr, val)  \
    do{                             \
            if((int)&(addr) % 4)    \
                WRITE_UINT32(*(uint32_t*)(uint32_t)((uint32_t)&addr & ~0x3L),                                           \
                        ((GET_UINT32(*(uint32_t*)(uint32_t)((uint32_t)&addr & ~0x3L)) & 0xffff0000) | (uint32_t)val));  \
            else                    \
                WRITE_UINT32(*(uint32_t*)&addr,                                                                         \
                        ((GET_UINT32(*(uint32_t*)&addr) & 0x0000ffff) | (uint32_t)val<<16));                            \
      }while(0);
#define GET_UINT16(addr) (((uint32_t)&addr%4) ?           \
       ((uint16_t)GET_UINT32(*(uint32_t*)(uint32_t)((uint32_t)&addr & ~0x3L))):  \
       ((uint16_t)(GET_UINT32(*(uint32_t*)(uint32_t)&addr) >> 16)))

#define WRITE_UINT8(addr,val) WRITE_UINT8_ERRATA(&addr,val)
#define GET_UINT8(addr) GET_UINT8_ERRATA(&addr)


static void WRITE_UINT8_ERRATA(uint8_t *addr, uint8_t val)
{
    uint32_t newAddr, newVal;
    newAddr = (uint32_t)addr & ~0x3L;
    switch ((uint32_t)addr%4)
    {
    case (0):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0x00ffffff) | (((uint32_t)val)<<24);
        WRITE_UINT32(*(uint32_t*)newAddr, newVal);
        break;
    case (1):
         newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0xff00ffff) | (((uint32_t)val)<<16);
        WRITE_UINT32(*(uint32_t*)newAddr, newVal);
        break;
    case (2):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0xffff00ff) | (((uint32_t)val)<<8);
        WRITE_UINT32(*(uint32_t*)newAddr, newVal);
        break;
    case (3):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0xffffff00) | val;
        WRITE_UINT32(*(uint32_t*)newAddr, newVal);
        break;
    }
}

static uint8_t GET_UINT8_ERRATA(uint8_t *addr)
{
    uint32_t newAddr, newVal=0;
    newAddr = (uint32_t)addr & ~0x3L;
    switch ((uint32_t)addr%4)
    {
    case (0):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0xff000000)>>24;
        break;
    case (1):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0x00ff0000)>>16;
        break;
    case (2):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0x0000ff00)>>8;
        break;
    case (3):
        newVal = GET_UINT32(*(uint32_t*)newAddr);
        newVal = (newVal & 0x000000ff);
        break;
    }

    return (uint8_t)newVal;
}

#endif /* CORE_8BIT_ACCESS_ERRATA */

static t_Error GetPrOffsetByNonHeader(uint8_t *parseArrayOffset)
{
    /*For now - the only field in the Parse Array from the NON_BY_TYPE can be e_FM_PCD_KG_EXTRACT_FROM_CURR_END_OF_PARSE*/
    /*Maybe extended in the future*/

    *parseArrayOffset = CC_PC_PR_NEXT_HEADER_OFFSET;

    return E_OK;
}

static t_Error UpdateManipIc(t_Handle h_Manip, uint8_t icOffset)
{
    t_FmPcdManip *p_Manip = (t_FmPcdManip *)h_Manip;
    t_Handle      p_Ad;
    uint32_t      tmpReg32 = 0;
    SANITY_CHECK_RETURN_ERROR(h_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad, E_INVALID_HANDLE);

    switch(p_Manip->type)
    {
        case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
            p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Ad;
           if(p_Manip->updateParams & INTERNAL_CONTEXT_OFFSET)
            {
                tmpReg32 = *(uint32_t *)&((t_AdOfTypeContLookup *)p_Ad)->pcAndOffsets;
                tmpReg32 |=  (uint32_t)((uint32_t)icOffset << 16);
                *(uint32_t *)&((t_AdOfTypeContLookup *)p_Ad)->pcAndOffsets = tmpReg32;
                p_Manip->updateParams &= ~INTERNAL_CONTEXT_OFFSET;
                p_Manip->icOffset = icOffset;
            }
            else
            {
                if(p_Manip->icOffset != icOffset)
                        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("this manipulation was updated previousely by different value"););
            }
            break;
#ifdef FM_CAPWAP_SUPPORT
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
            if(p_Manip->h_Frag)
            {
                if(p_Manip->updateParams & INTERNAL_CONTEXT_OFFSET)
                {
                    p_Ad     = (t_AdOfTypeContLookup *)p_Manip->h_Ad;
                    tmpReg32 |= GET_UINT32(((t_AdOfTypeContLookup *)p_Ad)->pcAndOffsets);
                    tmpReg32 |=  (uint32_t)((uint32_t)icOffset << 16);
                    WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->pcAndOffsets, tmpReg32);
                    p_Manip->updateParams &= ~INTERNAL_CONTEXT_OFFSET;
                    p_Manip->icOffset = icOffset;
                }
                else
                {
                    if(p_Manip->icOffset != icOffset)
                            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("this manipulation was updated previousely by different value"););
                }
            }
            break;
#endif /* FM_CAPWAP_SUPPORT */
    }

    return E_OK;
}

static t_Error UpdateInitMvIntFrameHeaderFromFrameToBufferPrefix(t_Handle h_FmPort, t_FmPcdManip *p_Manip, t_Handle h_Ad, bool validate)
{

    t_AdOfTypeContLookup    *p_Ad         = (t_AdOfTypeContLookup *)h_Ad;
    t_FmPortGetSetCcParams  fmPortGetSetCcParams;
    t_Error                 err;
    uint32_t                tmpReg32;

    memset(&fmPortGetSetCcParams, 0, sizeof(t_FmPortGetSetCcParams));

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Manip->type & HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX), E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(!p_Manip->muramAllocate, E_INVALID_STATE);

    if(p_Manip->updateParams)
    {
        if((!(p_Manip->updateParams & OFFSET_OF_PR)) ||
           (p_Manip->shadowUpdateParams & OFFSET_OF_PR))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has not be updated"));

        fmPortGetSetCcParams.getCcParams.type = p_Manip->updateParams;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_PSO;
        fmPortGetSetCcParams.setCcParams.psoSize = 16;

        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_PR)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Parser result offset wasn't configured previousely"));
#ifdef FM_LOCKUP_ALIGNMENT_ERRATA_FMAN_SW004
        ASSERT_COND(!(fmPortGetSetCcParams.getCcParams.prOffset % 16));
#endif
    }
    else if (validate)
    {
         if((!(p_Manip->shadowUpdateParams & OFFSET_OF_PR)) ||
           (p_Manip->updateParams & OFFSET_OF_PR))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has be updated"));
        fmPortGetSetCcParams.getCcParams.type = p_Manip->shadowUpdateParams;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_PSO;
        fmPortGetSetCcParams.setCcParams.psoSize = 16;

        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_PR)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Parser result offset wasn't configured previousely"));

    }

   if(p_Manip->updateParams & OFFSET_OF_PR)
   {
        tmpReg32 = 0;
        tmpReg32 |= fmPortGetSetCcParams.getCcParams.prOffset;
        WRITE_UINT32(p_Ad->matchTblPtr, (GET_UINT32(p_Ad->matchTblPtr) | tmpReg32));
        p_Manip->updateParams &= ~OFFSET_OF_PR;
        p_Manip->shadowUpdateParams |= OFFSET_OF_PR;
   }
   else if (validate)
   {
        tmpReg32 = GET_UINT32(p_Ad->matchTblPtr);
        if((uint8_t)tmpReg32 != fmPortGetSetCcParams.getCcParams.prOffset)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("this manipulation was updated previousely by different value"););
   }

    return E_OK;
}

#ifdef FM_CAPWAP_SUPPORT
static t_Error UpdateModifyCapwapFragmenation(t_FmPcdManip *p_Manip, t_Handle h_Ad, bool validate,t_Handle h_FmTree)
{
    t_AdOfTypeContLookup            *p_Ad         = (t_AdOfTypeContLookup *)h_Ad;
    t_FmPcdCcSavedManipParams       *p_SavedManipParams = NULL;
    uint32_t                        tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(((p_Manip->type == HMAN_OC_CAPWAP_FRAGMENTATION) || (p_Manip->type == HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER)), E_INVALID_STATE);

    p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Frag;

    if(p_Manip->updateParams)
    {

        if((!(p_Manip->updateParams & OFFSET_OF_DATA) &&
           !(p_Manip->updateParams & BUFFER_POOL_ID_FOR_MANIP)) ||
           ((p_Manip->shadowUpdateParams & OFFSET_OF_DATA) || (p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP)))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has not be updated"));
        p_SavedManipParams = FmPcdCcTreeGetSavedManipParams(h_FmTree, e_FM_MANIP_CAPWAP_INDX);
        if(!p_SavedManipParams)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("for this manipulation tree has to be configured previosely with this type"));
        p_Manip->fragParams.poolId = p_SavedManipParams->capwapParams.poolId;
        p_Manip->fragParams.dataOffset = p_SavedManipParams->capwapParams.dataOffset;

        tmpReg32 = GET_UINT32(p_Ad->pcAndOffsets);
        tmpReg32 |= ((uint32_t)p_Manip->fragParams.poolId << 8);
        tmpReg32 |= ((uint32_t)p_Manip->fragParams.dataOffset<< 16);
        WRITE_UINT32(p_Ad->pcAndOffsets,tmpReg32);

        p_Manip->updateParams &= ~OFFSET_OF_DATA;
        p_Manip->updateParams &= ~BUFFER_POOL_ID_FOR_MANIP;
        p_Manip->shadowUpdateParams |= (OFFSET_OF_DATA | BUFFER_POOL_ID_FOR_MANIP);
    }
   else if (validate)
   {

        p_SavedManipParams = FmPcdCcTreeGetSavedManipParams(h_FmTree, e_FM_MANIP_CAPWAP_INDX);
        if(!p_SavedManipParams)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("for this manipulation tree has to be configured previosely with this type"));
        if((p_Manip->fragParams.poolId != p_SavedManipParams->capwapParams.poolId) ||
           (p_Manip->fragParams.dataOffset != p_SavedManipParams->capwapParams.dataOffset))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("this manipulation was updated previousely by different value"));
   }

    return E_OK;
}

static t_Error UpdateInitCapwapFragmentation(t_Handle h_FmPort, t_FmPcdManip *p_Manip, t_Handle h_Ad, bool validate, t_Handle h_FmTree)
{
    t_AdOfTypeContLookup        *p_Ad;
    t_FmPortGetSetCcParams      fmPortGetSetCcParams;
    t_Error                     err;
    uint32_t                    tmpReg32 = 0;
    t_FmPcdCcSavedManipParams   *p_SavedManipParams;

    UNUSED(h_Ad);

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(((p_Manip->type == HMAN_OC_CAPWAP_FRAGMENTATION) || (p_Manip->type == HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER)), E_INVALID_STATE);

    p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Frag;

    if(p_Manip->updateParams)
    {
        if((!(p_Manip->updateParams & OFFSET_OF_DATA) &&
           !(p_Manip->updateParams & BUFFER_POOL_ID_FOR_MANIP)) ||
           ((p_Manip->shadowUpdateParams & OFFSET_OF_DATA) || (p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP)))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has not be updated"));
        fmPortGetSetCcParams.getCcParams.type = p_Manip->updateParams;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN | UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_FRAG | NIA_ENG_FM_CTL;
        fmPortGetSetCcParams.getCcParams.poolIndex = p_Manip->fragParams.poolIndx;
        /*for CAPWAP Rassembly used FMAN_CTRL2 hardcoded - so for fragmentation its better to use FMAN_CTRL1*/
        fmPortGetSetCcParams.setCcParams.orFmanCtrl = FPM_PORT_FM_CTL1;

        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_DATA)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Data offset wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & BUFFER_POOL_ID_FOR_MANIP)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Buffer pool doe header manipulation wasn't configured previousely"));

        p_SavedManipParams = (t_FmPcdCcSavedManipParams *)XX_Malloc(sizeof(t_FmPcdCcSavedManipParams));
        p_SavedManipParams->capwapParams.dataOffset = fmPortGetSetCcParams.getCcParams.dataOffset;
        p_SavedManipParams->capwapParams.poolId = fmPortGetSetCcParams.getCcParams.poolIdForManip;

#ifdef FM_LOCKUP_ALIGNMENT_ERRATA_FMAN_SW004
        ASSERT_COND(!(p_SavedManipParams->capwapParams.dataOffset % 16));
#endif

        FmPcdCcTreeSetSavedManipParams(h_FmTree, (t_Handle)p_SavedManipParams, e_FM_MANIP_CAPWAP_INDX);
    }
    else if (validate)
    {
        if ((!(p_Manip->shadowUpdateParams & OFFSET_OF_DATA) &&
            !(p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP)) ||
            ((p_Manip->updateParams & OFFSET_OF_DATA) ||
            (p_Manip->updateParams & BUFFER_POOL_ID_FOR_MANIP)))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has be updated"));
        fmPortGetSetCcParams.getCcParams.type = p_Manip->shadowUpdateParams;
        fmPortGetSetCcParams.getCcParams.poolIndex = p_Manip->fragParams.poolIndx;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN | UPDATE_FMFP_PRC_WITH_ONE_RISC_ONLY;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_FRAG | NIA_ENG_FM_CTL;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_DATA)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Data offset wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & BUFFER_POOL_ID_FOR_MANIP)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Buffer pool doe header manipulation wasn't configured previousely"));
    }

    if(p_Manip->updateParams)
    {
        tmpReg32 = GET_UINT32(p_Ad->pcAndOffsets);
        tmpReg32 |= ((uint32_t)fmPortGetSetCcParams.getCcParams.poolIdForManip << 8);
        tmpReg32 |= ((uint32_t)fmPortGetSetCcParams.getCcParams.dataOffset<< 16);
        WRITE_UINT32(p_Ad->pcAndOffsets,tmpReg32);

        p_Manip->updateParams &= ~OFFSET_OF_DATA;
        p_Manip->updateParams &= ~BUFFER_POOL_ID_FOR_MANIP;
        p_Manip->shadowUpdateParams |= (OFFSET_OF_DATA | BUFFER_POOL_ID_FOR_MANIP);
        p_Manip->fragParams.poolId = fmPortGetSetCcParams.getCcParams.poolIdForManip;
        p_Manip->fragParams.dataOffset = fmPortGetSetCcParams.getCcParams.dataOffset;
    }
    else if (validate)
    {
        if((p_Manip->fragParams.poolId != fmPortGetSetCcParams.getCcParams.poolIdForManip) ||
           (p_Manip->fragParams.dataOffset != fmPortGetSetCcParams.getCcParams.dataOffset))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("this manipulation was updated previousely by different value"));
    }

    return E_OK;
}

static t_Error UpdateInitCapwapReasm(t_Handle                   h_FmPcd,
                                     t_Handle                       h_FmPort,
                                     t_FmPcdManip                   *p_Manip,
                                     t_Handle                       h_Ad,
                                     bool                           validate)
{
    t_CapwapReasmPram  *p_ReassmTbl;
    t_Error             err;
    t_FmPortGetSetCcParams  fmPortGetSetCcParams;
    uint8_t             i = 0;
    uint16_t            size;
    uint32_t            tmpReg32;
    t_FmPcd             *p_FmPcd = (t_FmPcd *)h_FmPcd;
    t_FmPcdCcCapwapReassmTimeoutParams ccCapwapReassmTimeoutParams;

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Manip->frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Manip->type == HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST), E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(h_FmPcd,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPcd->h_Hc,E_INVALID_HANDLE);

    if(p_Manip->h_FmPcd != h_FmPcd)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("handler of PCD previously was initiated by different value"));

    UNUSED(h_Ad);

    memset(&fmPortGetSetCcParams, 0, sizeof(t_FmPortGetSetCcParams));
    p_ReassmTbl  = (t_CapwapReasmPram *)p_Manip->h_Frag;

    if(p_Manip->updateParams)
    {
        if((!(p_Manip->updateParams & NUM_OF_TASKS) && !(p_Manip->updateParams & BUFFER_POOL_ID_FOR_MANIP) &&
        !(p_Manip->updateParams & OFFSET_OF_DATA) && !(p_Manip->updateParams & OFFSET_OF_PR) &&
        !(p_Manip->updateParams & HW_PORT_ID)) ||
           ((p_Manip->shadowUpdateParams & NUM_OF_TASKS) || (p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP) ||
           (p_Manip->shadowUpdateParams & OFFSET_OF_DATA) || (p_Manip->shadowUpdateParams & OFFSET_OF_PR)
           ||(p_Manip->shadowUpdateParams & HW_PORT_ID)))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has not be updated"));

        fmPortGetSetCcParams.getCcParams.type = p_Manip->updateParams;
        fmPortGetSetCcParams.getCcParams.poolIndex = p_Manip->fragParams.poolIndx;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_FRAG | NIA_ENG_FM_CTL;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if(fmPortGetSetCcParams.getCcParams.type & NUM_OF_TASKS)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Num of tasks wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_DATA)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("offset of the data  wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & BUFFER_POOL_ID_FOR_MANIP)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("buffser pool id  wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & HW_PORT_ID)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("hwPortId wasn't updated"));
#ifdef FM_LOCKUP_ALIGNMENT_ERRATA_FMAN_SW004
        ASSERT_COND((fmPortGetSetCcParams.getCcParams.dataOffset % 16) == 0);
#endif
    }
    else if (validate)
    {
         if((!(p_Manip->shadowUpdateParams & NUM_OF_TASKS) && (!(p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP)) &&
         (!(p_Manip->shadowUpdateParams & OFFSET_OF_DATA)) && (!(p_Manip->shadowUpdateParams & OFFSET_OF_PR)) &&
         (!(p_Manip->shadowUpdateParams & HW_PORT_ID))) &&
           ((p_Manip->updateParams & NUM_OF_TASKS) ||
           (p_Manip->updateParams & BUFFER_POOL_ID_FOR_MANIP) ||
           (p_Manip->updateParams & OFFSET_OF_DATA) || (p_Manip->updateParams & OFFSET_OF_PR)||
            (p_Manip->updateParams & HW_PORT_ID)))

            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has be updated"));
        fmPortGetSetCcParams.getCcParams.type = p_Manip->shadowUpdateParams;
        fmPortGetSetCcParams.getCcParams.poolIndex = p_Manip->fragParams.poolIndx;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_FRAG | NIA_ENG_FM_CTL;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if(fmPortGetSetCcParams.getCcParams.type & NUM_OF_TASKS)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("NumOfTasks wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & BUFFER_POOL_ID_FOR_MANIP)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Buffer pool for header manipulation wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_DATA)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("offset of the data  wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & HW_PORT_ID)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("hwPortId wasn't updated"));
    }

    if(p_Manip->updateParams)
    {
        if(p_Manip->updateParams & NUM_OF_TASKS)
        {
            /*recommendation of Microcode team - (maxNumFramesInProcess * 2) */
            size = (uint16_t)(p_Manip->fragParams.maxNumFramesInProcess*2 + fmPortGetSetCcParams.getCcParams.numOfTasks);
            if(size  > 255)
                RETURN_ERROR(MAJOR,E_INVALID_VALUE, ("numOfOpenReassmEntries + numOfTasks per port can not be greater than 256"));

            p_Manip->fragParams.numOfTasks = fmPortGetSetCcParams.getCcParams.numOfTasks;

            /*p_ReassmFrmDescrIndxPoolTbl*/
            p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                              (uint32_t)(size + 1),
                                              4);
            if(!p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl)
                RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

            IOMemSet32(p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl, 0,  (uint32_t)(size + 1));

            for( i = 0; i < size; i++)
                WRITE_UINT8(*(uint8_t *)PTR_MOVE(p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl, i), (uint8_t)(i+1));

            tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl) - p_FmPcd->physicalMuramBase);

            WRITE_UINT32(p_ReassmTbl->reasmFrmDescIndexPoolTblPtr, tmpReg32);

            /*p_ReassmFrmDescrPoolTbl*/
            p_Manip->fragParams.p_ReassmFrmDescrPoolTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                              (uint32_t)((size + 1) * FM_PCD_MANIP_CAPWAP_REASM_RFD_SIZE),
                                              4);

           if(!p_Manip->fragParams.p_ReassmFrmDescrPoolTbl)
                RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

            IOMemSet32(p_Manip->fragParams.p_ReassmFrmDescrPoolTbl, 0,  (uint32_t)((size +1)* FM_PCD_MANIP_CAPWAP_REASM_RFD_SIZE));

            tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->fragParams.p_ReassmFrmDescrPoolTbl) - p_FmPcd->physicalMuramBase);

            WRITE_UINT32(p_ReassmTbl->reasmFrmDescPoolTblPtr, tmpReg32);

            /*p_TimeOutTbl*/

            p_Manip->fragParams.p_TimeOutTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                              (uint32_t)((size + 1)* FM_PCD_MANIP_CAPWAP_REASM_TIME_OUT_ENTRY_SIZE),
                                              4);

            if(!p_Manip->fragParams.p_TimeOutTbl)
                RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

            IOMemSet32(p_Manip->fragParams.p_TimeOutTbl, 0,  (uint16_t)((size + 1)*FM_PCD_MANIP_CAPWAP_REASM_TIME_OUT_ENTRY_SIZE));

            tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->fragParams.p_TimeOutTbl) - p_FmPcd->physicalMuramBase);
            WRITE_UINT32(p_ReassmTbl->timeOutTblPtr, tmpReg32);


            p_Manip->updateParams &= ~NUM_OF_TASKS;
            p_Manip->shadowUpdateParams |= NUM_OF_TASKS;
       }

       if(p_Manip->updateParams & BUFFER_POOL_ID_FOR_MANIP)
       {

            p_Manip->fragParams.poolId = fmPortGetSetCcParams.getCcParams.poolIdForManip;

            tmpReg32 = GET_UINT32(p_ReassmTbl->bufferPoolIdAndRisc1SetIndexes);
            tmpReg32 |= (uint32_t)p_Manip->fragParams.poolId << 16;
            WRITE_UINT32(p_ReassmTbl->bufferPoolIdAndRisc1SetIndexes, tmpReg32);

            p_Manip->updateParams &= ~BUFFER_POOL_ID_FOR_MANIP;
            p_Manip->shadowUpdateParams |= BUFFER_POOL_ID_FOR_MANIP;
       }

        if(p_Manip->updateParams & OFFSET_OF_DATA)
        {
            p_Manip->fragParams.dataOffset = fmPortGetSetCcParams.getCcParams.dataOffset;
            tmpReg32 = GET_UINT32(p_ReassmTbl->mode);
            tmpReg32|= p_Manip->fragParams.dataOffset;
            WRITE_UINT32(p_ReassmTbl->mode, tmpReg32);
            p_Manip->updateParams &= ~OFFSET_OF_DATA;
            p_Manip->shadowUpdateParams |= OFFSET_OF_DATA;
        }
        if(!(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_PR))
        {
            p_Manip->fragParams.prOffset = fmPortGetSetCcParams.getCcParams.prOffset;

            tmpReg32 = GET_UINT32(p_ReassmTbl->mode);
            tmpReg32|= FM_PCD_MANIP_CAPWAP_REASM_PR_COPY;
            WRITE_UINT32(p_ReassmTbl->mode, tmpReg32);

            tmpReg32 = GET_UINT32(p_ReassmTbl->intStatsTblPtr);
            tmpReg32 |= (uint32_t)p_Manip->fragParams.prOffset << 24;
            WRITE_UINT32(p_ReassmTbl->intStatsTblPtr, tmpReg32);
           p_Manip->updateParams &= ~OFFSET_OF_PR;
           p_Manip->shadowUpdateParams |= OFFSET_OF_PR;
       }
       else
       {
           p_Manip->fragParams.prOffset = 0xff;
           p_Manip->updateParams &= ~OFFSET_OF_PR;
           p_Manip->shadowUpdateParams |= OFFSET_OF_PR;

       }

        p_Manip->fragParams.hwPortId = fmPortGetSetCcParams.getCcParams.hardwarePortId;
        p_Manip->updateParams &= ~HW_PORT_ID;
        p_Manip->shadowUpdateParams |= HW_PORT_ID;

        /*timeout hc */
       ccCapwapReassmTimeoutParams.fqidForTimeOutFrames = p_Manip->fragParams.fqidForTimeOutFrames;
       ccCapwapReassmTimeoutParams.portIdAndCapwapReassmTbl = (uint32_t)p_Manip->fragParams.hwPortId << 24;
       ccCapwapReassmTimeoutParams.portIdAndCapwapReassmTbl |= (uint32_t)((XX_VirtToPhys(p_ReassmTbl) - p_FmPcd->physicalMuramBase));
       ccCapwapReassmTimeoutParams.timeoutRequestTime = (((uint32_t)1<<p_Manip->fragParams.bitFor1Micro) * p_Manip->fragParams.timeoutRoutineRequestTime)/2;
       return FmHcPcdCcCapwapTimeoutReassm(p_FmPcd->h_Hc,&ccCapwapReassmTimeoutParams);

    }
    else if(validate)
    {
        if(fmPortGetSetCcParams.getCcParams.hardwarePortId != p_Manip->fragParams.hwPortId)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Reassembly manipulation previously was assigned to another port"));
        if(fmPortGetSetCcParams.getCcParams.numOfTasks != p_Manip->fragParams.numOfTasks)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("numOfTasks for this manipulation previously was defined by another value "));

        if(fmPortGetSetCcParams.getCcParams.poolIdForManip != p_Manip->fragParams.poolId)
            RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("poolId for this manipulation previously was defined by another value "));

        if(!(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_PR))
        {
            if(p_Manip->fragParams.prOffset != fmPortGetSetCcParams.getCcParams.prOffset)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Parse result offset previously was defined by another value "));
        }
        else
        {
            if(p_Manip->fragParams.prOffset != 0xff)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Parse result offset previously was defined by another value "));
        }
        if(fmPortGetSetCcParams.getCcParams.dataOffset != p_Manip->fragParams.dataOffset)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Data offset previously was defined by another value "));
    }

    return E_OK;
}
#endif /* FM_CAPWAP_SUPPORT */

#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
static t_Error CreateIpReassCommonParamTable(t_FmPcdManip *p_Manip,
                                             t_FmPcd *p_FmPcd ,
                                             t_IpReasmCommonTbl *p_IpReasmCommonPramTbl)
{
    uint32_t    tmpReg32 = 0, i;
    uint64_t    tmpReg64, size;
    t_Error     err = E_OK;

    /* Allocation of the IP Reassembly Common Parameters table. This table is located in the
    MURAM. Its size is 64 bytes and its base address should be 8-byte aligned.
    It contains parameters that are common to both the IPv4 reassembly function and IPv6
    reassembly function.*/
    p_Manip->ipReassmParams.h_IpReassCommonParamsTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                                       FM_PCD_MANIP_IP_REASM_COMMON_PARAM_TABLE_SIZE,
                                                       FM_PCD_MANIP_IP_REASM_COMMON_PARAM_TABLE_ALIGN);

    if(!p_Manip->ipReassmParams.h_IpReassCommonParamsTbl)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

    p_IpReasmCommonPramTbl = (t_IpReasmCommonTbl *)(p_Manip->ipReassmParams.h_IpReassCommonParamsTbl);

    IOMemSet32(p_IpReasmCommonPramTbl, 0,  FM_PCD_MANIP_IP_REASM_COMMON_PARAM_TABLE_SIZE);

    /* Setting the TimeOut Mode.*/
    tmpReg32 = 0;
    if(p_Manip->ipReassmParams.timeOutMode == e_FM_PCD_MANIP_TIME_OUT_BETWEEN_FRAMES)
        tmpReg32 |= FM_PCD_MANIP_IP_REASM_TIME_OUT_BETWEEN_FRAMES;

    /* Setting TimeOut FQID - Frames that time out are enqueued to this FQID.
    In order to cause TimeOut frames to be discarded, this queue should be configured accordingly*/
    tmpReg32 |= p_Manip->ipReassmParams.fqidForTimeOutFrames;
    WRITE_UINT32(p_IpReasmCommonPramTbl->timeoutModeAndFqid, tmpReg32);

    /* Calculation the size of IP Reassembly Frame Descriptor - number of frames that are allowed to be reassembled simultaneously + 128.*/
    size = p_Manip->ipReassmParams.maxNumFramesInProcess + 128;

    /*Allocation of IP Reassembly Frame Descriptor Indexes Pool - This pool resides in the MURAM */
    p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                      (uint32_t)(size * 2),
                                      256);
    if(!p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

    IOMemSet32(p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl, 0,  (uint32_t)(size * 2));

    /* The entries in IP Reassembly Frame Descriptor Indexes Pool contains indexes starting with 1 up to
    the maximum number of frames that are allowed to be reassembled simultaneously + 128.
    The last entry in this pool must contain the index zero*/
    for( i = 0; i < size - 1; i++)
        WRITE_UINT16(*(uint16_t *)PTR_MOVE_16(p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl, i), (uint16_t)(i+1));

    /* Sets the IP Reassembly Frame Descriptor Indexes Pool offset from MURAM */
    tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl) - p_FmPcd->physicalMuramBase);
    WRITE_UINT32(p_IpReasmCommonPramTbl->reassFrmDescIndexPoolTblPtr, tmpReg32);

    /* Allocation of the Reassembly Frame Descriptors Pool - This pool resides in external memory.
    The number of entries in this pool should be equal to the number of entries in IP Reassembly Frame Descriptor Indexes Pool.*/
    p_Manip->ipReassmParams.h_ReassmFrmDescrPoolTbl = (t_Handle)XX_MallocSmart((uint32_t)(size * 64), p_Manip->ipReassmParams.dataMemId, 64);

    if(!p_Manip->ipReassmParams.h_ReassmFrmDescrPoolTbl)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation FAILED"));

    IOMemSet32(p_Manip->ipReassmParams.h_ReassmFrmDescrPoolTbl, 0,  (uint32_t)(size * 32));

    /* Sets the Reassembly Frame Descriptors Pool and liodn offset*/
    tmpReg64 = (uint64_t)(XX_VirtToPhys(p_Manip->ipReassmParams.h_ReassmFrmDescrPoolTbl));
    tmpReg64 |= ((uint64_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_LIODN_MASK) << (uint64_t)FM_PCD_MANIP_IP_REASM_LIODN_SHIFT);
    tmpReg64 |= ((uint64_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_ELIODN_MASK) << (uint64_t)FM_PCD_MANIP_IP_REASM_ELIODN_SHIFT);
    WRITE_UINT32(p_IpReasmCommonPramTbl->liodnAndReassFrmDescPoolPtrHi, (uint32_t)(tmpReg64 >> 32));
    WRITE_UINT32(p_IpReasmCommonPramTbl->reassFrmDescPoolPtrLow, (uint32_t)tmpReg64);

    /*Allocation of the TimeOut table - This table resides in the MURAM.
    The number of entries in this table is identical to the number of entries in the Reassembly Frame Descriptors Pool*/
    p_Manip->ipReassmParams.h_TimeOutTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                           (uint32_t)(size  * 8),8);

    if(!p_Manip->ipReassmParams.h_TimeOutTbl)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

    IOMemSet32(p_Manip->ipReassmParams.h_TimeOutTbl, 0,  (uint16_t)(size * 8));

    /* Sets the TimeOut table offset from MURAM*/
    tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->ipReassmParams.h_TimeOutTbl) - p_FmPcd->physicalMuramBase);
    WRITE_UINT32(p_IpReasmCommonPramTbl->timeOutTblPtr, tmpReg32);

    /* Sets the Expiration Delay */
    tmpReg32 = 0;
    tmpReg32 |= p_Manip->ipReassmParams.timeoutThresholdForReassmProcess * 256;
    WRITE_UINT32(p_IpReasmCommonPramTbl->expirationDelay, tmpReg32);

    /* Counts the number of TimeOut occurrences - Must be initialized to zero.*/
    WRITE_UINT32(p_IpReasmCommonPramTbl->totalTimeOutCounter, 0);
    /* Counts the number of failed attempts to allocate a Reassembly Frame Descriptor - Must be initialized to zero.*/
    WRITE_UINT32(p_IpReasmCommonPramTbl->totalRfdPoolBusyCounter, 0);
    /* Counts the number of times an internal buffer busy occured.*/
    WRITE_UINT32(p_IpReasmCommonPramTbl->totalInternalBufferBusy, 0);
    /* Counts the number of times external buffer busy occured. */
    WRITE_UINT32(p_IpReasmCommonPramTbl->totalExternalBufferBusy, 0);

    err = FmPcdRegisterReassmPort(p_FmPcd, p_IpReasmCommonPramTbl);
    if (err != E_OK)
    {
        FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_IpReasmCommonPramTbl);
        RETURN_ERROR(MAJOR, err, ("port registration"));
    }

    return err;
}

static t_Handle CreateIpReassParamTable(t_FmPcdManip *p_Manip,  bool ipv4)
{
    t_FmPcd                 *p_FmPcd = p_Manip->h_FmPcd;
    uint32_t                tmpReg32, autoLearnHashTblSize;
    uint32_t                numOfWays, setSize, setSizeCode, tmpSetSize;
    uint32_t                waySize, numOfSets, tmpNumOfSets, numOfEntries;
    uint64_t                tmpReg64;
    uint16_t                minFragSize;
    t_Handle                *h_AutoLearnHashTbl, *h_AutoLearnSetLockTblPtr, h_IpReassParamsTblPtr;
    t_IpReasmPram           *p_IpReassParamsTblPtr;

    /* Allocates the IP Reassembly Parameters Table - This table is located in the MURAM.*/
    h_IpReassParamsTblPtr = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                                        FM_PCD_MANIP_IP_REASM_TABLE_SIZE,
                                                        FM_PCD_MANIP_IP_REASM_TABLE_ALIGN);

    if(!h_IpReassParamsTblPtr)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
        return NULL;
    }

    p_IpReassParamsTblPtr = (t_IpReasmPram *)h_IpReassParamsTblPtr;
    memset(p_IpReassParamsTblPtr, 0, sizeof(t_IpReasmPram));

    /* Sets the IP Reassembly common Parameters table offset from MURAM in the IP Reassembly Table descriptor*/
    tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->ipReassmParams.h_IpReassCommonParamsTbl) - p_FmPcd->physicalMuramBase);
    WRITE_UINT32(p_IpReassParamsTblPtr->ipReassCommonPrmTblPtr, tmpReg32);

    /* Get user's requested number of ways */
    numOfWays = p_Manip->ipReassmParams.numOfFramesPerHashEntry;

    /*It is recommended that the total number of entries in this table
    (number of sets * number of ways) will be twice the number of frames that
     are expected to be reassembled simultaneously.*/
    numOfEntries = (uint32_t)(p_Manip->ipReassmParams.maxNumFramesInProcess * 2);

    /* sets number calculation - number of entries = number of sets * number of ways */
    numOfSets = numOfEntries / numOfWays;

    /* Calculates way size */
    switch(p_Manip->ipReassmParams.hdr)
    {
        case(HEADER_TYPE_IPv6):
            /* WaySize is rounded-up to next multiple of 8 */
            waySize = ROUND_UP(((16 + 16 + 4) /* * numOfWays*/),8);
            break;
        case(HEADER_TYPE_IPv4):
            waySize = ROUND_UP(((4 + 4 + 1 + 2)),8);
            break;
        default:
            REPORT_ERROR(MAJOR, E_INVALID_STATE, ("Unsupported header for reassembly"));
            return NULL;
    }

    /* Calculate set size (set size is rounded-up to next power of 2) */
    LOG2(numOfWays * waySize, tmpSetSize);
    setSize =  (uint32_t)(1 << (tmpSetSize + (POWER_OF_2(numOfWays * waySize) ? 0 : 1)));

    /* Get set size code */
    LOG2(setSize, setSizeCode);

    /* Sets ways number and set size code */
    WRITE_UINT16(p_IpReassParamsTblPtr->waysNumAndSetSize, (uint16_t)((numOfWays << 8) | setSizeCode));

    /* Sets AutoLearnHashKeyMask*/
    LOG2(numOfSets, tmpNumOfSets);
    numOfSets = (uint32_t)(1 << (tmpNumOfSets + (POWER_OF_2(numOfSets) ? 0 : 1)));
    WRITE_UINT16(p_IpReassParamsTblPtr->autoLearnHashKeyMask, (uint16_t)(numOfSets - 1));

    /* Allocation of IP Reassembly Automatic Learning Hash Table - This table resides in external memory.
    The size of this table is determined by the number of sets and the set size.
    Table size = set size * number of sets
    This table’s base address should be aligned to SetSize.*/
    autoLearnHashTblSize = numOfSets * setSize;

    if (ipv4)
        h_AutoLearnHashTbl = &p_Manip->ipReassmParams.h_Ipv4AutoLearnHashTbl;
    else
        h_AutoLearnHashTbl = &p_Manip->ipReassmParams.h_Ipv6AutoLearnHashTbl;

    *h_AutoLearnHashTbl = (t_Handle)XX_MallocSmart(autoLearnHashTblSize, p_Manip->ipReassmParams.dataMemId, setSize);

    if(!*h_AutoLearnHashTbl)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation FAILED"));
        return NULL;
    }
    IOMemSet32(*h_AutoLearnHashTbl, 0,  autoLearnHashTblSize);

    /* Sets the IP Reassembly Automatic Learning Hash Table and liodn offset */
    tmpReg64 = ((uint64_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_LIODN_MASK) << (uint64_t)FM_PCD_MANIP_IP_REASM_LIODN_SHIFT);
    tmpReg64 |= ((uint64_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_ELIODN_MASK) << (uint64_t)FM_PCD_MANIP_IP_REASM_ELIODN_SHIFT);
    tmpReg64 |= XX_VirtToPhys(*h_AutoLearnHashTbl);
    WRITE_UINT32(p_IpReassParamsTblPtr->liodnAlAndAutoLearnHashTblPtrHi, (uint32_t)(tmpReg64 >> 32));
    WRITE_UINT32(p_IpReassParamsTblPtr->autoLearnHashTblPtrLow, (uint32_t)tmpReg64);

    /* Allocation of the Set Lock table - This table resides in external memory
    The size of this table is (number of sets in the IP Reassembly Automatic Learning Hash table)*4 bytes.
    This table resides in external memory and its base address should be 4-byte aligned */
    if (ipv4)
        h_AutoLearnSetLockTblPtr = &p_Manip->ipReassmParams.h_Ipv4AutoLearnSetLockTblPtr;
    else
        h_AutoLearnSetLockTblPtr = &p_Manip->ipReassmParams.h_Ipv6AutoLearnSetLockTblPtr;

    *h_AutoLearnSetLockTblPtr = (t_Handle)XX_MallocSmart((uint32_t)(numOfSets * 4), p_Manip->ipReassmParams.dataMemId, 4);

    if(!*h_AutoLearnSetLockTblPtr)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation FAILED"));
        return NULL;
    }

    IOMemSet32(*h_AutoLearnSetLockTblPtr, 0,  (numOfSets * 4));

    /* sets Set Lock table pointer and liodn offset*/
    tmpReg64 = ((uint64_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_LIODN_MASK) << (uint64_t)FM_PCD_MANIP_IP_REASM_LIODN_SHIFT);
    tmpReg64 |= ((uint64_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_ELIODN_MASK) << (uint64_t)FM_PCD_MANIP_IP_REASM_ELIODN_SHIFT);
    tmpReg64 |= XX_VirtToPhys(*h_AutoLearnSetLockTblPtr);
    WRITE_UINT32(p_IpReassParamsTblPtr->liodnSlAndAutoLearnSetLockTblPtrHi, (uint32_t)(tmpReg64 >> 32));
    WRITE_UINT32(p_IpReassParamsTblPtr->autoLearnSetLockTblPtrLow, (uint32_t)tmpReg64);

    /* Sets user's requested minimum fragment size (in Bytes) for First/Middle fragment */
    minFragSize = ipv4 ? p_Manip->ipReassmParams.minFragSize[0] : p_Manip->ipReassmParams.minFragSize[1];
    WRITE_UINT16(p_IpReassParamsTblPtr->minFragSize, minFragSize);

    /* Zeroes all counters */
    WRITE_UINT32(p_IpReassParamsTblPtr->totalSuccessfullyReasmFramesCounter, 0);
    WRITE_UINT32(p_IpReassParamsTblPtr->totalValidFragmentCounter, 0);
    WRITE_UINT32(p_IpReassParamsTblPtr->totalProcessedFragCounter, 0);
    WRITE_UINT32(p_IpReassParamsTblPtr->totalMalformdFragCounter, 0);
    WRITE_UINT32(p_IpReassParamsTblPtr->totalSetBusyCounter, 0);
    WRITE_UINT32(p_IpReassParamsTblPtr->totalDiscardedFragsCounter, 0);
    WRITE_UINT32(p_IpReassParamsTblPtr->totalMoreThan16FramesCounter, 0);

    /* Return the pointer to the IP Reassembly table */
    return h_IpReassParamsTblPtr;
}

static t_Error UpdateInitIpReasm(t_Handle       h_FmPcd,
                                 t_Handle       h_PcdParams,
                                 t_Handle       h_FmPort,
                                 t_FmPcdManip   *p_Manip,
                                 t_Handle       h_Ad,
                                 bool           validate)
{
    t_AdOfTypeContLookup        *p_Ipv4Ad = NULL, *p_Ipv6Ad = NULL;
    t_FmPortGetSetCcParams      fmPortGetSetCcParams;
    uint32_t                    tmpReg32;
    t_Error                     err;
    t_FmPortPcdParams           *p_PcdParams = (t_FmPortPcdParams *)h_PcdParams;

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Manip->frag,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR((p_Manip->type == HMAN_OC_IP_REASSEMBLY), E_INVALID_STATE);
    SANITY_CHECK_RETURN_ERROR(h_FmPcd,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(!p_Manip->updateParams || h_PcdParams,E_INVALID_HANDLE);

    UNUSED(h_Ad);

    if( p_Manip->ipReassmParams.h_Ipv4Ad != NULL)
         p_Ipv4Ad = (t_AdOfTypeContLookup *)p_Manip->ipReassmParams.h_Ipv4Ad;
    if( p_Manip->ipReassmParams.h_Ipv6Ad != NULL)
         p_Ipv6Ad = (t_AdOfTypeContLookup *)p_Manip->ipReassmParams.h_Ipv6Ad;

    if(p_Manip->h_FmPcd != h_FmPcd)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("handler of PCD previously was initiated by different value"));

    memset(&fmPortGetSetCcParams, 0, sizeof(t_FmPortGetSetCcParams));


    if(p_Manip->updateParams)
    {
        if((!(p_Manip->updateParams & OFFSET_OF_DATA) && !(p_Manip->updateParams & HW_PORT_ID)) ||
           ((p_Manip->shadowUpdateParams & NUM_OF_TASKS) || (p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP) ||
           (p_Manip->shadowUpdateParams & OFFSET_OF_DATA) ||(p_Manip->shadowUpdateParams & HW_PORT_ID)))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has not be updated"));

        fmPortGetSetCcParams.setCcParams.type = UPDATE_IPR_EN;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_FRAG | NIA_ENG_FM_CTL;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        fmPortGetSetCcParams.getCcParams.type = p_Manip->updateParams;
        fmPortGetSetCcParams.getCcParams.poolIndex = p_Manip->fragParams.poolIndx;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_RFENE;
        fmPortGetSetCcParams.setCcParams.nia = (NIA_FM_CTL_AC_FRAG_CHECK | NIA_ORDER_RESTOR);
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if(fmPortGetSetCcParams.getCcParams.type & NUM_OF_TASKS)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Num of tasks wasn't configured previously"));
        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_DATA)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("offset of the data  wasn't configured previously"));
        if(fmPortGetSetCcParams.getCcParams.type & HW_PORT_ID)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("hwPortId wasn't updated"));

        if(p_Manip->ipReassmParams.h_Ipv4Scheme)
        {
            p_PcdParams->p_KgParams->h_Schemes[p_PcdParams->p_KgParams->numOfSchemes] = p_Manip->ipReassmParams.h_Ipv4Scheme;
            p_PcdParams->p_KgParams->numOfSchemes++;
        }
        if(p_Manip->ipReassmParams.h_Ipv6Scheme)
        {
            p_PcdParams->p_KgParams->h_Schemes[p_PcdParams->p_KgParams->numOfSchemes] = p_Manip->ipReassmParams.h_Ipv6Scheme;
            p_PcdParams->p_KgParams->numOfSchemes++;
        }
    }
    else if (validate)
    {
         if((!(p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP) &&
         (!(p_Manip->shadowUpdateParams & OFFSET_OF_DATA)) &&
         (!(p_Manip->shadowUpdateParams & HW_PORT_ID))) &&
           ((p_Manip->updateParams & NUM_OF_TASKS) ||
           (p_Manip->updateParams & OFFSET_OF_DATA) ||
            (p_Manip->updateParams & HW_PORT_ID)))
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("in this stage parameters from Port has be updated"));

        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNEN;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_FRAG | NIA_ENG_FM_CTL;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        fmPortGetSetCcParams.getCcParams.type = p_Manip->shadowUpdateParams;
        fmPortGetSetCcParams.getCcParams.poolIndex = p_Manip->fragParams.poolIndx;
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_RFENE;
        fmPortGetSetCcParams.setCcParams.nia = (NIA_FM_CTL_AC_FRAG_CHECK | NIA_ORDER_RESTOR);
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
        if(fmPortGetSetCcParams.getCcParams.type & NUM_OF_TASKS)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("NumOfTasks wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_DATA)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("offset of the data  wasn't configured previousely"));
        if(fmPortGetSetCcParams.getCcParams.type & HW_PORT_ID)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("hwPortId wasn't updated"));
    }

    if(p_Manip->updateParams)
    {
        if(p_Manip->updateParams & OFFSET_OF_DATA)
        {
            p_Manip->ipReassmParams.dataOffset = fmPortGetSetCcParams.getCcParams.dataOffset;
            if (p_Ipv4Ad != NULL)
            {
                tmpReg32 = GET_UINT32(p_Ipv4Ad->matchTblPtr);
                tmpReg32 |= (p_Manip->ipReassmParams.dataOffset << 16);
                WRITE_UINT32(p_Ipv4Ad->matchTblPtr, tmpReg32);
            }
            if (p_Ipv6Ad != NULL)
            {
                tmpReg32 = GET_UINT32(p_Ipv6Ad->matchTblPtr);
                tmpReg32 |= (p_Manip->ipReassmParams.dataOffset << 16);
                WRITE_UINT32(p_Ipv6Ad->matchTblPtr, tmpReg32);
            }

            p_Manip->updateParams &= ~OFFSET_OF_DATA;
            p_Manip->shadowUpdateParams |= OFFSET_OF_DATA;
        }

        p_Manip->updateParams &= ~HW_PORT_ID;
        p_Manip->shadowUpdateParams |= HW_PORT_ID;
    }
    else
    {
        if(validate)
        {
            /* TODO - Handle validate..*/
            /*if(fmPortGetSetCcParams.getCcParams.hardwarePortId != p_Manip->fragParams.hwPortId)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Reassembly manipulation previously was assigned to another port"));
            if(fmPortGetSetCcParams.getCcParams.numOfTasks != p_Manip->fragParams.numOfTasks)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("numOfTasks for this manipulation previously was defined by another value "));

            if(fmPortGetSetCcParams.getCcParams.poolIdForManip != p_Manip->fragParams.poolId)
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("poolId for this manipulation previously was defined by another value "));

            if(!(fmPortGetSetCcParams.getCcParams.type & OFFSET_OF_PR))
            {
                if(p_Manip->fragParams.prOffset != fmPortGetSetCcParams.getCcParams.prOffset)
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Parse result offset previously was defined by another value "));
            }
            else
            {
                if(p_Manip->fragParams.prOffset != 0xff)
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Parse result offset previously was defined by another value "));
            }
            if(fmPortGetSetCcParams.getCcParams.dataOffset != p_Manip->fragParams.dataOffset)
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Data offset previously was defined by another value "));*/

        }
    }

    return E_OK;
}

t_Error FmPcdFragHcScratchPoolFill(t_Handle h_FmPcd, uint8_t scratchBpid)
{
    t_FmPcd                             *p_FmPcd = (t_FmPcd*)h_FmPcd;
    t_FmPcdCcFragScratchPoolCmdParams   fmPcdCcFragScratchPoolCmdParams;
    t_Error                             err;

    SANITY_CHECK_RETURN_ERROR(p_FmPcd, E_INVALID_HANDLE);

    memset(&fmPcdCcFragScratchPoolCmdParams, 0, sizeof(t_FmPcdCcFragScratchPoolCmdParams));

    fmPcdCcFragScratchPoolCmdParams.numOfBuffers = NUM_OF_SCRATCH_POOL_BUFFERS;
    fmPcdCcFragScratchPoolCmdParams.bufferPoolId = scratchBpid;
    if ((err = FmHcPcdCcIpFragScratchPollCmd(p_FmPcd->h_Hc, TRUE, &fmPcdCcFragScratchPoolCmdParams)) != E_OK)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    if (fmPcdCcFragScratchPoolCmdParams.numOfBuffers != 0)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Fill scratch pool failed,"
                                              "Failed to release %d buffers to the BM (missing FBPRs)", fmPcdCcFragScratchPoolCmdParams.numOfBuffers));
    return E_OK;
}

t_Error FmPcdFragHcScratchPoolEmpty(t_Handle h_FmPcd, uint8_t scratchBpid)
{
    t_FmPcd                             *p_FmPcd = (t_FmPcd*)h_FmPcd;
    t_FmPcdCcFragScratchPoolCmdParams   fmPcdCcFragScratchPoolCmdParams;
    t_Error                             err;

    SANITY_CHECK_RETURN_ERROR(p_FmPcd, E_INVALID_HANDLE);

    memset(&fmPcdCcFragScratchPoolCmdParams, 0, sizeof(t_FmPcdCcFragScratchPoolCmdParams));

    fmPcdCcFragScratchPoolCmdParams.bufferPoolId = scratchBpid;
    if ((err = FmHcPcdCcIpFragScratchPollCmd(p_FmPcd->h_Hc, FALSE, &fmPcdCcFragScratchPoolCmdParams)) != E_OK)
        RETURN_ERROR(MAJOR, err, NO_MSG);

    if (fmPcdCcFragScratchPoolCmdParams.numOfBuffers != NUM_OF_SCRATCH_POOL_BUFFERS)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Empty scratch pool failed"));

    return E_OK;
}
#endif /*FM_IP_FRAG_N_REASSEM_SUPPORT*/

static void ReleaseManipHandler(t_FmPcdManip *p_Manip, t_FmPcd *p_FmPcd)
{
    if(p_Manip->h_Ad)
    {
        if(p_Manip->muramAllocate)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->h_Ad);
        else
            XX_Free(p_Manip->h_Ad);
        p_Manip->h_Ad = NULL;
    }
    if(p_Manip->p_Template)
    {
        FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->p_Template);
        p_Manip->p_Template = NULL;
    }
    if(p_Manip->h_Frag)
    {
        if(p_Manip->fragParams.p_AutoLearnHashTbl)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->fragParams.p_AutoLearnHashTbl);
        if(p_Manip->fragParams.p_ReassmFrmDescrPoolTbl)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->fragParams.p_ReassmFrmDescrPoolTbl);
        if(p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->fragParams.p_ReassmFrmDescrIndxPoolTbl);
        if(p_Manip->fragParams.p_TimeOutTbl)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->fragParams.p_TimeOutTbl);
        FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->h_Frag);

    }
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
    if (p_Manip->frag)
    {
        if (p_Manip->ipFragParams.p_Frag)
        {
            FmPcdFragHcScratchPoolEmpty((t_Handle)p_FmPcd, p_Manip->ipFragParams.scratchBpid);
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->ipFragParams.p_Frag);
        }
    }
    else if (p_Manip->reassm)
    {
        FmPcdUnregisterReassmPort(p_FmPcd, p_Manip->ipReassmParams.h_IpReassCommonParamsTbl);

        if(p_Manip->ipReassmParams.h_Ipv4AutoLearnHashTbl)
            XX_Free(p_Manip->ipReassmParams.h_Ipv4AutoLearnHashTbl);
        if(p_Manip->ipReassmParams.h_Ipv6AutoLearnHashTbl)
            XX_Free(p_Manip->ipReassmParams.h_Ipv6AutoLearnHashTbl);
        if(p_Manip->ipReassmParams.h_Ipv4AutoLearnSetLockTblPtr)
            XX_Free(p_Manip->ipReassmParams.h_Ipv4AutoLearnSetLockTblPtr);
        if(p_Manip->ipReassmParams.h_Ipv6AutoLearnSetLockTblPtr)
            XX_Free(p_Manip->ipReassmParams.h_Ipv6AutoLearnSetLockTblPtr);
        if(p_Manip->ipReassmParams.h_Ipv4ReassParamsTblPtr)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->ipReassmParams.h_Ipv4ReassParamsTblPtr);
        if(p_Manip->ipReassmParams.h_Ipv6ReassParamsTblPtr)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->ipReassmParams.h_Ipv6ReassParamsTblPtr);
        if(p_Manip->ipReassmParams.h_IpReassCommonParamsTbl)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->ipReassmParams.h_IpReassCommonParamsTbl);
        if(p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl)
            FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->ipReassmParams.h_ReassmFrmDescrIndxPoolTbl);
        if(p_Manip->ipReassmParams.h_ReassmFrmDescrPoolTbl)
            XX_Free(p_Manip->ipReassmParams.h_ReassmFrmDescrPoolTbl);

        if (p_Manip->ipReassmParams.h_Ipv4Scheme)
            FM_PCD_KgDeleteScheme(p_FmPcd, p_Manip->ipReassmParams.h_Ipv4Scheme);

        if (p_Manip->ipReassmParams.h_Ipv6Scheme)
            FM_PCD_KgDeleteScheme(p_FmPcd, p_Manip->ipReassmParams.h_Ipv4Scheme);
    }
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
    if(p_Manip->p_StatsTbl)
        FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->p_StatsTbl);
}

static t_Error CheckManipParamsAndSetType(t_FmPcdManip  *p_Manip, t_FmPcdManipParams *p_ManipParams)
{

    if(p_ManipParams->rmv)
    {
        switch(p_ManipParams->rmvParams.type)
        {
            case(e_FM_PCD_MANIP_RMV_FROM_START_OF_FRAME_INCLUDE_SPECIFIC_LOCATION):
                switch(p_ManipParams->rmvParams.rmvSpecificLocationParams.type)
                {
                    case(e_FM_PCD_MANIP_LOC_BY_HDR) :
                        switch(p_ManipParams->rmvParams.rmvSpecificLocationParams.manipByHdr.hdr)
                        {
                            case(HEADER_TYPE_CAPWAP_DTLS) :

                                p_Manip->type = HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST;
                                p_Manip->muramAllocate = TRUE;

                                if(p_ManipParams->insrt)
                                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("for  CAPWAP_DTLS_HDR remove can not be insrt manipualtion after"));

                                if(p_ManipParams->fragOrReasm)
                                {
                                    if(!p_ManipParams->fragOrReasmParams.frag)
                                    {
                                        switch(p_ManipParams->fragOrReasmParams.hdr)
                                        {
                                            case(HEADER_TYPE_CAPWAP):
                                                p_Manip->type = HMAN_OC_CAPWAP_REASSEMBLY;
                                                break;
                                            default:
                                                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("unsupported header for Reassembly"));
                                        }
                                    }
                                    else
                                        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("for this type of manipulation frag can not be TRUE"));
                                }
                            break;
                            default:
                                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("non valid net header of remove location"));

                        }
                        break;
                        default:
                            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("non valid type of remove location"));

                }
            break;
            case(e_FM_PCD_MANIP_RMV_INT_FRAME_HDR) :
                if(p_ManipParams->insrt || p_ManipParams->fragOrReasm)
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For the type of remove e_FM_PCD_MANIP_RMV_INT_FRAME_HDR the only valid option rmv = TRUE, insrt = FALSE, fragOrReasm = FALSE"));
                p_Manip->type = HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX;
                p_Manip->muramAllocate = FALSE;
            break;
            case(e_FM_PCD_MANIP_RMV_FROM_START_OF_FRAME_TILL_SPECIFIC_LOCATION) :
                if  (p_ManipParams->fragOrReasm ||
                    ((p_ManipParams->insrt) && p_ManipParams->insrtParams.type != e_FM_PCD_MANIP_INSRT_TO_START_OF_FRAME_INT_FRAME_HDR))
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("for the type of remove e_FM_PCD_MANIP_RMV_FROM_START_OF_FRAME_TILL_SPECIFIC_LOCATION the only allowed insertion type is e_FM_PCD_MANIP_INSRT_TO_START_OF_FRAME_INT_FRAME_HDR"));
                p_Manip->type = HMAN_OC_RMV_N_OR_INSRT_INT_FRM_HDR;
                p_Manip->muramAllocate = TRUE;
           break;
           default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("invalid type of remove manipulation"));
        }
    }
    else if(p_ManipParams->insrt)
    {
        switch(p_ManipParams->insrtParams.type)
        {
            case(e_FM_PCD_MANIP_INSRT_TO_START_OF_FRAME_TEMPLATE) :

                p_Manip->type = HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER;
                p_Manip->muramAllocate = FALSE;

                if(p_ManipParams->fragOrReasm)
                {
                    if(p_ManipParams->fragOrReasmParams.frag)
                    {
                           switch(p_ManipParams->fragOrReasmParams.hdr)
                           {
                                case(HEADER_TYPE_CAPWAP):
                                    p_Manip->type = HMAN_OC_CAPWAP_FRAGMENTATION;
                                    break;
                                break;
                                default:
                                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid header for fragmentation"));
                           }
                    }
                    else
                        RETURN_ERROR(MAJOR, E_INVALID_STATE,("can not reach this point"));
                }
            break;
            case(e_FM_PCD_MANIP_INSRT_TO_START_OF_FRAME_INT_FRAME_HDR) :
                if(p_ManipParams->fragOrReasm)
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For this type of insert can not be fragOrReasm = TRUE"));
                p_Manip->type = HMAN_OC_RMV_N_OR_INSRT_INT_FRM_HDR;
                p_Manip->muramAllocate = TRUE;
            break;
            default:
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("for only isert manipulation unsupported type"));
        }
    }
    else if(p_ManipParams->fragOrReasm)
    {
        if(p_ManipParams->fragOrReasmParams.frag)
        {
            switch(p_ManipParams->fragOrReasmParams.hdr)
             {
                 case(HEADER_TYPE_CAPWAP):
                     p_Manip->type = HMAN_OC_CAPWAP_FRAGMENTATION;
                     p_Manip->muramAllocate = FALSE;
                 break;
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
                 case(HEADER_TYPE_IPv4):
                 case(HEADER_TYPE_IPv6):
                     p_Manip->type = HMAN_OC_IP_FRAGMENTATION;
                     p_Manip->muramAllocate = TRUE;
                 break;
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
                 default:
                     RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Unsupported header for fragmentation"));
             }
        }
        else
        {
            switch (p_ManipParams->fragOrReasmParams.hdr)
            {
                case(HEADER_TYPE_CAPWAP):
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Reassembly has to be with additional operation - rmv = TRUE, type of remove - e_FM_PCD_MANIP_RMV_FROM_START_OF_FRAME_INCLUDE_SPECIFIC_LOCATION,type = e_FM_PCD_MANIP_LOC_BY_HDR, hdr = HEADER_TYPE_CAPWAP_DTLS"));
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
                case(HEADER_TYPE_IPv4):
                    p_Manip->type = HMAN_OC_IP_REASSEMBLY;
                    p_Manip->muramAllocate = TRUE;
                    p_Manip->ipReassmParams.hdr = HEADER_TYPE_IPv4;
                    break;
                case(HEADER_TYPE_IPv6):
                    p_Manip->type = HMAN_OC_IP_REASSEMBLY;
                    p_Manip->muramAllocate = TRUE;
                    p_Manip->ipReassmParams.hdr = HEADER_TYPE_IPv6;
                    break;
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
                default:
                     RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Unsupported header for reassembly"));

            }
        }

    }
    else
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("User didn't ask for any manipulation"));

    p_Manip->insrt = p_ManipParams->insrt;
    p_Manip->rmv   = p_ManipParams->rmv;

    return E_OK;
}
static t_Error UpdateIndxStats(  t_Handle                       h_FmPcd,
                                 t_Handle                       h_FmPort,
                                 t_FmPcdManip                   *p_Manip)
{
    t_FmPcd                 *p_FmPcd = (t_FmPcd *)h_FmPcd;
    uint32_t                tmpReg32 = 0;
    t_AdOfTypeContLookup    *p_Ad;
    t_FmPortGetSetCcParams  fmPortGetSetCcParams;
    t_Error                 err;

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);

    p_Ad    = (t_AdOfTypeContLookup *)p_Manip->h_Ad;
    if(p_Manip->h_FmPcd != h_FmPcd)
        RETURN_ERROR(MAJOR, E_INVALID_STATE,
                     ("handler of PCD previously was initiated by different value"));

    memset(&fmPortGetSetCcParams, 0, sizeof(t_FmPortGetSetCcParams));

    if(!p_Manip->p_StatsTbl)
    {

        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNDN;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_CC;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        tmpReg32 = GET_UINT32(p_Ad->ccAdBase);

        p_Manip->p_StatsTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                          (uint32_t)p_Manip->owner * FM_PCD_MANIP_INDEXED_STATS_ENTRY_SIZE,
                                          4);
        if(!p_Manip->p_StatsTbl)
            RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

        IOMemSet32(p_Manip->p_StatsTbl, 0,  (uint32_t)(p_Manip->owner * 4));

        tmpReg32 |= (uint32_t)(XX_VirtToPhys(p_Manip->p_StatsTbl) - p_FmPcd->physicalMuramBase);

        if(p_Manip->cnia)
            tmpReg32 |= FM_PCD_MANIP_INDEXED_STATS_CNIA;

        tmpReg32 |=  FM_PCD_MANIP_INDEXED_STATS_DPD;
        WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);

    }
    else
    {
        fmPortGetSetCcParams.setCcParams.type = UPDATE_NIA_PNDN;
        fmPortGetSetCcParams.setCcParams.nia = NIA_FM_CTL_AC_CC;
        err = FmPortGetSetCcParams(h_FmPort, &fmPortGetSetCcParams);
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);
    }
    return E_OK;
}

static t_Error FmPcdManipInitUpdate(t_Handle h_FmPcd, t_Handle h_PcdParams, t_Handle h_FmPort, t_Handle h_Manip, t_Handle h_Ad, bool validate, int level, t_Handle h_FmTree)
{
    t_FmPcdManip *p_Manip = (t_FmPcdManip *)h_Manip;
    t_Error      err = E_OK;
    SANITY_CHECK_RETURN_ERROR(h_Manip,E_INVALID_HANDLE);
    UNUSED(level);
    UNUSED(h_FmPcd);
    UNUSED(h_FmTree);
#ifndef FM_IP_FRAG_N_REASSEM_SUPPORT
    UNUSED(h_PcdParams);
#endif /* not FM_IP_FRAG_N_REASSEM_SUPPORT */

    switch(p_Manip->type)
    {
        case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
        err = UpdateInitMvIntFrameHeaderFromFrameToBufferPrefix(h_FmPort, p_Manip, h_Ad, validate);
        break;
#ifdef FM_CAPWAP_SUPPORT
        case(HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER):
            if(!p_Manip->h_Frag)
                break;
        case(HMAN_OC_CAPWAP_FRAGMENTATION):
            err = UpdateInitCapwapFragmentation(h_FmPort, p_Manip, h_Ad, validate, h_FmTree);
        break;
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
            if(p_Manip->h_Frag)
            {
                err = UpdateInitCapwapReasm(h_FmPcd, h_FmPort, p_Manip, h_Ad, validate);
            }
            break;
#endif /* FM_CAPWAP_SUPPORT */
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
        case(HMAN_OC_IP_REASSEMBLY):
            err = UpdateInitIpReasm(h_FmPcd, h_PcdParams, h_FmPort, p_Manip, h_Ad, validate);
            break;
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
        case(HMAN_OC_CAPWAP_INDEXED_STATS):
            err = UpdateIndxStats(h_FmPcd, h_FmPort, p_Manip);
            break;
        default:
            return E_OK;
    }
    return err;
}

static t_Error FmPcdManipModifyUpdate(t_Handle h_Manip, t_Handle h_Ad, bool validate, int level, t_Handle h_FmTree)
{

    t_FmPcdManip    *p_Manip = (t_FmPcdManip *)h_Manip;
    t_Error         err = E_OK;
    UNUSED(level);
    switch(p_Manip->type)
    {
        case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("modify node with this type of manipulation  is not suppported"));
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):

           if(p_Manip->h_Frag)
           {
               if(!(p_Manip->shadowUpdateParams & NUM_OF_TASKS) && !(p_Manip->shadowUpdateParams & BUFFER_POOL_ID_FOR_MANIP) &&
               !(p_Manip->shadowUpdateParams & OFFSET_OF_DATA) && !(p_Manip->shadowUpdateParams & OFFSET_OF_PR))
                    RETURN_ERROR(MAJOR, E_INVALID_STATE, ("modify node with this type of manipulation requires manipulation be updated previousely in SetPcd function"));
           }
           break;
#ifdef FM_CAPWAP_SUPPORT
        case(HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER):
            if(p_Manip->h_Frag)
            {
                err = UpdateModifyCapwapFragmenation(p_Manip, h_Ad, validate, h_FmTree);
            }
            break;
#endif /* FM_CAPWAP_SUPPORT */
        default:
            return E_OK;

    }
    return err;

}

static t_Error GetPrOffsetByHeaderOrField(t_FmPcdManipLocationParams *p_ManipParams, uint8_t *parseArrayOffset)
{
    e_NetHeaderType hdr         = p_ManipParams->manipByHdr.hdr;
    e_FmPcdHdrIndex hdrIndex    = p_ManipParams->manipByHdr.hdrIndex;
    bool            byField     = p_ManipParams->manipByHdr.byField;
    t_FmPcdFields   field;

    if(byField)
        field = p_ManipParams->manipByHdr.fullField;

    if(byField)
    {
        switch(hdr)
        {
            case(HEADER_TYPE_ETH):
                switch(field.eth)
                {
                    case(NET_HEADER_FIELD_ETH_TYPE):
                        *parseArrayOffset = CC_PC_PR_ETYPE_LAST_OFFSET;
                        break;
                    default:
                        RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Header manipulation of the type Ethernet with this field not supported"));
                }
                break;
            case(HEADER_TYPE_VLAN):
                switch(field.vlan)
                {
                    case(NET_HEADER_FIELD_VLAN_TCI) :
                        if((hdrIndex == e_FM_PCD_HDR_INDEX_NONE) || (hdrIndex == e_FM_PCD_HDR_INDEX_1))
                            *parseArrayOffset = CC_PC_PR_VLAN1_OFFSET;
                        else if(hdrIndex == e_FM_PCD_HDR_INDEX_LAST)
                             *parseArrayOffset = CC_PC_PR_VLAN2_OFFSET;
                        break;
                    default:
                       RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Header manipulation of the type VLAN with this field not supported"));
                }
                break;
           default:
               RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Header manipulation of this header by field not supported"));
        }
    }
    else
    {
        switch(hdr){
             case(HEADER_TYPE_ETH):
                 *parseArrayOffset = (uint8_t)CC_PC_PR_ETH_OFFSET;
                break;
            case(HEADER_TYPE_USER_DEFINED_SHIM1):
                *parseArrayOffset = (uint8_t)CC_PC_PR_USER_DEFINED_SHIM1_OFFSET;
                break;
            case(HEADER_TYPE_USER_DEFINED_SHIM2):
                *parseArrayOffset = (uint8_t)CC_PC_PR_USER_DEFINED_SHIM2_OFFSET;
                break;
            /* TODO - to take care about SHIM3
            case(HEADER_TYPE_USER_DEFINED_SHIM3):
                *parseArrayOffset = (uint8_t)CC_PC_PR_USER_DEFINED_SHIM3_OFFSET;
                break;
            */
            case(HEADER_TYPE_LLC_SNAP):
                *parseArrayOffset = CC_PC_PR_USER_LLC_SNAP_OFFSET;
                break;
            case(HEADER_TYPE_PPPoE):
                *parseArrayOffset = CC_PC_PR_PPPOE_OFFSET;
                break;
            case(HEADER_TYPE_MPLS):
                 if((hdrIndex == e_FM_PCD_HDR_INDEX_NONE) || (hdrIndex == e_FM_PCD_HDR_INDEX_1))
                        *parseArrayOffset = CC_PC_PR_MPLS1_OFFSET;
                else if(hdrIndex == e_FM_PCD_HDR_INDEX_LAST)
                        *parseArrayOffset = CC_PC_PR_MPLS_LAST_OFFSET;
                break;
            case(HEADER_TYPE_IPv4):
            case(HEADER_TYPE_IPv6):
              if((hdrIndex == e_FM_PCD_HDR_INDEX_NONE) || (hdrIndex == e_FM_PCD_HDR_INDEX_1))
                    *parseArrayOffset = CC_PC_PR_IP1_OFFSET;
              else if(hdrIndex == e_FM_PCD_HDR_INDEX_2)
                    *parseArrayOffset = CC_PC_PR_IP_LAST_OFFSET;
                break;
            case(HEADER_TYPE_MINENCAP):
                *parseArrayOffset = CC_PC_PR_MINENC_OFFSET;
                break;
            case(HEADER_TYPE_GRE):
                *parseArrayOffset = CC_PC_PR_GRE_OFFSET;
                break;
            case(HEADER_TYPE_TCP):
            case(HEADER_TYPE_UDP):
            case(HEADER_TYPE_IPSEC_AH):
            case(HEADER_TYPE_IPSEC_ESP):
            case(HEADER_TYPE_DCCP):
            case(HEADER_TYPE_SCTP):
                *parseArrayOffset = CC_PC_PR_L4_OFFSET;
                break;
            default:
                RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Header manipulation of this header is not supported"));
     }
    }
    return E_OK;
}

static t_Error RmvHdrTillSpecLocNOrInsrtIntFrmHdr(t_FmPcdManipRmvParams  *p_ManipParams, t_FmPcdManip *p_Manip)
{
    t_AdOfTypeContLookup    *p_Ad;
    uint32_t                tmpReg32 = 0;
    uint8_t                 prsArrayOffset = 0;
    t_Error                 err;

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_ManipParams,E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);

    p_Ad = (t_AdOfTypeContLookup *)p_Manip->h_Ad;
    if(p_Manip->rmv)
    {
        switch(p_ManipParams->rmvSpecificLocationParams.type)
        {
            case(e_FM_PCD_MANIP_LOC_BY_HDR) :
                err = GetPrOffsetByHeaderOrField(&p_ManipParams->rmvSpecificLocationParams, &prsArrayOffset);
                break;
            case(e_FM_PCD_MANIP_LOC_NON_HDR) :
                err = GetPrOffsetByNonHeader(&prsArrayOffset);
                break;
            default :
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Invalid type of location header manipulation of type Remove"));
        }
        if(err)
            RETURN_ERROR(MAJOR, err, NO_MSG);

        tmpReg32 |= (uint32_t)prsArrayOffset << 24;
        tmpReg32 |= HMAN_RMV_HDR;
    }

    if(p_Manip->insrt)
        tmpReg32 |= HMAN_INSRT_INT_FRM_HDR;

    tmpReg32 |= (uint32_t)HMAN_OC_RMV_N_OR_INSRT_INT_FRM_HDR;

    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);

    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);

    return E_OK;
}

static t_Error MvIntFrameHeaderFromFrameToBufferPrefix(t_FmPcdManip *p_Manip, bool caamUsed)
{
    t_AdOfTypeContLookup    *p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Ad;
    uint32_t                tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Ad,E_INVALID_HANDLE);

    p_Manip->updateParams |= OFFSET_OF_PR | INTERNAL_CONTEXT_OFFSET;

    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    *(uint32_t *)&p_Ad->ccAdBase = tmpReg32;

    /*TODO - update offsetInBufferPrefixForIntFrameHdr when port connected to tree
    tmpReg32 = 0;
    tmpReg32 |= offsetInBufferPrefixForIntFrameHdr;
    *(uint32_t *)&p_Ad->matchTblPtr = tmpReg32;*/

    tmpReg32 = 0;
    tmpReg32 |= HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX;
    tmpReg32 |= (uint32_t)0x16 << 16;
    *(uint32_t *)&p_Ad->pcAndOffsets = tmpReg32;

    if (caamUsed)
        *(uint32_t *)&p_Ad->gmask = 0xf0000000;

    return E_OK;
}

#ifdef FM_CAPWAP_SUPPORT
static t_Error CapwapRmvDtlsHdr(t_FmPcd *p_FmPcd, t_FmPcdManip *p_Manip)
{
    t_AdOfTypeContLookup    *p_Ad;
    uint32_t                tmpReg32 = 0;
    t_Error                 err = E_OK;

    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);

    p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Ad;

    tmpReg32 = 0;
    tmpReg32 |= (uint32_t)HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST;
    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);

    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;


    if(p_Manip->h_Frag)
    {
        p_Manip->updateParams |= INTERNAL_CONTEXT_OFFSET;
        tmpReg32 |= (uint32_t)(XX_VirtToPhys(p_Manip->h_Frag) - (p_FmPcd->physicalMuramBase));
    }

    WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);

    return err;
}

static t_Error CapwapReassembly(t_CapwapReassemblyParams *p_ManipParams,t_FmPcdManip *p_Manip,t_FmPcd *p_FmPcd, uint8_t poolIndex)
{
    t_Handle    p_Table;
    uint32_t    tmpReg32 = 0;
    int         i = 0;
    uint8_t     log2Num;
    uint8_t     numOfSets;
    uint32_t    j = 0;

    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPcd->h_Hc,E_INVALID_HANDLE);

    if(!p_FmPcd->h_Hc)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,("hc port has to be initialized in this mode"));
    if (!POWER_OF_2(p_ManipParams->timeoutRoutineRequestTime))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("timeoutRoutineRequestTime has to be power of 2"));
    if(!POWER_OF_2(p_ManipParams->maxNumFramesInProcess))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,("maxNumFramesInProcess has to be power of 2"));
    if(!p_ManipParams->timeoutRoutineRequestTime && p_ManipParams->timeoutThresholdForReassmProcess)
        DBG(WARNING, ("if timeoutRoutineRequestTime 0,  timeoutThresholdForReassmProcess is uselessly"));
    if(p_ManipParams->numOfFramesPerHashEntry == e_FM_PCD_MANIP_FOUR_WAYS_HASH)
    {
        if((p_ManipParams->maxNumFramesInProcess < 4) ||
            (p_ManipParams->maxNumFramesInProcess > 512))
            RETURN_ERROR(MAJOR,E_INVALID_VALUE, ("In the case of numOfFramesPerHashEntry = e_FM_PCD_MANIP_EIGHT_WAYS_HASH maxNumFramesInProcess has to be in the range 4-512"));
    }
    else
    {
        if((p_ManipParams->maxNumFramesInProcess < 8) ||
            (p_ManipParams->maxNumFramesInProcess > 2048))
            RETURN_ERROR(MAJOR,E_INVALID_VALUE, ("In the case of numOfFramesPerHashEntry = e_FM_PCD_MANIP_FOUR_WAYS_HASH maxNumFramesInProcess has to be in the range 8-2048"));
    }

    p_Manip->updateParams |= (NUM_OF_TASKS | BUFFER_POOL_ID_FOR_MANIP | OFFSET_OF_PR | OFFSET_OF_DATA | HW_PORT_ID);

    p_Manip->h_Frag = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                          FM_PCD_MANIP_CAPWAP_REASM_TABLE_SIZE,
                                          FM_PCD_MANIP_CAPWAP_REASM_TABLE_ALIGN);
    if(!p_Manip->h_Frag)
         RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

    IOMemSet32(p_Manip->h_Frag, 0,  FM_PCD_MANIP_CAPWAP_REASM_TABLE_SIZE);

    p_Table         = (t_CapwapReasmPram *)p_Manip->h_Frag;

    p_Manip->fragParams.p_AutoLearnHashTbl = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                          (uint32_t)(p_ManipParams->maxNumFramesInProcess * 2 * FM_PCD_MANIP_CAPWAP_REASM_AUTO_LEARNING_HASH_ENTRY_SIZE),
                                          FM_PCD_MANIP_CAPWAP_REASM_TABLE_ALIGN);

    IOMemSet32(p_Manip->fragParams.p_AutoLearnHashTbl, 0,  (uint32_t)(p_ManipParams->maxNumFramesInProcess * 2 * FM_PCD_MANIP_CAPWAP_REASM_AUTO_LEARNING_HASH_ENTRY_SIZE));


    tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->fragParams.p_AutoLearnHashTbl) - p_FmPcd->physicalMuramBase);

    WRITE_UINT32(((t_CapwapReasmPram *)p_Table)->autoLearnHashTblPtr, tmpReg32);

    tmpReg32 = 0;
    if(p_ManipParams->timeOutMode == e_FM_PCD_MANIP_TIME_OUT_BETWEEN_FRAMES)
        tmpReg32 |= FM_PCD_MANIP_CAPWAP_REASM_TIME_OUT_BETWEEN_FRAMES;
    if(p_ManipParams->haltOnDuplicationFrag)
        tmpReg32  |= FM_PCD_MANIP_CAPWAP_REASM_HALT_ON_DUPLICATE_FRAG;
    if(p_ManipParams->numOfFramesPerHashEntry == e_FM_PCD_MANIP_EIGHT_WAYS_HASH)
    {
        i = 8;
        tmpReg32  |= FM_PCD_MANIP_CAPWAP_REASM_AUTOMATIC_LEARNIN_HASH_8_WAYS;
    }
    else
        i = 4;

    numOfSets = (uint8_t)((p_ManipParams->maxNumFramesInProcess * 2) / i);
    LOG2(numOfSets, log2Num);
    tmpReg32 |= (uint32_t)(log2Num - 1) << 24;

    WRITE_UINT32(((t_CapwapReasmPram *)p_Table)->mode, tmpReg32);

    for(j = 0; j < p_ManipParams->maxNumFramesInProcess * 2; j++)
    {
        if(((j / i)  % 2)== 0)
        {
            WRITE_UINT32(*(uint32_t *)PTR_MOVE(p_Manip->fragParams.p_AutoLearnHashTbl, j * FM_PCD_MANIP_CAPWAP_REASM_AUTO_LEARNING_HASH_ENTRY_SIZE), 0x80000000);
        }
    }

    WRITE_UINT32(((t_CapwapReasmPram *)p_Table)->bufferPoolIdAndRisc1SetIndexes, 0x00008000);
    WRITE_UINT32(((t_CapwapReasmPram *)p_Table)->risc23SetIndexes, 0x80008000);
    WRITE_UINT32(((t_CapwapReasmPram *)p_Table)->risc4SetIndexesAndExtendedStatsTblPtr, 0x80000000);

    p_Manip->fragParams.maxNumFramesInProcess = p_ManipParams->maxNumFramesInProcess;

    p_Manip->fragParams.poolIndx =  poolIndex;

    p_Manip->fragParams.fqidForTimeOutFrames = p_ManipParams->fqidForTimeOutFrames;
    p_Manip->fragParams.timeoutRoutineRequestTime = p_ManipParams->timeoutRoutineRequestTime;
    /*TODO  - to take care about this function FmGetTimeStampScale - it return t_Error
     now we have problems with all calls to this fucntion*/
    p_Manip->fragParams.bitFor1Micro = FmGetTimeStampScale(p_FmPcd->h_Fm);

    tmpReg32 = 0;
    tmpReg32 |= (((uint32_t)1<<p_Manip->fragParams.bitFor1Micro) * p_ManipParams->timeoutThresholdForReassmProcess);
    WRITE_UINT32(((t_CapwapReasmPram *)p_Table)->expirationDelay, tmpReg32);

    return E_OK;

}

static t_Error CapwapFragmentation(t_CapwapFragmentationParams *p_ManipParams,t_FmPcdManip *p_Manip,t_FmPcd *p_FmPcd, uint8_t poolIndex)
{
    t_AdOfTypeContLookup    *p_Ad;
    uint32_t                tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);

    p_Manip->updateParams |= OFFSET_OF_DATA | BUFFER_POOL_ID_FOR_MANIP;

    p_Manip->frag = TRUE;

    p_Manip->h_Frag = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                          FM_PCD_CC_AD_ENTRY_SIZE,
                                          FM_PCD_CC_AD_TABLE_ALIGN);
    if(!p_Manip->h_Frag)
         RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

    IOMemSet32(p_Manip->h_Frag, 0,  FM_PCD_CC_AD_ENTRY_SIZE);

    p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Frag;

    tmpReg32 = 0;
    tmpReg32 |= (uint32_t)HMAN_OC_CAPWAP_FRAGMENTATION;

    if(p_ManipParams->headerOptionsCompr)
        tmpReg32 = FM_PCD_MANIP_CAPWAP_FRAG_COMPR_OPTION_FIELD_EN;
    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);

    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);


    p_Manip->sizeForFragmentation = p_ManipParams->sizeForFragmentation;
    p_Manip->fragParams.poolIndx = poolIndex;

    return E_OK;
}
#endif /* FM_CAPWAP_SUPPORT */

#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
static t_Error IpFragmentation(t_IpFragmentationParams *p_ManipParams,t_FmPcdManip *p_Manip, t_FmPcd *p_FmPcd)
{
    t_AdOfTypeContLookup    *p_Ad;
    uint32_t                tmpReg32 = 0;
    t_Error                 err = E_OK;

    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPcd->h_Hc,E_INVALID_HANDLE);

    /* Host Command module must be initialized when using IP Fragmentation manipulation */
    if (!p_FmPcd->h_Hc)
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("hc port has to be initialized in this mode"));

    /* Allocation of fragmentation Action Descriptor */
    p_Manip->ipFragParams.p_Frag = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                                               FM_PCD_CC_AD_ENTRY_SIZE,
                                                               FM_PCD_CC_AD_TABLE_ALIGN);

    if(!p_Manip->ipFragParams.p_Frag)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

    IOMemSet32( p_Manip->ipFragParams.p_Frag, 0,  FM_PCD_CC_AD_ENTRY_SIZE);

    p_Ad = (t_AdOfTypeContLookup *)p_Manip->ipFragParams.p_Frag;

    /* Sets the third Ad register (pcAndOffsets)- OperationCode */
    tmpReg32 = 0;
    tmpReg32 |= (uint32_t)HMAN_OC_IP_FRAGMENTATION;
    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);

    /* Sets the first Ad register (ccAdBase) - Don't frag action and Action descriptor type*/
    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    tmpReg32 |= (p_ManipParams->dontFragAction << FM_PCD_MANIP_IP_FRAG_DF_OFFSET);
    WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);

    /* Saves user's fragmentation manipulation parameters */
    p_Manip->ipFragParams.scratchBpid = p_ManipParams->scratchBpid;
    p_Manip->sizeForFragmentation = p_ManipParams->sizeForFragmentation;
    p_Manip->frag = TRUE;

    /* Sets the first Ad register (gmask) - scratch buffer pool id and Pointer to fragment ID */
    tmpReg32 = (uint32_t)(XX_VirtToPhys(p_FmPcd->h_FragIdPtr) - p_FmPcd->physicalMuramBase);
    tmpReg32 |= p_ManipParams->scratchBpid << FM_PCD_MANIP_IP_FRAG_SCRATCH_BPID;
    WRITE_UINT32(p_Ad->gmask, tmpReg32);

    /* scratch buffer pool initialization */
    if ((err = FmPcdFragHcScratchPoolFill((t_Handle)p_FmPcd, p_ManipParams->scratchBpid)) != E_OK)
    {
        FM_MURAM_FreeMem(p_FmPcd->h_FmMuram, p_Manip->ipFragParams.p_Frag);
        p_Manip->ipFragParams.p_Frag = NULL;
        RETURN_ERROR(MAJOR, err, NO_MSG);
    }
    return E_OK;
}

static t_Error FillReassmManipParams(t_FmPcdManip *p_Manip, t_Handle h_Ad, bool ipv4)
{
    t_AdOfTypeContLookup *p_Ad;
    t_FmPcd              *p_FmPcd   = (t_FmPcd *)p_Manip->h_FmPcd;
    uint32_t             tmpReg32;
    t_Error              err = E_OK;
    t_Handle             h_IpReassParamsTblPtr;

    /* Gets the required Action descriptor table pointer */
    if (ipv4)
        p_Ad = (t_AdOfTypeContLookup *)p_Manip->ipReassmParams.h_Ipv4Ad;
    else
        p_Ad = (t_AdOfTypeContLookup *)p_Manip->ipReassmParams.h_Ipv6Ad;

    /* Sets the first Ad register (ccAdBase) - Action Descriptor Type and Pointer to the IP Reassembly Parameters Table offset from MURAM*/
    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;

    /* Creates the IP Reassembly Parameters table. It contains parameters that are specific to either the IPv4 reassembly
     function or to the IPv6 reassembly function. If both IPv4 reassembly and IPv6 reassembly are required, then
     two separate IP Reassembly Parameter tables are required.*/
    if (ipv4)
    {
        p_Manip->ipReassmParams.h_Ipv4ReassParamsTblPtr = CreateIpReassParamTable(p_Manip, ipv4);;
        h_IpReassParamsTblPtr = p_Manip->ipReassmParams.h_Ipv4ReassParamsTblPtr;
    }
    else
    {
        p_Manip->ipReassmParams.h_Ipv6ReassParamsTblPtr = CreateIpReassParamTable(p_Manip, ipv4);
        h_IpReassParamsTblPtr = p_Manip->ipReassmParams.h_Ipv6ReassParamsTblPtr;
    }

    /* Gets the IP Reassemly parameter table offset from MURAM */
    if (h_IpReassParamsTblPtr != NULL)
        tmpReg32 |= (uint32_t)(XX_VirtToPhys(h_IpReassParamsTblPtr) - (p_FmPcd->physicalMuramBase));
    else
        return err;

    WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);

    /* Sets the second Ad register (matchTblPtr) - Buffer pool ID (BPID) and Scatter/Gather table offset*/
    /* mark the Scatter/Gather table offset to be set later on when the port will be known */
    p_Manip->updateParams = OFFSET_OF_DATA;

    tmpReg32 = (uint32_t)(p_Manip->ipReassmParams.bpid << 8);
    WRITE_UINT32(p_Ad->matchTblPtr, tmpReg32);

    /* Sets the third Ad register (pcAndOffsets)- liodn offset and IP Reassembly Operation Code*/
    tmpReg32 = 0;
    tmpReg32 |= (uint32_t)HMAN_OC_IP_REASSEMBLY;
    tmpReg32 |= (uint32_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_LIODN_MASK) << ((uint32_t)FM_PCD_MANIP_IP_REASM_LIODN_SHIFT-32);
    tmpReg32 |= (uint32_t)(p_Manip->ipReassmParams.liodnOffset & FM_PCD_MANIP_IP_REASM_ELIODN_MASK) << ((uint32_t)FM_PCD_MANIP_IP_REASM_ELIODN_SHIFT-32);
    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);

    p_Manip->reassm = TRUE;

    return E_OK;
}

static t_Error SetIpv4ReassmManip(t_FmPcdManip *p_Manip)
{
    t_FmPcd *p_FmPcd = (t_FmPcd *)p_Manip->h_FmPcd;

    /* Allocation if IPv4 Action descriptor */
    if(p_Manip->muramAllocate)
    {
        p_Manip->ipReassmParams.h_Ipv4Ad = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                             FM_PCD_CC_AD_ENTRY_SIZE,
                                             FM_PCD_CC_AD_TABLE_ALIGN);
        if(!p_Manip->ipReassmParams.h_Ipv4Ad)
        {
           ReleaseManipHandler(p_Manip, p_FmPcd);
           RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
        }

        IOMemSet32(p_Manip->ipReassmParams.h_Ipv4Ad, 0,  FM_PCD_CC_AD_ENTRY_SIZE);
    }
    else
    {
        p_Manip->ipReassmParams.h_Ipv4Ad = (t_Handle)XX_MallocSmart(FM_PCD_CC_AD_ENTRY_SIZE * sizeof(uint8_t), p_Manip->ipReassmParams.dataMemId, 0);
        if(!p_Manip->ipReassmParams.h_Ipv4Ad)
        {
            ReleaseManipHandler(p_Manip, p_FmPcd);
            RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
        }

        memset(p_Manip->ipReassmParams.h_Ipv4Ad, 0,  FM_PCD_CC_AD_ENTRY_SIZE * sizeof(uint8_t));
    }

    /* Fill reassembly manipulation parameter in the IP Reassembly Action Descriptor */
    FillReassmManipParams(p_Manip, p_Manip->ipReassmParams.h_Ipv4Ad, TRUE);

    return E_OK;
}

static t_Error SetIpv6ReassmManip(t_FmPcdManip *p_Manip)
{
    t_FmPcd *p_FmPcd = (t_FmPcd *)p_Manip->h_FmPcd;

    /* Allocation if IPv6 Action descriptor */
    if(p_Manip->muramAllocate)
    {
        p_Manip->ipReassmParams.h_Ipv6Ad = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                             FM_PCD_CC_AD_ENTRY_SIZE,
                                             FM_PCD_CC_AD_TABLE_ALIGN);
        if(!p_Manip->ipReassmParams.h_Ipv6Ad)
        {
           ReleaseManipHandler(p_Manip, p_FmPcd);
           RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
        }

        IOMemSet32(p_Manip->ipReassmParams.h_Ipv6Ad, 0,  FM_PCD_CC_AD_ENTRY_SIZE);
    }
    else
    {
         p_Manip->ipReassmParams.h_Ipv6Ad = (t_Handle)XX_MallocSmart(FM_PCD_CC_AD_ENTRY_SIZE * sizeof(uint8_t), p_Manip->ipReassmParams.dataMemId, 0);
         if(!p_Manip->ipReassmParams.h_Ipv6Ad)
         {
            ReleaseManipHandler(p_Manip, p_FmPcd);
            RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
         }

        memset(p_Manip->ipReassmParams.h_Ipv6Ad, 0,  FM_PCD_CC_AD_ENTRY_SIZE * sizeof(uint8_t));
    }

    /* Fill reassembly manipulation parameter in the IP Reassembly Action Descriptor */
    FillReassmManipParams(p_Manip, p_Manip->ipReassmParams.h_Ipv6Ad, FALSE);

    return E_OK;
}


static t_Error IpReassembly(t_FmPcdManipFragOrReasmParams *p_ManipParams,t_FmPcdManip *p_Manip, t_FmPcd *p_FmPcd)
{
    uint32_t                    maxSetNumber = 10000;
    t_IpReasmCommonTbl          *p_IpReasmCommonPramTbl = NULL;
    t_IpReassemblyParams        reassmManipParams = p_ManipParams->ipReasmParams;
    t_Error                     res;

    SANITY_CHECK_RETURN_ERROR(p_FmPcd->h_Hc,E_INVALID_HANDLE);

    /* Check validation of user's parameter.*/
    if ((reassmManipParams.timeoutThresholdForReassmProcess < 1000) || (reassmManipParams.timeoutThresholdForReassmProcess > 8000000))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE,("timeoutThresholdForReassmProcess should be 1msec - 8sec"));
    /* It is recommended that the total number of entries in this table (number of sets * number of ways)
       will be twice the number of frames that are expected to be reassembled simultaneously.*/
    if (reassmManipParams.maxNumFramesInProcess > (reassmManipParams.maxNumFramesInProcess * maxSetNumber / 2))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("maxNumFramesInProcess has to be less than (maximun set number * number of ways / 2)"));

    if ((p_ManipParams->hdr == HEADER_TYPE_IPv6) &&
        (reassmManipParams.minFragSize[1] < 256))
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("minFragSize[1] must be >= 256"));

    /* Saves user's reassembly manipulation parameters */
    p_Manip->ipReassmParams.relativeSchemeId[0] = p_ManipParams->ipReasmParams.relativeSchemeId[0];
    p_Manip->ipReassmParams.relativeSchemeId[1] = p_ManipParams->ipReasmParams.relativeSchemeId[1];
    p_Manip->ipReassmParams.maxNumFramesInProcess = reassmManipParams.maxNumFramesInProcess;
    p_Manip->ipReassmParams.timeOutMode = reassmManipParams.timeOutMode;
    p_Manip->ipReassmParams.fqidForTimeOutFrames = reassmManipParams.fqidForTimeOutFrames;
    p_Manip->ipReassmParams.numOfFramesPerHashEntry = reassmManipParams.numOfFramesPerHashEntry;
    p_Manip->ipReassmParams.timeoutThresholdForReassmProcess = reassmManipParams.timeoutThresholdForReassmProcess;
    p_Manip->ipReassmParams.liodnOffset = reassmManipParams.liodnOffset;
    p_Manip->ipReassmParams.minFragSize[0] = reassmManipParams.minFragSize[0];
    p_Manip->ipReassmParams.minFragSize[1] = reassmManipParams.minFragSize[1];
    p_Manip->ipReassmParams.dataMemId = reassmManipParams.dataMemId;
    p_Manip->ipReassmParams.bpid = p_ManipParams->extBufPoolIndx;

    /* Creates and initializes the IP Reassembly common parameter table */
    CreateIpReassCommonParamTable(p_Manip, p_FmPcd, p_IpReasmCommonPramTbl);

    /* Creation of IPv4 reassembly manipulation */
    if ((p_Manip->ipReassmParams.hdr == HEADER_TYPE_IPv6) || (p_Manip->ipReassmParams.hdr == HEADER_TYPE_IPv4))
    {
        res = SetIpv4ReassmManip(p_Manip);
        if (res != E_OK)
            return res;
    }

    /* Creation of IPv6 reassembly manipulation */
    if (p_Manip->ipReassmParams.hdr == HEADER_TYPE_IPv6)
    {
        res = SetIpv6ReassmManip(p_Manip);
        if (res != E_OK)
            return res;
    }

    return E_OK;
}

bool FmPcdManipIsIpv4Present(t_FmPcd *p_FmPcd, uint8_t netEnvId)
{
    uint8_t         res;

    ASSERT_COND(p_FmPcd);

    res = FmPcdNetEnvGetUnitId(p_FmPcd, netEnvId, HEADER_TYPE_IPv4, FALSE, IPV4_FRAG_1);
    if (res == HEADER_TYPE_USER_DEFINED_SHIM2)
        return TRUE;
    else
        return FALSE;
}

bool FmPcdManipIsIpv6Present(t_FmPcd *p_FmPcd, uint8_t netEnvId)
{
    uint8_t         res;

    ASSERT_COND(p_FmPcd);

    res = FmPcdNetEnvGetUnitId(p_FmPcd, netEnvId, HEADER_TYPE_IPv6, FALSE, IPV6_FRAG_1);
    if (res == HEADER_TYPE_USER_DEFINED_SHIM2)
        return TRUE;
    else
        return FALSE;
}

#endif /*FM_IP_FRAG_N_REASSEM_SUPPORT*/

static t_Error IndxStats(t_FmPcdStatsParams *p_StatsParams,t_FmPcdManip *p_Manip,t_FmPcd *p_FmPcd)
{
    t_AdOfTypeContLookup    *p_Ad;
    uint32_t                tmpReg32 = 0;

    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);

    UNUSED(p_FmPcd);

    p_Ad         = (t_AdOfTypeContLookup *)p_Manip->h_Ad;

    tmpReg32 = 0;
    tmpReg32 |= (uint32_t)HMAN_OC_CAPWAP_INDEXED_STATS;
    if(p_StatsParams->type == e_FM_PCD_STATS_PER_FLOWID)
        tmpReg32 |= (uint32_t)0x16 << 16;
    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);

    tmpReg32 = 0;
    tmpReg32 |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    WRITE_UINT32(p_Ad->ccAdBase, tmpReg32);

    return E_OK;
}

#ifdef FM_CAPWAP_SUPPORT
static t_Error InsrtHdrByTempl(t_FmPcdManipInsrtParams  *p_ManipParams, t_FmPcdManip *p_Manip, t_FmPcd *p_FmPcd)
{
    t_FmPcdManipInsrtByTemplateParams   *p_InsrtByTemplate = &p_ManipParams->insrtByTemplateParams;
    uint8_t                             tmpReg8 = 0xff;
    t_AdOfTypeContLookup                *p_Ad;
    bool                                ipModify = FALSE;
    uint32_t                            tmpReg32 = 0, tmpRegNia = 0;
    uint16_t                            tmpReg16 = 0;
    t_Error                             err = E_OK;
    uint8_t                             extraAddedBytes = 0, blockSize = 0, extraAddedBytesAlignedToBlockSize = 0;
    uint8_t                             *p_Template = NULL;

    SANITY_CHECK_RETURN_ERROR(p_ManipParams,E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Manip,E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_Manip->h_Ad,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPcd,E_NULL_POINTER);

    p_Ad = (t_AdOfTypeContLookup *)p_Manip->h_Ad;
    if(p_Manip->insrt)
    {
        if((!p_InsrtByTemplate->size && p_InsrtByTemplate->modifyOuterIp) ||
             (!p_InsrtByTemplate->size && p_InsrtByTemplate->modifyOuterVlan))
             RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistent parameters : asking for header template modifications with no template for insertion (template size)"));

         if (p_InsrtByTemplate->size && p_InsrtByTemplate->modifyOuterIp && (p_InsrtByTemplate->size <= p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset))
             RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistent parameters : size of template < ipOuterOffset"));

         if(p_InsrtByTemplate->size > 128)
             RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Size of header template for insertion can not be more than 128"));

         if(p_InsrtByTemplate->size)
         {
             p_Manip->p_Template = (uint8_t *)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                                p_InsrtByTemplate->size,
                                                FM_PCD_CC_AD_TABLE_ALIGN);
             if(!p_Manip->p_Template)
                 RETURN_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));

             tmpReg32 = (uint32_t)(XX_VirtToPhys(p_Manip->p_Template) - (p_FmPcd->physicalMuramBase));
             tmpReg32 |= (uint32_t)p_InsrtByTemplate->size << 24;
             *(uint32_t *)&p_Ad->matchTblPtr = tmpReg32;
         }

         tmpReg32 = 0;

        p_Template = (uint8_t *)XX_Malloc(p_InsrtByTemplate->size * sizeof(uint8_t));

        if(!p_Template)
            RETURN_ERROR(MAJOR, E_NO_MEMORY, ("XX_Malloc allocation FAILED"));

        memcpy(p_Template, p_InsrtByTemplate->hdrTemplate, p_InsrtByTemplate->size * sizeof(uint8_t));


         if(p_InsrtByTemplate->modifyOuterIp)
         {
             ipModify = TRUE;

             tmpReg8 = (uint8_t)p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset];

             if((tmpReg8 & 0xf0) == 0x40)
                 tmpReg8 = 4;
             else if((tmpReg8 & 0xf0) == 0x60)
                 tmpReg8 = 6;
             else
                 tmpReg8 = 0xff;

             if(tmpReg8 == 4)
             {
                 if((IP_HDRCHECKSUM_FIELD_OFFSET_FROM_IP + p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset) > p_InsrtByTemplate->size)
                     RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistent parameters : IP present in header template, user asked for IP modifications but ipOffset + ipTotalLengthFieldOffset in header template bigger than template size"));

                 if(p_InsrtByTemplate->modifyOuterIpParams.dscpEcn & 0xff00)
                     RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistent parameters : IPV4 present in header template, dscpEcn has to be only 1 byte"));

                 p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_DSCECN_FIELD_OFFSET_FROM_IP] = (uint8_t)p_InsrtByTemplate->modifyOuterIpParams.dscpEcn;

                 if(p_InsrtByTemplate->modifyOuterIpParams.recalculateLength)
                 {

                     if((p_InsrtByTemplate->modifyOuterIpParams.recalculateLengthParams.extraBytesAddedAlignedToBlockSize + p_InsrtByTemplate->modifyOuterIpParams.recalculateLengthParams.extraBytesAddedNotAlignedToBlockSize) > 255)
                            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("extra Byte added can not be more than 256 bytes"));
                     extraAddedBytes = (uint8_t) (p_InsrtByTemplate->modifyOuterIpParams.recalculateLengthParams.extraBytesAddedAlignedToBlockSize + p_InsrtByTemplate->modifyOuterIpParams.recalculateLengthParams.extraBytesAddedNotAlignedToBlockSize);
                     blockSize = p_InsrtByTemplate->modifyOuterIpParams.recalculateLengthParams.blockSize;
                     extraAddedBytesAlignedToBlockSize = p_InsrtByTemplate->modifyOuterIpParams.recalculateLengthParams.extraBytesAddedAlignedToBlockSize;
                     /*IP header template - IP totalLength -
                     (1 byte) extraByteForIp = headerTemplateSize - ipOffset + insertedBytesAfterThisStage ,
                     in the case of SEC insertedBytesAfterThisStage - SEC trailer (21/31) + header(13)
                     second byte - extraByteForIp = headerTemplate - ipOffset + insertedBytesAfterThisStage*/
                 }
                 if(blockSize)
                 {
                     if (!POWER_OF_2(blockSize))
                         RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("inputFrmPaddingUpToBlockSize has to be power of 2"));
                     blockSize -= 1;
                 }

                 if((p_InsrtByTemplate->size - p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + extraAddedBytes) > 255)
                     RETURN_ERROR(MAJOR, E_INVALID_STATE, ("p_InsrtByTemplate->size - p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + extraAddedBytes has to be less than 255"));

                p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_TOTALLENGTH_FIELD_OFFSET_FROM_IP + 1] = blockSize;
                p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_TOTALLENGTH_FIELD_OFFSET_FROM_IP] = (uint8_t)(p_InsrtByTemplate->size - p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + extraAddedBytes);

                p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_ID_FIELD_OFFSET_FROM_IP] = 0x00;
                p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_ID_FIELD_OFFSET_FROM_IP + 1] = extraAddedBytesAlignedToBlockSize;


                 /*IP header template - relevant only for ipv4 CheckSum = 0*/
                 p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_HDRCHECKSUM_FIELD_OFFSET_FROM_IP] = 0x00;
                 p_Template[p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset + IP_HDRCHECKSUM_FIELD_OFFSET_FROM_IP + 1] = 0x00;


                 /*UDP checksum has to be 0*/
                 if(p_InsrtByTemplate->modifyOuterIpParams.udpPresent)
                 {
                     if((p_InsrtByTemplate->modifyOuterIpParams.udpOffset + UDP_UDPHECKSUM_FIELD_OFFSET_FROM_UDP + UDP_UDPCHECKSUM_FIELD_SIZE) > p_InsrtByTemplate->size)
                         RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Inconsistent parameters : UDP present according to user but (UDP offset + UDP header size) < size of header template"));

                    p_Template[p_InsrtByTemplate->modifyOuterIpParams.udpOffset + UDP_UDPHECKSUM_FIELD_OFFSET_FROM_UDP ] = 0x00;
                    p_Template[p_InsrtByTemplate->modifyOuterIpParams.udpOffset + UDP_UDPHECKSUM_FIELD_OFFSET_FROM_UDP + 1] = 0x00;

                 }

                 if(p_InsrtByTemplate->modifyOuterIpParams.ipIdentGenId > 7)
                     RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("ipIdentGenId has to be one out of 8 sequence number generators (0 - 7) for IP identification field"));

                 tmpRegNia |= (uint32_t)p_InsrtByTemplate->modifyOuterIpParams.ipIdentGenId<<24;
             }
             else
                 RETURN_ERROR(MAJOR, E_INVALID_STATE, ("IP version supported only IPV4"));
         }

         tmpReg32 = tmpReg16 = tmpReg8 = 0;
         /*TODO - check it*/
         if(p_InsrtByTemplate->modifyOuterVlan)
         {
             if(p_InsrtByTemplate->modifyOuterVlanParams.vpri & ~0x07)
                 RETURN_ERROR(MAJOR, E_INVALID_STATE,("Inconsistent parameters : user asked for VLAN modifications but VPRI more than 3 bits"));

             memcpy(&tmpReg16, &p_Template[VLAN_TAG_FIELD_OFFSET_FROM_ETH], 2*(sizeof(uint8_t)));
             if((tmpReg16  != 0x9100) && (tmpReg16!= 0x9200) && (tmpReg16 != 0x8100))
                 RETURN_ERROR(MAJOR, E_INVALID_STATE,("Inconsistent parameters : user asked for VLAN modifications but Tag Protocol identifier is not VLAN "));

             memcpy(&tmpReg8, &p_Template[14],1*(sizeof(uint8_t)));
             tmpReg8 &= 0x1f;
             tmpReg8 |= (uint8_t)(p_InsrtByTemplate->modifyOuterVlanParams.vpri << 5);

             p_Template[14] = tmpReg8;
         }

        Mem2IOCpy32(p_Manip->p_Template, p_Template, p_InsrtByTemplate->size);

        XX_Free(p_Template);
    }

    tmpReg32 = 0;
    if(p_Manip->h_Frag)
    {
        tmpRegNia |= (uint32_t)(XX_VirtToPhys(p_Manip->h_Frag) - (p_FmPcd->physicalMuramBase));
        tmpReg32 |= (uint32_t)p_Manip->sizeForFragmentation << 16;
    }
    else
          tmpReg32 = 0xffff0000;

    if(ipModify)
        tmpReg32 |= (uint32_t)p_InsrtByTemplate->modifyOuterIpParams.ipOuterOffset << 8;
    else
        tmpReg32 |= (uint32_t)0x0000ff00;

    tmpReg32 |= (uint32_t)HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER;
    *(uint32_t *)&p_Ad->pcAndOffsets = tmpReg32;

    tmpRegNia |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    *(uint32_t *)&p_Ad->ccAdBase = tmpRegNia;

    return err;
}
#endif /* FM_CAPWAP_SUPPORT */

#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
static t_Error IPManip(t_FmPcdManipParams *p_ManipParams, t_FmPcdManip *p_Manip, t_FmPcd *p_FmPcd)
{

    t_Error                     err = E_OK;
    t_AdOfTypeContLookup        *p_Ad;
    uint32_t                    tmpReg32 = 0, tmpRegNia = 0;

    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_FmPcd,E_INVALID_HANDLE);

    p_Ad = (t_AdOfTypeContLookup *)p_Manip->h_Ad;

    if(p_Manip->frag == TRUE)
    {
        tmpRegNia = (uint32_t)(XX_VirtToPhys(p_Manip->ipFragParams.p_Frag) - (p_FmPcd->physicalMuramBase));
        tmpReg32  = (uint32_t)p_Manip->sizeForFragmentation << FM_PCD_MANIP_IP_FRAG_MTU_OFFSET;
    }
    else
          tmpReg32 = FM_PCD_MANIP_IP_FRAG_NO_FRAGMETATION;

    tmpRegNia |= FM_PCD_AD_CONT_LOOKUP_TYPE;
    tmpReg32  |= HMAN_OC_IP_MANIP;

    WRITE_UINT32(p_Ad->pcAndOffsets, tmpReg32);
    WRITE_UINT32(p_Ad->ccAdBase, tmpRegNia);
    WRITE_UINT32(p_Ad->gmask, 0); /* Total frame counter - MUST be initialized to zero.*/

/*
    TODO - Fill the following:
         - Over write OuterTos
         - SaveInnerTos
         - support in CNIA
*/

    return err;
}
#endif /* (defined(FM_IP_FRAG_N_REASSEM_SUPPORT) || ... */

static t_Error CheckStatsParamsAndSetType(t_FmPcdManip  *p_Manip, t_FmPcdStatsParams *p_StatsParams)
{

    switch(p_StatsParams->type)
    {
        case(e_FM_PCD_STATS_PER_FLOWID):
            p_Manip->type = HMAN_OC_CAPWAP_INDEXED_STATS;
            p_Manip->muramAllocate = TRUE;
        break;
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE, ("Unsupported statistics type"));
    }

    return E_OK;
}

static t_Handle ManipOrStatsSetNode(t_Handle h_FmPcd, t_Handle *p_Params, bool stats)
{
    t_FmPcdManip                *p_Manip;
    t_Error                     err;
    t_FmPcd                     *p_FmPcd = (t_FmPcd *)h_FmPcd;

    p_Manip = (t_FmPcdManip*)XX_Malloc(sizeof(t_FmPcdManip));
    if(!p_Manip)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("No memory"));
        return NULL;
    }
    memset(p_Manip, 0, sizeof(t_FmPcdManip));

    if(!stats)
    {
        err = CheckManipParamsAndSetType(p_Manip, (t_FmPcdManipParams *)p_Params);
    }
    else
    {
        err = CheckStatsParamsAndSetType(p_Manip, (t_FmPcdStatsParams *)p_Params);
    }

    if(err)
    {
        REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("INVALID HEADER MANIPULATION TYPE"));
        ReleaseManipHandler(p_Manip, p_FmPcd);
        XX_Free(p_Manip);
        return NULL;
    }

#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
    if(p_Manip->type != HMAN_OC_IP_REASSEMBLY)
    {
        /* In Case of IP reassembly manipulation the IPv4/IPv6 reassembly action descriptor will
           be defines later on */
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
    if(p_Manip->muramAllocate)
    {
        p_Manip->h_Ad = (t_Handle)FM_MURAM_AllocMem(p_FmPcd->h_FmMuram,
                                             FM_PCD_CC_AD_ENTRY_SIZE,
                                             FM_PCD_CC_AD_TABLE_ALIGN);
         if(!p_Manip->h_Ad)
         {
            REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
            ReleaseManipHandler(p_Manip, p_FmPcd);
            XX_Free(p_Manip);
            return NULL;
         }

        IOMemSet32(p_Manip->h_Ad, 0,  FM_PCD_CC_AD_ENTRY_SIZE);
    }
    else
    {
        p_Manip->h_Ad = (t_Handle)XX_Malloc(FM_PCD_CC_AD_ENTRY_SIZE * sizeof(uint8_t));
         if(!p_Manip->h_Ad)
         {
            REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Memory allocation in MURAM FAILED"));
            ReleaseManipHandler(p_Manip, p_FmPcd);
            XX_Free(p_Manip);
            return NULL;
         }

        memset(p_Manip->h_Ad, 0,  FM_PCD_CC_AD_ENTRY_SIZE * sizeof(uint8_t));
    }
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
    }
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
    p_Manip->h_FmPcd = h_FmPcd;

    return p_Manip;
}


t_Error FmPcdManipUpdate(t_Handle h_FmPcd, t_Handle h_PcdParams, t_Handle h_FmPort, t_Handle h_Manip, t_Handle h_Ad, bool validate, int level, t_Handle h_FmTree, bool modify)
{
    t_Error err;

    if(!modify)
    {
        err = FmPcdManipInitUpdate(h_FmPcd, h_PcdParams, h_FmPort, h_Manip, h_Ad, validate, level, h_FmTree);
    }
    else
    {
        err = FmPcdManipModifyUpdate(h_Manip, h_Ad, validate, level, h_FmTree);
    }
    return err;
}

uint32_t FmPcdManipGetRequiredAction (t_Handle h_Manip)
{
    t_FmPcdManip *p_Manip = (t_FmPcdManip *)h_Manip;

    ASSERT_COND(h_Manip);

    switch(p_Manip->type)
    {
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
        case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
            return UPDATE_NIA_ENQ_WITHOUT_DMA;
        default:
            return 0;
    }
}

void FmPcdManipUpdateOwner(t_Handle h_Manip, bool add)
{

    if(add)
        ((t_FmPcdManip *)h_Manip)->owner++;
    else
    {
        ASSERT_COND(((t_FmPcdManip *)h_Manip)->owner);
        ((t_FmPcdManip *)h_Manip)->owner--;
    }
}

t_Error FmPcdManipCheckParamsForCcNextEgine(t_FmPcdCcNextEngineParams *p_FmPcdCcNextEngineParams, uint32_t *requiredAction)
{
    t_FmPcdManip             *p_Manip;
    t_Error                   err;


    SANITY_CHECK_RETURN_ERROR(p_FmPcdCcNextEngineParams, E_NULL_POINTER);
    SANITY_CHECK_RETURN_ERROR(p_FmPcdCcNextEngineParams->h_Manip, E_NULL_POINTER);

    p_Manip = (t_FmPcdManip *)(p_FmPcdCcNextEngineParams->h_Manip);
    *requiredAction = 0;
    switch(p_Manip->type)
    {
        case(HMAN_OC_CAPWAP_INDEXED_STATS):
            if(p_FmPcdCcNextEngineParams->nextEngine != e_FM_PCD_DONE)
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For this type of header manipulation has to be nextEngine e_FM_PCD_DONE"));
            if(p_FmPcdCcNextEngineParams->params.enqueueParams.overrideFqid)
               p_Manip->cnia = TRUE;
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
            *requiredAction = UPDATE_NIA_ENQ_WITHOUT_DMA;
        case(HMAN_OC_RMV_N_OR_INSRT_INT_FRM_HDR):
            p_Manip->ownerTmp++;
        break;
        case(HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER):
            if((p_FmPcdCcNextEngineParams->nextEngine != e_FM_PCD_DONE) && !p_FmPcdCcNextEngineParams->params.enqueueParams.overrideFqid)
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For this type of header manipulation has to be nextEngine e_FM_PCD_DONE with fqidForCtrlFlow FALSE"));
            p_Manip->ownerTmp++;
        break;
        case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
            if((p_FmPcdCcNextEngineParams->nextEngine != e_FM_PCD_CC)  &&
               (FmPcdCcGetParseCode(p_FmPcdCcNextEngineParams->params.ccParams.h_CcNode) != CC_PC_GENERIC_IC_HASH_INDEXED))
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For this type of header manipulation next engine has to be CC and action = e_FM_PCD_ACTION_INDEXED_LOOKUP"));
            err = UpdateManipIc(p_FmPcdCcNextEngineParams->h_Manip, FmPcdCcGetOffset(p_FmPcdCcNextEngineParams->params.ccParams.h_CcNode));
            if(err)
                RETURN_ERROR(MAJOR, err, NO_MSG);
            *requiredAction = UPDATE_NIA_ENQ_WITHOUT_DMA;
        break;
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
        case(HMAN_OC_IP_MANIP):

            if((p_FmPcdCcNextEngineParams->nextEngine == e_FM_PCD_DONE) &&
               !p_FmPcdCcNextEngineParams->params.enqueueParams.overrideFqid)
               p_Manip->cnia = FALSE;
            else
               p_Manip->cnia = TRUE;
            if(!p_Manip->h_Frag)
            {
                p_Manip->ownerTmp++;
                break;
            }
        case(HMAN_OC_IP_FRAGMENTATION):
            if(p_FmPcdCcNextEngineParams->nextEngine != e_FM_PCD_DONE)
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For this type of header manipulation has to be nextEngine e_FM_PCD_DONE"));
            p_Manip->ownerTmp++;
        break;
        case(HMAN_OC_IP_REASSEMBLY):
            if(p_FmPcdCcNextEngineParams->nextEngine != e_FM_PCD_DONE)
                RETURN_ERROR(MAJOR, E_INVALID_STATE, ("For this type of header manipulation has to be nextEngine e_FM_PCD_DONE"));
            p_Manip->ownerTmp++;
        break;
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
        default:
            RETURN_ERROR(MAJOR, E_INVALID_STATE,("invalid type of header manipulation for this state"));
    }

    return E_OK;
}

t_Error FmPcdManipCheckParamsWithCcNodeParams(t_Handle h_Manip, t_Handle h_FmPcdCcNode)
{
    t_FmPcdManip *p_Manip = (t_FmPcdManip *)h_Manip;
    t_Error         err = E_OK;

    SANITY_CHECK_RETURN_ERROR(h_Manip, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(h_FmPcdCcNode, E_INVALID_HANDLE);

    switch(p_Manip->type)
    {
        case(HMAN_OC_CAPWAP_INDEXED_STATS):
            if(p_Manip->ownerTmp != FmPcdCcGetNumOfKeys(h_FmPcdCcNode))
                RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("The manipulation of the type statistics flowId if exist has to be pointed by all numOfKeys"));
        break;
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
            if(p_Manip->h_Frag)
            {
                if(p_Manip->ownerTmp != FmPcdCcGetNumOfKeys(h_FmPcdCcNode))
                    RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("The manipulation of the type remove DTLS if exist has to be pointed by all numOfKeys"));
                err = UpdateManipIc(h_Manip, FmPcdCcGetOffset(h_FmPcdCcNode));
                if(err)
                    RETURN_ERROR(MAJOR, err, NO_MSG);
            }
            break;
        default:
            break;
    }
    return err;
}

void FmPcdManipUpdateAdResultForCc(t_Handle h_Manip, t_Handle p_Ad, t_Handle *p_AdNew)
{
    t_FmPcdManip             *p_Manip = (t_FmPcdManip *)h_Manip;

    ASSERT_COND(p_Manip);

    FmPcdManipUpdateOwner(h_Manip, TRUE);

    switch(p_Manip->type)
    {
        case(HMAN_OC_RMV_N_OR_INSRT_INT_FRM_HDR):
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
        case(HMAN_OC_CAPWAP_INDEXED_STATS):
            *p_AdNew = p_Manip->h_Ad;
            break;
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
        case(HMAN_OC_IP_FRAGMENTATION):
            *p_AdNew = p_Manip->h_Ad;
            break;
        case(HMAN_OC_IP_REASSEMBLY):
            if (p_Manip->ipReassmParams.hdr == HEADER_TYPE_IPv4)
            {
                *p_AdNew = p_Manip->ipReassmParams.h_Ipv4Ad;
            }
            if (p_Manip->ipReassmParams.hdr == HEADER_TYPE_IPv6)
            {
                if (!p_Manip->ipReassmParams.ipv6Assigned)
                {
                    *p_AdNew = p_Manip->ipReassmParams.h_Ipv6Ad;
                    p_Manip->ipReassmParams.ipv6Assigned = TRUE;
                }
                else
                    *p_AdNew = p_Manip->ipReassmParams.h_Ipv4Ad;
            }
            break;
        case(HMAN_OC_IP_MANIP):
            if (p_Manip->cnia)
                *p_AdNew = p_Manip->h_Ad;
            else
            {
                WRITE_UINT32(((t_AdOfTypeResult *)p_Ad)->fqid,         ((t_AdOfTypeResult *)(p_Manip->h_Ad))->fqid);
                WRITE_UINT32(((t_AdOfTypeResult *)p_Ad)->plcrProfile,  ((t_AdOfTypeResult *)(p_Manip->h_Ad))->plcrProfile);
                WRITE_UINT32(((t_AdOfTypeResult *)p_Ad)->nia,          ((t_AdOfTypeResult *)(p_Manip->h_Ad))->nia);
                *p_AdNew = NULL;
            }
            break;
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
        case(HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER):
        case(HMAN_OC_CAPWAP_FRAGMENTATION):
            WRITE_UINT32(((t_AdOfTypeResult *)p_Ad)->fqid,         ((t_AdOfTypeResult *)(p_Manip->h_Ad))->fqid);
            WRITE_UINT32(((t_AdOfTypeResult *)p_Ad)->plcrProfile,  ((t_AdOfTypeResult *)(p_Manip->h_Ad))->plcrProfile);
            WRITE_UINT32(((t_AdOfTypeResult *)p_Ad)->nia,          ((t_AdOfTypeResult *)(p_Manip->h_Ad))->nia);
            *p_AdNew = NULL;
            break;
        default:
            break;
    }
}

void FmPcdManipUpdateAdContLookupForCc(t_Handle h_Manip, t_Handle p_Ad, t_Handle *p_AdNew, uint32_t adTableOffset)
{
    t_FmPcdManip             *p_Manip = (t_FmPcdManip *)h_Manip;

    ASSERT_COND(p_Manip);

    FmPcdManipUpdateOwner(h_Manip, TRUE);

    switch(p_Manip->type)
    {
        case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
            WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->ccAdBase,      ((t_AdOfTypeContLookup *)(p_Manip->h_Ad))->ccAdBase);
            WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->matchTblPtr,   ((t_AdOfTypeContLookup *)(p_Manip->h_Ad))->matchTblPtr);
            WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->pcAndOffsets,  ((t_AdOfTypeContLookup *)(p_Manip->h_Ad))->pcAndOffsets);
            WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->gmask,         ((t_AdOfTypeContLookup *)(p_Manip->h_Ad))->gmask);
            WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->ccAdBase,      (GET_UINT32(((t_AdOfTypeContLookup *)p_Ad)->ccAdBase) | adTableOffset));
            *p_AdNew = NULL;
            break;
        default:
            break;
    }
}
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
void setReassmSchemeParams(t_FmPcd* p_FmPcd, t_FmPcdKgSchemeParams *p_Scheme, t_Handle h_CcTree, bool ipv4, uint32_t groupId)
{
    uint32_t    j;
    uint8_t     res;

    /* Configures scheme's network environment parameters */
    p_Scheme->netEnvParams.numOfDistinctionUnits = 2;
    if (ipv4)
        res = FmPcdNetEnvGetUnitId(p_FmPcd, FmPcdGetNetEnvId(p_FmPcd, p_Scheme->netEnvParams.h_NetEnv), HEADER_TYPE_IPv4, FALSE, 0);
    else
        res = FmPcdNetEnvGetUnitId(p_FmPcd, FmPcdGetNetEnvId(p_FmPcd, p_Scheme->netEnvParams.h_NetEnv), HEADER_TYPE_IPv6, FALSE, 0);
    ASSERT_COND(res != FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS);
    p_Scheme->netEnvParams.unitIds[0] = res;

    res = FmPcdNetEnvGetUnitId(p_FmPcd, FmPcdGetNetEnvId(p_FmPcd, p_Scheme->netEnvParams.h_NetEnv), HEADER_TYPE_USER_DEFINED_SHIM2, FALSE, 0);
    ASSERT_COND(res != FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS);
    p_Scheme->netEnvParams.unitIds[1] = res;

    /* Configures scheme's next engine parameters*/
    p_Scheme->nextEngine = e_FM_PCD_CC;
    p_Scheme->kgNextEngineParams.cc.h_CcTree = h_CcTree;
    p_Scheme->kgNextEngineParams.cc.grpId = groupId;
    p_Scheme->useHash = TRUE;

    /* Configures scheme's key*/
    if (ipv4 == TRUE)
    {
        p_Scheme->keyExtractAndHashParams.numOfUsedExtracts = 4;
        p_Scheme->keyExtractAndHashParams.extractArray[0].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[0].extractByHdr.type = e_FM_PCD_EXTRACT_FULL_FIELD;
        p_Scheme->keyExtractAndHashParams.extractArray[0].extractByHdr.hdr = HEADER_TYPE_IPv4 ;
        p_Scheme->keyExtractAndHashParams.extractArray[0].extractByHdr.extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_DST_IP;
        p_Scheme->keyExtractAndHashParams.extractArray[1].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[1].extractByHdr.type = e_FM_PCD_EXTRACT_FULL_FIELD;
        p_Scheme->keyExtractAndHashParams.extractArray[1].extractByHdr.hdr = HEADER_TYPE_IPv4;
        p_Scheme->keyExtractAndHashParams.extractArray[1].extractByHdr.extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_SRC_IP;
        p_Scheme->keyExtractAndHashParams.extractArray[2].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.type = e_FM_PCD_EXTRACT_FULL_FIELD;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.hdr = HEADER_TYPE_IPv4;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_PROTO;
        p_Scheme->keyExtractAndHashParams.extractArray[3].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[3].extractByHdr.hdr = HEADER_TYPE_IPv4;
        p_Scheme->keyExtractAndHashParams.extractArray[3].extractByHdr.type = e_FM_PCD_EXTRACT_FROM_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[3].extractByHdr.ignoreProtocolValidation = FALSE;
        p_Scheme->keyExtractAndHashParams.extractArray[3].extractByHdr.extractByHdrType.fromHdr.size = 2;
        p_Scheme->keyExtractAndHashParams.extractArray[3].extractByHdr.extractByHdrType.fromHdr.offset = 4;
    }
    else /* IPv6 */
    {
        p_Scheme->keyExtractAndHashParams.numOfUsedExtracts = 3;
        p_Scheme->keyExtractAndHashParams.extractArray[0].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[0].extractByHdr.type = e_FM_PCD_EXTRACT_FULL_FIELD;
        p_Scheme->keyExtractAndHashParams.extractArray[0].extractByHdr.hdr = HEADER_TYPE_IPv6 ;
        p_Scheme->keyExtractAndHashParams.extractArray[0].extractByHdr.extractByHdrType.fullField.ipv6 = NET_HEADER_FIELD_IPv6_DST_IP;
        p_Scheme->keyExtractAndHashParams.extractArray[1].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[1].extractByHdr.type = e_FM_PCD_EXTRACT_FULL_FIELD;
        p_Scheme->keyExtractAndHashParams.extractArray[1].extractByHdr.hdr = HEADER_TYPE_IPv6;
        p_Scheme->keyExtractAndHashParams.extractArray[1].extractByHdr.extractByHdrType.fullField.ipv6 = NET_HEADER_FIELD_IPv6_SRC_IP;
        p_Scheme->keyExtractAndHashParams.extractArray[2].type = e_FM_PCD_EXTRACT_BY_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.hdr = HEADER_TYPE_USER_DEFINED_SHIM2;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.type = e_FM_PCD_EXTRACT_FROM_HDR;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.extractByHdrType.fromHdr.size  = 4;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.extractByHdrType.fromHdr.offset = 4;
        p_Scheme->keyExtractAndHashParams.extractArray[2].extractByHdr.ignoreProtocolValidation = TRUE;
    }

    p_Scheme->keyExtractAndHashParams.privateDflt0 = 0x01020304;
    p_Scheme->keyExtractAndHashParams.privateDflt1 = 0x11121314;
    p_Scheme->keyExtractAndHashParams.numOfUsedDflts = FM_PCD_KG_NUM_OF_DEFAULT_GROUPS;
    for(j=0; j<FM_PCD_KG_NUM_OF_DEFAULT_GROUPS; j++)
    {
        p_Scheme->keyExtractAndHashParams.dflts[j].type = (e_FmPcdKgKnownFieldsDfltTypes)j; /* all types */
        p_Scheme->keyExtractAndHashParams.dflts[j].dfltSelect = e_FM_PCD_KG_DFLT_GBL_0;
    }

    return;
}

t_Error FmPcdManipBuildIpReassmScheme(t_FmPcd *p_FmPcd, t_FmPcdCcTreeParams *p_PcdGroupsParam, t_Handle h_CcTree, t_Handle h_Manip, bool isIpv4, uint32_t groupId)
{
    t_FmPcdManip            *p_Manip = (t_FmPcdManip *)h_Manip;
    t_FmPcdKgSchemeParams   *p_scheme;

    ASSERT_COND(p_FmPcd);
    ASSERT_COND(p_PcdGroupsParam);
    ASSERT_COND(p_Manip);

    p_scheme = XX_Malloc(sizeof(t_FmPcdKgSchemeParams));

    if(!p_scheme)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, ("XX_Malloc allocation FAILED"));

    /* Configures the IPv4 or IPv6 scheme*/
    memset(p_scheme, 0, sizeof(*p_scheme));
    p_scheme->netEnvParams.h_NetEnv = p_PcdGroupsParam->h_NetEnv;
    p_scheme->id.relativeSchemeId = (isIpv4 == TRUE) ?  p_Manip->ipReassmParams.relativeSchemeId[0] : p_Manip->ipReassmParams.relativeSchemeId[1];
    p_scheme->schemeCounter.update = TRUE;
    p_scheme->baseFqid = 0xFFFFFF; /*TODO- baseFqid*/
    p_scheme->keyExtractAndHashParams.hashDistributionNumOfFqids = 1;

    setReassmSchemeParams(p_FmPcd, p_scheme, h_CcTree, isIpv4, groupId);

    /* Sets the new scheme */
    if (isIpv4)
        p_Manip->ipReassmParams.h_Ipv4Scheme = FM_PCD_KgSetScheme(p_FmPcd, p_scheme);
    else
        p_Manip->ipReassmParams.h_Ipv6Scheme = FM_PCD_KgSetScheme(p_FmPcd, p_scheme);

    XX_Free(p_scheme);

    return E_OK;
}
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */

t_Handle FM_PCD_ManipSetNode(t_Handle h_FmPcd, t_FmPcdManipParams *p_ManipParams)
{
    t_FmPcd                     *p_FmPcd = (t_FmPcd *)h_FmPcd;
    t_FmPcdManip                *p_Manip;
    t_Error                     err;

    SANITY_CHECK_RETURN_VALUE(h_FmPcd,E_INVALID_HANDLE,NULL);
    SANITY_CHECK_RETURN_VALUE(p_ManipParams,E_INVALID_HANDLE,NULL);

    p_Manip =  ManipOrStatsSetNode(h_FmPcd, (t_Handle)p_ManipParams, FALSE);
    if(!p_Manip)
        return NULL;

    switch(p_Manip->type)
    {
        case(HMAN_OC_RMV_N_OR_INSRT_INT_FRM_HDR):
            /* HmanType1 */
            err = RmvHdrTillSpecLocNOrInsrtIntFrmHdr(&p_ManipParams->rmvParams, p_Manip);
            break;
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
        case(HMAN_OC_IP_REASSEMBLY):
            /* IpReassembly */
            err = IpReassembly(&p_ManipParams->fragOrReasmParams, p_Manip, p_FmPcd);
            if(err)
            {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("UNSUPPORTED HEADER MANIPULATION TYPE"));
                ReleaseManipHandler(p_Manip, p_FmPcd);
                XX_Free(p_Manip);
                return NULL;
            }
            break;
       case(HMAN_OC_IP_FRAGMENTATION):
            /* IpFragmentation */
            err = IpFragmentation(&p_ManipParams->fragOrReasmParams.ipFragParams ,p_Manip, p_FmPcd);
            if(err)
            {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("UNSUPPORTED HEADER MANIPULATION TYPE"));
                ReleaseManipHandler(p_Manip, p_FmPcd);
                XX_Free(p_Manip);
                return NULL;
            }
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
#ifdef FM_IP_FRAG_N_REASSEM_SUPPORT
        case(HMAN_OC_IP_MANIP) :
            err = IPManip(p_ManipParams, p_Manip, p_FmPcd);
            break;
#endif /* FM_IP_FRAG_N_REASSEM_SUPPORT */
#ifdef FM_CAPWAP_SUPPORT
        case(HMAN_OC_CAPWAP_FRAGMENTATION):
            /* CapwapFragmentation */
            err = CapwapFragmentation(&p_ManipParams->fragOrReasmParams.capwapFragParams ,p_Manip, p_FmPcd, p_ManipParams->fragOrReasmParams.extBufPoolIndx);
            if(err)
            {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("UNSUPPORTED HEADER MANIPULATION TYPE"));
                ReleaseManipHandler(p_Manip, p_FmPcd);
                XX_Free(p_Manip);
                return NULL;
            }
            if(p_Manip->insrt)
                p_Manip->type = HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER;
        case(HMAN_OC_INSRT_HDR_BY_TEMPL_N_OR_FRAG_AFTER):
            /* HmanType2 + if user asked only for fragmentation still need to allocate HmanType2 */
            err = InsrtHdrByTempl(&p_ManipParams->insrtParams, p_Manip, p_FmPcd);
            break;
        case(HMAN_OC_CAPWAP_REASSEMBLY) :
            /*CAPWAP Reassembly*/
            err = CapwapReassembly(&p_ManipParams->fragOrReasmParams.capwapReasmParams,p_Manip, p_FmPcd, p_ManipParams->fragOrReasmParams.extBufPoolIndx);
            if(err)
            {
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("UNSUPPORTED HEADER MANIPULATION TYPE"));
                ReleaseManipHandler(p_Manip, p_FmPcd);
                XX_Free(p_Manip);
                return NULL;
            }
            if(p_Manip->rmv)
                p_Manip->type = HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST;
        case(HMAN_OC_CAPWAP_RMV_DTLS_IF_EXIST):
            /*CAPWAP decapsulation + if user asked only for reassembly still need to allocate CAPWAP decapsulation*/
            err = CapwapRmvDtlsHdr(p_FmPcd, p_Manip);
            break;
#endif /* FM_CAPWAP_SUPPORT */
       case(HMAN_OC_MV_INT_FRAME_HDR_FROM_FRM_TO_BUFFER_PREFFIX):
            /*Application Specific type 1*/
            err = MvIntFrameHeaderFromFrameToBufferPrefix(p_Manip, (bool)(p_ManipParams->treatFdStatusFieldsAsErrors ? TRUE : FALSE));
            break;
       default:
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("UNSUPPORTED HEADER MANIPULATION TYPE"));
                ReleaseManipHandler(p_Manip, p_FmPcd);
                XX_Free(p_Manip);
                return NULL;
    }
    if(err)
     {
         REPORT_ERROR(MAJOR, err, NO_MSG);
         ReleaseManipHandler(p_Manip, p_FmPcd);
         XX_Free(p_Manip);
         return NULL;
     }
     return p_Manip;
}

t_Error FM_PCD_ManipDeleteNode(t_Handle h_FmPcd, t_Handle h_ManipNode)
{
    t_FmPcd                     *p_FmPcd = (t_FmPcd *)h_FmPcd;
    t_FmPcdManip                *p_Manip = (t_FmPcdManip *)h_ManipNode;

    SANITY_CHECK_RETURN_ERROR(p_FmPcd, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_Manip,E_INVALID_HANDLE);

    if(p_Manip->owner)
        RETURN_ERROR(MAJOR, E_INVALID_STATE, ("This manipulation node not be removed because this node is occupied, first - unbind this node "));

    ReleaseManipHandler(p_Manip,p_FmPcd);

    XX_Free(h_ManipNode);

    return E_OK;
}


t_Handle FM_PCD_StatisticsSetNode(t_Handle h_FmPcd, t_FmPcdStatsParams *p_StatsParams)
{
    t_FmPcd                     *p_FmPcd = (t_FmPcd *)h_FmPcd;
    t_FmPcdManip                *p_Manip;
    t_Error                     err;

    SANITY_CHECK_RETURN_VALUE(h_FmPcd,E_INVALID_HANDLE,NULL);
    SANITY_CHECK_RETURN_VALUE(p_StatsParams,E_INVALID_HANDLE,NULL);

    p_Manip =  ManipOrStatsSetNode(h_FmPcd, (t_Handle)p_StatsParams, TRUE);
    if(!p_Manip)
        return NULL;

     switch(p_Manip->type)
    {
        case(HMAN_OC_CAPWAP_INDEXED_STATS):
            /* Indexed statistics */
            err = IndxStats(p_StatsParams, p_Manip, p_FmPcd);
            break;
       default:
                REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("UNSUPPORTED Statistics type"));
                ReleaseManipHandler(p_Manip, p_FmPcd);
                XX_Free(p_Manip);
                return NULL;

    }
     if(err)
     {
         REPORT_ERROR(MAJOR, err, NO_MSG);
         ReleaseManipHandler(p_Manip, p_FmPcd);
         XX_Free(p_Manip);
         return NULL;
     }
     return p_Manip;
}
