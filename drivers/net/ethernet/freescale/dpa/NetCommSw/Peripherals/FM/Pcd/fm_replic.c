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
 @File          fm_replic.c

 @Description   FM frame replicator
*//***************************************************************************/
#include "std_ext.h"
#include "error_ext.h"
#include "string_ext.h"
#include "debug_ext.h"
#include "fm_pcd_ext.h"
#include "fm_muram_ext.h"
#include "fm_common.h"
#include "fm_hc.h"
#include "fm_replic.h"
#include "fm_cc.h"
#include "list_ext.h"


/****************************************/
/*       static functions               */
/****************************************/

static t_Error FrmReplicGroupTryLock(t_Handle h_ReplicGroup)
{
    t_FmPcdFrmReplicGroup *p_ReplicGroup = (t_FmPcdFrmReplicGroup *)h_ReplicGroup;

    ASSERT_COND(h_ReplicGroup);

    if (FmPcdLockTryLock(p_ReplicGroup->p_Lock))
        return E_OK;

    return ERROR_CODE(E_BUSY);
}

static void FrmReplicGroupReleaseLock(t_Handle h_ReplicGroup)
{
    t_FmPcdFrmReplicGroup *p_ReplicGroup = (t_FmPcdFrmReplicGroup *)h_ReplicGroup;

    ASSERT_COND(h_ReplicGroup);

    FmPcdLockUnlock(p_ReplicGroup->p_Lock);
}

static void  UpdateGroupOwner(t_FmPcdFrmReplicGroup *p_ReplicGroup, bool add)
{
    if (add)
        p_ReplicGroup->owners++;
    else
    {
        ASSERT_COND(p_ReplicGroup->owners);
        p_ReplicGroup->owners--;
    }
}


static e_UseSourceTdOption  CheckUseSourceTdOptionType(t_FmPcdFrmReplicGroup *p_ReplicGroup)
{
    uint32_t                numOfOwnersInList = 0, numOfSourceTdOwners = 0;
    e_UseSourceTdOption     useSourceTdOption = e_USE_SOURCE_TD_NONE;
    t_List                  *p_Pos;
    t_CcNodeInformation     *p_ReplicGroupOwner;

    /* Check how the owners of the group use the source TD -
       1. use the source TD itself or
       2. copy the source Td to their own td or
       3. 1+2 (both ways) */

    /* count how many owners exist in the ownersList -
       means how many owners copy the source Td to their own descriptor  */
    for (p_Pos = LIST_FIRST(&p_ReplicGroup->ownersList); p_Pos != (&p_ReplicGroup->ownersList); p_Pos = LIST_NEXT(p_Pos))
    {
        /* get the cc node that points to this group */
        p_ReplicGroupOwner = CC_NODE_F_OBJECT(p_Pos);

        /* add the number of owners of this group on the specific cc node */
        numOfOwnersInList+=p_ReplicGroupOwner->index;
    }

    if (numOfOwnersInList)
        useSourceTdOption |= e_USE_SOURCE_TD_COPY_OPTION;
    ASSERT_COND(numOfOwnersInList <= p_ReplicGroup->owners);

    /* calculate the number of owners that use the source TD itself */
    numOfSourceTdOwners = p_ReplicGroup->owners - numOfOwnersInList;
    if (numOfSourceTdOwners)
        useSourceTdOption |= e_USE_SOURCE_TD_ITSELF_OPTION;

    return useSourceTdOption;
}

static uint8_t  GetMemberPosition(t_FmPcdFrmReplicGroup *p_ReplicGroup,
                                  uint32_t              memberIndex,
                                  bool                  isAddOperation)
{
    uint8_t     memberPosition;
    uint32_t    lastMemberIndex;

    ASSERT_COND(p_ReplicGroup);

    /* the last member index is different between add and remove operation -
    in case of remove - this is exactly the last member index
    in case of add - this is the last member index + 1 - e.g.
    if we have 4 members, the index of the actual last member is 3(because the
    index starts from 0) therefore in order to add a new member as the last
    member we shall use memberIndex = 4 and not 3
    */
    if (isAddOperation)
        lastMemberIndex = p_ReplicGroup->numOfEntries;
    else
        lastMemberIndex = p_ReplicGroup->numOfEntries-1;

    /* last */
    if (memberIndex == lastMemberIndex)
        memberPosition = FRM_REPLIC_LAST_MEMBER_INDEX;
    else
    {
        /* first */
        if (memberIndex == FRM_REPLIC_FIRST_MEMBER_INDEX)
            memberPosition = FRM_REPLIC_FIRST_MEMBER_INDEX;
        else
        {
            /* middle */
            ASSERT_COND(memberIndex < lastMemberIndex);
            memberPosition = FRM_REPLIC_MIDDLE_MEMBER_INDEX;
        }
    }
    return memberPosition;
}

static t_Error CheckParams(t_Handle                     h_FmPcd,
                           t_FmPcdFrmReplicGroupParams *p_ReplicGroupParam)
{
    int             i;
    t_Error         err;

    /* check that max num of entries is at least 2 */
    if (p_ReplicGroupParam->maxNumOfEntries <= 1)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("maxNumOfEntries in the frame replicator group should be greater than one"));

    if (p_ReplicGroupParam->maxNumOfEntries >= FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("maxNumOfEntries in the frame replicator should not be greater than %d",FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES));

    /* check that number of entries is greater than zero */
    if (!p_ReplicGroupParam->numOfEntries)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("numOFEntries in the frame replicator group should be greater than zero"));

    /* check that max num of entries is equal or greater than number of entries */
    if (p_ReplicGroupParam->maxNumOfEntries < p_ReplicGroupParam->numOfEntries)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("maxNumOfEntries should be equal or greater than numOfEntries"));

    for (i=0;i<p_ReplicGroupParam->numOfEntries;i++)
    {

        if (p_ReplicGroupParam->nextEngineParams[i].nextEngine == e_FM_PCD_CC)
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Nect engine of a member can't be cc"));

        if (p_ReplicGroupParam->nextEngineParams[i].nextEngine == e_FM_PCD_FR)
            RETURN_ERROR(MAJOR, E_NOT_SUPPORTED, ("Nect engine of a member can't be replicator group"));

        /* check the regular parameters of the next engine */
        err = ValidateNextEngineParams(h_FmPcd, &p_ReplicGroupParam->nextEngineParams[i]);
        if (err)
            RETURN_ERROR(MAJOR, err, ("member next engine parameters"));
    }
    return E_OK;
}

static t_FmPcdFrmReplicMember *GetAvailableMember(t_FmPcdFrmReplicGroup *p_ReplicGroup)
{
    t_FmPcdFrmReplicMember  *p_ReplicMember = NULL;
    t_List                  *p_Next;

    if (!LIST_IsEmpty(&p_ReplicGroup->availableMembersList))
    {
        p_Next = LIST_FIRST(&p_ReplicGroup->availableMembersList);
        p_ReplicMember = LIST_OBJECT(p_Next, t_FmPcdFrmReplicMember, node);
        ASSERT_COND(p_ReplicMember);
        LIST_DelAndInit(p_Next);
    }
    return p_ReplicMember;
}

static void PutAvailableMember(t_FmPcdFrmReplicGroup    *p_ReplicGroup,
                               t_FmPcdFrmReplicMember   *p_ReplicMember)
{
    LIST_AddToTail(&p_ReplicMember->node, &p_ReplicGroup->availableMembersList);
}

static void AddMemberToList(t_FmPcdFrmReplicGroup   *p_ReplicGroup,
                            t_FmPcdFrmReplicMember  *p_CurrentMember,
                            t_List                  *p_ListHead)
{
    LIST_Add(&p_CurrentMember->node, p_ListHead);

    p_ReplicGroup->numOfEntries++;
}

static void RemoveMemberFromList(t_FmPcdFrmReplicGroup  *p_ReplicGroup,
                                 t_FmPcdFrmReplicMember *p_CurrentMember)
{
    ASSERT_COND(p_ReplicGroup->numOfEntries);
    LIST_DelAndInit(&p_CurrentMember->node);
    p_ReplicGroup->numOfEntries--;
}

static void LinkSourceToMember(t_FmPcdFrmReplicGroup    *p_ReplicGroup,
                               t_AdOfTypeContLookup     *p_SourceTd,
                               t_FmPcdFrmReplicMember   *p_ReplicMember)
{
    t_FmPcd             *p_FmPcd;

    ASSERT_COND(p_SourceTd);
    ASSERT_COND(p_ReplicMember);
    ASSERT_COND(p_ReplicGroup);
    ASSERT_COND(p_ReplicGroup->h_FmPcd);

    /* Link the first member in the group to the source TD */
    p_FmPcd = p_ReplicGroup->h_FmPcd;

    WRITE_UINT32(p_SourceTd->matchTblPtr,
        (uint32_t)(XX_VirtToPhys(p_ReplicMember->p_MemberAd) - p_FmPcd->physicalMuramBase));
}

static void LinkMemberToMember(t_FmPcdFrmReplicGroup    *p_ReplicGroup,
                               t_FmPcdFrmReplicMember   *p_CurrentMember,
                               t_FmPcdFrmReplicMember   *p_NextMember)
{
    t_AdOfTypeResult    *p_CurrReplicAd = (t_AdOfTypeResult*)p_CurrentMember->p_MemberAd;
    t_AdOfTypeResult    *p_NextReplicAd = NULL;
    t_FmPcd             *p_FmPcd;
    uint32_t            address = 0;

    /* Check if the next member exists or it's NULL (- means that this is the last member) */
    if (p_NextMember)
    {
        p_NextReplicAd = (t_AdOfTypeResult*)p_NextMember->p_MemberAd;
        p_FmPcd = p_ReplicGroup->h_FmPcd;
        address = (XX_VirtToPhys(p_NextReplicAd) - (p_FmPcd->physicalMuramBase));
        address = ((address>>NEXT_FRM_REPLIC_ADDR_SHIFT)<< NEXT_FRM_REPLIC_MEMBER_INDEX_SHIFT);
    }

    /* link the current AD to point to the AD of the next member */
    WRITE_UINT32(p_CurrReplicAd->res, address);
}

static t_Error ModifyDescriptor(t_FmPcdFrmReplicGroup   *p_ReplicGroup,
                                void                    *p_OldDescriptor,
                                void                    *p_NewDescriptor)
{
    t_Handle            h_Hc;
    t_Error             err;
    uint32_t            oldDescriptorOffset, newDescriptorOffset;
    t_FmPcd             *p_FmPcd;

    ASSERT_COND(p_ReplicGroup);
    ASSERT_COND(p_ReplicGroup->h_FmPcd);

    p_FmPcd = p_ReplicGroup->h_FmPcd;
    h_Hc = FmPcdGetHcHandle(p_FmPcd);
    if (!h_Hc)
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("Host command"));

    oldDescriptorOffset =
        (uint32_t)(XX_VirtToPhys(p_OldDescriptor) - p_FmPcd->physicalMuramBase);
    newDescriptorOffset =
        (uint32_t)(XX_VirtToPhys(p_NewDescriptor) - p_FmPcd->physicalMuramBase);

    err = FmHcPcdCcDoDynamicChange(h_Hc, oldDescriptorOffset, newDescriptorOffset);
    if (err)
        RETURN_ERROR(MAJOR, err, ("Dynamic change host command"));

    return E_OK;
}

static t_Error ModifyGroupOwners(t_FmPcdFrmReplicGroup  *p_ReplicGroup,
                                 t_FmPcdFrmReplicMember *p_Member,
                                 bool                   useHostCommand)
{
    t_List                      h_List, adTablesList, *p_Pos, *p_Pos1;
    uint32_t                    numOfAdTables, numOfOwnersInList, intFlags;
    t_CcNodeInformation         *p_GroupOwner, *p_Node;
    t_Error                     err;

    /* Get the number of owners in the list */
    numOfOwnersInList = LIST_NumOfObjs(&p_ReplicGroup->ownersList);

    /* Go over all the nodes that use this replicator group */
    LIST_FOR_EACH(p_Pos, &p_ReplicGroup->ownersList)
    {
        /* Get the node that use this replic group */
        p_Node = CC_NODE_F_OBJECT(p_Pos);
        ASSERT_COND(p_Node);

        /* lock the specific node */
        INIT_LIST(&h_List);
        INIT_LIST(&adTablesList);

        intFlags = FmPcdLock(p_ReplicGroup->h_FmPcd);
        err = FmPcdCcNodeTreeTryLock(p_ReplicGroup->h_FmPcd,
                                     p_Node->h_CcNode,
                                     &h_List);
        FmPcdUnlock(p_ReplicGroup->h_FmPcd, intFlags);
        if (err)
            RETURN_ERROR(MAJOR, err, ("lock ccNode in ModifyGroupOwners"));

        /* Get the Ad tables pointers of the specific node that use this replicator group */
        FmPcdCcGetAdTablesThatPointOnReplicGroup(p_Node->h_CcNode,
                                                 p_ReplicGroup,
                                                 &adTablesList,
                                                 &numOfAdTables);

        /* go over all the ad tables that share the group handle and update them according to the
           main source TD of the group */
        LIST_FOR_EACH(p_Pos1, &adTablesList)
        {
            p_GroupOwner = CC_NODE_F_OBJECT(p_Pos1);
            ASSERT_COND(p_GroupOwner);

            /* Check if useHostCommand is set and this is the last owner in the list
               and the last ad table in the node -
               use a Host Command, otherwise there is no need of host command -
               just an update of the owner descriptor */
            if ((useHostCommand) && (numOfOwnersInList == 1) && (numOfAdTables == 1))
            {
                err = ModifyDescriptor(p_ReplicGroup,
                                       p_GroupOwner->h_CcNode,    /*old pointer*/
                                       p_ReplicGroup->p_SourceTd);/*new pointer*/
                if (err)
                {
                    FmPcdCcNodeTreeReleaseLock(&h_List);
                    RETURN_ERROR(MAJOR, err, ("Modify group owner"));
                }
            }
            else
                LinkSourceToMember(p_ReplicGroup, p_GroupOwner->h_CcNode, p_Member);

            numOfAdTables--;
        }
        /* unlock node */
        FmPcdCcNodeTreeReleaseLock(&h_List);
        numOfOwnersInList--;

        /* free the ad tables list */
        ReleaseLst(&adTablesList);
    }
    return E_OK;
}

static void BuildSourceTd(void *p_Ad)
{
    t_AdOfTypeContLookup    *p_SourceTd;

    ASSERT_COND(p_Ad);

    p_SourceTd = (t_AdOfTypeContLookup *)p_Ad;

    IOMemSet32((uint8_t*)p_SourceTd, 0, FM_PCD_CC_AD_ENTRY_SIZE);

    /* initialize the source table descriptor */
    WRITE_UINT32(p_SourceTd->ccAdBase,     FM_PCD_AD_CONT_LOOKUP_TYPE);
    WRITE_UINT32(p_SourceTd->pcAndOffsets, FRM_REPLIC_SOURCE_TD_OPCODE);
}

static void FillReplicAdOfTypeResult(void *p_ReplicAd, bool last)
{
    t_AdOfTypeResult    *p_CurrReplicAd = (t_AdOfTypeResult*)p_ReplicAd;
    uint32_t            tmp;

    tmp = GET_UINT32(p_CurrReplicAd->plcrProfile);
    if (last)
        /* clear the NL bit in case it's the last member in the group*/
        WRITE_UINT32(p_CurrReplicAd->plcrProfile,(tmp & ~FRM_REPLIC_NL_BIT));
    else
        /* set the NL bit in case it's not the last member in the group */
        WRITE_UINT32(p_CurrReplicAd->plcrProfile, (tmp |FRM_REPLIC_NL_BIT));

    /* set FR bit in the action descriptor */
    tmp = GET_UINT32(p_CurrReplicAd->nia);
    WRITE_UINT32(p_CurrReplicAd->nia,
        (tmp | FRM_REPLIC_FR_BIT | FM_PCD_AD_RESULT_EXTENDED_MODE ));
}

static t_FmPcdFrmReplicMember* GetMemberByIndex(t_FmPcdFrmReplicGroup   *p_ReplicGroup,
                                                uint16_t                memberIndex)
{
    int                     i=0;
    t_List                  *p_Pos;
    t_FmPcdFrmReplicMember  *p_Member = NULL;

    LIST_FOR_EACH(p_Pos, &p_ReplicGroup->membersList)
    {
        if (i == memberIndex)
        {
            p_Member = LIST_OBJECT(p_Pos, t_FmPcdFrmReplicMember, node);
            return p_Member;
        }
        i++;
    }
    return p_Member;
}

static t_Error BuildShadowAndModifyDescriptor(t_FmPcdFrmReplicGroup   *p_ReplicGroup,
                                              t_FmPcdFrmReplicMember  *p_NextMember,
                                              t_FmPcdFrmReplicMember  *p_CurrentMember,
                                              bool                    sourceDescriptor,
                                              bool                    last)
{
    t_FmPcd                 *p_FmPcd;
    t_FmPcdFrmReplicMember  shadowMember;
    t_Error                 err;

    ASSERT_COND(p_ReplicGroup);
    ASSERT_COND(p_ReplicGroup->h_FmPcd);

    p_FmPcd = p_ReplicGroup->h_FmPcd;
    ASSERT_COND(p_FmPcd->p_CcShadow);

    if (sourceDescriptor)
    {
        BuildSourceTd(p_FmPcd->p_CcShadow);
        LinkSourceToMember(p_ReplicGroup, p_FmPcd->p_CcShadow, p_NextMember);

        /* Modify the source table descriptor according to the prepared shadow descriptor */
        err = ModifyDescriptor(p_ReplicGroup,
                               p_ReplicGroup->p_SourceTd,
                               p_FmPcd->p_CcShadow/* new prepared source td */);
        if (err)
            RETURN_ERROR(MAJOR, err, ("Modify source Descriptor in BuildShadowAndModifyDescriptor"));

    }
    else
    {
        IO2IOCpy32(p_FmPcd->p_CcShadow,
                   p_CurrentMember->p_MemberAd,
                   FM_PCD_CC_AD_ENTRY_SIZE);

        FillReplicAdOfTypeResult(p_FmPcd->p_CcShadow, last);
        shadowMember.p_MemberAd = p_FmPcd->p_CcShadow;
        LinkMemberToMember(p_ReplicGroup, &shadowMember, p_NextMember);

        /* Modify the next member according to the prepared shadow descriptor */
        err = ModifyDescriptor(p_ReplicGroup,
                               p_CurrentMember->p_MemberAd,
                               &shadowMember.p_MemberAd);
        if (err)
            RETURN_ERROR(MAJOR, err, ("ModifyDescriptor in BuildShadowAndModifyDescriptor"));
    }


    return E_OK;
}

static t_FmPcdFrmReplicMember* InitMember(t_FmPcdFrmReplicGroup     *p_ReplicGroup,
                                          t_FmPcdCcNextEngineParams *p_MemberParams,
                                          bool                      last)
{
    t_FmPcdFrmReplicMember  *p_CurrentMember = NULL;

    ASSERT_COND(p_ReplicGroup);

    /* Get an available member from the internal members list */
    p_CurrentMember = GetAvailableMember(p_ReplicGroup);
    if (!p_CurrentMember)
    {
        REPORT_ERROR(MAJOR, E_NOT_FOUND, ("Available member"));
        return NULL;
    }
    p_CurrentMember->h_Manip = NULL;
    /* clear the Ad of the new member */
    IOMemSet32((uint8_t*)p_CurrentMember->p_MemberAd, 0, FM_PCD_CC_AD_ENTRY_SIZE);

    INIT_LIST(&p_CurrentMember->node);

    /* Initialize the Ad of the member */
    NextStepAd(p_CurrentMember->p_MemberAd, p_MemberParams, p_ReplicGroup->h_FmPcd);

    /* save Manip handle (for free needs) */
    if (p_MemberParams->h_Manip)
        p_CurrentMember->h_Manip = p_MemberParams->h_Manip;

    /* Initialize the relevant frame replicator fields in the AD */
    FillReplicAdOfTypeResult(p_CurrentMember->p_MemberAd, last);

    return p_CurrentMember;
}

static void FreeMember(t_FmPcdFrmReplicGroup    *p_ReplicGroup,
                       t_FmPcdFrmReplicMember   *p_Member)
{
    /* Note: Can't free the member+statistics AD since the free member returns to the available
       member list - only memset them */

    /* zero the AD */
    IOMemSet32(p_Member->p_MemberAd, 0, FM_PCD_CC_AD_ENTRY_SIZE);


    /* return the member to the available members list */
    PutAvailableMember(p_ReplicGroup, p_Member);
}

static t_Error RemoveMember(t_FmPcdFrmReplicGroup   *p_ReplicGroup,
                            uint16_t                memberIndex)
{
    t_FmPcd                 *p_FmPcd = NULL;
    t_FmPcdFrmReplicMember  *p_CurrentMember = NULL, *p_PreviousMember = NULL, *p_NextMember = NULL;
    t_Error                 err;
    uint8_t                 memberPosition;
    e_UseSourceTdOption     useSourceTdOption = e_USE_SOURCE_TD_NONE;

    p_FmPcd = p_ReplicGroup->h_FmPcd;

    p_CurrentMember = GetMemberByIndex(p_ReplicGroup, memberIndex);
    ASSERT_COND(p_CurrentMember);

    /* determine the member position in the group */
    memberPosition = GetMemberPosition(p_ReplicGroup, memberIndex, FALSE/*remove operation*/);

    switch (memberPosition)
    {
        case FRM_REPLIC_FIRST_MEMBER_INDEX:
            p_NextMember = GetMemberByIndex(p_ReplicGroup, (uint16_t)(memberIndex+1));
            ASSERT_COND(p_NextMember);

            /* Check how the owners of the group use the source TD -
               1. use the source TD itself or
               2. copy the source Td to their own td or
               3. 1+2 (both ways) */
            if (p_ReplicGroup->owners)
                useSourceTdOption = CheckUseSourceTdOptionType(p_ReplicGroup);

            /* The decision is to update the last table descriptor that use this group
               by host command (for sync purposes).
               if there are only owners that use the source Td itself- then we should update the
               source Td by a host command (for sync) ,
               if there are only owners that copy the source td to their own td so the
               host command will be called when the last owner will update its own td (In ModifyGroupOwners routine)
               If both cases - only one host command will be called when updating the source Td */

            /* check how the owners use the source Td and modify it accordingly */
            switch (useSourceTdOption)
            {
                case e_USE_SOURCE_TD_NONE:
                    LinkSourceToMember(p_ReplicGroup, p_ReplicGroup->p_SourceTd, p_NextMember);
                    break;

                case e_USE_SOURCE_TD_ITSELF_OPTION:
                    /* update the source td itself by using a host command */
                    if (!TRY_LOCK(p_FmPcd->h_ShadowSpinlock, &p_FmPcd->shadowLock))
                        return ERROR_CODE(E_BUSY);

                    err = BuildShadowAndModifyDescriptor(p_ReplicGroup,
                                                         p_NextMember,
                                                         NULL,
                                                         TRUE/*sourceDescriptor*/,
                                                         FALSE/*last*/);
                    RELEASE_LOCK(p_FmPcd->shadowLock);

                    if (err)
                        RETURN_ERROR(MAJOR, err, ("Build Shadow and modify descriptor in case of first member"));
                    break;

                case e_USE_SOURCE_TD_COPY_OPTION:
                    LinkSourceToMember(p_ReplicGroup, p_ReplicGroup->p_SourceTd, p_NextMember);

                    /* Update all the owners to point to the next member */
                    err = ModifyGroupOwners(p_ReplicGroup, p_NextMember, TRUE/*useHostCommand*/);
                    if (err)
                        RETURN_ERROR(MAJOR, err, ("Modify Group Owners"));
                    break;

                case e_USE_SOURCE_TD_ITSELF_AND_COPY_OPTION:
                    /* Update all the owners to point to the next member */
                    err = ModifyGroupOwners(p_ReplicGroup, p_NextMember, FALSE/*useHostCommand*/);
                    if (err)
                        RETURN_ERROR(MAJOR, err, ("Modify Group Owners"));

                    if (!TRY_LOCK(p_FmPcd->h_ShadowSpinlock, &p_FmPcd->shadowLock))
                        return ERROR_CODE(E_BUSY);

                    /* update the source TD by using a host command */
                    err = BuildShadowAndModifyDescriptor(p_ReplicGroup,
                                                         p_NextMember,
                                                         NULL,
                                                         TRUE/*sourceDescriptor*/,
                                                         FALSE/*last*/);
                    RELEASE_LOCK(p_FmPcd->shadowLock);
                    if (err)
                        RETURN_ERROR(MAJOR, err, ("Modify Group Owners"));
                    break;
                default:
                    RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("source td use"));
            }
            break;

        case FRM_REPLIC_MIDDLE_MEMBER_INDEX:
            p_PreviousMember = GetMemberByIndex(p_ReplicGroup, (uint16_t)(memberIndex-1));
            ASSERT_COND(p_PreviousMember);

            p_NextMember = GetMemberByIndex(p_ReplicGroup, (uint16_t)(memberIndex+1));
            ASSERT_COND(p_NextMember);

            if (!TRY_LOCK(p_FmPcd->h_ShadowSpinlock, &p_FmPcd->shadowLock))
                return ERROR_CODE(E_BUSY);

            err = BuildShadowAndModifyDescriptor(p_ReplicGroup,
                                                 p_NextMember,
                                                 p_PreviousMember,
                                                 FALSE/*sourceDescriptor*/,
                                                 FALSE/*last*/);

            RELEASE_LOCK(p_FmPcd->shadowLock);

            if (err)
                RETURN_ERROR(MAJOR, err, ("Build Shadow and modify descriptor in case of middle member"));
            break;

        case FRM_REPLIC_LAST_MEMBER_INDEX:
            p_PreviousMember = GetMemberByIndex(p_ReplicGroup, (uint16_t)(memberIndex-1));
            ASSERT_COND(p_PreviousMember);

            if (!TRY_LOCK(p_FmPcd->h_ShadowSpinlock, &p_FmPcd->shadowLock))
                return ERROR_CODE(E_BUSY);

            err = BuildShadowAndModifyDescriptor(p_ReplicGroup,
                                                 NULL,
                                                 p_PreviousMember,
                                                 FALSE/*sourceDescriptor*/,
                                                 TRUE/*last*/);
            RELEASE_LOCK(p_FmPcd->shadowLock);

            if (err)
                RETURN_ERROR(MAJOR, err, ("Build Shadow and modify descriptor in case of last member"));
            break;

        default:
            RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("member position in remove member"));
    }

    if (p_CurrentMember->h_Manip)
    {
        FmPcdManipUpdateOwner(p_CurrentMember->h_Manip, FALSE);
        p_CurrentMember->h_Manip = NULL;
    }

    /* remove the member from the driver internal members list */
    RemoveMemberFromList(p_ReplicGroup, p_CurrentMember);

    /* return the member to the available members list */
    FreeMember(p_ReplicGroup, p_CurrentMember);
    return E_OK;
}

static void DeleteGroup(t_FmPcdFrmReplicGroup *p_ReplicGroup)
{
    int                     i, j;
    t_Handle                h_Muram;
    t_FmPcdFrmReplicMember  *p_Member, *p_CurrentMember;

    if (p_ReplicGroup)
    {
        ASSERT_COND(p_ReplicGroup->h_FmPcd);
        h_Muram = FmPcdGetMuramHandle(p_ReplicGroup->h_FmPcd);
        ASSERT_COND(h_Muram);

        /* free the source table descriptor */
        if (p_ReplicGroup->p_SourceTd)
        {
            FM_MURAM_FreeMem(h_Muram, p_ReplicGroup->p_SourceTd);
            p_ReplicGroup->p_SourceTd = NULL;
        }

        /* Remove all members from the members linked list (hw and sw) and return the
           members to the available members list */
        if (p_ReplicGroup->numOfEntries)
        {
            j = p_ReplicGroup->numOfEntries-1;

            /* manually removal of the member because there are no owners of this group */
            for (i=j; i>=0; i--)
            {
                p_CurrentMember = GetMemberByIndex(p_ReplicGroup, i/*memberIndex*/);
                ASSERT_COND(p_CurrentMember);

                if (p_CurrentMember->h_Manip)
                {
                    FmPcdManipUpdateOwner(p_CurrentMember->h_Manip, FALSE);
                    p_CurrentMember->h_Manip = NULL;
                }

                /* remove the member from the internal driver members list */
                RemoveMemberFromList(p_ReplicGroup, p_CurrentMember);

                /* return the member to the available members list */
                FreeMember(p_ReplicGroup, p_CurrentMember);
            }
        }

        /* Free member AD + Statistics */
        for (i=0;i<p_ReplicGroup->maxNumOfEntries;i++)
        {
            p_Member = GetAvailableMember(p_ReplicGroup);
            if (p_Member)
            {
                if (p_Member->p_MemberAd)
                {
                    FM_MURAM_FreeMem(h_Muram, p_Member->p_MemberAd);
                    p_Member->p_MemberAd = NULL;
                }
                XX_Free(p_Member);
            }
            else
            {
                REPORT_ERROR(MAJOR, E_INVALID_HANDLE, ("Missing member from available"));
                return;
            }
        }

        /* release the lock */
        if (p_ReplicGroup->p_Lock)
            FmPcdReleaseLock(p_ReplicGroup->h_FmPcd, p_ReplicGroup->p_Lock);

        /* free the replicator group */
        XX_Free(p_ReplicGroup);
    }
}


/*****************************************************************************/
/*              Inter-module API routines                                    */
/*****************************************************************************/

void * FrmReplicGetSourceTableDescriptor(t_Handle h_ReplicGroup)
{
    t_FmPcdFrmReplicGroup   *p_ReplicGroup = (t_FmPcdFrmReplicGroup *)h_ReplicGroup;

    if (!p_ReplicGroup)
    {
        REPORT_ERROR(MAJOR, E_INVALID_HANDLE, ("Frame replicator group"));
        return NULL;
    }
    return (p_ReplicGroup->p_SourceTd);
}

void FrmReplicUpdateAdContLookupForCc(t_Handle  h_ReplicGroup,
                                      t_Handle  p_Ad,
                                      t_Handle  *h_AdNew)
{
    t_FmPcdFrmReplicGroup   *p_ReplicGroup = (t_FmPcdFrmReplicGroup *)h_ReplicGroup;

    /* Copy the already prepared source table descriptor to the current AD pointer */
    WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->ccAdBase,     GET_UINT32(((t_AdOfTypeContLookup *)(p_ReplicGroup->p_SourceTd))->ccAdBase));
    WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->matchTblPtr,  GET_UINT32(((t_AdOfTypeContLookup *)(p_ReplicGroup->p_SourceTd))->matchTblPtr));
    WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->pcAndOffsets, GET_UINT32(((t_AdOfTypeContLookup *)(p_ReplicGroup->p_SourceTd))->pcAndOffsets));
    WRITE_UINT32(((t_AdOfTypeContLookup *)p_Ad)->gmask,        GET_UINT32(((t_AdOfTypeContLookup *)(p_ReplicGroup->p_SourceTd))->gmask));
    *h_AdNew = NULL;
}

void  FrmReplicUpdateGroupOwner(t_Handle    h_ReplicGroup,
                                bool        add,
                                bool        fullUpdate,
                                t_Handle    h_FmPcdCcNode)
{
    t_CcNodeInformation     *p_ReplicGroupOwner = NULL;
    t_CcNodeInformation     replicGroupOwner;
    t_FmPcdFrmReplicGroup   *p_ReplicGroup = (t_FmPcdFrmReplicGroup *)h_ReplicGroup;

    ASSERT_COND(p_ReplicGroup);

    /* lock */
    /* TODO - ask p_ReplicGroup->h_SpinLock */
    /* check if to update the information in the owners list */
    if (fullUpdate)
    {
        ASSERT_COND(h_FmPcdCcNode);
        /* search if this node already exists in the owners list */
        p_ReplicGroupOwner = FindNodeInfoInReleventLst(&p_ReplicGroup->ownersList, h_FmPcdCcNode, NULL);

        /* add a new owner to the list */
        if (add)
        {
            /* found the owner in the list - just increment the counter */
            if (p_ReplicGroupOwner)
                p_ReplicGroupOwner->index++;
            else
            {
                replicGroupOwner.h_CcNode  = h_FmPcdCcNode;
                replicGroupOwner.index     = 1;
                /* the routine will allocate internally group owner structure, initialize it
                according to the replicGroupOwner parameters and insert it to the list */
                EnqueueNodeInfoToRelevantLst(&p_ReplicGroup->ownersList, &replicGroupOwner, NULL);
            }
        }
        else
        {
            /* update the groupOwner numbers */
            UpdateGroupOwner(p_ReplicGroup, add);
            if (p_ReplicGroupOwner)
            {
                p_ReplicGroupOwner->index--;
                if (p_ReplicGroupOwner->index == 0)
                {
                    /* No more group owners in the node - remove it from the list */
                    DequeueNodeInfoFromRelevantLst(&p_ReplicGroup->ownersList, p_ReplicGroupOwner, NULL);
                    /* Free the group owner structure */
                    XX_Free(p_ReplicGroupOwner);
                }
            }
        }
    }
    else
        /* update the groupOwner numbers */
        UpdateGroupOwner(p_ReplicGroup, add);

}


/*********************** End of inter-module routines ************************/

/****************************************/
/*       API Init unit functions        */
/****************************************/
t_Handle FM_PCD_FrmReplicSetGroup(t_Handle                      h_FmPcd,
                                  t_FmPcdFrmReplicGroupParams   *p_ReplicGroupParam)
{
    t_FmPcdFrmReplicGroup       *p_ReplicGroup;
    t_AdOfTypeResult            *p_Ad = NULL;
    t_FmPcdFrmReplicMember      *p_CurrentMember, *p_NextMember;
    int                         i;
    t_Error                     err;
    bool                        last = FALSE;
    t_Handle                    h_Muram;

    SANITY_CHECK_RETURN_VALUE(h_FmPcd, E_INVALID_HANDLE, NULL);
    SANITY_CHECK_RETURN_VALUE(p_ReplicGroupParam, E_INVALID_HANDLE, NULL);

    err = CheckParams(h_FmPcd, p_ReplicGroupParam);
    if (err)
    {
        REPORT_ERROR(MAJOR, err, (NO_MSG));
        return NULL;
    }

    p_ReplicGroup = (t_FmPcdFrmReplicGroup*)XX_Malloc(sizeof(t_FmPcdFrmReplicGroup));
    if (!p_ReplicGroup)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("No memory"));
        return NULL;
    }
    memset(p_ReplicGroup, 0, sizeof(t_FmPcdFrmReplicGroup));

    INIT_LIST(&p_ReplicGroup->availableMembersList);
    INIT_LIST(&p_ReplicGroup->membersList);
    INIT_LIST(&p_ReplicGroup->ownersList);

    /* save parameter in the internal structure of the frame replicator group */
    p_ReplicGroup->h_FmPcd = h_FmPcd;

    h_Muram = FmPcdGetMuramHandle(p_ReplicGroup->h_FmPcd);
    ASSERT_COND(h_Muram);

    /* initialize the group lock */
    p_ReplicGroup->p_Lock = FmPcdAcquireLock(p_ReplicGroup->h_FmPcd);
    if (!p_ReplicGroup->p_Lock)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Replic group lock"));
        DeleteGroup(p_ReplicGroup);
        return NULL;
    }

    /* update the shadow size - required for the host commands */
    err = FmPcdUpdateCcShadow(p_ReplicGroup->h_FmPcd,
                              FM_PCD_CC_AD_ENTRY_SIZE,
                              FM_PCD_CC_AD_TABLE_ALIGN);
    if (err)
    {
        REPORT_ERROR(MAJOR, err, ("Update CC shadow"));
        DeleteGroup(p_ReplicGroup);
        return NULL;
    }

    p_ReplicGroup->maxNumOfEntries   = p_ReplicGroupParam->maxNumOfEntries;

    /* Allocate the maximal number of members ADs and Statistics AD for the group
       It prevents allocation of Muram in run-time */
    for (i=0;i<p_ReplicGroup->maxNumOfEntries;i++)
    {
        /* Intialize an internal structure of a member to add to the available members list */
        p_CurrentMember = (t_FmPcdFrmReplicMember *)XX_Malloc(sizeof(t_FmPcdFrmReplicMember));
        if (!p_CurrentMember)
        {
            REPORT_ERROR(MAJOR, E_NO_MEMORY, ("Frame replicator member"));
            DeleteGroup(p_ReplicGroup);
            return NULL;
        }
        memset(p_CurrentMember, 0 ,sizeof(t_FmPcdFrmReplicMember));

        /* Allocate the member AD */
        p_Ad =
            (t_AdOfTypeResult*)FM_MURAM_AllocMem(h_Muram,
                                                 FM_PCD_CC_AD_ENTRY_SIZE,
                                                 FM_PCD_CC_AD_TABLE_ALIGN);
        if (!p_Ad)
        {
            REPORT_ERROR(MAJOR, E_NO_MEMORY, ("No memory in MURAM for an AD table"));
            DeleteGroup(p_ReplicGroup);
            return NULL;
        }
        IOMemSet32((uint8_t*)p_Ad, 0, FM_PCD_CC_AD_ENTRY_SIZE);

        p_CurrentMember->p_MemberAd   = (void*)p_Ad;

        /* Add the new member to the available members list */
        LIST_AddToTail(&p_CurrentMember->node, &(p_ReplicGroup->availableMembersList));
    }
    p_NextMember = NULL;

    /* Initialize the members linked list(hw - the one that is used by the FMan controller and
       the sw one thaat is managed by the driver internally) */
    for (i=(p_ReplicGroupParam->numOfEntries-1);i>=0;i--)
    {
        /* check if this is the last member in the group */
        if (i == (p_ReplicGroupParam->numOfEntries-1))
            last = TRUE;
        else
            last = FALSE;

        /* Initialize a new member */
        p_CurrentMember = InitMember(p_ReplicGroup,
                                  &(p_ReplicGroupParam->nextEngineParams[i]),
                                  last);
        if (!p_CurrentMember)
        {
            REPORT_ERROR(MAJOR, E_INVALID_HANDLE, ("No available member"));
            DeleteGroup(p_ReplicGroup);
            return NULL;
        }
        /* TODO - think if there is an option to combine those two routines to one routine ?? */

        /* Build the members group - link two consecutive members in the hw linked list */
        LinkMemberToMember(p_ReplicGroup, p_CurrentMember, p_NextMember);

        /* update the driver internal members list to be compatible to the hw members linked list */
        AddMemberToList(p_ReplicGroup, p_CurrentMember, &p_ReplicGroup->membersList);

        p_NextMember = p_CurrentMember;
    }

    /* Allocate the frame replicator source table descriptor */
    p_ReplicGroup->p_SourceTd =
        (t_Handle)FM_MURAM_AllocMem(h_Muram,
                                    FM_PCD_CC_AD_ENTRY_SIZE,
                                    FM_PCD_CC_AD_TABLE_ALIGN);
    if (!p_ReplicGroup->p_SourceTd)
    {
        REPORT_ERROR(MAJOR, E_NO_MEMORY, ("frame replicator source table descriptor"));
        DeleteGroup(p_ReplicGroup);
        return NULL;
    }

    /* initialize the source table descriptor */
    BuildSourceTd(p_ReplicGroup->p_SourceTd);

    /* link the source table descriptor to point to the first member in the group */
    LinkSourceToMember(p_ReplicGroup, p_ReplicGroup->p_SourceTd, p_NextMember);

    return p_ReplicGroup;
}

t_Error FM_PCD_FrmReplicDeleteGroup(t_Handle h_ReplicGroup)
{
    t_FmPcdFrmReplicGroup   *p_ReplicGroup = (t_FmPcdFrmReplicGroup *)h_ReplicGroup;

    SANITY_CHECK_RETURN_ERROR(p_ReplicGroup, E_INVALID_HANDLE);
    ASSERT_COND(p_ReplicGroup->h_FmPcd);

    if (p_ReplicGroup->owners)
        RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("the group has owners and can't be deleted"));

    DeleteGroup(p_ReplicGroup);

    return E_OK;
}


/*****************************************************************************/
/*       API Run-time Frame replicator Control unit functions                */
/*****************************************************************************/
t_Error FM_PCD_FrmReplicAddMember(t_Handle                  h_ReplicGroup,
                                  uint16_t                  memberIndex,
                                  t_FmPcdCcNextEngineParams *p_MemberParams)
{
    t_FmPcdFrmReplicGroup       *p_ReplicGroup = (t_FmPcdFrmReplicGroup*) h_ReplicGroup;
    t_FmPcdFrmReplicMember      *p_NewMember, *p_CurrentMember = NULL, *p_PreviousMember = NULL;
    t_Error                     err;
    uint8_t                     memberPosition;

    SANITY_CHECK_RETURN_ERROR(p_ReplicGroup, E_INVALID_HANDLE);
    SANITY_CHECK_RETURN_ERROR(p_MemberParams, E_INVALID_HANDLE);

    if (memberIndex > p_ReplicGroup->numOfEntries)
        RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("memberIndex is greater than the members in the list"));

    if (memberIndex >= p_ReplicGroup->maxNumOfEntries)
        RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("memberIndex is greater than the allowed number of members in the group"));

    if ((p_ReplicGroup->numOfEntries + 1) > FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("numOfEntries with new entry can not be larger than %d\n", FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES));

    /* lock */
    err = FrmReplicGroupTryLock(p_ReplicGroup);
    if (err)
        RETURN_ERROR(MAJOR, err, ("Lock in add operation"));

    /* determine the member position in the group */
    memberPosition = GetMemberPosition(p_ReplicGroup, memberIndex, TRUE/* add operation */);

    /* Initialize a new member */
    p_NewMember =  InitMember(p_ReplicGroup,
                              p_MemberParams,
                              (memberPosition == FRM_REPLIC_LAST_MEMBER_INDEX ? TRUE : FALSE));
    if (!p_NewMember)
    {
        /* unlock */
        FrmReplicGroupReleaseLock(p_ReplicGroup);
        RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("No available member"));
    }

    switch (memberPosition)
    {
        case FRM_REPLIC_FIRST_MEMBER_INDEX:

            p_CurrentMember = GetMemberByIndex(p_ReplicGroup, memberIndex);
            if (!p_CurrentMember)
            {
                /* unlock */
                FrmReplicGroupReleaseLock(p_ReplicGroup);
                RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("current member in add member operation (case first) "));
            }

            LinkMemberToMember(p_ReplicGroup, p_NewMember, p_CurrentMember);

            /* update the internal group source TD */
            LinkSourceToMember(p_ReplicGroup,
                               p_ReplicGroup->p_SourceTd,
                               p_NewMember);

            /* Go over all the nodes that use this replicator group and update all the ad tables
               that point to the replicator group */
            err = ModifyGroupOwners(p_ReplicGroup, p_NewMember, FALSE/*useHostCommand*/);
            if (err)
            {
                /* unlock */
                FrmReplicGroupReleaseLock(p_ReplicGroup);
                RETURN_ERROR(MAJOR, err, ("Update owners in add operation"));
            }

            AddMemberToList(p_ReplicGroup, p_NewMember, &p_ReplicGroup->membersList);
            break;

        case FRM_REPLIC_MIDDLE_MEMBER_INDEX:

            p_CurrentMember = GetMemberByIndex(p_ReplicGroup, memberIndex);
            if (!p_CurrentMember)
            {
                /* unlock */
                FrmReplicGroupReleaseLock(p_ReplicGroup);
                RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("current member in add member operation (case middle) "));
            }

            p_PreviousMember = GetMemberByIndex(p_ReplicGroup, (uint16_t)(memberIndex-1));
            if (!p_PreviousMember)
            {
                /* unlock */
                FrmReplicGroupReleaseLock(p_ReplicGroup);
                RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("previous member in add member operation (case middle) "));
            }
            LinkMemberToMember(p_ReplicGroup, p_NewMember, p_CurrentMember);
            LinkMemberToMember(p_ReplicGroup, p_PreviousMember, p_NewMember);

            AddMemberToList(p_ReplicGroup, p_NewMember, &p_PreviousMember->node);
            break;

        case FRM_REPLIC_LAST_MEMBER_INDEX:

            p_PreviousMember = GetMemberByIndex(p_ReplicGroup, (uint16_t)(memberIndex-1));
            if (!p_PreviousMember)
            {
                /* unlock */
                FrmReplicGroupReleaseLock(p_ReplicGroup);
                RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("previous member in add member operation (case last) "));
            }
            LinkMemberToMember(p_ReplicGroup, p_PreviousMember, p_NewMember);
            FillReplicAdOfTypeResult(p_PreviousMember->p_MemberAd, FALSE/*last*/);

            /* add the new member to the internal member list */
            AddMemberToList(p_ReplicGroup, p_NewMember, &p_PreviousMember->node);
           break;

        default:
            RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("member position in add member"));

    }

    /* unlock */
    FrmReplicGroupReleaseLock(p_ReplicGroup);
    return E_OK;
}

t_Error FM_PCD_FrmReplicRemoveMember(t_Handle   h_ReplicGroup,
                                     uint16_t   memberIndex)
{
    t_FmPcdFrmReplicGroup   *p_ReplicGroup = (t_FmPcdFrmReplicGroup*) h_ReplicGroup;
    t_Error                 err;

    SANITY_CHECK_RETURN_ERROR(p_ReplicGroup, E_INVALID_HANDLE);

    if (memberIndex >= p_ReplicGroup->numOfEntries)
        RETURN_ERROR(MAJOR, E_INVALID_SELECTION, ("member index to remove"));

    /* Design decision: group must contain at least one member
       No possibility to remove the last member from the group */
    if (p_ReplicGroup->numOfEntries == 1)
        RETURN_ERROR(MAJOR, E_CONFLICT, ("Can't remove the last member. At least one member should be related to a group."));

    /* lock */
    err = FrmReplicGroupTryLock(p_ReplicGroup);
    if (err)
        RETURN_ERROR(MAJOR, err, ("Lock in remove member"));

    err = RemoveMember(p_ReplicGroup, memberIndex);
    if (err)
    {
        /* unlock */
        FrmReplicGroupReleaseLock(p_ReplicGroup);
        RETURN_ERROR(MAJOR, err, ("Remove member"));
    }

    /* unlock */
    FrmReplicGroupReleaseLock(p_ReplicGroup);
    return E_OK;
}

/*********************** End of API routines ************************/
