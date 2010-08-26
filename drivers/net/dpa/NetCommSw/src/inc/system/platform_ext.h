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

/******************************************************************************

 @File          platform_ext.h

 @Description   Prototypes, externals and typedefs for platform routines
****************************************************************************/
#ifndef __PLATFORM_EXT_H
#define __PLATFORM_EXT_H

#include "std_ext.h"
#include "sys_ext.h"

/**************************************************************************//**
 @Group         platform_grp PLATFORM Application Programming Interface

 @Description   Generic Platform API, that must be implemented by each
                specific platform.

 @{
*//***************************************************************************/

#define MAX_CHIP_NAME_LEN   9   /* (including null character) */

#define PLATFORM_IO_MODE_ANY    (-1)    /**< Unspecified I/O mode */

/**************************************************************************//**
 @Description   Cache Operation Mode
*//***************************************************************************/
typedef enum e_CacheMode
{
    e_CACHE_MODE_DISABLED       = 0x00000000,   /**< Cache is disabled */
    e_CACHE_MODE_DATA_ONLY      = 0x00000001,   /**< Cache is enabled for data only */
    e_CACHE_MODE_INST_ONLY      = 0x00000002,   /**< Cache is enabled for instructions only */
    e_CACHE_MODE_DATA_AND_INST  = 0x00000003    /**< Cache is enabled for data and instructions */

} e_CacheMode;

/**************************************************************************//**
 @Description   Memory Partition Identifiers

                Note that not all memory partitions are supported by all
                platforms. Every platform may select which memory partitions
                to support.
*//***************************************************************************/
typedef enum e_MemoryPartitionId
{
    e_MEM_1ST_DDR_CACHEABLE     = 1, /**< Primary DDR cacheable memory partition */
    e_MEM_1ST_DDR_NON_CACHEABLE = 2, /**< Primary DDR non-cacheable memory partition */
    e_MEM_2ND_DDR_CACHEABLE     = 3, /**< Secondary DDR cacheable memory partition */
    e_MEM_2ND_DDR_NON_CACHEABLE = 4, /**< Secondary DDR non-cacheable memory partition */
    e_MEM_SDRAM                 = 5, /**< SDRAM non-cacheable memory partition */
    e_MEM_L2_SRAM               = 6, /**< L2 SRAM cacheable memory partition */
    e_MEM_MURAM_LOW             = 7, /**< Low-range MURAM partition */
    e_MEM_MURAM                 = 8  /**< Full-range MURAM partition */

} e_MemoryPartitionId;

/**************************************************************************//**
 @Description   Chip Type and Revision Information Structure
*//***************************************************************************/
typedef struct t_ChipRevInfo
{
    char            chipName[MAX_CHIP_NAME_LEN];
                    /**< Chip name (e.g. "P4080") */
    uint16_t        revMajor;
                    /**< Major chip revision */
    uint16_t        revMinor;
                    /**< Minor chip revision */
} t_ChipRevInfo;

/**************************************************************************//**
 @Description   Platform Events
*//***************************************************************************/
typedef enum e_PlatformEvent
{
    e_PLATFORM_EVENT_LINK_DOWN, /**< Link-up event */
    e_PLATFORM_EVENT_LINK_UP    /**< Link-down event */

} e_PlatformEvent;

/**************************************************************************//**
 @Description   Callback Function Prototype for Link Events
*//***************************************************************************/
typedef void (t_PlatformLinkEventsCb)(t_Handle          h_Controller,
                                      uint8_t           linkId,
                                      e_PlatformEvent   event);



/**************************************************************************//**
 @Description   Interrupt Source Types
*//***************************************************************************/
typedef enum e_InterruptType
{
    e_INTR_TYPE_GENERAL,
    e_INTR_TYPE_IRQ,
    e_INTR_TYPE_ERR,
    e_INTR_TYPE_GTIMERS_TIMER,
    e_INTR_TYPE_RTC_COUNT,
    e_INTR_TYPE_RTC_ALARM,
    e_INTR_TYPE_PCI_IRQ,
    e_INTR_TYPE_PCI_PME,
    e_INTR_TYPE_PCI_MSI,
    e_INTR_TYPE_ETSEC_TX,
    e_INTR_TYPE_ETSEC_RX,
    e_INTR_TYPE_ETSEC_ERROR,
    e_INTR_TYPE_RIO_PW,
    e_INTR_TYPE_RIO_DB_OUT,
    e_INTR_TYPE_RIO_DB_IN,
    e_INTR_TYPE_RIO_MSG_OUT,
    e_INTR_TYPE_RIO_MSG_IN,
    e_INTR_TYPE_QE_LOW,
    e_INTR_TYPE_QE_HIGH,
    e_INTR_TYPE_QE_IO_PORTS,
    e_INTR_TYPE_QE_IRAM_ERR,
    e_INTR_TYPE_QE_MURAM_ERR,
    e_INTR_TYPE_QE_RTT,
    e_INTR_TYPE_QE_SDMA,
    e_INTR_TYPE_QE_VT,
    e_INTR_TYPE_QE_EXT_REQ
} e_InterruptType;

/**************************************************************************//**
 @Description   Descriptor of Board Connections
*//***************************************************************************/
typedef struct t_BoardConnectorDesc
{
    struct
    {
        e_SysModule module;
        uint32_t    id;
    } source;

    struct
    {
        e_SysModule module;
        uint32_t    id;
    } dest;

} t_BoardConnectorDesc;

/**************************************************************************//**
 @Description   Platform configuration parameters structure.

                This structure must be specifically defined by each platform.
*//***************************************************************************/
struct t_PlatformParam;


t_Handle PLATFORM_Init(struct t_PlatformParam *p_PlatformParam);

t_Error  PLATFORM_Free(t_Handle h_Platform);

t_Error  PLATFORM_ConnectExternalRequest(t_Handle h_Platform, e_SysModule module,uint8_t extReqNum);

t_Error  PLATFORM_GetChipRevInfo(t_Handle h_Platform, t_ChipRevInfo *p_ChipRevInfo);

uint32_t PLATFORM_GetMemoryMappedModuleBase(t_Handle    h_Platform,
                                            e_SysModule module,
                                            uint32_t    id);

int PLATFORM_GetInterruptId(t_Handle        h_Platform,
                            e_SysModule     module,
                            uint32_t        id,
                            e_InterruptType intrType,
                            uint32_t        intrRelatedId);

uint32_t PLATFORM_GetCoreClk(t_Handle h_Platform);

uint32_t PLATFORM_GetSystemBusClk(t_Handle h_Platform);

uint32_t PLATFORM_GetControllerClk(t_Handle     h_Platform,
                                   e_SysModule  module,
                                   uint32_t     id);
/** @} */ /* end of platform_grp */

#endif /* __PLATFORM_EXT_H */
