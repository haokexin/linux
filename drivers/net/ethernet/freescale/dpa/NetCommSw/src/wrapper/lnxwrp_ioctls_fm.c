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

/*
 @File          lnxwrp_ioctls_fm.c
 @Author        Shlomi Gridish
 @Description   FM Linux wrapper functions.
*/

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
#include <linux/slab.h>
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
#include <sysdev/fsl_soc.h>
#include <linux/slab.h>

#if defined(CONFIG_COMPAT)
#include <linux/compat.h>
#endif

#include "part_ext.h"
#include "fm_ioctls.h"
#include "fm_pcd_ioctls.h"
#include "fm_port_ioctls.h"

#if defined(CONFIG_COMPAT)
#include "lnxwrp_ioctls_fm_compat.h"
#endif

#include "lnxwrp_fm.h"

#define CMP_IOC_DEFINE(def) (IOC_##def != def)

/* fm_pcd_ioctls.h === fm_pcd_ext.h assertions */
#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_PRIVATE_HDRS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_NUM_OF_HDRS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_INTERCHANGEABLE_HDRS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_NUM_OF_GENERIC_REGS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_MAX_NUM_OF_EXTRACTS_PER_KEY)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_NUM_OF_EXTRACT_MASKS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_KG_NUM_OF_DEFAULT_GROUPS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_NUM_OF_LABELS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_SW_PRS_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_SW_OFFSET)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_SW_PATCHES_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_PRS_SW_TAIL_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_SW_PRS_MAX_IMAGE_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_MANIP_INSRT_TEMPLATE_SIZE)
#error Error: please synchronize IOC_ defines!
#endif

#if DPAA_VERSION >= 11
#if CMP_IOC_DEFINE(FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES)
#error Error: please synchronize IOC_ defines!
#endif
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_NODES)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_TREES)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_GROUPS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_UNITS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_KEYS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_SIZE_OF_KEY)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(FM_PCD_LAST_KEY_INDEX)
#error Error: please synchronize IOC_ defines!
#endif

/* net_ioctls.h === net_ext.h assertions */
#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPP_PID)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPP_COMPRESSED)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPPoE_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPPMUX_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PPPMUX_SUBFRAME_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_ETH_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPv4_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPv6_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_ICMP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IGMP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_TCP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_SCTP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_DCCP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_UDP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_UDP_ENCAP_ESP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPHC_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_SCTP_CHUNK_DATA_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_L2TPv2_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_L2TPv3_CTRL_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_L2TPv3_SESS_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_VLAN_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_LLC_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_NLPID_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_SNAP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_LLC_SNAP_ALL_FIELDS)
#warning Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_ARP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_RFC2684_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_USER_DEFINED_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_PAYLOAD_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_GRE_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_MINENCAP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPSEC_AH_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_IPSEC_ESP_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_MPLS_LABEL_STACK_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

#if CMP_IOC_DEFINE(NET_HEADER_FIELD_MACSEC_ALL_FIELDS)
#error Error: please synchronize IOC_ defines!
#endif

/* fm_ioctls.h === fm_ext.h assertions */
#if CMP_IOC_DEFINE(FM_MAX_NUM_OF_VALID_PORTS)
#error Error: please synchronize IOC_ defines!
#endif

/* fm_port_ioctls.h === dpaa_integrations_ext.h assertions */
#if CMP_IOC_DEFINE(FM_PORT_NUM_OF_CONGESTION_GRPS)
#error Error: please synchronize IOC_ defines!
#endif

void LnxWrpPCDIOCTLTypeChecking(void)
{
    /* fm_ext.h == fm_ioctls.h */
    ASSERT_COND(sizeof(ioc_fm_port_bandwidth_params) == sizeof(t_FmPortsBandwidthParams));
    ASSERT_COND(sizeof(ioc_fm_revision_info_t) == sizeof(t_FmRevisionInfo));

    /* fm_pcd_ext.h == fm_pcd_ioctls.h */
    /*ioc_fm_pcd_counters_params_t  : NOT USED */
    /*ioc_fm_pcd_exception_params_t : private */
    ASSERT_COND(sizeof(ioc_fm_pcd_prs_label_params_t) == sizeof(t_FmPcdPrsLabelParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_prs_sw_params_t) == sizeof(t_FmPcdPrsSwParams));
    /*ioc_fm_pcd_kg_dflt_value_params_t : private */
    ASSERT_COND(sizeof(ioc_fm_pcd_hdr_protocol_opt_u) == sizeof(u_FmPcdHdrProtocolOpt));
    ASSERT_COND(sizeof(ioc_fm_pcd_fields_u) == sizeof(t_FmPcdFields));
    ASSERT_COND(sizeof(ioc_fm_pcd_from_hdr_t) == sizeof(t_FmPcdFromHdr));
    ASSERT_COND(sizeof(ioc_fm_pcd_from_field_t) == sizeof(t_FmPcdFromField));
    ASSERT_COND(sizeof(ioc_fm_pcd_distinction_unit_t) == sizeof(t_FmPcdDistinctionUnit));
    ASSERT_COND(sizeof(ioc_fm_pcd_net_env_params_t) == sizeof(t_FmPcdNetEnvParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_extract_entry_t) == sizeof(t_FmPcdExtractEntry));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_extract_mask_t) == sizeof(t_FmPcdKgExtractMask));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_extract_dflt_t) == sizeof(t_FmPcdKgExtractDflt));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_key_extract_and_hash_params_t) == sizeof(t_FmPcdKgKeyExtractAndHashParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_extracted_or_params_t) == sizeof(t_FmPcdKgExtractedOrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_scheme_counter_t) == sizeof(t_FmPcdKgSchemeCounter));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_plcr_profile_t) == sizeof(t_FmPcdKgPlcrProfile));
#if (DPAA_VERSION >= 11)
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_storage_profile_t) == sizeof(t_FmPcdKgStorageProfile));
#endif
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_cc_t) == sizeof(t_FmPcdKgCc));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_scheme_params_t) == sizeof(t_FmPcdKgSchemeParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_cc_params_t) == sizeof(t_FmPcdCcNextCcParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_plcr_params_t) == sizeof(t_FmPcdCcNextPlcrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_enqueue_params_t) == sizeof(t_FmPcdCcNextEnqueueParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_kg_params_t) == sizeof(t_FmPcdCcNextKgParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_next_engine_params_t) == sizeof(t_FmPcdCcNextEngineParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_key_params_t) == sizeof(t_FmPcdCcKeyParams));
    ASSERT_COND(sizeof(ioc_keys_params_t) == sizeof(t_KeysParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_node_params_t) == sizeof(t_FmPcdCcNodeParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_hash_table_params_t) == sizeof(t_FmPcdHashTableParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_grp_params_t) == sizeof(t_FmPcdCcGrpParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_cc_tree_params_t) == sizeof(t_FmPcdCcTreeParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_byte_rate_mode_param_t) == sizeof(t_FmPcdPlcrByteRateModeParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_non_passthrough_alg_param_t) == sizeof(t_FmPcdPlcrNonPassthroughAlgParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_next_engine_params_u) == sizeof(u_FmPcdPlcrNextEngineParams));
    /*fm_pcd_port_params_t : private */
    ASSERT_COND(sizeof(ioc_fm_pcd_plcr_profile_params_t) == sizeof(t_FmPcdPlcrProfileParams) + sizeof(void *));
    /*ioc_fm_pcd_cc_tree_modify_next_engine_params_t : private */

#ifdef FM_CAPWAP_SUPPORT
#error TODO: unsupported feature
/*
    ASSERT_COND(sizeof(TODO) == sizeof(t_FmPcdManipHdrInsrtByTemplateParams));
    ASSERT_COND(sizeof(TODO) == sizeof(t_CapwapFragmentationParams));
    ASSERT_COND(sizeof(TODO) == sizeof(t_CapwapReassemblyParams));
    ASSERT_COND(sizeof(TODO) == sizeof(t_FmPcdManipFragOrReasmParams));
    ASSERT_COND(sizeof(TODO) == sizeof(t_FmPcdManipHdrRmvByHdrParams));
*/
#endif

    /*ioc_fm_pcd_cc_node_modify_next_engine_params_t : private */
    /*ioc_fm_pcd_cc_node_remove_key_params_t : private */
    /*ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t : private */
    /*ioc_fm_pcd_cc_node_modify_key_params_t : private */
    /*ioc_fm_manip_hdr_info_t : private */
    /*ioc_fm_pcd_hash_table_set_t : private */
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_ip_params_t) == sizeof(t_FmPcdManipFragIpParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_ip_params_t) == sizeof(t_FmPcdManipReassemIpParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_special_offload_ipsec_params_t) == sizeof(t_FmPcdManipSpecialOffloadIPSecParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_special_offload_params_t) == sizeof(t_FmPcdManipSpecialOffloadParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_rmv_generic_params_t) == sizeof(t_FmPcdManipHdrRmvGenericParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_generic_params_t) == sizeof(t_FmPcdManipHdrInsrtGenericParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_insrt_params_t) == sizeof(t_FmPcdManipHdrInsrtParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_rmv_params_t) == sizeof(t_FmPcdManipHdrRmvParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_hdr_params_t) == sizeof(t_FmPcdManipHdrParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_params_t) == sizeof(t_FmPcdManipFragParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_params_t) == sizeof(t_FmPcdManipReassemParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_params_t) == sizeof(t_FmPcdManipParams) + sizeof(void *));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_ip_stats_t) == sizeof(t_FmPcdManipReassemIpStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_ip_stats_t) == sizeof(t_FmPcdManipFragIpStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_reassem_stats_t) == sizeof(t_FmPcdManipReassemStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_frag_stats_t) == sizeof(t_FmPcdManipFragStats));
    ASSERT_COND(sizeof(ioc_fm_pcd_manip_stats_t) == sizeof(t_FmPcdManipStats));
#if DPAA_VERSION >= 11
    ASSERT_COND(sizeof(ioc_fm_pcd_frm_replic_group_params_t) == sizeof(t_FmPcdFrmReplicGroupParams));
#endif

    /* fm_port_ext.h == fm_port_ioctls.h */
    ASSERT_COND(sizeof(ioc_fm_port_rate_limit_t) == sizeof(t_FmPortRateLimit));
    ASSERT_COND(sizeof(ioc_fm_port_pcd_params_t) == sizeof(t_FmPortPcdParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_kg_scheme_select_t) == sizeof(t_FmPcdKgSchemeSelect));
    ASSERT_COND(sizeof(ioc_fm_pcd_port_schemes_params_t) == sizeof(t_FmPcdPortSchemesParams));
    ASSERT_COND(sizeof(ioc_fm_pcd_prs_start_t) == sizeof(t_FmPcdPrsStart));

    return;
}

#define ASSERT_IOC_NET_ENUM(def) ASSERT_COND((unsigned long)e_IOC_NET_##def == (unsigned long)def)

void LnxWrpPCDIOCTLEnumChecking(void)
{
    /* net_ext.h == net_ioctls.h : sampling checks */
    ASSERT_IOC_NET_ENUM(HEADER_TYPE_MACSEC);
    ASSERT_IOC_NET_ENUM(HEADER_TYPE_PPP);
    ASSERT_IOC_NET_ENUM(MAX_HEADER_TYPE_COUNT);

    /* fm_ext.h == fm_ioctls.h */
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_TYPE_DUMMY == (unsigned long)e_FM_PORT_TYPE_DUMMY);
    ASSERT_COND((unsigned long)e_IOC_EX_MURAM_ECC == (unsigned long)e_FM_EX_MURAM_ECC);
    ASSERT_COND((unsigned long)e_IOC_FM_COUNTERS_DEQ_CONFIRM == (unsigned long)e_FM_COUNTERS_DEQ_CONFIRM);

    /* fm_pcd_ext.h == fm_pcd_ioctls.h */
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES == (unsigned long)e_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PRS_EXCEPTION_SINGLE_ECC == (unsigned long)e_FM_PCD_PRS_EXCEPTION_SINGLE_ECC);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PRS == (unsigned long)e_FM_PCD_PRS);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_EXTRACT_FULL_FIELD == (unsigned long)e_FM_PCD_EXTRACT_FULL_FIELD);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_EXTRACT_FROM_FLOW_ID == (unsigned long)e_FM_PCD_EXTRACT_FROM_FLOW_ID);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_KG_EXTRACT_PORT_PRIVATE_INFO == (unsigned long)e_FM_PCD_KG_EXTRACT_PORT_PRIVATE_INFO);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_KG_DFLT_ILLEGAL == (unsigned long)e_FM_PCD_KG_DFLT_ILLEGAL);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_KG_GENERIC_NOT_FROM_DATA == (unsigned long)e_FM_PCD_KG_GENERIC_NOT_FROM_DATA);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_HDR_INDEX_LAST == (unsigned long)e_FM_PCD_HDR_INDEX_LAST);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_SHARED == (unsigned long)e_FM_PCD_PLCR_SHARED);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_RFC_4115 == (unsigned long)e_FM_PCD_PLCR_RFC_4115);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_COLOR_AWARE == (unsigned long)e_FM_PCD_PLCR_COLOR_AWARE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_OVERRIDE == (unsigned long)e_FM_PCD_PLCR_OVERRIDE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_FULL_FRM_LEN == (unsigned long)e_FM_PCD_PLCR_FULL_FRM_LEN);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_ROLLBACK_FULL_FRM_LEN == (unsigned long)e_FM_PCD_PLCR_ROLLBACK_FULL_FRM_LEN);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_PACKET_MODE == (unsigned long)e_FM_PCD_PLCR_PACKET_MODE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_DROP_FRAME == (unsigned long)e_FM_PCD_DROP_FRAME);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_PLCR_PROFILE_RECOLOURED_RED_PACKET_TOTAL_COUNTER == (unsigned long)e_FM_PCD_PLCR_PROFILE_RECOLOURED_RED_PACKET_TOTAL_COUNTER);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_ACTION_INDEXED_LOOKUP == (unsigned long)e_FM_PCD_ACTION_INDEXED_LOOKUP);
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR == (unsigned long)e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR);
#if !defined(FM_CAPWAP_SUPPORT)
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_INSRT_GENERIC == (unsigned long)e_FM_PCD_MANIP_INSRT_GENERIC);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_RMV_GENERIC == (unsigned long)e_FM_PCD_MANIP_RMV_GENERIC);
#else
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_INSRT_BY_TEMPLATE == (unsigned long)e_FM_PCD_MANIP_INSRT_BY_TEMPLATE);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_RMV_BY_HDR == (unsigned long)e_FM_PCD_MANIP_RMV_BY_HDR);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_RMV_BY_HDR_FROM_START == (unsigned long)e_FM_PCD_MANIP_RMV_BY_HDR_FROM_START);
#endif
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_TIME_OUT_BETWEEN_FRAG == (unsigned long)e_FM_PCD_MANIP_TIME_OUT_BETWEEN_FRAG);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_EIGHT_WAYS_HASH == (unsigned long)e_FM_PCD_MANIP_EIGHT_WAYS_HASH);

#ifdef FM_CAPWAP_SUPPORT
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_STATS_PER_FLOWID == (unsigned long)e_FM_PCD_STATS_PER_FLOWID);
#endif
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_SPECIAL_OFFLOAD == (unsigned long)e_FM_PCD_MANIP_SPECIAL_OFFLOAD);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_CC_STATS_MODE_FRAME == (unsigned long)e_FM_PCD_CC_STATS_MODE_FRAME);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_CONTINUE_WITHOUT_FRAG == (unsigned long)e_FM_PCD_MANIP_CONTINUE_WITHOUT_FRAG);
    ASSERT_COND((unsigned long)e_IOC_FM_PCD_MANIP_SPECIAL_OFFLOAD_IPSEC == (unsigned long)e_FM_PCD_MANIP_SPECIAL_OFFLOAD_IPSEC);

    /* fm_port_ext.h == fm_port_ioctls.h */
#if !defined(FM_CAPWAP_SUPPORT)
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR == (unsigned long)e_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR);
#else
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR == (unsigned long)e_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR);
#endif
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_COUNTERS_DEQ_CONFIRM == (unsigned long)e_FM_PORT_COUNTERS_DEQ_CONFIRM);
    ASSERT_COND((unsigned long)e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8 == (unsigned long)e_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8);

    return;
}

static t_Error LnxwrpFmPcdIOCTL(t_LnxWrpFmDev *p_LnxWrpFmDev, unsigned int cmd, unsigned long arg, bool compat)
{
    t_Error err = E_READ_FAILED;

/*
    Status: PCD API to fmlib (file: drivers/net/dpa/NetCommSw/inc/Peripherals/fm_pcd_ext.h):

Status: not exported, should be thru sysfs
    FM_PCD_KgSchemeGetCounter
    FM_PCD_KgSchemeSetCounter
    FM_PCD_PlcrProfileGetCounter
    FM_PCD_PlcrProfileSetCounter

Status: not exported
    FM_PCD_MatchTableFindNRemoveKey
    FM_PCD_MatchTableFindNModifyNextEngine
    FM_PCD_MatchTableFindNModifyKeyAndNextEngine
    FM_PCD_MatchTableFindNModifyKey
    FM_PCD_MatchTableGetIndexedHashBucket
    FM_PCD_MatchTableGetNextEngine
    FM_PCD_MatchTableGetKeyCounter

Status: Exported, Not tested (no test available)
    FM_PCD_HashTableSet
    FM_PCD_HashTableDelete
    FM_PCD_HashTableAddKey
    FM_PCD_HashTableRemoveKey

Status: not exported, would be nice to have
    FM_PCD_HashTableModifyNextEngine
    FM_PCD_HashTableModifyMissNextEngine
    FM_PCD_HashTableGetMissNextEngine
    FM_PCD_ManipGetStatistics

Status: not exported
#if DPAA_VERSION >= 3
    FM_PCD_FrmReplicSetGroup
    FM_PCD_FrmReplicDeleteGroup
    FM_PCD_FrmReplicAddMember
    FM_PCD_FrmReplicRemoveMember
#endif

Status: feature not supported
#ifdef FM_CAPWAP_SUPPORT
#error unsported feature
    FM_PCD_StatisticsSetNode
#endif

*/
    _fm_ioctl_dbg("cmd:0x%08x(type:0x%02x, nr:%u).\n",
        cmd, _IOC_TYPE(cmd), _IOC_NR(cmd) - 20);

    switch (cmd)
    {
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_PRS_LOAD_SW_COMPAT:
#endif
        case FM_PCD_IOC_PRS_LOAD_SW:
        {
            ioc_fm_pcd_prs_sw_params_t *param;
            uint8_t                    *p_code;

            param = (ioc_fm_pcd_prs_sw_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_prs_sw_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_prs_sw_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_prs_sw_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_prs_sw_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_prs_sw_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_prs_sw_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_prs_sw_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_prs_sw_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_fm_pcd_prs_sw(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_prs_sw_params_t *)arg,
                            sizeof(ioc_fm_pcd_prs_sw_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            if (!param->p_code || !param->size)
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, err, NO_MSG);
            }

            p_code = (uint8_t *) XX_Malloc(param->size);
            if (!p_code)
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, err, NO_MSG);
            }

            memset(p_code, 0, param->size);
            if (copy_from_user(p_code, param->p_code, param->size)) {
                XX_Free(p_code);
                XX_Free(param);
                RETURN_ERROR(MINOR, err, NO_MSG);
            }

            param->p_code = p_code;

            err = FM_PCD_PrsLoadSw(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdPrsSwParams*)param);

            XX_Free(p_code);
            XX_Free(param);
            break;
        }

        case FM_PCD_IOC_SET_ADVANCED_OFFLOAD_SUPPORT:
            return  FM_PCD_SetAdvancedOffloadSupport(p_LnxWrpFmDev->h_PcdDev);

        case FM_PCD_IOC_ENABLE:
            return FM_PCD_Enable(p_LnxWrpFmDev->h_PcdDev);

        case FM_PCD_IOC_DISABLE:
            return FM_PCD_Disable(p_LnxWrpFmDev->h_PcdDev);

        case FM_PCD_IOC_FORCE_INTR:
        {
            int exception;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(exception, (int *) compat_ptr(arg)))
                    break;
            }
            else
#endif
            {
               if (get_user(exception, (int *)arg))
                   break;
            }

            return FM_PCD_ForceIntr(p_LnxWrpFmDev->h_PcdDev, (e_FmPcdExceptions)exception);
        }

        case FM_PCD_IOC_SET_EXCEPTION:
        {
            ioc_fm_pcd_exception_params_t *param;

            param = (ioc_fm_pcd_exception_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_exception_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_exception_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_pcd_exception_params_t *)compat_ptr(arg),
                                    sizeof(ioc_fm_pcd_exception_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_exception_params_t *)arg,
                                    sizeof(ioc_fm_pcd_exception_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev, param->exception, param->enable);

            XX_Free(param);
            break;
        }

        case FM_PCD_IOC_KG_SET_ADDITIONAL_DATA_AFTER_PARSING:
        {
            uint8_t payloadOffset;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(payloadOffset, (uint8_t*) compat_ptr(arg)))
                    break;
            }
            else
#endif
            {
                if (get_user(payloadOffset, (uint8_t*) arg))
                    break;
            }

            return FM_PCD_KgSetAdditionalDataAfterParsing(p_LnxWrpFmDev->h_PcdDev, payloadOffset);
        }

        case FM_PCD_IOC_KG_SET_DFLT_VALUE:
        {
            ioc_fm_pcd_kg_dflt_value_params_t *param;

            param = (ioc_fm_pcd_kg_dflt_value_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_kg_dflt_value_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_dflt_value_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_dflt_value_params_t *)compat_ptr(arg),
                                    sizeof(ioc_fm_pcd_kg_dflt_value_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_dflt_value_params_t *)arg,
                                    sizeof(ioc_fm_pcd_kg_dflt_value_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_KgSetDfltValue(p_LnxWrpFmDev->h_PcdDev, param->valueId, param->value);

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_SET_NET_ENV_CHARACTERISTICS_COMPAT:
#endif
        case FM_PCD_IOC_SET_NET_ENV_CHARACTERISTICS:
        {
            ioc_fm_pcd_net_env_params_t  *param;

            param = (ioc_fm_pcd_net_env_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_net_env_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_net_env_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_net_env_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_net_env_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_net_env_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_net_env_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_net_env_params_t *) compat_ptr(arg),
                                    sizeof(ioc_compat_fm_pcd_net_env_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_net_env(compat_param, param, COMPAT_US_TO_K);
                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_net_env_params_t *) arg,
                            sizeof(ioc_fm_pcd_net_env_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            param->id = FM_PCD_NetEnvCharacteristicsSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdNetEnvParams*)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_net_env_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_net_env_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_net_env_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_net_env_params_t));
                compat_copy_fm_pcd_net_env(compat_param, param, COMPAT_K_TO_US);
                if (param->id && !copy_to_user((ioc_compat_fm_pcd_net_env_params_t *) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_net_env_params_t)))
                    err = E_OK;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_net_env_params_t *)arg, param, sizeof(ioc_fm_pcd_net_env_params_t)))
                    err = E_OK;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_DELETE_NET_ENV_CHARACTERISTICS_COMPAT:
#endif
        case FM_PCD_IOC_DELETE_NET_ENV_CHARACTERISTICS:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_NetEnvCharacteristicsDelete(id.obj);
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_KG_SET_SCHEME_COMPAT:
#endif
        case FM_PCD_IOC_KG_SET_SCHEME:
        {
            ioc_fm_pcd_kg_scheme_params_t *param;

            param = (ioc_fm_pcd_kg_scheme_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_kg_scheme_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_scheme_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_params_t *compat_param = NULL;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));

                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_kg_scheme_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_kg_scheme_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_kg_scheme(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_scheme_params_t *)arg,
                            sizeof(ioc_fm_pcd_kg_scheme_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            param->id = FM_PCD_KgSchemeSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdKgSchemeParams*)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_params_t));
                compat_copy_fm_pcd_kg_scheme(compat_param, param, COMPAT_K_TO_US);
                if (param->id && !copy_to_user((ioc_compat_fm_pcd_kg_scheme_params_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_kg_scheme_params_t)))
                    err = E_OK;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_kg_scheme_params_t *)arg,
                            param,
                            sizeof(ioc_fm_pcd_kg_scheme_params_t)))
                    err = E_OK;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_KG_DEL_SCHEME_COMPAT:
#endif
        case FM_PCD_IOC_KG_DEL_SCHEME:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_KgSchemeDelete(id.obj);
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_SET_NODE_COMPAT:
#endif
        case FM_PCD_IOC_CC_SET_NODE:
        {
            ioc_fm_pcd_cc_node_params_t *param;
            uint8_t                     *keys;
            uint8_t                     *masks;
            int                         i,k;

            param = (ioc_fm_pcd_cc_node_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_params_t) +
                    2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_params_t) +
                    2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);

            keys = (uint8_t *) (param + 1);
            masks = keys + IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                        2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                        2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);

                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_params_t *)arg, sizeof(ioc_fm_pcd_cc_node_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            ASSERT_COND(param->keys_params.num_of_keys <= IOC_FM_PCD_MAX_NUM_OF_KEYS);
            ASSERT_COND(param->keys_params.key_size <= IOC_FM_PCD_MAX_SIZE_OF_KEY);

            /* support for indexed lookup */
            if( !(param->extract_cc_params.type == e_IOC_FM_PCD_EXTRACT_NON_HDR &&
                  param->extract_cc_params.extract_params.extract_non_hdr.src == e_IOC_FM_PCD_EXTRACT_FROM_HASH &&
                  param->extract_cc_params.extract_params.extract_non_hdr.action == e_IOC_FM_PCD_ACTION_INDEXED_LOOKUP))
            {
                for (i=0, k=0;
                     i < param->keys_params.num_of_keys;
                     i++, k += IOC_FM_PCD_MAX_SIZE_OF_KEY)
                {
                    if (param->keys_params.key_params[i].p_key &&
                            param->keys_params.key_size)
                    {
                        if (copy_from_user(&keys[k],
                                    param->keys_params.key_params[i].p_key,
                                    param->keys_params.key_size))
                        {
                            XX_Free(param);
                            RETURN_ERROR(MINOR, err, NO_MSG);
                        }

                        param->keys_params.key_params[i].p_key = &keys[k];
                    }
                    /* else
                       param->keys_params.key_params[i].p_key = NULL;
                       was taken care of by memset(0) above */

                    if (param->keys_params.key_params[i].p_mask)
                    {
                        if (copy_from_user(&masks[k],
                                    param->keys_params.key_params[i].p_mask,
                                    param->keys_params.key_size))
                        {
                            XX_Free(param);
                            RETURN_ERROR(MINOR, err, NO_MSG);
                        }

                        param->keys_params.key_params[i].p_mask = &masks[k];
                    }
                    /* else
                       param->keys_params.key_params[i].p_mask = NULL;
                       was taken care of by memset(0) above */
                }
            }

            param->id = FM_PCD_MatchTableSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdCcNodeParams*)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_params_t *compat_param;
                compat_param = (ioc_compat_fm_pcd_cc_node_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                        2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                /* setup user space structure */
                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_params_t) +
                        2 * IOC_FM_PCD_MAX_NUM_OF_KEYS * IOC_FM_PCD_MAX_SIZE_OF_KEY);
                compat_copy_fm_pcd_cc_node(compat_param, param, COMPAT_K_TO_US);
                compat_param->id = compat_add_ptr2id(param->id);

                if (param->id && !copy_to_user((ioc_compat_fm_pcd_cc_node_params_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_node_params_t)))
                    err = E_OK;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_cc_node_params_t *)arg, param, sizeof(ioc_fm_pcd_cc_node_params_t)))
                    err = E_OK;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_DELETE_NODE_COMPAT:
#endif
        case FM_PCD_IOC_CC_DELETE_NODE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_get_id2ptr(compat_id.obj);
                compat_del_ptr2id(id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_MatchTableDelete(id.obj);
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_BUILD_TREE_COMPAT:
#endif
        case FM_PCD_IOC_CC_BUILD_TREE:
        {
            ioc_fm_pcd_cc_tree_params_t *param;

            param = (ioc_fm_pcd_cc_tree_params_t *) XX_Malloc(sizeof(ioc_fm_pcd_cc_tree_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_tree_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tree_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tree_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_tree_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tree_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_tree(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_tree_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tree_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            param->id = FM_PCD_CcRootBuild(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdCcTreeParams*)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tree_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tree_params_t *) XX_Malloc(sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tree_params_t));
                compat_add_ptr2id(param->id);
                param->id = (void *)(uint64_t)compat_get_ptr2id(param->id);
                compat_copy_fm_pcd_cc_tree(compat_param, param, COMPAT_K_TO_US);

                if (param->id && !copy_to_user((ioc_compat_fm_pcd_cc_tree_params_t *)compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_cc_tree_params_t)))
                    err = E_OK;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_cc_tree_params_t *)arg, param, sizeof(ioc_fm_pcd_cc_tree_params_t)))
                    err = E_OK;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_DELETE_TREE_COMPAT:
#endif
        case FM_PCD_IOC_CC_DELETE_TREE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_get_id2ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_CcRootDelete(id.obj);
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_PLCR_SET_PROFILE_COMPAT:
#endif
        case FM_PCD_IOC_PLCR_SET_PROFILE:
        {
            ioc_fm_pcd_plcr_profile_params_t *param;

            param = (ioc_fm_pcd_plcr_profile_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_plcr_profile_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_plcr_profile_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_plcr_profile_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_plcr_profile_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_fm_pcd_plcr_profile_params_t));
                if (copy_from_user(compat_param, (
                            ioc_compat_fm_pcd_plcr_profile_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_plcr_profile_params_t))) {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_plcr_profile(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_plcr_profile_params_t *)arg,
                                    sizeof(ioc_fm_pcd_plcr_profile_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            if (!param->modify &&
                (((t_FmPcdPlcrProfileParams*)param)->id.newParams.profileType != e_FM_PCD_PLCR_SHARED))
            {
                t_Handle h_Port;
                fm_pcd_port_params_t *port_params;

                port_params = (fm_pcd_port_params_t*) XX_Malloc(sizeof(fm_pcd_port_params_t));
                if (!port_params)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(port_params, 0, sizeof(fm_pcd_port_params_t));
                if (copy_from_user(port_params, (fm_pcd_port_params_t*)((t_FmPcdPlcrProfileParams*)param)->id.newParams.h_FmPort,
                            sizeof(fm_pcd_port_params_t)))
                {
                    XX_Free(port_params);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                switch(port_params->port_type)
                {
                    case (e_IOC_FM_PORT_TYPE_RX):
                        if (port_params->port_id < FM_MAX_NUM_OF_1G_RX_PORTS) {
                            h_Port = p_LnxWrpFmDev->rxPorts[port_params->port_id].h_Dev;
                            break;
                        }
                        goto invalid_port_id;

                    case (e_IOC_FM_PORT_TYPE_RX_10G):
                        if (port_params->port_id < FM_MAX_NUM_OF_10G_RX_PORTS) {
                            h_Port = p_LnxWrpFmDev->rxPorts[port_params->port_id + FM_MAX_NUM_OF_1G_RX_PORTS].h_Dev;
                            break;
                        }
                        goto invalid_port_id;

                    case (e_FM_PORT_TYPE_OH_OFFLINE_PARSING):
                        if (port_params->port_id && port_params->port_id < FM_MAX_NUM_OF_OH_PORTS) {
                            h_Port = p_LnxWrpFmDev->opPorts[port_params->port_id - 1].h_Dev;
                            break;
                        }
                        goto invalid_port_id;

                    default:
invalid_port_id:
                        XX_Free(port_params);
                        XX_Free(param);
                        RETURN_ERROR(MINOR, E_INVALID_SELECTION, NO_MSG);
                }

                ((t_FmPcdPlcrProfileParams*)param)->id.newParams.h_FmPort = h_Port;
                XX_Free(port_params);
            }

            param->id = FM_PCD_PlcrProfileSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdPlcrProfileParams*)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_plcr_profile_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_plcr_profile_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_plcr_profile_params_t));
                compat_copy_fm_pcd_plcr_profile(compat_param, param, COMPAT_K_TO_US);
                if (param->id && !copy_to_user((ioc_compat_fm_pcd_plcr_profile_params_t *) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_plcr_profile_params_t)))
                    err = E_OK;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_plcr_profile_params_t *)arg, param, sizeof(ioc_fm_pcd_plcr_profile_params_t)))
                    err = E_OK;
            }

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_PLCR_DEL_PROFILE_COMPAT:
#endif
        case FM_PCD_IOC_PLCR_DEL_PROFILE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_PlcrProfileDelete(id.obj);
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_TREE_MODIFY_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_CC_TREE_MODIFY_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_tree_modify_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_tree_modify_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_tree_modify_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_tree_modify_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_fm_pcd_cc_tree_modify_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_tree_modify_next_engine_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_tree_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_CcRootModifyNextEngine(param->id,
                                                param->grp_indx,
                                                param->indx,
                                                (t_FmPcdCcNextEngineParams*)(&param->cc_next_engine_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_NODE_MODIFY_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_CC_NODE_MODIFY_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_node_modify_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_next_engine_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableModifyNextEngine(param->id,
                    param->key_indx,
                    (t_FmPcdCcNextEngineParams*)(&param->cc_next_engine_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_NODE_MODIFY_MISS_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_CC_NODE_MODIFY_MISS_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_node_modify_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *) compat_ptr(arg),
                                    sizeof(ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_next_engine_params_t *) arg,
                                    sizeof(ioc_fm_pcd_cc_node_modify_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableModifyMissNextEngine(param->id,
                    (t_FmPcdCcNextEngineParams*)(&param->cc_next_engine_params));

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_NODE_REMOVE_KEY_COMPAT:
#endif
        case FM_PCD_IOC_CC_NODE_REMOVE_KEY:
        {
            ioc_fm_pcd_cc_node_remove_key_params_t *param;

            param = (ioc_fm_pcd_cc_node_remove_key_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_remove_key_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_remove_key_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_remove_key_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_remove_key_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_remove_key_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_remove_key_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_remove_key_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_remove_key_params_t)))
                {
                    XX_Free(param);
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                param->id = compat_ptr(compat_param->id);
                param->key_indx = compat_param->key_indx;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_remove_key_params_t *) arg,
                            sizeof(ioc_fm_pcd_cc_node_remove_key_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableRemoveKey(param->id, param->key_indx);

            XX_Free(param);
            break;
        }
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_NODE_ADD_KEY_COMPAT:
#endif
        case FM_PCD_IOC_CC_NODE_ADD_KEY:
        {
            ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *param;
            uint8_t *p_key = NULL, *p_mask = NULL;

            param = (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(param);
                    XX_Free(compat_param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_key_and_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)arg,
                                    sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            /* copy key & mask: p_mask is optional! */
            p_key = (uint8_t *) XX_Malloc(2 * param->key_size);
            if(!p_key)
                RETURN_ERROR(MINOR, err, NO_MSG);

            p_mask = p_key + param->key_size;

            if (param->key_params.p_key && copy_from_user(p_key, param->key_params.p_key, param->key_size))
            {
                XX_Free(p_key);
                RETURN_ERROR(MINOR, err, NO_MSG);
            }
            param->key_params.p_key = p_key;

            if (param->key_params.p_mask && copy_from_user(p_mask, param->key_params.p_mask,param->key_size))
            {
                XX_Free(p_mask);
                RETURN_ERROR(MINOR, err, NO_MSG);
            }
            param->key_params.p_mask = p_mask;

            err = FM_PCD_MatchTableAddKey(param->id,
                    param->key_indx,
                    param->key_size,
                    (t_FmPcdCcKeyParams*)(&param->key_params));

            XX_Free(param);
            kfree(p_key);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_NODE_MODIFY_KEY_AND_NEXT_ENGINE_COMPAT:
#endif
        case FM_PCD_IOC_CC_NODE_MODIFY_KEY_AND_NEXT_ENGINE:
        {
            ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *param;

            param = (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (!compat_param)
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_key_and_next_engine(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *)arg,
                            sizeof(ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            err = FM_PCD_MatchTableModifyKeyAndNextEngine(param->id,
                    param->key_indx,
                    param->key_size,
                    (t_FmPcdCcKeyParams*)(&param->key_params));

            XX_Free(param);
            break;
        }
#if defined(CONFIG_COMPAT)
#error TODO: compat ioctl call not implemented!
        case FM_PCD_IOC_HASH_TABLE_SET_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_SET:
        {
            ioc_fm_pcd_hash_table_params_t *param;

            param = kmalloc(sizeof(*param), GFP_KERNEL);
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                /* TODO: return -ENOMEM;*/

            memset(param, 0, sizeof(*param)) ;

#if defined(CONFIG_COMPAT)
#warning TODO: compat ioctl call not implemented!
            if (compat)
            {
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_hash_table_params_t *)arg,
                                    sizeof(ioc_fm_pcd_hash_table_params_t)))
                    RETURN_ERROR(MINOR, err, NO_MSG);
            }

            param->id = FM_PCD_HashTableSet(p_LnxWrpFmDev->h_PcdDev, (t_FmPcdHashTableParams *) param);

#if defined(CONFIG_COMPAT)
#warning TODO: compat ioctl call not implemented!
            if (compat)
            {
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_hash_table_params_t *)arg,
                                        param, sizeof(ioc_fm_pcd_hash_table_params_t)))
                    err = E_OK;
            }

            kfree(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_HASH_TABLE_DELETE_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_DELETE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0, sizeof(ioc_fm_obj_t));
#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    RETURN_ERROR(MINOR, err, NO_MSG);

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_HashTableDelete(id.obj);
        }
#if defined(CONFIG_COMPAT)
#warning TODO: compat ioctl call not implemented!
        case FM_PCD_IOC_HASH_TABLE_ADD_KEY_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_ADD_KEY:
        {
            ioc_fm_pcd_hash_table_add_key_params_t *param = NULL;

            param = kmalloc(sizeof(*param), GFP_KERNEL);
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                /* TODO: return -ENOMEM;*/

            memset(param, 0, sizeof(*param)) ;

#if defined(CONFIG_COMPAT)
#warning TODO: compat ioctl call not implemented!
            if (compat)
            {
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_hash_table_add_key_params_t *)arg,
                                    sizeof(ioc_fm_pcd_hash_table_add_key_params_t)))
                    RETURN_ERROR(MINOR, err, NO_MSG);
            }

            err = FM_PCD_HashTableAddKey(param->p_hash_tbl, param->key_size, (t_FmPcdCcKeyParams  *)param->p_key_params);

            kfree(param);
            break;
        }

#if defined(CONFIG_COMPAT)
#warning TODO: compat ioctl call not implemented!
        case FM_PCD_IOC_HASH_TABLE_REMOVE_KEY_COMPAT:
#endif
        case FM_PCD_IOC_HASH_TABLE_REMOVE_KEY:
        {
            ioc_fm_pcd_hash_table_remove_key_params_t *param = NULL;

            param = kmalloc(sizeof(*param), GFP_KERNEL);
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                /* TODO: return -ENOMEM;*/

            memset(param, 0, sizeof(*param)) ;

#if defined(CONFIG_COMPAT)
#warning TODO: compat ioctl call not implemented!
            if (compat)
            {
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_hash_table_remove_key_params_t *)arg,
                                    sizeof(ioc_fm_pcd_hash_table_remove_key_params_t)))
                    RETURN_ERROR(MINOR, err, NO_MSG);
            }

            err = FM_PCD_HashTableRemoveKey(param->p_hash_tbl, param->key_size, param->p_key);

            kfree(param);
            break;
        }
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_CC_NODE_MODIFY_KEY_COMPAT:
#endif
        case FM_PCD_IOC_CC_NODE_MODIFY_KEY:
        {
            ioc_fm_pcd_cc_node_modify_key_params_t  *param;
            uint8_t                                 *key  = NULL;
            uint8_t                                 *mask = NULL;

            param = (ioc_fm_pcd_cc_node_modify_key_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_cc_node_modify_key_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_cc_node_modify_key_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_cc_node_modify_key_params_t  *compat_param;

                compat_param = (ioc_compat_fm_pcd_cc_node_modify_key_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_cc_node_modify_key_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_cc_node_modify_key_params_t));
                if (copy_from_user(compat_param, (ioc_compat_fm_pcd_cc_node_modify_key_params_t *)compat_ptr(arg),
                                    sizeof(ioc_compat_fm_pcd_cc_node_modify_key_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_copy_fm_pcd_cc_node_modify_key(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_cc_node_modify_key_params_t *)arg,
                                    sizeof(ioc_fm_pcd_cc_node_modify_key_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            if (!param->p_key)
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_NULL_POINTER, ("IOCTL FM PCD key"));
            }

            key = (uint8_t *) XX_Malloc(sizeof(uint8_t)*IOC_FM_PCD_MAX_SIZE_OF_KEY);
            if (!key)
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD key"));
            }

            memset(key, 0, sizeof(uint8_t)*IOC_FM_PCD_MAX_SIZE_OF_KEY);
            if (copy_from_user(key, param->p_key, param->key_size))
            {
                XX_Free(key);
                XX_Free(param);
                RETURN_ERROR(MINOR, err, NO_MSG);
            }

            param->p_key = key;

            if (param->p_mask)
            {
                mask = (uint8_t *) XX_Malloc(sizeof(uint8_t)*IOC_FM_PCD_MAX_SIZE_OF_KEY);
                if (!mask)
                {
                    if (key) XX_Free(key);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD mask"));
                }

                memset(mask, 0, sizeof(uint8_t)*IOC_FM_PCD_MAX_SIZE_OF_KEY);
                if (copy_from_user(mask, param->p_mask, param->key_size))
                {
                    XX_Free(mask);
                    if (key) XX_Free(key);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                param->p_mask = mask;
            }

            err = FM_PCD_MatchTableModifyKey(param->id,
                    param->key_indx,
                    param->key_size,
                    param->p_key,
                    param->p_mask);

            if (mask) XX_Free(mask);
            if (key)  XX_Free(key);

            XX_Free(param);
            break;
        }

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MANIP_SET_NODE_COMPAT:
#else
        case FM_PCD_IOC_MANIP_SET_NODE:
#endif
        {
            ioc_fm_pcd_manip_params_t *param;

            param = (ioc_fm_pcd_manip_params_t *) XX_Malloc(
                        sizeof(ioc_fm_pcd_manip_params_t));

            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_pcd_manip_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_manip_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_manip_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_manip_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_manip_params_t));
                if (copy_from_user(compat_param,
                            (ioc_compat_fm_pcd_manip_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_manip_params_t)))
                {
                    XX_Free(compat_param);
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }

                compat_fm_pcd_manip_set_node(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_manip_params_t *)arg,
                                            sizeof(ioc_fm_pcd_manip_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, err, NO_MSG);
                }
            }

            param->id = FM_PCD_ManipNodeSet(p_LnxWrpFmDev->h_PcdDev,
                            (t_FmPcdManipParams *)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_manip_params_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_manip_params_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_manip_params_t));
                if (!compat_param)
                {
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_manip_params_t));

                compat_fm_pcd_manip_set_node(compat_param, param, COMPAT_K_TO_US);
                compat_param->id = compat_add_ptr2id(param->id);

                if (param->id && !copy_to_user((ioc_compat_fm_pcd_manip_params_t *) compat_ptr(arg),
                            compat_param,
                            sizeof(ioc_compat_fm_pcd_manip_params_t)))
                    err = E_OK;

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (param->id && !copy_to_user((ioc_fm_pcd_manip_params_t *)arg,
                                        param, sizeof(ioc_fm_pcd_manip_params_t)))
                    err = E_OK;
            }

            XX_Free(param);
        }
        break;

#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_MANIP_DELETE_NODE_COMPAT:
#else
        case FM_PCD_IOC_MANIP_DELETE_NODE:
#endif
        {
            ioc_fm_obj_t id;

            memset(&id, 0, sizeof(ioc_fm_obj_t));
#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PCD_ManipNodeDelete(id.obj);
        }
        break;

#ifdef FM_CAPWAP_SUPPORT
#if defined(CONFIG_COMPAT)
        case FM_PCD_IOC_STATISTICS_SET_NODE_COMPAT:
#else
        case FM_PCD_IOC_STATISTICS_SET_NODE:
#endif
        {
            ioc_fm_pcd_stats_params_t param;
#warning "TODO"
            param->id = FM_PCD_StatisticsSetNode(p_LnxWrpFmDev->h_PcdDev,
                                (t_FmPcdStatsParams *)&param);
        }
        break;
#endif /* FM_CAPWAP_SUPPORT */

        default:
            RETURN_ERROR(MINOR, E_INVALID_SELECTION,
                ("invalid ioctl: cmd:0x%08x(type:0x%02x, nr:0x%02x.\n",
                cmd, _IOC_TYPE(cmd), _IOC_NR(cmd)));

            break;
    }

    return err;
}

t_Error LnxwrpFmIOCTL(t_LnxWrpFmDev *p_LnxWrpFmDev, unsigned int cmd, unsigned long arg, bool compat)
{
    t_Error err = E_READ_FAILED;

    switch (cmd)
    {
        case FM_IOC_SET_PORTS_BANDWIDTH:
        {
            ioc_fm_port_bandwidth_params *param;

            param = (ioc_fm_port_bandwidth_params*) XX_Malloc(sizeof(ioc_fm_port_bandwidth_params));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_port_bandwidth_params));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_bandwidth_params*)compat_ptr(arg), sizeof(ioc_fm_port_bandwidth_params)))
                {
                    XX_Free(param);
                    return err;
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_bandwidth_params*)arg, sizeof(ioc_fm_port_bandwidth_params)))
                {
                    XX_Free(param);
                    return err;
                }
            }

            err =  FM_SetPortsBandwidth(p_LnxWrpFmDev->h_Dev, (t_FmPortsBandwidthParams*) param);
            XX_Free(param);
            return err;
        }

        case FM_IOC_GET_REVISION:
        {
            ioc_fm_revision_info_t *param;

            param = (ioc_fm_revision_info_t *) XX_Malloc(sizeof(ioc_fm_revision_info_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            FM_GetRevision(p_LnxWrpFmDev->h_Dev, (t_FmRevisionInfo*)param);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_to_user((ioc_fm_revision_info_t *)compat_ptr(arg),
                            param,
                            sizeof(ioc_fm_revision_info_t)))
                    err = E_WRITE_FAILED;
                else
                    err = E_OK;
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_revision_info_t *)arg,
                            param,
                            sizeof(ioc_fm_revision_info_t)))
                    err = E_WRITE_FAILED;
                else
                    err = E_OK;
            }

            XX_Free(param);
            return err;
        }

        case FM_IOC_SET_COUNTER:
        {
            ioc_fm_counters_params_t *param;

            param = (ioc_fm_counters_params_t *) XX_Malloc(sizeof(ioc_fm_counters_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_counters_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)compat_ptr(arg), sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    return err;
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)arg, sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    return err;
                }
            }

            err = FM_ModifyCounter(p_LnxWrpFmDev->h_Dev, param->cnt, param->val);

            XX_Free(param);
            return err;
        }

        case FM_IOC_GET_COUNTER:
        {
            ioc_fm_counters_params_t *param;

            param = (ioc_fm_counters_params_t *) XX_Malloc(sizeof(ioc_fm_counters_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PCD"));

            memset(param, 0, sizeof(ioc_fm_counters_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)compat_ptr(arg), sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    return err;
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_counters_params_t *)arg, sizeof(ioc_fm_counters_params_t)))
                {
                    XX_Free(param);
                    return err;
                }
            }

            param->val = FM_GetCounter(p_LnxWrpFmDev->h_Dev, param->cnt);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_to_user((ioc_fm_counters_params_t *)compat_ptr(arg), param, sizeof(ioc_fm_counters_params_t)))
                    err = E_WRITE_FAILED;
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_counters_params_t *)arg, param, sizeof(ioc_fm_counters_params_t)))
                    err = E_WRITE_FAILED;
            }

            XX_Free(param);
            return err;
        }

        case FM_IOC_FORCE_INTR:
        {
            ioc_fm_exceptions param;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(param, (ioc_fm_exceptions*) compat_ptr(arg)))
                    break;
            }
            else
#endif
            {
                if (get_user(param, (ioc_fm_exceptions*)arg))
                    break;
            }

            return FM_ForceIntr(p_LnxWrpFmDev->h_Dev, (e_FmExceptions)param);
        }

        default:
            return LnxwrpFmPcdIOCTL(p_LnxWrpFmDev, cmd, arg, compat);
    }

    RETURN_ERROR(MINOR, E_INVALID_OPERATION, ("IOCTL FM"));
}

t_Error LnxwrpFmPortIOCTL(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev, unsigned int cmd, unsigned long arg, bool compat)
{
    t_Error err = E_READ_FAILED;

    _fm_ioctl_dbg("cmd:0x%08x(type:0x%02x, nr:%u).\n",
        cmd, _IOC_TYPE(cmd), _IOC_NR(cmd) - 50);

    switch (cmd)
    {
        case FM_PORT_IOC_DISABLE:
            FM_PORT_Disable(p_LnxWrpFmPortDev->h_Dev);
            return E_OK;

        case FM_PORT_IOC_ENABLE:
            FM_PORT_Enable(p_LnxWrpFmPortDev->h_Dev);
            return E_OK;

        case FM_PORT_IOC_SET_ERRORS_ROUTE:
        {
            ioc_fm_port_frame_err_select_t errs;

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(errs, (ioc_fm_port_frame_err_select_t*)compat_ptr(arg)))
                    break;
            }
            else
#endif
            {
                if (get_user(errs, (ioc_fm_port_frame_err_select_t*)arg))
                    break;
            }

            return FM_PORT_SetErrorsRoute(p_LnxWrpFmPortDev->h_Dev, (fmPortFrameErrSelect_t)errs);
        }

        case FM_PORT_IOC_SET_RATE_LIMIT:
        {
            ioc_fm_port_rate_limit_t *param;

            param = (ioc_fm_port_rate_limit_t *) XX_Malloc(sizeof(ioc_fm_port_rate_limit_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_rate_limit_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_rate_limit_t *)compat_ptr(arg), sizeof(ioc_fm_port_rate_limit_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_READ_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_rate_limit_t *)arg, sizeof(ioc_fm_port_rate_limit_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_READ_FAILED, NO_MSG);
                }
            }

            err =  FM_PORT_SetRateLimit(p_LnxWrpFmPortDev->h_Dev, (t_FmPortRateLimit *)param);

            XX_Free(param);
            return err;
        }

        case FM_PORT_IOC_REMOVE_RATE_LIMIT:
            FM_PORT_DeleteRateLimit(p_LnxWrpFmPortDev->h_Dev);
            return E_OK;

        case FM_PORT_IOC_ALLOC_PCD_FQIDS:
        {
            ioc_fm_port_pcd_fqids_params_t *param;

            if (!p_LnxWrpFmPortDev->pcd_owner_params.cba)
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("No one to listen on this PCD!!!"));

            param = (ioc_fm_port_pcd_fqids_params_t *) XX_Malloc(sizeof(ioc_fm_port_pcd_fqids_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_port_pcd_fqids_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_port_pcd_fqids_params_t *)compat_ptr(arg),
                                    sizeof(ioc_fm_port_pcd_fqids_params_t)))
                {
                    XX_Free(param);
                    return err;
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_port_pcd_fqids_params_t *)arg,
                                    sizeof(ioc_fm_port_pcd_fqids_params_t)))
                {
                    XX_Free(param);
                    return err;
                }
            }

            if (p_LnxWrpFmPortDev->pcd_owner_params.cba(p_LnxWrpFmPortDev->pcd_owner_params.dev,
                                                        param->num_fqids,
                                                        param->alignment,
                                                        &param->base_fqid))
            {
                XX_Free(param);
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("can't allocate fqids for PCD!!!"));
            }

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_to_user((ioc_fm_port_pcd_fqids_params_t *)compat_ptr(arg),
                                  param, sizeof(ioc_fm_port_pcd_fqids_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_to_user((ioc_fm_port_pcd_fqids_params_t *)arg,
                                  param, sizeof(ioc_fm_port_pcd_fqids_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            XX_Free(param);
            return E_OK;
        }

        case FM_PORT_IOC_FREE_PCD_FQIDS:
        {
            uint32_t base_fqid;

            if (!p_LnxWrpFmPortDev->pcd_owner_params.cbf)
                RETURN_ERROR(MINOR, E_INVALID_STATE, ("No one to listen on this PCD!!!"));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(base_fqid, (uint32_t*) compat_ptr(arg)))
                    break;
            }
            else
#endif
            {
                if (get_user(base_fqid, (uint32_t*)arg))
                    break;
            }

            if (p_LnxWrpFmPortDev->pcd_owner_params.cbf(p_LnxWrpFmPortDev->pcd_owner_params.dev, base_fqid))
                RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);

            return E_OK;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_SET_PCD_COMPAT:
#endif
        case FM_PORT_IOC_SET_PCD:
        {
            ioc_fm_port_pcd_params_t      *port_pcd_params;
            ioc_fm_port_pcd_prs_params_t  *port_pcd_prs_params;
            ioc_fm_port_pcd_cc_params_t   *port_pcd_cc_params;
            ioc_fm_port_pcd_kg_params_t   *port_pcd_kg_params;
            ioc_fm_port_pcd_plcr_params_t *port_pcd_plcr_params;

            long copy_fail = 0;

            port_pcd_params = (ioc_fm_port_pcd_params_t *) XX_Malloc(
                    sizeof(ioc_fm_port_pcd_params_t) +
                    sizeof(ioc_fm_port_pcd_prs_params_t) +
                    sizeof(ioc_fm_port_pcd_cc_params_t) +
                    sizeof(ioc_fm_port_pcd_kg_params_t) +
                    sizeof(ioc_fm_port_pcd_plcr_params_t));
            if (!port_pcd_params)
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(port_pcd_params, 0,
                    sizeof(ioc_fm_port_pcd_params_t) +
                    sizeof(ioc_fm_port_pcd_prs_params_t) +
                    sizeof(ioc_fm_port_pcd_cc_params_t) +
                    sizeof(ioc_fm_port_pcd_kg_params_t) +
                    sizeof(ioc_fm_port_pcd_plcr_params_t));

            port_pcd_prs_params  = (ioc_fm_port_pcd_prs_params_t *)  (port_pcd_params + 1);
            port_pcd_cc_params   = (ioc_fm_port_pcd_cc_params_t *)   (port_pcd_prs_params + 1);
            port_pcd_kg_params   = (ioc_fm_port_pcd_kg_params_t *)   (port_pcd_cc_params + 1);
            port_pcd_plcr_params = (ioc_fm_port_pcd_plcr_params_t *) (port_pcd_kg_params + 1);

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_port_pcd_params_t      *compat_port_pcd_params;
                ioc_fm_port_pcd_prs_params_t         *same_port_pcd_prs_params;
                ioc_compat_fm_port_pcd_cc_params_t   *compat_port_pcd_cc_params;
                ioc_compat_fm_port_pcd_kg_params_t   *compat_port_pcd_kg_params;
                ioc_compat_fm_port_pcd_plcr_params_t *compat_port_pcd_plcr_params;

                compat_port_pcd_params = (ioc_compat_fm_port_pcd_params_t *) XX_Malloc(
                                sizeof(ioc_compat_fm_port_pcd_params_t) +
                                sizeof(ioc_fm_port_pcd_prs_params_t) +
                                sizeof(ioc_compat_fm_port_pcd_cc_params_t) +
                                sizeof(ioc_compat_fm_port_pcd_kg_params_t) +
                                sizeof(ioc_compat_fm_port_pcd_plcr_params_t));
                if (!compat_port_pcd_params)
                {
                    XX_Free(port_pcd_params);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));
                }

                memset(compat_port_pcd_params, 0,
                        sizeof(ioc_compat_fm_port_pcd_params_t) +
                        sizeof(ioc_fm_port_pcd_prs_params_t) +
                        sizeof(ioc_compat_fm_port_pcd_cc_params_t) +
                        sizeof(ioc_compat_fm_port_pcd_kg_params_t) +
                        sizeof(ioc_compat_fm_port_pcd_plcr_params_t));
                same_port_pcd_prs_params    = (ioc_fm_port_pcd_prs_params_t *) (compat_port_pcd_params + 1);
                compat_port_pcd_cc_params   = (ioc_compat_fm_port_pcd_cc_params_t *) (same_port_pcd_prs_params + 1);
                compat_port_pcd_kg_params   = (ioc_compat_fm_port_pcd_kg_params_t *) (compat_port_pcd_cc_params + 1);
                compat_port_pcd_plcr_params = (ioc_compat_fm_port_pcd_plcr_params_t *) (compat_port_pcd_kg_params + 1);

                /* Pseudo-while */
                while (!(copy_fail = copy_from_user(compat_port_pcd_params,
                                        (ioc_compat_fm_port_pcd_params_t *)compat_ptr(arg),
                                        sizeof(ioc_compat_fm_port_pcd_params_t))))
                {
                    compat_copy_fm_port_pcd(compat_port_pcd_params, port_pcd_params, COMPAT_US_TO_K);

                    /* the prs member is the same, no compat structure...memcpy only */
                    if (port_pcd_params->p_prs_params && !copy_fail)
                    {
                        if(!(copy_fail = copy_from_user(same_port_pcd_prs_params,
                                port_pcd_params->p_prs_params,
                                sizeof(ioc_fm_port_pcd_prs_params_t))))
                        {
                            memcpy(port_pcd_prs_params, same_port_pcd_prs_params, sizeof(ioc_fm_port_pcd_prs_params_t));
                            port_pcd_params->p_prs_params = port_pcd_prs_params;
                        }
                        else
                            break;
                    }

                    if (port_pcd_params->p_cc_params && !copy_fail)
                    {
                        if(!(copy_fail = copy_from_user(compat_port_pcd_cc_params,
                                port_pcd_params->p_cc_params,
                                sizeof(ioc_compat_fm_port_pcd_cc_params_t))))
                        {
                            port_pcd_params->p_cc_params = port_pcd_cc_params;
                            port_pcd_params->p_cc_params->cc_tree_id = compat_get_id2ptr(compat_port_pcd_cc_params->cc_tree_id);
                        }
                        else
                            break;
                    }

                    if (port_pcd_params->p_kg_params && !copy_fail)
                    {
                        if(!(copy_fail = copy_from_user(compat_port_pcd_kg_params,
                                port_pcd_params->p_kg_params,
                                sizeof(ioc_compat_fm_port_pcd_kg_params_t))))
                        {
                            compat_copy_fm_port_pcd_kg(compat_port_pcd_kg_params, port_pcd_kg_params, COMPAT_US_TO_K);
                            port_pcd_params->p_kg_params = port_pcd_kg_params;
                        }
                        else
                            break;
                    }

                    if (port_pcd_params->p_plcr_params && !copy_fail)
                    {
                        if(!(copy_fail = copy_from_user(compat_port_pcd_plcr_params,
                                port_pcd_params->p_plcr_params,
                                sizeof(ioc_compat_fm_port_pcd_plcr_params_t))))
                        {
                            port_pcd_params->p_plcr_params = port_pcd_plcr_params;
                            port_pcd_params->p_plcr_params->plcr_profile_id = compat_ptr(compat_port_pcd_plcr_params->plcr_profile_id);
                        }
                    }

                    /* always run once! */
                    break;
                }

                XX_Free(compat_port_pcd_params);
            }
            else
#endif
            {
                /* Pseudo-while */
                while (!(copy_fail = copy_from_user(port_pcd_params,
                                        (ioc_fm_port_pcd_params_t *)arg,
                                        sizeof(ioc_fm_port_pcd_params_t))))
                {
                    if (port_pcd_params->p_prs_params && !copy_fail)
                    {
                        if (!(copy_fail = copy_from_user(port_pcd_prs_params,
                                port_pcd_params->p_prs_params,
                                sizeof(ioc_fm_port_pcd_prs_params_t))))
                            port_pcd_params->p_prs_params = port_pcd_prs_params;
                        else
                            break;
                    }

                    if (port_pcd_params->p_cc_params &&  !copy_fail)
                    {
                        if (!(copy_fail = copy_from_user(port_pcd_cc_params,
                                port_pcd_params->p_cc_params,
                                sizeof(ioc_fm_port_pcd_cc_params_t))))
                            port_pcd_params->p_cc_params = port_pcd_cc_params;
                        else
                            break;
                    }

                    if (port_pcd_params->p_kg_params && !copy_fail)
                    {
                        if (!(copy_fail = copy_from_user(port_pcd_kg_params,
                                port_pcd_params->p_kg_params,
                                sizeof(ioc_fm_port_pcd_kg_params_t))))
                            port_pcd_params->p_kg_params = port_pcd_kg_params;
                        else
                            break;
                    }

                    if (port_pcd_params->p_plcr_params && !copy_fail)
                    {
                        if (!(copy_fail = copy_from_user(port_pcd_plcr_params,
                                port_pcd_params->p_plcr_params,
                                sizeof(ioc_fm_port_pcd_plcr_params_t))))
                            port_pcd_params->p_plcr_params = port_pcd_plcr_params;
                    }

                    /* always run once! */
                    break;
                }
            }

            if (!copy_fail)
                err = FM_PORT_SetPCD(p_LnxWrpFmPortDev->h_Dev, (t_FmPortPcdParams*) port_pcd_params);
            else
                err = E_READ_FAILED;

            XX_Free(port_pcd_params);

            return err;
        }

        case FM_PORT_IOC_DELETE_PCD:
            return FM_PORT_DeletePCD(p_LnxWrpFmPortDev->h_Dev);

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME_COMPAT:
#endif
        case FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME:
        {
            ioc_fm_pcd_kg_scheme_select_t *param;

            param = (ioc_fm_pcd_kg_scheme_select_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_kg_scheme_select_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_pcd_kg_scheme_select_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_kg_scheme_select_t *compat_param;

                compat_param = (ioc_compat_fm_pcd_kg_scheme_select_t *) XX_Malloc(
                        sizeof(ioc_compat_fm_pcd_kg_scheme_select_t));
                if (!compat_param){
                    XX_Free(param);
                    RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));
                }

                memset(compat_param, 0, sizeof(ioc_compat_fm_pcd_kg_scheme_select_t));
                if (copy_from_user(compat_param,
                                   (ioc_compat_fm_pcd_kg_scheme_select_t *) compat_ptr(arg),
                                   sizeof(ioc_compat_fm_pcd_kg_scheme_select_t)))
                {
                    XX_Free(param);
                    XX_Free(compat_param);
                    RETURN_ERROR(MAJOR, E_READ_FAILED, NO_MSG);
                }

                compat_copy_fm_pcd_kg_scheme_select(compat_param, param, COMPAT_US_TO_K);

                XX_Free(compat_param);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_kg_scheme_select_t *)arg,
                                   sizeof(ioc_fm_pcd_kg_scheme_select_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_READ_FAILED, NO_MSG);
                }
            }

            err =  FM_PORT_PcdKgModifyInitialScheme(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdKgSchemeSelect *)param);

            XX_Free(param);
            return err;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE_COMPAT:
#endif
        case FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat) {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PORT_PcdPlcrModifyInitialProfile(p_LnxWrpFmPortDev->h_Dev, id.obj);
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_KG_BIND_SCHEMES_COMPAT:
#endif
        case FM_PORT_IOC_PCD_KG_BIND_SCHEMES:
        {
            ioc_fm_pcd_port_schemes_params_t *param;

            param = (ioc_fm_pcd_port_schemes_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_port_schemes_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0 , sizeof(ioc_fm_pcd_port_schemes_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_port_schemes_params_t compat_param;

                if (copy_from_user(&compat_param,
                            (ioc_compat_fm_pcd_port_schemes_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_port_schemes_params_t)))
                    break;

                compat_copy_fm_pcd_kg_schemes_params(&compat_param, param, COMPAT_US_TO_K);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_port_schemes_params_t *) arg,
                            sizeof(ioc_fm_pcd_port_schemes_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PORT_PcdKgBindSchemes(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdPortSchemesParams *)param);

            XX_Free(param);
            return err;
        }

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES_COMPAT:
#endif
        case FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES:
        {
            ioc_fm_pcd_port_schemes_params_t *param;

            param = (ioc_fm_pcd_port_schemes_params_t *) XX_Malloc(
                    sizeof(ioc_fm_pcd_port_schemes_params_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0 , sizeof(ioc_fm_pcd_port_schemes_params_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_pcd_port_schemes_params_t compat_param;

                if (copy_from_user(&compat_param,
                            (ioc_compat_fm_pcd_port_schemes_params_t *) compat_ptr(arg),
                            sizeof(ioc_compat_fm_pcd_port_schemes_params_t)))
                    break;

                compat_copy_fm_pcd_kg_schemes_params(&compat_param, param, COMPAT_US_TO_K);
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_port_schemes_params_t *) arg,
                        sizeof(ioc_fm_pcd_port_schemes_params_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err =  FM_PORT_PcdKgUnbindSchemes(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdPortSchemesParams *)param);

            XX_Free(param);
            return err;
        }

        case FM_PORT_IOC_PCD_PRS_MODIFY_START_OFFSET:
        {
            ioc_fm_pcd_prs_start_t *param;

            param = (ioc_fm_pcd_prs_start_t *) XX_Malloc(sizeof(ioc_fm_pcd_prs_start_t));
            if (!param)
                RETURN_ERROR(MINOR, E_NO_MEMORY, ("IOCTL FM PORT"));

            memset(param, 0, sizeof(ioc_fm_pcd_prs_start_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (copy_from_user(param, (ioc_fm_pcd_prs_start_t *)compat_ptr(arg),
                                   sizeof(ioc_fm_pcd_prs_start_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);
                }
            }
            else
#endif
            {
                if (copy_from_user(param, (ioc_fm_pcd_prs_start_t *)arg,
                                   sizeof(ioc_fm_pcd_prs_start_t)))
                {
                    XX_Free(param);
                    RETURN_ERROR(MAJOR, E_WRITE_FAILED, NO_MSG);
                }
            }

            err = FM_PORT_PcdPrsModifyStartOffset(p_LnxWrpFmPortDev->h_Dev, (t_FmPcdPrsStart *)param);

            XX_Free(param);
            return err;
        }

        case FM_PORT_IOC_PCD_PLCR_ALLOC_PROFILES:
        {
            uint16_t num;
            if (get_user(num, (uint16_t*) arg))
                break;

            return FM_PORT_PcdPlcrAllocProfiles(p_LnxWrpFmPortDev->h_Dev, num);
        }

        case FM_PORT_IOC_PCD_PLCR_FREE_PROFILES:
            return FM_PORT_PcdPlcrFreeProfiles(p_LnxWrpFmPortDev->h_Dev);

        case FM_PORT_IOC_DETACH_PCD:
            return FM_PORT_DetachPCD(p_LnxWrpFmPortDev->h_Dev);

        case FM_PORT_IOC_ATTACH_PCD:
            return FM_PORT_AttachPCD(p_LnxWrpFmPortDev->h_Dev);

#if defined(CONFIG_COMPAT)
        case FM_PORT_IOC_PCD_CC_MODIFY_TREE_COMPAT:
#endif
        case FM_PORT_IOC_PCD_CC_MODIFY_TREE:
        {
            ioc_fm_obj_t id;

            memset(&id, 0 , sizeof(ioc_fm_obj_t));

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                ioc_compat_fm_obj_t compat_id;

                if (copy_from_user(&compat_id, (ioc_compat_fm_obj_t *) compat_ptr(arg), sizeof(ioc_compat_fm_obj_t)))
                    break;

                id.obj = compat_get_id2ptr(compat_id.obj);
            }
            else
#endif
            {
                if (copy_from_user(&id, (ioc_fm_obj_t *) arg, sizeof(ioc_fm_obj_t)))
                    break;
            }

            return FM_PORT_PcdCcModifyTree(p_LnxWrpFmPortDev->h_Dev, id.obj);
        }
#if defined(FM_IPSEC_SUPPORT) || defined(FM_IP_FRAG_N_REASSEM_SUPPORT)
        case FM_PORT_SET_OP_WORKAROUNDS:
        {
            fmOpPortWorkaroundsSelect_t workarounds; /* uint32_t type */

#if defined(CONFIG_COMPAT)
            if (compat)
            {
                if (get_user(workarounds, (fmOpPortWorkaroundsSelect_t *) compat_ptr(arg)))
                    break;
            }
            else
#endif
            {
                if (get_user(workarounds, (fmOpPortWorkaroundsSelect_t *)arg))
                    break;
            }

            return FM_PORT_SetOpWorkarounds(p_LnxWrpFmPortDev->h_Dev, workarounds);
        }
#endif /* defined(FM_IPSEC_SUPPORT) || defined(FM_IP_FRAG_N_REASSEM_SUPPORT) */
        default:
            RETURN_ERROR(MINOR, E_INVALID_SELECTION,
                ("invalid ioctl: cmd:0x%08x(type:0x%02x, nr:0x%02x.\n",
                cmd, _IOC_TYPE(cmd), _IOC_NR(cmd)));
    }

    RETURN_ERROR(MINOR, E_INVALID_OPERATION, ("IOCTL port"));
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
    struct device_node  *fm_node;
    static struct of_device_id fm_node_of_match[] __devinitdata = {
        { .compatible = "fsl,fman", },
        { /* end of list */ },
    };

    DBG(TRACE, ("Opening minor - %d - ", minor));

    if (file->private_data != NULL)
        return 0;

    /* Get all the FM nodes */
    for_each_matching_node(fm_node, fm_node_of_match) {
        struct platform_device    *of_dev;

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
struct file_operations fm_fops =
{
    .owner =            THIS_MODULE,
    .unlocked_ioctl =   fm_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl =     fm_compat_ioctl,
#endif
    .open =             fm_open,
    .release =          fm_close,
};
