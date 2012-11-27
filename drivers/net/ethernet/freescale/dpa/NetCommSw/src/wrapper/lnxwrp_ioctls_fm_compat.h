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
 @File          lnxwrp_ioctls_fm_compat.h

 @Description   FM PCD compat structures definition.

*/

#ifndef __FM_COMPAT_IOCTLS_H
#define __FM_COMPAT_IOCTLS_H

#include <linux/compat.h>

#define COMPAT_K_TO_US 0 /* copy from Kernel to User */
#define COMPAT_US_TO_K 1 /* copy from User to Kernel */
#define COMPAT_GENERIC 2

#define COMPAT_COPY_K2US(dest, src, type)	compat_copy_##type(src, dest, 0)
#define COMPAT_COPY_US2K(dest, src, type)	compat_copy_##type(dest, src, 1)

/* mapping kernel pointers w/ UserSpace id's { */
/* Because compat_ptr(ptr_to_compat(X)) != X, this way we cannot exchange pointers
   back and forth (US - KS). compat_ptr is a cast and pointers are broken. */
#define COMPAT_PTR2ID_ARRAY_MAX (256+1) /* first location is not used */
#define COMPAT_PTR2ID_WATERMARK 0xface0000
#define COMPAT_PTR2ID_WM_MASK   0xffff0000

/* define it for debug trace */
/*#define FM_COMPAT_DBG*/

#define _fm_cpt_prk(stage, format, arg...)	\
	printk(stage "fm_cpt (cpu:%u): " format, smp_processor_id(), ##arg)

#define _fm_cpt_inf(format, arg...) _fm_cpt_prk(KERN_INFO, format, ##arg)
#define _fm_cpt_wrn(format, arg...) _fm_cpt_prk(KERN_WARNING, format, ##arg)
#define _fm_cpt_err(format, arg...) _fm_cpt_prk(KERN_ERR, format, ##arg)

/* used for compat IOCTL debugging */
#if defined(FM_COMPAT_DBG)
	#define _fm_cpt_dbg(from, format, arg...) \
		do{ \
			if (from == COMPAT_US_TO_K) \
				printk("fm_cpt to KS [%s:%u](cpu:%u) - " format,	\
					__func__, __LINE__, smp_processor_id(), ##arg); \
			else if (from == COMPAT_K_TO_US) \
				printk("fm_cpt to US [%s:%u](cpu:%u) - " format,	\
					__func__, __LINE__, smp_processor_id(), ##arg); \
            else \
                printk("fm_cpt [%s:%u](cpu:%u) - " format,    \
                    __func__, __LINE__, smp_processor_id(), ##arg); \
		}while(0)
#else
#	define _fm_cpt_dbg(arg...)
#endif

/*TODO: per FMan module:
 *
 *      Parser:  FM_MAP_TYPE_PARSER_NODE,
 *      Kg:      FM_MAP_TYPE_KG_NODE,
 *      Policer: FM_MAP_TYPE_POLICER_NODE
 *      Manip:   FM_MAP_TYPE_MANIP_NODE
 **/
enum fm_map_node_type {
    FM_MAP_TYPE_UNSPEC = 0,
    FM_MAP_TYPE_PCD_NODE,

    /* add types here, update the policy */

    __FM_MAP_TYPE_AFTER_LAST,
    FM_MAP_TYPE_MAX = __FM_MAP_TYPE_AFTER_LAST - 1
};

void compat_del_ptr2id(void *p, enum fm_map_node_type);
compat_uptr_t compat_add_ptr2id(void *p, enum fm_map_node_type);
compat_uptr_t compat_get_ptr2id(void *p, enum fm_map_node_type);
void *compat_get_id2ptr(compat_uptr_t comp, enum fm_map_node_type);

static inline compat_uptr_t compat_pcd_ptr2id(void *ptr) {
    return (ptr)? compat_get_ptr2id(ptr, FM_MAP_TYPE_PCD_NODE)
                : (compat_uptr_t) 0;
}

static inline void *compat_pcd_id2ptr(compat_uptr_t id) {
    return (id) ? compat_get_id2ptr(id, FM_MAP_TYPE_PCD_NODE)
                : NULL;
}

/* other similar inlines may be added as new nodes are added
   to enum fm_map_node_type above... */
/* } mapping kernel pointers w/ UserSpace id's  */

/* pcd compat structures { */
typedef struct ioc_compat_fm_pcd_cc_node_remove_key_params_t {
    compat_uptr_t                       id;
    uint16_t                            key_indx;
} ioc_compat_fm_pcd_cc_node_remove_key_params_t;

typedef union ioc_compat_fm_pcd_plcr_next_engine_params_u {
        ioc_fm_pcd_done_action     action;
        compat_uptr_t              p_profile;
        compat_uptr_t              p_direct_scheme;
} ioc_compat_fm_pcd_plcr_next_engine_params_u;

typedef struct ioc_compat_fm_pcd_plcr_profile_params_t {
    bool                                        modify;
    union {
        struct {
            ioc_fm_pcd_profile_type_selection   profile_type;
            compat_uptr_t                       p_fm_port;
            uint16_t                            relative_profile_id;
        } new_params;
        compat_uptr_t                           p_profile;
    } profile_select;
    ioc_fm_pcd_plcr_algorithm_selection         alg_selection;
    ioc_fm_pcd_plcr_color_mode                  color_mode;

    union {
        ioc_fm_pcd_plcr_color                   dflt_color;
        ioc_fm_pcd_plcr_color                   override;
    } color;

    ioc_fm_pcd_plcr_non_passthrough_alg_param_t non_passthrough_alg_param;

    ioc_fm_pcd_engine                           next_engine_on_green;
    ioc_compat_fm_pcd_plcr_next_engine_params_u params_on_green;

    ioc_fm_pcd_engine                           next_engine_on_yellow;
    ioc_compat_fm_pcd_plcr_next_engine_params_u params_on_yellow;

    ioc_fm_pcd_engine                           next_engine_on_red;
    ioc_compat_fm_pcd_plcr_next_engine_params_u params_on_red;

    bool                                        trap_profile_on_flow_A;
    bool                                        trap_profile_on_flow_B;
    bool                                        trap_profile_on_flow_C;
    compat_uptr_t                               id;
} ioc_compat_fm_pcd_plcr_profile_params_t;

typedef struct ioc_compat_fm_obj_t {
    compat_uptr_t obj;
} ioc_compat_fm_obj_t;

typedef struct ioc_compat_fm_pcd_kg_scheme_select_t {
    bool          direct;
    compat_uptr_t scheme_id;
} ioc_compat_fm_pcd_kg_scheme_select_t;

typedef struct ioc_compat_fm_pcd_port_schemes_params_t {
    uint8_t        num_of_schemes;
    compat_uptr_t  scheme_ids[FM_PCD_KG_NUM_OF_SCHEMES];
} ioc_compat_fm_pcd_port_schemes_params_t;

typedef struct ioc_compat_fm_pcd_net_env_params_t {
    uint8_t                         num_of_distinction_units;
    ioc_fm_pcd_distinction_unit_t   units[IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS]; /* same structure*/
    compat_uptr_t                   id;
} ioc_compat_fm_pcd_net_env_params_t;

typedef struct ioc_compat_fm_pcd_prs_sw_params_t {
    bool                            override;
    uint32_t                        size;
    uint16_t                        base;
    compat_uptr_t                   p_code;
    uint32_t                        sw_prs_data_params[IOC_FM_PCD_PRS_NUM_OF_HDRS];
    uint8_t                         num_of_labels;
    ioc_fm_pcd_prs_label_params_t   labels_table[IOC_FM_PCD_PRS_NUM_OF_LABELS];
} ioc_compat_fm_pcd_prs_sw_params_t;

typedef struct ioc_compat_fm_pcd_cc_next_kg_params_t {
    bool          override_fqid;
    uint32_t      new_fqid;
#if DPAA_VERSION >= 11
    uint8_t       new_relative_storage_profile_id;
#endif
    compat_uptr_t p_direct_scheme;
} ioc_compat_fm_pcd_cc_next_kg_params_t;

typedef struct ioc_compat_fm_pcd_cc_next_cc_params_t {
    compat_uptr_t       cc_node_id;
} ioc_compat_fm_pcd_cc_next_cc_params_t;

#if DPAA_VERSION >= 11
typedef struct ioc_compat_fm_pcd_cc_next_fr_params_t {
    compat_uptr_t       frm_replic_id;
} ioc_compat_fm_pcd_cc_next_fr_params_t;
#endif /* DPAA_VERSION >= 11 */

typedef struct ioc_compat_fm_pcd_cc_next_engine_params_t {
    ioc_fm_pcd_engine                          next_engine;
    union {
        ioc_compat_fm_pcd_cc_next_cc_params_t  cc_params;      /**< compat structure*/
        ioc_fm_pcd_cc_next_plcr_params_t       plcr_params;    /**< same structure*/
        ioc_fm_pcd_cc_next_enqueue_params_t    enqueue_params; /**< same structure*/
        ioc_compat_fm_pcd_cc_next_kg_params_t  kg_params;      /**< compat structure*/
#if DPAA_VERSION >= 11
        ioc_compat_fm_pcd_cc_next_fr_params_t  fr_params;      /**< compat structure*/
#endif /* DPAA_VERSION >= 11 */
    } params;
    compat_uptr_t                               manip_id;
    bool                                        statistics_en;
} ioc_compat_fm_pcd_cc_next_engine_params_t;

typedef struct ioc_compat_fm_pcd_cc_grp_params_t {
    uint8_t                             num_of_distinction_units;
    uint8_t                             unit_ids [IOC_FM_PCD_MAX_NUM_OF_CC_UNITS];
    ioc_compat_fm_pcd_cc_next_engine_params_t  next_engine_per_entries_in_grp[IOC_FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP];
} ioc_compat_fm_pcd_cc_grp_params_t;

typedef struct ioc_compat_fm_pcd_cc_tree_params_t {
    compat_uptr_t                   net_env_id;
    uint8_t                         num_of_groups;
    ioc_compat_fm_pcd_cc_grp_params_t      fm_pcd_cc_group_params [IOC_FM_PCD_MAX_NUM_OF_CC_GROUPS];
    compat_uptr_t                   id;
} ioc_compat_fm_pcd_cc_tree_params_t;

typedef struct ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t {
    compat_uptr_t                       id;
    uint8_t                             grp_indx;
    uint8_t                             indx;
    ioc_compat_fm_pcd_cc_next_engine_params_t  cc_next_engine_params;
} ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t;

typedef struct ioc_compat_fm_pcd_cc_key_params_t {
    compat_uptr_t                              p_key;
    compat_uptr_t                              p_mask;
    ioc_compat_fm_pcd_cc_next_engine_params_t  cc_next_engine_params; /**< compat structure*/
} ioc_compat_fm_pcd_cc_key_params_t;

typedef struct ioc_compat_keys_params_t {
    uint16_t                                   max_num_of_keys;
    bool                                       mask_support;
    ioc_fm_pcd_cc_stats_mode                   statistics_mode;
#if (DPAA_VERSION >= 11)
    uint16_t                                   frame_length_ranges[IOC_FM_PCD_CC_STATS_MAX_NUM_OF_FLR];
#endif /* (DPAA_VERSION >= 11) */
    uint16_t                                   num_of_keys;
    uint8_t                                    key_size;
    ioc_compat_fm_pcd_cc_key_params_t          key_params[IOC_FM_PCD_MAX_NUM_OF_KEYS]; /**< compat structure*/
    ioc_compat_fm_pcd_cc_next_engine_params_t  cc_next_engine_params_for_miss;         /**< compat structure*/
} ioc_compat_keys_params_t;

typedef struct ioc_compat_fm_pcd_cc_node_params_t {
    ioc_fm_pcd_extract_entry_t                 extract_cc_params;  /**< same structure*/
    ioc_compat_keys_params_t                   keys_params;        /**< compat structure*/
    compat_uptr_t                              id;
} ioc_compat_fm_pcd_cc_node_params_t;

/**************************************************************************//**
 @Description   Parameters for defining a hash table
*//***************************************************************************/
typedef struct ioc_compat_fm_pcd_hash_table_params_t {
    uint16_t                    max_num_of_keys;
    ioc_fm_pcd_cc_stats_mode    statistics_mode;
    uint16_t                    hash_res_mask;
    uint8_t                     hash_shift;
    uint8_t                     match_key_size;
    ioc_compat_fm_pcd_cc_next_engine_params_t   cc_next_engine_params_for_miss;
    compat_uptr_t               id;
} ioc_compat_fm_pcd_hash_table_params_t;

typedef struct ioc_compat_fm_pcd_hash_table_add_key_params_t {
    compat_uptr_t                       p_hash_tbl;
    uint8_t                             key_size;
    ioc_compat_fm_pcd_cc_key_params_t   key_params;
} ioc_compat_fm_pcd_hash_table_add_key_params_t;

typedef struct ioc_compat_fm_pcd_cc_node_modify_key_params_t {
    compat_uptr_t                       id;
    uint16_t                            key_indx;
    uint8_t                             key_size;
    compat_uptr_t                       p_key;
    compat_uptr_t                       p_mask;
} ioc_compat_fm_pcd_cc_node_modify_key_params_t;

typedef struct ioc_compat_fm_pcd_hash_table_remove_key_params_t {
    compat_uptr_t   p_hash_tbl;
    uint8_t         key_size;
    compat_uptr_t   p_key;
} ioc_compat_fm_pcd_hash_table_remove_key_params_t;

typedef struct ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t {
    compat_uptr_t                       id;
    uint16_t                            key_indx;
    uint8_t                             key_size;
    ioc_compat_fm_pcd_cc_key_params_t   key_params;
} ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t;

typedef struct ioc_compat_fm_port_pcd_plcr_params_t {
    compat_uptr_t                plcr_profile_id;
} ioc_compat_fm_port_pcd_plcr_params_t;

typedef struct ioc_compat_fm_port_pcd_cc_params_t {
    compat_uptr_t                cc_tree_id;
} ioc_compat_fm_port_pcd_cc_params_t;

typedef struct ioc_compat_fm_port_pcd_kg_params_t {
    uint8_t             num_of_schemes;
    compat_uptr_t       scheme_ids[FM_PCD_KG_NUM_OF_SCHEMES];
    bool                direct_scheme;
    compat_uptr_t       direct_scheme_id;
} ioc_compat_fm_port_pcd_kg_params_t;

typedef struct ioc_compat_fm_port_pcd_params_t {
    ioc_fm_port_pcd_support          pcd_support;
    compat_uptr_t                    net_env_id;
    compat_uptr_t                    p_prs_params;
    compat_uptr_t                    p_cc_params;
    compat_uptr_t                    p_kg_params;
    compat_uptr_t                    p_plcr_params;
    compat_uptr_t                    p_ip_reassembly_manip;
} ioc_compat_fm_port_pcd_params_t;

typedef struct ioc_compat_fm_pcd_kg_cc_t {
    compat_uptr_t                   tree_id;
    uint8_t                         grp_id;
    bool                            plcr_next;
    bool                            bypass_plcr_profile_generation;
    ioc_fm_pcd_kg_plcr_profile_t    plcr_profile;
} ioc_compat_fm_pcd_kg_cc_t;

typedef struct ioc_compat_fm_pcd_kg_scheme_params_t {
    bool                                modify;
    union {
        uint8_t                         relative_scheme_id;
        compat_uptr_t                   scheme_id;
    } scm_id;
    bool                                always_direct;
    struct {
        compat_uptr_t                   net_env_id;
        uint8_t                         num_of_distinction_units;
        uint8_t                         unit_ids[IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS];
    } net_env_params;
    bool                                use_hash;
    ioc_fm_pcd_kg_key_extract_and_hash_params_t key_extract_and_hash_params;
    bool                                bypass_fqid_generation;
    uint32_t                            base_fqid;
    uint8_t                             num_of_used_extracted_ors;
    ioc_fm_pcd_kg_extracted_or_params_t extracted_ors[IOC_FM_PCD_KG_NUM_OF_GENERIC_REGS];
#if DPAA_VERSION >= 11
    bool                                override_storage_profile;
    ioc_fm_pcd_kg_storage_profile_t     storage_profile;
#endif /* DPAA_VERSION >= 11 */
    ioc_fm_pcd_engine                   next_engine;
    union{
        ioc_fm_pcd_done_action          done_action;
        ioc_fm_pcd_kg_plcr_profile_t    plcr_profile;
        ioc_compat_fm_pcd_kg_cc_t       cc;
    } kg_next_engine_params;
    ioc_fm_pcd_kg_scheme_counter_t      scheme_counter;
    compat_uptr_t                       id;
} ioc_compat_fm_pcd_kg_scheme_params_t;

typedef struct ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t {
    compat_uptr_t                       id;
    uint16_t                            key_indx;
    uint8_t                             key_size;
    ioc_compat_fm_pcd_cc_next_engine_params_t  cc_next_engine_params;
} ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t;

typedef struct ioc_compat_fm_pcd_manip_hdr_insrt_generic_params_t {
    uint8_t                         offset;
    uint8_t                         size;
    bool                            replace;
    compat_uptr_t                   p_data;
} ioc_compat_fm_pcd_manip_hdr_insrt_generic_params_t;

typedef struct ioc_compat_fm_pcd_manip_hdr_insrt_specific_l2_params_t {
    ioc_fm_pcd_manip_hdr_insrt_specific_l2  specific_l2;
    bool                                    update;
    uint8_t                                 size;
    compat_uptr_t                           p_data;
} ioc_compat_fm_pcd_manip_hdr_insrt_specific_l2_params_t;

typedef struct ioc_compat_fm_pcd_manip_hdr_insrt_by_hdr_params_t {
    ioc_fm_pcd_manip_hdr_insrt_by_hdr_type                      type;
    union {
       ioc_compat_fm_pcd_manip_hdr_insrt_specific_l2_params_t   specific_l2_params;
    } u;
} ioc_compat_fm_pcd_manip_hdr_insrt_by_hdr_params_t;

typedef struct ioc_compat_fm_pcd_manip_hdr_insrt_params_t {
    ioc_fm_pcd_manip_hdr_insrt_type                         type;
    union {
        ioc_compat_fm_pcd_manip_hdr_insrt_by_hdr_params_t   by_hdr;
        ioc_compat_fm_pcd_manip_hdr_insrt_generic_params_t  generic;
#ifdef FM_CAPWAP_SUPPORT
#error CAPWAP not supported!
        ioc_fm_pcd_manip_hdr_insrt_by_template_params_t     by_template;
#endif /* FM_CAPWAP_SUPPORT */
    } u;
} ioc_compat_fm_pcd_manip_hdr_insrt_params_t;

typedef struct ioc_compat_fm_pcd_manip_hdr_params_t {
    bool                                        rmv;
    ioc_fm_pcd_manip_hdr_rmv_params_t           rmv_params;
    bool                                        insrt;
    ioc_compat_fm_pcd_manip_hdr_insrt_params_t  insrt_params;
    bool                                        field_update;
    ioc_fm_pcd_manip_hdr_field_update_params_t  field_update_params;
    bool                                        custom;
    ioc_fm_pcd_manip_hdr_custom_params_t        custom_params;
    bool                                        dont_parse_after_manip;
} ioc_compat_fm_pcd_manip_hdr_params_t;

typedef struct ioc_compat_fm_pcd_manip_params_t {
    ioc_fm_pcd_manip_type                         type;
    union {
        ioc_compat_fm_pcd_manip_hdr_params_t      hdr;
        ioc_fm_pcd_manip_reassem_params_t         reassem;
        ioc_fm_pcd_manip_frag_params_t            frag;
        ioc_fm_pcd_manip_special_offload_params_t special_offload;
    } u;
    compat_uptr_t                                 p_next_manip;
#ifdef FM_CAPWAP_SUPPORT
#error "FM_CAPWAP_SUPPORT feature not supported!"
    bool                                          frag_or_reasm;
    ioc_fm_pcd_manip_frag_or_reasm_params_t       frag_or_reasm_params;
#endif /* FM_CAPWAP_SUPPORT */
    compat_uptr_t                                 id;
} ioc_compat_fm_pcd_manip_params_t;

/* } pcd compat structures */

/* pcd compat functions { */
void compat_copy_fm_pcd_plcr_profile(
        ioc_compat_fm_pcd_plcr_profile_params_t *compat_param,
        ioc_fm_pcd_plcr_profile_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_key(
        ioc_compat_fm_pcd_cc_key_params_t *compat_param,
        ioc_fm_pcd_cc_key_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_node_modify_key_and_next_engine(
        ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t *compat_param,
        ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_node_modify_next_engine(
        ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t *compat_param,
        ioc_fm_pcd_cc_node_modify_next_engine_params_t *param,
        uint8_t compat);

void compat_fm_pcd_cc_tree_modify_next_engine(
        ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t *compat_param,
        ioc_fm_pcd_cc_tree_modify_next_engine_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_hash_table(
        ioc_compat_fm_pcd_hash_table_params_t *compat_param,
        ioc_fm_pcd_hash_table_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_grp(
        ioc_compat_fm_pcd_cc_grp_params_t *compat_param,
        ioc_fm_pcd_cc_grp_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_tree(
        ioc_compat_fm_pcd_cc_tree_params_t *compat_param,
        ioc_fm_pcd_cc_tree_params_t *param,
        uint8_t compat);

void compat_fm_pcd_prs_sw(
        ioc_compat_fm_pcd_prs_sw_params_t *compat_param,
        ioc_fm_pcd_prs_sw_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_kg_scheme(
        ioc_compat_fm_pcd_kg_scheme_params_t *compat_param,
        ioc_fm_pcd_kg_scheme_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_kg_scheme_select(
        ioc_compat_fm_pcd_kg_scheme_select_t *compat_param,
        ioc_fm_pcd_kg_scheme_select_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_kg_schemes_params(
        ioc_compat_fm_pcd_port_schemes_params_t *compat_param,
        ioc_fm_pcd_port_schemes_params_t *param,
        uint8_t compat);

void compat_copy_fm_port_pcd_kg(
        ioc_compat_fm_port_pcd_kg_params_t *compat_param,
        ioc_fm_port_pcd_kg_params_t *param,
        uint8_t compat);

void compat_copy_fm_port_pcd(
        ioc_compat_fm_port_pcd_params_t *compat_param,
        ioc_fm_port_pcd_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_net_env(
        ioc_compat_fm_pcd_net_env_params_t *compat_param,
        ioc_fm_pcd_net_env_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_node_modify_key(
        ioc_compat_fm_pcd_cc_node_modify_key_params_t *compat_param,
        ioc_fm_pcd_cc_node_modify_key_params_t *param,
        uint8_t compat);

void compat_copy_keys(
        ioc_compat_keys_params_t *compat_param,
        ioc_keys_params_t *param,
        uint8_t compat);

void compat_copy_fm_pcd_cc_node(
        ioc_compat_fm_pcd_cc_node_params_t *compat_param,
        ioc_fm_pcd_cc_node_params_t *param,
        uint8_t compat);

void compat_fm_pcd_manip_set_node(
        ioc_compat_fm_pcd_manip_params_t *compat_param,
        ioc_fm_pcd_manip_params_t *param,
        uint8_t compat);

void compat_copy_fm_net_env_delete(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat);

void compat_copy_fm_pcd_cc_delete_node(
    ioc_compat_fm_obj_t *compat_id,
    ioc_fm_obj_t *id,
    uint8_t compat);

void compat_copy_fm_pcd_cc_delete_tree(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat);

void compat_copy_fm_port_pcd_modify_tree(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat);

void compat_copy_fm_pcd_manip_delete_node(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat);

void compat_copy_fm_pcd_scheme_delete(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat);

void compat_copy_fm_pcd_plcr_del_profile(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat);

/* } pcd compat functions */
#endif
