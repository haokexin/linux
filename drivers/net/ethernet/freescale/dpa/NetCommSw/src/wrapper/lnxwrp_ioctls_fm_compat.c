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
 @File          lnxwrp_fm_compat_ioctls.c

 @Description   FM PCD compat functions

*/

#if !defined(CONFIG_COMPAT)
#error "missing COMPAT layer..."
#endif


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <sysdev/fsl_soc.h>

#include "part_ext.h"
#include "fm_ioctls.h"
#include "fm_pcd_ioctls.h"
#include "fm_port_ioctls.h"
#include "lnxwrp_ioctls_fm_compat.h"

#if defined(FM_COMPAT_DBG)
static void hex_dump(void * p_addr, unsigned int size)
{
   int i;

   for(i=0; i<size; i+=16)
   {
       printk("%p: 0x%08x 0x%08x 0x%08x 0x%08x\n", p_addr + i,
           *(unsigned int *)(p_addr + i),
           *(unsigned int *)(p_addr + i + 4),
           *(unsigned int *)(p_addr + i + 8),
           *(unsigned int *)(p_addr + i +12)
           );
   }
}
#endif

/* maping kernel pointers w/ UserSpace id's { */
/*TODO: per FMan module:
 *
 *      Parser: FM_PARSER_NODE,
 *      Kg: FM_KG_NODE,
 *      Policer: FM_POLICER_NODE
 *      Manip: FM_MANIP_NODE
 **/
const unsigned char map_node_name[][ID_MAP_NAME_SIZE] = {
    "FM_NODE", /* 0 */
    "FM_PORT_NODE", /* 1 */
    "FM_PCD_NODE" /* 2 */
};

#define FM_NODE map_node_name[0]
#define PORT_NODE map_node_name[1]
#define PCD_NODE map_node_name[2]

struct map_node {
	void *ptr;
	const unsigned char *name;
};

static struct map_node compat_ptr2id_array[COMPAT_PTR2ID_ARRAY_MAX] = {{0},{0}};

void compat_del_ptr2id(void *p, const unsigned char *name)
{
    compat_uptr_t k;

    _fm_cpt_dbg(COMPAT_GENERIC, "delete (%p)\n", p);

    for(k=1; k < COMPAT_PTR2ID_ARRAY_MAX; k++)
        if(compat_ptr2id_array[k].ptr == p){
            compat_ptr2id_array[k].ptr = NULL;
			compat_ptr2id_array[k].name = NULL;
		}
}
EXPORT_SYMBOL(compat_del_ptr2id);

compat_uptr_t compat_add_ptr2id(void *p, const unsigned char *name)
{
    compat_uptr_t k;

    _fm_cpt_dbg(COMPAT_GENERIC, " (%p) do ->\n", p);

    if(!p)
        return 0;

    for(k=1; k < COMPAT_PTR2ID_ARRAY_MAX; k++)
        if(compat_ptr2id_array[k].ptr == NULL)
        {
            compat_ptr2id_array[k].ptr = p;
			compat_ptr2id_array[k].name = name;
            _fm_cpt_dbg(COMPAT_GENERIC, "0x%08x \n", k | COMPAT_PTR2ID_WATERMARK);
            return k | COMPAT_PTR2ID_WATERMARK;
        }

	printk(KERN_WARNING "FMan map list full! No more PCD space on kernel!\n");
    return 0;
}
EXPORT_SYMBOL(compat_add_ptr2id);

compat_uptr_t compat_get_ptr2id(void *p, const unsigned char *name)
{
    compat_uptr_t k;

    _fm_cpt_dbg(COMPAT_GENERIC, " (%p) get -> \n", p);

    for(k=1; k < COMPAT_PTR2ID_ARRAY_MAX; k++)
        if(compat_ptr2id_array[k].ptr == p && compat_ptr2id_array[k].name == name) {
            _fm_cpt_dbg(COMPAT_GENERIC, "0x%08x\n", k | COMPAT_PTR2ID_WATERMARK);
            return k | COMPAT_PTR2ID_WATERMARK;
        }

    return 0;
}
EXPORT_SYMBOL(compat_get_ptr2id);

void *compat_get_id2ptr(compat_uptr_t comp, const unsigned char *name)
{

    _fm_cpt_dbg(COMPAT_GENERIC, " (0x%08x) get -> \n", comp);

    if((COMPAT_PTR2ID_WM_MASK & comp) != COMPAT_PTR2ID_WATERMARK) {
        _fm_cpt_dbg(COMPAT_GENERIC, "Error, invalid watermark!\n\n");
        dump_stack();
        return compat_ptr(comp);
    }

    comp &= ~COMPAT_PTR2ID_WM_MASK;

    if(((0 < comp) && (comp < COMPAT_PTR2ID_ARRAY_MAX) && (compat_ptr2id_array[comp].ptr != NULL)
		&& compat_ptr2id_array[comp].name == name)) {
        _fm_cpt_dbg(COMPAT_GENERIC, "%p\n", compat_ptr2id_array[comp].ptr);
        return compat_ptr2id_array[comp].ptr;
    }
    return NULL;
}
EXPORT_SYMBOL(compat_get_id2ptr);
/* } maping kernel pointers w/ UserSpace id's  */

static inline void compat_copy_fm_pcd_plcr_next_engine(
        ioc_compat_fm_pcd_plcr_next_engine_params_u *compat_param,
        ioc_fm_pcd_plcr_next_engine_params_u        *param,
        ioc_fm_pcd_engine                           next_engine,
        uint8_t                                     compat)
{
    _fm_cpt_dbg (compat, " Calling...\n");

    switch (next_engine)
    {
        case e_IOC_FM_PCD_PLCR:
            if (compat == COMPAT_US_TO_K)
                param->p_profile = compat_get_id2ptr(compat_param->p_profile, PCD_NODE);
            else
                compat_param->p_profile = compat_get_ptr2id(param->p_profile, PCD_NODE);
        break;
        case e_IOC_FM_PCD_KG:
            if (compat == COMPAT_US_TO_K)
                param->p_direct_scheme = compat_get_id2ptr(compat_param->p_direct_scheme, PCD_NODE);
            else
                compat_param->p_direct_scheme = compat_get_ptr2id(param->p_direct_scheme, PCD_NODE);
        break;
        default:
            if (compat == COMPAT_US_TO_K)
                param->action = compat_param->action;
            else
                compat_param->action = param->action;
        break;
    }

    _fm_cpt_dbg (compat, " Called.\n");
}

void compat_copy_fm_pcd_plcr_profile(
        ioc_compat_fm_pcd_plcr_profile_params_t *compat_param,
        ioc_fm_pcd_plcr_profile_params_t        *param,
        uint8_t                                 compat)
{
    _fm_cpt_dbg (compat, " Calling...\n");

    if (compat == COMPAT_US_TO_K)
    {
        param->modify = compat_param->modify;

        /* profile_select */
        if (!compat_param->modify)
        {
            param->profile_select.new_params.profile_type =
                compat_param->profile_select.new_params.profile_type;
            param->profile_select.new_params.p_port =
                compat_ptr(compat_param->profile_select.new_params.p_port);
            param->profile_select.new_params.relative_profile_id =
                compat_param->profile_select.new_params.relative_profile_id;
        }
        else
            param->profile_select.p_profile =
                compat_ptr(compat_param->profile_select.p_profile);

        param->alg_selection    = compat_param->alg_selection;
        param->color_mode       = compat_param->color_mode;

        /* both parameters in the union has the same size, so memcpy works */
        memcpy(&param->color, &compat_param->color, sizeof(param->color));

        memcpy(&param->non_passthrough_alg_param,
               &compat_param->non_passthrough_alg_param,
               sizeof(ioc_fm_pcd_plcr_non_passthrough_alg_param_t));

        param->next_engine_on_green = compat_param->next_engine_on_green;
        param->next_engine_on_yellow = compat_param->next_engine_on_yellow;
        param->next_engine_on_red = compat_param->next_engine_on_red;

        param->trap_profile_on_flow_A = compat_param->trap_profile_on_flow_A;
        param->trap_profile_on_flow_B = compat_param->trap_profile_on_flow_B;
        param->trap_profile_on_flow_C = compat_param->trap_profile_on_flow_C;
    }
    else
    {
        compat_param->modify = param->modify;

        /* profile_select */
        if(!param->modify){
            compat_param->profile_select.new_params.profile_type =
                param->profile_select.new_params.profile_type;
            compat_param->profile_select.new_params.p_port =
                ptr_to_compat(param->profile_select.new_params.p_port);
            compat_param->profile_select.new_params.relative_profile_id =
                param->profile_select.new_params.relative_profile_id;
        }
        else
            compat_param->profile_select.p_profile =
                ptr_to_compat(param->profile_select.p_profile);

        compat_param->alg_selection = param->alg_selection;
        compat_param->color_mode    = param->color_mode;

        /* both parameters in the union has the same size, so memcpy works */
        memcpy(&compat_param->color, &param->color, sizeof(compat_param->color));

        memcpy(&compat_param->non_passthrough_alg_param,
               &param->non_passthrough_alg_param,
               sizeof(ioc_fm_pcd_plcr_non_passthrough_alg_param_t));

        compat_param->next_engine_on_green = param->next_engine_on_green;
        compat_param->next_engine_on_yellow = param->next_engine_on_yellow;
        compat_param->next_engine_on_red = param->next_engine_on_red;

        compat_param->trap_profile_on_flow_A = param->trap_profile_on_flow_A;
        compat_param->trap_profile_on_flow_B = param->trap_profile_on_flow_B;
        compat_param->trap_profile_on_flow_C = param->trap_profile_on_flow_C;

        compat_param->id = compat_add_ptr2id(param->id, PCD_NODE);
    }

    compat_copy_fm_pcd_plcr_next_engine(&compat_param->params_on_green,
            &param->params_on_green, param->next_engine_on_green, compat);

    compat_copy_fm_pcd_plcr_next_engine(&compat_param->params_on_yellow,
            &param->params_on_yellow, param->next_engine_on_yellow, compat);

    compat_copy_fm_pcd_plcr_next_engine(&compat_param->params_on_red,
            &param->params_on_red, param->next_engine_on_red, compat);

    _fm_cpt_dbg (compat, " Called.\n");
}

void compat_copy_fm_pcd_plcr_del_profile(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
        compat_del_ptr2id(id->obj, PCD_NODE);
    }
}

static inline void compat_copy_fm_pcd_cc_next_kg(
        ioc_compat_fm_pcd_cc_next_kg_params_t   *compat_param,
        ioc_fm_pcd_cc_next_kg_params_t          *param,
        uint8_t                                 compat)
{
    _fm_cpt_dbg (compat, " Calling...\n");

    if (compat == COMPAT_US_TO_K)
    {
        param->new_fqid         = compat_param->new_fqid;
        param->override_fqid    = compat_param->override_fqid;
        param->p_direct_scheme  = compat_get_id2ptr(compat_param->p_direct_scheme, PCD_NODE);
    }
    else
    {
        compat_param->new_fqid          = param->new_fqid;
        compat_param->override_fqid     = param->override_fqid;
        compat_param->p_direct_scheme   = compat_get_ptr2id(param->p_direct_scheme, PCD_NODE);
    }

    _fm_cpt_dbg (compat, " Called.\n");
}

static inline void compat_copy_fm_pcd_cc_next_cc(
        ioc_compat_fm_pcd_cc_next_cc_params_t   *compat_param,
        ioc_fm_pcd_cc_next_cc_params_t          *param,
        uint8_t                                 compat)
{
    _fm_cpt_dbg (compat, " Calling...\n");

    if (compat == COMPAT_US_TO_K)
        param->cc_node_id = compat_get_id2ptr(compat_param->cc_node_id, PCD_NODE);
    else
        compat_param->cc_node_id = compat_get_ptr2id(param->cc_node_id, PCD_NODE);

    _fm_cpt_dbg (compat, " Called.\n");
}

static inline void compat_copy_fm_pcd_cc_next_engine(
        ioc_compat_fm_pcd_cc_next_engine_params_t   *compat_param,
        ioc_fm_pcd_cc_next_engine_params_t          *param,
        uint8_t                                     compat)
{
    /*_fm_cpt_dbg (compat, " Calling...\n");*/

    if (compat == COMPAT_US_TO_K)
    {
        param->next_engine = compat_param->next_engine;
        if (param->next_engine != e_IOC_FM_PCD_INVALID )
            _fm_cpt_dbg(compat, " param->next_engine = %i \n", param->next_engine);

        switch (param->next_engine)
        {
            case e_IOC_FM_PCD_KG:
            case e_IOC_FM_PCD_CC:
                break;
            default:
                memcpy(&param->params, &compat_param->params, sizeof(param->params));
            break;
        }
        if (compat_param->manip_id)
            param->manip_id = compat_get_id2ptr(compat_param->manip_id, PCD_NODE);
        param->statistics_en = compat_param->statistics_en;
    }
    else
    {
        compat_param->next_engine = param->next_engine;

        switch (compat_param->next_engine)
        {
            case e_IOC_FM_PCD_KG:
            case e_IOC_FM_PCD_CC:
                break;
            default:
                memcpy(&compat_param->params, &param->params, sizeof(compat_param->params));
            break;
        }
        if (param->manip_id)
            compat_param->manip_id = compat_get_ptr2id(param->manip_id, PCD_NODE);
        compat_param->statistics_en = param->statistics_en;
    }

    switch (compat_param->next_engine)
    {
        case e_IOC_FM_PCD_KG:
            compat_copy_fm_pcd_cc_next_kg(&compat_param->params.kg_params, &param->params.kg_params, compat);
            break;
        case e_IOC_FM_PCD_CC:
            compat_copy_fm_pcd_cc_next_cc(&compat_param->params.cc_params, &param->params.cc_params, compat);
            break;
        default:
        break;
    }

    /*_fm_cpt_dbg (compat, " Called.\n");*/
}

void compat_copy_fm_pcd_cc_key(
        ioc_compat_fm_pcd_cc_key_params_t   *compat_param,
        ioc_fm_pcd_cc_key_params_t          *param,
        uint8_t                             compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->p_key = compat_ptr(compat_param->p_key);
        param->p_mask = compat_ptr(compat_param->p_mask);
    }
    else
    {
        compat_param->p_key = ptr_to_compat(param->p_key);
        compat_param->p_mask = ptr_to_compat(param->p_mask);
    }

    compat_copy_fm_pcd_cc_next_engine(
            &compat_param->cc_next_engine_params,
            &param->cc_next_engine_params,
            compat);
}

void compat_copy_fm_pcd_cc_node_modify_key_and_next_engine(
        ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t   *compat_param,
        ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t          *param,
        uint8_t                                                         compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->id       = compat_ptr(compat_param->id);
        param->key_indx = compat_param->key_indx;
        param->key_size = compat_param->key_size;
        compat_copy_fm_pcd_cc_key(
            &compat_param->key_params,
            &param->key_params,
            compat);
    }
    else
    {
        compat_param->id       = ptr_to_compat(param->id);
        compat_param->key_indx = param->key_indx;
        compat_param->key_size = param->key_size;
        compat_copy_fm_pcd_cc_key(
            &compat_param->key_params,
            &param->key_params,
            compat);
    }
}

void compat_copy_fm_pcd_cc_node_modify_next_engine(
        ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t   *compat_param,
        ioc_fm_pcd_cc_node_modify_next_engine_params_t          *param,
        uint8_t                                                 compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->id = compat_ptr(compat_param->id);
        param->key_indx = compat_param->key_indx;
        param->key_size = compat_param->key_size;
    }
    else
    {
        compat_param->id = ptr_to_compat(param->id);
        compat_param->key_indx = param->key_indx;
        compat_param->key_size = param->key_size;
    }

    compat_copy_fm_pcd_cc_next_engine(
            &compat_param->cc_next_engine_params,
            &param->cc_next_engine_params,
            compat);
}

void compat_fm_pcd_cc_tree_modify_next_engine(
        ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t   *compat_param,
        ioc_fm_pcd_cc_tree_modify_next_engine_params_t          *param,
        uint8_t                                                 compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->id       = compat_ptr(compat_param->id);
        param->grp_indx = compat_param->grp_indx;
        param->indx     = compat_param->indx;
    }
    else
    {
        compat_param->id       = ptr_to_compat(param->id);
        compat_param->grp_indx = param->grp_indx;
        compat_param->indx     = param->indx;
    }

    compat_copy_fm_pcd_cc_next_engine(
            &compat_param->cc_next_engine_params,
            &param->cc_next_engine_params,
            compat);
}

void compat_copy_fm_pcd_cc_grp(
        ioc_compat_fm_pcd_cc_grp_params_t *compat_param,
        ioc_fm_pcd_cc_grp_params_t *param,
        uint8_t compat)
{
    int k;

    _fm_cpt_dbg (compat, " Calling...\n");

    if (compat == COMPAT_US_TO_K)
    {
        param->num_of_distinction_units = compat_param->num_of_distinction_units;
        memcpy(param->unit_ids, compat_param->unit_ids, IOC_FM_PCD_MAX_NUM_OF_CC_UNITS);
    }
    else
    {
        compat_param->num_of_distinction_units = param->num_of_distinction_units;
        memcpy(compat_param->unit_ids, param->unit_ids, IOC_FM_PCD_MAX_NUM_OF_CC_UNITS);
    }

    for (k=0; k < IOC_FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP; k++)
        compat_copy_fm_pcd_cc_next_engine(
                &compat_param->next_engine_per_entries_in_grp[k],
                &param->next_engine_per_entries_in_grp[k],
                compat);

    _fm_cpt_dbg (compat, " Called.\n");
}

void compat_copy_fm_pcd_cc_tree(
        ioc_compat_fm_pcd_cc_tree_params_t *compat_param,
        ioc_fm_pcd_cc_tree_params_t *param,
        uint8_t compat)
{
    int k;
    _fm_cpt_dbg (compat, " Calling...\n");

    if (compat == COMPAT_US_TO_K)
    {
        param->net_env_id = compat_get_id2ptr(compat_param->net_env_id, PCD_NODE);
        param->num_of_groups = compat_param->num_of_groups;
    }
    else
    {
        compat_param->net_env_id = compat_get_ptr2id(param->net_env_id, PCD_NODE);
        compat_param->num_of_groups = param->num_of_groups;

        compat_param->id = compat_add_ptr2id(param->id, PCD_NODE);
    }

    for (k=0; k < IOC_FM_PCD_MAX_NUM_OF_CC_GROUPS; k++)
        compat_copy_fm_pcd_cc_grp(
                &compat_param->fm_pcd_cc_group_params[k],
                &param->fm_pcd_cc_group_params[k],
                compat);

    _fm_cpt_dbg (compat, " Called.\n");
}

void compat_copy_fm_pcd_cc_delete_tree(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
        compat_del_ptr2id(id->obj, PCD_NODE);
    }
}

void compat_fm_pcd_prs_sw(
        ioc_compat_fm_pcd_prs_sw_params_t *compat_param,
        ioc_fm_pcd_prs_sw_params_t *param,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->override = compat_param->override;
        param->size = compat_param->size;
        param->base = compat_param->base;
        param->p_code = compat_ptr(compat_param->p_code);
        memcpy(param->sw_prs_data_params,compat_param->sw_prs_data_params,IOC_FM_PCD_PRS_NUM_OF_HDRS*sizeof(uint32_t));
        param->num_of_labels = compat_param->num_of_labels;
        memcpy(param->labels_table,compat_param->labels_table,IOC_FM_PCD_PRS_NUM_OF_LABELS*sizeof(ioc_fm_pcd_prs_label_params_t));
    }
}

void compat_copy_fm_pcd_kg_scheme(
        ioc_compat_fm_pcd_kg_scheme_params_t    *compat_param,
        ioc_fm_pcd_kg_scheme_params_t           *param,
        uint8_t                                 compat)
{
    _fm_cpt_dbg(compat," Calling...\n");

    if (compat == COMPAT_US_TO_K)
    {
        param->modify = compat_param->modify;

        /* scm_id */
        if(compat_param->modify){
            param->scm_id.scheme_id = compat_ptr(compat_param->scm_id.scheme_id);
            _fm_cpt_dbg(compat," param->scm_id.scheme_id = %p \n", param->scm_id.scheme_id);
        }
        else
            param->scm_id.relative_scheme_id = compat_param->scm_id.relative_scheme_id;

        param->always_direct = compat_param->always_direct;
        /* net_env_params */
        param->net_env_params.net_env_id = compat_get_id2ptr(compat_param->net_env_params.net_env_id, PCD_NODE);
        param->net_env_params.num_of_distinction_units = compat_param->net_env_params.num_of_distinction_units;
        memcpy(param->net_env_params.unit_ids,
               compat_param->net_env_params.unit_ids,
               IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS);

        param->use_hash = compat_param->use_hash;
        memcpy(&param->key_extract_and_hash_params,
               &compat_param->key_extract_and_hash_params,
               sizeof(ioc_fm_pcd_kg_key_extract_and_hash_params_t));
        param->bypass_fqid_generation = compat_param->bypass_fqid_generation;
        param->base_fqid = compat_param->base_fqid;
        param->numOfUsedExtractedOrs = compat_param->numOfUsedExtractedOrs;
		memcpy(param->extracted_ors,
               compat_param->extracted_ors,
               IOC_FM_PCD_KG_NUM_OF_GENERIC_REGS * sizeof(ioc_fm_pcd_kg_extracted_or_params_t));
        param->next_engine = compat_param->next_engine;

    	/* kg_next_engine_params */
        if(param->next_engine == e_IOC_FM_PCD_CC){
            param->kg_next_engine_params.cc.tree_id = compat_get_id2ptr(compat_param->kg_next_engine_params.cc.tree_id, PCD_NODE);

            param->kg_next_engine_params.cc.grp_id = compat_param->kg_next_engine_params.cc.grp_id;
            param->kg_next_engine_params.cc.plcr_next = compat_param->kg_next_engine_params.cc.plcr_next;
            param->kg_next_engine_params.cc.bypass_plcr_profile_generation = compat_param->kg_next_engine_params.cc.bypass_plcr_profile_generation;
            memcpy(&param->kg_next_engine_params.cc.plcr_profile,
                   &compat_param->kg_next_engine_params.cc.plcr_profile,
                   sizeof(ioc_fm_pcd_kg_plcr_profile_t));
        }
        else
			memcpy(&param->kg_next_engine_params,
                   &compat_param->kg_next_engine_params,
                   sizeof(param->kg_next_engine_params));

        memcpy(&param->scheme_counter,
               &compat_param->scheme_counter,
               sizeof(ioc_fm_pcd_kg_scheme_counter_t));
    }
    else
    {
        compat_param->modify = param->modify;

        /* scm_id */
        if(param->modify)
            compat_param->scm_id.scheme_id = ptr_to_compat(param->scm_id.scheme_id);
        else
            compat_param->scm_id.relative_scheme_id = param->scm_id.relative_scheme_id;

        compat_param->always_direct = param->always_direct;

        /* net_env_params */
        compat_param->net_env_params.net_env_id = ptr_to_compat(param->net_env_params.net_env_id);
        compat_param->net_env_params.num_of_distinction_units = param->net_env_params.num_of_distinction_units;
        memcpy(compat_param->net_env_params.unit_ids, param->net_env_params.unit_ids, IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS);

        compat_param->use_hash = param->use_hash;
        memcpy(&compat_param->key_extract_and_hash_params, &param->key_extract_and_hash_params, sizeof(ioc_fm_pcd_kg_key_extract_and_hash_params_t));
        compat_param->bypass_fqid_generation = param->bypass_fqid_generation;
        compat_param->base_fqid = param->base_fqid;
        compat_param->numOfUsedExtractedOrs = param->numOfUsedExtractedOrs;
		memcpy(compat_param->extracted_ors, param->extracted_ors, IOC_FM_PCD_KG_NUM_OF_GENERIC_REGS * sizeof(ioc_fm_pcd_kg_extracted_or_params_t));
        compat_param->next_engine = param->next_engine;

        /* kg_next_engine_params */
        if(compat_param->next_engine == e_IOC_FM_PCD_CC){
            compat_param->kg_next_engine_params.cc.tree_id = compat_get_ptr2id(param->kg_next_engine_params.cc.tree_id, PCD_NODE);

            compat_param->kg_next_engine_params.cc.grp_id = param->kg_next_engine_params.cc.grp_id;
            compat_param->kg_next_engine_params.cc.plcr_next = param->kg_next_engine_params.cc.plcr_next;
            compat_param->kg_next_engine_params.cc.bypass_plcr_profile_generation = param->kg_next_engine_params.cc.bypass_plcr_profile_generation;
            memcpy(&compat_param->kg_next_engine_params.cc.plcr_profile,
                   &param->kg_next_engine_params.cc.plcr_profile,
                   sizeof(ioc_fm_pcd_kg_plcr_profile_t));
        }
        else
            memcpy(&param->kg_next_engine_params, &compat_param->kg_next_engine_params, sizeof(compat_param->kg_next_engine_params));

        memcpy(&compat_param->scheme_counter, &param->scheme_counter, sizeof(ioc_fm_pcd_kg_scheme_counter_t));

        compat_param->id = compat_add_ptr2id(param->id, PCD_NODE);
    }

    _fm_cpt_dbg(compat," Called.\n");
}

void compat_copy_fm_pcd_scheme_delete(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
        compat_del_ptr2id(id->obj, PCD_NODE);
    }
}

void compat_copy_fm_pcd_kg_scheme_select(
        ioc_compat_fm_pcd_kg_scheme_select_t *compat_param,
        ioc_fm_pcd_kg_scheme_select_t *param,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K){
        param->direct = compat_param->direct;
        param->scheme_id = compat_ptr(compat_param->direct);
    }
}

void compat_copy_fm_pcd_kg_schemes_params(
        ioc_compat_fm_pcd_port_schemes_params_t *compat_param,
        ioc_fm_pcd_port_schemes_params_t *param,
        uint8_t compat)
{
    int k;

    if (compat == COMPAT_US_TO_K) {
        param->num_of_schemes = compat_param->num_of_schemes;
        for(k=0; k < compat_param->num_of_schemes; k++)
            param->scheme_ids[k] = compat_get_id2ptr(compat_param->schemes_ids[k], PCD_NODE);
    }
}

void compat_copy_fm_port_pcd_cc(
    ioc_compat_fm_port_pcd_cc_params_t *compat_cc_params ,
    ioc_fm_port_pcd_cc_params_t *p_cc_params,
    uint8_t compat)
{
    if (compat == COMPAT_US_TO_K){
        p_cc_params->cc_tree_id = compat_get_id2ptr(compat_cc_params->cc_tree_id, PCD_NODE);
    }
}

void compat_copy_fm_port_pcd_kg(
        ioc_compat_fm_port_pcd_kg_params_t *compat_param,
        ioc_fm_port_pcd_kg_params_t *param,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K){
        uint8_t k;

        param->num_of_schemes = compat_param->num_of_schemes;
        for(k=0; k<compat_param->num_of_schemes; k++)
            if (compat_param->schemes_ids[k])
                param->schemes_ids[k] = compat_get_id2ptr(compat_param->schemes_ids[k], PCD_NODE);

        param->direct_scheme = compat_param->direct_scheme;
        if (param->direct_scheme)
            param->direct_scheme_id = compat_get_id2ptr(compat_param->direct_scheme_id, PCD_NODE);
    }
}

void compat_copy_fm_port_pcd(
        ioc_compat_fm_port_pcd_params_t *compat_param,
        ioc_fm_port_pcd_params_t *param,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K){
        ioc_fm_port_pcd_prs_params_t         *same_port_pcd_prs_params;
        ioc_compat_fm_port_pcd_cc_params_t   *compat_port_pcd_cc_params;
        ioc_compat_fm_port_pcd_kg_params_t   *compat_port_pcd_kg_params;
        ioc_compat_fm_port_pcd_plcr_params_t *compat_port_pcd_plcr_params;

        same_port_pcd_prs_params    = (ioc_fm_port_pcd_prs_params_t *) (compat_param + 1);
        compat_port_pcd_cc_params   = (ioc_compat_fm_port_pcd_cc_params_t *) (same_port_pcd_prs_params + 1);
        compat_port_pcd_kg_params   = (ioc_compat_fm_port_pcd_kg_params_t *) (compat_port_pcd_cc_params + 1);
        compat_port_pcd_plcr_params = (ioc_compat_fm_port_pcd_plcr_params_t *) (compat_port_pcd_kg_params + 1);

        _fm_cpt_dbg(compat,"\n param->p_prs_params=%p \n", param->p_prs_params);
        _fm_cpt_dbg(compat," param->p_cc_params=%p  \n", param->p_cc_params);
        _fm_cpt_dbg(compat," param->p_kg_params=%p  \n", param->p_kg_params);
        _fm_cpt_dbg(compat," param->p_plcr_params=%p  \n", param->p_plcr_params);
        _fm_cpt_dbg(compat," param->p_ip_reassembly_manip=%p  \n", param->p_ip_reassembly_manip);

        param->pcd_support = compat_param->pcd_support;
        param->net_env_id = compat_get_id2ptr(compat_param->net_env_id, PCD_NODE);

        if (param->p_cc_params)
            compat_copy_fm_port_pcd_cc(compat_port_pcd_cc_params, param->p_cc_params, COMPAT_US_TO_K);
        if (param->p_kg_params)
            compat_copy_fm_port_pcd_kg(compat_port_pcd_kg_params, param->p_kg_params, COMPAT_US_TO_K);
        if (param->p_plcr_params)
            param->p_plcr_params->plcr_profile_id = compat_ptr(compat_port_pcd_plcr_params->plcr_profile_id);
        if (param->p_ip_reassembly_manip)
            param->p_ip_reassembly_manip = compat_get_id2ptr(compat_param->p_ip_reassembly_manip, PCD_NODE);
    }
}

void compat_copy_fm_port_pcd_modify_tree(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
}

void compat_copy_fm_pcd_net_env(
        ioc_compat_fm_pcd_net_env_params_t *compat_param,
        ioc_fm_pcd_net_env_params_t *param,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->num_of_distinction_units = compat_param->num_of_distinction_units;
        memcpy(param->units, compat_param->units, sizeof(ioc_fm_pcd_distinction_unit_t)*IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS);
        param->id = compat_ptr(compat_param->id); /* this should be null */
    }
    else
    {
        compat_param->num_of_distinction_units = param->num_of_distinction_units;
        memcpy(compat_param->units, param->units, sizeof(ioc_fm_pcd_distinction_unit_t)*IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS);
		compat_param->id = compat_add_ptr2id(param->id, PCD_NODE);
    }
}

void compat_copy_fm_net_env_delete(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
        compat_del_ptr2id(id->obj, PCD_NODE);
    }
}

void compat_copy_fm_pcd_cc_node_modify_key(
        ioc_compat_fm_pcd_cc_node_modify_key_params_t   *compat_param,
        ioc_fm_pcd_cc_node_modify_key_params_t          *param,
        uint8_t                                         compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        param->id       = compat_ptr(compat_param->id);
        _fm_cpt_dbg(compat," param->id = %p \n", param->id);
        param->key_indx = compat_param->key_indx;
        param->key_size = compat_param->key_size;
        param->p_key    = (uint8_t *)compat_ptr(compat_param->p_key);
        _fm_cpt_dbg(compat," param->p_key = %p \n", param->p_key);
        param->p_mask   = (uint8_t *)compat_ptr(compat_param->p_mask);
        _fm_cpt_dbg(compat," param->p_mask = %p\n", param->p_mask);
    }
    else
    {
        compat_param->key_indx  = param->key_indx;
        compat_param->key_size  = param->key_size;
        compat_param->p_key     = ptr_to_compat((void *)param->p_key);
        compat_param->p_mask    = ptr_to_compat((void *)param->p_mask);
        compat_param->id        = compat_add_ptr2id(param->id, PCD_NODE);
    }
}

void compat_copy_keys(
        ioc_compat_keys_params_t *compat_param,
        ioc_keys_params_t *param,
        uint8_t compat)
{
    int k = 0;

    _fm_cpt_dbg(compat," Calling...\n");

    if (compat == COMPAT_US_TO_K){
        param->max_num_of_keys = compat_param->max_num_of_keys;
        param->mask_support = compat_param->mask_support;
        param->statistics_mode = compat_param->statistics_mode;
        param->num_of_keys = compat_param->num_of_keys;
        param->key_size = compat_param->key_size;
    }
    else {

        compat_param->max_num_of_keys = param->max_num_of_keys;
        compat_param->mask_support = param->mask_support;
        compat_param->statistics_mode = param->statistics_mode;
        compat_param->num_of_keys = param->num_of_keys;
        compat_param->key_size = param->key_size;
    }

    for(k=0;k<IOC_FM_PCD_MAX_NUM_OF_KEYS;k++)
        compat_copy_fm_pcd_cc_key(
            &compat_param->key_params[k],
            &param->key_params[k],
             compat);

    compat_copy_fm_pcd_cc_next_engine(
            &compat_param->cc_next_engine_params_for_miss,
            &param->cc_next_engine_params_for_miss,
            compat);

    _fm_cpt_dbg(compat," Called.\n");
}

void compat_copy_fm_pcd_cc_node(
        ioc_compat_fm_pcd_cc_node_params_t  *compat_param,
        ioc_fm_pcd_cc_node_params_t         *param,
        uint8_t                             compat)
{
    _fm_cpt_dbg(compat," Calling...\n");

    if (compat == COMPAT_US_TO_K)
        memcpy(&param->extract_cc_params, &compat_param->extract_cc_params, sizeof(ioc_fm_pcd_extract_entry_t));

    else
    {
        compat_copy_keys(&compat_param->keys_params, &param->keys_params, compat);

        compat_param->id = compat_add_ptr2id(param->id, PCD_NODE);
        _fm_cpt_dbg(compat," param->id = %p \n", param->id);
    }

    compat_copy_keys(&compat_param->keys_params, &param->keys_params, compat);

    _fm_cpt_dbg(compat," Called.\n");
}

void compat_copy_fm_pcd_cc_delete_node(
    ioc_compat_fm_obj_t *compat_id,
    ioc_fm_obj_t *id,
    uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
        compat_del_ptr2id(id->obj, PCD_NODE);
    }
}

void compat_fm_pcd_manip_set_node(
        ioc_compat_fm_pcd_manip_params_t *compat_param,
        ioc_fm_pcd_manip_params_t *param,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K) {
        param->type = compat_param->type;
        memcpy(&param->u, &compat_param->u, sizeof(param->u));

        /* user garbage - it could break the US application! */
        if (compat_param->type == e_IOC_FM_PCD_MANIP_HDR &&
            compat_param->u.hdr.insrt_params.type == e_IOC_FM_PCD_MANIP_INSRT_GENERIC)
                param->u.hdr.insrt_params.u.generic.p_data =
                    compat_ptr(compat_param->u.hdr.insrt_params.u.generic.p_data);

        if (compat_param->p_next_manip)
            param->p_next_manip = compat_get_id2ptr(compat_param->id, PCD_NODE);
    }
    else {
        compat_param->type = param->type;
        memcpy(&compat_param->u, &param->u, sizeof(compat_param->u));

        /* user garbage - it could break the US application! */
        if (param->type == e_IOC_FM_PCD_MANIP_HDR &&
            param->u.hdr.insrt_params.type == e_IOC_FM_PCD_MANIP_INSRT_GENERIC)
                compat_param->u.hdr.insrt_params.u.generic.p_data =
                    ptr_to_compat(param->u.hdr.insrt_params.u.generic.p_data);

        if (param->p_next_manip)
            compat_param->p_next_manip = compat_get_ptr2id(param->id, PCD_NODE);

        compat_param->id = compat_add_ptr2id(param->id, PCD_NODE);
    }
}

void compat_copy_fm_pcd_manip_delete_node(
        ioc_compat_fm_obj_t *compat_id,
        ioc_fm_obj_t *id,
        uint8_t compat)
{
    if (compat == COMPAT_US_TO_K)
    {
        id->obj = compat_get_id2ptr(compat_id->obj, PCD_NODE);
        compat_del_ptr2id(id->obj, PCD_NODE);
    }
}
