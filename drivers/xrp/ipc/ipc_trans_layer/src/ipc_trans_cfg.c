/**
 * @file  ipc_trans_cfg.c
 * @brief this file is used as ipc global configuration management, you could get any config info from here
 * @note
 * @details feature list
 * 1. filtering rule definition
 */

#include "ipc_trans_cfg.h"
#include "../../ipc_hw_layer/include/ipc_hw_layer.h"
#include "../include/config.h"
// #include <stdio.h>

enum flt_status
{
    FLT_UNUSED = 0,
    FLT_SETTED = 1,
    FLT_ERR,
    FLT_STATUS_MAX,
};

struct ipc_flt_cfg
{
    struct msgbx_flt_cfg cfg;
    struct msgbx_flt_info info;
    struct msgbx_flt_rule_cfg rule;
    enum flt_status status; // 0 unset, 1 setted
};

static struct ipc_flt_cfg cfg_map[CHANNEL_COUNT];

/**
 * @name  flt_cfg_init
 * @param in: 
 * @param out: 
 * @return result
 * @details
 */
int32_t flt_cfg_init()
{
    uint8_t cnt = 0;

    for (cnt = 0; cnt < CHANNEL_COUNT; cnt++) {
        cfg_map[cnt].status = FLT_UNUSED;
    }

    return 0;
}

/**
 * @name  get_filter_rules
 * @param in: fid, cfg_loc, sid_range, ver
 * @param out: res_min, res_max, flt_thr
 * @return send result
 * @details
 */
int32_t get_flt_rules(const uint8_t fid, struct msgbx_flt_rule_cfg *rule)
{
#if (IPC_HW_LAYER_VERSION == 1)
if (rule->cfg_type == IPC_FLT_RULE_USER) {
    rule->rule_user.cfg_loc = IPC_FLT_RULE_HEADER;
    rule->rule_user.msgh_combi_lh_comp = 1;
    rule->rule_user.msgh_flilter_invert = 0;

    rule->rule_user.rx_res_min = fid << 8;
    rule->rule_user.rx_res_max = rule->rule_user.rx_res_min | 0xF0;
    rule->rule_user.tx_reserved_filter_mask = 8;

}
#endif
    return 0;
}

int32_t set_flt_rules(const uint8_t fid)
{
    int32_t ret = -1;

    if (fid >= CHANNEL_COUNT)
        return -1;

    if (cfg_map[fid].status != FLT_UNUSED)
        return -2;

    if (fid == 0) {
        cfg_map[fid].status = FLT_SETTED;
        return 0;
    }
    // set rules
    cfg_map[fid].cfg.flt_id = fid;
    cfg_map[fid].rule.rule_user.cfg_loc = IPC_FLT_RULE_HEADER;
    cfg_map[fid].rule.cfg_type = IPC_FLT_RULE_USER;

    get_flt_rules(fid, &cfg_map[fid].rule);
    printf("flt min = %ld, max = %ld, mask = %ld\n", cfg_map[fid].rule.rule_user.rx_res_min, cfg_map[fid].rule.rule_user.rx_res_max,
           cfg_map[fid].rule.rule_user.tx_reserved_filter_mask);

#ifdef IPC_STATE_MGT_ENABLE
    cfg_map[fid].cfg.mbx_flt_mgt_flag = MSG_FLT_MGT_CONFIG;
#endif

    ret = ipc_hw_layer_flt_rule_set(cfg_map[fid].cfg.flt_id, &cfg_map[fid].rule);
    if (ret < 0)
    {
        printf("ipc_hw_layer_flt_rule_set fail ret = %d\n", ret);
        return ret;
    }

    cfg_map[fid].status = FLT_SETTED;
    return 0;
}


// int32_t dump_flt_status(const uint8_t fid)
// {
//     return 0;
// }

// int32_t set_flt_rules_userdefined(const uint8_t fid, const struct msgbx_flt_cfg cfg)
// {

// }