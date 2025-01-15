#ifndef _IPC_TRANS_CFG_H
#define _IPC_TRANS_CFG_H

#include "../../ipc_hw_layer/include/config.h"
#include "bstipc_cfg.h"

int32_t set_flt_rules(const uint8_t fid);
int32_t dump_flt_status(const uint8_t fid);
int32_t flt_cfg_init(void);
// int32_t set_flt_rules_userdefined(const uint8_t fid, const struct msgbx_flt_cfg cfg);

#endif