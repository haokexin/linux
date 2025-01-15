#ifndef _IPC_TRANS_SES_MGT_H
#define _IPC_TRANS_SES_MGT_H

#include "bstipc_cfg.h"

typedef struct ipc_ses_base
{
    uint8_t sid;
    uint8_t fid;
    uint8_t role;
    uint8_t pid;
} ses_base;

typedef enum ipc_session_status {
    SES_STATE_INVALID = 0, 
    SES_STATE_INIT = 1,
    SES_STATE_DESTROY
} ses_status_e;

static inline int32_t session_comb(uint8_t sid, uint8_t fid, uint32_t *info)
{
    *info = ((sid << 8) | fid);
    return 0;
}

static inline int32_t session_dist(uint32_t info, uint8_t *sid, uint8_t *fid)
{
    *sid = info >> 8;
    *fid = info & 0x0f;
    return 0;
}

int32_t session_mgt_init(void);
int32_t session_register(const ses_base info, uint32_t *session_id);
int32_t session_destroy(const uint32_t session_id);

int8_t session_isvalid(const uint32_t session_id);

int32_t session_msg_in(const uint32_t session_id, const rw_msg *msg);
int32_t session_msg_out(const uint32_t session_id, const uint8_t type, rw_msg *msg);
int32_t session_set_status(const uint32_t session_id, const ses_status_e status);
ses_status_e session_get_status(const uint32_t session_id);

#endif