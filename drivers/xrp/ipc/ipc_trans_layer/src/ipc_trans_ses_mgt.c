/**
 * @file  ipc_trans_ses_mgt.c
 * @brief this file is used as session management
 * @note
 * @details feature list
 * 1.
 */

#include "ipc_trans_ses_mgt.h"
#include "../include/config.h"
#include "../include/ipc_trans_common.h"
#include "ipc_trans_buffer.h"

// if info.role == client, then quque_1 means reply message, quque_2 means signal message
// if info.role == server, then these 2 queues all mean method message

struct client_msg_buf
{
    struct buffer reply_buf;
    struct buffer signal_buf;
};

struct server_msg_buf
{
    struct buffer method_buf;
};

struct ipc_ses
{
    ses_base info;
    ses_status_e status;
    enum ipc_ses_role_e ses_type;
    union
    {
        struct client_msg_buf client_buf;
        struct server_msg_buf server_buf;
    };
};

#define SID_INVALID_VALUE 16

/**
 * prerequisition:
 * 1. each filter has a independent and consecutive session range
 *  which means we should get filter range from hw_layer
 */
#if (IPC_TRANS_LAYER_SES_MODE == 1)
static struct ipc_ses ses_map;
#else
static struct ipc_ses ses_map[CHANNEL_COUNT][SESSION_COUNT];
#endif

int32_t session_mgt_init()
{
#if (IPC_TRANS_LAYER_SES_MODE == 1)  
    ses_map.info.sid = SID_INVALID_VALUE;
    ses_map.info.fid = SID_INVALID_VALUE;
    ses_map.info.role = SID_INVALID_VALUE;
    ses_map.info.pid = SID_INVALID_VALUE;
    ses_map.status = SES_STATE_INVALID;
    ses_map.ses_type = IPC_SES_ROLE_INVALID;
#else
    uint8_t fid_cnt = 0, sid_cnt = 0;
    for (fid_cnt = 0; fid_cnt < CHANNEL_COUNT; fid_cnt++) {
        for (sid_cnt = 0; sid_cnt < SESSION_COUNT; sid_cnt++) {
            ses_map[fid_cnt][sid_cnt].info.sid = SID_INVALID_VALUE;
            ses_map[fid_cnt][sid_cnt].info.fid = SID_INVALID_VALUE;
            ses_map[fid_cnt][sid_cnt].info.role = SID_INVALID_VALUE;
            ses_map[fid_cnt][sid_cnt].info.pid = SID_INVALID_VALUE;
            ses_map[fid_cnt][sid_cnt].status = SES_STATE_INVALID;
            ses_map[fid_cnt][sid_cnt].ses_type = IPC_SES_ROLE_INVALID;
        }
    }
#endif
    return 0;
}

// note: sid valid range is from 0 ~ 15
// we assign 16 as invalid sid value, and session mgt need init ses_map to 16
int8_t session_isvalid(const uint32_t session_id)
{
#if (IPC_TRANS_LAYER_SES_MODE == 1)
    return 0;
#else
    uint8_t sid = 0, fid = 0;
    session_dist(session_id, &sid, &fid);
    if (sid > SESSION_COUNT || fid > CHANNEL_COUNT)
    {
        return -1;
    }
    if (ses_map[fid][sid].info.sid != sid)
        return -2;
    else
        return 0;
#endif
}

int32_t session_register(const ses_base info, uint32_t *session_id)
{
    // sid valid check
    // keyword: repetition and out of range
    if (info.sid > SESSION_COUNT || info.fid > CHANNEL_COUNT)
    {
        return -1;
    }

#if (IPC_TRANS_LAYER_SES_MODE == 1)
    if (ses_map.info.role != SID_INVALID_VALUE) {
        printf("session is occupied\n");
        return -2;
    }

    ses_map.info.sid = info.sid;
    ses_map.info.fid = info.fid;
    ses_map.info.role = info.role;
    ses_map.info.pid = info.pid;
    ses_map.status = SES_STATE_INIT;
    ses_map.ses_type = info.role;
    if (ses_map.ses_type == IPC_SES_ROLE_CLIENT) {
        buffer_Malloc(&ses_map.client_buf.reply_buf);
        buffer_Malloc(&ses_map.client_buf.signal_buf);
    } else if (ses_map.ses_type == IPC_SES_ROLE_SERVER){
        buffer_Malloc(&ses_map.server_buf.method_buf);
    } else {
        printf("ses role is invalid \n");
    }
    // todo: here need a special id
    *session_id = info.sid;
#else
    if (ses_map[info.fid][info.sid].info.role != SID_INVALID_VALUE)
    {
        printf("session is occupied\n");
        return -2;
    }

    ses_map[info.fid][info.sid].info.sid = info.sid;
    ses_map[info.fid][info.sid].info.fid = info.fid;
    ses_map[info.fid][info.sid].info.role = info.role;
    ses_map[info.fid][info.sid].info.pid = info.pid;
    ses_map[info.fid][info.sid].status = SES_STATE_INIT;
    ses_map[info.fid][info.sid].ses_type = info.role;
    if (ses_map[info.fid][info.sid].ses_type == IPC_SES_ROLE_CLIENT) {
        buffer_Malloc(&ses_map[info.fid][info.sid].client_buf.reply_buf);
        buffer_Malloc(&ses_map[info.fid][info.sid].client_buf.signal_buf);
    } else if (ses_map[info.fid][info.sid].ses_type == IPC_SES_ROLE_SERVER){
        buffer_Malloc(&ses_map[info.fid][info.sid].server_buf.method_buf);
    } else {
        printf("ses role is invalid \n");
    }
    session_comb(info.sid, info.fid, session_id);
#endif
    return 0;
}

int32_t session_map_clear(uint8_t sid, uint8_t fid)
{
#if (IPC_TRANS_LAYER_SES_MODE == 1)
    ses_map.info.sid = SID_INVALID_VALUE;
    ses_map.info.fid = SID_INVALID_VALUE;
    ses_map.info.role = SID_INVALID_VALUE;
    ses_map.info.pid = SID_INVALID_VALUE;
    ses_map.status = SES_STATE_INVALID;
    ses_map.ses_type = IPC_SES_ROLE_INVALID;
#else
    ses_map[fid][sid].info.sid = SID_INVALID_VALUE;
    ses_map[fid][sid].info.fid = SID_INVALID_VALUE;
    ses_map[fid][sid].info.role = SID_INVALID_VALUE;
    ses_map[fid][sid].info.pid = SID_INVALID_VALUE;
    ses_map[fid][sid].status = SES_STATE_INVALID;
    ses_map[fid][sid].ses_type = IPC_SES_ROLE_INVALID;
#endif
    return 0;
}

int32_t session_destroy(const uint32_t session_id)
{
    if (session_isvalid(session_id) < 0)
    {
        printf("session destroy sid %u is invalid\n", session_id);
        return -1;
    }
    uint8_t sid = 0, fid = 0;

#if (IPC_TRANS_LAYER_SES_MODE == 1)
    ses_map.status = SES_STATE_DESTROY;
    if (ses_map.ses_type == IPC_SES_ROLE_CLIENT) {
        buffer_Free(&ses_map.client_buf.reply_buf);
        buffer_Free(&ses_map.client_buf.signal_buf);
    } else if (ses_map.ses_type == IPC_SES_ROLE_SERVER) {
        buffer_Free(&ses_map.server_buf.method_buf);
    } else {
        printf("session destroy type is invalid\n");
    }
#else
    session_dist(session_id, &sid, &fid);
    ses_map[fid][sid].status = SES_STATE_DESTROY;
    if (ses_map[fid][sid].ses_type == IPC_SES_ROLE_CLIENT) {
        buffer_Free(&ses_map[fid][sid].client_buf.reply_buf);
        buffer_Free(&ses_map[fid][sid].client_buf.signal_buf);
    } else if (ses_map[fid][sid].ses_type == IPC_SES_ROLE_SERVER) {
        buffer_Free(&ses_map[fid][sid].server_buf.method_buf);
    } else {
        printf("session destroy type is invalid\n");
    }
#endif
    session_map_clear(sid, fid);
    return 0;
}

int32_t session_msg_in(const uint32_t session_id, const rw_msg *msg)
{
    int32_t ret = 0;
    if (session_isvalid(session_id) < 0) {
        printf("session_msg_in sid %d is invalid\n", session_id);
        return -1;
    }

    // check session role
#if (IPC_TRANS_LAYER_SES_MODE == 1)
    if (ses_map.status == SES_STATE_DESTROY) {
        printf("session_msg_in sid %d is destroy\n", session_id);
        return -2;
    }

    if (ses_map.info.role == IPC_SES_ROLE_CLIENT)
    {
        // put msg in different queue according to msg type
        if (!buffer_IsFull(&(ses_map.client_buf.reply_buf)) && msg->header.typ == IPC_MSG_TYPE_REPLY)
        {
            ret = buffer_In(&(ses_map.client_buf.reply_buf), (void*)msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        }
        if (!buffer_IsFull(&(ses_map.client_buf.signal_buf)) && msg->header.typ == IPC_MSG_TYPE_BROADCAST)
        {
            ret = buffer_In(&(ses_map.client_buf.signal_buf), (void*)msg, sizeof(rw_msg));
            return ((ret == 0) ? -4 : 0);
        } else {
            return -5;
        }
    }
    else if (ses_map.info.role == IPC_SES_ROLE_SERVER)
    {
        // put msg in spare queue
        if (!buffer_IsFull(&(ses_map.server_buf.method_buf)))
        {
            ret = buffer_In(&(ses_map.server_buf.method_buf), (void*)msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        } else {
            return -5;
        }
    }
    else
    {
        return -6;
    }
#else
    uint8_t sid = 0, fid = 0;
    session_dist(session_id, &sid, &fid);

    if (ses_map[fid][sid].status == SES_STATE_DESTROY) {
        printf("session_msg_in sid %d fid %d is destroy\n", sid, fid);
        return -2;
    }

    if (ses_map[fid][sid].info.role == IPC_SES_ROLE_CLIENT)
    {
        // put msg in different queue according to msg type
        if (!buffer_IsFull(&(ses_map[fid][sid].client_buf.reply_buf)) && msg->header.typ == IPC_MSG_TYPE_REPLY)
        {
            ret = buffer_In(&(ses_map[fid][sid].client_buf.reply_buf), (void*)msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        }
        if (!buffer_IsFull(&(ses_map[fid][sid].client_buf.signal_buf)) && msg->header.typ == IPC_MSG_TYPE_BROADCAST)
        {
            ret = buffer_In(&(ses_map[fid][sid].client_buf.signal_buf), (void*)msg, sizeof(rw_msg));
            return ((ret == 0) ? -4 : 0);
        }
        else
        {
            return -5;
        }
    }
    else if (ses_map[fid][sid].info.role == IPC_SES_ROLE_SERVER)
    {
        // put msg in spare queue
        if (!buffer_IsFull(&(ses_map[fid][sid].server_buf.method_buf)))
        {
            ret = buffer_In(&(ses_map[fid][sid].server_buf.method_buf), (void*)msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        } else {
            return -5;
        }
    }
    else
    {
        return -6;
    }
#endif
    return 0;
}

int32_t session_msg_out(const uint32_t session_id, const uint8_t type, rw_msg *msg)
{
    if (session_isvalid(session_id) < 0) {
        printf("session_msg_out sid %d is invalid\n", session_id);
        return -1;
    }
    int32_t ret = 0;

#if (IPC_TRANS_LAYER_SES_MODE == 1)
    if (ses_map.status == SES_STATE_DESTROY) {
        printf("session_msg_out sid %d is destroy\n", session_id);
        return -2;
    }

    if (ses_map.info.role == IPC_SES_ROLE_CLIENT)
    {
        // put msg in different queue according to msg type
        if (!buffer_IsEmpty(&(ses_map.client_buf.reply_buf)) && type == IPC_MSG_TYPE_REPLY)
        {
            ret = buffer_Out(&(ses_map.client_buf.reply_buf), msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        }
        if (!buffer_IsEmpty(&(ses_map.client_buf.signal_buf)) && type == IPC_MSG_TYPE_BROADCAST)
        {
            ret = buffer_Out(&(ses_map.client_buf.signal_buf), msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        }
        else
        {
            return -4;
        }
    }
    else if (ses_map.info.role == IPC_SES_ROLE_SERVER)
    {
        // put msg in spare queue
        if (!buffer_IsEmpty(&(ses_map.server_buf.method_buf)))
        {
            ret = buffer_Out(&(ses_map.server_buf.method_buf), msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        } else {
            return -4;
        }
    }
    else
    {
        return -5;
    }
#else
    uint8_t sid = 0, fid = 0;
    session_dist(session_id, &sid, &fid);

    if (ses_map[fid][sid].status == SES_STATE_DESTROY) {
        printf("session_msg_in sid %d fid %d is destroy\n", sid, fid);
        return -2;
    }

    if (ses_map[fid][sid].info.role == IPC_SES_ROLE_CLIENT)
    {
        // put msg in different queue according to msg type
        if (!buffer_IsEmpty(&(ses_map[fid][sid].client_buf.reply_buf)) && type == IPC_MSG_TYPE_REPLY)
        {
            ret = buffer_Out(&(ses_map[fid][sid].client_buf.reply_buf), msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        }
        if (!buffer_IsEmpty(&(ses_map[fid][sid].client_buf.signal_buf)) && type == IPC_MSG_TYPE_BROADCAST)
        {
            ret = buffer_Out(&(ses_map[fid][sid].client_buf.signal_buf), msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        }
        else
        {
            return -4;
        }
    }
    else if (ses_map[fid][sid].info.role == IPC_SES_ROLE_SERVER)
    {
        // put msg in spare queue
        if (!buffer_IsEmpty(&(ses_map[fid][sid].server_buf.method_buf)))
        {
            ret = buffer_Out(&(ses_map[fid][sid].server_buf.method_buf), msg, sizeof(rw_msg));
            return ((ret == 0) ? -3 : 0);
        } else {
            return -4;
        }
    }
    else
    {
        return -5;
    }
#endif
    return 0;
}

int32_t ipc_trans_get_avail_msg_typ(const uint32_t session_id, uint8_t *type)
{
#if (IPC_TRANS_LAYER_SES_MODE == 1)
    if (ses_map.info.role == IPC_SES_ROLE_CLIENT)
    {
        // put msg in different queue according to msg type
        if (!buffer_IsEmpty(&(ses_map.client_buf.reply_buf)))
        {
            *type = IPC_MSG_TYPE_REPLY;
        }
        else if (!buffer_IsEmpty(&(ses_map.client_buf.signal_buf)))
        {
            *type = IPC_MSG_TYPE_BROADCAST;
        }
        else
        {
            return -1;
        }
    }
    else if (ses_map.info.role == IPC_SES_ROLE_SERVER)
    {
        // put msg in spare queue
        if (!buffer_IsEmpty(&(ses_map.server_buf.method_buf)))
        {
            *type = IPC_MSG_TYPE_METHOD;
            return 0;
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -2;
    }
#else
    uint8_t sid = 0, fid = 0;

    session_dist(session_id, &sid, &fid);
    if (ses_map[fid][sid].info.role == IPC_SES_ROLE_CLIENT)
    {
        // put msg in different queue according to msg type
        if (!buffer_IsEmpty(&(ses_map[fid][sid].client_buf.reply_buf)))
        {
            *type = IPC_MSG_TYPE_REPLY;
        }
        else if (!buffer_IsEmpty(&(ses_map[fid][sid].client_buf.signal_buf)))
        {
            *type = IPC_MSG_TYPE_BROADCAST;
        }
        else
        {
            return -1;
        }
    }
    else if (ses_map[fid][sid].info.role == IPC_SES_ROLE_SERVER)
    {
        // put msg in spare queue
        if (!buffer_IsEmpty(&(ses_map[fid][sid].server_buf.method_buf)))
        {
            *type = IPC_MSG_TYPE_METHOD;
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -2;
    }
#endif
    return 0;
}

int32_t session_set_status(const uint32_t session_id, const ses_status_e status)
{
    if (session_isvalid(session_id) < 0) {
        printf("session_set_status sid %d is invalid\n", session_id);
        return -1;
    }
#if (IPC_TRANS_LAYER_SES_MODE == 1)
    ses_map.status = status;
#else
    uint8_t sid = 0, fid = 0;
    session_dist(session_id, &sid, &fid);
    ses_map[fid][sid].status = status;
#endif
    return 0;
}

ses_status_e session_get_status(const uint32_t session_id)
{
    if (session_isvalid(session_id) < 0) {
        printf("session_get_status sid %d is invalid\n", session_id);
        return SES_STATE_INVALID;
    }
    #if (IPC_TRANS_LAYER_SES_MODE == 1)
        return ses_map.status;
    #else
        uint8_t sid = 0, fid = 0;
        session_dist(session_id, &sid, &fid);
        return ses_map[fid][sid].status;
    #endif
}