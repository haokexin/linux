#ifndef IPC_APP_LAYER_SERDES_H
#define IPC_APP_LAYER_SERDES_H

#include "ipc_app_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        rw_msg_header header;
        uint8_t *ptr;
        uint32_t index;
        uint32_t pos;
        uint32_t rcv_index;
        rw_msg msg_pool[IPC_MAX_SUB_MSG_NUM];
    } serdes_t;

    static inline int32_t ipc_ser_init(serdes_t *serdes)
    {
        if (!serdes)
            return -1;
        serdes->index = 0;
        serdes->pos = 0;
        serdes->ptr = (uint8_t *)(serdes->msg_pool[serdes->index].payload);
        return 0;
    }

    static inline int32_t ipc_serdes_next_msg(serdes_t *serdes)
    {
        if (!serdes)
            return -1;
        ++serdes->index;
        serdes->ptr = (uint8_t *)(serdes->msg_pool[serdes->index].payload);
        serdes->pos = 0;
        return 0;
    }

    static inline int32_t ipc_ser_set_header(serdes_t *serdes, rw_msg_header header)
    {
        if (!serdes)
            return -1;
        serdes->header = header;
        return 0;
    }

    static inline int32_t ipc_ser_put(serdes_t *serdes, const uint8_t *data, uint32_t size)
    {
        if (!serdes || !data || size > (IPC_MAX_SUB_MSG_NUM - serdes->index) * IPC_PAYLOAD_SIZE - serdes->pos)
            return -1;

        uint32_t i = 0;
        while (i < size)
        {
            if (serdes->pos % 4 == 0 && size >= i + 4)
            {
                *(uint32_t *)serdes->ptr = *(uint32_t *)data;
                serdes->ptr += 4;
                serdes->pos += 4;
                data += 4;
                i += 4;
            }
            else
            {
                *(serdes->ptr++) = *data++;
                ++serdes->pos;
                ++i;
            }

            if (serdes->pos == IPC_PAYLOAD_SIZE)
                ipc_serdes_next_msg(serdes);
        }
        return 0;
    }

    static inline int32_t ipc_ser_put_string(serdes_t *serdes, const char *data)
    {
        if (!serdes || !data)
            return -1;

        uint32_t size = 0;
        const char *ptr = data;
        while (*ptr++ != '\0')
            ++size;
        return ipc_ser_put(serdes, (uint8_t *)data, ++size);
    }

    static inline int32_t ipc_ser_finish(serdes_t *serdes)
    {
        uint32_t i;
        if (!serdes)
            return -1;
        for (i = 0; i <= serdes->index; ++i)
        {
            serdes->msg_pool[i].header = serdes->header;
            serdes->msg_pool[i].header.len = IPC_PAYLOAD_SIZE / 8U;
            serdes->msg_pool[i].header.idx = i;
            serdes->msg_pool[i].header.is_eof = 0U;
        }
        serdes->msg_pool[serdes->index].header.len = serdes->pos / 8U + 1;
        serdes->msg_pool[serdes->index].header.is_eof = 1U;

        return 0;
    }

    static inline int32_t ipc_des_init(serdes_t *serdes)
    {
        if (!serdes)
            return -1;
        serdes->index = 0;
        serdes->rcv_index = 0;
        serdes->pos = 0;
        serdes->ptr = (uint8_t *)(serdes->msg_pool[serdes->index].payload);
        return 0;
    }

    static inline int32_t ipc_des_move_first(serdes_t *serdes)
    {
        if (!serdes)
            return -1;
        serdes->index = 0;
        serdes->pos = 0;
        serdes->msg_pool[0] = serdes->msg_pool[serdes->rcv_index];
        serdes->rcv_index = 0;
        return 0;
    }

    static inline int32_t ipc_des_validate_all(serdes_t *serdes)
    {
        if (!serdes)
            return -1;

        rw_msg *eof = &serdes->msg_pool[serdes->rcv_index];
        if (eof->header.is_eof != 1 || eof->header.idx != serdes->rcv_index)
            return -1;
        serdes->header = eof->header;
        uint32_t i = 0;
        for (; i < serdes->rcv_index; ++i)
        {
            rw_msg_header *header = &serdes->msg_pool[i].header;
            if (header->idx != i || header->is_eof != 0 || header->cid != serdes->header.cid ||
                header->cmd != serdes->header.cmd || header->fid != serdes->header.fid ||
                header->len != IPC_PAYLOAD_SIZE / 8U || header->pid != serdes->header.pid ||
                header->sid != serdes->header.sid || header->tok != serdes->header.tok ||
                header->typ != serdes->header.typ)
                return -1;
        }
        serdes->index = 0;
        serdes->pos = 0;
        serdes->ptr = (uint8_t *)serdes->msg_pool[0].payload;

        return 0;
    }

    static inline rw_msg *ipc_des_get_current_msg(serdes_t *serdes)
    {
        if (!serdes)
            return NULL;

        return &serdes->msg_pool[serdes->rcv_index];
    }

    static inline int32_t ipc_des_validate_msg(serdes_t *serdes)
    {
        if (!serdes)
            return -1;

        rw_msg *msg = ipc_des_get_current_msg(serdes);
        printf("message idx %d, index %d, eof %d\n", msg->header.idx, serdes->rcv_index, msg->header.is_eof);
        // check if index not match.
        if (msg->header.idx == 0)
        {
            serdes->header = msg->header;
            if (serdes->rcv_index != 0)
                (void)ipc_des_move_first(serdes);
        }
        else
        {
            if (msg->header.idx != serdes->rcv_index || msg->header.cid != serdes->header.cid ||
                msg->header.cmd != serdes->header.cmd || msg->header.fid != serdes->header.fid ||
                msg->header.pid != serdes->header.pid || msg->header.sid != serdes->header.sid ||
                msg->header.tok != serdes->header.tok || msg->header.typ != serdes->header.typ)
            {
                printf("message validation fail.\n");
                printf("message idx not match: get %d, expect %d\n", msg->header.idx, serdes->rcv_index);
                return -1;
            }
        }

        // check if end of frame.
        if (!msg->header.is_eof)
        {
            if (msg->header.len == IPC_PAYLOAD_SIZE / 8U)
                ++serdes->rcv_index;
            return -1;
        }

        serdes->index = 0;
        serdes->pos = 0;
        serdes->ptr = (uint8_t *)serdes->msg_pool[0].payload;

        return 0;
    }

    static inline int32_t ipc_des_get(serdes_t *serdes, uint8_t *data, uint32_t size)
    {
        if (!serdes || !data)
            return -1;

        uint32_t total_size =
            serdes->rcv_index * IPC_PAYLOAD_SIZE + serdes->msg_pool[serdes->rcv_index].header.len * 8U;
        uint32_t used = serdes->index * IPC_PAYLOAD_SIZE + serdes->pos;
        uint32_t rest = total_size - used;
        if (size > rest)
            return -1;

        uint32_t i = 0;
        while (i < size)
        {
            if (serdes->pos % 4 == 0 && size >= i + 4)
            {
                *(uint32_t *)data = *(uint32_t *)serdes->ptr;
                serdes->ptr += 4;
                serdes->pos += 4;
                data += 4;
                i += 4;
            }
            else
            {
                *data++ = *(serdes->ptr++);
                ++serdes->pos;
                ++i;
            }

            if (serdes->pos == IPC_PAYLOAD_SIZE)
                ipc_serdes_next_msg(serdes);
        }
        return 0;
    }

    static inline int32_t ipc_des_get_string(serdes_t *serdes, char *data, uint32_t size)
    {
        if (!serdes)
            return -1;

        uint32_t total_size =
            serdes->rcv_index * IPC_PAYLOAD_SIZE + serdes->msg_pool[serdes->rcv_index].header.len * 8U;
        uint32_t used = serdes->index * IPC_PAYLOAD_SIZE + serdes->pos;
        uint32_t rest = total_size - used;

        uint32_t old_index = serdes->index;
        uint32_t old_pos = serdes->pos;
        uint8_t *old_ptr = serdes->ptr;

        uint32_t i = 0;
        if (!data)
        {
            while (i < rest)
            {
                if (*(serdes->ptr++) == '\0')
                    break;
                ++serdes->pos;
                ++i;
                if (serdes->pos == IPC_PAYLOAD_SIZE)
                    ipc_serdes_next_msg(serdes);
            }
            serdes->index = old_index;
            serdes->pos = old_pos;
            serdes->ptr = old_ptr;
            if (i >= rest)
                return -3;
            return i + 1;
        }

        uint32_t num = rest < size ? rest : size;
        while (i < num)
        {
            if (*(serdes->ptr) == '\0')
            {
                *data = '\0';
                ++serdes->ptr;
                ++serdes->pos;
                if (serdes->pos == IPC_PAYLOAD_SIZE)
                    ipc_serdes_next_msg(serdes);
                break;
            }

            *data++ = *(serdes->ptr++);
            ++serdes->pos;
            ++i;
            if (serdes->pos == IPC_PAYLOAD_SIZE)
                ipc_serdes_next_msg(serdes);
        }
        if (i >= num)
        {
            data[i - 1] = '\0';
            serdes->index = old_index;
            serdes->pos = old_pos;
            serdes->ptr = old_ptr;
            return -2;
        }
        return i + 1;
    }

    static inline int32_t ipc_serdes_print(serdes_t *serdes)
    {
        int i, j;
        if (!serdes)
            return -1;

        for (i = 0; i < IPC_MAX_SUB_MSG_NUM; ++i)
            for (j = 0; j < IPC_PAYLOAD_SIZE/8; ++j)
                printf("msg %d, payload %d : %0lx\n", i,j,serdes->msg_pool[i].payload[j]);

        return 0;
    }

#ifdef __cplusplus
}
#endif

#endif
