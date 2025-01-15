#ifndef IPC_APP_LAYER_COMMON_H
#define IPC_APP_LAYER_COMMON_H

#include <bst/ipc_serdes.h>

//macro definition
#define IPC_PAYLOAD_SIZE 32U
#define IPC_MAX_DATA_SIZE IPC_MAX_SUB_MSG_NUM *IPC_PAYLOAD_SIZE
#define IPC_TOKEN_NUM 16U

//version definition
typedef struct
{
    uint8_t major;
    uint8_t minor;
}ipc_inf_version_t;

typedef struct 
{
    uint8_t *data;
    uint32_t size;
}byte_buffer;

typedef struct
{
    uint16_t uuid;
    uint16_t res[3];
    uint64_t begin;
    uint64_t end;
}ext_info_t;

#endif
