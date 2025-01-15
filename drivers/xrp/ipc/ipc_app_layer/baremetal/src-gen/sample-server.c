#include "sample-server.h"
#include "ipc_app_serdes.h"
#include "ipc_app_svr_utils.h"
#include "ipc_trans_common.h"
#include "ipc_trans_layer.h"

// macro definitions
#define PID 8U
#define FID 0U
#define SID 1U

#define MAX_TOKEN_NUM 16U
#define MAX_PAYLOAD_NUM 32U
#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// typde definitions

// local variables
static ipc_inf_version s_version = {.major = 1, .minor = 0};
static uint32_t s_handle = 0U;
static test_server s_server = {0};
static uint8_t s_token = 0;
static test_hello_t s_hello_ptr = NULL;
static broadcast_registry s_heartbeat_registry = {0};
static int8_t s_hello_name[MAX_STRING_SIZE] = {0};
static int8_t s_hello_message[MAX_STRING_SIZE] = {0};

static inline void increase_token()
{
    if (++s_token >= MAX_TOKEN_NUM)
        s_token = 0;
}

// interface implementation
// get interface version
static ipc_inf_version get_ipc_inf_version()
{
    return s_version;
}

// method
static int32_t register_test_hello(test_hello_t func)
{
    s_hello_ptr = func;
    return 0;
}
// broadcast
static int32_t test_heartbeat(uint8_t status)
{
    int32_t ret = 0;
    // int32_t send_ret = 0;
    int32_t index = 0;
    // rw_msg msg;
    rw_msg_header header = {0};
    header.pid = PID;
    header.len = sizeof(status) / 8 + 1;
    header.cmd = CMD_BROADCAST_HEARTBEAT;
    header.typ = IPC_MSG_TYPE_BROADCAST;

    serdes_t serdes = {0};
    (void)ipc_ser_init(&serdes);
    (void)ipc_ser_put(&serdes, &status, sizeof(status));

    broadcast_reg_entry *entry = s_heartbeat_registry.entries;
    for (index = s_heartbeat_registry.start, entry += s_heartbeat_registry.start; index < s_heartbeat_registry.end;
         ++index, ++entry)
    {
        if (entry->pid != 0)
        {
            uint32_t i = 0;
            header.cid = entry->pid;
            header.fid = entry->fid;
            header.sid = entry->sid;
            header.tok = s_token;
            ipc_ser_set_header(&serdes, header);
            ipc_ser_finish(&serdes);
            for (; i <= serdes.index; ++i)
            {
                int32_t send_ret = ipc_trans_layer_stub_send_broadcast(s_handle, serdes.msg_pool[i]);
                if (send_ret < 0)
                {
                    printf("send broadcast fail %d.\n", send_ret);
                    break;
                }
            }
            if (i > serdes.index)
                ++ret;
        }
    }
    increase_token();
    return ret;
}
// receive messages
static int32_t receive_message()
{
    return ipc_trans_layer_get_msg(s_handle);
}
// dispatch messages
static int32_t dispatch_message()
{
    int32_t ret = 0;
    static serdes_t deserializer = {0};
    static serdes_t serializer = {0};

    while (ipc_trans_layer_stub_get_method_msg(s_handle, ipc_des_get_current_msg(&deserializer)) >= 0)
    {
        ret = ipc_des_validate_msg(&deserializer);
        if (ret < 0)
            continue;
        // init serializer.
        // it cannot fail, as &serializer won't be NULL.
        (void)ipc_ser_init(&serializer);
        // process message.
        switch (deserializer.header.cmd)
        {
        case CMD_METHOD_HELLO:
            if (s_hello_ptr)
            {
                ret = ipc_des_get_string(&deserializer, s_hello_name, MAX_STRING_SIZE);
                if (ret >= 0)
                {
                    (*s_hello_ptr)(s_hello_name, s_hello_message);
                    ret = ipc_ser_put_string(&serializer, s_hello_message);
                }
            }
            else
            {
                ret = -1;
                ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
            }
            break;
        case CMD_METHOD_SUB_HEARTBEAT:
            ret = add_registration(&s_heartbeat_registry, (uint8_t)deserializer.header.pid,
                                   (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
            ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
            break;
        case CMD_METHOD_UNSUB_HEARTBEAT:
            ret = remove_registration(&s_heartbeat_registry, (uint8_t)deserializer.header.pid,
                                      (uint8_t)deserializer.header.fid, (uint8_t)deserializer.header.sid);
            ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
            break;
        default:
            ret = -1;
            ret = ipc_ser_put(&serializer, (uint8_t *)&ret, sizeof(ret));
            break;
        }
        serializer.header = deserializer.header;
        serializer.header.cid = deserializer.header.pid;
        serializer.header.pid = PID;
        serializer.header.typ = IPC_MSG_TYPE_REPLY;
        if (ret >= 0)
            ret = ipc_ser_finish(&serializer);

        if (ret >= 0)
        {
            for (uint32_t i = 0; i <= serializer.index; ++i)
            {
                ret = ipc_trans_layer_stub_send_reply_msg(s_handle, serializer.msg_pool[i]);
                if (ret < 0)
                {
                    printf("send reply fail %d.\n", ret);
                    break;
                }
            }
        }

        deserializer.rcv_index = 0;
    }
    return ret;
}

// init server
test_server *test_server_init()
{
    // start trans layer.
    int32_t ret = ipc_trans_layer_start(0);
    if (ret < 0)
        return NULL;
    // reset hello method pointer.
    s_hello_ptr = NULL;
    // create server handle.
    ret = ipc_trans_layer_stub_create_handle(FID, SID, &s_handle);
    if (ret < 0)
        return NULL;

    s_server.version = get_ipc_inf_version;
    s_server.register_hello = register_test_hello;
    s_server.heartbeat = test_heartbeat;
    s_server.receive_message = receive_message;
    s_server.dispatch_message = dispatch_message;
    return &s_server;
}

// destory client
int32_t test_server_destroy()
{
    int32_t ret = ipc_trans_layer_destory_handle(s_handle);
    if (ret < 0)
        return ret;
    ret = ipc_trans_layer_stop();
    if (ret < 0)
        return ret;

    s_server.version = NULL;
    s_server.register_hello = NULL;
    s_server.heartbeat = NULL;
    s_server.receive_message = NULL;
    s_server.dispatch_message = NULL;
    return ret;
}