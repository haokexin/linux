#include "sample-client.h"
#include "ipc_app_serdes.h"
#include "ipc_trans_common.h"
#include "ipc_trans_layer.h"

// macro definitions
#define PID CPU_0
#define CID CPU_7
#define FID 0U
#define SID 1U
#define MAX_TOKEN_NUM 16U
#define MAX_PAYLOAD_NUM 32U
#define CMD_METHOD_HELLO 1U
#define CMD_METHOD_SUB_HEARTBEAT 10U
#define CMD_METHOD_UNSUB_HEARTBEAT 11U
#define CMD_BROADCAST_HEARTBEAT 1U

// typde definitions
typedef struct
{
    void *cb;
    void *ext;
} callback_registration_t;

// local variables
static ipc_inf_version s_version = {.major = 1, .minor = 0};
static uint32_t s_handle = 0U;
static test_client s_client = {0};
static uint8_t s_token = 0;
static callback_registration_t s_heartbeat_registry = {0};
static callback_registration_t s_method_registry[MAX_TOKEN_NUM] = {0};
static int8_t s_hello_message[MAX_STRING_SIZE] = {0};

// local methods
static inline void increase_token()
{
    if (++s_token >= MAX_TOKEN_NUM)
        s_token = 0;
}

static inline void add_registry(callback_registration_t *reg, void *cb, void *ext)
{
    reg->cb = cb;
    reg->ext = ext;
}

static ipc_inf_version get_ipc_inf_version()
{
    return s_version;
}

// call hello method
static int32_t call_test_hello_async(const char *name, test_hello_callback_t cb, void *ext)
{
    int32_t ret = 0;
    serdes_t serdes = {0};

    // prepare message
    serdes.header.pid = PID;
    serdes.header.cid = CID;
    serdes.header.fid = FID;
    serdes.header.sid = SID;
    serdes.header.tok = s_token;
    serdes.header.cmd = CMD_METHOD_HELLO;
    serdes.header.typ = IPC_MSG_TYPE_METHOD;

    (void)ipc_ser_init(&serdes);
    ret = ipc_ser_put_string(&serdes, name);
    if (ret < 0)
        printf("cannot serialize method!\n");
    (void)ipc_ser_finish(&serdes);

    // send message
    uint32_t i = 0;
    for (; i <= serdes.index; ++i)
    {
        ret = ipc_trans_layer_proxy_send_method(s_handle, serdes.msg_pool[i]);
        if (ret < 0)
            break;
    }
    if (i > serdes.index)
    {
        add_registry(&s_method_registry[s_token], (void *)cb, ext);
        increase_token();
        return 0;
    }
    else
    {
        printf("send method hello fail %d.\n", ret);
        return ret;
    }
}
// subscribe heartbeat broadcast
static int32_t subscribe_test_heartbeat(test_heartbeat_callback_t cb, void *ext, test_heartbeat_sub_callback_t cb2,
                                        void *ext2)
{
    int32_t ret = 0;
    rw_msg msg = {0};

    // prepare message
    msg.header.pid = PID;
    msg.header.cid = CID;
    msg.header.fid = FID;
    msg.header.sid = SID;
    msg.header.tok = s_token;
    msg.header.cmd = CMD_METHOD_SUB_HEARTBEAT;
    msg.header.typ = IPC_MSG_TYPE_METHOD;
    msg.header.len = 0;
    msg.header.is_eof = 1;

    // send message
    ret = ipc_trans_layer_proxy_send_method(s_handle, msg);
    if (ret < 0)
    {
        printf("send method hello fail %d.\n", ret);
        return ret;
    }
    else
    {
        add_registry(&s_heartbeat_registry, (void *)cb, ext);
        add_registry(&s_method_registry[s_token], cb2, ext2);
        increase_token();
        return 0;
    }
}
// unsubscribe heartbeat broadcast
static int32_t unscribe_test_heartbeat(test_heartbeat_unsub_callback_t cb, void *ext)
{
    int32_t ret = 0;
    rw_msg msg = {0};

    // prepare message
    msg.header.pid = PID;
    msg.header.cid = CID;
    msg.header.fid = FID;
    msg.header.sid = SID;
    msg.header.tok = s_token;
    msg.header.cmd = CMD_METHOD_UNSUB_HEARTBEAT;
    msg.header.typ = IPC_MSG_TYPE_METHOD;
    msg.header.len = 0;
    msg.header.is_eof = 1;

    // send message
    ret = ipc_trans_layer_proxy_send_method(s_handle, msg);
    if (ret < 0)
    {
        printf("send method hello fail %d.\n", ret);
        return ret;
    }
    else
    {
        add_registry(&s_method_registry[s_token], (void *)cb, ext);
        increase_token();
        return 0;
    }
}

// message router, call in main loop.
// receive messages
static int32_t receive_test_message()
{
    return ipc_trans_layer_get_msg(s_handle);
}
// dispatch message
static int32_t dispatch_test_message()
{
    int32_t ret = 0;

    static serdes_t broadcast_des = {0};
    static serdes_t reply_des = {0};

    while (ipc_trans_layer_proxy_get_broadcast_msg(s_handle, ipc_des_get_current_msg(&broadcast_des)) >= 0)
    {
        ret = ipc_des_validate_msg(&broadcast_des);
        if (ret < 0)
            continue;
        switch (broadcast_des.header.cmd)
        {
        case CMD_BROADCAST_HEARTBEAT: {
            uint8_t status;
            ret = ipc_des_get(&broadcast_des, &status, sizeof(status));
            test_heartbeat_callback_t cb = (test_heartbeat_callback_t)(s_heartbeat_registry.cb);
            if (cb)
                cb(status, s_heartbeat_registry.ext);
            break;
        }
        default:
            break;
        }
        broadcast_des.rcv_index = 0;
    }
    while (ipc_trans_layer_proxy_get_reply_msg(s_handle, ipc_des_get_current_msg(&reply_des)) >= 0)
    {
        ret = ipc_des_validate_msg(&reply_des);
        if (ret < 0)
            continue;
        switch (reply_des.header.cmd)
        {
        case CMD_METHOD_HELLO: {
            ret = ipc_des_get_string(&reply_des, s_hello_message, MAX_STRING_SIZE);
            if (ret >= 0)
            {
                callback_registration_t *reg = &s_method_registry[reply_des.header.tok];
                test_hello_callback_t cb = (test_hello_callback_t)(reg->cb);
                if (cb)
                    cb(s_hello_message, reg->ext);
            }
            break;
        }
        case CMD_METHOD_SUB_HEARTBEAT: {
            int32_t err = 0;
            ret = ipc_des_get(&reply_des, (uint8_t *)&err, sizeof(err));
            if (ret >= 0)
            {
                callback_registration_t *reg = &s_method_registry[reply_des.header.tok];
                test_heartbeat_sub_callback_t cb = (test_heartbeat_sub_callback_t)(reg->cb);
                if (cb)
                    cb(err, reg->ext);
            }
            break;
        }
        case CMD_METHOD_UNSUB_HEARTBEAT: {
            int32_t err = 0;
            ret = ipc_des_get(&reply_des, (uint8_t *)&err, sizeof(err));
            if (ret >= 0)
            {
                callback_registration_t *reg = &s_method_registry[reply_des.header.tok];
                test_heartbeat_unsub_callback_t cb = (test_heartbeat_unsub_callback_t)(reg->cb);
                if (cb)
                    cb(err, reg->ext);
            }
            break;
        }
        default:
            break;
        }
        reply_des.rcv_index = 0;
    }
    return ret;
}

// init client
test_client *test_client_init()
{
    int32_t ret = ipc_trans_layer_start(1);
    if (ret < 0)
        return NULL;
    ret = ipc_trans_layer_proxy_create_handle(FID, SID, &s_handle);
    if (ret < 0)
        return NULL;

    s_client.version = get_ipc_inf_version;
    s_client.hello = call_test_hello_async;
    s_client.heartbeat_sub = subscribe_test_heartbeat;
    s_client.heartbeat_unsub = unscribe_test_heartbeat;
    s_client.receive_message = receive_test_message;
    s_client.dispatch_message = dispatch_test_message;
    return &s_client;
}
// destory client
int32_t test_client_destroy()
{
    int32_t ret = ipc_trans_layer_destory_handle(s_handle);
    if (ret < 0)
        return ret;
    ret = ipc_trans_layer_stop();
    if (ret < 0)
        return ret;

    s_client.version = NULL;
    s_client.hello = NULL;
    s_client.heartbeat_sub = NULL;
    s_client.heartbeat_unsub = NULL;
    s_client.receive_message = NULL;
    s_client.dispatch_message = NULL;
    return ret;
}