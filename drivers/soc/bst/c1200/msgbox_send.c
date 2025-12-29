#include "msgbox_send.h"



// local variables
static msgbox_send_t* s_data = NULL;
static _Atomic volatile uint8_t s_token = 0;
static msgbox_send_t s_internal_data;
#ifdef CONFIG_BST_C1200_ADAS
uint32_t PID = CPU_5;
#endif


#ifdef CONFIG_BST_C1200_IVI
uint32_t PID = CPU_1;
#endif


#ifdef CONFIG_BST_C1200_DB
uint32_t PID = CPUMP2_1;
#endif

// get interface version
static ipc_inf_version_t get_ipc_inf_version(void)
{
    return s_data->version;
}

// method

static inline int32_t serialize_scmi_msg(serdes_t* ser, const uint64_t addr)
{
    int32_t ret = 0;
    if (ret >= 0)
    {
        ret = ipc_ser_put_64(ser, &addr);
    }
    
    if (ret < 0)
        return -ERR_APP_SERDES;
    else
        return RESULT_SUCCESS;
}

static inline int32_t serialize_param(serdes_t* ser, const uint8_t param)
{
    int32_t ret = 0;
    if (ret >= 0)
    {
        ret = ipc_ser_put_8(ser, &param);
    }
    
    if (ret < 0)
        return -ERR_APP_SERDES;
    else
        return RESULT_SUCCESS;
}

static inline int32_t sendmsg(serdes_t* ser, int8_t cmd)
{
    int32_t ret = 0;
    if (!ser)
        return -ERR_APP_PARAM;

    ser->header.pid = PID; //
    ser->header.cid = CID;
    ser->header.fid = FID;
    ser->header.sid = SID;
    ser->header.cmd = cmd;
    ser->header.typ = MSGBX_MSG_TYPE_METHOD;
    ser->header.tok = 0;

    (void)ipc_ser_finish(ser);
    // send message
    ret = ipc_trans_layer_proxy_send_method(PID, s_data->handle, ser);
    return ret;
}

int32_t send_by_msgbox(const uint64_t addr)
{
    int32_t ret = 0;
    serdes_t serdes = {0};
    serdes_t* ser = &serdes;
    // serialize
    (void)ipc_ser_init(ser);
    (void)serialize_scmi_msg(ser, addr);
    // send request
    ret = sendmsg(ser, CMD_METHOD_SCMI);
    if (ret < 0)
    {
        //printf("send msg fail %d.\n", ret);
        return ret;
    }
    return RESULT_SUCCESS;
}

// initialize client
msgbox_send_client_t *msgbox_client_init(void)
{
    int32_t ret;
    s_data = &s_internal_data;
    
    ipc_memset(s_data, 0, sizeof(msgbox_send_client_t));

    PID = setup_max_cpus >= 2 ? PID : current->thread_info.cpu + msgbx_get_start_pid();

    // create client handle.
    ret = ipc_trans_layer_proxy_create_handle(PID, FID, SID, CID, 0, &s_data->handle);
    if (ret < 0)
        return NULL;

    // set version
    s_data->version.major = 1;
    s_data->version.minor = 0;

    // set client
    s_data->client.version = get_ipc_inf_version;
    
    mutex_init(&s_data->send_mtx);
    init_registry_list(s_data->method_registry, IPC_TOKEN_NUM);
    return &s_data->client;
}

// destroy client
int32_t msgbox_client_destroy(void)
{
    int32_t ret = ipc_trans_layer_destroy_handle(PID, s_data->handle);
    if (ret < 0)
        return ret;

    
    
    destroy_registry_list(s_data->method_registry, IPC_TOKEN_NUM);
    ipc_memset(s_data, 0, sizeof(msgbox_send_client_t));
    s_data = NULL;

    return ret;
}
