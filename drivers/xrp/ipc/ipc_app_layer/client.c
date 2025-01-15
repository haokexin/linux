// #include "sample-client.h"
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "test_client.h"

static uint32_t s_index = 0;

static void on_hello_reply(const char *message, const test_ErrorEnum_t err, void *ext)
{
    if (message)
        printf("Receive hello reply : %s.\n", message);

    ++s_index;
}

static inline void call_hello(test_client* client)
{
    if (!client)
        return;
    int32_t ret = client->hello("Client", on_hello_reply, NULL);
    if (ret < 0)
        printf("send method hello failed. ret is %u\n", ret);
}

static void on_complex_method_reply(const uint8_t response, const test_Array_Uint8_t resp_data, const test_ErrorEnum_t err, void *ext)
{
    printf("Receive complex_method reply.\n");
#if 0
    printf("out1: %d, out2: %s, out3.size: %d, out4.size: %d, out5.m1: %d, out5.m2: %d, out5.m3.size: %d, out5.m4: %s, out5.m5.size: %d, out6.m3: %d, err: %d\n",
        out1, out2, out3.size, out4.size, out5.m1, out5.m2, out5.m3.size, out5.m4, out5.m5.size, out6.m3, err);
    printf("out3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", out3.data[0], out3.data[1], out3.data[2], out3.data[3], out3.data[4], out3.data[5], out3.data[6], out3.data[7]);
    printf("out4 data: %d, %d, %d, %d\n", out4.data[0], out4.data[1], out4.data[2], out4.data[3]);
    printf("out5.m3 data: %d, %d, %d, %d, %d\n", out5.m3.data[0], out5.m3.data[1], out5.m3.data[2], out5.m3.data[3], out5.m3.data[4]);
    printf("out5.m5 data: %d, %d, %d, %d, %d, %d\n", out5.m5.data[0], out5.m5.data[1], out5.m5.data[2], out5.m5.data[3], out5.m5.data[4], out5.m5.data[5]);
#endif
    ++s_index;
}

static inline void call_complex_method(test_client* client)
{
    if (!client)
        return;

    uint8_t opcode = 100;
    
    test_Array_Uint8_t user_data = {0};
    uint8_t in3_data[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    user_data.data = in3_data;
    user_data.size = 15;
    test_Array_Uint8_t i_name_space_id = {0};
    uint8_t in4_data[16] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,0x00};
    i_name_space_id.data = in4_data;
    i_name_space_id.size = 16;

    int32_t ret = client->hifi_a78_msg_sync(  opcode,   user_data,   i_name_space_id, on_complex_method_reply, NULL);
    if (ret < 0)
        printf("send method complex_method failed. ret is %u\n", ret);
}

static void on_heartbeat_triggered(uint8_t status, void *ext)
{
    printf("Receive heartbeat broadcast. status is %d.\n", status);
    call_hello((test_client *)ext);
    call_complex_method((test_client *)ext);
}

static void on_heartbeat_sub_reply(uint32_t err, void *ext)
{
    if (err == 0)
        printf("Subscribe heartbeat success.\n");
    else
        printf("Subscribe heartbeat fail. ret is %u.\n", err);
}

static void on_heartbeat_unsub_reply(uint32_t err, void *ext)
{
    if (err == 0)
        printf("Unsubscribe heartbeat success.\n");
    else
        printf("Unsubscribe heartbeat fail. ret is %u.\n", err);
}

int test_client_main_loop(void *arg)
{
    test_client *client = test_client_init();
    if (!client)
    {
        printf("init client fail.\n");
        return -1;
    }

    // get version
    ipc_inf_version version = client->version();
    printf("Interface version: major %d, minor %d.\n", version.major, version.minor);

    // subscribe broadcast.
    int32_t ret = client->heartbeat_sub(on_heartbeat_triggered, (void *)client, on_heartbeat_sub_reply, NULL);
    if (ret < 0)
    {
        printf("send subscribe message fail.\n");
        return -2;
    }

    // call method.
    call_hello(client);
    call_complex_method(client);

    uint32_t total = 4u;

    printf("\n [%s] enter main loop\n", __FUNCTION__);
    while (1)
    {
        ret = client->receive_message();
        if (ret < 0)
            continue;
        ret = client->dispatch_message();
        if (ret < 0)
            continue;

        if (s_index > total)
            break;

        msleep(10);
    }

    ret = test_client_destroy();
    if (ret < 0)
    {
        printf("destory client fail.\n");
        return -3;
    }

    return 0;
}

struct task_struct *start_client_test(void)
{
    return  kthread_create(test_client_main_loop, NULL, "ipc-client-msgbox-%u", smp_processor_id());
}