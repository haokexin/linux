#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "test_client.h"

static uint32_t s_index = 0;
static bool s_exit = false;

static void on_hello_reply(const char *message, const test_ErrorEnum_t err, void *ext)
{
    // if (message)
        // printf("hello_async reply : %s.\n", message);
}

static inline void call_hello_async(test_client* client)
{
    if (!client)
        return;
    int32_t ret = client->hello_async("Client", on_hello_reply, NULL);
    if (ret < 0)
        printf("send method hello failed. ret is %d\n", ret);
}

static void on_complex_method_reply(const uint32_t out1, const char * out2, const byte_buffer out3, const test_MyArray_t out4, const test_MyStruct_t out5, const test_MyUnion_t out6, const test_ErrorEnum_t err, void *ext)
{
     printf("complex_method_async reply.\n");
    // printf("out1: %d, out2: %s, out3.size: %d, out4.size: %d, out5.m1: %d, out5.m2: %d, out5.m3.size: %d, out5.m4: %s, out5.m5.size: %d, out6.m3: %d, err: %d\n",
    //     out1, out2, out3.size, out4.size, out5.m1, out5.m2, out5.m3.size, out5.m4, out5.m5.size, out6.m3, err);
    // printf("out3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", out3.data[0], out3.data[1], out3.data[2], out3.data[3], out3.data[4], out3.data[5], out3.data[6], out3.data[7]);
    // printf("out4 data: %d, %d, %d, %d\n", out4.data[0], out4.data[1], out4.data[2], out4.data[3]);
    // printf("out5.m3 data: %d, %d, %d, %d, %d\n", out5.m3.data[0], out5.m3.data[1], out5.m3.data[2], out5.m3.data[3], out5.m3.data[4]);
    // printf("out5.m5 data: %d, %d, %d, %d, %d, %d\n", out5.m5.data[0], out5.m5.data[1], out5.m5.data[2], out5.m5.data[3], out5.m5.data[4], out5.m5.data[5]);
}

static inline void call_complex_method_async(test_client* client)
{
    if (!client)
        return;

    uint32_t in1 = 100;
    char* in2 = "input2";
    byte_buffer in3 = {0};
    uint8_t in3_data[8] = {1,2,3,4,5,6,7,8};
    in3.data = in3_data;
    in3.size = 8;
    test_MyArray_t in4 = {0};
    uint16_t in4_data[4] = {11,12,13,14};
    in4.data = in4_data;
    in4.size = 4;
    test_MyStruct_t in5 = {0};
    in5.m1 = 128;
    in5.m2 = true;
    uint16_t m3_data[5] = {21,22,23,24,25};
    in5.m3.data = m3_data;
    in5.m3.size = 5;
    in5.m4 = "input5";
    uint8_t m5_data[6] = {31,32,33,34,35,36};
    in5.m5.data = m5_data;
    in5.m5.size = 6;
    test_MyUnion_t in6 = {0};
    in6.m3 = 12345;
    int32_t ret = client->complex_method_async(in1, in2, in3, in4, in5, in6, on_complex_method_reply, NULL);
    if (ret < 0)
        printf("send method complex_method failed. ret is %d\n", ret);
    
}

static inline void call_complex_method_sync(test_client* client)
{
    if (!client)
        return;

    uint32_t in1 = 100;
    char* in2 = "input2";
    byte_buffer in3 = {0};
    uint8_t in3_data[8] = {1,2,3,4,5,6,7,8};
    in3.data = in3_data;
    in3.size = 8;
    test_MyArray_t in4 = {0};
    uint16_t in4_data[4] = {11,12,13,14};
    in4.data = in4_data;
    in4.size = 4;
    test_MyStruct_t in5 = {0};
    in5.m1 = 128;
    in5.m2 = true;
    uint16_t m3_data[5] = {21,22,23,24,25};
    in5.m3.data = m3_data;
    in5.m3.size = 5;
    in5.m4 = "input5";
    uint8_t m5_data[6] = {31,32,33,34,35,36};
    in5.m5.data = m5_data;
    in5.m5.size = 6;
    test_MyUnion_t in6 = {0};
    in6.m3 = 12345;

    uint32_t out1 = 0;
    char* out2 = NULL;
    byte_buffer out3 = {0};
    test_MyArray_t out4 = {0};
    test_MyStruct2_t out5 = {0};
    test_MyUnion_t out6 = {0};
    test_ErrorEnum_t err = 0;
    int32_t ret = client->complex_method_sync(in1, in2, in3, in4, in5, in6, &out1, &out2, &out3, &out4, &out5, &out6, &err, 1000);
    if (ret < 0)
        printf("send method complex_method_sync failed. ret is %d\n", ret);


     printf("complex_method_sync reply.\n");
     printf("out1: %d, out2: %s, out3.size: %d, out4.size: %d, out5.m1: %d, out5.m2: %d, out5.m3.size: %d, out5.m4: %s, out5.m5.size: %d, out6.m3: %d, err: %d\n",
         out1, out2, out3.size, out4.size, out5.m1, out5.m2, out5.m3.size, out5.m4, out5.m5.size, out6.m3, err);
#if 0
     printf("out3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", out3.data[0], out3.data[1], out3.data[2], out3.data[3], out3.data[4], out3.data[5], out3.data[6], out3.data[7]);
     printf("out4 data: %d, %d, %d, %d\n", out4.data[0], out4.data[1], out4.data[2], out4.data[3]);
     printf("out5.m3 data: %d, %d, %d, %d, %d\n", out5.m3.data[0], out5.m3.data[1], out5.m3.data[2], out5.m3.data[3], out5.m3.data[4]);
     printf("out5.m5 data: %d, %d, %d, %d, %d, %d\n", out5.m5.data[0], out5.m5.data[1], out5.m5.data[2], out5.m5.data[3], out5.m5.data[4], out5.m5.data[5]);
#endif
    kfree(out3.data);
    kfree(out4.data);
    kfree(out5.m3.data);
    kfree(out5.m4);
    kfree(out5.m5.data);
}

static void on_heartbeat_triggered(uint8_t status, void *ext)
{
    // printf("Receive heartbeat broadcast. status is %d.\n", status);
    call_complex_method_async((test_client*)ext);
}

static void on_heartbeat_sub_reply(int32_t err, void *ext)
{
    if (err == 0)
        printf("Subscribe heartbeat success.\n");
    else
        printf("Subscribe heartbeat fail. ret is %d.\n", err);
}

static void on_heartbeat_unsub_reply(int32_t err, void *ext)
{
    if (err == 0)
        printf("Unsubscribe heartbeat success.\n");
    else
        printf("Unsubscribe heartbeat fail. ret is %d.\n", err);

    s_exit = true;
}

static int test_client_main_loop(void *arg)
{
    test_client *client = test_client_init((rw_msg_header_t *)arg);
    if (!client)
    {
        printf("init client fail.\n");
        return -1;
    }

    // get version
    ipc_inf_version_t version = client->version();
    printf("Interface version: major %d, minor %d.\n", version.major, version.minor);

    // start test_client
    int32_t ret = client->start();
    // int32_t ret = test_client_start(client);
    if (ret < 0)
    {
        printf("start test client failed!\n");
        return -2;
    }

    // subscribe broadcast.
    ret = client->heartbeat_sub(on_heartbeat_triggered, (void *)client, on_heartbeat_sub_reply, NULL);
    if (ret < 0)
    {
        printf("send subscribe message fail. ret = %d\n", ret);
        return -3;
    }

    // call method.
    call_hello_async(client);
    call_complex_method_async(client);

    char name[20] = {0};
    char *message = NULL;
    test_ErrorEnum_t err = 0;
    uint32_t total = 10u;
    while (s_index < total)
    {
        sprintf(name, "Client %d", s_index);
        client->hello_sync(name, &message, &err, 1000);
        // printf("hello_sync reply : %s.\n", message);

        call_hello_async(client);

        call_complex_method_sync(client);
        call_complex_method_async(client);

        ++s_index;
        msleep(1000);
        printf("\n total:%u s_index:%u\n", total, s_index);
    }

    // unsubscribe broadcast.
    printf("Unsubscribe heartbeat!\n");
    ret = client->heartbeat_unsub(on_heartbeat_unsub_reply, NULL);
    if (ret < 0)
    {
        printf("send unsubscribe message fail.\n");
        return -4;
    }

    //stop test client
    ret = client->stop();
    if (ret < 0)
    {
        printf("stop client thread failed!\n");
        return -5;
    }

    ret = test_client_destroy();
    if (ret < 0)
    {
        printf("destroy client fail.\n");
        return -6;
    }

    return 0;
}

struct task_struct *start_client_test(void *data)
{
    return  kthread_create(test_client_main_loop, data, "ipc-client-msgbox-%u", smp_processor_id());
}
EXPORT_SYMBOL(start_client_test);
