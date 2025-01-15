// #include "sample-server.h"
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>
#include "test_server.h"

static uint8_t s_index = 0;
static test_server *s_server = NULL;
static void hello(const char *name, char **message, test_ErrorEnum_t *error)
{
    if (name)
    {
        char msg[IPC_MAX_DATA_SIZE];
        const char *src = name;
        char *ptr = msg;
        *ptr++ = 'h';
        *ptr++ = 'e';
        *ptr++ = 'l';
        *ptr++ = 'l';
        *ptr++ = 'o';
        *ptr++ = ' ';
        while (*name != '\0')
            *ptr++ = *name++;
        *ptr = '\0';
        *message = msg;
        printf("%s: %s, %s\n", __func__, src, msg);
        if (s_server && s_server->heartbeat)
            s_server->heartbeat(s_index++);

        *error = NO_ERROR;
    }
}

static void complex_method(const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct_t in5, const test_MyUnion_t in6,
        uint32_t *out1, char **out2, byte_buffer *out3, test_MyArray_t *out4, test_MyStruct_t *out5, test_MyUnion_t *out6, test_ErrorEnum_t *error)
{
    //Attention: all out buffer, e.g. String/ByteBuffer/Array, should be allocated first.
    //This sample only shows a shallow copy, with no buffer allocated.
    *out1 = in1;
    *out2 = (char *)in2;
    *out3 = in3;
    *out4 = in4;
    *out5 = in5;
    *out6 = in6;
    *error = NO_ERROR;
    printf("in1: %d, in2: %s, in3.size: %d, in4.size: %d, in5.m1: %d, in5.m2: %d, in5.m3.size: %d, in5.m4: %s, in5.m5.size: %d, in6.m3: %d\n",
        in1, in2, in3.size, in4.size, in5.m1, in5.m2, in5.m3.size, in5.m4, in5.m5.size, in6.m3);
    printf("in3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", in3.data[0], in3.data[1], in3.data[2], in3.data[3], in3.data[4], in3.data[5], in3.data[6], in3.data[7]);
    printf("in4 data: %d, %d, %d, %d\n", in4.data[0], in4.data[1], in4.data[2], in4.data[3]);
    printf("in5.m3 data: %d, %d, %d, %d, %d\n", in5.m3.data[0], in5.m3.data[1], in5.m3.data[2], in5.m3.data[3], in5.m3.data[4]);
    printf("in5.m5 data: %d, %d, %d, %d, %d, %d\n", in5.m5.data[0], in5.m5.data[1], in5.m5.data[2], in5.m5.data[3], in5.m5.data[4], in5.m5.data[5]);
}

int32_t test_ipc_main(void *arg)
{
    int32_t ret = 0;
    s_server = test_server_init();
    if (!s_server)
    {
        printf("init server fail.\n");
        return -1;
    }

    // get version
    ipc_inf_version version = s_server->version();
    printf("Interface version: major %d, minor %d!-----\n", version.major, version.minor);

    // register hello method
    s_server->register_hello(&hello);
    if (ret < 0)
    {
        printf("register hello error\n");
        return -2;
    }
    // register complex_method
    s_server->register_complex_method(&complex_method);
    if (ret < 0)
    {
        printf("register complex_method error\n");
        return -2;
    }
    printf("\n enter while\n");
    while (1)
    {
        ret = s_server->receive_message(); //收到中断消息
        //break;
        if (ret < 0)
        {
            continue;
        }
        ret = s_server->dispatch_message();
        if (ret < 0)
        {
            continue;
        }
        msleep(10);

    }

    ret = test_server_destroy();
    if (ret < 0)
    {
        printf("destory client fail.\n");
        return -3;
    }
    return 0;
}

struct task_struct *start_server_test(void)
{
    return  kthread_create(test_ipc_main, NULL, "ipc-msgbox-%u", smp_processor_id());
}