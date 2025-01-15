/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

#include "sample_server.h"
#include <linux/delay.h>

#define USE_SEPARATE_REPLY

static uint8_t s_index         = 0;
static sample_server_t *s_server = NULL;
#ifdef USE_SEPARATE_REPLY
static void hello(const char *name, const rw_msg_header_t context, const ext_info_t *info)
#else
static void hello(const char *name, char **message, test_ErrorEnum_t *error, const ext_info_t *info)
#endif
{
    if (name) {
        char msg[IPC_MAX_DATA_SIZE];
        const char *src = name;
        char *ptr       = msg;
        *ptr++          = 'h';
        *ptr++          = 'e';
        *ptr++          = 'l';
        *ptr++          = 'l';
        *ptr++          = 'o';
        *ptr++          = ' ';
        while (*name != '\0')
            *ptr++ = *name++;
        *ptr = '\0';
        // printk("%s: %s, %s\n", __func__, src, msg);
        if (s_server && s_server->test_server.heartbeat)
            s_server->test_server.heartbeat(s_index++);

#ifdef USE_SEPARATE_REPLY
        s_server->test_server.reply_hello(msg, TEST_NO_ERROR, context);
#else
        *message = msg;
        *error   = TEST_NO_ERROR;
#endif
    }
}

#ifdef USE_SEPARATE_REPLY
static void complex_method(const uint32_t in1, const char *in2, const byte_buffer_t in3, const test_MyArray_t in4,
                           const test_MyStruct2_t in5, const test_MyUnion_t in6, rw_msg_header_t context,
                           const ext_info_t *info)
#else
static void complex_method(const uint32_t in1, const char *in2, const byte_buffer_t in3, const test_MyArray_t in4,
                           const test_MyStruct_t in5, const test_MyUnion_t in6, uint32_t *out1, char **out2,
                           byte_buffer_t *out3, test_MyArray_t *out4, test_MyStruct_t *out5, test_MyUnion_t *out6,
                           test_ErrorEnum_t *error, const ext_info_t *info)
#endif
{
    // printk("%s\n", __func__);
    // printk("client uuid %d\n", info->uuid);
    // printk("in1: %d, in2: %s, in3.size: %d, in4.size: %d, in5.m1: %d, in5.m2: %d, in5.m3.size: %d, in5.m4: %s,
    // in5.m5.size: %d, in6.m3: %d\n",
    //     in1, in2, in3.size, in4.size, in5.m1, in5.m2, in5.m3.size, in5.m4, in5.m5.size, in6.m3);
    // printk("in3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", in3.data[0], in3.data[1], in3.data[2], in3.data[3],
    // in3.data[4], in3.data[5], in3.data[6], in3.data[7]); printk("in4 data: %d, %d, %d, %d\n", in4.data[0],
    // in4.data[1], in4.data[2], in4.data[3]); printk("in5.m3 data: %d, %d, %d, %d, %d\n", in5.m3.data[0],
    // in5.m3.data[1], in5.m3.data[2], in5.m3.data[3], in5.m3.data[4]); printk("in5.m5 data: %d, %d, %d, %d, %d, %d\n",
    // in5.m5.data[0], in5.m5.data[1], in5.m5.data[2], in5.m5.data[3], in5.m5.data[4], in5.m5.data[5]);
#ifdef USE_SEPARATE_REPLY
    s_server->test_server.reply_complex_method(in1, in2, in3, in4, in5, in6, TEST_NO_ERROR, context);
#else
    // Attention: all out buffer, e.g. String/ByteBuffer/Array, should be allocated first.
    // This sample only shows a shallow copy, with no buffer allocated.
    *out1  = in1;
    *out2  = (char *)in2;
    *out3  = in3;
    *out4  = in4;
    *out5  = in5;
    *out6  = in6;
    *error = TEST_NO_ERROR;
#endif
}

static void no_reply_method(const uint8_t status, const ext_info_t* info)
{
    printk("%s: status = %d, uuid = %d, timestamp = %ld\n", __func__, status, info->uuid, info->timestamp);
}

static int32_t on_heartbeat_sub(uint8_t pid, uint8_t fid, uint8_t sid)
{
    printk("Receive heartbeat subscription: PID %d, FID %d, SID %d\n", pid, fid, sid);
    return 0;
}

static int32_t on_heartbeat_unsub(uint8_t pid, uint8_t fid, uint8_t sid)
{
    printk("Receive heartbeat un-subscription: PID %d, FID %d, SID %d\n", pid, fid, sid);
    return 0;
}

int start_server(void *arg)
{
    int32_t ret = 0;
    sample_server_data_t data = {0};
    s_server = sample_server_init(&data);
    if (!s_server) {
        printk("initialize server fail.\n");
        return -1;
    }

    // get version
    ipc_inf_version_t version = s_server->test_server.version();
    printk("Interface version: major %d, minor %d.\n", version.major, version.minor);

    // register hello method
    ret = s_server->test_server.register_hello(&hello);
    if (ret < 0) {
        printk("register hello error\n");
        return -2;
    }
    // register complex_method
    ret = s_server->test_server.register_complex_method(&complex_method);
    if (ret < 0) {
        printk("register complex_method error\n");
        return -2;
    }
    // register no_reply_method
    ret = s_server->test_server.register_no_reply_method(&no_reply_method);
    if (ret < 0) {
        printk("register no_reply_method error\n");
        return -2;
    }
    // register heartbeat sub and unsub callback
    (void)s_server->test_server.register_heartbeat_subcribed(on_heartbeat_sub);
    (void)s_server->test_server.register_heartbeat_unsubcribed(on_heartbeat_unsub);

    ret = s_server->start();
    if (ret != 0) {
        printk("start server failed.\n");
        return -3;
    }
    while (1)
        msleep(1000);

    ret = s_server->stop();
    if (ret != 0) {
        printk("stop server failed.\n");
        return -4;
    }

    ret = sample_server_destroy();
    if (ret < 0) {
        printk("destroy client fail.\n");
        return -5;
    }
    return 0;
}