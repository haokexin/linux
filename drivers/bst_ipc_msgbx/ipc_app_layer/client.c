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
#include "sample_client.h"
#include <linux/delay.h>
sample_client_t* s_client = NULL;

static void on_hello_reply(const char *message, const test_ErrorEnum_t err,
			   void *ext, const ext_info_t *info)
{
	if (message)
	    printk("hello_async reply : %s.\n", message);
}

static inline void call_hello_async(test_client_t *client, uint32_t i)
{
	if (!client)
		return;
	char name[24] = {0};
	sprintf(name, "Client async %d", i);
	int32_t ret = client->hello_async(name, on_hello_reply, NULL, NULL);
	if (ret < 0)
		printk("Client: send method hello failed. ret is %d\n", ret);
}

static void on_complex_method_reply(const uint32_t out1, const char *out2,
				    const byte_buffer_t out3,
				    const test_MyArray_t out4,
				    const test_MyStruct_t out5,
				    const test_MyUnion_t out6,
				    const test_ErrorEnum_t err, void *ext,
				    const ext_info_t *info)
{
	if (err != TEST_NO_ERROR) {
		printk("%s: err code %d\n", __func__, err);
		return;
	}
	printk("complex_method_async reply.\n");
	// printk("out1: %d, out2: %s, out3.size: %d, out4.size: %d, out5.m1: %d, out5.m2: %d, out5.m3.size: %d, out5.m4:
	// %s, out5.m5.size: %d, out6.m3: %d, err: %d\n",
	//     out1, out2, out3.size, out4.size, out5.m1, out5.m2, out5.m3.size, out5.m4, out5.m5.size, out6.m3, err);
	// printk("out3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", out3.data[0], out3.data[1], out3.data[2], out3.data[3],
	// out3.data[4], out3.data[5], out3.data[6], out3.data[7]); printk("out4 data: %d, %d, %d, %d\n", out4.data[0],
	// out4.data[1], out4.data[2], out4.data[3]); printk("out5.m3 data: %d, %d, %d, %d, %d\n", out5.m3.data[0],
	// out5.m3.data[1], out5.m3.data[2], out5.m3.data[3], out5.m3.data[4]); printk("out5.m5 data: %d, %d, %d, %d, %d,
	// %d\n", out5.m5.data[0], out5.m5.data[1], out5.m5.data[2], out5.m5.data[3], out5.m5.data[4], out5.m5.data[5]);
}

static inline void call_complex_method_async(test_client_t *client)
{
	if (!client)
		return;

	uint32_t in1 = 100;
	char *in2 = "input2";
	byte_buffer_t in3 = { 0 };
	uint8_t in3_data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	in3.data = in3_data;
	in3.size = 8;
	test_MyArray_t in4 = { 0 };
	uint16_t in4_data[4] = { 11, 12, 13, 14 };
	in4.data = in4_data;
	in4.size = 4;
	test_MyStruct_t in5 = { 0 };
	in5.m1 = 128;
	in5.m2 = true;
	uint16_t m3_data[5] = { 21, 22, 23, 24, 25 };
	in5.m3.data = m3_data;
	in5.m3.size = 5;
	in5.m4 = "input5";
	uint8_t m5_data[6] = { 31, 32, 33, 34, 35, 36 };
	in5.m5.data = m5_data;
	in5.m5.size = 6;
	test_MyUnion_t in6 = { 0 };
	in6.m3 = 12345;
	int32_t ret = client->complex_method_async(in1, in2, in3, in4, in5, in6,
						   on_complex_method_reply,
						   NULL, NULL);
	if (ret < 0)
		printk("Client: send method complex_method failed. ret is %d\n", ret);
}

static inline void call_complex_method_sync(test_client_t *client, des_buf_t *buffer)
{
	if (!client)
		return;

	uint32_t in1 = 100;
	char *in2 = "input2";
	byte_buffer_t in3 = { 0 };
	uint8_t in3_data[8] = { 9, 8, 7, 6, 5, 4, 3, 2 };
	in3.data = in3_data;
	in3.size = 8;
	test_MyArray_t in4 = { 0 };
	uint16_t in4_data[4] = { 19, 18, 17, 16 };
	in4.data = in4_data;
	in4.size = 4;
	test_MyStruct_t in5 = { 0 };
	in5.m1 = 128;
	in5.m2 = true;
	uint16_t m3_data[5] = { 29, 28, 27, 26, 25 };
	in5.m3.data = m3_data;
	in5.m3.size = 5;
	in5.m4 = "input5";
	uint8_t m5_data[6] = { 39, 38, 37, 36, 35, 34 };
	in5.m5.data = m5_data;
	in5.m5.size = 6;
	test_MyUnion_t in6 = { 0 };
	in6.m3 = 54321;

	uint32_t out1 = 0;
	char *out2 = NULL;
	byte_buffer_t out3 = { 0 };
	test_MyArray_t out4 = { 0 };
	test_MyStruct2_t out5 = { 0 };
	test_MyUnion_t out6 = { 0 };
	test_ErrorEnum_t err = 0;
	int32_t ret =
		client->complex_method_sync(in1, in2, in3, in4, in5, in6, &out1,
					    &out2, &out3, &out4, &out5, &out6,
					    &err, 1000, buffer);
	if (ret < 0)
		printk("Client: send method complex_method_sync failed. ret is %d\n", ret);

	printk("complex_method_sync reply.\n");
	// printk("out1: %d, out2: %s, out3.size: %d, out4.size: %d, out5.m1: %d, out5.m2: %d, out5.m3.size: %d, out5.m4:
	// %s, out5.m5.size: %d, out6.m3: %d, err: %d\n",
	//     out1, out2, out3.size, out4.size, out5.m1, out5.m2, out5.m3.size, out5.m4, out5.m5.size, out6.m3, err);
	// printk("out3 data: %d, %d, %d, %d, %d, %d, %d, %d\n", out3.data[0], out3.data[1], out3.data[2], out3.data[3],
	// out3.data[4], out3.data[5], out3.data[6], out3.data[7]); printk("out4 data: %d, %d, %d, %d\n", out4.data[0],
	// out4.data[1], out4.data[2], out4.data[3]); printk("out5.m3 data: %d, %d, %d, %d, %d\n", out5.m3.data[0],
	// out5.m3.data[1], out5.m3.data[2], out5.m3.data[3], out5.m3.data[4]); printk("out5.m5 data: %d, %d, %d, %d, %d,
	// %d\n", out5.m5.data[0], out5.m5.data[1], out5.m5.data[2], out5.m5.data[3], out5.m5.data[4], out5.m5.data[5]);
}

static void on_heartbeat_triggered(uint8_t status, void *ext,
				   const ext_info_t *info)
{
	printk("Receive heartbeat broadcast. status is %d.\n", status);
	// call_complex_method_async((test_client_t *)ext);
}

static void on_heartbeat_sub_reply(int32_t err, void *ext,
				   const ext_info_t *info)
{
	if (err == 0)
		printk("Subscribe heartbeat success.\n");
	else
		printk("Subscribe heartbeat fail. ret is %d.\n", err);
}

static void on_heartbeat_unsub_reply(int32_t err, void *ext,
				     const ext_info_t *info)
{
	if (err == 0)
		printk("Unsubscribe heartbeat success.\n");
	else
		printk("Unsubscribe heartbeat fail. ret is %d.\n", err);
}

static void on_dst_changed(bool flag, void *ext)
{
	if (flag)
		printk("dst is online\n");
	else
		printk("dst is offline\n");

	*((bool *)ext) = flag;
}

static int posix_client(int32_t cycle_num)
{
	int32_t ret = 0;

	sample_client_data_t data = {0};
	sample_client_t *client = sample_client_init(&data);
	if (!client) {
		printk("Client: init client fail.\n");
		return -1;
	}
	s_client = client;

	// get version
	ipc_inf_version_t version = client->test_client.version();
	printk("Client: Interface version: major %d, minor %d.\n", version.major,
	       version.minor);

	// start test_client_t
	ret = client->start();
	if (ret < 0) {
		printk("Client: start test client failed!\n");
		return -2;
	}

	// bool dst_avail = false;
	// ret = client->test_client.register_avail_changed(on_dst_changed, &dst_avail);
	// while (!dst_avail)
	// 	sleep(1);

	// subscribe broadcast.
	ret = client->test_client.heartbeat_sub(on_heartbeat_triggered, (void *)client,
				    NULL, on_heartbeat_sub_reply, NULL);
	if (ret < 0) {
		printk("Client: send subscribe message fail. ret = %d\n", ret);
		return -3;
	}

	// call method.
	call_hello_async(&client->test_client, 0);
	call_complex_method_async(&client->test_client);

	char name[24] = { 0 };
	char *message = NULL;
	test_ErrorEnum_t err = 0;
	uint32_t total = cycle_num;
	des_buf_t buffer = { 0 };
	uint32_t i = 0;
	while (i < total) {
		sprintf(name, "Client sync %d", i);
		client->test_client.hello_sync(name, &message, &err, 1000, &buffer);
		printk("hello_sync reply : %s.\n", message);

		call_hello_async(&client->test_client, i);

		call_complex_method_sync(&client->test_client, &buffer);
		call_complex_method_async(&client->test_client);

		++i;
		msleep(1);
	}

	// unsubscribe broadcast.
	printk("Client: Unsubscribe heartbeat!\n");
	ret = client->test_client.heartbeat_unsub(on_heartbeat_unsub_reply, NULL);
	if (ret < 0) {
		printk("Client: send unsubscribe message fail.\n");
		return -4;
	}

	// stop test client
	ret = client->stop();
	if (ret < 0) {
		printk("Client: stop client thread failed!\n");
		return -5;
	}
	printk("Client: stopped.\n");
	ret = sample_client_destroy();
	if (ret < 0) {
		printk("destroy client fail.\n");
		return -6;
	}
	printk("Client: destroyed.\n");

	return 0;
}

int start_run_client(int cycle_num)
{
	int ret = 0;
 	cycle_num = 10;

	//start client sessions.
	posix_client(cycle_num);

	return 0;
}