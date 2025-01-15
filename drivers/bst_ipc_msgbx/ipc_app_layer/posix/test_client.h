
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */


#ifndef TEST_CLIENT_H
#define TEST_CLIENT_H

#include <bst/ipc_app_common.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // user defined types
    
	typedef struct
	{
	    uint16_t *data;
	    uint32_t size;
	}test_MyArray_t;
	
	typedef enum
	{
	    NO_ERROR = 0,
		SERVER_FAIL = -1,
		ERROR_2 = -2,
		ERROR_3 = -3
	}test_ErrorEnum_t;
	
	typedef struct
	{
	    uint8_t m1;
		bool m2;
		test_MyArray_t m3;
		char * m4;
		byte_buffer m5;
	}test_MyStruct_t;
	
	typedef union
	{
	    uint8_t m1;
		uint16_t m2;
		uint32_t m3;
	}test_MyUnion_t;
	
	typedef test_MyStruct_t test_MyStruct2_t;

    // constants
    static const bool b1 = true;
	static const uint32_t MAX_COUNT = 10000;
	static const uint16_t SOME_ID = 40971;
	static const uint8_t BYTE_ME = 51;
	static const char * foo = "bar";
	static const double pi = 3.141500;
	static const uint32_t twentyfive = 5*5;
	static const bool b2 = MAX_COUNT>3;
	static const bool b3 = b1&&b2||foo=="bar";
	
    // method types
    typedef void (*test_hello_callback_t)(const char * message, const test_ErrorEnum_t err, void *ext);
	typedef void (*test_complex_method_callback_t)(const uint32_t out1, const char * out2, const byte_buffer out3, const test_MyArray_t out4, const test_MyStruct2_t out5, const test_MyUnion_t out6, const test_ErrorEnum_t err, void *ext);
	
    // Broadcast types
    typedef void (*test_heartbeat_callback_t)(const uint8_t status, void *ext);
	typedef void (*test_heartbeat_sub_callback_t)(int32_t err, void *ext);
	typedef void (*test_heartbeat_unsub_callback_t)(int32_t err, void *ext);
	
    // Interface server
    typedef struct
    {
        // get version
        ipc_inf_version_t (*version)(void);

        // methods
        int32_t (*hello_sync)(const char * name, char ** message, test_ErrorEnum_t* err, int64_t timeout_ms);
		int32_t (*hello_async)(const char * name, test_hello_callback_t cb, void *ext);
		int32_t (*complex_method_sync)(const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct2_t in5, const test_MyUnion_t in6, uint32_t* out1, char ** out2, byte_buffer* out3, test_MyArray_t* out4, test_MyStruct2_t* out5, test_MyUnion_t* out6, test_ErrorEnum_t* err, int64_t timeout_ms);
		int32_t (*complex_method_async)(const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct2_t in5, const test_MyUnion_t in6, test_complex_method_callback_t cb, void *ext);
		
        // broadcasts
        int32_t (*heartbeat_sub)(test_heartbeat_callback_t cb, void *ext, test_heartbeat_sub_callback_t cb2, void *ext2);
		int32_t (*heartbeat_unsub)(test_heartbeat_unsub_callback_t cb, void *ext);
		
        // start message router.
        int32_t (*start)(void);
        // stop message router.
        int32_t (*stop)(void);
    }test_client;

    // init server
    test_client *test_client_init(rw_msg_header_t *);
    // destroy client
    int32_t test_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_CLIENT_H
