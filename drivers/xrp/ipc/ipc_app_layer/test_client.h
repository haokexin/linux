
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

#include "ipc_app_common.h"

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
	
	typedef struct
	{
	    uint8_t *data;
	    uint32_t size;
	}test_Array_Uint8_t;
	
	typedef enum
	{
	    NO_ERROR = 0,
		SERVER_FAIL = -1,
		ERROR_2 = -2,
		ERROR_3 = -3
	}test_ErrorEnum_t;
	
	typedef struct
	{
	    uint8_t opcode;
		test_Array_Uint8_t user_data;
		test_Array_Uint8_t i_name_space_id;
	}test_hifi_a78_msg_t;
	
	typedef struct
	{
	    uint8_t opcode;
		uint32_t data_paddr;
		uint8_t status;
	}test_hifi_a78_result_t;
	
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
	typedef test_hifi_a78_msg_t test_xrp_msg_t;
	typedef test_hifi_a78_result_t test_xrp_result_t;

    // constants
	static const uint32_t MAX_COUNT = 10000;
	static const uint16_t SOME_ID = 40971;
	static const uint8_t BYTE_ME = 51;
	static const double pi = 3.141500;
	static const uint32_t twentyfive = 5*5;

	
    // method types
    typedef void (*test_hello_callback_t)(const char * message, const test_ErrorEnum_t err, void *ext);
	typedef void (*test_hifi_a78_msg_sync_callback_t)(const uint8_t response, const test_Array_Uint8_t resp_data, const test_ErrorEnum_t err, void *ext);
	
    // Broadcast types
    typedef void (*test_heartbeat_callback_t)(const test_xrp_result_t fs_result, void *ext);
	typedef void (*test_heartbeat_sub_callback_t)(uint32_t err, void *ext);
	typedef void (*test_heartbeat_unsub_callback_t)(uint32_t err, void *ext);
	
    // Interface server
    typedef struct
    {
        // get version
        ipc_inf_version (*version)(void);

        // methods
        int32_t (*hello)(const char * name, test_hello_callback_t cb, void *ext);
		int32_t (*hifi_a78_msg_sync)(const test_xrp_msg_t fs_msg, test_hifi_a78_msg_sync_callback_t cb, void *ext);
		
        // broadcasts
        int32_t (*heartbeat_sub)(test_heartbeat_callback_t cb, void *ext, test_heartbeat_sub_callback_t cb2, void *ext2);
		int32_t (*heartbeat_unsub)(test_heartbeat_unsub_callback_t cb, void *ext);
		
        // message router, call in main loop.
        // receive messages
        int32_t (*receive_message)(void);
        // dispatch message
        int32_t (*dispatch_message)(void);
		test_xrp_msg_t *def_xrp_msg;
    }test_hifi_dsp_client;

    // init server
    test_hifi_dsp_client *test_hifi_dsp_client_init(void);
    // destroy client
    int32_t test_hifi_dsp_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_CLIENT_H
