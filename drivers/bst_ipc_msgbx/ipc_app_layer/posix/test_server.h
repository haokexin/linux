
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */


#ifndef TEST_SERVER_H
#define TEST_SERVER_H

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
	
    // method type
    typedef void (*test_hello_t)(const char * name, char ** message, test_ErrorEnum_t* err, ext_info_t* info);
	typedef void (*test_complex_method_t)(const uint32_t in1, const char * in2, const byte_buffer in3, const test_MyArray_t in4, const test_MyStruct2_t in5, const test_MyUnion_t in6, uint32_t* out1, char ** out2, byte_buffer* out3, test_MyArray_t* out4, test_MyStruct2_t* out5, test_MyUnion_t* out6, test_ErrorEnum_t* err, ext_info_t* info);
	
    // broadcast type
    typedef void (*test_broadcast_sub_t)(uint8_t pid, uint8_t fid, uint8_t sid);
    // Interface server
    typedef struct
    {
        // get version
        ipc_inf_version_t (*version)(void);

        // register methods
        int32_t (*register_hello)(test_hello_t func);
		int32_t (*register_complex_method)(test_complex_method_t func);
		
        //register broadcasts
        int32_t (*heartbeat)(uint8_t status);
		int32_t (*register_heartbeat_subcribed)(test_broadcast_sub_t func);
		int32_t (*register_heartbeat_unsubcribed)(test_broadcast_sub_t func);
		
        // start message router.
        int32_t (*start)(void);
        // stop message router.
        int32_t (*stop)(void);
    }test_server;

    // init server
    test_server *test_server_init(rw_msg_header_t *);
    // destroy client
    int32_t test_server_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // TEST_SERVER_H
