
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights reserved. 
 * This file contains proprietary information that is the sole intellectual property of 
 * Black Sesame Technologies (Chengdu) Co., Ltd. No part of this material or its 
 * documentation may be reproduced, distributed, transmitted, displayed or published in
 * any manner without the written permission of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. Anyone who infringes on the copyright of Black Sesame Technologies (Chengdu) Co.,
 * Ltd. shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */


#ifndef VIDEO_SERVER_H
#define VIDEO_SERVER_H

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
	}video_MyArray_t;
	
	typedef enum
	{
	    NO_ERROR = 0,
		SERVER_FAIL = -1,
		ERROR_2 = -2,
		ERROR_3 = -3
	}video_ErrorEnum_t;
	

    // constants
    
    // method type
    typedef void (*video_isp2arm_t)(const uint32_t msgAddr, uint32_t* result, video_ErrorEnum_t* err, ext_info_t* info, void *ext);
	
    // broadcast type
    typedef void (*video_broadcast_sub_t)(uint8_t pid, uint8_t fid, uint8_t sid, void *ext);
    // Interface server
    typedef struct
    {
        // get version
        ipc_inf_version_t (*version)(void);

        // methods
        int32_t (*register_isp2arm)(video_isp2arm_t func, void *ext);
		
        // broadcasts
        int32_t (*arm2isp)(uint32_t msgAddr);
		int32_t (*register_arm2isp_subcribed)(video_broadcast_sub_t func, void *ext);
		int32_t (*register_arm2isp_unsubcribed)(video_broadcast_sub_t func, void *ext);
		
        // start message router.
        int32_t (*start)(void);
        // stop message router.
        int32_t (*stop)(void);
    }video_server;

    // initialize server
    video_server *video_server_init(void);
    // destroy client
    int32_t video_server_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // VIDEO_SERVER_H
