
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights
 * reserved. This file contains proprietary information that is the sole
 * intellectual property of Black Sesame Technologies (Chengdu) Co., Ltd. No
 * part of this material or its documentation may be reproduced, distributed,
 * transmitted, displayed or published in any manner without the written
 * permission of Black Sesame Technologies (Chengdu) Co., Ltd. Anyone who
 * infringes on the copyright of Black Sesame Technologies (Chengdu) Co., Ltd.
 * shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */

#ifndef BSTN_CLIENT_H
#define BSTN_CLIENT_H

#include <bst/ipc_app_common.h>

#ifdef __cplusplus
extern "C" {
#endif

// user defined types

// constants

// method types
typedef void (*disp_run_callback_t)(const uint32_t status,
                                    const uint32_t perf_us, void *ext);
typedef void (*disp_req_callback_t)(const uint32_t rep, void *ext);

// Broadcast types

//
typedef void (*avail_changed_callback_t)(bool avail, void *ext);

// Interface server
typedef struct {
  // get version
  ipc_inf_version_t (*version)(void);

  // Register availability changed callback.
  int32_t (*register_avail_changed)(avail_changed_callback_t cb, void *ext);

  // methods
  int32_t (*disp_run_sync)(const uint32_t opcode, const uint32_t opdata,
                           const uint32_t *status, const uint32_t *perf_us,
                           int64_t timeout_ms);
  int32_t (*disp_run_async)(const uint32_t opcode, const uint32_t opdata,
                            disp_run_callback_t cb, void *ext);

  int32_t (*disp_req_sync)(const uint32_t req,
                           const uint32_t *rep,
                           int64_t timeout_ms);
  int32_t (*disp_req_async)(const uint32_t req,
                            disp_req_callback_t cb, void *ext);

  // broadcasts

  // start message router.
  int32_t (*start)(void);
  // stop message router.
  int32_t (*stop)(void);
} bstn_client;

// init server
bstn_client *bstn_client_init(void);
// destroy client
int32_t bstn_client_destroy(void);

#ifdef __cplusplus
}
#endif

#endif // BSTN_CLIENT_H
