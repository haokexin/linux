
/* © Copyright  Black Sesame Technologies (Chengdu) Co., Ltd. 2021. All rights
 * reserved. This file contains proprietary information that is the sole
 * intellectual property of Black Sesame Technologies (Chengdu) Co., Ltd. No
 * part of this material or its documentation may be reproduced, distributed,
 * transmitted, displayed or published in any manner without the written
 * permission of Black Sesame Technologies (Chengdu) Co., Ltd. Anyone who
 * infringes on the copyright of Black Sesame Technologies (Chengdu) Co., Ltd.
 * shall be held accountable by Black Sesame Technologies (Chengdu) Co., Ltd..
 */

#include <bst/ipc_serdes.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <bst/ipc_app_client_utils.h>
#include <bst/ipc_app_common.h>
#include <bst/ipc_trans_common.h>
#include <bst/ipc_trans_layer.h>
#include "bstcv_client.h"

// macro definitions
#define PID CPU_3
#define CID ISPCV_1
#define FID DEF
#define SID 7U

#define CMD_METHOD_DISP_RUN 0U

// local variables
static ipc_inf_version_t s_version = {.major = 1, .minor = 0};
static uint8_t s_handle = 0U;
static bstcv_client s_client = {0};
static volatile _Atomic uint8_t s_token = 0;
static int8_t s_recv_buffer[IPC_MAX_DATA_SIZE] = {0};
static callback_registration_t s_method_registry[IPC_TOKEN_NUM] = {0};
static struct mutex s_send_mtx = {0};
static avail_changed_callback_t s_avail_cb = NULL;
static void *s_avail_ext = NULL;

// private data structure

typedef struct {
  uint32_t *status;
  uint32_t *perf_us;
} disp_run_out_t;

// interface implementation
// get interface version
static ipc_inf_version_t get_ipc_inf_version(void) { return s_version; }

// method

static void disp_run_sync_callback(const uint32_t status,
                                   const uint32_t perf_us, void *ext) {
  disp_run_out_t *out = (disp_run_out_t *)ext;
  if (!out)
    return;
  *out->status = status;
  *out->perf_us = perf_us;
}

static inline int32_t serialize_disp_run(serdes_t *ser, const uint32_t opcode,
                                         const uint32_t opdata) {
  int32_t ret = 0;
  ser->header.pid = PID;
  ser->header.cid = CID;
  ser->header.fid = FID;
  ser->header.sid = SID;
  ser->header.cmd = CMD_METHOD_DISP_RUN;
  ser->header.typ = IPC_MSG_TYPE_METHOD;
  ret = ipc_ser_put(ser, (uint8_t *)&opcode, sizeof(uint32_t));
  if (ret < 0)
    return -1;
  ret = ipc_ser_put(ser, (uint8_t *)&opdata, sizeof(uint32_t));
  if (ret < 0)
    return -1;
  return 0;
}

static int32_t call_disp_run_sync(const uint32_t opcode, const uint32_t opdata,
                                  const uint32_t *status,
                                  const uint32_t *perf_us, int64_t timeout_ms) {
  int32_t ret = 0;
  serdes_t serdes = {0};

  // serialize
  (void)ipc_ser_init(&serdes);

  ret = serialize_disp_run(&serdes, opcode, opdata);
  if (ret != 0) {
    pr_err("Serialize disp_run fail.\n");
    return -1;
  }

  // take and set registry
  uint8_t tok = 0;
  callback_registration_t *reg =
      take_registry(s_method_registry, &s_token, &tok);
  if (!reg)
    return -2;
  disp_run_out_t out = {.status = status, .perf_us = perf_us};
  (void)add_registry(reg, (void *)disp_run_sync_callback, (void *)&out);
  serdes.header.tok = tok;

  (void)ipc_ser_finish(&serdes);

  // send message
  uint32_t i = 0;
  mutex_lock(&s_send_mtx);
  ret = ipc_trans_layer_proxy_send_method(PID, s_handle, &serdes);
  mutex_unlock(&s_send_mtx);
  if (ret < 0) {
    clear_registry(reg);
    pr_err("send method disp_run fail %d.\n", ret);
    return ret;
  }
  // pr_info("call_disp_run_sync has send and begin to wait.");

  // wait for reply
  if (timeout_ms <= 0)
    ret = wait_on_registry(reg);
  else
    ret = timedwait_on_registry(reg, timeout_ms);
  if (ret < 0) {
    clear_registry(reg);
    pr_err("%s wait timeout\n", __func__);
  }

  return ret;
}

static int32_t call_disp_run_async(const uint32_t opcode, const uint32_t opdata,
                                   disp_run_callback_t cb, void *ext) {
  int32_t ret = 0;
  serdes_t serdes = {0};

  // serialize
  (void)ipc_ser_init(&serdes);

  ret = serialize_disp_run(&serdes, opcode, opdata);
  if (ret != 0) {
    pr_err("Serialize disp_run fail.\n");
    return -1;
  }

  // take and set registry
  uint8_t tok = 0;
  callback_registration_t *reg =
      take_registry(s_method_registry, &s_token, &tok);
  if (!reg)
    return -2;
  (void)add_registry(reg, (void *)cb, ext);
  serdes.header.tok = tok;

  (void)ipc_ser_finish(&serdes);

  // send message
  uint32_t i = 0;
  mutex_lock(&s_send_mtx);
  ret = ipc_trans_layer_proxy_send_method(PID, s_handle, &serdes);
  mutex_unlock(&s_send_mtx);
  if (ret < 0) {
    clear_registry(reg);
    pr_err("send method disp_run fail %d.\n", ret);
    return ret;
  }

  return 0;
}

static inline int32_t call_disp_run_callback(serdes_t *des) {
  if (!des)
    return -1;

  int32_t ret = 0;
  uint32_t length = 0;
  uint32_t status;
  uint32_t perf_us;

  // pr_info("call_disp_run_callback\n");

  ret = ipc_des_get(des, (uint8_t *)&status, sizeof(int32_t));
  if (ret < 0) {
    return -1;
  }
  ret = ipc_des_get(des, (uint8_t *)&perf_us, sizeof(int32_t));
  if (ret < 0) {
    return -1;
  }

  callback_registration_t *reg = &s_method_registry[des->header.tok];
  // pr_info("call_disp_run_callback reg->busy %d\n", reg->busy);
  if (reg->busy) {
    disp_run_callback_t cb = (disp_run_callback_t)(reg->cb);
    if (cb)
      cb(status, perf_us, reg->ext);
    notify_callback_registry(reg);
    // pr_info("call_disp_run_callback notify_callback_registry\n");
    clear_registry(reg);
  } else {
    pr_err("callback registry is invalid.\n");
  }
  return 0;
}

// broadcast

// receive messages
static int32_t receive_message(void) {
  int32_t ret = ipc_trans_layer_query_info(PID, s_handle);
#if 1
  // check if availability changed
  if (s_avail_cb) {
    if (QUERY_INFO_DST_STS_OFFLINE == ret)
      s_avail_cb(false, s_avail_ext);
    else if (QUERY_INFO_DST_STS_ONLINE == ret)
      s_avail_cb(true, s_avail_ext);
  }
#endif
  return ret;
}

// dispatch messages
static int32_t dispatch_message(void) {
  int32_t ret = 0;
  static serdes_t reply_des = {0};

  while (ipc_trans_layer_proxy_get_reply_msg(PID, s_handle, &reply_des) >= 0) {
    // pr_info("dispatch_message get_reply_msg cmd %d\n", reply_des.header.cmd);

    switch (reply_des.header.cmd) {
    case CMD_METHOD_DISP_RUN:
      ret = call_disp_run_callback(&reply_des);
      break;
    default:
      break;
    }
    (void)ipc_des_init(&reply_des);
    if (ret < 0)
      pr_err("deserialization failed.\n");
  }
  return ret;
}

static struct task_struct *route_task = NULL;
static volatile bool bRunning = false;
static int router_func(void *arg) {
  int32_t ret = 0;
  while (unlikely(!kthread_should_stop())) {
    ret = receive_message();
    if (ret < 0) {
      yield();
      continue;
    }
    ret = dispatch_message();
    if (ret < 0) {
      yield();
      continue;
    }
    yield();
  }

  return 0;
}

// start message router
static int32_t start(void) {
  if (bRunning)
    return 0;

  route_task = kthread_run(router_func, NULL, "bstcv1_client_thread");
  if (unlikely(!route_task)) {
    return -1;
  }
  bRunning = true;

  return 0;
}

// stop message router.
static int32_t stop(void) {
  if (!bRunning)
    return 0;

  // sleep 1 seconds.
  msleep(1000);
  if (likely(route_task)) {
    int32_t ret = kthread_stop(route_task);
    if (unlikely(ret))
      return -1;
  }
  bRunning = false;

  return 0;
}

// register availablity changed callback function
static int32_t register_avail_changed_cb(avail_changed_callback_t cb,
                                         void *ext) {
  s_avail_cb = cb;
  s_avail_ext = ext;
  return 0;
}

// init server
bstcv_client *bstcv1_client_init(void) {
  
  // create client handle.
  int ret = ipc_trans_layer_proxy_create_handle(PID, FID, SID, CID, &s_handle);
  if (ret < 0) {
    pr_err("%s() create client handle fail, ret=%d\n", __func__, ret);
    return NULL;
  }

  s_client.version = get_ipc_inf_version;
  s_client.register_avail_changed = register_avail_changed_cb;
  s_client.disp_run_sync = call_disp_run_sync;
  s_client.disp_run_async = call_disp_run_async;

  s_client.start = start;
  s_client.stop = stop;

  mutex_init(&s_send_mtx);
  // init_registry(&s_heartbeat_registry);
  init_registry_list(s_method_registry, IPC_TOKEN_NUM);
  return &s_client;
}
EXPORT_SYMBOL(bstcv1_client_init);

// destory client
int32_t bstcv1_client_destroy(void) {
  int32_t ret = ipc_trans_layer_destroy_handle(PID, s_handle);
  if (ret < 0)
    return ret;

  s_client.version = NULL;
  s_client.disp_run_sync = NULL;
  s_client.disp_run_async = NULL;

  s_client.start = NULL;
  s_client.stop = NULL;

  // destroy_registry(&s_heartbeat_registry);
  destroy_registry_list(s_method_registry, IPC_TOKEN_NUM);
  return ret;
}
EXPORT_SYMBOL(bstcv1_client_destroy);
