/* SPDX-License-Identifier: GPL-2.0 OR Apache 2.0
 *
 * Copyright (c) 2024 Black Sesame Technologies
 *
 * This program is also distributed under the terms of the Apache 2.0
 * License.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _IPC_TRANS_MSG_MGT_H
#define _IPC_TRANS_MSG_MGT_H

#include "ipc_trans_common.h"
#include <bst/ipc_serdes.h>

#ifdef __cplusplus
extern "C" {
#endif

int8_t msg_queue_init(bst_msg_queue_t *msg_queue);
int8_t msg_queue_in(bst_msg_queue_t *msg_queue, rw_msg_t *msg,
		    uint64_t timestamp);
int8_t msg_queue_out(bst_msg_queue_t *msg_queue, serdes_t *out);
int8_t rwmsg_queue_out(bst_msg_queue_t *msg_queue, rw_msg_t *out);
int8_t msg_queue_collate(bst_msg_queue_t *msg_queue, uint8_t is_err_handle);
int8_t msg_queue_recycle(void *ses);

#ifdef __cplusplus
}
#endif
#endif
