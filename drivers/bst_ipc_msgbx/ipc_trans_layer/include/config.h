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

#ifndef _IPC_TRANS_CONFIG_H
#define _IPC_TRANS_CONFIG_H

#include <bst/bstipc_cfg.h>

/* Define the central monitor end id
 */
#define CENTRAL_MONITOR_END_ID SAFETY_0 // release setting

#ifdef BAREMETAL_VERSION_TRUNCATE
#define CHANNEL_COUNT_BITS  0U  //rw
#define SESSION_COUNT       2U  // rw
#define REMOVE_STS_MGT
#else
#define CHANNEL_COUNT_BITS  CONFIG_MSGBOX_CHANNEL_COUNT_BITS  //rw
#define SESSION_COUNT       16U // rw
#define DEBUG_MODE_ENABLE
#define TIMESTAMP_DEBUG_ENABLE
// #define ENABLE_REMOTE_LOG_PROCESS_FUNC
#define USE_EXTERNAL_MSG_BUFFER
// #define ENABLE_SERVER_ERROR_CODE_REPLY
#endif

#define CHANNEL_COUNT (1U << CHANNEL_COUNT_BITS)
/* Define the msgbx receive mode
 * 1 means interrupt; 2 means polling
 */
#define MSGBX_RECV_MODE 1

/* Define the msgbx filter rule version
 */
#define FILTER_RULE_VERSION 0

#endif
