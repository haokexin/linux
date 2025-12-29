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

#ifndef _IPC_TRANS_STS_MGT_H
#define _IPC_TRANS_STS_MGT_H

#include <bst/bstipc_cfg.h>
#ifdef __cplusplus
extern "C" {
#endif

int32_t ipc_end_register(const uint8_t pid, void *addr);
int32_t ipc_end_unregister(const uint8_t pid, void *addr);
int32_t ipc_end_is_ready(const uint8_t endid, const uint8_t chipid, void *addr);

#ifdef __cplusplus
}
#endif
#endif
