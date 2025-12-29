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

#include "../include/config.h"
#include "ipc_trans_utl.h"

const uint8_t end_id_idx[MSGBX_END_COUNT_MAX] = MSGBX_END_VALUES_DEF;

int32_t query_end_id_idx(uint8_t end_id)
{
	uint8_t cnt = 0;

	for (; cnt < MSGBX_END_COUNT_MAX; ++cnt)
		if (end_id_idx[cnt] == end_id)
			return cnt;
	return -1;
}

int32_t end_idx_to_id(uint8_t idx)
{
	if (idx >= MSGBX_END_COUNT_MAX)
		return -1;

	return end_id_idx[idx];
}
