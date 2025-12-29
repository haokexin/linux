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

#ifndef _MSGBX_C1200_H
#define _MSGBX_C1200_H

/* The file defines the fundamental properties of the hardware and SHOULD NOT be modified. */
#define MSGBX_END_COUNT_MAX 35U

// msgbx raw msg format definition
struct _rw_msg_header_t {
	uint32_t pid : 8;
	uint32_t cid : 8;
	uint32_t len : 4;
	uint32_t is_64b : 1;
	uint32_t is_nonsec : 1;
	uint32_t is_eof : 1;
	uint32_t resh : 1;
	uint32_t sid : 4;
	uint32_t fid : 4;
	uint32_t cmd : 8;
	uint32_t typ : 4;
	uint32_t res : 4;
	uint32_t idx : 4;
	uint32_t tok : 8;
	uint32_t ver : 4;
};
#define rw_msg_header_t struct _rw_msg_header_t

// msgbx endid definition
enum msgbox_pid_e {
	CPU_0 = ((1U << 4) | 0U),
	CPU_1 = ((1U << 4) | 1U),
	CPU_2 = ((1U << 4) | 2U),
	CPU_3 = ((1U << 4) | 3U),
	CPU_4 = ((1U << 4) | 4U),
	CPU_5 = ((1U << 4) | 5U),
	CPU_6 = ((1U << 4) | 6U),
	CPU_7 = ((1U << 4) | 7U),
	CPUMP2_0 = ((2U << 4) | 0U),
	CPUMP2_1 = ((2U << 4) | 1U),
	ISPCV_0 = ((3U << 4) | 0U),
	ISPCV_1 = ((3U << 4) | 1U),
	ISPCV_2 = ((3U << 4) | 2U),
	ISPCV_3 = ((3U << 4) | 3U),
	ISPCV_4 = ((3U << 4) | 4U),
	NET_0 = ((4U << 4) | 0U),
	DMA_0 = ((5U << 4) | 0U),
	DMA_1 = ((5U << 4) | 1U),
	SWITCH_0 = ((6U << 4) | 0U),
	SWITCH_1 = ((6U << 4) | 1U),
	SWITCH_2 = ((6U << 4) | 2U),
	SWITCH_3 = ((6U << 4) | 3U),
	SWITCH_4 = ((6U << 4) | 4U),
	SWITCH_5 = ((6U << 4) | 5U),
	SECURE_0 = ((7U << 4) | 0U),
	SECURE_1 = ((7U << 4) | 1U),
	SAFETY_0 = ((8U << 4) | 0U),
	SAFETY_1 = ((8U << 4) | 1U),
	REALTIME_0 = ((9U << 4) | 0U),
	REALTIME_1 = ((9U << 4) | 1U),
	REALTIME_2 = ((9U << 4) | 2U),
	REALTIME_3 = ((9U << 4) | 3U),
	REALTIME_4 = ((9U << 4) | 4U),
	REALTIME_5 = ((9U << 4) | 5U),
	MEDIA_0 = ((10U << 4) | 0U),
};

#define MSGBX_END_VALUES_DEF {CPU_0, CPU_1, CPU_2, CPU_3, CPU_4, CPU_5, CPU_6, CPU_7, CPUMP2_0, CPUMP2_1, \
	ISPCV_0, ISPCV_1, ISPCV_2, ISPCV_3, ISPCV_4, NET_0, DMA_0, DMA_1, SWITCH_0, SWITCH_1, \
	SWITCH_2, SWITCH_3, SWITCH_4, SWITCH_5, SECURE_0, SECURE_1, SAFETY_0, SAFETY_1, \
	REALTIME_0, REALTIME_1, REALTIME_2, REALTIME_3, REALTIME_4, REALTIME_5, MEDIA_0};

static inline int32_t end_is_valid(const uint8_t end_id)
{
	if (end_id == CPU_0)
		return 1;
	else if (end_id == CPU_1)
		return 1;
	else if (end_id == CPU_2)
		return 1;
	else if (end_id == CPU_3)
		return 1;
	else if (end_id == CPU_4)
		return 1;
	else if (end_id == CPU_5)
		return 1;
	else if (end_id == CPU_6)
		return 1;
	else if (end_id == CPU_7)
		return 1;
	else if (end_id == CPUMP2_0)
		return 1;
	else if (end_id == CPUMP2_1)
		return 1;
	else if (end_id == ISPCV_0)
		return 1;
	else if (end_id == ISPCV_1)
		return 1;
	else if (end_id == ISPCV_2)
		return 1;
	else if (end_id == ISPCV_3)
		return 1;
	else if (end_id == ISPCV_4)
		return 1;
	else if (end_id == NET_0)
		return 1;
	else if (end_id == DMA_0)
		return 1;
	else if (end_id == DMA_1)
		return 1;
	else if (end_id == SWITCH_0)
		return 1;
	else if (end_id == SWITCH_1)
		return 1;
	else if (end_id == SWITCH_2)
		return 1;
	else if (end_id == SWITCH_3)
		return 1;
	else if (end_id == SWITCH_4)
		return 1;
	else if (end_id == SWITCH_5)
		return 1;
	else if (end_id == SECURE_0)
		return 1;
	else if (end_id == SECURE_1)
		return 1;
	else if (end_id == SAFETY_0)
		return 1;
	else if (end_id == SAFETY_1)
		return 1;
	else if (end_id == REALTIME_0)
		return 1;
	else if (end_id == REALTIME_1)
		return 1;
	else if (end_id == REALTIME_2)
		return 1;
	else if (end_id == REALTIME_3)
		return 1;
	else if (end_id == REALTIME_4)
		return 1;
	else if (end_id == REALTIME_5)
		return 1;
	else if (end_id == MEDIA_0)
		return 1;
	else
		return 0;
}
#define MULTI_DIE_ENDMAP_OFFSET 0U
#define MULTI_DST_CLIENT 0U

#define MSG_END_MGT_CONFIG                                                     \
	(MSGBX_ECC_RX_MULTIP_EN_BIT |  MSGBX_ECC_RX_DETECT_EN_BIT |            \
	 MSGBX_PARITY_HWDATA_EN_BIT | MSGBX_PARITY_HADDR_EN_BIT)
#define MSG_DEF_FLT_MGT_CONFIG                                                 \
	(MSGBX_RX_OVERFLOW_EN_BIT | MSGBX_RX_UNDERFLOW_EN_BIT)

/**
 * @brief Get the UUID from the header
 *
 * This function calculates the UUID from the given header by combining the
 * PID, SID, and FID fields.
 *
 * @param header The header containing the PID, SID, and FID fields
 * @return The calculated UUID
 */
static inline uint16_t ipc_msg_get_uuid(rw_msg_header_t header)
{
	return (header.pid << 8) | (header.sid << 4) | (header.fid);
}
#endif
