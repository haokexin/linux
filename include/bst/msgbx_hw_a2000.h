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

#ifndef _MSGBX_A2000_H
#define _MSGBX_A2000_H

/* The file defines the fundamental properties of the hardware and SHOULD NOT be modified. */
#define MSGBX_END_COUNT_MAX 32U

// msgbx raw msg format definition
struct _rw_msg_header_t {
	uint32_t pid : 8;
	uint32_t cid : 8;
	uint32_t len : 4;
	uint32_t is_64b : 1;
	uint32_t non_sec : 1;
	uint32_t chip_pid : 2;
	uint32_t chip_cid : 2;
	uint32_t is_loop : 1;
	uint32_t res : 5;
	uint32_t typ : 4;
	uint32_t sid : 4;
	uint32_t fid : 3;
	uint32_t is_eof : 1;
	uint32_t cmd : 8;
	uint32_t tok : 8;
	uint32_t idx : 4;
};
#define rw_msg_header_t struct _rw_msg_header_t

// msgbx endid definition
enum msgbox_pid_e {
	CPU_0_0 = ((0U << 4) | 0U),
	CPU_0_1 = ((0U << 4) | 1U),
	CPU_0_2 = ((0U << 4) | 2U),
	CPU_0_3 = ((0U << 4) | 3U),
	CPU_1_0 = ((1U << 4) | 0U),
	CPU_1_1 = ((1U << 4) | 1U),
	CPU_1_2 = ((1U << 4) | 2U),
	CPU_1_3 = ((1U << 4) | 3U),
	CPU_2_0 = ((2U << 4) | 0U),
	CPU_2_1 = ((2U << 4) | 1U),
	CPU_2_2 = ((2U << 4) | 2U),
	CPU_2_3 = ((2U << 4) | 3U),
	CPU_3_0 = ((3U << 4) | 0U),
	CPU_3_1 = ((3U << 4) | 1U),
	CPU_3_2 = ((3U << 4) | 2U),
	CPU_3_3 = ((3U << 4) | 3U),
	DNPAD_0 = ((4U << 4) | 0U),
	DNPAD_1 = ((4U << 4) | 1U),
	DNPAD_2 = ((4U << 4) | 2U),
	DNPAD_3 = ((4U << 4) | 3U),
	DNPAD_4 = ((4U << 4) | 4U),
	DNPAD_5 = ((4U << 4) | 5U),
	ISPCV_0 = ((5U << 4) | 0U),
	ISPCV_1 = ((5U << 4) | 1U),
	ISPCV_2 = ((5U << 4) | 2U),
	ISPCV_3 = ((5U << 4) | 3U),
	SECURE_0 = ((6U << 4) | 0U),
	SAFETY_0 = ((7U << 4) | 0U),
	NET_0 = ((8U << 4) | 0U),
	NET_1 = ((8U << 4) | 1U),
	UPPAD_0 = ((9U << 4) | 0U),
	UPPAD_1 = ((9U << 4) | 1U),
};

enum msgbox_chipid_e {
	MULTI_DIE_CHIP_0,
	MULTI_DIE_CHIP_1,
};
#define MULTI_DIE_ENDMAP_OFFSET 64U

#define MSGBX_END_VALUES_DEF {CPU_0_0, CPU_0_1, CPU_0_2, CPU_0_3, CPU_1_0, CPU_1_1, CPU_1_2, CPU_1_3, CPU_2_0, CPU_2_1, \
	CPU_2_2, CPU_2_3, CPU_3_0, CPU_3_1, CPU_3_2, CPU_3_3, DNPAD_0, DNPAD_1, DNPAD_2, DNPAD_3, \
	DNPAD_4, DNPAD_5, ISPCV_0, ISPCV_1, ISPCV_2, ISPCV_3, SECURE_0, SAFETY_0, \
	NET_0, NET_1, UPPAD_0, UPPAD_1};

static inline int32_t end_is_valid(const uint8_t end_id)
{
	if (end_id == CPU_0_0)
		return 1;
	else if (end_id == CPU_0_1)
		return 1;
	else if (end_id == CPU_0_2)
		return 1;
	else if (end_id == CPU_0_3)
		return 1;
	else if (end_id == CPU_1_0)
		return 1;
	else if (end_id == CPU_1_1)
		return 1;
	else if (end_id == CPU_1_2)
		return 1;
	else if (end_id == CPU_1_3)
		return 1;
	else if (end_id == CPU_2_0)
		return 1;
	else if (end_id == CPU_2_1)
		return 1;
	else if (end_id == CPU_2_2)
		return 1;
	else if (end_id == CPU_2_3)
		return 1;
	else if (end_id == CPU_3_0)
		return 1;
	else if (end_id == CPU_3_1)
		return 1;
	else if (end_id == CPU_3_2)
		return 1;
	else if (end_id == CPU_3_3)
		return 1;
	else if (end_id == DNPAD_0)
		return 1;
	else if (end_id == DNPAD_1)
		return 1;
	else if (end_id == DNPAD_2)
		return 1;
	else if (end_id == DNPAD_3)
		return 1;
	else if (end_id == DNPAD_4)
		return 1;
	else if (end_id == DNPAD_5)
		return 1;
	else if (end_id == ISPCV_0)
		return 1;
	else if (end_id == ISPCV_1)
		return 1;
	else if (end_id == ISPCV_2)
		return 1;
	else if (end_id == ISPCV_3)
		return 1;
	else if (end_id == SECURE_0)
		return 1;
	else if (end_id == SAFETY_0)
		return 1;
	else if (end_id == NET_0)
		return 1;
	else if (end_id == UPPAD_0)
		return 1;
	else if (end_id == UPPAD_1)
		return 1;
	else
		return 0;
}

#define MULTI_DST_CLIENT 0xff
#ifdef IPC_STATE_MGT_ENABLE
#define MSG_END_MGT_CONFIG                                                     \
	(MSGBX_ECC_RX_MULTIP_EN_BIT | MSGBX_ECC_RX_DETECT_EN_BIT |             \
	 MSGBX_PARITY_HWDATA_EN_BIT | MSGBX_PARITY_HADDR_EN_BIT)
#define MSG_DEF_FLT_MGT_CONFIG                                                 \
	(MSGBX_TX_MSGBX_ERR_EN_BIT | MSGBX_RX_OVERFLOW_EN_BIT |                 \
	 MSGBX_RX_UNDERFLOW_EN_BIT)
#define MSG_FLT_MGT_CONFIG                                                     \
	(MSGBX_RX_OVERFLOW_EN_BIT | MSGBX_RX_UNDERFLOW_EN_BIT)
#else
#define MSG_END_MGT_CONFIG 0
#define MSG_DEF_FLT_MGT_CONFIG 0
#define MSG_FLT_MGT_CONFIG 0
#endif

/**
 * @brief Get the UUID from the header
 *
 * This function calculates the UUID from the given header by combining the
 * CHIP_PID, PID, SID, and FID fields.
 *
 * @param header The header containing the CHIP_PID, PID, SID, and FID fields
 * @return The calculated UUID
 */
static inline uint16_t ipc_msg_get_uuid(rw_msg_header_t header)
{
	return (header.chip_pid << 8) | (header.pid << 8) | (header.sid << 4) | (header.fid);
}
#endif
