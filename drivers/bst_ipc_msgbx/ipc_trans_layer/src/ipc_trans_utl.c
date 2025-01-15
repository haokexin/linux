// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is also distributed under the terms of the BSD 3-Clause
 * License.
 *
 * Copyright (C) 2023 Black Sesame Technologies. Inc.
 */

#include "../include/config.h"
#include "ipc_trans_utl.h"

const uint8_t end_id_idx[MSGBX_END_COUNT_MAX] = {
	((1U << 4) | 0U), ((1U << 4) | 1U), ((1U << 4) | 2U),  ((1U << 4) | 3U),
	((1U << 4) | 4U), ((1U << 4) | 5U), ((1U << 4) | 6U),  ((1U << 4) | 7U),
	((2U << 4) | 0U), ((2U << 4) | 1U), ((3U << 4) | 0U),  ((3U << 4) | 1U),
	((3U << 4) | 2U), ((3U << 4) | 3U), ((3U << 4) | 4U),  ((4U << 4) | 0U),
	((5U << 4) | 0U), ((5U << 4) | 1U), ((6U << 4) | 0U),  ((6U << 4) | 1U),
	((6U << 4) | 2U), ((6U << 4) | 3U), ((6U << 4) | 4U),  ((6U << 4) | 5U),
	((7U << 4) | 0U), ((7U << 4) | 1U), ((8U << 4) | 0U),  ((8U << 4) | 1U),
	((9U << 4) | 0U), ((9U << 4) | 1U), ((9U << 4) | 2U),  ((9U << 4) | 3U),
	((9U << 4) | 4U), ((9U << 4) | 5U), ((10U << 4) | 0U),
};

int32_t end_is_valid(const uint8_t end_id)
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
