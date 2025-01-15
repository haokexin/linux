/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 *
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

#ifndef _IPC_TRANS_UTL_H
#define _IPC_TRANS_UTL_H

#include <bst/bstipc_cfg.h>
#include <bst/ipc_trans_common.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t end_is_valid(const uint8_t end_id);
int32_t query_end_id_idx(uint8_t end_id);
int32_t end_idx_to_id(uint8_t idx);

#ifdef __cplusplus
}
#endif
#endif
