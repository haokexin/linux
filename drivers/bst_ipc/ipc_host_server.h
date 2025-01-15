/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

/**
 *This header file shows up all the API for firmwares and user space.
====================================================================
	      | method_call/subscribe/reply callback
	      -----------------------------------------------------
  user space  | bst_read/bst_write/bst_alloc/bst_free  <-------------the 2 level APIs are same--|
	      -----------------------------------------------------                |            |
	      | ioctl                                                              |            |
====================================================================               |call        |
	      | ioctl                                                              |            |
	      ------------------------------------------------------               √            |
 kernel space |bst_alloc/bst_free/....                    <------------this file purpose        |
	      ------------------------------------------------------               ^            |
	      | ipc_cma_alloc / ipc_cma_free/.....                                 |            |
====================================================================               |call        |
	      |  method_call/.....                                                 |            |
	      -----------------------------------------------------                |            |
DSP firmware  | bst_read/bst_write/bst_alloc/bst_free  <-------------the 2 level APIs are same--|
	      -----------------------------------------------------
	      | read_in_reg/write_out_reg/...
====================================================================
*/
#ifndef IPC_GENERIC_API_H
#define IPC_GENERIC_API_H

#include "ipc_common.h"

struct ipc_buffer;

int32_t bst_alloc(struct ipc_buffer *ipc_buffer, bool from_user);
int32_t bst_free(phys_addr_t phy_addr, bool from_user);
void bst_print(const char *fmt, ...) __printf(1, 2);
void *get_kaddr_from_phy(phys_addr_t phy);

#endif
