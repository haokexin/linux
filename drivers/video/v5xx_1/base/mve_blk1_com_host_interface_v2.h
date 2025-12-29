/*
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 * Copyright (C) 2021-2024 Black Sesame Technologies. All Rights Reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */

#ifndef MVE_COM_HOST_INTERFACE_V2_H
#define MVE_COM_HOST_INTERFACE_V2_H

#include "mve_blk1_com.h"
#include "mve_blk1_mem_region.h"

struct session;
struct mve_rsrc_dma_mem_t;

struct mve_com *mve_blk1_com_host_interface_v2_new(void);
const struct mve_mem_virt_region *mve_blk1_com_hi_v2_get_mem_virt_region(void);

uint32_t mve_blk1_com_message_ready(struct mve_session *session, struct mve_rsrc_dma_mem_t *host, struct mve_rsrc_dma_mem_t *mve);

#endif /* MVE_COM_HOST_INTERFACE_V2_H */
