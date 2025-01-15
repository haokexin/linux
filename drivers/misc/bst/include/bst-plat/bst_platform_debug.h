/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#ifndef __BST_PLATFORM_DEBUG_H__
#define __BST_PLATFORM_DEBUG_H__

#ifdef CONFIG_BST_PLAT_SRAM_FLAG
/* plat_sram_flag */
extern int set_sram_flag_lastpc_valid(void);
extern int set_sram_flag_dfd_valid(void);
extern int set_sram_flag_etb_user(unsigned int etb_id, unsigned int user_id);

#define ETB_USER_BIG_CORE       0x0
#define ETB_USER_CM4            0x1
#define ETB_USER_AUDIO_CM4      0x2
#define ETB_USER_BUS_TRACER     0x3
#define ETB_USER_MCSIB_TRACER   0x4
#endif

#ifdef CONFIG_BST_DFD_INTERNAL_DUMP
extern int dfd_setup(void);
#endif

#endif /* __BST_PLATFORM_DEBUG_H__ */
