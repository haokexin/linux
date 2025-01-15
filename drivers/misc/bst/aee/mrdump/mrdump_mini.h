/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * Copyright (C) 2024 Black Sesame Technologies. Inc.
 */

#if !defined(__MRDUMP_MINI_H__)
#ifdef CONFIG_KASAN
#ifdef CONFIG_ARM64_4K_PAGES
#define KASAN_SR_START	0xffff600000000000ul
#define KASAN_SR_END	0xffff7ffffffffffful
#elif defined(CONFIG_ARM64_64K_PAGES)
#define KASAN_SR_START	0xfffd800000000000ul
#define KASAN_SR_END	0xffff7ffffffffffful
#else
#define KASAN_SR_START	0xfffffffffffffffful
#define KASAN_SR_END	0xfffffffffffffffful
#endif  //CONFIG_ARM64_PAGES
#endif //CONFIG_KASAN

struct mrdump_mini_extra_misc {
	void (*dump_func)(unsigned long *vaddr, unsigned long *size);
	const char *dump_name;
	unsigned long max_size;
};
void mrdump_mini_add_extra_misc(void);
void mrdump_mini_add_hang_raw(unsigned long vaddr, unsigned long size);
int mrdump_mini_init(void);
int mrdump_mini_filp_init(void);
extern raw_spinlock_t logbuf_lock;
extern unsigned long *stack_trace;
extern void get_kernel_log_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start);
extern void get_hang_detect_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start);
#if defined(CONFIG_TRUSTY_LOG)
extern void get_gz_log_buffer(unsigned long *addr, unsigned long *size,
		unsigned long *start);
#endif
extern struct ram_console_buffer *ram_console_buffer;

#ifdef CONFIG_IKCONFIG_PROC
extern char kernel_config_data;
extern char kernel_config_data_end;
#endif
#endif
