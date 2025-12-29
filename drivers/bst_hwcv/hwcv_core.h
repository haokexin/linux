/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __BST_HWCV_CORE_H__
#define __BST_HWCV_CORE_H__

#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/idr.h>
#include <linux/dev_printk.h>
#ifdef CONFIG_BST_HWCV_MULTI_OS
#include "linux/bst_samphore.h"
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define HWCV_DRIVER_NAME "bst_hwcv"

/* version */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define DRIVER_MAJOR_VERISON 1
#define DRIVER_MINOR_VERSION 3
#define DRIVER_REVISION_VERSION 0
#define DRIVER_PATCH_VERSION

#define DRIVER_VERSION                                                    \
	(STR(DRIVER_MAJOR_VERISON) "." STR(DRIVER_MINOR_VERSION) "." STR( \
		DRIVER_REVISION_VERSION) STR(DRIVER_PATCH_VERSION))

#define HWCV_MAX_GWARP_NUM 2

extern struct hwcv_drvdata *hwcv_drvdata;

struct hwcv_core;

enum hwcv_device_type {
	HWCV_DEVICE_A1000,
	HWCV_DEVICE_A2000,
	HWCV_DEVICE_C1200
};

enum hwcv_job_state {
	HWCV_SCALER_DONE,
	HWCV_GWARP0_NORMAL_DONE,
	HWCV_GWARP0_SNR0_DONE,
	HWCV_GWARP0_SNR1_DONE,
	HWCV_GWARP0_SNR2_DONE,
	HWCV_GWARP0_SNR3_DONE,
	HWCV_GWARP1_NORMAL_DONE,
	HWCV_GWARP1_SNR0_DONE,
	HWCV_GWARP1_SNR1_DONE,
	HWCV_GWARP1_SNR2_DONE,
	HWCV_GWARP1_SNR3_DONE,
};

enum hwcv_req_state {
	HWCV_REQ_SCALER = 1 << 0,
	HWCV_REQ_GWARP0 = 1 << 1,
	HWCV_REQ_GWARP1 = 1 << 2,
	HWCV_REQ_MASK = 0xff
};

struct hwcv_session {
	int id;
	pid_t tgid;
	char *pname;
};

struct hwcv_session_manager {
	struct mutex lock;
	struct idr ctx_id_idr;
	u32 session_cnt;
};

struct hwcv_backend_ops {
	int (*init_hw)(struct hwcv_core *core);
	bool (*is_ready)(struct hwcv_core *core);
	int (*debug_sys)(struct hwcv_core *core, struct seq_file *m);

	int (*reset_scaler)(struct hwcv_core *core);
	int (*do_scaler)(struct hwcv_core *core, void *param);
	int (*debug_scaler)(struct hwcv_core *core, struct seq_file *m);
	int (*poll_scaler)(struct hwcv_core *core, u64 timeout_us);
	int (*dump_scaler_regs)(struct hwcv_core *core);

	int (*reset_gwarp)(struct hwcv_core *core, u8 id);
	int (*do_gwarp)(struct hwcv_core *core, void *param);
	int (*debug_gwarp)(struct hwcv_core *core, struct seq_file *m);
	int (*poll_gwarp)(struct hwcv_core *core, u8 id, u64 timeout_us);
	int (*dump_gwarp_regs)(struct hwcv_core *core, u8 id);

	int (*irq)(struct hwcv_core *core);
	int (*isr_thread)(struct hwcv_core *core);
};

struct hwcv_match_data {
	enum hwcv_device_type device_type;
	const struct hwcv_backend_ops *ops;
	const struct hwcv_hw_data *hw_data;
};

struct hwcv_hw_data {
	/* Gwarp */
	u8 gwarp_num;
	bool support_sbs;
};

struct hwcv_profiling {
	ktime_t request;
	ktime_t lock_done;
	ktime_t config_done;
	ktime_t frame_done;
	ktime_t notify_done;
	u32 work_cycle;
};

struct hwcv_ktime {
	u64 cur;
	u64 max;
	u64 min;
	u64 average;
	u64 sum;
	u64 count;
};

struct hwcv_stat {
	u64 total_frames;
	u64 success_frames;
	u64 failed_frames;

	struct hwcv_ktime get_lock;
	struct hwcv_ktime config_reg;
	struct hwcv_ktime complete_frame;
	struct hwcv_ktime notify_upper;
	struct hwcv_ktime hw_cycle;
};

struct hwcv_core {
	struct device *dev;

	unsigned long request_state;
	unsigned long job_state;
	struct mutex scaler_lock;
	struct completion scaler_done;
	struct mutex gwarp_lock[HWCV_MAX_GWARP_NUM];
	struct completion gwarp_done[HWCV_MAX_GWARP_NUM];

	/* Hardware control */
	void __iomem *sys_base;
	void __iomem *scaler_base;
	void __iomem *gwarp_base[HWCV_MAX_GWARP_NUM];
	int irq;
	const struct hwcv_backend_ops *ops;
	const struct hwcv_hw_data *hw_data;

	/* Cross-Domain control */
#ifdef CONFIG_BST_HWCV_MULTI_OS
	struct bst_samphore *scaler_hw_lock;
	struct bst_samphore *gwarp_hw_lock[HWCV_MAX_GWARP_NUM];
#endif

	/* stat */
	struct hwcv_profiling scaler_profiling;
	struct hwcv_profiling gwarp_profiling[HWCV_MAX_GWARP_NUM];
	struct hwcv_stat scaler_stat;
	struct hwcv_stat gwarp_stat[HWCV_MAX_GWARP_NUM];
};

struct hwcv_drvdata {
	struct hwcv_core *core;

	struct hwcv_mm *mm;

	struct hwcv_session_manager *session_manager;

#ifdef CONFIG_BST_HWCV_DEBUGGER
	struct hwcv_debugger *debugger;
#endif
};

static inline u32 hwcv_sys_read(struct hwcv_core *core, u32 offset)
{
	u32 value;

	value = readl_relaxed(core->sys_base + offset);
	pr_debug("value[0x%08x], offset[0x%x]", value, offset);

	return value;
}

static inline u32 hwcv_scaler_read(struct hwcv_core *core, u32 offset)
{
	u32 value;

	value = readl_relaxed(core->scaler_base + offset);
	pr_debug("value[0x%08x], offset[0x%x]", value, offset);

	return value;
}

static inline u32 hwcv_gwarp_read(struct hwcv_core *core, u8 id, u32 offset)
{
	u32 value;

	value = readl_relaxed(core->gwarp_base[id] + offset);
	pr_debug("id[%u], value[0x%08x], offset[0x%x]", id, value, offset);

	return value;
}

static inline void hwcv_sys_write(struct hwcv_core *core, u32 value, u32 offset)
{
	pr_debug("value[0x%08x], offset[0x%x]", value, offset);
	writel_relaxed(value, core->sys_base + offset);
}

static inline void hwcv_scaler_write(struct hwcv_core *core, u32 value,
				     u32 offset)
{
	pr_debug("value[0x%08x], offset[0x%x]", value, offset);
	writel_relaxed(value, core->scaler_base + offset);
}

static inline void hwcv_gwarp_write(struct hwcv_core *core, u8 id, u32 value,
				    u32 offset)
{
	pr_debug("id[%u], value[0x%08x], offset[0x%x]", id, value, offset);
	writel_relaxed(value, core->gwarp_base[id] + offset);
}

#endif
