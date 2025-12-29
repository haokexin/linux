/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */

#include <asm/ioctl.h>
#include <linux/workqueue.h>
// #include <linux/ipc_interface.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/kfifo.h>

enum health_monitor_policy {
	RESTART_SYS,
	RESET_TASK,
	DEFAULT_POLICY,
};

struct hm_heart_beats {
	struct task_struct *hb_thread;
	struct workqueue_struct *wq;
	bool hb_thread_wakeup;
	struct work_struct hb_work; //heart beat wq.

	int session_id;
};

struct hm_safety_svc {
	uint32_t dtc;
	uint32_t errinfo;
};

struct hm_errinfo_wk {
	struct work_struct	dtc_errinfo_work;
};

struct hm_dtc_svc_wk {
	struct work_struct	dtc_work;
};

struct psm_msg{
	uint8_t block_id_in;
	uint8_t block_id_out;
	uint32_t psm_id_out[4];
};

int bst_health_monitor_print_level = 1;

#define BST_HEALTH_MONITOR_DEBUG_PRINT 2
#define BST_HEALTH_MONITOR_LOG_PRINT 1
#define BST_HEALTH_MONITOR_NO_PRINT 0

#define BST_HEALTH_MONITOR_DRIVER_NAME "bst_health_monitor"

#define BST_HEALTH_MONITOR_DEBUG_PRINTK(format, ...)                                   \
	do {                                                                 \
		if (bst_health_monitor_print_level >= BST_HEALTH_MONITOR_DEBUG_PRINT)            \
			printk(KERN_INFO "[%s]: %s %d: " format "\n",        \
			       BST_HEALTH_MONITOR_DRIVER_NAME, __FUNCTION__, __LINE__, \
			       ##__VA_ARGS__);                               \
	} while (0)

#define BST_HEALTH_MONITOR_LOG_PRINTK(format, ...)                                   \
	do {                                                                 \
		if (bst_health_monitor_print_level >= BST_HEALTH_MONITOR_LOG_PRINT)              \
			printk(KERN_INFO "[%s]: %s %d: " format "\n",        \
			       BST_HEALTH_MONITOR_DRIVER_NAME, __FUNCTION__, __LINE__, \
			       ##__VA_ARGS__);                               \
	} while (0)

#define BST_HEALTH_MONITOR_ERR_PRINTK(format, ...)                                   \
	do {                                                                 \
		if (bst_health_monitor_print_level >= BST_HEALTH_MONITOR_LOG_PRINT)              \
			pr_err("[E][%s]: %s %d:" format "\n", \
				BST_HEALTH_MONITOR_DRIVER_NAME, __func__, __LINE__, \
				##__VA_ARGS__);                         \
	} while (0)

#define HEALTH_MONITOR_IOC_MAGIC 'h'
#define HEALTH_MONITOR_IOC_MAXNR 6

#define HEALTH_MONITOR_CONF_POLICY \
		_IOW(HEALTH_MONITOR_IOC_MAGIC, 1, int)

#define HEALTH_MONITOR_HEART_BEATS \
		_IO(HEALTH_MONITOR_IOC_MAGIC, 2)

#define HEALTH_MONITOR_RESET_A55 \
		_IO(HEALTH_MONITOR_IOC_MAGIC, 3)

#define HEALTH_MONITOR_REPORT_DTC \
		_IOW(HEALTH_MONITOR_IOC_MAGIC, 4, int)

#define HEALTH_MONITOR_GET_PSMID \
		_IOW(HEALTH_MONITOR_IOC_MAGIC, 5, int)

#define IPC_STRATEGY_CODE		0x4
#define IPC_FAULT_CODE			0x0

#define SEC_TO_MSEC(x)		(x * 1000UL)
#define COMPLETED		1
#define WAIT_SEC		CONFIG_TIMEOUT_FOR_ROOTFS
#define IPC_RESTART_CMD		2
#define IPC_RESTART_DTC		1

#ifdef CONFIG_HEALTH_MONITOR_KHB
int health_monitor_khb_init(void);
void health_monitor_khb_exit(void);
extern unsigned int khb_core;
extern unsigned int khb_timeout;
#endif
extern unsigned int bst_sip_reset_a55(void);
#ifdef CONFIG_HEALTH_MONITOR_SYSFS
extern int health_monitor_sysfs_init(void);
extern void health_monitor_sysfs_exit(void);
extern struct completion rootfs_completion;
extern bool health_monitor_debug;
#endif
