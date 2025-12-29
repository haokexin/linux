/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */

#include <asm/ioctl.h>
#include <linux/workqueue.h>
// #include <linux/ipc_interface.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/kfifo.h>

int bst_safetylib_print_level = 1;

#define BST_SAFETYLIB_DEBUG_PRINT 2
#define BST_SAFETYLIB_LOG_PRINT 1
#define BST_SAFETYLIB_NO_PRINT 0

#define BST_SAFETYLIB_DRIVER_NAME "bst_fusa_error_inject"

#define BST_SAFETYLIB_DEBUG_PRINTK(format, ...)                                   \
	do {                                                                 \
		if (bst_safetylib_print_level >= BST_SAFETYLIB_DEBUG_PRINT)            \
			printk(KERN_INFO "[%s]: %s %d: " format "\n",        \
			       BST_SAFETYLIB_DRIVER_NAME, __FUNCTION__, __LINE__, \
			       ##__VA_ARGS__);                               \
	} while (0)

#define BST_SAFETYLIB_LOG_PRINTK(format, ...)                                   \
	do {                                                                 \
		if (bst_safetylib_print_level >= BST_SAFETYLIB_LOG_PRINT)              \
			printk(KERN_INFO "[%s]: %s %d: " format "\n",        \
			       BST_SAFETYLIB_DRIVER_NAME, __FUNCTION__, __LINE__, \
			       ##__VA_ARGS__);                               \
	} while (0)

#define BST_SAFETYLIB_ERR_PRINTK(format, ...)                                   \
	do {                                                                 \
		if (bst_safetylib_print_level >= BST_SAFETYLIB_NO_PRINT)              \
			pr_err("[E][%s]: %s %d:" format "\n", \
				BST_SAFETYLIB_DRIVER_NAME, __func__, __LINE__, \
				##__VA_ARGS__);                         \
	} while (0)


#define RETRY_START_CLIENT_TIMES 5
#define IPC_STRATEGY_CODE		0x4
#define IPC_FAULT_CODE			0x0

#define SEC_TO_MSEC(x)		(x * 1000UL)
#define COMPLETED		1
#define WAIT_SEC		CONFIG_TIMEOUT_FOR_ROOTFS
#define IPC_RESTART_CMD		2
#define IPC_RESTART_DTC		1

