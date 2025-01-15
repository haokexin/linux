// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 * IPC: Linux device driver for Blck Sesame Technologies inter-processor
 * communication
 *
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/mailbox_controller.h>
#include <linux/of_reserved_mem.h>
#include <linux/mailbox_client.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>

#include <bst/bstipc_cfg.h>
#include <bst/config.h>
#include <bst/ipc_common.h>
#include <bst/ipc_app_base.h>
#include <ipc_trans_layer.h>

/********************* macros *******************/
#define CPU_NR		 8
#define SEND_RETRY_TIMES 10000

enum msgbox_test_role {
	ROLES_NULL,
	ROLES_CLIENT,
	ROLES_SERVICE,
	ROLES_MUL_END,
	ROLES_SERVER_CLIENT,
};

static struct task_struct *g_client_tid, *g_server_tid;
#if 0
extern struct task_struct *start_client_test(void *data);
extern struct task_struct *start_server_test(void *data);
extern struct task_struct *start_trans_client_test(void);
extern struct task_struct *start_trans_server_test(void);
#endif

static uint32_t role;
module_param(role, uint, 0644);
MODULE_PARM_DESC(role, "The role for test(default INVALID)");

static uint32_t cpid;
module_param(cpid, uint, 0644);
MODULE_PARM_DESC(cpid, "The cpid for test(default CPU_0)");

static uint32_t ccid;
module_param(ccid, uint, 0644);
MODULE_PARM_DESC(ccid, "The ccid for test(default CPU_0)");

static uint32_t cfid;
module_param(cfid, uint, 0644);
MODULE_PARM_DESC(cfid, "The cfid for test(default FID_0)");

static uint32_t csid;
module_param(csid, uint, 0644);
MODULE_PARM_DESC(csid, "The csid for test(default SID_0)");

static uint32_t spid;
module_param(spid, uint, 0644);
MODULE_PARM_DESC(spid, "The spid for test(default CPU_0)");

static uint32_t scid;
module_param(scid, uint, 0644);
MODULE_PARM_DESC(scid, "The scid for test(default CPU_0)");

static uint32_t sfid;
module_param(sfid, uint, 0644);
MODULE_PARM_DESC(sfid, "The sfid for test(default FID_0)");

static uint32_t ssid;
module_param(ssid, uint, 0644);
MODULE_PARM_DESC(ssid, "The ssid for test(default SID_0)");

static rw_msg_header_t c_header;
static rw_msg_header_t s_header;

static int msgbox_test_init(void)
{
	u8 cpuid;

	c_header.pid = cpid;
	c_header.cid = ccid;
	c_header.sid = csid;
	c_header.fid = cfid;
	s_header.pid = spid;
	s_header.cid = scid;
	s_header.sid = ssid;
	s_header.fid = sfid;
	if (role != ROLES_CLIENT) {
		cpuid = get_cpuid_by_endid(spid);
		if (cpuid >= NR_CPUS)
			return -EINVAL;
	}

	pr_emerg("%s: client pid(0x%x) cid(0x%x)\n",
			__func__, c_header.pid, c_header.cid);
#if 0
	switch (role) {
	case ROLES_CLIENT:
		g_client_tid = start_client_test((void *)&c_header);
		wake_up_process(g_client_tid);
		break;
	case ROLES_SERVICE:
		g_server_tid = start_server_test((void *)&s_header);
		if (IS_ERR(g_server_tid))
			IPC_LOG_ERR("create server tast faill!");
		else
			kthread_bind(g_server_tid, cpuid);
		wake_up_process(g_server_tid);
		break;
	case ROLES_MUL_END:
		g_server_tid = start_trans_server_test();
		g_client_tid = start_trans_client_test();
		if (IS_ERR(g_server_tid))
			IPC_LOG_ERR("create server tast faill!");
		else
			kthread_bind(g_server_tid, cpuid);
		wake_up_process(g_server_tid);
		msleep(2000);
		wake_up_process(g_client_tid);
		break;
	case ROLES_SERVER_CLIENT:
		g_server_tid = start_server_test((void *)&s_header);
		g_client_tid = start_client_test((void *)&c_header);
		if (IS_ERR(g_server_tid))
			IPC_LOG_ERR("create server tast faill!");
		else
			kthread_bind(g_server_tid, cpuid);
		wake_up_process(g_server_tid);
		msleep(2000);
		wake_up_process(g_client_tid);
		break;
	default:
		pr_info("invalid role\n");
		break;
	}
#endif

	return 0;
}

static void msgbox_test_exit(void)
{
	if (g_client_tid)
		kthread_stop(g_client_tid);
	if (g_server_tid)
		kthread_stop(g_server_tid);
}

module_init(msgbox_test_init);
module_exit(msgbox_test_exit);

MODULE_DESCRIPTION("BST Msgbox test module");
MODULE_LICENSE("GPL v2");
