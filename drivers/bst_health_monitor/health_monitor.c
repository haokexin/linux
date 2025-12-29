// SPDX-License-Identifier: (GPL-2.0 OR MIT)

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <asm-generic/signal.h>

#include "health_monitor.h"

#define FIVE_SEC		5
#define SEC_TO_USEC(x)		(x * 1000000UL)
#define UHB_IPC_FAULT_CODE	0xa00001
#define GET_SAFETY_CFG_CMD	30
#define DTC_TO_SAFETY_SVC_CMD	31
#define INFO_TO_SAFETY_SVC_CMD	32
#define GET_SAFETY_CFG_TIMEOUT	1000
#define HEALTH_MONITOR_IPC_SHM	CONFIG_HEALTH_MONITOR_IPC_SHM
#define HEALTH_MONITOR_SHM_SIZE	PAGE_SIZE
#define HEALTH_MONITOR_DATA_NUM	(HEALTH_MONITOR_SHM_SIZE / 8UL)
#define MODULE_ID_MASK		0xFF
#define SAFETY_ACK_MASK		0xFFFFFF
#define STATUS_CODE		BIT(31)

// #ifdef CONFIG_SECOND_KERNEL
extern u8 msgbx_get_start_pid(void);
// #endif


// static int hm_policy = DEFAULT_POLICY;
// static u64 current_jiffies;
// static struct hm_safety_svc __iomem *shm_data;
static struct proc_dir_entry *health_monitor_proc;

#ifdef CONFIG_HEALTH_MONITOR_KHB
//Set the core for handling kernel heart beats in kernel module
unsigned int khb_core = CONFIG_HEALTH_MONITOR_KHB_CORE;
EXPORT_SYMBOL(khb_core);
module_param(khb_core, int, 0644);
MODULE_PARM_DESC(khb_core, "The core for handling kernel heart beats (default core0)");

//Set the timeout for handling kernel heart beats in kernel module
unsigned int khb_timeout = FIVE_SEC;
EXPORT_SYMBOL(khb_timeout);
module_param(khb_timeout, int, 0644);
MODULE_PARM_DESC(khb_timeout, "The timeout for handling kernel heart beats (default 1s)");
#endif


//Set the signal num for restarting uhb task in kernel module
static int uhb_sig = SIGSEGV;
module_param(uhb_sig, int, 0644);
MODULE_PARM_DESC(uhb_sig, "The signal for restarting uhb task (default SIGSEGV)");

//Set the timeout for restarting uhb task in kernel module
static int uhb_timeout = FIVE_SEC;
module_param(uhb_timeout, int, 0644);
MODULE_PARM_DESC(uhb_timeout, "The timeout for restarting uhb task (default 1s)");

// static struct hm_heart_beats	uhbs;
// static struct task_struct	*cur_task;
static struct kfifo		dtc_fifo;
// static struct kfifo		dtc_errinfo_fifo;
static struct workqueue_struct	*dtc_wq;
// static int			dtc_session[NR_CPUS];
// static int			err_session[NR_CPUS];

#ifdef CONFIG_HEALTH_MONITOR_FIFO_DEPTH
static u32 fifo_depth =		CONFIG_HEALTH_MONITOR_FIFO_DEPTH;
#else
static u32 fifo_depth =		1000;
#endif
#include "msgbox/safety-client/src-gen/HealthmonitorClient.h"
static HealthmonitorClient_t *m_client = NULL;
static HealthmonitorClient_data_t m_data={0};

static void dtc_wq_handler(struct work_struct *work)
{
	struct hm_dtc_svc_wk *wk =
		container_of(work, struct hm_dtc_svc_wk, dtc_work);

	// int session_id;
	u32 dtc = 0;
	int ret = -1;
	static int count;
	healthmonitor_ErrorEnum_t err = 0;

	ret = kfifo_out(&dtc_fifo, &dtc, sizeof(dtc));
	if (ret != sizeof(dtc))
		goto out;
	pr_debug("%s(%d) queue%d out: dtc(0x%x)\n",
			current->comm, current->pid, count, dtc);
	count = (count + 1) & (fifo_depth - 1);
	
	if(m_client)
		ret = m_client->healthmonitor_client.fusacoreip_method_sync(dtc,&err,1000,NULL);
	if (ret < 0 || err != HEALTHMONITOR_NO_ERROR)
		BST_HEALTH_MONITOR_ERR_PRINTK("healthmonitor_client send dtc %u failed,ret = %d, err = %d !!", dtc,ret,err);
	BST_HEALTH_MONITOR_DEBUG_PRINTK("send dtc %u.",dtc);
out:
	kfree(wk);
}

/*
 * send_dtc_to_safety_svc: send dtc ipc msg to R5,
 * this function is called by driver.
 * @dtc: the DTC value.
 * @return 0 if it successes to send ipc msg.
 */

int send_dtc_to_safety_svc(u32 dtc)
{
	struct hm_dtc_svc_wk *work;
	int ret = 0;
	if(!m_client){
		BST_HEALTH_MONITOR_ERR_PRINTK("healthmonitor_client is not ready !!");
		return -78;
	}

	ret = kfifo_is_full(&dtc_fifo);
	if (ret) {
		BST_HEALTH_MONITOR_ERR_PRINTK("kfifo_is_full, dtc(0x%x)", dtc);
		kfifo_skip(&dtc_fifo);
		goto out_queue;
	}

	ret = kfifo_in(&dtc_fifo, &dtc, sizeof(dtc));
	if (ret != sizeof(dtc))
		return ret;
out_queue:
	work = kzalloc(sizeof(*work), GFP_KERNEL);
	if (unlikely(!work))
		return -ENOMEM;
	INIT_WORK(&work->dtc_work, dtc_wq_handler);
	queue_work(dtc_wq, &work->dtc_work);

	return 0;
}
EXPORT_SYMBOL(send_dtc_to_safety_svc);

/*
 * get_psmid_from_safety_lib: send block_id to safetylib get psm status,
 * this function is called by driver.
 * @block_id_in: the block_id value.
 * @block_id_out: get block_id form safetylib .
 * @psm_id_out: get psm id form safetylib.
 * @return 0 if it successes to get psm status.
 */
int get_psmid_from_safety_lib(uint8_t block_id_in ,uint8_t *block_id_out, uint32_t *psm_id_out){
	int ret = -1;
	int i = 0;
	uint8_t blockid = 0;
    healthmonitor_UInt32Array4_t *psm_id=NULL;
	healthmonitor_ErrorEnum_t err=0;

	if(!m_client)
		return ret;

	ret = m_client->healthmonitor_client.fusaenable_method_sync(block_id_in,&blockid,&psm_id,&err,5000,NULL);
    if(ret < 0|| err != 0){
        BST_HEALTH_MONITOR_ERR_PRINTK("%s,%d,get psmid from safety lib fail, ret is %d,err = %d.", __func__, __LINE__,ret,(int)err);
        return -2;
    }
	*block_id_out = blockid;
	if(psm_id != NULL){
		for(i=0;i<4;i++)
			psm_id_out[i]=(*psm_id)[i];
	}
    pr_debug("%s,%d,get psmid from safety lib block=%d,result =%x %x %x %x.\n", __func__, __LINE__,*block_id_out, 
																									psm_id_out[0],
																									psm_id_out[1],
																									psm_id_out[2],
																									psm_id_out[3]);

	return 0;
}
EXPORT_SYMBOL(get_psmid_from_safety_lib);

static int32_t health_monitor_psmid_ioctl(struct file *filp,
				       struct psm_msg __user *p)
{
	struct psm_msg _msg;
	int ret = -1;

	if (copy_from_user(&_msg, p, sizeof(*p))) {
		BST_HEALTH_MONITOR_ERR_PRINTK("health monitor copy_from_user error");
		return ret;
	}

	ret = get_psmid_from_safety_lib(_msg.block_id_in,&(_msg.block_id_out),_msg.psm_id_out);
	if(ret < 0)
		return ret;

	if (copy_to_user(p, &_msg, sizeof(*p)) == 0)
		return 0;

	return -4;
}

static long health_monitor_ioctl(struct file *f,
				 unsigned int cmd, unsigned long args)
{
	long errno = 0;
	u32 dtc;

	if (_IOC_TYPE(cmd) != HEALTH_MONITOR_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HEALTH_MONITOR_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case HEALTH_MONITOR_REPORT_DTC:
		errno = copy_from_user(&dtc, (void __user *)args,
				       _IOC_SIZE(cmd));
		if (errno == 0)
			send_dtc_to_safety_svc(dtc);
		break;
	case HEALTH_MONITOR_GET_PSMID:
		errno = health_monitor_psmid_ioctl(f,
					      (struct psm_msg __user *)args);
		break;
	default:
		errno = -EINVAL;
	}

	return errno;
}

static struct proc_ops ops = {
	.proc_ioctl   = health_monitor_ioctl,
};

static void on_dst_changed(bool flag, void *ext)
{
	if (flag)
		BST_HEALTH_MONITOR_LOG_PRINTK("Healthmonitor dst is online.");
	else
		BST_HEALTH_MONITOR_LOG_PRINTK("Healthmonitor dst is offline.");

	*((bool *)ext) = flag;
}

static int init_sessions(void)
{
	int ret = -1;
	ipc_inf_version_t version = {0};
	bool dst_avail = false;

	// m_data.com_data.pid = CONFIG_HEALTH_MONITOR_PID;
	// m_data.com_data.sid = CONFIG_HEALTH_MONITOR_SID;
	// m_data.com_data.fid = CONFIG_HEALTH_MONITOR_FID;

// #ifdef CONFIG_SECOND_KERNEL
	m_data.com_data.pid = msgbx_get_start_pid();
	m_data.com_data.fid = CONFIG_HEALTH_MONITOR_FID;
	m_data.com_data.sid = CONFIG_HEALTH_MONITOR_SID;
// #endif
	m_client = HealthmonitorClient_init(&m_data);
	if (!m_client) {
		BST_HEALTH_MONITOR_ERR_PRINTK("HealthmonitorClient init client fail.");
		return ret;
	}

	// get version
	version = m_client->healthmonitor_client.version();
	BST_HEALTH_MONITOR_LOG_PRINTK("HealthmonitorClient interface version : major %d, minor %d.\n", version.major, version.minor);
	// start test_client_t
	ret = m_client->start();
	if (ret < 0) {
		return ret;
	}

	ret = m_client->healthmonitor_client.register_avail_changed(on_dst_changed, &dst_avail);

	BST_HEALTH_MONITOR_LOG_PRINTK("healthmonitor-client msgbox start.");	

	return 0;
}

static void destroy_sessions(void)
{
	int ret = 0;
	if (!m_client) 
		return;
    // stop to receive msg, must
    ret = m_client->stop();
    if (ret < 0)
    {
        BST_HEALTH_MONITOR_ERR_PRINTK("healthmonitor_client stop failed,ret = %d !",ret);
		return;
    }
    ret = HealthmonitorClient_destroy();
	if (ret)
		BST_HEALTH_MONITOR_ERR_PRINTK("healthmonitor_client destroy failed,ret = %d !",ret);
    return ; 
}

static int health_monitor_init(void)
{
	int ret;
	
	ret = init_sessions();
	if (ret)
		goto exit_session;
		
	health_monitor_proc = proc_create("health_monitor",
					  0644, NULL, &ops);
	if (!health_monitor_proc) {
		ret = -EINVAL;
		// goto exit_proc;
	}

	ret = kfifo_alloc(&dtc_fifo,
			  sizeof(u32) * fifo_depth,
			  GFP_KERNEL);
	if (ret)
		goto exit_session;

	dtc_wq = alloc_workqueue("hm_dtc_wq", WQ_MEM_RECLAIM | WQ_UNBOUND, 0);
	if (unlikely(!dtc_wq)) {
		kfifo_free(&dtc_fifo);
		ret = -ENOMEM;
		goto exit_session;
	}

	return 0;

exit_session:
	destroy_sessions();
	return ret;
}

static void health_monitor_exit(void)
{
	destroy_workqueue(dtc_wq);
	kfifo_free(&dtc_fifo);

	destroy_sessions();
}

module_init(health_monitor_init);
module_exit(health_monitor_exit);

MODULE_DESCRIPTION("BST-C1200 health monitor device driver");
MODULE_LICENSE("GPL v2");
