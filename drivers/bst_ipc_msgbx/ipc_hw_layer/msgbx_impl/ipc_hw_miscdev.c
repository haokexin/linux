// SPDX-License-Identifier: (GPL-2.0 OR MIT)

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/dma-direct.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/hashtable.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>
#include <asm/mman.h>
#include <asm/cacheflush.h>
#include <linux/fs.h>
//#include "<linux/kstrtox.h>"

#include <bst/ipc_hw_layer.h>
#include <bst/ipc_hw_impl.h>
#include <bst/ipc_trans_layer.h>

#include "../../ipc_trans_layer/src/ipc_trans_runtime.h"

#include "ipc_hw_miscdev.h"
//device id cannot be greater or equal to 10
#define IPC_MSGBX_DEV_ID_LEN           2
// this dev name such as /dev/ipc_msgbox0
#define IPC_MSGBX_MISC_DEV_LEN (20)
#define MAX_DATA_SIZE (3)
static struct miscdevice miscdev;
static spinlock_t msgbx_lock;
static LIST_HEAD(msgbox_list);

uint8_t bstn_print_log_flag = 0;
EXPORT_SYMBOL(bstn_print_log_flag);
uint64_t bstn_max_recv_time = 0;
EXPORT_SYMBOL(bstn_max_recv_time);

static int32_t ipc_msgbx_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct handle_list *mlist;
	struct list_head *list, *n, *head = &msgbox_list;
	unsigned long size = vma->vm_end - vma->vm_start;
	void* addr = NULL;

	spin_lock(&msgbx_lock);
	list_for_each_safe(list, n, head) {
		mlist = list_entry(list, struct handle_list, list);
		if (unlikely(!mlist))
			continue;

		if (strlen(mlist->comm) > 0 && !strcmp(mlist->comm, current->comm) && !(mlist->mapped_flag)) {
			mlist->mapped_flag = 1;
			addr = mlist->map_addr;
		}
	}
	spin_unlock(&msgbx_lock);
	if (addr)
		return remap_pfn_range(vma, vma->vm_start, virt_to_phys(addr) >> PAGE_SHIFT, size, vma->vm_page_prot);
	else
		return -EAGAIN; 
}

static int32_t ipc_msgbx_open(struct inode *inode, struct file *filp)
{
	return simple_open(inode, filp);
}

static int32_t ipc_msgbx_close(struct inode *inode, struct file *filp)
{
	struct handle_list *mlist;
	struct list_head *list, *n, *head = &msgbox_list;
	spin_lock(&msgbx_lock);
	list_for_each_safe(list, n, head) {
		mlist = list_entry(list, struct handle_list, list);
		if (unlikely(!mlist))
			continue;

		if (strlen(mlist->comm) > 0 && !strcmp(mlist->comm, current->comm) && (mlist->pid == current->tgid) && !(mlist->close_flag)) {
			ipc_trans_layer_unregister_method(mlist->endid, mlist->handle);
			ipc_trans_layer_destroy_handle(mlist->endid, mlist->handle);
			list_del(list);
			kfree(mlist);
		}
	}
	spin_unlock(&msgbx_lock);
	return 0;
}

// fasync function is deprecated
static int32_t ipc_msgbx_fasync(int32_t fd, struct file *file, int32_t on)
{
	return 0;
}

static void msgbx_stats_print(struct msgbox_statis *stats)
{
	u32 cpu = 0, endid = 0;
	u64 rx_stats = 0, tx_stats = 0;
	u64 rx_overflow_stats = 0, rx_underflow_stats = 0, rx_thrs_stats = 0;
	u64 rx_stats_cpu = 0, tx_stats_cpu = 0;
	u64 rx_pid_stats_cpu = 0, tx_cid_stats_cpu = 0;
	u64 tx_fifo_unavail_stats = 0, tx_overflow_stats = 0;

	pr_info("\n");
	for_each_online_cpu(cpu) {
		tx_fifo_unavail_stats += stats->tx_fifo_unavail_stats[cpu];
		tx_overflow_stats += stats->tx_overflow_stats[cpu];
		rx_thrs_stats += stats->rx_thrs_stats[cpu];
		rx_overflow_stats += stats->rx_overflow_stats[cpu];
		rx_underflow_stats += stats->rx_underflow_stats[cpu];
		rx_stats_cpu = 0;
		tx_stats_cpu = 0;
		for (endid = 0; endid < MAX_END_NUM; endid++) {
			rx_stats_cpu += stats->rx_stats[cpu][endid];
			tx_stats_cpu += stats->tx_stats[cpu][endid];
		}
		rx_stats += rx_stats_cpu;
		tx_stats += tx_stats_cpu;
		pr_info("statistic on cpu%u :\n", cpu);
		pr_info("rx_thrs_stats : %llu\n", stats->rx_thrs_stats[cpu]);
		pr_info("rx_overflow_stats : %llu\n", stats->rx_overflow_stats[cpu]);
		pr_info("rx_underflow_stats : %llu\n", stats->rx_underflow_stats[cpu]);
		pr_info("tx_overflow_stats : %llu\n", stats->tx_overflow_stats[cpu]);
		pr_info("tx_fifo_unavail_stats : %llu\n", stats->tx_fifo_unavail_stats[cpu]);
		pr_info("tx_stats : %llu\n", tx_stats_cpu);
		pr_info("rx_stats : %llu\n", rx_stats_cpu);
	}

#if defined(MSGBX_HW_TYPE_C1200)
	for (endid = 0x10; endid < MAX_END_ID; endid++) {
		for_each_online_cpu(cpu) {
			rx_pid_stats_cpu += stats->rx_pidcid_stats[cpu][endid];
		}
		if (rx_pid_stats_cpu)
			pr_info(" rx msg from end%u : %llu\n", endid, rx_pid_stats_cpu);
		rx_pid_stats_cpu = 0;
	}

	for (endid = 0x10; endid < MAX_END_ID; endid++) {
		for_each_online_cpu(cpu) {
			tx_cid_stats_cpu += stats->tx_pidcid_stats[cpu][endid];
		}
		if (tx_cid_stats_cpu)
			pr_info(" tx msg to end%u : %llu\n", endid, tx_cid_stats_cpu);
		tx_cid_stats_cpu = 0;
	}
#elif defined(MSGBX_HW_TYPE_A2000)
	for (endid = 0; endid < MAX_END_ID; endid++) {
		for_each_online_cpu(cpu) {
			rx_pid_stats_cpu += stats->rx_pidcid_stats[cpu][endid];
		}
		if (rx_pid_stats_cpu)
			pr_info(" rx msg from end%u : %llu\n", endid, rx_pid_stats_cpu);
		rx_pid_stats_cpu = 0;
	}

	for (endid = 0; endid < MAX_END_ID; endid++) {
		for_each_online_cpu(cpu) {
			tx_cid_stats_cpu += stats->tx_pidcid_stats[cpu][endid];
		}
		if (tx_cid_stats_cpu)
			pr_info(" tx msg to end%u : %llu\n", endid, tx_cid_stats_cpu);
		tx_cid_stats_cpu = 0;
	}
#endif
	pr_info("total statistic :\n");
	pr_info("rx_thrs_stats : %llu\n", rx_thrs_stats);
	pr_info("rx_overflow_stats : %llu\n", rx_overflow_stats);
	pr_info("rx_underflow_stats : %llu\n", rx_underflow_stats);
	pr_info("tx_overflow_stats : %llu\n", tx_overflow_stats);
	pr_info("tx_fifo_unavail_stats : %llu\n", tx_fifo_unavail_stats);
	pr_info("tx_stats : %llu\n", tx_stats);
	pr_info("rx_stats : %llu\n", rx_stats);
	pr_info("\n");
}


// read function is deprecated
static ssize_t ipc_msgbx_read(struct file *filp, char __user *buf, size_t count,
			loff_t *ppos)
{
	struct msgbox_statis *stats = msgbx_get_statis();
#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
	u32 cpu;
	size_t to_copy = 0;
	ssize_t result = 0;
	int i = 0;
	uint8_t data_array[CONFIG_MSGBOX_SLT_OVERFLOW_MSG_BUF_SIZE] = {0};
#endif

	if (likely(stats))
		msgbx_stats_print(stats);

#if defined(CONFIG_C1200_SLT) || defined(CONFIG_C1200_MASS)
	// spec return overflow result
	if (*ppos >= CONFIG_MSGBOX_SLT_OVERFLOW_MSG_BUF_SIZE) {
		return 0;
	}

	// copy overflow data
	for_each_online_cpu(cpu) {
		data_array[i] = stats->rx_overflow_stats[cpu];
		i++;
	}

	to_copy = (count < (CONFIG_MSGBOX_SLT_OVERFLOW_MSG_BUF_SIZE - *ppos)) ? count : (CONFIG_MSGBOX_SLT_OVERFLOW_MSG_BUF_SIZE - *ppos);

	if (copy_to_user(buf, &data_array[*ppos], to_copy) == 0) {
		*ppos += to_copy;
		result = to_copy;
	} else
		result = -EFAULT;
	return result;
#endif
	return 0;
}

static void msgbx_end_session_print(uint8_t cpuid){
	uint8_t handle = 0;
	debug_info_t info = { 0 };
	uint8_t fid = 0,sid = 0;
	const char *param_names[5] = {"session", "fid", "sid", "cid", "role"};
	
    pr_info("%-8s%-8s%-8s%-8s%-8s\n", param_names[0], param_names[1], param_names[2], param_names[3], param_names[4]);

	pr_info("----------------------------------------\n");

	for(fid = 0;fid < CHANNEL_COUNT;fid++){
		for(sid = 0; sid < SESSION_COUNT; sid++){
			handle = sid << CHANNEL_COUNT_BITS | fid;
			

			memset(&info, 0, sizeof(debug_info_t));
			ipc_trans_get_debug_info(handle, &info, g_ipc_end_array[cpuid]);

			if(info.role != MSGBX_SES_ROLE_INVALID)
				pr_info("%-8u%-8u%-8u%-8u%-8u\n", handle, fid, sid, info.cid, info.role);		
		}
	}
}

static void msgbox_end_cmd_print(uint8_t cpuid) {
	uint8_t i = 0;
	msgbx_end_device_t *module = g_ipc_end_array[cpuid];
	pr_info("----------------------------------------\n");
	for(i = 0; i < CMD_MAX_COUNT;i++){
		if (module->method_register_map[i] != -1)
			pr_info("cmd: %-8u session id:%-8u", i, module->method_register_map[i]);
	}
}

static ssize_t ipc_msgbx_write(struct file *fp, const char __user *buf,
			       size_t count, loff_t *lf)
{
	struct msgbox_statis *stats = msgbx_get_statis();
	uint8_t endid, handle, cpuid;
	
	uint8_t data[MAX_DATA_SIZE];
	debug_info_t info = { 0 };
	uint64_t max_send_time = 0;

	if (likely(stats))
		memset(stats, 0, sizeof(struct msgbox_statis));

	if (count > MAX_DATA_SIZE)
		return -EINVAL;

	if (copy_from_user(data, buf, MAX_DATA_SIZE))
		return -EFAULT;

	endid = data[0];
	cpuid = ENDID_TO_CPUID(endid);
	if(count == MAX_DATA_SIZE){
		handle = data[2] << CHANNEL_COUNT_BITS | data[1];
		pr_info("query end %u, fid %u sid %u , handle %u info:\n",
			data[0], data[1], data[2], handle);

		ipc_trans_get_debug_info(handle, &info, g_ipc_end_array[cpuid]);

		pr_info("\nsession %u info cid: %u, role: %u, send_msg_cnt:%u, send_rw_msg_cnt:%u, "
			"send_fail_cnt:%u\n",handle, info.cid, info.role, info.send_msg_cnt, info.send_rw_msg_cnt,
			info.send_fail_cnt);
		pr_info("recv_msg_1_cnt:%u, recv_msg_2_cnt:%u, recv_rw_msg_cnt:%u in_rw_msg_cnt:%u\n", info.recv_msg_1_cnt,
			info.recv_msg_2_cnt, info.recv_rw_msg_cnt, info.in_rw_msg_cnt);
		pr_info("send_start_time: %llu, send_end_time:%llu\n", info.send_start_time, info.send_end_time);

		if ((handle == 80 && endid == CPU_7)) {
			bstn_print_log_flag ^= 1;
			max_send_time = msgbx_get_max_send_time(cpuid, handle);
			pr_info("bstn_print_log_flag %u , bstn_max_recv_time %llu us, bstn_max_send_time %llu us\n", bstn_print_log_flag, bstn_max_recv_time, max_send_time);
		}
	}else if(count == 1){
		pr_info("end %d used info:\n",endid);
		msgbx_end_session_print(cpuid);
		pr_info("end %u cmd used info:\n", endid);
		msgbox_end_cmd_print(cpuid);
	}else{}
	
	return count;
}



static int32_t ipc_ioctl_parse_arg(void *arg_new,
				   void __user *arg, unsigned int cmd)
{
	int32_t ret;

	if (unlikely(!arg_new))
		return -EINVAL;
	
	memset(arg_new, 0, _IOC_SIZE(cmd));

	ret = copy_from_user(arg_new, arg, _IOC_SIZE(cmd));
	if (unlikely(ret)) {
        memset(arg_new, 0, _IOC_SIZE(cmd));
        return ret;
    }

	return 0;
}

static int32_t ipc_ioctl_layer_start(unsigned int cmd,
					 void __user *arg)
{
	return 0;
}

static int32_t ipc_ioctl_layer_stop(unsigned int cmd, void __user *arg)
{
	return 0;
}

static int32_t ipc_ioctl_create_handle(unsigned int cmd, void __user *arg)
{
	struct handle_list *cur_list;
	int32_t ret = 0;
	struct handle_info_t handle = {};
	void* map_addr;

	ret = ipc_ioctl_parse_arg(&handle, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	switch (handle.role) {
	case MSGBX_SES_ROLE_CLIENT:
		ret = ipc_trans_layer_proxy_create_handle(handle.endid, handle.fid, handle.sid, handle.cid, 0, &handle.handle);
		break;
	case MSGBX_SES_ROLE_SERVER:
	ret = ipc_trans_layer_stub_create_handle(handle.endid, handle.fid, handle.sid, handle.cid, &handle.handle);
		break;
	case MSGBX_SES_ROLE_FASTPATH:
		ret = ipc_trans_layer_create_handle(handle.endid, handle.fid, handle.sid, handle.cid, &handle.handle);
		break;
	default:
		ret = -EINVAL;
	}

	if (unlikely(ret)) {
		IPC_LOG_DEBUG("layer can't create handle! ret = %d", ret);
		return ret < 0 ? ret : 0;
	}
	ret = ipc_trans_layer_mmap_session(handle.endid, handle.handle, (void**)&map_addr);
	if (ret < 0) {
		return ipc_trans_layer_destroy_handle(handle.endid, handle.handle);
	}
	cur_list = kzalloc(sizeof(*cur_list), GFP_KERNEL);
	if (unlikely(!cur_list))
		return -ENOMEM;

	cur_list->handle = handle.handle;
	cur_list->endid = handle.endid;
	cur_list->map_addr = map_addr;
	cur_list->mapped_flag = 0;
	cur_list->pid = current->tgid;
	cur_list->close_flag = 0;
	memcpy(cur_list->comm, current->comm, strlen(current->comm));
	spin_lock(&msgbx_lock);
	list_add(&cur_list->list, &msgbox_list);
	spin_unlock(&msgbx_lock);

	if (likely(!ret))
		ret = copy_to_user(arg, &handle, _IOC_SIZE(cmd));

	return ret < 0 ? ret : 0;
}

static int32_t ipc_ioctl_register_method(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct method_info_t method = {0};

	ret = ipc_ioctl_parse_arg(&method, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_register_method(method.endid, method.handle, method.cmd);
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't register method! ret = %d", ret);

	return ret;
}

static int32_t ipc_ioctl_unregister_method(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct handle_t info = {};

	ret = ipc_ioctl_parse_arg(&info, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_unregister_method(info.endid, info.handle);
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't unregister method! ret = %d", ret);

	return ret;
}

static int32_t ipc_ioctl_destory_handle(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct handle_info_t handle = {0};
	struct handle_list *mlist;
	struct list_head *list, *n, *head = &msgbox_list;

	ret = ipc_ioctl_parse_arg(&handle, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;
	ret = ipc_trans_layer_destroy_handle(handle.endid, handle.handle);
	spin_lock(&msgbx_lock);
	list_for_each_safe(list, n, head) {
		mlist = list_entry(list, struct handle_list, list);
		if (unlikely(!mlist))
			continue;

		if (strlen(mlist->comm) > 0 && !strcmp(mlist->comm, current->comm) && (mlist->pid == current->tgid)) {
			mlist->close_flag = 1;
		}
	}
	spin_unlock(&msgbx_lock);
	return ret;
}

static int32_t ipc_ioctl_send_msg(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct msg_t msg = {0};

	ret = ipc_ioctl_parse_arg(&msg, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	switch (msg.msg.header.typ) {
	case MSGBX_MSG_TYPE_REPLY:
		ret = ipc_trans_layer_stub_send_reply_msg(msg.endid, msg.handle, &msg.msg);
		break;
	case MSGBX_MSG_TYPE_BROADCAST:
		ret = ipc_trans_layer_stub_send_broadcast(msg.endid, msg.handle, &msg.msg);
		break;
	case MSGBX_MSG_TYPE_METHOD:
		ret = ipc_trans_layer_proxy_send_method(msg.endid, msg.handle, &msg.msg);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int32_t ipc_ioctl_get_msg(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct msg_t msg = {};

	ret = ipc_ioctl_parse_arg(&msg, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_get_msg(msg.endid, msg.handle, &msg.msg);
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't get msg! ret = %d", ret);

	if (likely(!ret))
		ret = copy_to_user(arg, &msg, _IOC_SIZE(cmd));

	return ret < 0 ? -EINVAL : 0;
}

static int32_t ipc_ioctl_query_msg(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct query_info_t info = {};

	ret = ipc_ioctl_parse_arg(&info, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

#ifdef ipc_trans_layer_query_msg
	ret = ipc_trans_layer_query_msg(info.end_id, info.handle);
#else
	if (info.polling_times > 10000)
		info.polling_times = 10000;
	ret = ipc_trans_layer_query_info(info.end_id, info.handle, info.polling_times, 1);
	
#endif
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't query info! ret = %d", ret);

	return ret;
}

static int32_t ipc_ioctl_user_send_msg(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct user_msg_t msg = {};

	ret = ipc_ioctl_parse_arg(&msg, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_send_msg(msg.end_id, msg.handle, &msg.msg);
	if (unlikely(ret < 0))
		IPC_LOG_DEBUG("layer can't send msg! ret = %d", ret);

	return ret;
}

static int32_t ipc_ioctl_user_get_msg(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct user_msg_t msg = {};

	ret = ipc_ioctl_parse_arg(&msg, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_get_rwmsg(msg.end_id, msg.handle, msg.timeout,
				      &msg.msg, &msg.timestamp);
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't get msg! ret = %d", ret);

	if (likely(!ret))
		ret = copy_to_user(arg, &msg, _IOC_SIZE(cmd));

	return ret < 0 ? -EINVAL : 0;
}

static int32_t ipc_ioctl_user_get_endmap(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct endmap_t endmap = {};

	ret = ipc_ioctl_parse_arg(&endmap, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_get_endmap(endmap.end_id, &endmap.endmap);
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't get endmap! ret = %d", ret);

	if (likely(!ret))
		ret = copy_to_user(arg, &endmap, _IOC_SIZE(cmd));

	return ret < 0 ? -EINVAL : 0;
}

static int32_t ipc_ioctl_rel_recv_wait(unsigned int cmd, void __user *arg)
{
	int32_t ret = 0;
	struct handle_t info = {};

	ret = ipc_ioctl_parse_arg(&info, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	ret = ipc_trans_layer_release_recv_wait(info.endid, info.handle);
	if (unlikely(ret))
		IPC_LOG_DEBUG("layer can't unregister method! ret = %d", ret);

	return ret < 0 ? ret : 0;
}

static void ipc_msgbox_set_reg(u32 reg, u32 val, u32 fid, struct hw_regs_cfg *cfg)
{
	void *flt_addr = msgbx_get_filter_base(fid);

	if (cfg->reg_type == REG_TYPE_RX_FLT_EN)
		val |= readl_relaxed(flt_addr + reg);

	writel_relaxed(val, flt_addr + reg);
	udelay(5);

	cfg->val_get = readl_relaxed(flt_addr + reg);
}

static void ipc_smp_set_regs(void *info)
{
	struct hw_regs_cfg *cfg = (struct hw_regs_cfg *)info;
	u32 reg, val = cfg->val_set, fid = cfg->fid;
	u32 pay_idx = 0;

	switch (cfg->reg_type) {
	case REG_TYPE_RX_FF_ADDR :
		reg = fid == 0 ? DEF_RXFIFO_ADDR : FILTER1_RXFIFO_ADDRR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_RX_FF_THRD :
		reg = fid == 0 ? DEF_DEFAULT_RXTHRS_CFGR : End_filter_Thrs_CFGR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_RX_IRQ_EN :
		reg = fid == 0 ? DEF_FLT_INTER_EN : FILTER1_EN_INTER;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_RX_FLT_EN :
		reg = fid == 0 ? DEF_FLT_MSG_PIDF_CFGR : End_filter1_RxFIFO_ADDRR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_PID_FLT :
		reg = fid == 0 ? DEF_FLT_MSG_PIDF_CFGR : End_filter1_MsgH_PIDF_CFGR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_LEN_FLT :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_LenF_CFGR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgH_LenF_CFGR);
			return;
		}
		reg = End_filter1_MsgH_LenF_CFGR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_MSGH_MASKH :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_ResF_MaskHR(0x%x) "
				"is useful for non-default filter\n",
				End_filter1_MsgH_ResF_MaskHR);
			return;
		}
		reg = End_filter1_MsgH_ResF_MaskHR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_MSGH_MINH :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_ResF_MinHR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgH_ResF_MinHR);
			return;
		}
		reg = End_filter1_MsgH_ResF_MinHR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_MSGH_MAXH :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_ResF_MaxHR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgH_ResF_MaxHR);
			return;
		}
		reg = End_filter1_MsgH_ResF_MaxHR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_MSGH_MASK :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_ResF_MaskR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgH_ResF_MaskR);
			return;
		}
		reg = End_filter1_MsgH_ResF_MaskR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_MSGH_MIN :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_ResF_MinR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgH_ResF_MinR);
			return;
		}
		reg = End_filter1_MsgH_ResF_MinR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_MSGH_MAX :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgH_ResF_MaxR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgH_ResF_MaxR);
			return;
		}
		reg = End_filter1_MsgH_ResF_MaxR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_FLT_CFGR :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_RxFIFO_CFGR(0x%x) "
				"is useful for non-default filter\n", End_filter1_RxFIFO_CFGR);
			return;
		}
		reg = End_filter1_RxFIFO_CFGR;
		ipc_msgbox_set_reg(reg, val, fid, cfg);
		break;
	case REG_TYPE_PAY_MASK :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgPx_MaskR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgP1_MaskR);
			return;
		}
		for_each_payloads(pay_idx) {
			if (pay_idx == 0)
				reg = End_filter1_MsgP1_MaskR;
			else if (pay_idx == 1)
				reg = End_filter1_MsgP2_MaskR;
			else if (pay_idx == 2)
				reg = End_filter1_MsgP3_MaskR;
			else if (pay_idx == 3)
				reg = End_filter1_MsgP4_MaskR;
			//low 32bits of payload mask
			ipc_msgbox_set_reg(reg, val, fid, cfg);
			//high 32bits of payload mask
			ipc_msgbox_set_reg(reg + 0x8, val, fid, cfg);
		}
		break;
	case REG_TYPE_PAY_MIN :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgPx_MinR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgP1_MinR);
			return;
		}
		for_each_payloads(pay_idx) {
			if (pay_idx == 0)
				reg = End_filter1_MsgP1_MinR;
			else if (pay_idx == 1)
				reg = End_filter1_MsgP2_MinR;
			else if (pay_idx == 2)
				reg = End_filter1_MsgP3_MinR;
			else if (pay_idx == 3)
				reg = End_filter1_MsgP4_MinR;
			//low 32bits of payload min
			ipc_msgbox_set_reg(reg, val, fid, cfg);
			//high 32bits of payload min
			ipc_msgbox_set_reg(reg + 0x8, val, fid, cfg);
		}
		break;
	case REG_TYPE_PAY_MAX :
		if (fid == 0) {
			IPC_LOG_ERR("End_filter1_MsgPx_MaxR(0x%x) "
				"is useful for non-default filter\n", End_filter1_MsgP1_MaxR);
			return;
		}
		for_each_payloads(pay_idx) {
			if (pay_idx == 0)
				reg = End_filter1_MsgP1_MaxR;
			else if (pay_idx == 1)
				reg = End_filter1_MsgP2_MaxR;
			else if (pay_idx == 2)
				reg = End_filter1_MsgP3_MaxR;
			else if (pay_idx == 3)
				reg = End_filter1_MsgP4_MaxR;
			//low 32bits of payload max
			ipc_msgbox_set_reg(reg, val, fid, cfg);
			//high 32bits of payload max
			ipc_msgbox_set_reg(reg + 0x8, val, fid, cfg);
		}
		break;
	}
}

static int32_t ipc_ioctl_set_regs(unsigned int cmd, void __user *arg)
{
	int ret = 0;
	int cpu;
	struct hw_regs_cfg cfg = {};

	ret = ipc_ioctl_parse_arg(&cfg, arg, cmd);
	if (unlikely(ret))
		return -EINVAL;

	cpu = ENDID_TO_CPUID(cfg.endid);
	if (cpu < 0)
		return -EINVAL;
	switch (cfg.reg_type) {
	case REG_TYPE_RX_FF_ADDR :
	case REG_TYPE_RX_FF_THRD :
	case REG_TYPE_RX_IRQ_EN :
	case REG_TYPE_PID_FLT :
	case REG_TYPE_LEN_FLT :
	case REG_TYPE_MSGH_MASKH :
	case REG_TYPE_MSGH_MINH :
	case REG_TYPE_MSGH_MAXH :
	case REG_TYPE_MSGH_MASK :
	case REG_TYPE_MSGH_MIN :
	case REG_TYPE_MSGH_MAX :
	case REG_TYPE_FLT_CFGR :
		smp_call_function_single(cpu, ipc_smp_set_regs, &cfg, 1);
		break;
	case REG_TYPE_PAY_MASK :
	case REG_TYPE_PAY_MIN :
	case REG_TYPE_PAY_MAX :
		for_each_online_cpu(cpu) {
			smp_call_function_single(cpu, ipc_smp_set_regs, &cfg, 1);
		}
		break;
	default:
		IPC_LOG_ERR("%s invalid reg type(%u)\n", __func__, cfg.reg_type);
		return -EINVAL;
	}

	ret = copy_to_user(arg, &cfg, _IOC_SIZE(cmd));

	return ret < 0 ? ret : 0;
}

static long ipc_msgbx_ioctl(struct file *filp, uint32_t cmd, unsigned long arg)
{
	int32_t ret = 0;

	switch (cmd) {
	case IPC_MSG_IO_LAYER_START:
		ret = ipc_ioctl_layer_start(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_LAYER_STOP:
		ret = ipc_ioctl_layer_stop(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_CREATE_HANDLE:
		ret = ipc_ioctl_create_handle(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_REGISTER_METHOD:
		ret = ipc_ioctl_register_method(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_UNREGISTER_METHOD:
		ret = ipc_ioctl_unregister_method(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_DESTORY_HANDLE:
		ret = ipc_ioctl_destory_handle(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_QUERY_MSG:
		ret = ipc_ioctl_query_msg(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_RELEASE_RECV_WAIT:
		ret = ipc_ioctl_rel_recv_wait(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_SEND_MSG:
		ret = ipc_ioctl_send_msg(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_GET_MSG:
		ret = ipc_ioctl_get_msg(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_SET_REGS:
		ret = ipc_ioctl_set_regs(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_USER_SEND_MSG:
		ret = ipc_ioctl_user_send_msg(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_USER_GET_MSG:
		ret = ipc_ioctl_user_get_msg(cmd, (void __user *)arg);
		break;
	case IPC_MSG_IO_USER_GET_ENDMAP:
		ret = ipc_ioctl_user_get_endmap(cmd, (void __user *)arg);
		break;	
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		IPC_LOG_DEBUG("ipc_ioctl exit, ret: %d", ret);
	return ret;
}

/*******************************************************************************
 * ipc_msgbx MISC Initialization
 ******************************************************************************/
static const struct file_operations ipc_msgbx_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = ipc_msgbx_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ipc_msgbx_ioctl,
#endif
	.release = ipc_msgbx_close,
	.read = ipc_msgbx_read,
	.write = ipc_msgbx_write,
	.mmap = ipc_msgbx_mmap,
	.open = ipc_msgbx_open,
	.fasync = ipc_msgbx_fasync,
};

/*
 * @func    ipc_msgbx_misc_init
 * @brief   This is the initialization function of the ipc_msgbx misc device.
 * @params  ipc_msgbx - the pointer to the ipc_msgbx device
 * @params  devid - the msgbox devices id also is cpu id
 * @return  0 - success
 *          error code - failure
 */
int ipc_msgbx_miscdev_init(void)
{
	int ret = 0;
	char ipc_msgbx_dev_name[sizeof(IPC_DRIVER_NAME) + IPC_MSGBX_DEV_ID_LEN];

	snprintf(ipc_msgbx_dev_name, sizeof(IPC_DRIVER_NAME) + IPC_MSGBX_DEV_ID_LEN,
		 "%s%01d", IPC_DRIVER_NAME, 0);
	IPC_LOG_DEBUG("-------create device name: %s\n", ipc_msgbx_dev_name);
	spin_lock_init(&msgbx_lock);

	// init & register ipc_msgbx miscdev
	miscdev.minor = MISC_DYNAMIC_MINOR;
	miscdev.fops = &ipc_msgbx_fops;
	miscdev.name = devm_kstrdup(&g_ipc_msgbx_pdev->dev, ipc_msgbx_dev_name, GFP_KERNEL);
	miscdev.nodename = devm_kstrdup(&g_ipc_msgbx_pdev->dev, ipc_msgbx_dev_name, GFP_KERNEL);

	ret = misc_register(&miscdev);
	if (ret < 0)
		IPC_LOG_ERR("register device name: %s error!", ipc_msgbx_dev_name);

	return ret;
}

/*
 * @func    ipc_msgbx_misc_exit
 * @brief   This is the exit function of the ipc_msgbx misc device.
 * @params  pipc_msgbx - the pointer to the ipc_msgbx device
 * @return  0 - success
 *          error code - failure
 */
void ipc_msgbx_miscdev_exit(void)
{
	misc_deregister(&miscdev);
}

#ifdef CONFIG_MSGBOX_MISCDEV_MOD
static int msgbox_misc_dev_init(void)
{
	return ipc_msgbx_miscdev_init();
}

static void msgbox_misc_dev_exit(void)
{
	ipc_msgbx_miscdev_exit();
}

module_init(msgbox_misc_dev_init);
module_exit(msgbox_misc_dev_exit);

MODULE_ALIAS("platform: msgbox-misc");
MODULE_DESCRIPTION("msgbox misc driver");
MODULE_LICENSE("GPL v2");
#endif
