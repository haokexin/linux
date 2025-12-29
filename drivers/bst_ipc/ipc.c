// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <asm/memory.h>
#include <linux/mman.h>

#include <bst/ipc_interface.h>

#include "ipc.h"
#include "ipc_mempool.h"
#include "ipc_session.h"
#include "ipc_msg_manager.h"
#include "ipc_mailbox_controller.h"
#include "ipc_nodemanager.h"
#include "user_head.h"
#include "ipc_mempool.h"
#include "ipc_common.h"

/********************* macros *******************/
#define IPC_INDEX_SIZE	50
#define IPC_DRIVER_NAME "ipc"

/********************* local variables ***************************/
static struct platform_device *g_ipc_main_pdev;

/********************* function declaration ***************************/
static int32_t ipc_mempool_init(struct bstipc *bstipc)
{
	int32_t ret = 0;

	ret = of_reserved_mem_device_init(bstipc->dev);
	if (ret < 0) {
		IPC_LOG_ERR("of_reserved_mem_device_init fail, ret: %d", ret);
		return -ENODEV;
	}
	return ipc_init_cma_mempool(&bstipc->pool, bstipc->dev);
}

// ipc memory alloc, this function is deprecated now
static int32_t ipc_ioctl_alloc(struct file *filp, struct ipc_buffer __user *p)
{
	return 0;
}

// ipc memory free, this function is deprecated now
static int32_t ipc_ioctl_free(struct file *filp, struct ipc_buffer __user *p)
{
	return 0;
}

// register method function
static int32_t ipc_ioctl_register_info(struct file *filp,
				       struct msg_subscribe __user *p)
{
	int32_t ret;
	struct msg_subscribe info;

	// copy register info
	if (copy_from_user(&info, p, sizeof(*p))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("register type = %d, session_id = %d, cmd= %d", info.type,
		     info.session_id, info.cmd);

	if (info.type == IPC_MSG_TYPE_METHOD) {
		ret = ipc_method_register(info.session_id, info.cmd);
		if (ret < 0) {
			IPC_LOG_WARNING("session %d register %d method fail",
					info.session_id, info.cmd);
			return -1;
		}
	} else if (info.type == IPC_MSG_TYPE_SIGNAL) {
		ret = ipc_signal_subscribe(info.session_id, info.cmd);
		if (ret < 0) {
			IPC_LOG_WARNING("session %d subscribe %d signal fail",
					info.session_id, info.cmd);
			return -1;
		}
	} else {
		IPC_LOG_WARNING("session %d register type %d fail",
				info.session_id, info.type);
	}

	return 0;
}

static int32_t ipc_ioctl_session_create(struct file *filp,
					struct _session_init *init_msg)
{
	struct _session_init init;
	int32_t session_id = -1;

	if (copy_from_user(&init, init_msg, sizeof(*init_msg))) {
		IPC_LOG_ERR("copy_to_user fail!");
		return -EFAULT;
	}
	IPC_LOG_INFO("init src = %d, dst = %d", init.src, init.dst);

	if((init.dst<IPC_CORE_MAX)&&(init.src<IPC_CORE_MAX))
		session_id = ipc_init(init.dst, init.src, NULL);
	
	IPC_LOG_INFO("session id %d", session_id);

	if (session_id < 0) {
		IPC_LOG_WARNING("ipc_init(%d) failed, ret = %d", init.dst,
				session_id);
	}
	return session_id;
}

static int32_t ipc_ioctl_session_destroy(struct file *filp,
					 uint64_t __user *session_id)
{
	int32_t ret = 0;
	uint64_t id;

	if (copy_from_user(&id, session_id, sizeof(*session_id))) {
		IPC_LOG_ERR("copy_to_user fail!");
		return -EFAULT;
	}

	ret = ipc_close(id);
	if (ret < 0)
		IPC_LOG_WARNING("close session %lld fail!", id);

	return ret;
}

static int32_t ipc_ioctl_send_msg_func(struct file *filp,
				       struct send_or_recv_msg __user *p)
{
	int32_t ret;
	struct send_or_recv_msg _msg;
	ipc_msg send_msg;
	bool need_reply;

	if (copy_from_user(&_msg, p, sizeof(*p))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

	IPC_LOG_INFO("session_id = %d, data= %x, cmd= %d, type= %d ",
		     _msg.session_id, _msg.msg.data, _msg.msg.cmd,
		     _msg.msg.type);

	if (!ipc_session_valid(_msg.session_id)) {
		IPC_LOG_WARNING("session is invalid!");
		return -1;
	}

	send_msg.type = _msg.msg.type;
	send_msg.cmd = _msg.msg.cmd;
	send_msg.data = _msg.msg.data;
	send_msg.token = _msg.msg.token;
#ifdef MSG_SIZE_EXTENSION
	send_msg.long_data = _msg.msg.long_data;
#endif
	need_reply = _msg.reply_flag;

	IPC_LOG_INFO("send msg need_reply_flag is %d", need_reply);

	ret = ipc_send(_msg.session_id, &send_msg, _msg.timeout, need_reply);
	if (ret < 0) {
		IPC_LOG_ERR("ipc_send session %d failed, ret = %d",
			    _msg.session_id, ret);
		return ret;
	}
	_msg.msg.token = send_msg.token;
	if (copy_to_user(p, &_msg, sizeof(*p)) == 0)
		return 0;

	return -1;
}

static int32_t ipc_ioctl_recv_msg_func(struct file *filp,
				       struct send_or_recv_msg __user *p)
{
	int32_t ret;
	struct send_or_recv_msg _msg;
	ipc_msg recv_msg;

	if (copy_from_user(&_msg, p, sizeof(*p))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d, timeout = %d", _msg.session_id,
		     _msg.timeout);

	if (!ipc_session_valid(_msg.session_id)) {
		IPC_LOG_INFO("session is invalid!");
		return -1;
	}

	recv_msg.type = _msg.msg.type;
	ret = ipc_recv(_msg.session_id, &recv_msg, _msg.timeout);
	if (ret < 0) {
		IPC_LOG_INFO("ipc recv session %d fail, ret = %d",
			     _msg.session_id, ret);
		return ret;
	}

	_msg.msg.type = recv_msg.type;
	_msg.msg.cmd = recv_msg.cmd;
	_msg.msg.data = recv_msg.data;
	_msg.msg.token = recv_msg.token;
#ifdef MSG_SIZE_EXTENSION
	_msg.msg.long_data = recv_msg.long_data;
#endif

	IPC_LOG_INFO("recv data= %x, cmd= %d, type= %d", _msg.msg.data,
		     _msg.msg.cmd, _msg.msg.type);

	if (copy_to_user(p, &_msg, sizeof(*p)) == 0)
		return 0;

	return -1;
}

static int32_t ipc_ioctl_get_payload_addr(struct file *filp,
					  struct session_payload_info __user *p)
{
	struct session_payload_info info;
	struct ipc_session *session;
	uint64_t uaddr;
	uint64_t tmp;

	struct bstipc *bstipc =
		container_of(filp->private_data, struct bstipc, miscdev);

	if (copy_from_user(&info, p, sizeof(*p))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}
	IPC_LOG_INFO("session_id = %d", info.session_id);
	session = get_session_by_id(info.session_id);
	if (IS_ERR_OR_NULL(session)) {
		IPC_LOG_WARNING("get payload session %d is NULL",
				info.session_id);
		return -1;
	}

	uaddr = vm_mmap(filp, 0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED,
			0x80ff00000ULL);
	if (!uaddr || IS_ERR_VALUE(uaddr)) {
		bstipc->pool->ops->free(g_ipc_memblock);
		IPC_LOG_ERR("vm_mmap fail! ret: %lld", uaddr);
		return -EFAULT;
	}

	tmp = (uint64_t) &
	      (g_ipc_all_cores_register_addr_uaddr->addr[session->src].payload);
	info.send_addr = uaddr + tmp;
	tmp = (uint64_t) &
	      (g_ipc_all_cores_register_addr_uaddr->addr[session->dest].payload);
	info.recv_addr = uaddr + tmp;

	IPC_LOG_INFO("send addr is %llx, recv addr is %llx", info.send_addr,
		     info.recv_addr);

	if (copy_to_user(p, &info, sizeof(*p)) == 0)
		return 0;

	return -1;
}

static int32_t ipc_ioctl_get_info(struct file *filp, uint32_t __user *core_id)
{
	struct bstipc *bstipc = NULL;
	uint32_t coid;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!");
		return -EFAULT;
	}

	if (copy_from_user(&coid, core_id, sizeof(*core_id))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

	// coding print info
#ifdef MSG_DUMP
	struct ipc_fill_register_msg save_msg;
	int32_t cnt = save_cnt[coid];
	int32_t i;

	IPC_INFO_PRINT("       %-10s  %-6s %-3s %-2s wakeup type", "data",
		       "token", "cmd", "ack");
	for (i = 0; i < cnt; i++) {
		if (i >= IPC_INFO_MAX_NUM)
			break;
		save_msg = *((struct ipc_fill_register_msg *)&(
			g_ipc_all_cores_register_addr
				->data_saved[coid][i % IPC_INFO_MAX_NUM]));
		IPC_INFO_PRINT("%-5d  %#010x  %-6d %-3d %-2d  %-2d     %-2d", i,
			       save_msg.long_param, save_msg.short_param,
			       save_msg.cmd, save_msg.ack, save_msg.wakeup,
			       save_msg.type);
	}
#endif

	return 0;
}

static int32_t ipc_ioctl_cfg_info_enable(struct file *filp,
					 uint32_t __user *core_id)
{
	struct bstipc *bstipc = NULL;
	uint32_t coid;

	bstipc = container_of(filp->private_data, struct bstipc, miscdev);
	if (!bstipc) {
		IPC_LOG_ERR("can not find bstipc dev!");
		return -EFAULT;
	}

	if (copy_from_user(&coid, core_id, sizeof(*core_id))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

#ifdef MSG_DUMP
	save_flag[coid] = true;
#endif

	return 0;
}

static int32_t ipc_ioctl_cfg_info_disable(struct file *filp,
					  uint32_t __user *core_id)
{
	uint32_t coid;

	if (copy_from_user(&coid, core_id, sizeof(*core_id))) {
		IPC_LOG_ERR("copy_from_user error");
		return -EFAULT;
	}

#ifdef MSG_DUMP
	save_flag[coid] = false;
	int32_t cnt = save_cnt[coid];
	int32_t i;

	for (i = 0; i < cnt; i++) {
		if (i >= IPC_INFO_MAX_NUM)
			break;
		g_ipc_all_cores_register_addr
			->data_saved[coid][i % IPC_INFO_MAX_NUM] = 0;
	}
	save_cnt[coid] = 0;
#endif

	return 0;
}

static int32_t ipc_ioctl_get_sys_info(struct file *filp, void *arg)
{
	int i;
	struct diag_info te = diagnose_info[0];

	IPC_INFO_PRINT(
		"                                    |                 %-29s|                   %-30s",
		"send_error", "recv_error");
	IPC_INFO_PRINT(
		"%-11s%-4s%-4s%-9s%-9s%-13s%-16s%-7s%-11s%-13s%-16s%-8s%-12s",
		"session_id", "src", "dst", "send_msg", "recv_msg",
		"ipc_no_ready", "session_invalid", "No_ACK", "queue_full",
		"ipc_no_ready", "session_invalid", "timeout", "queue_empty");

	for (i = 1; i < SESSION_NUM; i++) {
		if (diagnose_info[i].session_id > 0 &&
		    diagnose_info[i].session_id < SESSION_NUM) {
			te = diagnose_info[i];
			IPC_INFO_PRINT(
				"%-11d%-4d%-4d%-9d%-9d%-13d%-16d%-7d%-11d%-13d%-16d%-8d%-12d",
				te.session_id, te.src, te.dst,
				te.num_of_send_msg, te.num_of_recv_msg,
				te.send_err.ipc_no_ready,
				te.send_err.session_invalid, te.send_err.no_ACK,
				te.send_err.queue_full,
				te.recv_err.ipc_no_ready,
				te.recv_err.session_invalid,
				te.recv_err.timeout, te.recv_err.queue_empty);
		}
	}
	return 0;
}

static long ipc_ioctl(struct file *filp, uint32_t cmd, unsigned long arg)
{
	int32_t ret = 0;

	IPC_LOG_INFO("%s, cmd: %d", __func__, cmd);
	switch (cmd) {
	case IPC_IO_MEM_ALLOC:
		// function is deprecated
		ret = ipc_ioctl_alloc(filp, (struct ipc_buffer __user *)arg);
		break;
	case IPC_IO_MEM_FREE:
		// function is deprecated
		ret = ipc_ioctl_free(filp, (struct ipc_buffer __user *)arg);
		break;
	case IPC_IO_SES_CREATE:
		ret = ipc_ioctl_session_create(filp,
					       (struct _session_init *)arg);
		break;
	case IPC_IO_SES_DESTROY:
		ret = ipc_ioctl_session_destroy(filp, (uint64_t __user *)arg);
		break;
	case IPC_IO_REGISTER_INFO:
		ret = ipc_ioctl_register_info(filp,
					      (struct msg_subscribe *)arg);
		break;
	case IPC_IO_MSG_SEND:
		ret = ipc_ioctl_send_msg_func(filp,
					      (struct send_or_recv_msg *)arg);
		break;
	case IPC_IO_MSG_RECV:
		ret = ipc_ioctl_recv_msg_func(filp,
					      (struct send_or_recv_msg *)arg);
		break;
	case IPC_IO_GET_PAYLOAD_ADDR:
		ret = ipc_ioctl_get_payload_addr(
			filp, (struct session_payload_info __user *)arg);
		break;
	case IPC_IO_GET_INFO:
		ret = ipc_ioctl_get_info(filp, (uint32_t __user *)arg);
		break;
	case IPC_IO_CFG_INFO_ENABLE:
		ret = ipc_ioctl_cfg_info_enable(filp, (uint32_t __user *)arg);
		break;
	case IPC_IO_CFG_INFO_DISABLE:
		ret = ipc_ioctl_cfg_info_disable(filp, (uint32_t __user *)arg);
		break;
	case IPC_IO_GET_SYS_INFO:
		ret = ipc_ioctl_get_sys_info(filp, (void *)arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	IPC_LOG_INFO("%s exit, ret: %d", __func__, ret);
	return ret;
}

// mmap function is deprecated
static int32_t ipc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return 0;
}

static int32_t ipc_open(struct inode *inode, struct file *filp)
{
	struct bstipc *bstipc = platform_get_drvdata(g_ipc_main_pdev);

	bstipc->private_data = filp;

	return 0;
}

// close function is deprecated
static int32_t ipc_drv_close(struct inode *inode, struct file *filp)
{
	return 0;
}

// fasync function is deprecated
static int32_t ipc_fasync(int32_t fd, struct file *file, int32_t on)
{
	return 0;
}

// read function is deprecated
static ssize_t ipc_read(struct file *filp, char __user *buf, size_t count,
			loff_t *ppos)
{
	return 0;
}

static const struct file_operations ipc_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = ipc_ioctl,
	.read = ipc_read,
	.mmap = ipc_mmap,
	.open = ipc_open,
	.release = ipc_drv_close,
	.fasync = ipc_fasync,
};

static const struct miscdevice ipc_misc_base = {
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &ipc_fops,
};

static int32_t ipc_probe(struct platform_device *pdev)
{
	int32_t ret = 0;
	char dev_name[sizeof(IPC_DRIVER_NAME) + IPC_INDEX_SIZE];
	struct bstipc *bstipc = NULL;

	g_ipc_main_pdev = pdev;

	memset(dev_name, 0, sizeof(IPC_DRIVER_NAME) + IPC_INDEX_SIZE);

	// create bstipc device
	bstipc = devm_kzalloc(&pdev->dev, sizeof(struct bstipc), GFP_KERNEL);
	if (!bstipc) {
		IPC_LOG_ERR("no enough memory!");
		return -ENOMEM;
	}

	// init bstipc device
	bstipc->dev = &pdev->dev;
	platform_set_drvdata(pdev, bstipc);

	// init ipc share mempool
	ret = dma_set_coherent_mask(bstipc->dev, DMA_BIT_MASK(36));
	if (ret < 0)
		IPC_LOG_ERR("dma_set_coherent_mask fail, ret %d", ret);

	ret = ipc_mempool_init(bstipc);
	IPC_LOG_INFO("ipc mempool init result: %d", ret);
	if (ret < 0) {
		IPC_LOG_ERR("ipc_mempool_init fail, ret %d", ret);
		return ret;
	}

	// init ipc miscdev
	sprintf(dev_name, "bstipc%ud", bstipc->id);

	IPC_LOG_INFO("dev_name: %s", dev_name);

	memcpy(&bstipc->miscdev, &ipc_misc_base, sizeof(struct miscdevice));
	bstipc->miscdev.name = devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);
	bstipc->miscdev.nodename =
		devm_kstrdup(&pdev->dev, dev_name, GFP_KERNEL);

	// register miscdev
	ret = misc_register(&bstipc->miscdev);
	if (ret < 0)
		goto err_destory_pool;

	IPC_LOG_INFO("device %s is registered.", dev_name);

	return ret;

err_destory_pool:
	ipc_destroy_cma_mempool(bstipc->pool);
	return ret;
}

static int32_t ipc_remove(struct platform_device *pdev)
{
	int32_t ret;
	struct bstipc *bstipc = platform_get_drvdata(pdev);

	misc_deregister(&bstipc->miscdev);
	ret = ipc_session_destroy_by_pid(current->pid);

	if (ret < 0) {
		IPC_LOG_ERR("ipc_session_destroy fail, ret %d", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id ipc_of_match[] = {
	{.compatible = "bst,bst-ipc",},
	{},
};

static struct platform_driver ipc_driver = {
	.probe   = ipc_probe,
	.remove  = ipc_remove,
	.driver  = {
		.name = IPC_DRIVER_NAME,
		.of_match_table = of_match_ptr(ipc_of_match),
	},
};

static int32_t __init ipc_driver_init(void)
{
	return platform_driver_register(&ipc_driver);
}

device_initcall_sync(ipc_driver_init);
