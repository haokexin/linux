// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direct.h>
#include <asm/io.h>
#include <uapi/linux/sched/types.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include "linux/of_address.h"
#include "linux/types.h"
#include "bst_display_platform.h"
#include "bst_display_global_api.h"
#include "bst/ipc_interface.h"
#include "bst/ipc_common.h"
#include "bst_display_dc_cmdset.h"
#define EVENTS_PADDR_OFFSET        512
#define DDR_INTERLEAVING_ON
#define BST_DISPLAY_IPC_TIMEOUT    -1
#define IPC_CMD_SRV_DISPLAY        1
#define WAIT_CMD_TIMEOUT           (5000)

enum {
	DISP_KTHREAD_STATUS_NONE = 0,
	DISP_KTHREAD_STATUS_RECV_MSG,
	DISP_KTHREAD_STATUS_RECV_COMPLETE,
	DISP_KTHREAD_STATUS_COPY_MSG,
	DISP_KTHREAD_STATUS_COPY_COMPLETE,
};

enum {
	IPC_CMD_TYPE_NONE,
	IPC_CMD_TYPE_MSG,
	IPC_CMD_TYPE_EVTS,
};

struct fw_msg_events {
	disp_event_callback_t cb;
	void *ext;
	uint32_t subdev;
	uint8_t sub_status;
};

struct fw_msg_events_cmd {
	uint32_t subdev;
	uint32_t sub_status;
	struct bst_display_events_status evts;
};

struct bst_fw_ipc_device {
    struct device *dev;
    struct platform_device *pdev;
    int32_t msg_session_id;
    int32_t evt_session_id;
	uint32_t ipc_core_id;
    struct task_struct *kthread_recv;  // for display message receiving
    int msg_kthread_status;
    spinlock_t msg_shmem_lock;
    struct mutex ipc_tx_mutex; /* For IPC TX*/
    struct mutex msg_mutex;
	struct completion msg_completion;
    phys_addr_t msg_paddr;
    void __iomem *msg_vaddr;
    phys_addr_t evts_paddr;
    void __iomem *evts_vaddr;
	uint32_t shmem_size;
	bool inited;
    struct fw_msg_data msg;
	struct fw_msg_events display_events[BST_SUBDEV_MAX - 1];
};

static struct bst_fw_ipc_device *g_fw_ipc;

/**
 * @brief Convert a 64-bit physical address to a 32-bit physical address for r5.
 *
 * @param paddr        64-bit physical address.
 * @param interleaving Whether interleaving is on or off.
 * @return 32-bit physical address.
 */
static inline uint32_t _paddr_64_to_paddr_32(phys_addr_t paddr, bool interleaving)
{
	uint32_t paddr_32bit;

	if (interleaving) {
		if (paddr < 0xc40000000 && paddr >= 0xc00000000) {
			paddr_32bit = paddr - 0xb40000000;
		} else if (paddr < 0x840000000 && paddr >= 0x800000000) {
			paddr_32bit = paddr - 0x780000000;
		} else {
			paddr_32bit = 0;
		}
	} else {
		if (paddr < 0x880000000 && paddr >= 0x800000000) {
			paddr_32bit = paddr - 0x780000000;
		} else {
			paddr_32bit = 0;
		}
	}
	return paddr_32bit;
}


#ifdef DDR_INTERLEAVING_ON
#define paddr_64_to_paddr_32(addr) _paddr_64_to_paddr_32(addr, true)
#else
#define paddr_64_to_paddr_32(addr) _paddr_64_to_paddr_32(addr, false)
#endif

static int ipc_pid_map(const char* type)
{
	u32 value;

	if (!strcmp(type, "ARM0")) {
		value = IPC_CORE_ARM0;
	} else if(!strcmp(type, "ARM1")) {
		value = IPC_CORE_ARM1;
	} else if(!strcmp(type, "ARM2")) {
		value = IPC_CORE_ARM2;
	} else if(!strcmp(type, "ARM3")) {
		value = IPC_CORE_ARM3;
	} else if(!strcmp(type, "DB0")) {
		value = IPC_CORE_DB0;
	} else if(!strcmp(type, "DB1")) {
		value = IPC_CORE_DB1;
	} else if(!strcmp(type, "SEC")) {
		value = IPC_CORE_SEC;
	} else if(!strcmp(type, "SAFE")) {
		value = IPC_CORE_SAFE;
	} else if(!strcmp(type, "RT0")) {
		value = IPC_CORE_RT0;
	} else if(!strcmp(type, "RT1")) {
		value = IPC_CORE_RT1;
	} else if(!strcmp(type, "RT2")) {
		value = IPC_CORE_RT2;
	} else if(!strcmp(type, "SW0")) {
		value = IPC_CORE_SW0;
	} else if(!strcmp(type, "SW1")) {
		value = IPC_CORE_SW1;
	} else if(!strcmp(type, "SW2")) {
		value = IPC_CORE_SW2;
	} else {
		DISP_ERR("ipc pid type is wrong!");
		return -EINVAL;
	}

	return value;
}

static void fw_msg_ipc_events_handler(uint32_t subdev, struct fw_msg_events_cmd *cmd) {
	struct bst_fw_ipc_device *ipc = g_fw_ipc;

	if (subdev > BST_SUBDEV_NONE && ipc->display_events[subdev - 1].cb) {
		ipc->display_events[subdev - 1].cb(cmd->evts, ipc->display_events[subdev - 1].ext);
	}
}

static int bst_display_kthread_recv(void *data) {
	struct bst_fw_ipc_device *ipc = (struct bst_fw_ipc_device *)data;
	struct sched_param param;
	struct fw_msg_events_cmd evts_cmd;
	ipc_msg msg;
	int ret;
	int timeout = BST_DISPLAY_IPC_TIMEOUT;
	int count;
	
	for (count = 0; count < ARRAY_SIZE(ipc->display_events); count++) {
		ipc->display_events[count].cb = NULL;
		ipc->display_events[count].subdev = BST_SUBDEV_NONE;
		ipc->display_events[count].sub_status = 0;
	}
	param.sched_priority = MAX_RT_PRIO - 1;
	sched_setscheduler(current, SCHED_FIFO, &param);
	while(1) {
		ipc->msg_kthread_status = DISP_KTHREAD_STATUS_RECV_MSG;
		ret = ipc_recv(ipc->evt_session_id, &msg, timeout);
		if (ret < 0) {
			DISP_ERR("%s:%d ipc recv error, ret:%d\n", __func__, __LINE__, ret);
			continue;
		}
		ipc->msg_kthread_status = DISP_KTHREAD_STATUS_RECV_COMPLETE;
		switch (msg.cmd & GENMASK(3, 0)) {
			case IPC_CMD_TYPE_MSG: {
				// complete(&ipc->msg_completion);
			} break;
			case IPC_CMD_TYPE_EVTS: {
				memset(&evts_cmd, 0, sizeof(struct fw_msg_events_cmd));
				evts_cmd.evts.events = (msg.data & GENMASK(15, 0));
				evts_cmd.subdev = (msg.data & GENMASK(31, 16)) >> 16;
				fw_msg_ipc_events_handler(evts_cmd.subdev, &evts_cmd);
			} break;
			default:
				break;
			}
	}
	return ret;
}

static int fw_ipc_send(struct bst_fw_ipc_device *fw_dev, ipc_msg *msg) {
	int ret;

	mutex_lock(&fw_dev->ipc_tx_mutex);
	// ret = ipc_send(fw_dev->msg_session_id, msg, BST_DISPLAY_IPC_TIMEOUT, 1);
	ret = ipc_send_sync(fw_dev->msg_session_id, msg);
	if (ret < 0) {
		DISP_ERR("ipc transfer fw msg failed!\n");
		mutex_unlock(&fw_dev->ipc_tx_mutex);
		return ret;
	}
	mutex_unlock(&fw_dev->ipc_tx_mutex);
	return ret;
}

int fw_msg_events_sub(uint32_t subdev, disp_event_callback_t cb, void *ext)
{
	struct bst_fw_ipc_device *ipc = g_fw_ipc;
	struct fw_msg_events_cmd evts_data = {0};
	u32 buf_offset;
	ipc_msg msg;

	if (!ipc->inited)
		return 0;

	ipc->display_events[subdev - 1].subdev = subdev;
	ipc->display_events[subdev - 1].cb = cb;
	ipc->display_events[subdev - 1].ext = ext;
	ipc->display_events[subdev - 1].sub_status = 1;
	buf_offset = sizeof(struct fw_msg_events_cmd) * (subdev - 1);
	evts_data.sub_status = ipc->display_events[subdev - 1].sub_status;
	evts_data.subdev = subdev;
	memset(ipc->evts_vaddr + buf_offset, 0, sizeof(struct fw_msg_events_cmd));
	memcpy(ipc->evts_vaddr + buf_offset, &evts_data, sizeof(struct fw_msg_events_cmd));
	ipc_dsb();

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.cmd = (IPC_CMD_SRV_DISPLAY << 4) | (IPC_CMD_TYPE_EVTS & GENMASK(3, 0));
	msg.data = paddr_64_to_paddr_32(ipc->evts_paddr + buf_offset);

	return fw_ipc_send(ipc, &msg);
}
int fw_msg_events_unsub(uint32_t subdev)
{
	struct bst_fw_ipc_device *ipc = g_fw_ipc;
	struct fw_msg_events_cmd evts_data = {0};
	uint32_t buf_offset;
	ipc_msg msg;

	if (!ipc->display_events[subdev - 1].sub_status) {
		DISP_ERR("Error, Subdev(%d) not subscribed!\n", subdev);
		return -1;
	}

	if (!ipc->inited)
		return 0;
	buf_offset = sizeof(struct fw_msg_events_cmd) * (subdev - 1);
	ipc->display_events[subdev - 1].cb = NULL;
	ipc->display_events[subdev - 1].sub_status = 0;
	evts_data.sub_status = ipc->display_events[subdev - 1].sub_status;
	evts_data.subdev = subdev;
	memset(ipc->evts_vaddr + buf_offset, 0, sizeof(struct fw_msg_events_cmd));
	memcpy(ipc->evts_vaddr + buf_offset, &evts_data, sizeof(struct fw_msg_events_cmd));

	ipc_dsb();

	msg.type = IPC_MSG_TYPE_SIGNAL;
	msg.cmd = (IPC_CMD_SRV_DISPLAY << 4) | (IPC_CMD_TYPE_EVTS & GENMASK(3, 0));
	msg.data = paddr_64_to_paddr_32(ipc->evts_paddr + buf_offset);

	return fw_ipc_send(ipc, &msg);
}

int transfer_fw_msg(struct fw_msg_data *msg_data)
{
	struct bst_fw_ipc_device *ipc = g_fw_ipc;
	ipc_msg msg;
	int ret, str_msg = 0;

	if (!ipc->inited)
		return 0;

	mutex_lock(&ipc->msg_mutex);
	memset(ipc->msg_vaddr, 0, sizeof(struct fw_msg_data));
	memcpy(ipc->msg_vaddr, msg_data, sizeof(struct fw_msg_data));
	ipc_dsb();
	msg.type = IPC_MSG_TYPE_METHOD;

	//STR case: change to signal type, do not need reply
	if((BST_DISPLAY_DC_SUBDEV == msg_data->cmdset) &&
		(DC_CMD_DISABLE_SUBMODULE == msg_data->cmdid) ) {
		struct bst_display_submodule_req *dis_moddule = (struct bst_display_submodule_req *)(&msg_data->user_cmd_data[0]);

		if(SUBMODULE_ID_DC_PIPE_STR == dis_moddule->submodule_id) {
			str_msg = 1;
			msg.type = IPC_MSG_TYPE_SIGNAL;
		}
	}
	msg.cmd = (IPC_CMD_SRV_DISPLAY << 4) | (IPC_CMD_TYPE_MSG & GENMASK(3, 0));
	msg.data = paddr_64_to_paddr_32(ipc->msg_paddr);

	ret = fw_ipc_send(ipc, &msg);
	if (ret < 0) {
		mutex_unlock(&ipc->msg_mutex);
		return ret;
	}

	if (str_msg) {
		dev_info(ipc->dev, "%s: STR signal msg!\n", __func__);
	} else {
		do {
			ret = ipc_recv(ipc->msg_session_id, &msg, WAIT_CMD_TIMEOUT);
			if (ret == IPC_RECV_ERR_TIMEOUT) {
				dev_err(ipc->dev, "%s: timeout to wait fw msg recv!\n", __func__);
			} else if (ret < 0) {
				DISP_ERR("%s:%d ipc recv error, ret:%d\n", __func__, __LINE__, ret);
			}
		} while (ret < 0);
		memcpy(msg_data, ipc->msg_vaddr, sizeof(struct fw_msg_data));
	}
	mutex_unlock(&ipc->msg_mutex);

	return 0;
}

static int fw_ipc_send_open(struct seq_file *s, void *unused)
{
	struct bst_fw_ipc_device *dev = s->private;
	int ret;

	memcpy(&dev->msg, dev->msg_vaddr, sizeof(struct fw_msg_data));
	ret = transfer_fw_msg(&dev->msg);
	seq_printf(s, "send %s!\n", !!ret ? "Success" : "Failed");

	return ret;
}
static int bst_fw_ipc_send_open(struct inode *inode, struct file *file)
{
	return single_open(file, fw_ipc_send_open, inode->i_private);
}


static const struct file_operations bst_fw_ipc_send_fops = {
	.open = bst_fw_ipc_send_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int fw_ipc_read_open(struct seq_file *s, void *unused)
{
	struct bst_fw_ipc_device *dev = s->private;
	u32 *msg = (u32 *)dev->msg_vaddr;
	int i;
	
	seq_printf(s, "\t == fw msg data dump ==\n");
	for (i = 0; i < (sizeof(struct fw_msg_data) / sizeof(u32) + 1); i++) {
		seq_printf(s, "\t  %#llx: %#x\n", dev->msg_paddr + 4 * i, msg[i]);
	}
	seq_printf(s, "\n\n");
	
	msg = (u32 *)dev->evts_vaddr;
	seq_printf(s, "\t == fw evts data dump ==\n");
	for (i = 0; i < ((sizeof(struct fw_msg_events_cmd) * (BST_SUBDEV_MAX - 1)) / sizeof(u32) + 1); i++) {
		seq_printf(s, "\t  %#llx: %#x\n", dev->evts_paddr + 4 * i, msg[i]);
	}
	seq_printf(s, "\n\n");

	return 0;
}
static int bst_fw_ipc_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, fw_ipc_read_open, inode->i_private);
}

static const struct file_operations bst_fw_ipc_read_fops = {
	.open = bst_fw_ipc_read_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void bst_fw_ipc_debugfs_init(void) {
	struct dentry *root;
	struct dentry *file;

	root = debugfs_create_dir("fw_ipc", NULL);
	if (IS_ERR_OR_NULL(root)) {
		DISP_ERR("Can't create fw ipc debugfs root\n");
		return;
	}
	file = debugfs_create_file("send", 0644, root, g_fw_ipc, &bst_fw_ipc_send_fops);
	if (!file)
		DISP_ERR("Can't create fw ipc send debugfs\n");
	file = debugfs_create_file("dump", 0644, root, g_fw_ipc, &bst_fw_ipc_read_fops);
	if (!file)
		DISP_ERR("Can't create fw ipc dump debugfs\n");
}

static int bst_fw_ipc_parse_dt(struct platform_device *pdev)
{
	struct bst_fw_ipc_device *fw_ipc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	const char *pid_type;
	uint32_t ipc_core;
	int ret = 0;
	
	ret = of_property_read_string(dev->of_node, "ipc-pid", &pid_type);
	if (ret) {
		DISP_ERR("%pOF: invalid 'ipc-pid' property: %d\n",
				dev->of_node, ret);
		return -EINVAL;
	}
	ipc_core = ipc_pid_map(pid_type);
	if (ipc_core >= IPC_CORE_MAX)
		return -EINVAL;
	
	fw_ipc->ipc_core_id = (enum ipc_core_e)ipc_core;

	return ret;
}

static int bst_fw_ipc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bst_fw_ipc_device *ipc;
	struct resource iomem_msg;
	dma_addr_t dma_addr;
	int ret;
	
	ipc = devm_kzalloc(dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;

	ipc->dev = &pdev->dev;
	ipc->pdev = pdev;

	platform_set_drvdata(pdev, ipc);
	
	ret = bst_fw_ipc_parse_dt(pdev);
	if (ret)
		goto fail;
	
	/* Initialize reserved memory resources */
	ret = of_reserved_mem_device_init(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to reserve memory, ret(%d)\n", ret);
		goto fail;
	}

	dev_info(dev, "Reserved 0x%08llX - 0x%08llX for message exchange\n",
		 iomem_msg.start, iomem_msg.end);
	ipc->shmem_size = PAGE_ALIGN(sizeof(struct fw_msg_data) + sizeof(struct fw_msg_events_cmd));
	ipc->msg_vaddr = dma_alloc_coherent(dev, ipc->shmem_size, &dma_addr, GFP_KERNEL);
	if (IS_ERR(ipc->msg_vaddr)) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to map message memory region: %ld (size: %u)\n", PTR_ERR(ipc->msg_vaddr), ipc->shmem_size);
		goto fail;
	}

	ipc->msg_paddr = dma_to_phys(&pdev->dev, dma_addr);
	ipc->evts_paddr = EVENTS_PADDR_OFFSET + ipc->msg_paddr;
	ipc->evts_vaddr = EVENTS_PADDR_OFFSET + ipc->msg_vaddr;
	
	memset(ipc->msg_vaddr, 0, resource_size(&iomem_msg));
	
	dev_info(dev, "Allocated msg coherent memory (vaddr: 0x%0llx, paddr: 0x%0llx size: %u aligned size: %u)\n",
		(u64)ipc->msg_vaddr, ipc->msg_paddr, ipc->shmem_size, PAGE_ALIGN(ipc->shmem_size));
	dev_info(dev, "Allocated msg coherent memory evts (vaddr: 0x%0llx, paddr: 0x%0llx)\n",(u64)ipc->evts_vaddr, ipc->evts_paddr);
	
	ipc->msg_session_id = ipc_init(IPC_CORE_SAFE, ipc->ipc_core_id, NULL);
	if (ipc->msg_session_id < 0) {
		dev_err(&pdev->dev, "msg ipc_init(%d) failed\n", ipc->ipc_core_id);
		goto fail;
	}
	ipc->evt_session_id = ipc_init(IPC_CORE_SAFE, ipc->ipc_core_id, NULL);
	if (ipc->evt_session_id < 0) {
		dev_err(&pdev->dev, "evt ipc_init(%d) failed\n", ipc->ipc_core_id);
		goto fail;
	}
	ret = ipc_signal_subscribe(ipc->evt_session_id, (IPC_CMD_SRV_DISPLAY << 4) | IPC_CMD_TYPE_EVTS);
	if (ret) {
		dev_err(&pdev->dev, "msg ipc signal subscribe(%d) failed\n", ret);
		goto fail;
	}

	ipc->kthread_recv =
			kthread_run(bst_display_kthread_recv, ipc, "disp-recv-msg");
	if (IS_ERR(ipc->kthread_recv)) {
		dev_err(dev, "create display kthread msg recv error\n");
		goto fail;
	}

	init_completion(&ipc->msg_completion);
	mutex_init(&ipc->ipc_tx_mutex);
	mutex_init(&ipc->msg_mutex);
	spin_lock_init(&ipc->msg_shmem_lock);
	ipc->inited = true;
	g_fw_ipc = ipc;
#ifdef CONFIG_DEBUG_FS
	/* debugfs init */
	bst_fw_ipc_debugfs_init();
#endif
	return ret;
fail:
	devm_kfree(&pdev->dev, ipc);
	return ret;
}

static int bst_fw_ipc_remove(struct platform_device *pdev)
{
	struct bst_fw_ipc_device *ipc = platform_get_drvdata(pdev);

	dma_free_coherent(&pdev->dev, ipc->shmem_size, ipc->msg_vaddr,
						phys_to_dma(&pdev->dev, ipc->msg_paddr));
	of_reserved_mem_device_release(&pdev->dev);
	ipc_close(ipc->msg_session_id);
	ipc_close(ipc->evt_session_id);
	return 0;
}

static const struct of_device_id drv_dt_ids[] = {
	{ .compatible = "bst,bst-display-firmware-ipc" },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct platform_driver bst_fw_ipc_driver = {
	.probe = bst_fw_ipc_probe,
	.remove = bst_fw_ipc_remove,
	.driver = {
		.name = "disp-fw-ipc",
		.of_match_table = drv_dt_ids,
	},
};

module_platform_driver(bst_fw_ipc_driver);
MODULE_AUTHOR("BST Ltd.");
MODULE_DESCRIPTION("BST Display-FW-IPC Driver");
MODULE_LICENSE("GPL v2");
