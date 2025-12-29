// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_fw_manager.c
 * @brief   This file is the source code file of the firmware manager part of
 *          the BSTN driver. It contains function definitions of initialization
 *          and cleanup of the firmware manager.
 */

#include "bstn.h"
#include "bstn_mem_manager.h"
#include <linux/iommu.h>
#include <linux/mm.h>

/*******************************************************************************
 * BSTN FIRMWARE
 ******************************************************************************/
#ifdef BSTN_DEBUG
/*
 * @func    _firmware_dump
 * @brief   This is a debug function to dump the firmware buffer.
 * @params  fw_buf - the pointer to the firmware buffer
 *          size - the dumped size
 * @return  void
 */
static void _firmware_dump(char *fw_buf, int size)
{
	int i;
	for (i = 0; i < size / BSTN_FIRMWARE_DUMP_LINE_SIZE; i++) {
		BSTN_TRACE_PRINTK(
			"%02x %02x %02x %02x %02x %02x %02x %02x",
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 1),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 2),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 3),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 4),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 5),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 6),
			*(fw_buf + i * BSTN_FIRMWARE_DUMP_LINE_SIZE + 7));
	}
	return;
}
#endif

/*
 * @func    bstn_firmware_load
 * @brief   This function loads the firmware.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bstn_firmware_load(struct bstn_device *pbstn)
{
	int ret = 0;
	int count = 10;
	const struct firmware *fw;
	do {
		if (unlikely(bstn_msg_interface == BSTN_MSG_INTERFACE_IPC)) {
			ret = request_firmware(&fw, pbstn->fw_manager.name,
					       &pbstn->pdev->dev);
		} else {
			ret = request_firmware(&fw,
					       pbstn->fw_manager.name_msgbox,
					       &pbstn->pdev->dev);
		}
		if (unlikely(ret < 0)) {
			if (ret == -EINTR) {
				count--;
				continue;
			}
		}
		break;
	} while (count > 0);

	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "Failed to request firmware: %d", ret);
		return ret; // Return early to prevent resource leak
	}
	BSTN_STAGE_PRINTK("bstn firmware requested");

	if (!pbstn->fw_manager.fwmem_base ||
	    pbstn->fw_manager.fwmem_size < fw->size) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "FW doesn't get memory assigned or "
			     "assigned memory size is insufficient");
		ret = -EINVAL;
		release_firmware(fw); // Release firmware data before returning
		return ret; // Return with proper error code
	}

	BSTN_TRACE_PRINTK("firmware mem base: 0x%px, size: %lld",
			  pbstn->fw_manager.fwmem_base,
			  pbstn->fw_manager.fwmem_size);
	BSTN_TRACE_PRINTK("firmware data: 0x%px, size: %ld", fw->data,
			  fw->size);

	memcpy_toio(pbstn->fw_manager.fwmem_base, fw->data, fw->size);
	release_firmware(fw);

#ifdef BSTN_DEBUG
	_firmware_dump(pbstn->fw_manager.fwmem_base, BSTN_FIRMWARE_DUMP_SIZE);
#endif

	return ret; // Return success or error code
}

void bstn_firmware_stall(struct bstn_device *pbstn)
{
	bstn_sys_ctrl_t sys_ctrl = { .all = 0 };

	sys_ctrl.all =
		readl_relaxed(pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	// runstall
	sys_ctrl.b.dsp_runstall = 1;
	writel_relaxed(sys_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	BSTN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", sys_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);

	return; // Return success or error code
}

void bstn_firmware_unstall(struct bstn_device *pbstn)
{
	bstn_sys_ctrl_t sys_ctrl = { .all = 0 };

	sys_ctrl.all =
		readl_relaxed(pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	// runstall
	sys_ctrl.b.dsp_runstall = 0; // bit27
	writel_relaxed(sys_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	BSTN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", sys_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);

	return; // Return success or error code
}

/*
 * @func    bstn_firmware_boot
 * @brief   This fucntion resets the firmware to boot it up.
 * @params  pbstn - the pointer to the BSTN device
 * @return  void
 */
void bstn_firmware_boot(struct bstn_device *pbstn)
{
	bstn_sys_ctrl_t sys_ctrl = { .all = 0 };
	bstn_inten_ctrl_t inten_ctrl = { .all = 0 };
	bstn_core0_ctrl_t core0_ctrl = { .all = 0 };

	sys_ctrl.all =
		readl_relaxed(pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	// runstall
	sys_ctrl.b.dsp_runstall = 1;
	writel_relaxed(sys_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	BSTN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", sys_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);

	// setup fw addr to reset vector
	writel_relaxed(
		addr_truncate(phys_to_bus(pbstn->fw_manager.fwmem_phys_addr)),
		pbstn->fw_manager.net_sreg_base +
			BSTN_DSP_ALT_RESET_VEC_OFFSET);
	BSTN_STAGE_PRINTK(
		"set vector: 0x%x, addr: 0x%px",
		addr_truncate(phys_to_bus(pbstn->fw_manager.fwmem_phys_addr)),
		pbstn->fw_manager.net_sreg_base +
			BSTN_DSP_ALT_RESET_VEC_OFFSET);

	// dsp int enable
	inten_ctrl.b.net0_core2dsp_int_en = 1;
	inten_ctrl.b.msgbox_0_int_en = 1;
	inten_ctrl.b.msgbox_1_int_en = 1;
	inten_ctrl.b.msgbox_2_int_en = 1;
	inten_ctrl.b.msgbox_3_int_en = 1;
	writel_relaxed(inten_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_INTEN_CTRL);
	BSTN_STAGE_PRINTK("set int en ctrl value: 0x%x, addr: 0x%px",
			  inten_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_INTEN_CTRL);

	// core0 enable
	core0_ctrl.b.core0_greg_clk_en = 1;
	core0_ctrl.b.core0_gemm_clk_en = 1;
	core0_ctrl.b.core0_edp_clk_en = 1;
	core0_ctrl.b.core0_hctl_clk_en = 1;
	core0_ctrl.b.core0_dctl_clk_en = 1; // bit4
	core0_ctrl.b.core0_dbuf_clk_en = 1;
	core0_ctrl.b.core0_slice3_clk_en = 1;
	core0_ctrl.b.core0_slice2_clk_en = 1;
	core0_ctrl.b.core0_slice1_clk_en = 1; // bit8
	core0_ctrl.b.core0_slice0_clk_en = 1;
	core0_ctrl.b.core0_conv_clk_en = 1;
	core0_ctrl.b.core0_btmem_clk_en = 1;
	core0_ctrl.b.core0_ahb_clk_en = 1; // bit12
	core0_ctrl.b.core0_clk_en = 1;
	core0_ctrl.b.soft_rst_core0_greg_n = 1; // bit16
	core0_ctrl.b.soft_rst_core0_gemm_n = 1;
	core0_ctrl.b.soft_rst_core0_edp_n = 1;
	core0_ctrl.b.soft_rst_core0_hctl_n = 1;
	core0_ctrl.b.soft_rst_core0_dctl_n = 1; // bit20
	core0_ctrl.b.soft_rst_core0_dbuf_n = 1;
	core0_ctrl.b.soft_rst_core0_slice3_n = 1;
	core0_ctrl.b.soft_rst_core0_slice2_n = 1;
	core0_ctrl.b.soft_rst_core0_slice1_n = 1; // bit24
	core0_ctrl.b.soft_rst_core0_slice0_n = 1;
	core0_ctrl.b.soft_rst_core0_conv_n = 1;
	core0_ctrl.b.soft_rst_core0_btmem_n = 1;
	core0_ctrl.b.soft_rst_core0_ahb_n = 1; // bit28
	core0_ctrl.b.soft_rst_core0_n = 1;
	writel_relaxed(core0_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_CORE0_CTRL);
	BSTN_STAGE_PRINTK("set core0 ctrl value: 0x%x, addr: 0x%px",
			  core0_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_CORE0_CTRL);

	// start dsp
	sys_ctrl.b.safety_apb_parity_chk_dec_en = 1; // bit12
	sys_ctrl.b.safety_apb_parity_chk_enc_en = 1; // bit13
	sys_ctrl.b.soft_rst_msgbox_n = 1; // bit14
	sys_ctrl.b.net_parity_chk_dec_en = 1; // bit17
	sys_ctrl.b.dsp_parity_chk_dec_en = 1; // bit18
	sys_ctrl.b.soft_rst_dsp_debug_n = 1; // bit19
	sys_ctrl.b.net_parity_chk_en = 1; // bit22
	sys_ctrl.b.dsp_parity_chk_en = 1; // bit23
	sys_ctrl.b.dsp_axi_ecc_en = 1; // bit24
	sys_ctrl.b.msgbox_parity_chk_dec_en = 0; // bit25
	sys_ctrl.b.msgbox_parity_chk_enc_en = 1; // bit26
	sys_ctrl.b.dsp_runstall = 1; // bit27
	sys_ctrl.b.core_clk_en = 1; // bit28
	sys_ctrl.b.dsp_clk_en = 1; // bit29
	sys_ctrl.b.soft_rst_core_n = 0; // bit30
	sys_ctrl.b.soft_rst_dsp_n = 0; // bit31
	writel_relaxed(sys_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	BSTN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", sys_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);

	sys_ctrl.b.core_clk_en = 1; // bit28
	sys_ctrl.b.dsp_clk_en = 1; // bit29
	sys_ctrl.b.soft_rst_core_n = 1; // bit30
	sys_ctrl.b.soft_rst_dsp_n = 1; // bit31
	writel_relaxed(sys_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	BSTN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", sys_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);

	sys_ctrl.b.dsp_runstall = 0; // bit27
	writel_relaxed(sys_ctrl.all,
		       pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	BSTN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", sys_ctrl.all,
			  pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	return;
}

int bstn_fw_manager_map(struct bstn_device *pbstn)
{
	int ret;
	int retry_times = 0;
	uint32_t val = 0;
	struct resource *res;
	int assigned_memsize;
	phys_addr_t res_paddr;
	resource_size_t res_size;
	dma_addr_t iova;
	uint32_t dag_iova;
	uint32_t dag_size;
	uint32_t log_iova;
	uint32_t log_size;

	pbstn->fw_manager.hwsem = bstn_hwsem_init();
	if(NULL == pbstn->fw_manager.hwsem)
	{
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "bstn_hwsem_init failled.");
		ret = -ENOENT;
		return ret;
	}
	pbstn->fw_manager.main_os = (bool)bstn_hwsem_get(pbstn->fw_manager.hwsem, &val);
	BSTN_STAGE_PRINTK("fw hwsem:%s(0x%08x)", pbstn->fw_manager.main_os ? "main-os" : "no-main_os", val);

	pbstn->fw_manager.fw_boot_done = bstn_fw_is_boot(pbstn);

	if (pbstn->mem_manager.enable_smmu) {
		/* boot_done  && main_os,  skip */
		/* boot_done  && !main_os, skip */
		if(pbstn->fw_manager.fw_boot_done) {
			BSTN_STAGE_PRINTK("bstn firmware mapped, don't map again.");
		}
		else if(pbstn->fw_manager.main_os) { /* !boot_done && main_os,  boot  */
			BSTN_STAGE_PRINTK("main os mapping fw");
			// get firmware name(ipc)init
			ret = device_property_read_string(
				&pbstn->pdev->dev, "firmware",
				(const char **)(&(pbstn->fw_manager.name)));
			if (ret == -EINVAL || ret == -ENODATA) {
				dev_dbg(&pbstn->pdev->dev, "no bstn-firmware property found!");
				return ret;
			} else if (ret < 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev, "invalid firmware name, ret %d",
						 ret);
				return ret;
			}
			BSTN_STAGE_PRINTK("firmware(ipc) name: %s", pbstn->fw_manager.name);

			// get firmware name(msgbox)
			ret = device_property_read_string(
				&pbstn->pdev->dev, "firmware1",
				(const char **)(&(pbstn->fw_manager.name_msgbox)));
			if (ret == -EINVAL || ret == -ENODATA) {
				dev_dbg(&pbstn->pdev->dev, "no bstn-firmware1 property found!");
				return ret;
			} else if (ret < 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
						 "invalid firmware1 name, ret %d", ret);
				return ret;
			}
			BSTN_STAGE_PRINTK("firmware1(msgbox) name: %s",
					  pbstn->fw_manager.name_msgbox);

			// map firmware memory
			res = platform_get_resource(pbstn->pdev, IORESOURCE_MEM, 1);
			if (!res) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
						 "Could not get firmware resource.");
				ret = -ENOENT;
				return ret;
			}
			BSTN_STAGE_PRINTK("bstn firmware mem start: 0x%llx, end: 0x%llx",
					  res->start, res->end);

			pbstn->fw_manager.fwmem_base =
				devm_ioremap_resource(&pbstn->pdev->dev, res);
			if (IS_ERR(pbstn->fw_manager.fwmem_base)) {
				ret = PTR_ERR(pbstn->fw_manager.fwmem_base);
				BSTN_DEV_ERR(&pbstn->pdev->dev,
						 "Failed to remap bstn firmware mem: %d", ret);
				return ret;
			}
			res_paddr = res->start;
			res_size = res->end - res->start + 1;
			pbstn->fw_manager.fwmem_size = res_size;
			pbstn->fw_manager.fwmem_phys_addr = res_paddr;

			iova = pbstn->mem_manager.ops->iommu_map_fw(
					pbstn, (uint32_t)res_size, PAGE_SIZE,
					(phys_addr_t)res_paddr);
			pbstn->fw_manager.fwmem_iova = iova;
			BSTN_STAGE_PRINTK("main os map fw: pa 0x%llx iova 0x%llx size 0x%llx", res_paddr, iova, res_size);
			dag_iova = 0x62400000;
			dag_size = 0x00400000;
			BSTN_STAGE_PRINTK("main os fw dag: pa 0x%llx iova 0x%08x size 0x%08x", bus_to_phys(dag_iova), dag_iova, dag_size);
			log_iova = dag_iova + dag_size;
			log_size = 0x01000000 - dag_size;
			BSTN_STAGE_PRINTK("main os fw log: pa 0x%llx iova 0x%08x size 0x%08x", bus_to_phys(log_iova), log_iova, log_size);
		} else { /* !boot_done && !main_os, wait boot done. if boot done, map must be done. */
			retry_times = 1;
			while(!bstn_fw_is_boot(pbstn)) {
				msleep(10);
				BSTN_STAGE_PRINTK("wait main(other) os boot(map) fw done: %dms", 10 * retry_times);
				retry_times++;
			}

			/* update fw_boot_done flag */
			pbstn->fw_manager.fw_boot_done = bstn_fw_is_boot(pbstn);
			BSTN_STAGE_PRINTK("main(other) os boot(map) fw done");
		}
		BSTN_STAGE_PRINTK("bstn firmware map done.");
	} else {
		BSTN_STAGE_PRINTK("bstn not use smmu.");
	}

	// assign memory for firmware
	ret = device_property_read_u32_array(
		&pbstn->pdev->dev, "assigned-mem-size", &assigned_memsize, 1);
	if (ret == -EINVAL || ret == -ENODATA) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "no assigned-mem-size property, ret %d", ret);
		return ret;
	} else if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid assigned-mem-size, ret %d", ret);
		return ret;
	}

	pbstn->fw_manager.assigned_mem =
		pbstn->mem_manager.ops->alloc(pbstn, assigned_memsize, 0, 0);
	if (pbstn->fw_manager.assigned_mem == NULL) {
		ret = -ENOMEM;
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "cannot allocate assigned-mem, ret %d", ret);
		return ret;
	}
	BSTN_STAGE_PRINTK("assigned mem: 0x%px, 0x%llx, size: %d",
			  pbstn->fw_manager.assigned_mem->kern_addr,
			  pbstn->fw_manager.assigned_mem->dma_addr,
			  assigned_memsize);
	return 0;
}

/*
 * @func    bstn_fw_manager_init
 * @brief   This function initializes the firmware manager.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bstn_fw_manager_init(struct bstn_device *pbstn)
{
	int ret;
	struct resource *res;

	// map registers
	res = platform_get_resource(pbstn->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "Could not get net regs resource.");
		ret = -ENOENT;
		return ret;
	}
	BSTN_STAGE_PRINTK("pbstn net regs start: 0x%llx, end: 0x%llx",
			  res->start, res->end);

	pbstn->fw_manager.net_sreg_base =
		devm_ioremap_resource(&pbstn->pdev->dev, res);
	if (IS_ERR(pbstn->fw_manager.net_sreg_base)) {
		ret = PTR_ERR(pbstn->fw_manager.net_sreg_base);
		return ret;
	}

	pbstn->fw_manager.main_os = false;
	pbstn->fw_manager.fw_boot_done = false;

	return 0;
}

/*
 * @func    bstn_fw_manager_exit
 * @brief   This is the cleanup function of the firmware manager.
 * @params  pbstn - the pointer to the BSTN device
 * @return  void
 */
void bstn_fw_manager_exit(struct bstn_device *pbstn)
{
	bstn_fw_manager_unmap(pbstn);
	return;
}

void bstn_fw_manager_unmap(struct bstn_device *pbstn)
{
	/* avoid make other os dead */
#if 0
	if (pbstn->mem_manager.enable_smmu) {
		if(pbstn->fw_manager.main_os) {
			pbstn->mem_manager.ops->iommu_free(
					pbstn, pbstn->fw_manager.fwmem_size, 0,
					addr_truncate(pbstn->fw_manager.fwmem_iova));
		}
	}
#endif
	if (pbstn->fw_manager.assigned_mem) {
		pbstn->mem_manager.ops->free(pbstn->fw_manager.assigned_mem);
		pbstn->fw_manager.assigned_mem = NULL;
	}

	if(pbstn->fw_manager.hwsem) {
		bstn_hwsem_uninit(pbstn->fw_manager.hwsem);
	}

	return;
}

/*
 * @func    bstn_fw_rt_exit
 * @brief   This is the cleanup function which corresponds to the runtime
 * setup.
 * @params  pbstn - the pointer to the BSTN device
 * @return  void
 */
void bstn_fw_rt_exit(struct bstn_device *pbstn)
{
	/* avoid make other os dead */
	struct bsnn_msg_exchange msg;
	if (pbstn->state == BSTN_ONLINE) {
		if (bstn_msg_interface == BSTN_MSG_INTERFACE_MSGBOX &&
		    pbstn->msg_manager.msgbx_client) {
			BSTN_STAGE_PRINTK("notify fw ko down...");

			msg.req.flag = 0x0000FFFF;
#ifdef CONFIG_BST_C1200_IVI
			msg.req.flag |= (RT_HOST_IVI << 24);
#elif defined(CONFIG_BST_C1200_DB)
			msg.req.flag |= (RT_HOST_DB << 24);
#elif defined(CONFIG_BST_C1200_ADAS)
			msg.req.flag |= (RT_HOST_ADAS << 24);
#else
			msg.req.flag |= (RT_HOST_ERR << 24);
			BSTN_DEV_ERR(&pbstn->pdev->dev, "host id invalid");
#endif

			msg.req.opcode = RT_CMD_EXIT;
			msg.rsp.status = -1;
			msg.req.pdata = 0;
			bstn_msg_exchange(pbstn, &msg);
			msleep(100);
			BSTN_STAGE_PRINTK("notify over: %d", msg.rsp.status);
		}
	}

#if 0
	uint32_t reg;

	if (pbstn->state == BSTN_ONLINE) {
		if (bstn_msg_interface == BSTN_MSG_INTERFACE_MSGBOX &&
		    pbstn->msg_manager.msgbx_client) {
			struct bsnn_msg_exchange exchange_msg;
			exchange_msg.req.opcode = RT_CMD_EXIT;
			exchange_msg.req.pdata = 0;
			// bstn_msg_exchange(pbstn, &exchange_msg);
			//wait fw exit and msg end offline
			// bstn_msg_is_bootdone(pbstn);
			// msleep(100);
		}
	}

	reg = readl_relaxed(pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	reg |= (1 << BSTN_DSP_RUNSTALL_BIT);
	writel_relaxed(reg, pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	reg &= ~((1 << BSTN_DSP_SOFT_RESET_BIT) |
		 (1 << BSTN_NET_SOFT_RESET_BIT));
	writel_relaxed(reg, pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
	reg &= ~((1 << BSTN_DSP_CLK_EN_BIT) | (1 << BSTN_CORE_CLK_EN_BIT));
	writel_relaxed(reg, pbstn->fw_manager.net_sreg_base + BSTN_SYS_CTRL);
#endif

	return;
}

void bstn_fw_set_boot_flag(struct bstn_device *pbstn)
{
	uint32_t reg = 0;
	writel_relaxed(BSTN_FW_STATUS_BOOT_DONE, pbstn->fw_manager.net_sreg_base + BSTN_FW_BOOT_FLAG_OFFSET);

	reg = readl_relaxed(pbstn->fw_manager.net_sreg_base + BSTN_FW_BOOT_FLAG_OFFSET);
	BSTN_STAGE_PRINTK("fw boot flag to:  0x%08x", reg);
}

bool bstn_fw_is_boot(struct bstn_device *pbstn)
{
	uint32_t reg = readl_relaxed(pbstn->fw_manager.net_sreg_base + BSTN_FW_BOOT_FLAG_OFFSET);
	BSTN_STAGE_PRINTK("bstn fw boot flag:0x%08x", reg);
	if(BSTN_FW_STATUS_BOOT_DONE == reg)
	{
		return true;
	}
	else if(BSTN_FW_STATUS_INIT == reg)
	{
		return false;
	}
	else /* others modify this regs, fw must bootted aready, send notify. */
	{
		BSTN_DEV_ERR(&pbstn->pdev->dev, "others modify this reg(0x50020084):0x%08x", reg);
		return true;
	}
}

