// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn: Linux device driver for Black Sesame Technologies Computer Vision IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_fw_manager.c
 * @brief   This file is the source code file of the firmware manager part of
 *          the bst_lwnn driver. It contains function definitions of
 *          initialization, cleanup and exit of the firmware manager and the
 *          runtime firmware setup.
 * @note    As the index of the DSP in the metadata array is not the same as
 *          the index of the DSP in the CV subsystem, the array index is
 *          referred as DSP id to differentiate from the hardware DSP index.
 */

#include "bst_lwnn.h"
#include <linux/iommu.h>
#include <linux/mm.h>

extern int has_cv_dsp2_iommu_map;
extern int has_cv_dsp3_iommu_map;
/*******************************************************************************
 * bst_lwnn FIRMWARE
 ******************************************************************************/

#ifdef BST_LWNN_DEBUG
/*!
 * @brief       This is a debug function to dump the firmware buffer.
 * @param[in]   fw_buf The pointer to the firmware buffer
 * @param[in]   size The dumped size
 * @return      Void
 */
static void _dump_firmware(char *fw_buf, int size)
{
	int i;

	for (i = 0; i < size / BST_LWNN_FIRMWARE_DUMP_LINE_SIZE; i++) {
		BST_LWNN_TRACE_PRINTK(
			"%02x %02x %02x %02x %02x %02x %02x %02x",
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 1),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 2),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 3),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 4),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 5),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 6),
			*(fw_buf + i * BST_LWNN_FIRMWARE_DUMP_LINE_SIZE + 7));
	}
	return;
}
#endif

/*!
 * @brief       This function loads the firmware.
 * @param[in]   pbst_lwnn The  bst_lwnn driver
 * @param[in]   dsp The DSP id
 * @param[in]   fw The pointer to the requested firmware
 * @return      0 - success
 *              Error code - failure
 */
static int _load_firmware(struct bst_lwnn *pbst_lwnn, int dsp,
			  struct firmware *fw)
{
	if (pbst_lwnn->fw_manager.dsps[dsp].fwmem_size < fw->size) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "firmware too large for DSP %d", dsp);
		return -EINVAL;
	}

	BST_LWNN_TRACE_PRINTK("firmware data: 0x%px, size: %ld", fw->data,
			      fw->size);

	memcpy_toio(pbst_lwnn->fw_manager.dsps[dsp].fwmem_base, fw->data,
		    fw->size);

#ifdef BST_LWNN_DEBUG
	_dump_firmware(pbst_lwnn->fw_manager.dsps[dsp].fwmem_base,
		       BST_LWNN_FIRMWARE_DUMP_SIZE);
#endif

	return 0;
}

/*!
 * @brief       This fucntion resets the firmware to boot it up.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   dsp The DSP id
 * @return      Void
 */
static void _pre_load_firmware(struct bst_lwnn *pbst_lwnn, int dsp)
{
	uint32_t reg;

	// Enable cv parity
	reg = readl_relaxed(pbst_lwnn->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0);
	reg |= 0x3; // [1:0] - cv_internal_pty_en, cv_sram_ecc_en. disable: reg &= ~0x3;
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0);

	//disable cv chk intr en
	reg = readl_relaxed(pbst_lwnn->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_DSP_CLUSTER_INTR_EN_REG);
	reg &= ~(0x7 << (16 + pbst_lwnn->dsp_indices[dsp] * 3));
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_DSP_CLUSTER_INTR_EN_REG);

	//setup reset vector
	writel_relaxed(
		phys_to_bus(pbst_lwnn->fw_manager.dsps[dsp].fwmem_phys_addr),
		pbst_lwnn->fw_manager.lb_cv_reg_base +
			LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET +
			pbst_lwnn->dsp_indices[dsp] * BST_LWNN_REG_WIDTH);

	reg = readl_relaxed(pbst_lwnn->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	//Attention
	//all clocks must be enabled, otherwise, the DSP cannot load elf firmware
	reg |= (0x0F << (BST_LWNN_DSP0_CLK_EN)); //clk enable
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg |= (1 << (BST_LWNN_RUNSTALL_BIT +
		      pbst_lwnn->dsp_indices[dsp])); //dsp stop
	reg |= (1 << (BST_LWNN_SOFT_RESET_BIT +
		      pbst_lwnn->dsp_indices[dsp])); //reset disable
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
}

/*
 * @func    _post_load_firmware_success
 * @brief   End work after loading is completed.
 * @params  pbst_lwnn - the pointer to the bst_lwnn device
 * @return  void
 */
static void _post_load_firmware_success(struct bst_lwnn *pbst_lwnn, int dsp)
{
	uint32_t reg;
	//start dsp
	reg = readl_relaxed(pbst_lwnn->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg |= (1 << (BST_LWNN_SOFT_RESET_BIT +
		      pbst_lwnn->dsp_indices[dsp])); //reset disable
	reg |= (1 << (BST_LWNN_DSP0_CLK_EN +
		      pbst_lwnn->dsp_indices[dsp])); //clk enable
	reg &= ~(1 << (BST_LWNN_RUNSTALL_BIT +
		       pbst_lwnn->dsp_indices[dsp])); //dsp run
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	BST_LWNN_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", reg,
			      pbst_lwnn->fw_manager.lb_cv_reg_base +
				      LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	return;
}

/*
 * @func    _post_load_firmware_failed
 * @brief   End work after loading is completed.
 * @params  pbst_lwnn - the pointer to the bst_lwnn device
 * @return  void
 */
static void _post_load_firmware_failed(struct bst_lwnn *pbst_lwnn, int dsp)
{
	uint32_t reg;
	//stop dsp
	reg = readl_relaxed(pbst_lwnn->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg &= ~(1 << (BST_LWNN_SOFT_RESET_BIT +
		       pbst_lwnn->dsp_indices[dsp])); //reset enable
	reg &= ~(1 << (BST_LWNN_DSP0_CLK_EN +
		       pbst_lwnn->dsp_indices[dsp])); //clk disable
	reg |= (1 << (BST_LWNN_RUNSTALL_BIT +
		      pbst_lwnn->dsp_indices[dsp])); //dsp stop
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	return;
}

/*!
 * @brief       This function loads the runtime firmware and boots it up.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
static int _bringup_firmware(struct bst_lwnn *pbst_lwnn)
{
	int ret;
	int i;
	struct firmware *fw;
	bool firmware_load = false;

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (pbst_lwnn->dsp_online[i]) {
			firmware_load = false;
			if (bst_lwnn_msg_interface ==
			    BST_LWNN_MSG_INTERFACE_IPC) {
				ret = request_firmware(
					(const struct firmware **)&fw,
					pbst_lwnn->fw_manager.dsps[i].name,
					&pbst_lwnn->pdev->dev);
			} else {
				ret = request_firmware(
					(const struct firmware **)&fw,
					pbst_lwnn->fw_manager.dsps[i]
						.name_msgbox,
					&pbst_lwnn->pdev->dev);
			}

			if (ret < 0) {
				BST_LWNN_DEV_ERR(
					&pbst_lwnn->pdev->dev,
					"failed to request firmware for DSP %d: %d",
					pbst_lwnn->dsp_indices[i], ret);
				pbst_lwnn->dsp_online[i] = 0;
				continue;
			}
			BST_LWNN_STAGE_PRINTK(
				"bst_lwnn firmware requested for DSP %d", i);

			_pre_load_firmware(pbst_lwnn, i);
			ret = _load_firmware(pbst_lwnn, i, fw);
			release_firmware(fw);
			firmware_load = true;
			if (ret < 0) {
				continue;
			}

			if (firmware_load == true) {
				if (ret < 0) {
					BST_LWNN_DEV_ERR(
						&pbst_lwnn->pdev->dev,
						"failed to load firmware for DSP %d: %d",
						i, ret);
					pbst_lwnn->dsp_online[i] = 0;
					_post_load_firmware_failed(pbst_lwnn,
								   i);
				} else {
					_post_load_firmware_success(pbst_lwnn,
								    i);
					pbst_lwnn->fw_manager.dsps[i].boot = 1;
					BST_LWNN_STAGE_PRINTK(
						"bst_lwnn firmware booted for DSP %d",
						i);
				}
			}
		}
	}

	return bst_lwnn_check_online(pbst_lwnn) ? 0 : -EFAULT;
}

/*!
 * @brief       This function initializes the firmware manager.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_fw_manager_init(struct bst_lwnn *pbst_lwnn)
{
	int i;
	int ret;
	struct resource *res;
	uint32_t assigned_mem_size;
	int msgbox_endid;
	struct device_node *bst_lwnn_node, *dsp_node;

	bst_lwnn_node = pbst_lwnn->pdev->dev.of_node;
	//map registers
	pbst_lwnn->fw_manager.lb_cv_reg_base =
		devm_of_iomap(&pbst_lwnn->pdev->dev, bst_lwnn_node, 0, NULL);
	if (IS_ERR(pbst_lwnn->fw_manager.lb_cv_reg_base)) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "failed to map cv regs");
		return PTR_ERR(pbst_lwnn->fw_manager.lb_cv_reg_base);
	}
	//get the dsp number
	ret = device_property_read_u32(&pbst_lwnn->pdev->dev, "dsp-num",
				       &pbst_lwnn->dsp_num);
	if (ret == -EINVAL || ret == -ENODATA) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "no dsp-num property");
		return ret;
	} else if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid dsp-num property");
		return ret;
	}
	//get the ipc register address
	ret = device_property_read_u32(
		&pbst_lwnn->pdev->dev, "ipc-register-addr",
		&pbst_lwnn->fw_manager.ipc_register_addr);
	if (ret == -EINVAL || ret == -ENODATA) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "no ipc-register-addr property");
		return ret;
	} else if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid ipc-register-addr property");
		return ret;
	}

	pbst_lwnn->dsp_num = clamp(pbst_lwnn->dsp_num, 0, BST_LWNN_MAX_DSP_NUM);

	if (pbst_lwnn->dsp_num != of_get_child_count(bst_lwnn_node)) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "inconsistent DSP number in device tree");
		return -EFAULT;
	}

	if (bst_lwnn_dspcnt > pbst_lwnn->dsp_num) {
		BST_LWNN_DEV_ERR(
			&pbst_lwnn->pdev->dev,
			"invalid bst_lwnn_dspcnt %d > pbst_lwnn->dsp_num %d",
			bst_lwnn_dspcnt, pbst_lwnn->dsp_num);
		return -EFAULT;
	}
	// msgbox_endid for msgbox cpu client to firmware dsp server
	ret = device_property_read_u32(&pbst_lwnn->pdev->dev, "msgbox-endid",
				       &msgbox_endid);
	if (ret == -EINVAL || ret == -ENODATA) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "no msgbox-endid property");
		return ret;
	} else if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid msgbox-endid property");
		return ret;
	}
	pbst_lwnn->msg_manager.msgbx_data.com_data.pid = msgbox_endid;
	BST_LWNN_TRACE_PRINTK("msgbox-endid: 0x%x", msgbox_endid);

	i = 0;
	for_each_child_of_node(bst_lwnn_node, dsp_node) {
		if (i < pbst_lwnn->dsp_num - bst_lwnn_dspcnt) {
			BST_LWNN_STAGE_PRINTK("%d dsp do not start.", i);
			goto cur_iter_failure;
		}
		//get the CV DSP index
		ret = of_property_read_u32(dsp_node, "index",
					   &pbst_lwnn->dsp_indices[i]);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "no index property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "invalid index property");
			goto cur_iter_failure;
		}
		BST_LWNN_TRACE_PRINTK("CV DSP index of DSP %d: %d", i,
				      pbst_lwnn->dsp_indices[i]);

		//get the DSP initialization address
		ret = of_property_read_u32(
			dsp_node, "rt-init-addr",
			&pbst_lwnn->fw_manager.dsps[i].rt_init_addr);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "no rt-init-addr property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "invalid rt-init-addr property");
			goto cur_iter_failure;
		}
		BST_LWNN_STAGE_PRINTK(
			"rt-init-addr of DSP %d: 0x%x", i,
			pbst_lwnn->fw_manager.dsps[i].rt_init_addr);

		//get the assigned memory size
		ret = of_property_read_u32(dsp_node, "assigned-mem-size",
					   &assigned_mem_size);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "no assigned-mem-size property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "invalid assigned-mem-size property");
			goto cur_iter_failure;
		}
		//get the firmware name(ipc)
		ret = of_property_read_string(
			dsp_node, "firmware",
			(const char **)&pbst_lwnn->fw_manager.dsps[i].name);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "no firmware property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "invalid firmware property");
			goto cur_iter_failure;
		}
		BST_LWNN_STAGE_PRINTK("firmware(ipc) name %d: %s", i,
				      pbst_lwnn->fw_manager.dsps[i].name);

		//get the firmware name(msgbox)
		ret = of_property_read_string(
			dsp_node, "firmware1",
			(const char **)&pbst_lwnn->fw_manager.dsps[i]
				.name_msgbox);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "no firmware1 property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "invalid firmware1 property");
			goto cur_iter_failure;
		}
		BST_LWNN_STAGE_PRINTK(
			"firmware1(msgbox) name %d: %s", i,
			pbst_lwnn->fw_manager.dsps[i].name_msgbox);

		//get the ipc-src-core
		ret = of_property_read_u32(
			dsp_node, "ipc-src-core",
			&pbst_lwnn->fw_manager.dsps[i].ipc_src_core);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "no ipc-src-core property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "invalid ipc-src-core property");
			goto cur_iter_failure;
		}
		//map firmware memory
		res = platform_get_resource(pbst_lwnn->pdev, IORESOURCE_MEM,
					    1 + i);
		if (!res) {
			BST_LWNN_DEV_ERR(
				&pbst_lwnn->pdev->dev,
				"could not get firmware resource for DSP %d",
				i);
			goto cur_iter_failure;
		}
		BST_LWNN_STAGE_PRINTK(
			"firmware mem of DSP %d start: 0x%llx, end: 0x%llx", i,
			res->start, res->end);

		pbst_lwnn->fw_manager.dsps[i].fwmem_base = devm_ioremap(
			&pbst_lwnn->pdev->dev, res->start, resource_size(res));

		if (IS_ERR(pbst_lwnn->fw_manager.dsps[i].fwmem_base)) {
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "failed to remap firmware mem: %ld",
					 PTR_ERR(pbst_lwnn->fw_manager.dsps[i]
							 .fwmem_base));
			goto cur_iter_failure;
		}
		pbst_lwnn->fw_manager.dsps[i].fwmem_size =
			res->end - res->start + 1;
		pbst_lwnn->fw_manager.dsps[i].fwmem_phys_addr = res->start;
		BST_LWNN_STAGE_PRINTK(
			"firmware mem base for DSP %d: 0x%px(%llx,%llx), size: %llx", i,
			pbst_lwnn->fw_manager.dsps[i].fwmem_base,
			pbst_lwnn->fw_manager.dsps[i].fwmem_phys_addr,
			phys_to_bus(pbst_lwnn->fw_manager.dsps[i].fwmem_phys_addr),
			pbst_lwnn->fw_manager.dsps[i].fwmem_size);

		//check rt_init_addr
		if (pbst_lwnn->fw_manager.dsps[i].rt_init_addr <
			    phys_to_bus(res->start) ||
		    pbst_lwnn->fw_manager.dsps[i].rt_init_addr >
			    phys_to_bus(res->end)) {

			BST_LWNN_DEV_ERR(
				&pbst_lwnn->pdev->dev,
				"rt_init_addr of DSP %d is outside of firmware memory\nrt_init_addr:%x,start:%llx,end:%llx",
				i, pbst_lwnn->fw_manager.dsps[i].rt_init_addr, phys_to_bus(res->start), phys_to_bus(res->end));

			goto cur_iter_failure;
		}
		//assign memory for firmware
		pbst_lwnn->fw_manager.dsps[i].assigned_mem =
			pbst_lwnn->mem_manager.ops->alloc(
				pbst_lwnn, assigned_mem_size, 0, 0);
		if (pbst_lwnn->fw_manager.dsps[i].assigned_mem == NULL) {
			BST_LWNN_DEV_ERR(
				&pbst_lwnn->pdev->dev,
				"could not allocate assigned-mem for DSP %d",
				i);
			goto cur_iter_failure;
		}
		BST_LWNN_STAGE_PRINTK(
			"assigned mem of DSP %d: 0x%px, 0x%llx, size: 0x%x", i,
			pbst_lwnn->fw_manager.dsps[i].assigned_mem->kern_addr,
			pbst_lwnn->fw_manager.dsps[i].assigned_mem->dma_addr,
			assigned_mem_size);

		pbst_lwnn->dsp_online[i] = 1;
		pbst_lwnn->fw_manager.dsps[i].init = 1;
		goto loop_continue;

cur_iter_failure:
		pbst_lwnn->dsp_online[i] = 0;
		pbst_lwnn->fw_manager.dsps[i].init = 0;
loop_continue:
		i++;
	}

	return bst_lwnn_check_online(pbst_lwnn) ? 0 : -EFAULT;
}

/*!
 * @brief       This function initializes the runtime firmware. Because it
 *              relies on messaging and the message manager can only be
 *              initialized after getting some required firmware information in
 *              bst_lwnn_fw_manager_init, this part is separated from the
 *              firmware manager initialization.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_fw_rt_setup(struct bst_lwnn *pbst_lwnn)
{
	int i, j;
	int ret = 0;
	struct bst_lwnn_rt_init *init;
	struct bst_lwnn_msg_xchg msg_xchg;

	msg_xchg.req.opcode = RT_CMD_INIT;
	msg_xchg.req.pdata = 0;

	//boot firmware
	ret = _bringup_firmware(pbst_lwnn);
	if (ret < 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "bst_lwnn_boot_firmware failed for all DSPs");
		return ret;
	}
	BST_LWNN_STAGE_PRINTK("bst_lwnn_boot_firmware OK");

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (pbst_lwnn->dsp_online[i]) {
			msg_xchg.target_dsp = i;

			init = (void *)(pbst_lwnn->fw_manager.dsps[i].fwmem_base +
					(pbst_lwnn->fw_manager.dsps[i]
						 .rt_init_addr -
					 phys_to_bus(
						 pbst_lwnn->fw_manager.dsps[i]
							 .fwmem_phys_addr)));
			init->assigned_mem = kern_to_bus(
				pbst_lwnn,
				pbst_lwnn->fw_manager.dsps[i]
					.assigned_mem->kern_addr,
				pbst_lwnn->fw_manager.dsps[i].assigned_mem);
			init->assigned_mem_size =
				pbst_lwnn->fw_manager.dsps[i].assigned_mem->size;
			init->init_status = BST_LWNN_INIT_START;
			init->dest_core_id =
				IPC_CORE_ARM0 +
				pbst_lwnn->fw_manager.dsps[i].ipc_src_core;
			init->src_core_id =
				IPC_CORE_CV0 + pbst_lwnn->dsp_indices[i];
			init->ipc_register_addr =
				pbst_lwnn->fw_manager.ipc_register_addr;

			for (j = 0; j < 100; j++) {
				if (init->init_status != BST_LWNN_INIT_START) {
					break;
				}
				msleep(10);
			}

			if (!bst_lwnn_msg_is_bootdone(pbst_lwnn, i)) {
				pbst_lwnn->dsp_online[i] = 0;
				BST_LWNN_DEV_ERR(
					&pbst_lwnn->pdev->dev,
					"bst_lwnn_msg_is_bootdone target %d failed",
					i);
				continue;
			}

			if (init->init_status != BST_LWNN_INIT_END) {
				BST_LWNN_DEV_ERR(
					&pbst_lwnn->pdev->dev,
					"initialization for target %d failed",
					i);
				pbst_lwnn->dsp_online[i] = 0;
				continue;
			} else {
				if (pbst_lwnn->fw_manager.ver_info
					    .release_year == 0) {
					if (init->ver_info.major !=
						    BST_LWNN_FW_VER_MAJOR ||
					    init->ver_info.minor <
						    BST_LWNN_FW_VER_MINOR) {
						pbst_lwnn->dsp_online[i] = 0;
						BST_LWNN_DEV_ERR(
							&pbst_lwnn->pdev->dev,
							"target %d firmware version mismatched"
							"(require v%d.%d.* or higher minor versions)",
							i,
							BST_LWNN_FW_VER_MAJOR,
							BST_LWNN_FW_VER_MINOR);
						continue;
					}
					pbst_lwnn->fw_manager.ver_info
						.ver_major =
						init->ver_info.major;
					pbst_lwnn->fw_manager.ver_info
						.ver_minor =
						init->ver_info.minor;
					pbst_lwnn->fw_manager.ver_info
						.ver_patch =
						init->ver_info.patch;
					pbst_lwnn->fw_manager.ver_info
						.release_month =
						init->ver_info.month;
					pbst_lwnn->fw_manager.ver_info
						.release_date =
						init->ver_info.date;
					pbst_lwnn->fw_manager.ver_info
						.release_year =
						init->ver_info.year;
					BST_LWNN_STAGE_PRINTK(
						"target %d firmware version %d.%d.%d released on %d/%d/%d",
						i,
						pbst_lwnn->fw_manager.ver_info
							.ver_major,
						pbst_lwnn->fw_manager.ver_info
							.ver_minor,
						pbst_lwnn->fw_manager.ver_info
							.ver_patch,
						pbst_lwnn->fw_manager.ver_info
							.release_month,
						pbst_lwnn->fw_manager.ver_info
							.release_date,
						pbst_lwnn->fw_manager.ver_info
							.release_year);
				} else {
					if (pbst_lwnn->fw_manager.ver_info
							    .ver_major !=
						    init->ver_info.major ||
					    pbst_lwnn->fw_manager.ver_info
							    .ver_minor !=
						    init->ver_info.minor ||
					    pbst_lwnn->fw_manager.ver_info
							    .ver_patch !=
						    init->ver_info.patch) {
						pbst_lwnn->dsp_online[i] = 0;
						BST_LWNN_DEV_ERR(
							&pbst_lwnn->pdev->dev,
							"target %d firmware version mismatched with the first valid DSP",
							i);
						continue;
					}
				}
			}

			if (bst_lwnn_msg_xchg(pbst_lwnn, &msg_xchg) < 0) {
				BST_LWNN_DEV_ERR(
					&pbst_lwnn->pdev->dev,
					"handshake messsge for target %d failed",
					i);
				pbst_lwnn->dsp_online[i] = 0;
			} else if (msg_xchg.rsp.status !=
				   RT_CMD_STATUS_SUCCESS) {
				BST_LWNN_DEV_ERR(
					&pbst_lwnn->pdev->dev,
					"target %d returned failure status", i);
				pbst_lwnn->dsp_online[i] = 0;
			}
			pbst_lwnn->dsp_rtinit[i] = 1;
		}
	}

	bst_lwnn_fw_rt_cleanup(pbst_lwnn);
	return bst_lwnn_check_online(pbst_lwnn) ? 0 : -EFAULT;
}

/*!
 * @brief       This function cleans up the resources allocated in firmware
 *              manager initialization for subsequential failure of DSPs or even
 *              the driver during the entire initialization process.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      Void
 */
void bst_lwnn_fw_manager_cleanup(struct bst_lwnn *pbst_lwnn)
{
	int i;

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (!pbst_lwnn->dsp_online[i] &&
		    pbst_lwnn->fw_manager.dsps[i].init) {
			pbst_lwnn->mem_manager.ops->free(
				pbst_lwnn->fw_manager.dsps[i].assigned_mem);
		}
	}
	return;
}

/*!
 * @brief       This function resets the specified DSP.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   dsp The CV DSP index
 * @return      Void
 */
static inline void _release_rt_fw(struct bst_lwnn *pbst_lwnn, int dsp)
{
	uint32_t reg;

	//stall the DSP first
	reg = readl_relaxed(pbst_lwnn->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg |= (1 << (BST_LWNN_RUNSTALL_BIT + pbst_lwnn->dsp_indices[dsp]));
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	//then reset the DSP
	// writel_relaxed(LB_CV_REG_R_CV_SYS_CTRL_DEFAULT, pbst_lwnn->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg &= ~(1 << (BST_LWNN_SOFT_RESET_BIT + pbst_lwnn->dsp_indices[dsp]));
	writel_relaxed(reg, pbst_lwnn->fw_manager.lb_cv_reg_base +
				    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);

	BST_LWNN_STAGE_PRINTK("DSP %d released", dsp);
	return;
}

/*!
 * @brief       This function resets the DSPs for subsequential failure of DSPs
 *              or even the driver during the entire initialization process.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      Void
 */
void bst_lwnn_fw_rt_cleanup(struct bst_lwnn *pbst_lwnn)
{
	int i;

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (!pbst_lwnn->dsp_online[i] &&
		    pbst_lwnn->fw_manager.dsps[i].boot) {
			_release_rt_fw(pbst_lwnn, i);
		}
	}
	return;
}

/*!
 * @brief       This is the exit function of the firmware manager.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      Void
 */
void bst_lwnn_fw_manager_exit(struct bst_lwnn *pbst_lwnn)
{
	int i;

	for (i = 0; i < min(pbst_lwnn->dsp_num, BST_LWNN_MAX_DSP_NUM); i++) {
		if (pbst_lwnn->mem_manager.enable_smmu &&
		    pbst_lwnn->fw_manager.dsps[i].fwmem_iova != 0) {
			pbst_lwnn->mem_manager.ops->iommu_free(
				pbst_lwnn,
				pbst_lwnn->fw_manager.dsps[i].fwmem_size, 0,
				addr_truncate(pbst_lwnn->fw_manager.dsps[i]
						      .fwmem_iova));
			if (i == 2) {
				has_cv_dsp2_iommu_map = 0;
			}
			if (i == 3) {
				has_cv_dsp3_iommu_map = 0;
			}
		}
		if (pbst_lwnn->dsp_online[i]) {
			pbst_lwnn->mem_manager.ops->free(
				pbst_lwnn->fw_manager.dsps[i].assigned_mem);
		}
	}
	if (pbst_lwnn->mem_manager.enable_smmu) {
		// TODO, dts configuration
		if (pbst_lwnn->fw_manager.res_bypass[0].iova != 0) {
			pbst_lwnn->mem_manager.ops->iommu_free(
				pbst_lwnn,
				pbst_lwnn->fw_manager.res_bypass[0].size,
				PAGE_SIZE,
				addr_truncate(
					pbst_lwnn->fw_manager.res_bypass[0]
						.iova));
		}

		if (pbst_lwnn->fw_manager.res_bypass[1].iova != 0) {
			pbst_lwnn->mem_manager.ops->iommu_free(
				pbst_lwnn,
				pbst_lwnn->fw_manager.res_bypass[1].size,
				PAGE_SIZE,
				addr_truncate(
					pbst_lwnn->fw_manager.res_bypass[1]
						.iova));
		}
	}
	return;
}

/*!
 * @brief       This is the exit function of the runtime firmware.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      Void
 */
void bst_lwnn_fw_rt_exit(struct bst_lwnn *pbst_lwnn)
{
	int i, j;
	struct bst_lwnn_msg_xchg msg_xchg;

	for (i = 0; i < pbst_lwnn->dsp_num; i++) {
		if (pbst_lwnn->dsp_online[i]) {
			if (pbst_lwnn->dsp_rtinit[i]) {
				struct bst_lwnn_rt_init *init;

				msg_xchg.target_dsp = i;
				msg_xchg.req.opcode = RT_CMD_EXIT;
				msg_xchg.req.pdata = 0;
				bst_lwnn_msg_xchg(pbst_lwnn, &msg_xchg);
				init = (void *)(pbst_lwnn->fw_manager.dsps[i]
							.fwmem_base +
						(pbst_lwnn->fw_manager.dsps[i]
							 .rt_init_addr -
						 phys_to_bus(
							 pbst_lwnn->fw_manager
								 .dsps[i]
								 .fwmem_phys_addr)));
				for (j = 0; j < 100; j++) {
					if (init->init_status ==
					    BST_LWNN_INIT_EXIT) {
						break;
					}
					msleep(10);
				}
			}
			_release_rt_fw(pbst_lwnn, i);
		}
	}
	return;
}
