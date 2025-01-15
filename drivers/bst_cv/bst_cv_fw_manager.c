/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * bst_cv: Linux device driver for Black Sesame Technologies Computer Vision IP
 * author: AI Tools Team, BST Ltd.
 *
 * @file    bst_cv_fw_manager.c
 * @brief   This file is the source code file of the firmware manager part of
 *          the bst_cv driver. It contains function definitions of
 *          initialization, cleanup and exit of the firmware manager and the
 *          runtime firmware setup.
 */

#include "bst_cv.h"
#include "bst_cv_wdt.h"
#include <linux/iommu.h>

// the hard coded constants for firmware initialization
static const uint32_t ipc_dsp_irq_clear_addrs[BST_CV_DSP_NUM] = { 0x47103040, 0x47103144 };	//((0x47100000)|(1<<13)|(16<<8)|(16<2))

static const uint32_t ipc_dsp_irq_trigger_addr = 0x47103000;
static const uint32_t ipc_dsp_ireq_enable_addrs[BST_CV_DSP_NUM] = { 0x471030c0, 0x471031c4 };	//((0x47100000)|(1<<13)|(16<<8)|(1<<7)|(16<2))

static const uint32_t ipc_host_irq_clear_addr = 0x30102000;

static const uint32_t ipc_device_irq_indices[BST_CV_DSP_NUM] = { 0, 0 };

const char *bst_cv_support_firmware_suffix[BST_CV_SUPPORT_FIRMWARE_NUM] = {
	"\0",
	".rbf\0",
};

// EXPORT_SYMBOL(has_cv_dsp2_iommu_map);
// EXPORT_SYMBOL(has_cv_dsp3_iommu_map);
extern int has_cv_dsp2_iommu_map;
extern int has_cv_dsp3_iommu_map;
/*******************************************************************************
 * bst_cv FIRMWARE
 ******************************************************************************/
#ifdef BST_CV_DEBUG
/*
 * @func    _dump_firmware
 * @brief   This is a debug function to dump the firmware buffer.
 * @params  fw_buf - the pointer to the firmware buffer
 *          size - the dumped size
 * @return  void
 */
static void _dump_firmware(char *fw_buf, int size)
{
	int i;
	for (i = 0; i < size / BST_CV_FIRMWARE_DUMP_LINE_SIZE; i++) {
		BST_CV_TRACE_PRINTK("%02x %02x %02x %02x %02x %02x %02x %02x",
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 1),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 2),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 3),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 4),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 5),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 6),
				    *(fw_buf +
				      i * BST_CV_FIRMWARE_DUMP_LINE_SIZE + 7));
	}
	return;
}
#endif

/*
 * @func    _load_firmware_elf
 * @brief   load elf firmware
 * @params  pbst_cv - the pointer to the bst_cv device
 *          fw - the pointer to the firmware file
 * @return  0 - success
 *          error code - failure
 */
static int _load_firmware_elf(struct bst_cv *pbst_cv, int dsp_id,
			      struct firmware *fw)
{
	Elf32_Ehdr *ehdr = (Elf32_Ehdr *) fw->data;
	struct bst_cv_dsp_fw_ctl *dsp = &pbst_cv->fw_manager.dsps[dsp_id];
	int i;
	void __iomem *membase;
	u32 dsp_ram_base;
	Elf32_Phdr *phdr = NULL;

	//for (i = 0; i < ehdr->e_phnum; ++i) {
	for (i = ehdr->e_phnum; i >= 0; i--) {
		phdr = (void *)fw->data + ehdr->e_phoff + i * ehdr->e_phentsize;

		/* Only load non-empty loadable segments, R/W/X */
		if (!(phdr->p_type == PT_LOAD &&
		      (phdr->p_flags & (PF_X | PF_R | PF_W)) &&
		      phdr->p_memsz > 0))
			continue;

		if (phdr->p_offset >= fw->size ||
		    phdr->p_offset + phdr->p_filesz > fw->size) {
			BST_CV_TRACE_PRINTK
			    ("bad firmware ELF program header entry %d\n", i);
			return -EINVAL;
		}

		if ((phdr->p_paddr >= dsp->fwmem_phys_addr)
		    && (phdr->p_paddr + phdr->p_memsz <=
			dsp->fwmem_phys_addr + dsp->fwmem_size)) {
			membase =
			    dsp->fwmem_base + (phdr->p_paddr -
					       dsp->fwmem_phys_addr);
			BST_CV_TRACE_PRINTK
			    ("segment %d phys base: 0x%x remap base 0x%px size 0x%x loading",
			     i, phdr->p_paddr, (void __iomem *)membase,
			     phdr->p_filesz);
			memcpy_toio(membase, fw->data + phdr->p_offset,
				    phdr->p_filesz);
			BST_CV_TRACE_PRINTK("segment %d load success", i);
		} else {
			if ((phdr->p_paddr & 0xff000000) == BST_CV_DSP_RAM_BASE) {
				dsp_ram_base =
				    BST_CV_DSP_RAM_BASE +
				    BST_CV_DSP_RAM_BASE_OFFSET +
				    (BST_CV_DSP_RAM_BASE_INTERVAL * dsp_id);
				dsp_ram_base += (phdr->p_paddr & 0x000fffff);
				membase = ioremap(dsp_ram_base, phdr->p_memsz);
				BST_CV_TRACE_PRINTK
				    ("segment %d phys base: 0x%x remap base 0x%px size 0x%x loading",
				     i, dsp_ram_base, (void __iomem *)membase,
				     phdr->p_filesz);
				memcpy_toio(membase, fw->data + phdr->p_offset,
					    phdr->p_filesz);
				iounmap(membase);
				BST_CV_TRACE_PRINTK("segment %d load success",
						    i);
			} else {
				//if segment out of dsp ram
				BST_CV_TRACE_PRINTK
				    ("segment %d phys base: 0x%x size 0x%x out of range",
				     i, phdr->p_paddr, phdr->p_filesz);
				return -EINVAL;
			}
		}
	}
	return 0;
}

/*
 * @func    _check_firmware_is_elf
 * @brief   check the legality of elf files.
 * @params  fw - the pointer to the firmware file
 * @return  0 - binary file
 *          1 - elf file
 */
static int _check_firmware_is_elf(struct firmware *fw)
{
	Elf32_Ehdr *ehdr = (Elf32_Ehdr *) fw->data;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG))
		return 0;

	if (ehdr->e_type != ET_EXEC)
		return 0;

	if (ehdr->e_machine != 94 /*EM_XTENSA */ )
		return 0;

	if (ehdr->e_phoff >= fw->size ||
	    ehdr->e_phoff + ehdr->e_phentsize * ehdr->e_phnum > fw->size) {
		return 0;
	}

	return 1;
}

/*
 * @func    _load_firmware
 * @brief   This function loads the firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int _load_firmware(struct bst_cv *pbst_cv, int i, struct firmware *fw)
{
	if (pbst_cv->fw_manager.dsps[i].fwmem_size < fw->size) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "firmware too large for DSP %d", i);
		return -EINVAL;
	}

	BST_CV_STAGE_PRINTK("firmware mem base: 0x%px, size: %lld",
			    pbst_cv->fw_manager.dsps[i].fwmem_base,
			    pbst_cv->fw_manager.dsps[i].fwmem_size);
	BST_CV_STAGE_PRINTK("firmware data: 0x%px, size: %ld", fw->data,
			    fw->size);

	//check firmware is elf file or binary file
	if (_check_firmware_is_elf(fw)) {
		BST_CV_STAGE_PRINTK("load elf firmware for DSP %d", i);
		return _load_firmware_elf(pbst_cv, i, fw);
	} else {
		BST_CV_STAGE_PRINTK("load rbf(binary) firmware for DSP %d", i);
		memcpy_toio(pbst_cv->fw_manager.dsps[i].fwmem_base, fw->data,
			    fw->size);
	}

#ifdef BST_CV_DEBUG
	_dump_firmware(pbst_cv->fw_manager.dsps[i].fwmem_base,
		       BST_CV_FIRMWARE_DUMP_SIZE);
#endif

	return 0;
}

/*
 * @func    _pre_load_firmware
 * @brief   Preparation before loading completion.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
static void _pre_load_firmware(struct bst_cv *pbst_cv, int i)
{
	uint32_t reg;
	uint32_t cv_parity_en = 0;

	//disable cv parity
	reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0);
	if (cv_parity_en) {
		reg |= 0x3;	// [1:0] - cv_internal_pty_en, cv_sram_ecc_en
		writel_relaxed(reg,
		    pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0);
	} else {
		reg &= ~0x3;
		writel_relaxed(reg,
			pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_CV_PARITY_CTRL_REG0);
	}

	//disable cv chk intr en
	reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_CLUSTER_INTR_EN_REG);
	reg &= ~(0x7 << (16 + pbst_cv->dsp_indices[i]*3));
	writel_relaxed(reg,
				   pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_DSP_CLUSTER_INTR_EN_REG);

	//setup reset vector
	BST_CV_STAGE_PRINTK("set vector: 0x%x, addr: 0x%px",
			    addr_truncate(phys_to_bus(
					   pbst_cv->fw_manager.dsps[i].
					   fwmem_phys_addr)),
			    pbst_cv->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET +
			    i * BST_CV_REG_WIDTH);
	writel_relaxed(addr_truncate
		       (phys_to_bus(pbst_cv->fw_manager.dsps[i].fwmem_phys_addr)),
		       pbst_cv->fw_manager.lb_cv_reg_base +
		       LB_CV_REG_R_CV_DSP_ALT_RESET_VEC_OFFSET +
		       i * BST_CV_REG_WIDTH);

	reg =
	    readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base +
			  LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	//Attention
	//all clocks must be enabled, otherwise, the DSP cannot load elf firmware
	reg |= (0x0F << (BST_CV_CLK_EN_BIT));	//clk enable
	writel_relaxed(reg,
		       pbst_cv->fw_manager.lb_cv_reg_base +
		       LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg |= (1 << (BST_CV_RUNSTALL_BIT + i));	//dsp stop
	reg |= (1 << (BST_CV_SOFT_RESET_BIT + i));	//reset disable
	writel_relaxed(reg,
		       pbst_cv->fw_manager.lb_cv_reg_base +
		       LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg =
	    readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base +
			  LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
}

/*
 * @func    _post_load_firmware_success
 * @brief   End work after loading is completed.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
static void _post_load_firmware_success(struct bst_cv *pbst_cv, int i)
{
	uint32_t reg;
	//start dsp
	reg =
	    readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base +
			  LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg |= (1 << (BST_CV_SOFT_RESET_BIT + i));	//reset disable
	reg |= (1 << (BST_CV_CLK_EN_BIT + i));	//clk enable
	reg &= ~(1 << (BST_CV_RUNSTALL_BIT + i));	//dsp run
	writel_relaxed(reg,
		       pbst_cv->fw_manager.lb_cv_reg_base +
		       LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	BST_CV_STAGE_PRINTK("set ctrl value: 0x%x, addr: 0x%px", reg,
			    pbst_cv->fw_manager.lb_cv_reg_base +
			    LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	return;
}

/*
 * @func    _post_load_firmware_failed
 * @brief   End work after loading is completed.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
static void _post_load_firmware_failed(struct bst_cv *pbst_cv, int i)
{
	uint32_t reg;
	//stop dsp
	reg =
	    readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base +
			  LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	reg &= ~(1 << (BST_CV_SOFT_RESET_BIT + i));	//reset enable
	reg &= ~(1 << (BST_CV_CLK_EN_BIT + i));	//clk disable
	reg |= (1 << (BST_CV_RUNSTALL_BIT + i));	//dsp stop
	writel_relaxed(reg,
		       pbst_cv->fw_manager.lb_cv_reg_base +
		       LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	return;
}

/*
 * @func    bst_cv_boot_firmware
 * @brief   This function loads the runtime firmware and boots it up.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
static int bst_cv_boot_firmware(struct bst_cv *pbst_cv)
{
	int ret = 0;
	int i;
	struct firmware *fw;
	char *firmware_name = NULL;
	int firmware_id;
	bool wdt_config_flag = false;
	bool firmware_load = false;

	firmware_name = kzalloc(BST_CV_FIRMWARE_NAME_MAX_SIZE, GFP_KERNEL);
	if (firmware_name == NULL) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "alloc firmware name err.");
		goto err_exit;
	}

	for (i = 0; i < BST_CV_DSP_NUM; i++) {
		if (pbst_cv->dsp_online[i]) {
			firmware_load = false;
			for (firmware_id = 0; firmware_id < 1; firmware_id++) {
				memset(firmware_name, 0,
				       BST_CV_FIRMWARE_NAME_MAX_SIZE);
				strcpy(firmware_name,
				       pbst_cv->fw_manager.dsps[i].name);
				strcat(firmware_name,
				       bst_cv_support_firmware_suffix
				       [firmware_id]);
				BST_CV_STAGE_PRINTK("request");
				ret =
				    request_firmware((const struct firmware **)
						     &fw, firmware_name,
						     &pbst_cv->pdev->dev);
				if (ret < 0) {
					continue;
				}
				BST_CV_STAGE_PRINTK
				    ("bst_cv firmware(%s) requested for DSP %d",
				     firmware_name, i);

				_pre_load_firmware(pbst_cv, i);
				ret = _load_firmware(pbst_cv, i, fw);
				release_firmware(fw);
				firmware_load = true;
				if (ret < 0) {
					continue;
				}
				break;
			}

			if (firmware_load == true) {
				BST_CV_STAGE_PRINTK("firmware_load");
				if (ret < 0) {
					BST_CV_STAGE_PRINTK("firmware_load<0");
					pbst_cv->dsp_online[i] = 0;
					_post_load_firmware_failed(pbst_cv, i);
				} else {
					if (wdt_config_flag == false) {
						// wdt_cv_init();
						// wdt_cv_config(WDT_BST_CV_DSP_ID, WDT_PING_TIME_DEFAULT);
						wdt_config_flag = true;
					}
					//_boot_firmware(pbst_cv, i);
					BST_CV_STAGE_PRINTK
					    ("firmware_load else");
					_post_load_firmware_success(pbst_cv, i);
					pbst_cv->fw_manager.dsps[i].boot = 1;
					BST_CV_STAGE_PRINTK
					    ("bst_cv firmware(%s) booted for DSP %d",
					     firmware_name, i);
				}
			}
		}
	}
	kfree(firmware_name);
	firmware_name = NULL;
	return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;

err_exit:
	return -EFAULT;
}

/*
 * @func    bst_cv_fw_manager_init
 * @brief   This function initializes the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_fw_manager_init(struct bst_cv *pbst_cv)
{
	int i;
	int ret;
	struct resource *res;
	uint32_t assigned_mem_size;
	struct device_node *bst_cv_node, *dsp_node;

	bst_cv_node = pbst_cv->pdev->dev.of_node;
	//map registers
	res = platform_get_resource(pbst_cv->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "failed to retrieve cv register");
		ret = -ENODEV;
		return ret;
	}
	pbst_cv->fw_manager.lb_cv_reg_base = devm_ioremap(&pbst_cv->pdev->dev,
							  res->start,
							  resource_size(res));
	if (IS_ERR(pbst_cv->fw_manager.lb_cv_reg_base)) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "failed to remap cv register");
		return PTR_ERR(pbst_cv->fw_manager.lb_cv_reg_base);
	}
	//get the ipc register address
	ret = device_property_read_u32(&pbst_cv->pdev->dev, "ipc-register-addr",
				       &pbst_cv->fw_manager.ipc_register_addr);
	if (ret == -EINVAL || ret == -ENODATA) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "no ipc-register-addr property");
		return ret;
	} else if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "invalid ipc-register-addr property");
		return ret;
	}
	//get assigned memory sizes
	ret = device_property_read_u32(&pbst_cv->pdev->dev, "assigned-mem-size",
				       &assigned_mem_size);
	if (ret == -EINVAL || ret == -ENODATA) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "no assigned-mem-size property");
		return ret;
	} else if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "invalid assigned-mem-size");
		return ret;
	}
	//assign memory for firmware handshake
	pbst_cv->fw_manager.assigned_mem =
	    pbst_cv->mem_manager.ops->alloc(pbst_cv, assigned_mem_size, 0, 0);
	if (pbst_cv->fw_manager.assigned_mem == NULL) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "could not allocate assigned-mem");
		ret = -ENOMEM;
		return ret;
	}

	//get the dsp number
	ret = device_property_read_u32(&pbst_cv->pdev->dev, "dsp-num",
				       &pbst_cv->dsp_num);
	if (ret == -EINVAL || ret == -ENODATA) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "no dsp-num property");
		return ret;
	} else if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev, "invalid dsp-num property");
		return ret;
	}

	if (pbst_cv->dsp_num != of_get_child_count(bst_cv_node)) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "inconsistent DSP number in device tree");
		return -EFAULT;
	}

	if (bst_cv_dspcnt > pbst_cv->dsp_num) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "invalid bst_cv_dspcnt %d > pbst_cv->dsp_num %d",
			       bst_cv_dspcnt, pbst_cv->dsp_num);
		return -EFAULT;
	}

	i = 0;
	for_each_child_of_node(bst_cv_node, dsp_node) {
		if (i >= bst_cv_dspcnt) {
			BST_CV_STAGE_PRINTK("%d dsp do not start.", i);
			goto cur_iter_failure;
		}
		//get the CV DSP index
		ret =
		    of_property_read_u32(dsp_node, "index",
					 &pbst_cv->dsp_indices[i]);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "no index property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "invalid index property");
			goto cur_iter_failure;
		}
		BST_CV_TRACE_PRINTK("CV DSP index of DSP %d: %d", i,
				    pbst_cv->dsp_indices[i]);

		//get the DSP initialization address
		ret = of_property_read_u32(dsp_node, "rt-init-addr",
					   &pbst_cv->fw_manager.dsps[i].
					   rt_init_addr);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "no rt-init-addr property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "invalid rt-init-addr property");
			goto cur_iter_failure;
		}
		BST_CV_TRACE_PRINTK("rt-init-addr of DSP %d: 0x%x", i,
				    pbst_cv->fw_manager.dsps[i].rt_init_addr);

		//get the firmware name
		ret = of_property_read_string(dsp_node, "firmware",
					      (const char **)&pbst_cv->
					      fw_manager.dsps[i].name);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "no firmware property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "invalid firmware property");
			goto cur_iter_failure;
		}
		BST_CV_TRACE_PRINTK("firmware of DSP %d: %s", i,
				    pbst_cv->fw_manager.dsps[i].name);

		//get the firmware name
		ret = of_property_read_u32(dsp_node, "ipc-src-core",
					   &pbst_cv->fw_manager.dsps[i].
					   ipc_src_core);
		if (ret == -EINVAL || ret == -ENODATA) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "no ipc-src-core property");
			goto cur_iter_failure;
		} else if (ret < 0) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "invalid ipc-src-core property");
			goto cur_iter_failure;
		}
		//map firmware memory
		res =
		    platform_get_resource(pbst_cv->pdev, IORESOURCE_MEM, i + 1);
		if (!res) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "could not get firmware resource for DSP %d",
				       i);
			goto cur_iter_failure;
		}
		BST_CV_TRACE_PRINTK
		    ("firmware mem of DSP %d start: 0x%llx, end: 0x%llx", i,
		     res->start, res->end);

		pbst_cv->fw_manager.dsps[i].fwmem_base =
		    devm_ioremap(&pbst_cv->pdev->dev, res->start,
				 resource_size(res));
		if (IS_ERR(pbst_cv->fw_manager.dsps[i].fwmem_base)) {
			BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
				       "failed to remap firmware memory: %ld",
				       PTR_ERR(pbst_cv->fw_manager.dsps[i].
					       fwmem_base));
			goto cur_iter_failure;
		}
		pbst_cv->fw_manager.dsps[i].fwmem_size = resource_size(res);
		pbst_cv->fw_manager.dsps[i].fwmem_phys_addr = res->start;
		BST_CV_TRACE_PRINTK
		    ("firmware mem base for DSP %d: 0x%px, size: %llx", i,
		     pbst_cv->fw_manager.dsps[i].fwmem_base,
		     pbst_cv->fw_manager.dsps[i].fwmem_size);

		if (pbst_cv->mem_manager.enable_smmu) {
			if (i == 0 && has_cv_dsp2_iommu_map != 0) {
				goto skip_iommu_map;
			}
			if (i == 1 && has_cv_dsp3_iommu_map != 0) {
				goto skip_iommu_map;
			}
			pbst_cv->mem_manager.ops->iommu_bypass(
				pbst_cv,
				pbst_cv->fw_manager.dsps[i].fwmem_size,
				PAGE_SIZE,
				pbst_cv->fw_manager.dsps[i].fwmem_phys_addr,
				IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV);
			if (i == 0) {
				has_cv_dsp2_iommu_map = 1;
			}
			if (i == 1) {
				has_cv_dsp3_iommu_map = 1;
			}
			BST_CV_STAGE_PRINTK("iommu_bypass ok: 0x%lx", (unsigned long)pbst_cv->fw_manager.dsps[i].fwmem_phys_addr);
		}
skip_iommu_map:

		//check rt_init_addr
		if (pbst_cv->fw_manager.dsps[i].rt_init_addr < phys_to_bus(res->start)
		 || pbst_cv->fw_manager.dsps[i].rt_init_addr > phys_to_bus(res->end)) {
			BST_CV_DEV_ERR(
				&pbst_cv->pdev->dev,
				"rt_init_addr= %x, phys_to_bus start=%llx, phys_to_bus end=%llx",
				pbst_cv->fw_manager.dsps[i].rt_init_addr,
				phys_to_bus(res->start),
				phys_to_bus(res->end)
			);
			BST_CV_DEV_ERR(
				&pbst_cv->pdev->dev,
				"rt_init_addr of DSP %d is outside of firmware memory",
				i
			);
			goto cur_iter_failure;
		}
		//assign memory for firmware
		pbst_cv->fw_manager.dsps[i].sync_virt_base =
		    pbst_cv->fw_manager.assigned_mem->kern_addr +
		    BST_CV_HANDSHAKE_BUF_SIZE * i;
		BST_CV_STAGE_PRINTK(
			"---------------------DSP %d handshake buffer: 0x%px, size: 0x%x",
			i,
			pbst_cv->fw_manager.dsps[i].sync_virt_base,
			BST_CV_HANDSHAKE_BUF_SIZE
		);

		pbst_cv->dsp_online[i] = 1;
		pbst_cv->fw_manager.dsps[i].init = 1;
		goto loop_continue;

cur_iter_failure:
		pbst_cv->dsp_online[i] = 0;
		pbst_cv->fw_manager.dsps[i].init = 0;
loop_continue:
		i++;
	}
	return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    _dump_xrp_dsp_sync
 * @brief   This function dumps the xrp_dsp_sync structure used in the handshake
 *          with the firmware
 * @params  pbst_cv - the pointer to the bst_cv device
 *          xrp_dsp_sync_base - the base address of the xrp_dsp_sync structure
 * @return  void
 */
static inline void _dump_xrp_dsp_sync(struct bst_cv *pbst_cv,
				      struct xrp_dsp_sync *xrp_dsp_sync_base)
{
	int j;

	for (j = 0; j < (sizeof(*xrp_dsp_sync_base) + 15) / 16; j++) {
		BST_CV_DEV_ERR(
			&pbst_cv->pdev->dev,
			"0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x",
			kern_to_bus(pbst_cv,
						xrp_dsp_sync_base,
						pbst_cv->fw_manager.assigned_mem) + j * 16,
			*((uint32_t *) xrp_dsp_sync_base + j * 4),
			*((uint32_t *) xrp_dsp_sync_base + j * 4 + 1),
			*((uint32_t *) xrp_dsp_sync_base + j * 4 + 2),
			*((uint32_t *) xrp_dsp_sync_base + j * 4 + 3));
	}
	return;
}

/*
 * @func    bst_cv_fw_rt_setup
 * @brief   This function initializes the runtime firmware. Because it relies on
 *          messaging and the message manager can only be initialized after
 *          getting some required firmware information in bst_cv_fw_manager_init,
 *          this part is separated from the firmware manager initialization.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  0 - success
 *          error code - failure
 */
int bst_cv_fw_rt_setup(struct bst_cv *pbst_cv)
{
	int i, count;
	int ret = 0;
	struct xrp_dsp_sync *xrp_dsp_sync_base;
	uint32_t data;
	int reg;

	//boot firmware
	ret = bst_cv_boot_firmware(pbst_cv);
	if (ret < 0) {
		BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
			       "bst_cv_boot_firmware failed for all DSPs");
		return ret;
	}
	BST_CV_STAGE_PRINTK("bst_cv_boot_firmware OK");

	reg = readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base +
			  LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	BST_CV_STAGE_PRINTK("reg = %x", reg);
	for (i = 0; i < BST_CV_DSP_NUM; i++) {
		if (pbst_cv->dsp_online[i]) {
			//sync setup(really stupid and unnecessary step from firmware side)
			xrp_dsp_sync_base =
			    pbst_cv->fw_manager.dsps[i].sync_virt_base;
			xrp_dsp_sync_base->device_mmio_base = 0;
			xrp_dsp_sync_base->host_irq_mode =
			    XRP_DSP_SYNC_IRQ_MODE_BST_IPC;
			xrp_dsp_sync_base->host_irq_offset = 0;
			xrp_dsp_sync_base->host_irq_bit = IPC_CORE_CV0 + i;	/*  */
			xrp_dsp_sync_base->device_irq_mode =
			    XRP_DSP_SYNC_IRQ_MODE_BST_IPC;
			xrp_dsp_sync_base->device_irq_offset = 0;
			xrp_dsp_sync_base->device_irq_bit = IPC_CORE_ARM3;
			xrp_dsp_sync_base->device_irq =
			    ipc_device_irq_indices[i];
			xrp_dsp_sync_base->host_ipc_irq_clear =
			    ipc_host_irq_clear_addr;
			xrp_dsp_sync_base->host_irq = 0x91;
			xrp_dsp_sync_base->device_ipc_irq_clear =
			    ipc_dsp_irq_clear_addrs[i];
			xrp_dsp_sync_base->host_ipc_irq_trig =
			    ipc_dsp_irq_trigger_addr + i * 0x100;
			xrp_dsp_sync_base->device_ipc_irq_trig =
			    0x47102328 + i * 4;
			xrp_dsp_sync_base->debug_buffer_base =
			    bst_cv_firmware_log_buffer +
			    i * (bst_cv_firmware_log_length / BST_CV_DSP_NUM);
			xrp_dsp_sync_base->debug_buffer_length =
			    (bst_cv_firmware_log_length / BST_CV_DSP_NUM);
			xrp_dsp_sync_base->debug_level =
			    bst_cv_firmware_log_level;
			//TODO, Fixme
			xrp_dsp_sync_base->ipc_host_to_dsp_addr =
			    0x8ff00000 +
			    0x40 * pbst_cv->fw_manager.dsps[i].ipc_src_core;
			xrp_dsp_sync_base->ipc_dsp_to_host_addr =
			    0x8ff00000 + 0x40 * (IPC_CORE_CV0 + i);
			xrp_dsp_sync_base->device_ipc_irq_enable =
			    ipc_dsp_ireq_enable_addrs[i];
			xrp_dsp_sync_base->device_ipc_irq_enable_mask =
			    (1 << 30) | (1 << IPC_CORE_ARM3);

			BST_CV_STAGE_PRINTK
			    ("DSP %d log buffer: base %#x, length %#x, level %d",
			     i, xrp_dsp_sync_base->debug_buffer_base,
			     xrp_dsp_sync_base->debug_buffer_length,
			     xrp_dsp_sync_base->debug_level);

#ifdef BST_CV_DEBUG
			_dump_xrp_dsp_sync(pbst_cv, xrp_dsp_sync_base);
#endif
			//start handshaking
			xrp_dsp_sync_base->sync = XRP_DSP_SYNC_START;
			count = 0;
			while (xrp_dsp_sync_base->sync != XRP_DSP_SYNC_DSP_READY
			       && count < BST_CV_HANDSHAKE_RETRY_NUM) {
				msleep(BST_CV_HANDSHAKE_SLEEP_INTERVAL);
				count++;
			}
			BST_CV_STAGE_PRINTK("xrp_dsp_sync_base->sync = %x",
					    xrp_dsp_sync_base->sync);
			if (count == BST_CV_HANDSHAKE_RETRY_NUM) {
				BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
					       "failed to wait for handshake ready from DSP %d",
					       i);
				pbst_cv->dsp_online[i] = 0;
				continue;
			}
			xrp_dsp_sync_base->sync = XRP_DSP_SYNC_HOST_TO_DSP;
			count = 0;
			while (xrp_dsp_sync_base->sync !=
			       XRP_DSP_SYNC_DSP_TO_HOST
			       && count < BST_CV_HANDSHAKE_RETRY_NUM) {
				msleep(BST_CV_HANDSHAKE_SLEEP_INTERVAL);
				count++;
			}
			BST_CV_STAGE_PRINTK("xrp_dsp_sync_base->sync2 = %x",
					    xrp_dsp_sync_base->sync);
			if (count == BST_CV_HANDSHAKE_RETRY_NUM) {
				BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
					       "failed to wait for handshake sync from DSP %d",
					       i);
				pbst_cv->dsp_online[i] = 0;
				continue;
			}
			ret = bst_cv_msg_send(pbst_cv, i, 0);
			if (ret < 0) {
				BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
					       "failed to send interrupt handshake to DSP %d, ret %d",
					       i, ret);
				pbst_cv->dsp_online[i] = 0;
				continue;
			}
			if (bst_cv_msg_recv
			    (pbst_cv, i, &data, BST_CV_HANDSHAKE_TIMEOUT) < 0) {
				BST_CV_DEV_ERR(&pbst_cv->pdev->dev,
					       "ipc timeout, failed to receive interrupt handshake from DSP %d",
					       i);
				pbst_cv->dsp_online[i] = 0;
				continue;
			}
		}
	}

	bst_cv_fw_rt_cleanup(pbst_cv);
	return bst_cv_check_online(pbst_cv) ? 0 : -EFAULT;
}

/*
 * @func    bst_cv_fw_manager_cleanup
 * @brief   This function cleans up the resources allocated in firmware
 *          manager initialization for subsequential failure of DSPs or even
 *          the driver during the entire initialization process.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_manager_cleanup(struct bst_cv *pbst_cv)
{
	int i;
	int online_bit = 0;
	for (i = 0; i < BST_CV_DSP_NUM; i++) {
		online_bit |= pbst_cv->dsp_online[i] << i;
	}
	if (online_bit == 0) {
		pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.
					       assigned_mem);
		pbst_cv->fw_manager.assigned_mem = NULL;
	}
	return;
}

/*
 * @func    _release_rt_firmware
 * @brief   This function resets the specified DSP.
 * @params  pbst_cv - the pointer to the bst_cv device
 *          dsp - the CV DSP number
 * @return  void
 */
static inline void _release_rt_fw(struct bst_cv *pbst_cv, int dsp)
{
	uint32_t reg;

	reg =
	    readl_relaxed(pbst_cv->fw_manager.lb_cv_reg_base +
			  LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	//stall the DSP first
	//reg |= (1 << (BST_CV_RUNSTALL_BIT + dsp));
	//writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	//then reset the DSP
	reg &= ~(1 << (BST_CV_SOFT_RESET_BIT + dsp));
	// writel_relaxed(reg, pbst_cv->fw_manager.lb_cv_reg_base + LB_CV_REG_R_CV_SYS_CTRL_OFFSET);
	return;
}

/*
 * @func    bst_cv_fw_rt_cleanup
 * @brief   This function resets the DSPs for subsequential failure of DSPs or
 *          even the driver during the entire initialization process.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_rt_cleanup(struct bst_cv *pbst_cv)
{
	int i;

	for (i = 0; i < BST_CV_DSP_NUM; i++) {
		if (!pbst_cv->dsp_online[i] && pbst_cv->fw_manager.dsps[i].boot) {
			_release_rt_fw(pbst_cv, i);
		}
	}
	return;
}

/*
 * @func    bst_cv_fw_manager_exit
 * @brief   This is the exit function of the firmware manager.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_manager_exit(struct bst_cv *pbst_cv)
{
	int i = 0;
	for (; i < pbst_cv->dsp_num; ++i) {
		if (pbst_cv->mem_manager.enable_smmu) {
			pbst_cv->mem_manager.ops->iommu_free(
				pbst_cv,
				pbst_cv->fw_manager.dsps[i].fwmem_size,
				0,
				addr_truncate(phys_to_bus(pbst_cv->fw_manager.dsps[i].fwmem_phys_addr))
			);
			if (i == 0) {
				has_cv_dsp2_iommu_map = 0;
			}
			if (i == 1) {
				has_cv_dsp3_iommu_map = 0;
			}
		}
	}
	if (pbst_cv->fw_manager.assigned_mem) {
		pbst_cv->mem_manager.ops->free(pbst_cv->fw_manager.
					       assigned_mem);
	}
	return;
}

/*
 * @func    bst_cv_fw_rt_exit
 * @brief   This is the exit function of the runtime firmware.
 * @params  pbst_cv - the pointer to the bst_cv device
 * @return  void
 */
void bst_cv_fw_rt_exit(struct bst_cv *pbst_cv)
{
	int i;

	for (i = 0; i < BST_CV_DSP_NUM; i++) {
		if (pbst_cv->dsp_online[i]) {
			_release_rt_fw(pbst_cv, i);
		}
	}
	return;
}
