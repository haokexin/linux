// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/io.h>

#include <linux/coreip/proto_api_common.h>

#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_hw.h"

static const uint32_t ISP_SW_RESET = 0x30002184;
static const uint32_t REG_ISP_MAILBOX_IN = 0x52030090; // to store INIT physical
						       // address
static const uint32_t REG_ISP_MAILBOX_OUT = 0x52030094; // to store IPC message
							// physical address
static const uint32_t REG_CTRL1_ADDR = 0x52030000;
static const uint32_t REG_CTRL2_ADDR = 0x52030004;
static const uint32_t REG_RESET_ADDR = 0x52030004;
static const uint32_t REG_STATUS_ADDR = 0x52034280;
static const uint32_t ISP_FW_IMAGE_BASE = 0x52040000;
static const uint32_t IPC_REGISTER_ADDR = 0x8FF00000;

#define ISP_CTRL_R_CORENOC_PARITY_ENABLE (0x33000084)
#define CV_PARITY_EN (0x2)
#define ISP_PARITY_EN (0x8)
#define ISP_CORE_TOP_CTRL2C 0x5203002c
#define ISP_CORE_TOP_CTRL30 0x52030030

static int parse_elf(const u8 *data, int size, struct c1200_isp_device *isp);

static int enable_isp_ecc_and_parity(void)
{
	uint32_t *isp_ctrl30_reg;
	uint32_t status;

	isp_ctrl30_reg = (uint32_t *)ioremap(ISP_CORE_TOP_CTRL30, 0x4);
	if (isp_ctrl30_reg == NULL) {
		pr_err("ioremap(C1200 ISP_CORE_TOP_CTRL30 _REG failed\n");
		return -1;
	}

	status = *isp_ctrl30_reg;
	status = 0xffffffff;
	*isp_ctrl30_reg = status;
	status = *isp_ctrl30_reg;

	return 0;
}

int bst_load_isp_fw(const char *fw_name, const char *slab_name,
		    struct c1200_isp_device *isp)
{
	int ret;
	uint32_t val;
	const struct firmware *firmware_p;

	val = readl_relaxed(isp->ctrl + R_TOP_SW_RST);
	dev_err(isp->dev, "%s: request fw %s, 0x%08X\n", __func__, fw_name, val);
	/* Before we load Firmware, let RISC-V enter reset state to avoid exception */
	writel_relaxed(val & 0xFFFFFFBF, isp->ctrl + R_TOP_SW_RST);
	ret = request_firmware(&firmware_p, fw_name, isp->dev);
	if (ret) {
		dev_err(isp->dev, "There is no firmware %s could be used\n",
			fw_name);
		return -1;
	}

	ret = parse_elf(firmware_p->data, firmware_p->size, isp);
	if (ret) {
		dev_err(isp->dev, "Failed to parse firmware %s\n", fw_name);
		return -1;
	}
	release_firmware(firmware_p);

	dev_err(isp->dev, "%s: request slab %s\n", __func__, slab_name);
	ret = request_firmware(&firmware_p, slab_name, isp->dev);
	if (ret) {
		dev_err(isp->dev, "There is no slab %s could be used\n",
			slab_name);
		return -1;
	}
	memcpy(isp->slab_vaddr, firmware_p->data, firmware_p->size);
	release_firmware(firmware_p);

	return 0;
}

/*
 * target_freq: need to product fps
 * fsync_out: which to isp_fsync_pin to out target_freq
 * pulse_width: the pusle_width of target_freq
 * fsync_source: which source to product target_freq
 */

int isp_internal_trigger(struct c1200_isp_device *isp, int target_freq, int fsync_out, uint32_t pulse_width,
						 int fsync_source)
{
	uint32_t val;

	if (fsync_out < FSYNC_PIN_MIN || fsync_out > FSYNC_PIN_MAX) {
		pr_err("%s: Invalid internal FSYNC out pin %d\n", __func__,
		       fsync_out);
		return -EINVAL;
	}

	/* 1. Enable given source */
	val = readl_relaxed(isp->ctrl + REG_SRC_EN);
	writel_relaxed(val | (1 << fsync_source), isp->ctrl + REG_SRC_EN);

	/* 2. Set period */
	writel_relaxed(FREQ_TO_PERIOD(target_freq),
				   isp->ctrl + REG_INNER_SRC1_PERIOD + 4 * fsync_source);

	/* 3. Set pulse width */
	if (pulse_width == 0)
		pulse_width = FSYNC_DEFAULT_PULSE_WIDTH; /* 0x640*2.5ns = 4us */
	writel_relaxed(NS_TO_PERIOD(pulse_width),
				   isp->ctrl + REG_PULSE0_WIDTH + 4 * fsync_out);

	/* 4. Set INTx_SEL(From which internal counter) */
	val = readl_relaxed(isp->ctrl + REG_INTX_SEL);
	val &= ~(0x7 << (8 + 3 * fsync_out)); /* Clear INT_SELx first */
	val |= fsync_source << (8 + 3 * fsync_out); /* Set INT_SELx */
	writel_relaxed(val, isp->ctrl + REG_INTX_SEL);

	/* 5. Set FSYNCx_SEL to 0(From FSYNCx_inside), and FSYNx_OEN to
	 * 0(output)
	 */
	val = readl_relaxed(isp->ctrl + REG_FSYNX_OEN);
	/* Set FSYNCx_SEL to 0 for inner mode, FSYNCx_OEN to 1 for output */
	val &= ~(0x7 << (8 + 3 * fsync_out));
	val |= (0x1 << fsync_out);
	writel_relaxed(val, isp->ctrl + REG_FSYNX_OEN);

	return 0;
}

/*
 * external_freq: ouside lidar fps
 * target_freq: need fps
 * fsync_in: which isp_fsync_pin to receive lidar trigger
 * fsync_out: which to isp_fsync_pin to out target_freq
 * pulse_width: the pusle_width of target_freq
 * fsync_source: which source to product target_freq
 */
int isp_external_trigger(struct c1200_isp_device *isp, int external_freq, int target_freq, int fsync_in,
						 int fsync_out, uint32_t pulse_width, int fsync_source)
{
	uint32_t val;

	if (fsync_in < FSYNC_PIN_MIN || fsync_in > FSYNC_PIN_MAX) {
		pr_err("%s: Invalid external FSYNC in pin %d\n", __func__,
			   fsync_in);
		return -EINVAL;
	}

	if (fsync_out < FSYNC_PIN_MIN || fsync_out > FSYNC_PIN_MAX) {
		pr_err("%s: Invalid external FSYNC out pin %d\n", __func__,
			   fsync_out);
		return -EINVAL;
	}

	if (pulse_width == 0)
		pulse_width = FSYNC_DEFAULT_PULSE_WIDTH;

	/* 1.1 Enable given source */
	val = readl_relaxed(isp->ctrl + REG_SRC_EN);
	val |= (1 << (4 + fsync_source));
	/* 1.2 Set INC_OUTSIDEx */
	if (target_freq > external_freq)
		val |= (1 << (8 + fsync_source));
	else
		val &= ~(1 << (8 + fsync_source));

	/* 1.3 Set PULSEx_OUTSIDE_SEL */
	val &= ~(0x7 << (16 + 3 * fsync_source));	/* Clear bits first */
	val |= fsync_in << (16 + 3 * fsync_source); /* Set bits */
	writel_relaxed(val, isp->ctrl + REG_SRC_EN);
	if (target_freq > external_freq) {
		/* 2. Set OUT_SRCx_PERIOD */
		writel_relaxed(FREQ_TO_PERIOD(target_freq),
					   isp->ctrl + REG_OUTER_SRC1_PERIOD + fsync_source * 4);
		/*auto skew 1 will be same to fad,out_org_mult for raise fps*/
		val = readl_relaxed(isp->ctrl + REG_ORG_MULT + fsync_source * 4);
		val |= FREQ_TO_PERIOD(target_freq) * (target_freq / external_freq);
		writel_relaxed(val, isp->ctrl + REG_ORG_MULT + fsync_source * 4);
	} else {
		/* 2. Set DECREASE_PERIODx */
		uint32_t ratio;

		/* NOTE: must dived by 2 first to get correct register offset */
		val = readl_relaxed(isp->ctrl + REG_DECREASE_PERIOD1 +
							fsync_source / 2 * 4);
		val &= ~(0xFFFF << (fsync_source % 2 * 16)); /* Clear bits */
		ratio = external_freq / target_freq;
		val |= ratio << (fsync_source % 2 * 16); /* Set bits */
		writel_relaxed(val, isp->ctrl + REG_DECREASE_PERIOD1 +
								fsync_source / 2 * 4);
	}

	/* 3. Set pulse width */
	writel_relaxed(NS_TO_PERIOD(pulse_width),
				   isp->ctrl + REG_PULSE0_WIDTH + fsync_out * 4);
	/* 4. Set INTx_SEL, map source to outside_gen_fsyncX */
	val = readl_relaxed(isp->ctrl + REG_INTX_SEL);
	val &= ~(0x7 << (8 + 3 * fsync_out));			  /* Clear INT_SELx first */
	val |= (fsync_source + 2) << (8 + 3 * fsync_out); /* Set INT_SELx */
	writel_relaxed(val, isp->ctrl + REG_INTX_SEL);
	/* 5. Set FSYNCx_SEL to 0(From FSYNCx_inside), and FSYNx_OEN to 0(output) */
	val = readl_relaxed(isp->ctrl + REG_FSYNX_OEN);
	/* Set FSYNCx_SEL to 0 for FSYNCx_inside mode */
	val &= ~(0x7 << (8 + 3 * fsync_out));
	val |= (1 << fsync_out);   /* Set FSYNCx_OEN to 1 for input modify*/
	val &= ~(0x1 << fsync_in); /* Set FSYNCx_OEN to 0 for output modify*/
	writel_relaxed(val, isp->ctrl + REG_FSYNX_OEN);

	return 0;
}

int isp_fsync_work(struct c1200_isp_device *isp_dev)
{
	int ret;
	int source;
	int tx_pin;
	int rx_pin;
	int tx_fps;
	int rx_fps;
	int pulse_width;

	struct device_node *node = isp_dev->dev->of_node;
	struct device_node *fsync_dt = NULL;

	for_each_child_of_node(node, fsync_dt) {
		if (!fsync_dt->name || of_node_cmp(fsync_dt->name, "isp-fsync"))
			continue;
		ret = of_property_read_s32(fsync_dt, "source", &source);
		if (ret < 0 || source > FSYNC_OUTER_SRC_MAX || source < FSYNC_INNER_SRC_MIN)
			return -EINVAL;
		ret = of_property_read_s32(fsync_dt, "tx-pin", &tx_pin);
		if (ret < 0)
			return -EINVAL;
		ret = of_property_read_s32(fsync_dt, "rx-pin", &rx_pin);

		ret = of_property_read_s32(fsync_dt, "tx-fps", &tx_fps);
		if (ret < 0)
			return -EINVAL;
		ret = of_property_read_s32(fsync_dt, "rx-fps", &rx_fps);

		ret = of_property_read_s32(fsync_dt, "pulse-width", &pulse_width);
		if (ret < 0)
			pulse_width = FSYNC_DEFAULT_PULSE_WIDTH;

		if (source >= FSYNC_INNER_SRC_MIN && source <= FSYNC_INNER_SRC_MAX)
			isp_internal_trigger(isp_dev, tx_fps, tx_pin, pulse_width, source);
		else {
			/*need to write rx-fps = <10>;rx-pin = <2>;;pulse-width = <4000> */
			source -= FSYNC_OUTER0;
			isp_external_trigger(isp_dev, rx_fps, tx_fps, rx_pin, tx_pin, pulse_width, source);
		}
	}

	return 0;
}

void bst_start_isp_fw(struct c1200_isp_device *isp)
{
	uint32_t *reg_ctrl1;
	uint32_t *reg_reset;
	uint32_t *reg_vaddr;
	uint32_t reg_val;

	uint64_t reg_ctrl1_addr;
	uint64_t reg_reset_addr;

	void __iomem *corenoc_parity_enable = NULL;
	uint32_t isp_parity_en;

	reg_ctrl1_addr = REG_CTRL1_ADDR;
	reg_reset_addr = REG_RESET_ADDR;
	dev_err(isp->dev, "start fw !!!\n");
	if (0) {
		corenoc_parity_enable = devm_ioremap(
			isp->dev, ISP_CTRL_R_CORENOC_PARITY_ENABLE, 0x4);
		if (corenoc_parity_enable == NULL) {
			dev_err(isp->dev,
				"IOREMAP ISP_CTRL_R_CORENOC_PARITY_ENABLE 0x%8X FAILED\n",
				ISP_CTRL_R_CORENOC_PARITY_ENABLE);
			return;
		}
		isp_parity_en = readl_relaxed(corenoc_parity_enable) &
				ISP_PARITY_EN;
		devm_iounmap(isp->dev, corenoc_parity_enable);

		if (isp_parity_en)
			enable_isp_ecc_and_parity();
	}
	/* write INIT physical addr */
	reg_vaddr = (uint32_t *)ioremap(REG_ISP_MAILBOX_IN, 4);
	if (reg_vaddr == NULL) {
		dev_err(isp->dev, "IOREMAP REG_ISP_MAILBOX_IN 0x%x FAILED\n",
			REG_ISP_MAILBOX_IN);
		return;
	}
	reg_val = ((0x80000000 + isp->init_paddr) & LOW_32_BIT_MASK);
	*reg_vaddr = reg_val;

	dev_err(isp->dev, "init_paddr is 0x%08x !!!\n", reg_val);
	reg_val = 0;
	reg_val = readl_relaxed(reg_vaddr);
	dev_err(isp->dev, "read msg box in reg = 0x%08x !!!\n", reg_val);
	/* write IPC Message Start Addr */
	reg_vaddr = (uint32_t *)ioremap(REG_ISP_MAILBOX_OUT, 4);
	if (reg_vaddr == NULL) {
		dev_err(isp->dev, "IOREMAP REG_ISP_MAILBOX_OUT 0x%x FAILED\n",
			REG_ISP_MAILBOX_OUT);
		return;
	}
	*reg_vaddr = IPC_REGISTER_ADDR;

	/* write ctrl1_reg */
	reg_ctrl1 = (uint32_t *)ioremap(reg_ctrl1_addr, 4);
	if (reg_ctrl1 == NULL) {
		dev_err(isp->dev, "IOREMAP reg_ctrl10x%llx FAILED\n",
			reg_ctrl1_addr);
		return;
	}
	*reg_ctrl1 = 0xffffffff;

	/* write reset_reg */
	reg_reset = (uint32_t *)ioremap(reg_reset_addr, 4);
	if (reg_reset == NULL)
		dev_err(isp->dev, "IOREMAP reg_reset 0x%llx FAILED\n",
			reg_reset_addr);
	else
		*reg_reset = 0xffffffff;
}

int bst_boot_isp_fw(const char *fw_name, const char *slab_name,
		    struct c1200_isp_device *isp)
{
	int ret;

	ret = bst_load_isp_fw(fw_name, slab_name, isp);
	if (ret)
		return ret;

	bst_start_isp_fw(isp);
	return 0;
}

static int parse_elf32(const u8 *data, int size, struct c1200_isp_device *isp)
{
	int i;
	const u8 *dst_ptr;
	const u8 *src_ptr;
	u64 phy_addr;

	Elf32_Ehdr *ehdr = (Elf32_Ehdr *)data;
	Elf32_Shdr *shdr = (Elf32_Shdr *)(data + ehdr->e_shoff);
	Elf32_Shdr *shstr =
		(Elf32_Shdr *)((char *)shdr +
			       ehdr->e_shstrndx * ehdr->e_shentsize);
	const char *shstr_tbl = data + shstr->sh_offset;

	for (i = 0; i < ehdr->e_shnum; i++) {
		Elf32_Shdr *sptr =
			(Elf32_Shdr *)((char *)shdr + i * ehdr->e_shentsize);

		// copy .text section to specified addr
		if (!strncmp((shstr_tbl + sptr->sh_name), TEXT_SECTION_NAME,
			     sizeof(TEXT_SECTION_NAME))) {
			src_ptr = data + sptr->sh_offset;
			phy_addr = ISP_FW_IMAGE_BASE;

			memcpy(&isp->fw_pack_version, src_ptr + ISP_PACK_OFFSET,
			       4);
			memcpy(&isp->fw_svn_version, src_ptr + ISP_SVN_OFFSET,
			       4);
			memcpy(&isp->fw_build_date,
			       src_ptr + ISP_BUILD_DATE_OFFSET, 4);
			dev_info(
				isp->dev,
				"fw pack version is 0x%04x,svn_version is 0x%x,build date is 0x%x size=%d\r\n",
				isp->fw_pack_version, isp->fw_svn_version,
				isp->fw_build_date, sptr->sh_size);

			dst_ptr = (u8 *)ioremap(phy_addr, sptr->sh_size);
			if (dst_ptr == NULL) {
				dev_err(isp->dev,
					"Failed to ioremap 0x%016llX\n",
					phy_addr);
				return -EIO;
			}

			memcpy_toio(dst_ptr, src_ptr, sptr->sh_size);
		}
	}

	return 0;
}

static int parse_elf(const u8 *data, int size, struct c1200_isp_device *isp)
{
	if (!((data[EI_MAG0] == 0x7F) && (data[EI_MAG1] == 'E') &&
	      (data[EI_MAG2] == 'L') && (data[EI_MAG3] == 'F'))) {
		dev_err(isp->dev, "not elf file\n");

		return -1;
	}

	dev_info(isp->dev, "parse elf file\n");
	if (data[EI_CLASS] == ELFCLASS32)
		return parse_elf32(data, size, isp);

	dev_err(isp->dev, "wrong elf class 0x%02X\n", data[EI_CLASS]);

	return -2;
}
