// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/smp.h>

#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
#include <linux/bst_samphore.h>
#endif
#ifdef CONFIG_BST_HEALTH_MONITOR
#include <bst/bst_common_api.h>
#endif

#include "isp_hw.h"

#include "isp_core.h"

static int load_fw_bin(struct isp_device *isp)
{
	int rv;
	int i;
	struct device *dev;
	const struct firmware *firmware;
	const u8 *data;
	Elf32_Ehdr *ehdr;
	Elf32_Shdr *shdr;
	Elf32_Shdr *shstr;
	const char *shstr_tbl;

	dev = isp->dev;
	dev_info(dev, "CPU: %u, TID: %7u: Load firmware: %s\n",
		 smp_processor_id(), current->pid, isp->fw.bin);
	rv = request_firmware(&firmware, isp->fw.bin, dev);
	if (rv) {
		dev_err(dev,
			"CPU: %u, TID: %7u: Failed to request firmware %s, rv: %d\n",
			smp_processor_id(), current->pid, isp->fw.bin, rv);
		return -EIO;
	}

	data = firmware->data;
	if (!((data[EI_MAG0] == 0x7F) && (data[EI_MAG1] == 'E') &&
	      (data[EI_MAG2] == 'L') && (data[EI_MAG3] == 'F') &&
	      (data[EI_CLASS] == ELFCLASS32))) {
		dev_err(dev, "CPU: %u, TID: %7u: %s is not valid elf32 file\n",
			smp_processor_id(), current->pid, isp->fw.bin);
		rv = -EINVAL;
		goto exit;
	}

	rv = -ENOEXEC;
	ehdr = (Elf32_Ehdr *)data;
	shdr = (Elf32_Shdr *)(data + ehdr->e_shoff);
	shstr = (Elf32_Shdr *)((char *)shdr +
			       ehdr->e_shstrndx * ehdr->e_shentsize);
	shstr_tbl = data + shstr->sh_offset;
	for (i = 0; i < ehdr->e_shnum; ++i) {
		Elf32_Shdr *sptr;
		const u8 *src;

		sptr = (Elf32_Shdr *)((char *)shdr + i * ehdr->e_shentsize);
		/* Copy the .text section be found firstly to specified addr */
		if (strncmp((shstr_tbl + sptr->sh_name), ELF_CODE_SEC_NAME,
			    sizeof(ELF_CODE_SEC_NAME)))
			continue;

		src = data + sptr->sh_offset;
		memcpy(&isp->shared->fw.version, src + FW_VERSION_OFFSET,
		       sizeof(isp->shared->fw.version));
		memcpy(&isp->shared->fw.scm_id, src + FW_SCM_ID_OFFSET,
		       sizeof(isp->shared->fw.scm_id));
		memcpy(&isp->shared->fw.build_date, src + FW_BUILD_DATE_OFFSET,
		       sizeof(isp->shared->fw.build_date));
		dev_info(
			isp->dev,
			"CPU: %u, TID: %7u: Firmware version: %d.%d.%d.%d, SCM ID: %x, date: %04d-%02d-%02d, code size: %d bytes\n",
			smp_processor_id(), current->pid,
			FW_VER_MAJOR(isp->shared->fw.version),
			FW_VER_MINOR(isp->shared->fw.version),
			FW_VER_PATCH(isp->shared->fw.version),
			FW_VER_CUST(isp->shared->fw.version),
			isp->shared->fw.scm_id,
			FW_DATE_YEAR(isp->shared->fw.build_date),
			FW_DATE_MONTH(isp->shared->fw.build_date),
			FW_DATE_DAY(isp->shared->fw.build_date), sptr->sh_size);

		memcpy_toio(isp->pram, src, sptr->sh_size);
		rv = 0;
		break;
	}

exit:
	release_firmware(firmware);

	return rv;
}

static int load_fw_slab(struct isp_device *isp)
{
	int rv;
	struct device *dev;
	const struct firmware *firmware;

	dev = isp->dev;
	dev_info(dev, "CPU: %u, TID: %7u: Load slab: %s\n", smp_processor_id(),
		 current->pid, isp->fw.slab);
	rv = request_firmware(&firmware, isp->fw.slab, dev);
	if (rv) {
		dev_err(dev,
			"CPU: %u, TID: %7u: Failed to request slab %s, rv: %d\n",
			smp_processor_id(), current->pid, isp->fw.slab, rv);
		return -EIO;
	}
	memcpy(isp->msg.slab_va, firmware->data, firmware->size);
	release_firmware(firmware);

	return 0;
}

static void start_fw(struct isp_device *isp)
{
	u32 val;

#ifdef CONFIG_BST_IPC
	if (isp->use_ipc) {
		writel_relaxed(isp->ipc.msg_dma, isp->ctrl + R_TOP_MAILBOX_OUT);
		writel_relaxed(isp->ipc.cpu, isp->ctrl + R_TOP_IPC_ID);
	}
#endif
	writel_relaxed(isp->fw.rsv_dma, isp->ctrl + R_TOP_RSV_BUF);
	writel_relaxed(isp->msg.init_dma, isp->ctrl + R_TOP_MAILBOX_IN);

	/* Deassert reset */
	val = readl_relaxed(isp->ctrl + R_TOP_SW_RST);
	writel_relaxed(val | RESET_DEASSERT_BITS, isp->ctrl + R_TOP_SW_RST);
}

#ifdef CONFIG_BST_HEALTH_MONITOR
static void get_psm(struct isp_device *isp)
{
	int i;

	for (i = 0; i < MAX_ISP_CORE; ++i) {
		u8 block_id;
		u32 psm[PSM_BLOCK_CFG_SIZE];
		int rv;

		memset(psm, 0, sizeof(psm));
		rv = get_psmid_from_safety_lib((PSM_BLOCK_ID_ISP_BASE + i),
					       &block_id, psm);
		if (rv) {
			dev_err(isp->dev, "Failed to get PSM\n");

			return;
		}

		/* Only has limited safety mechanism */
		isp->shared->fw.psm_core[i] = (u8)psm[0];
		dev_dbg(isp->dev, "PSM %d: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", i,
			psm[0], psm[1], psm[2], psm[3]);
	}
	dev_info(isp->dev, "PSM all: 0x%08X\n", isp->shared->fw.psm_all);
}
#endif

bool isp_hw_has_inited(struct isp_device *isp)
{
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	return readl_relaxed(isp->ctrl + R_TOP_MAILBOX_IN);
#else
	return false;
#endif
}

/*
 * Initialize ISP firmware: load, boot, config.
 * Should be called after setup PLL and memories.
 * @isp: ISP device
 *
 * Returns:
 *     0 for success.
 *     -EACCES for load from slave.
 *     -EIO for others.
 */
int isp_fw_init(struct isp_device *isp)
{
	int rv;
	u32 val;
	int timeout;
	struct device *dev;

	mutex_lock(&isp->lock);
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	get_sem_lock(isp->hwlock);
#endif
	if (isp->role != ROLE_MASTER) {
		if (isp->shared->fw.stage == FS_RUNNING)
			rv = 0;
		else
			rv = -EACCES;
		goto exit;
	}

	if (isp->shared->fw.stage >= FS_LOADED) {
		rv = 0;
		goto exit;
	}

	dev = isp->dev;
#ifdef CONFIG_BST_HEALTH_MONITOR
	get_psm(isp);
#endif
	/* Enable clock */
	val = readl_relaxed(isp->ctrl + R_TOP_CLK_EN);
	val |= CLK_EN_BITS;
	if (!(isp->shared->fw.psm_all & CTRL_TIMEOUT_BITS))
		val &= ~(BIT(15));
	writel_relaxed(val, isp->ctrl + R_TOP_CLK_EN);

	/* Put ISP subsystem to reset state */
	val = readl_relaxed(isp->ctrl + R_TOP_SW_RST);
	writel_relaxed(val & ~RESET_DEASSERT_BITS, isp->ctrl + R_TOP_SW_RST);

	/* Enable safety for firmware */
	val = readl_relaxed(isp->ctrl + R_TOP_PARITY_ECC);
	if (isp->shared->fw.psm_all & MEM_ECC_BITS)
		val |= BIT(5) | BIT(6);
	else
		val &= ~(BIT(5) | BIT(6));

	if (isp->shared->fw.psm_all & MEM_PARITY_BITS)
		val |= BIT(23);
	else
		val &= ~(BIT(23));
	writel_relaxed(val, isp->ctrl + R_TOP_PARITY_ECC);

	rv = -EIO;
	if (load_fw_bin(isp)) {
		dev_err(dev, "Failed to load firmware\n");
		goto exit;
	}

	if (load_fw_slab(isp)) {
		dev_err(dev, "Failed to load slab\n");
		goto exit;
	}

	isp->shared->fw.stage = FS_LOADED;
	dev_info(dev, "Starting firmware\n");
	/* We reinit here to support reset on runtime */
	reinit_completion(&isp->fw.boot_comp);
	start_fw(isp);
	timeout = wait_for_completion_timeout(
		&(isp->fw.boot_comp), msecs_to_jiffies(ISP_FW_BOOT_TIME));
	if (timeout == 0) {
		dev_err(dev, "Initializating firmware timed out\n");
		rv = -EIO;
		goto exit;
	}
	isp->shared->fw.stage = FS_RUNNING;
	dev_info(dev, "Firmware booted successfully\n");
	rv = 0;

exit:
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
	release_sem_lock(isp->hwlock);
#endif
	mutex_unlock(&isp->lock);

	return rv;
}

/*
 * Stop ISP firmware
 * @isp: ISP device
 */
void isp_fw_exit(struct isp_device *isp)
{
	u32 val;

	/* Put ISP subsystem to reset state */
	val = readl_relaxed(isp->ctrl + R_TOP_SW_RST);
	writel_relaxed(val & ~RESET_DEASSERT_BITS, isp->ctrl + R_TOP_SW_RST);

	/* Disable clock */
	val = readl_relaxed(isp->ctrl + R_TOP_CLK_EN);
	writel_relaxed(val & ~CLK_EN_BITS, isp->ctrl + R_TOP_CLK_EN);
}

/*
 * Set ISP to forward trigger signal from outer source
 * @rx_pin: Which PIN's signal is referenced as input
 * @tx_pin: Which PIN is used to output signal
 * @src_id: Which outer source ID is used to generate signal
 *
 * Returns 0 for success
 */
static int isp_fsync_set_bypass(struct isp_device *isp, int rx_pin, int tx_pin,
				int src_id)
{
	u32 val;

	/* 1. Set FSYNCx_SEL to src_id + 1, and FSYNx_OEN to 1(output) */
	val = readl_relaxed(isp->ctrl + R_FSYNC_FSYNX_OEN);
	val &= ~(0x7 << (8 + 3 * tx_pin)); /* Clear bits first */
	val |= (src_id + 1) << (8 + 3 * tx_pin); /* Set bits */

	val |= (1 << tx_pin); /* Set FSYNCx_OEN to 1 for output */
	val &= ~(0x1 << rx_pin); /* Set FSYNCx_OEN to 0 for input */
	writel_relaxed(val, isp->ctrl + R_FSYNC_FSYNX_OEN);

	/* 2. Set PULSEx_OUTSIDE_SEL to choose outside_fsyncX */
	val = readl_relaxed(isp->ctrl + R_FSYNC_SRC_EN);
	val &= ~(0x7 << (16 + 3 * src_id)); /* Clear bits first */
	val |= rx_pin << (16 + 3 * src_id); /* Set bits */
	writel_relaxed(val, isp->ctrl + R_FSYNC_SRC_EN);

	return 0;
}

/*
 * Set ISP to generate trigger signal from outer source
 * @rx_fps: The referenced input frequency
 * @tx_fps: The expected output frequency
 * @rx_pin: Which PIN's signal is referenced as input
 * @tx_pin: Which PIN is used to output signal
 * @pulse_width: The signal's pusle width by ns
 * @src_id: Which outer source ID is used to generate signal
 *
 * Returns 0 for success
 */
static int isp_fsync_set_outer(struct isp_device *isp, int rx_fps, int tx_fps,
			       int rx_pin, int tx_pin, u32 pulse_width,
			       int src_id)
{
	u32 val;
	struct device *dev;

	dev = isp->dev;

	if (rx_pin < FSYNC_PIN_MIN || rx_pin > FSYNC_PIN_MAX) {
		dev_err(dev, "Invalid outer fsync rx pin %d\n", rx_pin);
		return -EINVAL;
	}

	if (tx_pin < FSYNC_PIN_MIN || tx_pin > FSYNC_PIN_MAX) {
		dev_err(dev, "Invalid outer fsync tx pin %d\n", tx_pin);
		return -EINVAL;
	}

	if (src_id < FSYNC_OUTER_SRC_MIN || src_id > FSYNC_OUTER_SRC_MAX) {
		dev_err(dev, "Invalid outer fsync source id %d\n", src_id);
		return -EINVAL;
	}

	src_id -= FSYNC_OUTER_SRC_MIN; /* Remap it for convenience */

	/* NOTE: BYPASS mode is special */
	if (tx_fps == rx_fps && pulse_width == 0)
		return isp_fsync_set_bypass(isp, rx_pin, tx_pin, src_id);

	/* 1.1 Enable given source */
	val = readl_relaxed(isp->ctrl + R_FSYNC_SRC_EN);
	val |= (1 << (4 + src_id));
	/* 1.2 Set INC_OUTSIDEx */
	if (tx_fps > rx_fps)
		val |= (1 << (8 + src_id));
	else
		val &= ~(1 << (8 + src_id));

	/* 1.3 Set PULSEx_OUTSIDE_SEL */
	val &= ~(0x7 << (16 + 3 * src_id)); /* Clear bits first */
	val |= rx_pin << (16 + 3 * src_id); /* Set bits */
	writel_relaxed(val, isp->ctrl + R_FSYNC_SRC_EN);

	if (tx_fps > rx_fps) {
		/* 2. Set OUT_SRCx_PERIOD */
		writel_relaxed(FREQ_TO_PERIOD(tx_fps),
			       isp->ctrl + R_FSYNC_OUTER_SRC1_PERIOD +
				       src_id * 4);
		val = FREQ_TO_PERIOD(rx_fps);
		/* Set OUT_ORGx_MULT to auto skew with input frequency */
		writel_relaxed(val, isp->ctrl + R_OUT_ORG1_MULT + src_id * 4);
	} else {
		/* 2. Set DECREASE_PERIODx */
		u32 ratio;

		/* NOTE: must dived by 2 first to get correct register offset */
		val = readl_relaxed(isp->ctrl + R_FSYNC_DECREASE_PERIOD1 +
				    src_id / 2 * 4);
		val &= ~(0xFFFF << (src_id % 2 * 16)); /* Clear bits */
		ratio = rx_fps / tx_fps;
		val |= ratio << (src_id % 2 * 16); /* Set bits */
		writel_relaxed(val, isp->ctrl + R_FSYNC_DECREASE_PERIOD1 +
					    src_id / 2 * 4);
	}

	/* 3. Set pulse width */
	if (pulse_width == 0)
		pulse_width = FSYNC_DEFAULT_PULSE_WIDTH;
	writel_relaxed(NS_TO_PERIOD(pulse_width),
		       isp->ctrl + R_FSYNC_PULSE0_WIDTH + tx_pin * 4);

	/* 4. Set INTx_SEL, map source to outside_gen_fsyncX */
	val = readl_relaxed(isp->ctrl + R_FSYNC_INTX_SEL);
	val &= ~(0x7 << (8 + 3 * tx_pin)); /* Clear INT_SELx first */
	val |= (src_id + 2) << (8 + 3 * tx_pin); /* Set INT_SELx */
	writel_relaxed(val, isp->ctrl + R_FSYNC_INTX_SEL);

	/* 5. Set FSYNCx_SEL to 0(From FSYNCx_inside), and FSYNx_OEN to 1(output) */
	val = readl_relaxed(isp->ctrl + R_FSYNC_FSYNX_OEN);
	/* Set FSYNCx_SEL to 0 for FSYNCx_inside mode */
	val &= ~(0x7 << (8 + 3 * tx_pin));
	val |= (1 << tx_pin); /* Set FSYNCx_OEN to 1 for output */
	val &= ~(0x1 << rx_pin); /* Set FSYNCx_OEN to 0 for input */
	writel_relaxed(val, isp->ctrl + R_FSYNC_FSYNX_OEN);

	return 0;
}

/*
 * Set ISP to generate trigger signal from inner source
 * @tx_fps: The expected output frequency
 * @tx_pin: Which PIN is used to output signal
 * @pulse_width: The signal's pusle width by ns
 * @src_id: Which inner source ID is used to generate signal
 * @map_to_outer: Map to outer mode, the fsync input signal is from SoC
 * @rx_pin: Which PIN is selected from SoC, only valid when mapping to outer
 *
 * Returns 0 for success
 */
static int isp_fsync_set_inner(struct isp_device *isp, int tx_fps, int tx_pin,
			       u32 pulse_width, int src_id, int map_to_outer,
			       int rx_pin, int rx_fps)
{
	u32 val;
	struct device *dev;

	dev = isp->dev;
	if (tx_pin < FSYNC_PIN_MIN || tx_pin > FSYNC_PIN_MAX) {
		dev_err(dev, "Invalid inner fsync tx pin %d\n", tx_pin);
		return -EINVAL;
	}

	if (src_id < FSYNC_INNER_SRC_MIN || src_id > FSYNC_INNER_SRC_MAX) {
		dev_err(dev, "Invalid inner fsync source id %d\n", src_id);
		return -EINVAL;
	}

	if (map_to_outer >= FSYNC_OUTER_SRC_MIN &&
	    map_to_outer <= FSYNC_OUTER_SRC_MAX) {
		/* NOTE: Workaround for frequency is inaccurate,
		 * The FSYNC signal is from SoC.
		 */
		val = readl_relaxed(isp->ctrl + R_FSYNC_SEL_SOC_FSYNC);
		val |= (1 << rx_pin);
		writel_relaxed(val, isp->ctrl + R_FSYNC_SEL_SOC_FSYNC);
		return isp_fsync_set_outer(isp, rx_fps, tx_fps, rx_pin, tx_pin,
					   pulse_width, map_to_outer);
	}

	/* 1. Enable given source */
	val = readl_relaxed(isp->ctrl + R_FSYNC_SRC_EN);
	writel_relaxed(val | (1 << src_id), isp->ctrl + R_FSYNC_SRC_EN);

	/* 2. Set period */
	writel_relaxed(FREQ_TO_PERIOD(tx_fps),
		       isp->ctrl + R_FSYNC_INNER_SRC1_PERIOD + 4 * src_id);

	/* 3. Set pulse width */
	if (pulse_width == 0)
		pulse_width = FSYNC_DEFAULT_PULSE_WIDTH;
	writel_relaxed(NS_TO_PERIOD(pulse_width),
		       isp->ctrl + R_FSYNC_PULSE0_WIDTH + 4 * tx_pin);

	/* 4. Set INTx_SEL(From which internal counter) */
	val = readl_relaxed(isp->ctrl + R_FSYNC_INTX_SEL);
	val &= ~(0x7 << (8 + 3 * tx_pin)); /* Clear INT_SELx first */
	val |= src_id << (8 + 3 * tx_pin); /* Set INT_SELx */
	writel_relaxed(val, isp->ctrl + R_FSYNC_INTX_SEL);

	/* 5. Set FSYNCx_SEL to 0 for inner mode, FSYNCx_OEN to 1 for output */
	val = readl_relaxed(isp->ctrl + R_FSYNC_FSYNX_OEN);
	val &= ~(0x7 << (8 + 3 * tx_pin));
	val |= (0x1 << tx_pin);
	writel_relaxed(val, isp->ctrl + R_FSYNC_FSYNX_OEN);

	return 0;
}

/*
 * Parse device tree and apply ISP fsync settings
 * @isp: ISP device, includes isp-fsync@X nodes
 *
 * Returns 0 for success
 */
int isp_fsync_setup(struct isp_device *isp)
{
	struct device *dev;
	struct device_node *node;
	struct device_node *fsync_dt;

	dev = isp->dev;
	node = dev->of_node;
	for_each_child_of_node(node, fsync_dt) {
		int rv;
		int source;
		int tx_pin;
		int rx_pin;
		int tx_fps;
		int rx_fps;
		int pulse_width;
		int map_to_outer;

		if (!fsync_dt->name || of_node_cmp(fsync_dt->name, "fsync"))
			continue;
		rv = of_property_read_s32(fsync_dt, "source", &source);
		if (rv || source > FSYNC_OUTER_SRC_MAX ||
		    source < FSYNC_INNER_SRC_MIN) {
			dev_err(dev, "Invalid source for %s\n", fsync_dt->name);
			return -EINVAL;
		}
		rv = of_property_read_s32(fsync_dt, "tx-pin", &tx_pin);
		if (rv) {
			dev_err(dev, "Invalid tx-pin for %s\n", fsync_dt->name);
			return -EINVAL;
		}
		rv = of_property_read_s32(fsync_dt, "tx-fps", &tx_fps);
		if (rv) {
			dev_err(dev, "Invalid tx-fps for %s\n", fsync_dt->name);
			return -EINVAL;
		}
		rv = of_property_read_s32(fsync_dt, "pulse-width",
					  &pulse_width);
		if (rv)
			pulse_width = FSYNC_DEFAULT_PULSE_WIDTH;
		rv = of_property_read_s32(fsync_dt, "map-to-outer",
					  &map_to_outer);
		if (rv)
			map_to_outer = FSYNC_OUTER_SRC_INVALID;

		if (source >= FSYNC_INNER_SRC_MIN &&
		    source <= FSYNC_INNER_SRC_MAX) {
			rx_pin = FSYNC_PIN_INVALID;
			if (map_to_outer >= FSYNC_OUTER_SRC_MIN &&
			    map_to_outer <= FSYNC_OUTER_SRC_MAX) {
				rv = of_property_read_s32(fsync_dt, "rx-pin",
							  &rx_pin);
				if (rv) {
					dev_err(dev, "Invalid rx-pin for %s\n",
						fsync_dt->name);
					return -EINVAL;
				}
			}
			rx_fps = FSYNC_DEFAULT_SOC_FPS;
			rv = isp_fsync_set_inner(isp, tx_fps, tx_pin,
						 pulse_width, source,
						 map_to_outer, rx_pin, rx_fps);
			if (rv) {
				dev_err(dev,
					"Failed to setup inner fsync for %s\n",
					fsync_dt->name);
				return rv;
			}
		} else {
			rv = of_property_read_s32(fsync_dt, "rx-pin", &rx_pin);
			if (rv) {
				dev_err(dev, "Invalid rx-pin for %s\n",
					fsync_dt->name);
				return -EINVAL;
			}

			rv = of_property_read_s32(fsync_dt, "rx-fps", &rx_fps);
			if (rv) {
				dev_err(dev, "Invalid rx-fps for %s\n",
					fsync_dt->name);
				return -EINVAL;
			}

			rv = isp_fsync_set_outer(isp, rx_fps, tx_fps, rx_pin,
						 tx_pin, pulse_width, source);
			if (rv) {
				dev_err(dev,
					"Failed to setup outer fsync for %s\n",
					fsync_dt->name);
				return rv;
			}
		}
	}

	return 0;
}
