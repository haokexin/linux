// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2021 INTEL Corporation

/*
 * ===========================================================================
 * ===========================================================================
 * Private
 * ===========================================================================
 * ===========================================================================
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/linkage.h>
#include <linux/axxia-oem.h>
#include <linux/uaccess.h>
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/cacheflush.h>

/*
 * DSP Cluster Control
 */

static ssize_t
axxia_dspc_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (finished) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_dspc_get_state());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_dspc_write(struct file *file, const char __user *buffer,
		 size_t count, loff_t *ppos)
{
	char *input;
	int rc;
	unsigned long res;

	input = kmalloc(count + 1, GFP_KERNEL);

	if (!input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;
	rc = kstrtoul(input, 0, &res);

	if (rc) {
		kfree(input);

		return rc;
	}

	axxia_dspc_set_state(res);
	kfree(input);

	return count;
}

static const struct proc_ops axxia_dspc_proc_ops = {
	.proc_read       = axxia_dspc_read,
	.proc_write      = axxia_dspc_write,
	.proc_lseek      = noop_llseek,
};

/*
 * ACTLR_EL3 Control
 */

static ssize_t
axxia_actlr_el3_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (finished != 0) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_actlr_el3_get());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_actlr_el3_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;
	int rc;
	unsigned long res;

	input = kmalloc(count + 1, __GFP_RECLAIMABLE);

	if (!input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;
	rc = kstrtoul(input, 0, &res);

	if (rc) {
		kfree(input);

		return rc;
	}

	axxia_actlr_el3_set(res);
	kfree(input);

	return count;
}

static const struct proc_ops axxia_actlr_el3_proc_ops = {
	.proc_read       = axxia_actlr_el3_read,
	.proc_write      = axxia_actlr_el3_write,
	.proc_lseek      = noop_llseek,
};

/*
 * ACTLR_EL2 Control
 */

static ssize_t
axxia_actlr_el2_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (finished != 0) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_actlr_el2_get());

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_actlr_el2_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;
	int rc;
	unsigned long res;

	input = kmalloc(count + 1, __GFP_RECLAIMABLE);

	if (!input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;
	rc = kstrtoul(input, 0, &res);

	if (rc) {
		kfree(input);

		return rc;
	}

	axxia_actlr_el2_set(res);
	kfree(input);

	return count;
}

static const struct proc_ops axxia_actlr_el2_proc_ops = {
	.proc_read       = axxia_actlr_el2_read,
	.proc_write      = axxia_actlr_el2_write,
	.proc_lseek      = noop_llseek,
};

/*
 * CCN Access
 */

#define DMA_X_SRC_COUNT                        0x00
#define DMA_Y_SRC_COUNT                        0x04
#define DMA_X_MODIF_SRC                        0x08
#define DMA_Y_MODIF_SRC                        0x0c
#define DMA_SRC_CUR_ADDR                       0x10
#define DMA_SRC_ACCESS                         0x14
#define    DMA_SRC_ACCESS_TAIL_LENGTH(x)       (((x) & 0xf) << 11)
#define    DMA_SRC_ACCESS_SRC_MASK_LENGTH(x)   (((x) & 0x1f) << 6)
#define    DMA_SRC_ACCESS_SRC_SIZE(x)          (((x) & 7) << 3)
#define    DMA_SRC_ACCESS_SRC_BURST(x)         (((x) & 7) << 0)
#define DMA_SRC_MASK                           0x18
#define DMA_X_DST_COUNT                        0x1c
#define DMA_Y_DST_COUNT                        0x20
#define DMA_X_MODIF_DST                        0x24
#define DMA_DST_CUR_ADDR                       0x2c
#define DMA_DST_ACCESS                         0x30
#define    DMA_DST_ACCESS_DST_SIZE(x)          (((x) & 7) << 3)
#define    DMA_DST_ACCESS_DST_BURST(x)         (((x) & 7) << 0)
#define DMA_CHANNEL_CONFIG                     0x38
#define    DMA_CONFIG_DST_SPACE(x)             (((x) & 7) << 26)
#define    DMA_CONFIG_SRC_SPACE(x)             (((x) & 7) << 23)
#define    DMA_CONFIG_PRIORITY_ROW             BIT(21)
#define    DMA_CONFIG_PRIORITY                 BIT(20)
#define    DMA_CONFIG_CH_FULL_PRIORITY         BIT(19)
#define    DMA_CONFIG_LAST_BLOCK               BIT(15)
#define    DMA_CONFIG_CLEAR_FIFO               BIT(14)
#define    DMA_CONFIG_START_MEM_LOAD           BIT(13)
#define    DMA_CONFIG_STOP_DST_EOB             BIT(11)
#define    DMA_CONFIG_FULL_DESCR_ADDR          BIT(8)
#define    DMA_CONFIG_INT_DST_EOT              BIT(7)
#define    DMA_CONFIG_INT_DST_EOB              BIT(6)
#define    DMA_CONFIG_WAIT_FOR_TASK_CNT2       BIT(5)
#define    DMA_CONFIG_TASK_CNT2_RESET          BIT(4)
#define    DMA_CONFIG_WAIT_FOR_TASK_CNT1       BIT(3)
#define    DMA_CONFIG_TASK_CNT1_RESET          BIT(2)
#define    DMA_CONFIG_TX_EN                    BIT(1)
#define    DMA_CONFIG_CHAN_EN                  BIT(0)
#define DMA_STATUS                             0x3c
#define    DMA_STATUS_WAIT_TASK_CNT2           BIT(20)
#define    DMA_STATUS_TASK_CNT2_OVERFLOW       BIT(19)
#define    DMA_STATUS_WAIT_TASK_CNT1           BIT(18)
#define    DMA_STATUS_TASK_CNT1_OVERFLOW       BIT(17)
#define    DMA_STATUS_CH_PAUS_WR_EN            BIT(16)
#define    DMA_STATUS_ERR_ACC_DESCR            BIT(14)
#define    DMA_STATUS_ERR_ACC_DST              BIT(13)
#define    DMA_STATUS_ERR_ACC_SRC              BIT(12)
#define    DMA_STATUS_ERR_OVERFLOW             BIT(9)
#define    DMA_STATUS_ERR_UNDERFLOW            BIT(8)
#define    DMA_STATUS_CH_PAUSE                 BIT(7)
#define    DMA_STATUS_CH_WAITING               BIT(5)
#define    DMA_STATUS_CH_ACTIVE                BIT(4)
#define    DMA_STATUS_TR_COMPLETE              BIT(3)
#define    DMA_STATUS_BLK_COMPLETE             BIT(2)
#define    DMA_STATUS_UNALIGNED_READ           BIT(1)
#define    DMA_STATUS_UNALIGNED_WRITE          BIT(0)
#define    DMA_STATUS_UNALIGNED_ERR            (DMA_STATUS_UNALIGNED_READ | \
						DMA_STATUS_UNALIGNED_WRITE)
#define DMA_SRC_ADDR_SEG                       0x54
#define DMA_DST_ADDR_SEG                       0x58

static int ccn_oem_available = 1;
static unsigned long long ccn_phys_base;

static int
enable_ccn_access(void)
{
	unsigned long *value;
	void __iomem *gpdma0;
	void __iomem *mmap_scb;
	unsigned int gpdma0_axprot_override;
	unsigned int gpdma0_status;
	int retries = 1000;

	/*
	 * Update CCN 0, bit 0
	 */

	/*
	 * Map Addresses
	 */

	if (of_find_compatible_node(NULL, NULL, "axxia,axc6732")) {
		gpdma0 = ioremap(0x8005020000, SZ_64K);
	} else if (of_find_compatible_node(NULL, NULL, "axxia,axm5616")) {
		gpdma0 = ioremap(0x8004120000, SZ_64K);
	} else {
		pr_err("Only Valid for Axxia 6700 and 5600!\n");

		return -EFAULT;
	}

	mmap_scb = ioremap(0x8032000000, SZ_512K);

	/*
	 * Input for the DMA Transfer
	 */

	value = kmalloc(sizeof(unsigned long), GFP_ATOMIC);
	if (!value)
		return -ENOMEM;

	*value = 1;
	dcache_clean_inval_poc((unsigned long)value, (unsigned long)value + sizeof(unsigned long));
	mb(); /* Ensure that the data held in page is written back */

	/* Make sure no other transactions are in process. */
	gpdma0_status = readl(gpdma0 + DMA_STATUS);

	if (0 != (gpdma0_status & DMA_STATUS_CH_ACTIVE)) {
		iounmap(mmap_scb);
		iounmap(gpdma0);
		kfree(value);
		pr_err("Error Reading DMA_STATUS!\n");

		return -1;
	}

	/* Clear status bits. */
	writel((DMA_STATUS_TR_COMPLETE | DMA_STATUS_BLK_COMPLETE),
	       gpdma0 + DMA_STATUS);

	/* Set gpdma0_axprot_override to secure or non-secure. */
	gpdma0_axprot_override = readl(mmap_scb + 0x48800);
	writel(2, mmap_scb + 0x48800);

	/* Destination is 0x80_0000_0000 */
	writel(0x80, (gpdma0 + DMA_DST_ADDR_SEG));
	writel(0, (gpdma0 + DMA_DST_CUR_ADDR));

	/* value is the source */
	writel((unsigned long)((virt_to_phys(value) & 0xff00000000) >> 32),
	       gpdma0 + DMA_SRC_ADDR_SEG);
	writel((unsigned long)((virt_to_phys(value) & 0xffffffff)),
	       gpdma0 + DMA_SRC_CUR_ADDR);

	/* Remaing setup. */
	writel(DMA_SRC_ACCESS_SRC_MASK_LENGTH(8) |
	       DMA_SRC_ACCESS_SRC_SIZE(3) |
	       DMA_SRC_ACCESS_SRC_BURST(5),
	       gpdma0 + DMA_SRC_ACCESS);
	writel(DMA_DST_ACCESS_DST_SIZE(3) |
	       DMA_DST_ACCESS_DST_BURST(5),
	       gpdma0 + DMA_DST_ACCESS);
	writel(0xffffffff, gpdma0 + DMA_SRC_MASK);
	writel(sizeof(unsigned long), gpdma0 + DMA_X_MODIF_SRC);
	writel(sizeof(unsigned long), gpdma0 + DMA_X_MODIF_DST);
	writel(1, gpdma0 + DMA_X_SRC_COUNT);
	writel(0, gpdma0 + DMA_Y_SRC_COUNT);
	writel(1, gpdma0 + DMA_X_DST_COUNT);
	writel(0, gpdma0 + DMA_Y_DST_COUNT);

	/* Start the transfer. */
	writel(DMA_CONFIG_DST_SPACE(1) |
	       DMA_CONFIG_SRC_SPACE(1) |
	       DMA_CONFIG_CH_FULL_PRIORITY |
	       DMA_CONFIG_LAST_BLOCK |
	       DMA_CONFIG_FULL_DESCR_ADDR |
	       DMA_CONFIG_WAIT_FOR_TASK_CNT2 |
	       DMA_CONFIG_WAIT_FOR_TASK_CNT1 |
	       DMA_CONFIG_TX_EN |
	       DMA_CONFIG_CHAN_EN, gpdma0 + DMA_CHANNEL_CONFIG);

	/* Wait for completion. */
	while (retries-- > 0) {
		if ((readl(gpdma0 + DMA_STATUS) & 0x8) != 0 &&
		    readl(gpdma0 + DMA_X_DST_COUNT) == 0)
			break;

		udelay(1);
	}

	/* Restore gpdma0_axprot_override. */
	writel(gpdma0_axprot_override, (mmap_scb + 0x48800));

	if (retries <= 0) {
		iounmap(mmap_scb);
		iounmap(gpdma0);
		kfree(value);
		pr_err("gpdma_xfer timed out\n");

		return -EFAULT;
	}

	iounmap(mmap_scb);
	iounmap(gpdma0);
	kfree(value);

	return 0;
}

static unsigned long ccn_offset;

static ssize_t
axxia_ccn_offset_read(struct file *filp, char *buffer, size_t length,
		      loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (finished != 0) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", ccn_offset);

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_ccn_offset_write(struct file *file, const char __user *buffer,
		       size_t count, loff_t *ppos)
{
	char *input;
	unsigned int new_ccn_offset;
	int rc;
	unsigned long res;

	if (!buffer)
		return -EINVAL;

	input = kmalloc(count + 1, GFP_KERNEL);

	if (!input)
		return -ENOSPC;

	rc = copy_from_user(input, buffer, count);

	if (rc) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;
	rc = kstrtoul(input, 0, &res);

	if (rc) {
		kfree(input);

		return rc;
	}

	new_ccn_offset = (unsigned int)res;

	if (of_find_compatible_node(NULL, NULL, "axxia,axc6732")) {
		if (new_ccn_offset > 0x9cff00)
			pr_err("Invalid CCN Offset!\n");
	} else if (of_find_compatible_node(NULL, NULL, "axxia,axm5616")) {
		if (new_ccn_offset > 0x94ff00)
			pr_err("Invalid CCN Offset!\n");
	} else {
		pr_err("Internal Error!\n");
	}

	ccn_offset = (unsigned int)new_ccn_offset;
	kfree(input);

	return count;
}

static const struct proc_ops axxia_ccn_offset_proc_ops = {
	.proc_read       = axxia_ccn_offset_read,
	.proc_write      = axxia_ccn_offset_write,
	.proc_lseek      = noop_llseek,
};

static ssize_t
axxia_ccn_value_read(struct file *filp, char *buffer, size_t length,
		     loff_t *offset)
{
	static int finished;
	char return_buffer[80];

	if (finished != 0) {
		finished = 0;

		return 0;
	}

	finished = 1;
	sprintf(return_buffer, "0x%lx\n", axxia_ccn_get(ccn_offset));

	if (copy_to_user(buffer, return_buffer, strlen(return_buffer)))
		return -EFAULT;

	return strlen(return_buffer);
}

static ssize_t
axxia_ccn_value_write(struct file *file, const char __user *buffer,
		      size_t count, loff_t *ppos)
{
	char *input;
	int rc;
	unsigned long res;

	input = kmalloc(count + 1, __GFP_RECLAIMABLE);

	if (!input)
		return -ENOSPC;

	if (copy_from_user(input, buffer, count)) {
		kfree(input);

		return -EFAULT;
	}

	input[count] = 0;
	rc = kstrtoul(input, 0, &res);

	if (rc) {
		kfree(input);

		return rc;
	}

	axxia_ccn_set(ccn_offset, res);
	kfree(input);

	return count;
}

static const struct proc_ops axxia_ccn_value_proc_ops = {
	.proc_read       = axxia_ccn_value_read,
	.proc_write      = axxia_ccn_value_write,
	.proc_lseek      = noop_llseek,
};

/*
 * -----------------------------------------------------------------------------
 * setup_ccn_proc
 */

static int
setup_ccn_proc(void)
{
	int rc = 0;

	if (of_find_compatible_node(NULL, NULL, "axxia,axc6732")) {
		ccn_phys_base = 0x4000000000ULL;
	} else if (of_find_compatible_node(NULL, NULL, "axxia,axm5616")) {
		ccn_phys_base = 0x8000000000ULL;
	} else {
		pr_err("Only Valid for Axxia 6700 and 5600!\n");

		return -EFAULT;
	}

	ccn_offset = 0;

	if (proc_create("driver/axxia_ccn_offset", 0200, NULL,
			&axxia_ccn_offset_proc_ops) == NULL) {
		pr_err("Could not create /proc/driver/axxia_ccn_offset!\n");
		rc = -EFAULT;
	} else if (proc_create("driver/axxia_ccn_value", 0200, NULL,
			       &axxia_ccn_value_proc_ops) == NULL) {
		pr_err("Could not create /proc/driver/axxia_ccn_value!\n");
		rc = -EFAULT;
	}

	if (rc) {
		pr_err("Axxia CCN Access is Not Available!\n");
	} else {
		if (ccn_oem_available)
			pr_info("Axxia CCN (scm) Initialized\n");
		else
			pr_info("Axxia CCN (direct) Initialized\n");
	}

	return rc;
}

/*
 * ===========================================================================
 * ===========================================================================
 * Public
 * ===========================================================================
 * ===========================================================================
 */

/*
 * ---------------------------------------------------------------------------
 * axxia_dspc_get_state
 */

unsigned long
axxia_dspc_get_state(void)
{
	struct arm_smccc_res res;

	__arm_smccc_smc(0xc3000000, 0, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Getting the DSP State Failed!\n");

	return res.a1;
}
EXPORT_SYMBOL(axxia_dspc_get_state);

/*
 * ---------------------------------------------------------------------------
 * axxia_dspc_set_state
 */

void
axxia_dspc_set_state(unsigned long state)
{
	struct arm_smccc_res res;

	__arm_smccc_smc(0xc3000001, state, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Getting the DSP State Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_dspc_set_state);

/*
 * ---------------------------------------------------------------------------
 * axxia_actlr_el3_get
 */

unsigned long
axxia_actlr_el3_get(void)
{
	struct arm_smccc_res res;

	__arm_smccc_smc(0xc3000002, 0, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Getting ACTLR_EL3 Failed!\n");

	return res.a1;
}
EXPORT_SYMBOL(axxia_actlr_el3_get);

/*
 * ---------------------------------------------------------------------------
 * axxia_actlr_el3_set
 */

void
axxia_actlr_el3_set(unsigned long input)
{
	struct arm_smccc_res res;

	__arm_smccc_smc(0xc3000003, input, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Setting ACTLR_EL3 Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_actlr_el3_set);

/*
 * ---------------------------------------------------------------------------
 * axxia_actlr_el2_get
 */

unsigned long
axxia_actlr_el2_get(void)
{
	struct arm_smccc_res res;

	__arm_smccc_smc(0xc3000004, 0, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Getting ACTLR_EL2 Failed!\n");

	return res.a1;
}
EXPORT_SYMBOL(axxia_actlr_el2_get);

/*
 * ---------------------------------------------------------------------------
 * axxia_actlr_el2_set
 */

void
axxia_actlr_el2_set(unsigned long input)
{
	struct arm_smccc_res res;

	__arm_smccc_smc(0xc3000005, input, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Setting ACTLR_EL2 Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_actlr_el2_set);

/*
 * ---------------------------------------------------------------------------
 * axxia_ccn_get
 */

unsigned long
axxia_ccn_get(unsigned int offset)
{
	struct arm_smccc_res res;

	if (ccn_oem_available == 0) {
		void __iomem *ccn;

		ccn = ioremap(ccn_phys_base +
			      ((offset / SZ_1K) * SZ_1K), SZ_1K);
		res.a1 = readq(ccn + (offset % SZ_1K));
		iounmap(ccn);
	} else {
		__arm_smccc_smc(0xc3000006, offset, 0, 0, 0, 0, 0, 0,
				&res, NULL);

		if (res.a0 != 0)
			pr_warn("Getting CCN Register Failed!\n");
	}

	return res.a1;
}
EXPORT_SYMBOL(axxia_ccn_get);

/*
 * ---------------------------------------------------------------------------
 * axxia_ccn_set
 */

void
axxia_ccn_set(unsigned int offset, unsigned long value)
{
	struct arm_smccc_res res;

	if (ccn_oem_available == 0) {
		void __iomem *ccn;

		ccn = ioremap(ccn_phys_base +
			      ((offset / SZ_1K) * SZ_1K), SZ_1K);
		writeq(value, (ccn + (offset % SZ_1K)));
		iounmap(ccn);
	} else {
		__arm_smccc_smc(0xc3000007, offset, value, 0, 0, 0, 0, 0,
				&res, NULL);

		if (res.a0 != 0)
			pr_warn("Getting CCN Register Failed!\n");
	}

	return;
}
EXPORT_SYMBOL(axxia_ccn_set);

/*
 * ---------------------------------------------------------------------------
 * axxia_oem_init
 */

static int
axxia_oem_init(void)
{
	struct arm_smccc_res res;
	int rc = 0;

	if (of_find_compatible_node(NULL, NULL, "axxia,axc6732")) {
		/* Only applicable to the 6700. */
		if (proc_create("driver/axxia_dspc", 0200, NULL,
				&axxia_dspc_proc_ops) == NULL)
			pr_err("Could not create /proc/driver/axxia_dspc!\n");
		else
			pr_info("Axxia DSP Control Initialized\n");
	}

	if (proc_create("driver/axxia_actlr_el3", 0200, NULL,
			&axxia_actlr_el3_proc_ops) == NULL)
		pr_err("Could not create /proc/driver/axxia_actlr_el3!\n");
	else
		pr_info("Axxia ACTLR_EL3 Control Initialized\n");

	if (proc_create("driver/axxia_actlr_el2", 0200, NULL,
			&axxia_actlr_el2_proc_ops) == NULL)
		pr_err("Could not create /proc/driver/axxia_actlr_el2!\n");
	else
		pr_info("Axxia ACTLR_EL3 Control Initialized\n");

	/*
	 * In some cases, CCN access is required (SBB work around for
	 * example) but OEM access is not available (version of ATF
	 * before 1.38).  As it happens, in U-Boot versions prior to
	 * 1.79, the hni_axprot_override is set.  This allows the
	 * cores, while non-secure, to access the SCB blocks.  The
	 * following provides CCN acces to non-secure masters by doing
	 * the following:
	 *
	 * -1-
	 * Check for OEM access.  If available, skip the reset.
	 *
	 * -2-
	 * If OEM access is not avialable, try to enable non-secure
	 * access by setting bit 0 of CCN offset 0 (the 'secure_acces'
	 * bit).  See enable_ccn_access() above for details.
	 */

	__arm_smccc_smc(0xc3000006, 0xfe0, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0) {
		ccn_oem_available = 0;
		rc = enable_ccn_access();

		if (rc)
			pr_err("Unable to Enable CCN Access via GPDMA!\n");
	}

	if (!rc)
		rc = setup_ccn_proc();

	return rc;
}

device_initcall(axxia_oem_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Axxia OEM Control");
