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

	axxia_dspc_set_state(res);
	kfree(input);

	return count;
}

static const struct file_operations axxia_dspc_proc_ops = {
	.read       = axxia_dspc_read,
	.write      = axxia_dspc_write,
	.llseek     = noop_llseek,
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

static const struct file_operations axxia_actlr_el3_proc_ops = {
	.read       = axxia_actlr_el3_read,
	.write      = axxia_actlr_el3_write,
	.llseek     = noop_llseek,
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

static const struct file_operations axxia_actlr_el2_proc_ops = {
	.read       = axxia_actlr_el2_read,
	.write      = axxia_actlr_el2_write,
	.llseek     = noop_llseek,
};

/*
 * CCN Access
 */

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

static const struct file_operations axxia_ccn_offset_proc_ops = {
	.read       = axxia_ccn_offset_read,
	.write      = axxia_ccn_offset_write,
	.llseek     = noop_llseek,
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

static const struct file_operations axxia_ccn_value_proc_ops = {
	.read       = axxia_ccn_value_read,
	.write      = axxia_ccn_value_write,
	.llseek     = noop_llseek,
};

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

	__arm_smccc_smc(0xc3000006, offset, 0, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Getting CCN Register Failed!\n");

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

	__arm_smccc_smc(0xc3000007, offset, value, 0, 0, 0, 0, 0, &res, NULL);

	if (res.a0 != 0)
		pr_warn("Getting CCN Register Failed!\n");

	return;
}
EXPORT_SYMBOL(axxia_ccn_set);

/*
 * ---------------------------------------------------------------------------
 * axxia_dspc_init
 */

static int
axxia_oem_init(void)
{
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

	/* For CCN access, create two files, offset and value. */

	ccn_offset = 0;

	if (proc_create("driver/axxia_ccn_offset", 0200, NULL,
			&axxia_ccn_offset_proc_ops) == NULL)
		pr_err("Could not create /proc/driver/axxia_ccn_offset!\n");
	else if (proc_create("driver/axxia_ccn_value", 0200, NULL,
			     &axxia_ccn_value_proc_ops) == NULL)
		pr_err("Could not create /proc/driver/axxia_ccn_value!\n");
	else
		pr_info("Axxia CCN Initialized\n");

	return 0;
}

device_initcall(axxia_oem_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Axxia OEM Control");
