// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/dma-direct.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/iommu.h>

#include "isp_sysfs.h"

#include "isp_core.h"
#include "isp_hw.h"
#include "isp_msg.h"

static ssize_t files_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct isp_device *isp;
	int len;
	int i;

	isp = dev_get_drvdata(dev);
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%11s %9s %s\n", "Start PA",
			"Size", "Path");
	if (!isp->shared)
		return len;

	for (i = 0; i < isp->shared->file.num; ++i) {
		struct isp_file *file;

		file = &isp->shared->file.files[i];
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%llX %9zu %s\n",
				dma_to_phys(isp->dev, file->dma), file->size,
				file->path);
	}

	return len;
}
static struct device_attribute files_attr =
	__ATTR(files, 0444, files_show, NULL);

static ssize_t flags_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct isp_device *isp;
	int len;

	isp = dev_get_drvdata(dev);
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%-11s: 0x%08X\n", "Flags",
			isp->flags);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-11s: %u\n", "Merge msg",
			isp->merge_msg);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-11s: %u\n",
			"Cfg updated", isp->cfg_updated);

	return len;
}

static ssize_t flags_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int rv;
	struct isp_device *isp;
	u32 value;

	isp = dev_get_drvdata(dev);
	rv = kstrtou32(buf, 0, &value);
	if (rv)
		return rv;
	isp->flags = value;

	return count;
}
static struct device_attribute flags_attr =
	__ATTR(flags, 0644, flags_show, flags_store);

static ssize_t info_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct isp_device *isp;
	int len;
	int i;

	isp = dev_get_drvdata(dev);
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"%2s %20s %6s %4s %10s %12s\n", "id", "name", "online",
			"fps", "row time", "I2C-Cam/Ser");

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		struct isp_channel *channel;
		struct camera_dev *cam;

		channel = &isp->channels[i];
		if (!channel->enabled)
			continue;

		cam = channel->cam_dev;
		len += snprintf(buf + len, PAGE_SIZE - len,
				"%2d %20s %6d %4d %10d %2d-0x%02X/0x%02X\n",
				channel->cid, cam->name,
				channel->cfg->sensorOnline, cam->sensor_fps,
				cam->row_time, cam->i2c_client->adapter->nr,
				cam->sensor_alias, cam->ser_alias);
	}

	return len;
}
static struct device_attribute info_attr = __ATTR(info, 0444, info_show, NULL);

static ssize_t mem_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct isp_device *isp;

	isp = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%08X\n", isp->rw_addr_msg.value);
}

static ssize_t mem_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int rv;
	struct isp_device *isp;
	char op;
	u64 pa;
	u32 value;
	dma_addr_t iova;
	size_t off;
	size_t size;

	isp = dev_get_drvdata(dev);
	rv = sscanf(buf, "%c %llx %x", &op, &pa, &value);
	if (op == 'r') {
		iova = phys_to_dma(dev, pa);
		isp->rw_addr_msg.value = 0;
		isp->rw_addr_msg.flag = 0;
	} else if (op == 'w') {
		iova = phys_to_dma(dev, pa);
		isp->rw_addr_msg.value = value;
		isp->rw_addr_msg.flag = 1;
	} else {
		dev_err(dev, "Invalid rw command: %s\n", buf);
		return -EINVAL;
	}

	dev_dbg(dev,
		"count: %zu, op: %c, addr: 0x%08llX -> 0x%llX, value: 0x%X\n",
		count, op, iova, pa, value);
	if (iova == DMA_MAPPING_ERROR) {
		dev_err(dev, "Invalid rw addr: 0x%llX\n", pa);
		return -EINVAL;
	}

	off = 0;
	size = sizeof(value);
	if (isp->iommud && !isp_is_mapped_addr(isp, iova)) {
		off = iova - PTR_ALIGN_DOWN(iova, IOVA_ALIGN_SIZE);
		iova -= off;
		pa -= off;
		size = ALIGN(size + off, IOVA_ALIGN_SIZE);
		if (isp_is_fixed_iova(iova))
			pa = iova;
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		if (isp_is_fixed_iova(iova))
			rv = iommu_map_by_proxy(COREIP_ISP_SID, iova, pa, size);
		else
			rv = iommu_map(isp->iommud, iova, pa, size,
				       IOMMU_READ | IOMMU_WRITE);
#else
		rv = iommu_map(isp->iommud, iova, pa, size,
			       IOMMU_READ | IOMMU_WRITE);
#endif
		if (rv < 0) {
			dev_err(dev,
				"Failed to map 0x%08llX, size: %lu, rv: %d\n",
				pa, PAGE_SIZE, rv);
			return -EFAULT;
		}
	}

	dev_dbg(dev,
		"Map 0x%08lX -> 0x%llX, size: %10lu/0x%08zX, access: 0x%08lX\n",
		(unsigned long)iova, pa, size, size, (unsigned long)iova + off);
	isp->rw_addr_msg.addr = iova + off;
	rv = isp_msg_rw_addr(isp);
	if (rv) {
		dev_err(dev, "Failed to r/w mem by firmware: %d\n", rv);
		rv = -EFAULT;
	} else {
		rv = count;
	}

	if (isp->iommud && !isp_is_mapped_addr(isp, iova)) {
#ifdef CONFIG_VIDEO_BST_ISP_MULTI_OS
		if (isp_is_fixed_iova(iova))
			iommu_unmap_by_proxy(COREIP_ISP_SID, iova, 0, size);
		else
			iommu_unmap(isp->iommud, iova, size);
#else
		if (isp->iommud)
			iommu_unmap(isp->iommud, iova, size);
#endif
	}

	return rv;
}
static struct device_attribute mem_attr =
	__ATTR(mem, 0644, mem_show, mem_store);

static ssize_t mmap_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	int rv;
	struct isp_device *isp;
	unsigned long iova;
	phys_addr_t pa;
	size_t size;

	isp = dev_get_drvdata(dev);
	if (!isp->iommud)
		return -EINVAL;

	if (sscanf(buf, "%lx %llx %zx", &iova, &pa, &size) != 3)
		return -EINVAL;

	rv = iommu_map(isp->iommud, iova, pa, size, IOMMU_READ | IOMMU_WRITE);
	if (rv < 0) {
		dev_err(dev,
			"Failed to map 0x%08lX to 0x%08llX, size: %lu, rv: %d\n",
			iova, pa, size, rv);
		return -EFAULT;
	}

	return count;
}
static struct device_attribute mmap_attr = __ATTR(mmap, 0644, NULL, mmap_store);

static ssize_t munmap_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct isp_device *isp;
	unsigned long iova;
	size_t size;

	isp = dev_get_drvdata(dev);
	if (!isp->iommud)
		return -EINVAL;

	if (sscanf(buf, "%lx %zx", &iova, &size) != 2)
		return -EINVAL;

	iommu_unmap(isp->iommud, iova, size);

	return count;
}
static struct device_attribute munmap_attr =
	__ATTR(munmap, 0644, NULL, munmap_store);

static ssize_t msg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct isp_device *isp;
	tSoneCmdp *cmdp;
	ptrdiff_t diff;
	int len;

	isp = dev_get_drvdata(dev);
	cmdp = (tSoneCmdp *)isp->msg.cmdp_va;
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "Memory Layout:\n");
	len += snprintf(buf + len, PAGE_SIZE - len,
			"    %-15s: 0x%09llX/0x%08X\n", "init start",
			(u64)isp->msg.init_pa, (u32)isp->msg.init_dma);
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: 0x%09llX\n",
			"init size", (u64)isp->msg.init_size);
	diff = (void *)cmdp->ch[FW_CH_INDEX].cqueue.c0 - isp->msg.init_va;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"    %-15s: 0x%09llX/0x%08X\n", "cmd fw start",
			(u64)(isp->msg.init_pa + diff),
			(u32)(isp->msg.init_dma + diff));
	diff = (void *)isp->msg.cmds - isp->msg.init_va;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"    %-15s: 0x%09llX/0x%08X\n", "cmd drv start",
			(u64)(isp->msg.init_pa + diff),
			(u32)(isp->msg.init_dma + diff));
#ifdef CONFIG_BST_IPC
	len += snprintf(buf + len, PAGE_SIZE - len,
			"    %-15s: 0x%09llX/0x%08X\n", "ipc base",
			(u64)isp->ipc.msg_pa, (u32)isp->ipc.msg_dma);
#endif
	diff = (void *)isp->shared - isp->msg.init_va;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"    %-15s: 0x%09llX/0x%08X\n", "shared start",
			(u64)(isp->msg.init_pa + diff),
			(u32)(isp->msg.init_dma + diff));
	len += snprintf(buf + len, PAGE_SIZE - len, "Message Statistic:\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: %8lu\n",
			"TX all", isp->msg.tx_all);
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: %8lu\n",
			"TX done", isp->msg.tx_done);
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: %8lu\n",
			"TX fail", isp->msg.tx_fail);
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: %8lu\n",
			"RX all", isp->msg.rx_all);
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: %8lu\n",
			"RX good", isp->msg.rx_good);
	len += snprintf(buf + len, PAGE_SIZE - len, "    %-15s: %8lu\n",
			"RX bad", isp->msg.rx_bad);

	return len;
}
static struct device_attribute msg_attr = __ATTR(msg, 0444, msg_show, NULL);

static ssize_t rsv_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct isp_device *isp;
	int len;

	isp = dev_get_drvdata(dev);
	len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "0x%llX\n", isp->fw.rsv_pa);

	return len;
}
static struct device_attribute rsv_attr = __ATTR(rsv, 0444, rsv_show, NULL);

static ssize_t trigger_cfg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_trigger *trigger = &isp->trigger;

	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n", "GPIO",
			trigger->gpio);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n", "Period(us)",
			trigger->period);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n", "Polarity",
			trigger->polarity);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n",
			"Pulse Width(us)", trigger->width);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n",
			"Trigger num", trigger->num);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n", "Run CPU",
			trigger->cpu);

	return len;
}

static ssize_t trigger_cfg_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_trigger *trigger = &isp->trigger;

	if (trigger->running) {
		dev_err(dev, "Trigger is running, can not set\n");
		return -EBUSY;
	}

	if (sscanf(buf, "%u %u %u %u %u %u", &trigger->gpio, &trigger->period,
		   &trigger->polarity, &trigger->width, &trigger->num,
		   &trigger->cpu) != 6) {
		dev_err(dev,
			"Invalid trigger command: %s\nShould be: <gpio> <period> <polarity> <width> <num> <cpu>\n",
			buf);
		return -EINVAL;
	}

	if (!gpio_to_desc(trigger->gpio)) {
		dev_err(dev, "Invalid trigger gpio %u\n", trigger->gpio);
		return -EINVAL;
	}

	trigger->interval = trigger->period * 1000;

	return count;
}
static struct device_attribute trigger_cfg_attr =
	__ATTR(trigger_cfg, 0644, trigger_cfg_show, trigger_cfg_store);

static inline void trigger_signal(struct isp_trigger *trigger)
{
	int rv;
	struct isp_device *isp = trigger_to_isp(trigger);

	rv = gpio_direction_output(trigger->gpio, trigger->polarity);
	if (rv)
		dev_err(isp->dev, "Failed to set gpio %u to %u\n",
			trigger->gpio, trigger->polarity);
	udelay(trigger->width);
	rv = gpio_direction_output(trigger->gpio, !trigger->polarity);
	if (rv)
		dev_err(isp->dev, "Failed to set gpio %u to %u\n",
			trigger->gpio, !trigger->polarity);
}

static enum hrtimer_restart hrtimer_trigger(struct hrtimer *timer)
{
	struct isp_trigger *trigger = timer_to_trigger(timer);

	local_irq_disable();
	if (trigger->curr >= trigger->num) {
		local_irq_enable();
		return HRTIMER_NORESTART;
	}

	*(trigger->ts + trigger->curr) = ktime_get_boottime_ns();
	++trigger->curr;
	trigger_signal(trigger);
	trigger->next = ktime_add_ns(trigger->next, trigger->interval);
	hrtimer_set_expires(timer, trigger->next);

	local_irq_enable();
	return HRTIMER_RESTART;
}

static ssize_t trigger_run_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u32 curr;
	int len = 0;
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_trigger *trigger = &isp->trigger;
	u64 *ts;
	u32 i;
	u64 gap;

	curr = trigger->curr;
	ts = trigger->ts;
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %s\n", "Running",
			trigger->running ? "true" : "false");
	if (!trigger->ts)
		return len;

	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %u\n", "Triggered",
			curr);
	gap = (*(trigger->ts + curr - 1) - *trigger->ts) / 1000;
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %lluus\n",
			"Total gap", gap);
	len += snprintf(buf + len, PAGE_SIZE - len, "%-16s: %lluus\n",
			"Average gap", gap / (curr - 1));
	len += snprintf(buf + len, PAGE_SIZE - len, "First 10 timestamps:\n");
	for (i = 0; i < min_t(u32, 10, curr); ++i)
		len += snprintf(buf + len, PAGE_SIZE - len, "    %12lluns\n",
				*(ts + i));
	if (curr > 10) {
		ts = trigger->ts + curr - 10;
		len += snprintf(buf + len, PAGE_SIZE - len,
				"Last 10 timestamps:\n");
		for (i = 0; i < min_t(u32, 10, curr); ++i)
			len += snprintf(buf + len, PAGE_SIZE - len,
					"    %12lluns\n", *(ts + i));
	}

	return len;
}

static ssize_t trigger_run_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_trigger *trigger = &isp->trigger;
	int status;
	u32 val;
	int rv;

	status = kstrtou32(buf, 0, &val);
	if (status) {
		dev_err(dev, "Failed to get run val, status: %d\n", status);
		return -EIO;
	}

	if (!trigger->gpio || !trigger->period || !trigger->width ||
	    !trigger->num) {
		dev_err(dev, "Trigger config is invalid\n");
		return -EINVAL;
	}

	if (val) {
		struct cpumask mask;

		if (trigger->running)
			return -EBUSY;

		if (!trigger->ts)
			trigger->ts =
				vmalloc(sizeof(*trigger->ts) * trigger->num);
		if (!trigger->ts) {
			dev_err(dev,
				"Failed to allocate memory to recode timestamp\n");
			return -ENOMEM;
		}

		rv = gpio_request(trigger->gpio, "isp-trigger");
		if (rv) {
			dev_err(dev,
				"Failed to request trigger gpio %u, rv: %d\n",
				trigger->gpio, rv);
			return -EINVAL;
		}
		cpumask_clear(&mask);
		cpumask_set_cpu(trigger->cpu, &mask);
		rv = set_cpus_allowed_ptr(current, &mask);
		if (rv)
			dev_err(dev,
				"Failed to set trigger timer to CPU %u, rv: %d\n",
				trigger->cpu, rv);
		trigger->curr = 0;
		hrtimer_init(&trigger->timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_ABS);
		trigger->timer.function = hrtimer_trigger;
		trigger->next = ktime_get_boottime();
		trigger_signal(trigger);
		*(trigger->ts + trigger->curr++) = ktime_to_ns(trigger->next);
		trigger->next = ktime_add_ns(trigger->next, trigger->interval);
		hrtimer_start(&trigger->timer, trigger->next, HRTIMER_MODE_ABS);
		trigger->running = true;
	} else {
		if (trigger->running)
			hrtimer_cancel(&trigger->timer);
		gpio_free(trigger->gpio);
		vfree(trigger->ts);
		trigger->ts = NULL;
		trigger->running = false;
	}

	return count;
}
static struct device_attribute trigger_run_attr =
	__ATTR(trigger_run, 0644, trigger_run_show, trigger_run_store);

static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int rv;
	struct isp_device *isp;
	u32 ver;
	u32 date;

	isp = dev_get_drvdata(dev);
	ver = isp->shared->fw.version;
	date = isp->shared->fw.build_date;

	rv = snprintf(
		buf, PAGE_SIZE,
		"ISP driver: %s, Firmware: %d.%d.%d.%d, SCM ID: %x, date: %04d-%02d-%02d, fw stage: %s, role: %s\n",
		ISP_DRIVER_VERSION, FW_VER_MAJOR(ver), FW_VER_MINOR(ver),
		FW_VER_PATCH(ver), FW_VER_CUST(ver), isp->shared->fw.scm_id,
		FW_DATE_YEAR(date), FW_DATE_MONTH(date), FW_DATE_DAY(date),
		isp_str_fw_stage(isp->shared->fw.stage),
		isp_str_role(isp->role));

	return rv;
}
static struct device_attribute version_attr =
	__ATTR(version, 0444, version_show, NULL);

static ssize_t videos_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct isp_device *isp;
	struct isp_channel *channel;
	struct isp_video *video;
	int i;
	int len;

	isp = dev_get_drvdata(dev);
	len = 0;
	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		channel = &isp->channels[i];
		if (!channel->enabled)
			continue;

		video = &channel->views_video;

		/*Dump video info*/
		len += snprintf(buf + len, PAGE_SIZE - len,
				"===== V%02d Mapping info =====\n", video->vid);
		len += snprintf(buf + len, PAGE_SIZE - len, "%-20s:%-36s\n",
				"Camera name", channel->cam_dev->name);
		len += snprintf(buf + len, PAGE_SIZE - len, "%-20s:%-3d\n",
				"Camera fps", channel->cam_dev->sensor_fps);
		len += snprintf(buf + len, PAGE_SIZE - len, "%-20s:%-3d\n",
				"isp_channel_index", video->cid);
		len += snprintf(buf + len, PAGE_SIZE - len, "%-20s:%-3d\n",
				"mipi_index", channel->cfg->mipiSensorIndex);
		len += snprintf(buf + len, PAGE_SIZE - len, "%-20s:%-10d\n",
				"row_time", channel->cam_dev->row_time);
		len += snprintf(buf + len, PAGE_SIZE - len, "%-20s:%-5s\n",
				"connected",
				channel->cfg->sensorOnline ? "true" : "false");
		len += snprintf(buf + len, PAGE_SIZE - len,
				"=================================\n");
	}

	for (i = 0; i < ARRAY_SIZE(isp->channels); ++i) {
		channel = &isp->channels[i];
		if (!channel->enabled) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"ispChannel[%d] is disabled\n", i);
			continue;
		}
		if (channel->cam_dev == NULL) {
			len += snprintf(
				buf + len, PAGE_SIZE - len,
				"ispChannel[%d]'s cam_dev is disabled\n", i);
			continue;
		}
	}

	return len;
}
static struct device_attribute videos_attr =
	__ATTR(videos, 0444, videos_show, NULL);

/* clang-format off */
static struct attribute *attrs[] = {
	&files_attr.attr,
	&flags_attr.attr,
	&info_attr.attr,
	&mem_attr.attr,
	&mmap_attr.attr,
	&munmap_attr.attr,
	&msg_attr.attr,
	&rsv_attr.attr,
	&trigger_cfg_attr.attr,
	&trigger_run_attr.attr,
	&version_attr.attr,
	&videos_attr.attr,
	NULL,
};
/* clang-format on */

static const struct attribute_group attr_group = {
	.attrs = attrs,
};

int isp_sysfs_init(struct isp_device *isp)
{
	int rv;
	struct device *dev;

	dev = isp->dev;

	rv = sysfs_create_group(&dev->kobj, &attr_group);
	if (rv) {
		dev_err(dev, "Failed to create sysfs group, rv: %d\n", rv);
		return rv;
	}

	/* TODO: support multiple devices, the name should be isp%d */
	rv = sysfs_create_link(NULL, &dev->kobj, "isp");
	if (rv) {
		dev_err(dev, "Failed to create sysfs link, rv: %d\n", rv);

		return rv;
	}

	return 0;
}

void isp_sysfs_exit(struct isp_device *isp)
{
	/* TODO: support multiple devices, the name should be isp%d */
	sysfs_remove_link(NULL, "isp");
	sysfs_remove_group(&isp->dev->kobj, &attr_group);
}
