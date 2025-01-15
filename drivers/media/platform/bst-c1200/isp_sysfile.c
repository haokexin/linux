// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>

#include <bst/ipc_interface.h>
#include <linux/coreip/proto_api_common.h>

#include "isp_core.h"
#include "isp_fw_loader.h"
#include "isp_sysfile.h"
#include "isp_video.h"
#include "proto_isp_ipc.h"

#include "video_server.h"

static struct c1200_isp_device *s_isp;

static void dump_tx_data(struct c1200_isp_device *isp)
{
	uint32_t *msg;
	uint32_t msg_addr_offset;

	if (!isp->isp_client_registed)
		return;

	dev_err(isp->dev, "%s: media_cmd_paddr: 0x%08X\n", __func__,
		isp->media_cmd_paddr);
	msg_addr_offset = isp->media_cmd_paddr - 0x80000000 -
			  (isp->init_paddr & LOW_32_BIT_MASK);
	msg = (uint32_t *)(isp->init_vaddr + msg_addr_offset);

	dev_err(isp->dev, "%s: media_cmd_data: 0x%08X 0x%08X 0x%08X 0x%08X\n",
		__func__, msg[0], msg[1], msg[2], msg[3]);
}

static inline int send_msg_async(struct c1200_isp_device *isp)
{
	struct media_command *media_cmd;
	int ret;
	uint32_t msg;

	media_cmd = isp_get_media_cmd(isp);
	msg = isp_cmd_pa(isp, media_cmd);

	memset(media_cmd, 0, sizeof(struct media_command));
	media_cmd->cmd_hdr.hdr_info.cmd_type_minor = MINOR_ISP_BOOTLD_RECONF;
	pr_err("start send msgbox to riscv");
	if (!isp->msg_server)
		return -1;
	ret = isp->msg_server->video_server.arm2isp(msg);
	if (ret < 0)
		pr_err("send method failed. ret is %d\n", ret);
	dump_tx_data(isp);
	return ret;
}

ssize_t show_isp_fw(struct kobject *object, struct kobj_attribute *attr,
		    char *buf)
{
	pr_err("start send msgbox to riscv");
	if (!s_isp->use_ipc)
		send_msg_async(s_isp);
	return 0;
}

ssize_t store_isp_fw(struct kobject *object, struct kobj_attribute *attr,
		     const char *buf, size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	if (value == ISP_FW_LOAD_RUN) { /*load fw*/
		bst_boot_isp_fw(ISP_FW_BIN_PATH, ISP_FW_SLAB_PATH, s_isp);
		atomic_set(&(s_isp->FW_load_started), 1);
	}
#ifdef ECHO_TEST
	else if (value == ISP_ECHO_TEST) { /*run fw*/
		isp_fw_echo_test(s_isp);
	}
#endif
	else if (value == ISP_CATCH_TEST) { /*run fw*/
		// isp_cache_test(s_isp);
	} else {
		pr_info("please type in correct number\n");
	}

	return count;
}

static struct kobj_attribute s_isp_fw_attribute =
	__ATTR(isp_fw, 0664, show_isp_fw, store_isp_fw);

ssize_t isp_log_level_show(struct kobject *object, struct kobj_attribute *attr,
			   char *buf)
{
	return 0;
}

ssize_t isp_log_level_store(struct kobject *object, struct kobj_attribute *attr,
			    const char *buf, size_t count)
{
	return count;
}

static struct kobj_attribute s_isp_log_level_attribute =
	__ATTR(isp_log_level, 0664, isp_log_level_show, isp_log_level_store);

static uint32_t date_convert_hex_to_dec(uint32_t hex_date)
{
	uint32_t dec_date;
	uint32_t year, month, day;

	day = hex_date % 0x100;
	month = (hex_date / 0x100) % 0x100;
	year = hex_date / 0x10000;
	dec_date = year * 10000 + month * 100 + day;

	return dec_date;
}

static ssize_t isp_version_show(struct kobject *object,
				struct kobj_attribute *attr, char *buf)
{
	int ret;
	int isp_fpk_v = s_isp->fw_pack_version;

	ret = sprintf(
		buf,
		"ISP driver: %s, Firmware: %x.%x.%x.%x, ID: %x, date: %d\n",
		s_isp->revision, isp_fpk_v / 0x1000,
		(isp_fpk_v % 0x1000) / 0x100, (isp_fpk_v % 0x100) / 0x10,
		isp_fpk_v % 0x10, s_isp->fw_svn_version,
		date_convert_hex_to_dec(s_isp->fw_build_date));

	return ret;
}

static struct kobj_attribute s_isp_version_attribute =
	__ATTR(isp_version, 0444, isp_version_show, NULL);

ssize_t isp_video_debug_show(struct kobject *object,
			     struct kobj_attribute *attr, char *buf)
{
	int ret;

	dump_video_index_mapping_info(s_isp, buf);
	ret = strlen(buf);

	return ret > PAGE_SIZE ? PAGE_SIZE : ret;
}

ssize_t isp_video_debug_store(struct kobject *object,
			      struct kobj_attribute *attr, const char *buf,
			      size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	if (value < MAX_ISP_CHANNEL)
		dump_video_debug_info(&(s_isp->channels[value].views_video));
	else
		pr_err("error video index = %lu\n", value);

	return count;
}

static struct kobj_attribute s_isp_video_debug_attribute = __ATTR(
	isp_video_debug, 0664, isp_video_debug_show, isp_video_debug_store);

static struct attribute *attrs[] = {
	&s_isp_fw_attribute.attr, &s_isp_video_debug_attribute.attr,
	&s_isp_log_level_attribute.attr, &s_isp_version_attribute.attr,
	//&s_isp_sub_attribute.attr,
	NULL, /* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *isp_kobj;

int isp_sysfs_init(struct c1200_isp_device *isp)
{
	int ret = -1;

	s_isp = isp;

	isp_kobj = kobject_create_and_add("isp", NULL);
	if (!isp_kobj) {
		pr_err("kobject_create_and_add failed");
		return -ENOMEM;
	}

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(isp_kobj, &attr_group);
	if (ret) {
		pr_err("sysfs_create_group failed, error code = %d", ret);
		kobject_put(isp_kobj);
	}

	return ret;
}
