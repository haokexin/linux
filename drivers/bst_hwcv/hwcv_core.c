// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string_helpers.h>
#include "hwcv_common.h"
#include "hwcv_core.h"
#include "hwcv_debugger.h"
#include "hwcv_hw_v2.h"
#include "hwcv_mm.h"
#include "hwcv_uapi.h"

#define WAIT_TIME_OUT_MSEC 10000

struct hwcv_drvdata *hwcv_drvdata;

static int
hwcv_session_manager_init(struct hwcv_session_manager **session_manager_ptr)
{
	struct hwcv_session_manager *session_manager;

	*session_manager_ptr =
		kzalloc(sizeof(struct hwcv_session_manager), GFP_KERNEL);
	if (*session_manager_ptr == NULL)
		return -ENOMEM;

	session_manager = *session_manager_ptr;

	mutex_init(&session_manager->lock);

	idr_init_base(&session_manager->ctx_id_idr, 1);

	return 0;
}

static int hwcv_session_free_remove_idr_cb(int id, void *ptr, void *data)
{
	struct hwcv_session *session = ptr;

	idr_remove(&hwcv_drvdata->session_manager->ctx_id_idr, session->id);
	kfree(session);

	return 0;
}

static int hwcv_session_free_remove_idr(struct hwcv_session *session)
{
	struct hwcv_session_manager *session_manager;

	session_manager = hwcv_drvdata->session_manager;

	mutex_lock(&session_manager->lock);

	session_manager->session_cnt--;
	idr_remove(&session_manager->ctx_id_idr, session->id);

	mutex_unlock(&session_manager->lock);

	return 0;
}

static int
hwcv_session_manager_remove(struct hwcv_session_manager **session_manager_ptr)
{
	struct hwcv_session_manager *session_manager = *session_manager_ptr;

	mutex_lock(&session_manager->lock);

	idr_for_each(&session_manager->ctx_id_idr,
		     &hwcv_session_free_remove_idr_cb, session_manager);
	idr_destroy(&session_manager->ctx_id_idr);

	mutex_unlock(&session_manager->lock);

	kfree(*session_manager_ptr);

	*session_manager_ptr = NULL;

	return 0;
}

#ifdef CONFIG_BST_HWCV_DEBUGGER
static int hwcv_debugger_init(struct hwcv_debugger **debugger_p)
{
	struct hwcv_debugger *debugger;

	*debugger_p = kzalloc(sizeof(struct hwcv_debugger), GFP_KERNEL);
	if (*debugger_p == NULL) {
		pr_err("can not alloc for hwcv debugger\n");
		return -ENOMEM;
	}

	debugger = *debugger_p;

#ifdef CONFIG_BST_HWCV_DEBUG_FS
	mutex_init(&debugger->debugfs_lock);
	INIT_LIST_HEAD(&debugger->debugfs_entry_list);
#endif

#ifdef CONFIG_BST_HWCV_PROC_FS
	mutex_init(&debugger->procfs_lock);
	INIT_LIST_HEAD(&debugger->procfs_entry_list);
#endif

	hwcv_debugfs_init();
	hwcv_procfs_init();

	return 0;
}

static int hwcv_debugger_remove(struct hwcv_debugger **debugger_p)
{
	hwcv_debugfs_remove();
	hwcv_procfs_remove();

	kfree(*debugger_p);
	*debugger_p = NULL;

	return 0;
}
#endif

static struct hwcv_session *hwcv_session_init(void)
{
	int new_id;

	struct hwcv_session_manager *session_manager = NULL;
	struct hwcv_session *session = NULL;

	session_manager = hwcv_drvdata->session_manager;
	if (session_manager == NULL) {
		pr_err("hwcv_session_manager is null!\n");
		return ERR_PTR(-EFAULT);
	}

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&session_manager->lock);

	idr_preload(GFP_KERNEL);
	new_id = idr_alloc_cyclic(&session_manager->ctx_id_idr, session, 1, 0,
				  GFP_NOWAIT);
	idr_preload_end();
	if (new_id < 0) {
		mutex_unlock(&session_manager->lock);

		pr_err("hwcv_session alloc id failed!\n");
		kfree(session);
		return ERR_PTR(new_id);
	}

	session->id = new_id;
	session_manager->session_cnt++;

	mutex_unlock(&session_manager->lock);

	session->tgid = current->tgid;
	session->pname = kstrdup_quotable_cmdline(current, GFP_KERNEL);

	return session;
}

static int hwcv_session_deinit(struct hwcv_session *session)
{
	hwcv_mm_session_release_buffer(session);

	hwcv_session_free_remove_idr(session);

	kfree(session->pname);
	kfree(session);

	return 0;
}

static long hwcv_ioctl_alloc_buffer(struct hwcv_core *core, unsigned long arg,
				    struct hwcv_session *session)
{
	int ret = 0;
	struct hwcv_allocation_data data;
	struct hwcv_buf *buf;

	if (unlikely(copy_from_user(&data, (struct hwcv_allocation_data *)arg,
				    sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy from user\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	if (data.length == 0) {
		dev_err(core->dev, "%s: Invalid request: length[%u]\n",
			__func__, data.length);
		ret = -EINVAL;
		goto err;
	}

	buf = hwcv_mm_alloc_buf(data.length);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		goto err;
	}

	buf->id = hwcv_mm_alloc_id(buf);
	if (buf->id < 0) {
		ret = buf->id;
		goto err_release_buf;
	}

	buf->session = session;

	data.length = buf->length;
	data.fd = buf->fd;
	data.handle = buf->id;
	data.dma_addr = buf->dma_addr;

	if (unlikely(copy_to_user((void *)arg, &data, sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy to user\n", __func__);
		ret = -EFAULT;
		goto err_remove_id;
	}

	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %u", "ID", buf->id);
	pr_debug("%-20s | %u", "fd", buf->fd);
	pr_debug("%-20s | 0x%llx", "PHY ADDR", buf->phys_addr);
	pr_debug("%-20s | 0x%llx", "DMA ADDR", buf->dma_addr);
	pr_debug("%-20s | %u", "Byteused", buf->bytesused);
	pr_debug("%-20s | %u", "Size", buf->length);
	pr_debug("------------------------------------------------------\n");

	return 0;

err_remove_id:
	hwcv_mm_remove_id(buf->id);
err_release_buf:
	hwcv_mm_release_buf(buf);
err:
	return ret;
}

static long hwcv_ioctl_import_buffer(struct hwcv_core *core, unsigned long arg,
				     struct hwcv_session *session)
{
	int ret = 0;
	struct hwcv_import_data data;
	struct hwcv_buf *buf;

	if (unlikely(copy_from_user(&data, (struct hwcv_import_data *)arg,
				    sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy from user\n", __func__);
		ret = -EFAULT;
		goto err;
	}

	if (data.fd == 0 || data.length == 0) {
		dev_err(core->dev, "%s: Invalid request, fd[%u], length[%u]\n",
			__func__, data.fd, data.length);
		ret = -EINVAL;
		goto err;
	}

	buf = hwcv_mm_import_buf(data.fd, data.length);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		goto err;
	}

	buf->id = hwcv_mm_alloc_id(buf);
	if (buf->id < 0) {
		ret = buf->id;
		goto err_release_buf;
	}

	buf->session = session;

	data.length = buf->length;
	data.handle = buf->id;
	data.dma_addr = buf->dma_addr;

	if (unlikely(copy_to_user((void *)arg, &data, sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy to user\n", __func__);
		ret = -EFAULT;
		goto err_remove_id;
	}

	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %u", "ID", buf->id);
	pr_debug("%-20s | %u", "fd", buf->fd);
	pr_debug("%-20s | 0x%llx", "PHY ADDR", buf->phys_addr);
	pr_debug("%-20s | 0x%llx", "DMA ADDR", buf->dma_addr);
	pr_debug("%-20s | %u", "Byteused", buf->bytesused);
	pr_debug("%-20s | %u", "Size", buf->length);
	pr_debug("------------------------------------------------------\n");

	return 0;

err_remove_id:
	hwcv_mm_remove_id(buf->id);
err_release_buf:
	hwcv_mm_release_buf(buf);
err:
	return ret;
}

static long hwcv_ioctl_sync_buffer(struct hwcv_core *core, unsigned long arg)
{
	int i;
	int *handles;
	struct hwcv_sync_data data;
	struct hwcv_buf *buf;

	if (unlikely(copy_from_user(&data, (struct hwcv_sync_data *)arg,
				    sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy from user\n", __func__);
		return -EFAULT;
	}

	if (data.handle_ptr == 0 || data.size == 0 ||
	    data.dir > HWCV_SYNC_FOR_CPU) {
		dev_err(core->dev,
			"%s: Invalid request, ptr[0x%llx], size[%u], dir[%d]\n",
			__func__, data.handle_ptr, data.size, data.dir);
		return -EINVAL;
	}

	handles = kzalloc(sizeof(int) * data.size, GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (unlikely(copy_from_user(handles, u64_to_user_ptr(data.handle_ptr),
				    sizeof(int) * data.size))) {
		kfree(handles);
		dev_err(core->dev, "%s: Failed to copy from user handle_ptr\n",
			__func__);
		return -EFAULT;
	}

	for (i = 0; i < data.size; i++) {
		buf = hwcv_mm_lookup_id(handles[i]);
		if (!buf) {
			kfree(handles);
			return -EINVAL;
		}
		hwcv_mm_sync_buf(buf, data.dir);

		pr_debug(
			"------------------------------------------------------\n");
		pr_debug("%-20s | %u", "Dir", data.dir);
		pr_debug("%-20s | %u", "ID", buf->id);
		pr_debug(
			"------------------------------------------------------\n");
	}

	kfree(handles);

	return 0;
}

static long hwcv_ioctl_release_buffer(struct hwcv_core *core, unsigned long arg)
{
	int id;
	struct hwcv_buf *buf;

	if (unlikely(copy_from_user(&id, (uint32_t *)arg, sizeof(uint32_t)))) {
		dev_err(core->dev, "%s: Failed to copy from user\n", __func__);
		return -EFAULT;
	}

	if (id == 0) {
		dev_err(core->dev, "%s: Invalid request, id[%u]\n", __func__,
			id);
		return -EINVAL;
	}

	buf = hwcv_mm_remove_id(id);
	if (!buf)
		return -ENODEV;

	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %u", "ID", buf->id);
	pr_debug("%-20s | %u", "fd", buf->fd);
	pr_debug("%-20s | 0x%llx", "PHY ADDR", buf->phys_addr);
	pr_debug("%-20s | 0x%llx", "DMA ADDR", buf->dma_addr);
	pr_debug("%-20s | %u", "Byteused", buf->bytesused);
	pr_debug("%-20s | %u", "Size", buf->length);
	pr_debug("------------------------------------------------------\n");

	hwcv_mm_release_buf(buf);

	return 0;
}

static void hwcv_ktime_update(struct hwcv_ktime *kt, u64 new_cur)
{
	kt->cur = new_cur;

	if (kt->min == 0)
		kt->min = new_cur;

	if (new_cur > kt->max)
		kt->max = new_cur;
	else if (new_cur < kt->min)
		kt->min = new_cur;

	kt->sum += new_cur;
	kt->count++;
	kt->average = DIV_ROUND_CLOSEST(kt->sum, kt->count);
}

static void hwcv_stat_update(struct hwcv_profiling *p, struct hwcv_stat *s,
			     bool success)
{
	u64 cur;

	s->total_frames++;

	if (success)
		s->success_frames++;
	else
		s->failed_frames++;

	if (success) {
		cur = ktime_us_delta(p->lock_done, p->request);
		hwcv_ktime_update(&s->get_lock, cur);

		cur = ktime_us_delta(p->config_done, p->lock_done);
		hwcv_ktime_update(&s->config_reg, cur);

		cur = ktime_us_delta(p->frame_done, p->config_done);
		hwcv_ktime_update(&s->complete_frame, cur);

		cur = ktime_us_delta(p->notify_done, p->frame_done);
		hwcv_ktime_update(&s->notify_upper, cur);

		cur = p->work_cycle / 1000;
		hwcv_ktime_update(&s->hw_cycle, cur);
	}
}

static void hwcv_dump_cur_stat(struct hwcv_stat *s)
{
	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %lluus", "Get Lock", s->get_lock.cur);
	pr_debug("%-20s | %lluus", "Config Reg", s->config_reg.cur);
	pr_debug("%-20s | %lluus", "Complete Frame", s->complete_frame.cur);
	pr_debug("%-20s | %lluus", "Notify Upper", s->notify_upper.cur);
	pr_debug("%-20s | %lluus", "HW Cycle", s->hw_cycle.cur);
	pr_debug("------------------------------------------------------\n");
}

static void hwcv_dump_scaler_task(struct hwcv_core *core,
				  struct hwcv_scaler_data *data)
{
	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %s", "Mode", hwcv_get_scaler_mode_str(data->mode));
	pr_debug("%-20s | %s", "Format", hwcv_get_format_name(data->format));
	pr_debug("%-20s | %ux%u(%u)->%ux%u(%u)", "Resolution", data->src_width,
		 data->src_height, data->src_stride, data->dst_width,
		 data->dst_height, data->dst_stride);

	pr_debug("%-20s | x[%u], y[%u]", "Polyphase ratio", data->x_ratio,
		 data->y_ratio);
	pr_debug("%-20s | x[%u], y[%u]", "Polyphase phase", data->x_init_phase,
		 data->y_init_phase);
	pr_debug("%-20s | addr[0x%08x], size[0x%08x]", "Polyphase table",
		 data->coeff_dma_addr, data->coeff_size);

	pr_debug("%-20s | gauss[%u], leftedge[%u], layer_num[%u]",
		 "Pyramid Mode", data->gauss_enable, data->leftedge_split_flag,
		 data->layer_num);

	pr_debug("%-20s | 0x%08x, 0x%08x, 0x%08x", "Src addr",
		 data->src_dma_addr[0], data->src_dma_addr[1],
		 data->src_dma_addr[2]);
	pr_debug("%-20s | 0x%08x, 0x%08x, 0x%08x", "Dst-0 addr",
		 data->dst_dma_addr[0][0], data->dst_dma_addr[0][1],
		 data->dst_dma_addr[0][2]);
	pr_debug("%-20s | 0x%08x, 0x%08x, 0x%08x", "Dst-1 addr",
		 data->dst_dma_addr[1][0], data->dst_dma_addr[1][1],
		 data->dst_dma_addr[1][2]);
	pr_debug("%-20s | 0x%08x, 0x%08x, 0x%08x", "Dst-2 addr",
		 data->dst_dma_addr[2][0], data->dst_dma_addr[2][1],
		 data->dst_dma_addr[2][2]);
	pr_debug("------------------------------------------------------\n");
}

static long hwcv_ioctl_do_scaler(struct hwcv_core *core, unsigned long arg)
{
	int ret;
	struct hwcv_scaler_data data;

	if (unlikely(copy_from_user(&data, (uint32_t *)arg, sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy from user\n", __func__);
		return -EFAULT;
	}

	core->scaler_profiling.request = ktime_get();

	mutex_lock(&core->scaler_lock);
#ifdef CONFIG_BST_HWCV_MULTI_OS
	get_sem_lock(core->scaler_hw_lock);
#endif
	set_bit(HWCV_REQ_SCALER, &core->request_state);
	core->scaler_profiling.lock_done = ktime_get();

	hwcv_dump_scaler_task(core, &data);

	ret = core->ops->do_scaler(core, &data);
	if (ret < 0)
		goto exit;
	core->scaler_profiling.config_done = ktime_get();

#ifdef CONFIG_BST_HWCV_MULTI_OS
	ret = core->ops->poll_scaler(core, WAIT_TIME_OUT_MSEC * 1000);
	if (ret < 0) {
		dev_err(core->dev, "Do scaler timeout");
		core->ops->dump_scaler_regs(core);
		core->ops->reset_scaler(core);
		goto exit;
	}
#else
	ret = wait_for_completion_timeout(&core->scaler_done,
					  msecs_to_jiffies(WAIT_TIME_OUT_MSEC));
	if (ret == 0) {
		dev_err(core->dev, "Do scaler timeout");
		core->ops->dump_scaler_regs(core);
		core->ops->reset_scaler(core);
		ret = -ETIMEDOUT;
		goto exit;
	}
#endif
	core->scaler_profiling.notify_done = ktime_get();

	hwcv_stat_update(&core->scaler_profiling, &core->scaler_stat, true);
	hwcv_dump_cur_stat(&core->scaler_stat);

	clear_bit(HWCV_REQ_SCALER, &core->request_state);
#ifdef CONFIG_BST_HWCV_MULTI_OS
	release_sem_lock(core->scaler_hw_lock);
#endif
	mutex_unlock(&core->scaler_lock);

	return 0;

exit:
	clear_bit(HWCV_REQ_SCALER, &core->request_state);
	hwcv_stat_update(&core->scaler_profiling, &core->scaler_stat, false);

#ifdef CONFIG_BST_HWCV_MULTI_OS
	release_sem_lock(core->scaler_hw_lock);
#endif
	mutex_unlock(&core->scaler_lock);

	return ret;
}

static void hwcv_dump_gwarp_task(struct hwcv_core *core,
				 struct hwcv_gwarp_data *data)
{
	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %s", "Mode", hwcv_get_gwarp_mode_str(data->mode));
	pr_debug("%-20s | %u, %u", "ID", data->engine_id, data->sensor_id);
	pr_debug("%-20s | %s", "Interpolation",
		 hwcv_get_gwarp_algo_str(data->interpolation));

	pr_debug("%-20s | %s->%s", "Format",
		 hwcv_get_format_name(data->src_format),
		 hwcv_get_format_name(data->dst_format));
	pr_debug("%-20s | %ux%u(%u)->%ux%u(%u)", "Resolution", data->src_width,
		 data->src_height, data->src_stride, data->dst_width,
		 data->dst_height, data->dst_stride);

	pr_debug("%-20s | 0x%08x, 0x%08x, 0x%08x", "Src addr",
		 data->src_dma_addr[0], data->src_dma_addr[1],
		 data->src_dma_addr[2]);
	pr_debug("%-20s | 0x%08x, 0x%08x, 0x%08x", "Dst addr",
		 data->dst_dma_addr[0], data->dst_dma_addr[1],
		 data->dst_dma_addr[2]);

	pr_debug("%-20s | addr[0x%08x], stride[0x%08x]", "LUT",
		 data->lut_dma_addr, data->lut_stride);
	pr_debug("------------------------------------------------------\n");
}

static long hwcv_ioctl_do_gwarp(struct hwcv_core *core, unsigned long arg)
{
	int ret;
	int id;
	struct hwcv_gwarp_data data;

	if (unlikely(copy_from_user(&data, (uint32_t *)arg, sizeof(data)))) {
		dev_err(core->dev, "%s: Failed to copy from user\n", __func__);
		return -EFAULT;
	}

	id = data.engine_id;
	if (id >= HWCV_MAX_GWARP_NUM) {
		dev_err(core->dev, "%s: Invalid id %d\n", __func__, id);
		return -EINVAL;
	}

	core->gwarp_profiling[id].request = ktime_get();
	mutex_lock(&core->gwarp_lock[id]);
#ifdef CONFIG_BST_HWCV_MULTI_OS
	get_sem_lock(core->gwarp_hw_lock[id]);
#endif
	set_bit(HWCV_REQ_GWARP0 + id, &core->request_state);
	core->gwarp_profiling[id].lock_done = ktime_get();

	hwcv_dump_gwarp_task(core, &data);

	ret = core->ops->do_gwarp(core, &data);
	if (ret < 0)
		goto exit;
	core->gwarp_profiling[id].config_done = ktime_get();

#ifdef CONFIG_BST_HWCV_MULTI_OS
	ret = core->ops->poll_gwarp(core, id, WAIT_TIME_OUT_MSEC * 1000);
	if (ret < 0) {
		dev_err(core->dev, "Do gwarp %d timeout", id);
		core->ops->dump_gwarp_regs(core, id);
		core->ops->reset_gwarp(core, id);
		goto exit;
	}
#else
	ret = wait_for_completion_timeout(&core->gwarp_done[id],
					  msecs_to_jiffies(WAIT_TIME_OUT_MSEC));
	if (ret == 0) {
		dev_err(core->dev, "Do gwarp %d timeout", id);
		core->ops->dump_gwarp_regs(core, id);
		core->ops->reset_gwarp(core, id);
		ret = -ETIMEDOUT;
		goto exit;
	}
#endif
	core->gwarp_profiling[id].notify_done = ktime_get();

	hwcv_stat_update(&core->gwarp_profiling[id], &core->gwarp_stat[id],
			 true);
	hwcv_dump_cur_stat(&core->gwarp_stat[id]);

	clear_bit(HWCV_REQ_GWARP0 + id, &core->request_state);
#ifdef CONFIG_BST_HWCV_MULTI_OS
	release_sem_lock(core->gwarp_hw_lock[id]);
#endif
	mutex_unlock(&core->gwarp_lock[id]);

	return 0;

exit:
	hwcv_stat_update(&core->gwarp_profiling[id], &core->gwarp_stat[id],
			 false);

	clear_bit(HWCV_REQ_GWARP0 + id, &core->request_state);
#ifdef CONFIG_BST_HWCV_MULTI_OS
	release_sem_lock(core->gwarp_hw_lock[id]);
#endif
	mutex_unlock(&core->gwarp_lock[id]);
	return ret;
}

static long hwcv_ioctl(struct file *file, uint32_t cmd, unsigned long arg)
{
	int ret = 0;
	struct hwcv_session *session = file->private_data;
	struct hwcv_core *core = hwcv_drvdata->core;
	struct hwcv_version driver_version;

	switch (cmd) {
	case HWCV_IOCTL_ALLOC:
		ret = hwcv_ioctl_alloc_buffer(core, arg, session);

		break;
	case HWCV_IOCTL_IMPORT:
		ret = hwcv_ioctl_import_buffer(core, arg, session);

		break;
	case HWCV_IOCTL_SYNC:
		ret = hwcv_ioctl_sync_buffer(core, arg);

		break;
	case HWCV_IOCTL_RELEASE:
		ret = hwcv_ioctl_release_buffer(core, arg);

		break;
	case HWCV_IOCTL_SCALER:
		ret = hwcv_ioctl_do_scaler(core, arg);

		break;
	case HWCV_IOCTL_GWARP:
		ret = hwcv_ioctl_do_gwarp(core, arg);

		break;
	case HWCV_IOCTL_GET_DRVIER_VERSION:
		driver_version.major = DRIVER_MAJOR_VERISON;
		driver_version.minor = DRIVER_MINOR_VERSION;
		driver_version.revision = DRIVER_REVISION_VERSION;
		strscpy((char *)driver_version.str, DRIVER_VERSION,
			sizeof(driver_version.str));

		if (copy_to_user((void *)arg, &driver_version,
				 sizeof(driver_version)))
			ret = -EFAULT;
		else
			ret = true;

		break;
	default:
		dev_err(core->dev, "Invalid cmd: 0x%x", cmd);
		ret = -EINVAL;
	}

	return ret;
}

static int hwcv_open(struct inode *inode, struct file *file)
{
	struct hwcv_session *session = NULL;

	session = hwcv_session_init();
	if (IS_ERR(session))
		return PTR_ERR(session);

	file->private_data = (void *)session;

	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %d", "ID", session->id);
	pr_debug("%-20s | %d", "PID", session->tgid);
	pr_debug("%-20s | %s", "CMD", session->pname);
	pr_debug("%-20s | %d", "CNT",
		 hwcv_drvdata->session_manager->session_cnt);
	pr_debug("------------------------------------------------------\n");

	return nonseekable_open(inode, file);
}

static int hwcv_release(struct inode *inode, struct file *file)
{
	struct hwcv_session *session = file->private_data;

	pr_debug("------------------------------------------------------\n");
	pr_debug("%-20s | %d", "ID", session->id);
	pr_debug("%-20s | %d", "PID", session->tgid);
	pr_debug("%-20s | %s", "CMD", session->pname);
	pr_debug("%-20s | %d", "CNT",
		 hwcv_drvdata->session_manager->session_cnt);
	pr_debug("------------------------------------------------------\n");

	hwcv_session_deinit(session);

	return 0;
}

static irqreturn_t __maybe_unused hwcv_irq_handler(int irq, void *data)
{
	irqreturn_t irq_ret = IRQ_NONE;
	struct hwcv_core *core = data;

	if (core->ops->irq)
		irq_ret = core->ops->irq(core);

	return irq_ret;
}

static irqreturn_t __maybe_unused hwcv_isr_thread(int irq, void *data)
{
	int i;
	irqreturn_t irq_ret = IRQ_NONE;
	struct hwcv_core *core = data;
	unsigned long offset =
		HWCV_GWARP1_NORMAL_DONE - HWCV_GWARP0_NORMAL_DONE;

	if (core->ops->isr_thread)
		irq_ret = core->ops->isr_thread(core);

	if (test_bit(HWCV_SCALER_DONE, &core->job_state)) {
		clear_bit(HWCV_SCALER_DONE, &core->job_state);
		complete(&core->scaler_done);
	}

	/* Normal Gwarp Done */
	for (i = 0; i < core->hw_data->gwarp_num; i++) {
		if (test_bit(HWCV_GWARP0_NORMAL_DONE + i * offset,
			     &core->job_state)) {
			clear_bit(HWCV_GWARP0_NORMAL_DONE + i * offset,
				  &core->job_state);
			complete(&core->gwarp_done[i]);
		}
	}

	/* Sbs Gwarp Done */
	if (core->hw_data->support_sbs) {
		for (i = 0; i < core->hw_data->gwarp_num; i++) {
			if (test_bit(HWCV_GWARP0_SNR0_DONE + i * offset,
				     &core->job_state)) {
				clear_bit(HWCV_GWARP0_SNR0_DONE + i * offset,
					  &core->job_state);
				complete(&core->gwarp_done[i]);
			} else if (test_bit(HWCV_GWARP0_SNR1_DONE + i * offset,
					    &core->job_state)) {
				clear_bit(HWCV_GWARP0_SNR1_DONE + i * offset,
					  &core->job_state);
				complete(&core->gwarp_done[i]);
			} else if (test_bit(HWCV_GWARP0_SNR2_DONE + i * offset,
					    &core->job_state)) {
				clear_bit(HWCV_GWARP0_SNR2_DONE + i * offset,
					  &core->job_state);
				complete(&core->gwarp_done[i]);
			} else if (test_bit(HWCV_GWARP0_SNR3_DONE + i * offset,
					    &core->job_state)) {
				clear_bit(HWCV_GWARP0_SNR3_DONE + i * offset,
					  &core->job_state);
				complete(&core->gwarp_done[i]);
			}
		}
	}

	return irq_ret;
}

static const struct file_operations hwcv_fops = {
	.owner = THIS_MODULE,
	.open = hwcv_open,
	.release = hwcv_release,
	.unlocked_ioctl = hwcv_ioctl,
};

struct miscdevice hwcv_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HWCV_DRIVER_NAME,
	.fops = &hwcv_fops,
};

static const struct hwcv_match_data c1200_match_data = {
	.device_type = HWCV_DEVICE_C1200,
	.ops = &hwcv_v2_ops,
	.hw_data = &hwcv_v2_data,
};

static const struct of_device_id hwcv_dt_ids[] = {
	{
		.compatible = "bst,c1200-hwcv",
		.data = &c1200_match_data,
	},
	{},
};

static int init_hwcv_core(struct hwcv_core *core, struct device *dev,
			  const struct hwcv_match_data *match_data)
{
	int i;
#ifdef CONFIG_BST_HWCV_MULTI_OS
	int ret;
	u32 sem_id[3];
	int mst_id, bank_id, msg_id;
#endif

	if (match_data->device_type > HWCV_DEVICE_C1200)
		return -EINVAL;

	core->ops = match_data->ops;
	core->hw_data = match_data->hw_data;
	core->dev = dev;
	mutex_init(&core->scaler_lock);
	init_completion(&core->scaler_done);
	for (i = 0; i < core->hw_data->gwarp_num; i++) {
		mutex_init(&core->gwarp_lock[i]);
		init_completion(&core->gwarp_done[i]);
	}

#ifdef CONFIG_BST_HWCV_MULTI_OS
	ret = of_property_read_u32_array(dev->of_node, "ipc-sem", sem_id, 3);
	if (ret) {
		dev_err(dev, "Failed to get ipc-sem in device tree\n");
		return -EINVAL;
	}
	mst_id = sem_id[0];
	bank_id = sem_id[1];
	msg_id = sem_id[2];
	dev_info(dev, "ipc-sem id: <%d %d %d>", mst_id, bank_id, msg_id);

	core->scaler_hw_lock = bst_semaphore_init(mst_id, bank_id, msg_id);
	msg_id++;
	for (i = 0; i < core->hw_data->gwarp_num; i++)
		core->gwarp_hw_lock[i] =
			bst_semaphore_init(mst_id, bank_id, msg_id + i);
#endif

	return 0;
}

void *hwcv_ioremap_regs(struct platform_device *pdev, int res_index)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, res_index);
	if (!res) {
		dev_err(&pdev->dev, "Failed to IORESOURCE_MEM %d\n", res_index);
		return res;
	}

	dev_info(&pdev->dev, "Reg region %d start: 0x%08llX, end: 0x%08llX\n",
		 res_index, res->start, res->end);

	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int hwcv_probe(struct platform_device *pdev)
{
	int ret;
	struct hwcv_core *core;
	const struct hwcv_match_data *match_data;
	struct hwcv_drvdata *drvdata = hwcv_drvdata;
	struct device *dev = &pdev->dev;

	if (!dev->of_node) {
		pr_err("%s: dts node is null\n", __func__);
		return -EFAULT;
	}

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;
	drvdata->core = core;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	ret = init_hwcv_core(core, dev, match_data);
	if (ret < 0) {
		dev_err(dev, "Failed to init hwcv core: %d\n", ret);
		return ret;
	}

	core->sys_base = hwcv_ioremap_regs(pdev, 0);
	if (IS_ERR_OR_NULL(core->sys_base))
		return PTR_ERR(core->sys_base);

	core->scaler_base = hwcv_ioremap_regs(pdev, 1);
	if (IS_ERR_OR_NULL(core->scaler_base))
		return PTR_ERR(core->scaler_base);

	core->gwarp_base[0] = hwcv_ioremap_regs(pdev, 2);
	if (IS_ERR_OR_NULL(core->gwarp_base[0]))
		return PTR_ERR(core->gwarp_base[0]);

	core->gwarp_base[1] = hwcv_ioremap_regs(pdev, 3);
	if (IS_ERR_OR_NULL(core->gwarp_base[1]))
		return PTR_ERR(core->gwarp_base[1]);

#ifdef CONFIG_BST_HWCV_MULTI_OS
	dev_info(dev, "Support multi os\n");
#else
	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0) {
		dev_err(dev, "Failed to get irq in dts: %d\n", ret);
		return core->irq;
	}

	ret = devm_request_threaded_irq(dev, core->irq, hwcv_irq_handler,
					hwcv_isr_thread,
					IRQF_TRIGGER_HIGH | IRQF_SHARED,
					dev_driver_string(dev), core);
	if (ret < 0) {
		dev_err(dev, "Failed to request irq: %d\n", ret);
		return ret;
	}
#endif

#ifdef CONFIG_BST_HWCV_MULTI_OS
	get_sem_lock(core->scaler_hw_lock);
	if (!core->ops->is_ready(core))
		core->ops->init_hw(core);
	else
		dev_info(dev, "Hardware has been inited");
	release_sem_lock(core->scaler_hw_lock);
#else
	core->ops->init_hw(core);
#endif

	platform_set_drvdata(pdev, core);

	dev_info(dev, "probe successfully, irq[%d]\n", core->irq);

	return 0;
}

static int hwcv_remove(struct platform_device *pdev)
{
	return 0;
}

static int hwcv_suspend(struct device *dev)
{
	dev_info(dev, "Suspend\n");

	return 0;
}

static int hwcv_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hwcv_core *core = platform_get_drvdata(pdev);

	dev_info(dev, "Resume\n");
#ifdef CONFIG_BST_HWCV_MULTI_OS
	get_sem_lock(core->scaler_hw_lock);
	if (!core->ops->is_ready(core))
		core->ops->init_hw(core);
	else
		dev_info(dev, "Hardware has been inited");
	release_sem_lock(core->scaler_hw_lock);
#else
	core->ops->init_hw(core);
#endif

	return 0;
}

static const struct dev_pm_ops hwcv_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hwcv_suspend, hwcv_resume)
};

static struct platform_driver hwcv_driver = {
	.probe   = hwcv_probe,
	.remove  = hwcv_remove,
	.driver  = {
		.name = HWCV_DRIVER_NAME,
		.of_match_table = of_match_ptr(hwcv_dt_ids),
		.pm = &hwcv_pm_ops,
	},
};

static int __init hwcv_driver_init(void)
{
	int ret;

	hwcv_drvdata = kzalloc(sizeof(struct hwcv_drvdata), GFP_KERNEL);
	if (hwcv_drvdata == NULL)
		return -ENOMEM;

	ret = platform_driver_register(&hwcv_driver);
	if (ret < 0) {
		pr_err("%s: Failed to register platform driver hwcv: %d\n",
		       __func__, ret);
		goto err_free_drvdata;
	}

	ret = misc_register(&hwcv_dev);
	if (ret < 0) {
		pr_err("%s: Failed to register miscdev hwcv: %d\n", __func__,
		       ret);
		goto err_unregister_hwcv;
	}

	ret = hwcv_mm_init(&hwcv_drvdata->mm);
	if (ret < 0) {
		pr_err("%s: Failed to init mm: %d\n", __func__, ret);
		goto err_unregister_misc;
	}

	ret = hwcv_session_manager_init(&hwcv_drvdata->session_manager);
	if (ret < 0) {
		pr_err("%s: Failed to init session manager: %d\n", __func__,
		       ret);
		goto err_remove_mm;
	}

#ifdef CONFIG_BST_HWCV_DEBUGGER
	ret = hwcv_debugger_init(&hwcv_drvdata->debugger);
	if (ret < 0) {
		pr_err("%s: Failed to init session debugger: %d\n", __func__,
		       ret);
		goto err_remove_session_manager;
	}
#endif

	pr_info("HWCV module initialized, version[%s]\n", DRIVER_VERSION);

	return 0;

#ifdef CONFIG_BST_HWCV_DEBUGGER
err_remove_session_manager:
	hwcv_session_manager_remove(&hwcv_drvdata->session_manager);
#endif

err_remove_mm:
	hwcv_mm_remove(&hwcv_drvdata->mm);

err_unregister_misc:
	misc_deregister(&hwcv_dev);

err_unregister_hwcv:
	platform_driver_unregister(&hwcv_driver);

err_free_drvdata:
	kfree(hwcv_drvdata);

	return ret;
}

late_initcall(hwcv_driver_init);

static void __exit hwcv_driver_exit(void)
{
#ifdef CONFIG_BST_HWCV_DEBUGGER
	hwcv_debugger_remove(&hwcv_drvdata->debugger);
#endif

	hwcv_mm_remove(&hwcv_drvdata->mm);

	hwcv_session_manager_remove(&hwcv_drvdata->session_manager);

	platform_driver_unregister(&hwcv_driver);

	misc_deregister(&hwcv_dev);

	pr_info("HWCV module exited\n");
}

module_exit(hwcv_driver_exit);

#ifdef MODULE_IMPORT_NS
MODULE_IMPORT_NS(DMA_BUF);
#endif
MODULE_DESCRIPTION("BST HWCV driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BST Ltd.");
