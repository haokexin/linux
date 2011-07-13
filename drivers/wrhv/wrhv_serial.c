/*
 *  Copyright (C) 2011 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_console.h>
#include <vbi/vbi.h>
#include <vbi/pdc.h>
#include <vbi/duart.h>
#include <linux/scatterlist.h>
#include <linux/hrtimer.h>
#include "wrhv_devices.h"

static struct hrtimer *wrhv_serial_timer;

extern vbi_pdc_handle duart_pdc;
extern struct intr_device_channel_buffer idc;
extern char wrhv_duart_name[];

#ifdef CONFIG_WRHV_VIRTIO_SERIAL_CONSOLE
/* interface to wrhv duart actual device driver */
#define WRHV_DUART_RX_SIZE	(16)
#define WRHV_DUART_TX_SIZE	(1024)

void wrhv_duart_putc(char c);
void wrhv_duart_init(void);

static int __init wrhv_early_put_chars(u32 vtermo, const char *buf, int count)
{
	int i;

	for (i = 0; i < count; i++, buf++)
		wrhv_duart_putc(*buf);

	return count;
}

static int __init wrhv_virtio_console_init(void)
{
	wrhv_duart_init();

	return virtio_cons_early_init(wrhv_early_put_chars);
}
console_initcall(wrhv_virtio_console_init);
#endif

#define WRHV_MAX_SERIAL_DEVICES 4
struct wrhv_serial_dev {
	char name[16];
	int irq;
};
struct wrhv_serial_dev wrhv_serial_devs[WRHV_MAX_SERIAL_DEVICES];

#define WRHV_CLASS_SERIAL	0
#define WRHV_TYPE_ADD		1

extern struct vb_config *wr_vb_config;
void wrhv_get_serial_devices(void)
{
	struct vb_dev_info *pdev = wr_vb_config->deviceConfiguration;
	struct vb_dev_int_info *pint;
	int num = wr_vb_config->numDevices, i, j;
	struct wrhv_serial_dev *p;
	static int done;

	if (done)
		return;
	else
		done = 1;

	j = 0;
	for (i = 0; i < num; i++, pdev++) {
		if (pdev->deviceClass != WRHV_CLASS_SERIAL ||
			pdev->deviceType != WRHV_TYPE_ADD)
			continue;

		p = &wrhv_serial_devs[j];
		strcpy(p->name, pdev->deviceName);
		if (pdev->numInterrupts) {
			pint = (struct vb_dev_int_info *)((char *)pdev +
					pdev->intInfoOffset);
			p->irq = pint ? pint->intNum : 0;
		}
		j++;
		if (j >= WRHV_MAX_SERIAL_DEVICES)
			break;
	}
}
EXPORT_SYMBOL_GPL(wrhv_get_serial_devices);

char *wrhv_get_serial_dev_name(int index)
{
	if (index < 0 || index >= WRHV_MAX_SERIAL_DEVICES)
		return NULL;

	return wrhv_serial_devs[index].name;
}
EXPORT_SYMBOL(wrhv_get_serial_dev_name);

static int is_console_device(struct wrhv_device *vdev)
{
	return !strcmp(vdev->desc->name, wrhv_duart_name);
}

extern void hvc_kick(void);
static enum hrtimer_restart wrhv_serial_kick(struct hrtimer *t)
{
	ktime_t now;

	hvc_kick();

	now = hrtimer_cb_get_time(t);
	hrtimer_forward(t, now, ktime_set(0, 50000000));
	return HRTIMER_RESTART;
}

static int wrhv_serial_init_device(struct wrhv_device *vdev)
{
	if (is_console_device(vdev))
		vdev->pdc_handle = duart_pdc;
	else {
		/* init channel */
		if (vbi_pdc_init(vdev->desc->name, &vdev->pdc_handle))
			return -1;

		/* init device */
		if (vbi_pdc_op(vdev->pdc_handle, PDC_REQUEST_INIT, 0,
			vdev->idc, 0, 0))
			return -1;
	}

	if (!(vdev->mode & WRHV_DEVICE_MODE_INT) && !wrhv_serial_timer) {
		struct hrtimer *t = kzalloc(sizeof(*t), GFP_KERNEL);

		if (!t)
			return -1;
		hrtimer_init(t, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		t->function = wrhv_serial_kick;
		hrtimer_start(t, ktime_set(0, 1000000000), HRTIMER_MODE_REL);
		wrhv_serial_timer = t;
	}

	return wrhv_init_device(vdev);
}

static struct intr_device_channel_buffer *wrhv_serial_alloc_idc(struct wrhv_device *vdev)
{
	if (is_console_device(vdev))
		vdev->idc = &idc;
	else
		vdev->idc = kzalloc(sizeof(*vdev->idc), GFP_KERNEL);

	return vdev->idc;
}

static int wrhv_serial_init_idc(struct wrhv_device *vdev, unsigned int index,
				struct wrhv_virtqueue *vq)
{
	if (is_console_device(vdev)) {
		switch (index) {
		case 0:
			vq->buf = vdev->idc->rxBuf;
			break;
		case 1:
			vq->buf = vdev->idc->txBuf;
			break;
		}
		vq->mode |= WRHV_DEVICE_VQ_NOFREE;
	} else {
		vq->buf = kmalloc(vq->len, GFP_KERNEL);
		if (!vq->buf)
			return -ENOMEM;
	}

	wrhv_init_vq(vdev, index, vq);
	return 0;
}

static struct wrhv_device_ops wrhv_serial_ops = {
	.init_dev	= wrhv_serial_init_device,
	.alloc_idc	= wrhv_serial_alloc_idc,
	.init_idc	= wrhv_serial_init_idc,
};

struct wrhv_device_desc wrhv_init_serial_device = {
	.name		= "uart0",
	.type		= VIRTIO_ID_CONSOLE,
	.num_vq		= 2,
	.feature_len	= 1,
	.config_len	= 1,
	.vqconfig[0]	= {
	.num		= 16,
		.irq	= 20,
	},
	.vqconfig[1]	= {
		.num	= 1024,
	},
};

static __init int wrhv_add_serial(void)
{
	struct wrhv_device_desc *desc;
	struct wrhv_serial_dev *p;
	int i;

	wrhv_get_serial_devices();

	p = wrhv_serial_devs;
	for (i = 0; i < WRHV_MAX_SERIAL_DEVICES; i++, p++) {
		if (!p->name[0])
			break;

		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			return -ENOMEM;

		*desc = wrhv_init_serial_device;
		sprintf(desc->name, p->name);
		desc->vqconfig[0].irq = p->irq;
		wrhv_add_device(desc, &wrhv_serial_ops);
	}

	return 0;
}
module_init(wrhv_add_serial);
