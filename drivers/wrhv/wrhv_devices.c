/*
 *  Copyright (C) 2011 Wind River Systems, Inc.
 *
 *  This file is based on drivers/lguest/lguest_device.c
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
#include <linux/circ_buf.h>
#include "wrhv_devices.h"

/*
 * Device configurations
 *
 * The configuration information for a device consists of
 * virtqueues, a feature bitmap, and some configuration bytes.
 */
static inline struct wrhv_vqconfig *wrhv_vq(struct wrhv_device_desc *desc)
{
	return desc->vqconfig;
}

static inline u8 *wrhv_features(struct wrhv_device_desc *desc)
{
	return desc->features;
}

static inline u8 *wrhv_config(struct wrhv_device_desc *desc)
{
	return desc->config;
}

/* The total size of the config page used by this device (incl. desc) */
static inline unsigned desc_size(struct wrhv_device_desc *desc)
{
	return sizeof(*desc);
}

static u32 wrhv_get_features(struct virtio_device *vdev)
{
	unsigned int i;
	u32 features = 0;
	struct wrhv_device_desc *desc = to_wrhvdev(vdev)->desc;
	u8 *in_features = wrhv_features(desc);

	for (i = 0; i < min(desc->feature_len * 8, 32); i++)
		if (in_features[i / 8] & (1 << (i % 8)))
			features |= (1 << i);

	return features;
}

/*
 * The virtio core takes the features the Host offers, and copies the ones
 * supported by the driver into the vdev->features array.  Once that's all
 * sorted out, this routine is called so we can tell the Host which features we
 * understand and accept.
 */
static void wrhv_finalize_features(struct virtio_device *vdev)
{
	unsigned int i, bits;
	struct wrhv_device_desc *desc = to_wrhvdev(vdev)->desc;
	u8 *out_features = wrhv_features(desc) + WRHV_DEVICE_MAX_FEATURES_LEN;

	memset(out_features, 0, desc->feature_len);
	bits = min_t(unsigned, desc->feature_len, sizeof(vdev->features)) * 8;
	for (i = 0; i < bits; i++) {
		if (test_bit(i, vdev->features))
			out_features[i / 8] |= (1 << (i % 8));
	}
}

/*
 * Reading and writing elements in config space
 */
static void wrhv_get(struct virtio_device *vdev, unsigned int offset,
			void *buf, unsigned len)
{
	struct wrhv_device_desc *desc = to_wrhvdev(vdev)->desc;

	BUG_ON(offset + len > desc->config_len);
	memcpy(buf, wrhv_config(desc) + offset, len);
}

static void wrhv_set(struct virtio_device *vdev, unsigned int offset,
			const void *buf, unsigned len)
{
	struct wrhv_device_desc *desc = to_wrhvdev(vdev)->desc;

	BUG_ON(offset + len > desc->config_len);
	memcpy(wrhv_config(desc) + offset, buf, len);
}

/*
 * The operations to get and set the status word just access the status field
 * of the device descriptor.
 */
static u8 wrhv_get_status(struct virtio_device *vdev)
{
	return to_wrhvdev(vdev)->desc->status;
}

static void wrhv_set_status(struct virtio_device *vdev, u8 status)
{
	to_wrhvdev(vdev)->desc->status = status;
}

static void wrhv_reset(struct virtio_device *vdev)
{
	wrhv_set_status(vdev, 0);
}

static int wrhv_vq_buf_space(struct wrhv_virtqueue *vq)
{
	u32 h = *vq->ph, t = *vq->pt;
	size_t len = vq->len;

	return CIRC_SPACE(h, t, len);
}

static int wrhv_vq_buf_count(struct wrhv_virtqueue *vq)
{
	u32 h = *vq->ph, t = *vq->pt;
	size_t len = vq->len;

	return CIRC_CNT(h, t, len);
}

static int wrhv_add_sg(struct wrhv_virtqueue *vq,
			struct scatterlist *sg)
{
	u32 len = sg->length;
	size_t l = vq->len;
	u32 h = *vq->ph, t = *vq->pt;

	while (len) {
		u32 free = wrhv_vq_buf_space(vq);
		u32 s, r;

		if (!free) {
			vq->vq.vq_ops->kick(&vq->vq);
			continue;
		}

		s = min(len, free);
		len -= s;

		r = CIRC_SPACE_TO_END(h, t, l);
		if (s  > r) {
			memcpy(vq->buf + h, sg_virt(sg), r);
			memcpy(vq->buf, sg_virt(sg) + r, s - r);
		} else
			memcpy(vq->buf + h, sg_virt(sg), s);

		*vq->ph = (h + s) % vq->len;
	}
	return 0;
}

static int wrhv_add_buf(struct virtqueue *_vq,
			struct scatterlist *sg,
			unsigned int out,
			unsigned int in,
			void *data)
{
	struct wrhv_virtqueue *vq = to_wrhvvq(_vq);

	vq->data = data;
	while (out--) {
		wrhv_add_sg(vq, sg);
		sg = sg_next(sg);
	}

	WARN_ONCE(in > 1, "We only support one read buffer now");

	if (in) {
		vq->read_buf = sg_virt(sg);
		vq->read_len = sg->length;
	}

	return 0;
}

static void *wrhv_get_buf(struct virtqueue *_vq, unsigned int *len)
{
	struct wrhv_virtqueue *vq = to_wrhvvq(_vq);
	u32 n, r;
	u32 h = *vq->ph, t = *vq->pt;
	size_t l = vq->len;
	unsigned char *pt = vq->buf + t;

	if (!(vq->mode & WRHV_DEVICE_VQ_IN))
		return vq->data;

	n = wrhv_vq_buf_count(vq);
	if (!n) {
		*len = 0;
		return NULL;
	}

	n = min(n, vq->read_len);
	r = CIRC_CNT_TO_END(h, t, l);
	if (n > r) {
		memcpy(vq->read_buf, pt, r);
		memcpy(vq->read_buf + r, vq->buf, n - r);
	} else
		memcpy(vq->read_buf, pt, n);

	*vq->pt = (t + n) % vq->len;

	*len = n;
	return vq->data;
}

static void wrhv_kick(struct virtqueue *_vq)
{
	struct wrhv_virtqueue *vq = to_wrhvvq(_vq);
	struct wrhv_device *vdev = to_wrhvdev(vq->vq.vdev);

	if (!(vq->mode & WRHV_DEVICE_VQ_OUT))
		return;

	vbi_pdc_op(vdev->pdc_handle, PDC_REQUEST_WRITE, 0,
			vq->buf, vq->len, 0);
}

static struct virtqueue_ops wrhv_vq_ops = {
	.add_buf = wrhv_add_buf,
	.get_buf = wrhv_get_buf,
	.kick = wrhv_kick,
};

static irqreturn_t wrhv_interrupt(int irq, void *_vq)
{
	struct wrhv_virtqueue *vq = _vq;

	/*
	 * Don't run the input virtual queue callback function before we
	 * fill the read buffer.
	 */
	if ((vq->mode & WRHV_DEVICE_VQ_IN) &&
		(!vq->read_buf || !vq->len))
		return IRQ_HANDLED;

	if (vq->vq.callback)
		vq->vq.callback(&vq->vq);

	return IRQ_HANDLED;
}

int wrhv_init_vq(struct wrhv_device *vdev, unsigned int index,
			struct wrhv_virtqueue *vq)
{
	struct intr_device_channel_buffer *idc = vdev->idc;

	/* index 0 should be rx queue, index 1 should be tx queue */
	switch (index) {
	case 0:
		idc->rxBuf = vq->buf;
		idc->rxBufLen = vq->len;
		vq->ph = &idc->rxBufWrPtr;
		vq->pt = &idc->rxBufRdPtr;
		vq->mode |= WRHV_DEVICE_VQ_IN;
		break;
	case 1:
		idc->txBuf = vq->buf;
		idc->txBufLen = vq->len;
		vq->ph = &idc->txBufWrPtr;
		vq->pt = &idc->txBufRdPtr;
		vq->mode |= WRHV_DEVICE_VQ_OUT;
		break;
	default:
		pr_err("unsupported queue for wrhv virtual device\n");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wrhv_init_vq);

/*
 * This routine finds the Nth virtqueue described in the configuration of
 * this device and sets it up.
 */
static struct virtqueue *wrhv_find_vq(struct virtio_device *vdev,
					unsigned int index,
					void (*callback)(struct virtqueue *vq),
					const char *name)
{
	struct wrhv_device *wdev = to_wrhvdev(vdev);
	struct wrhv_virtqueue *vq;
	struct wrhv_vqconfig *vconfig = wrhv_vq(wdev->desc) + index;
	int err;

	if (index >= wdev->desc->num_vq)
		return ERR_PTR(-ENOENT);

	vq = kzalloc(sizeof(*vq), GFP_KERNEL);
	if (!vq)
		return ERR_PTR(-ENOMEM);

	vq->vq.callback = callback;
	vq->vq.vdev = vdev;
	vq->vq.vq_ops = &wrhv_vq_ops;
	vq->vq.name = name;
	vq->vq.priv = vconfig;
	vq->len = vconfig->num;

	if (wdev->ops && wdev->ops->init_idc)
		wdev->ops->init_idc(wdev, index, vq);

	if (vconfig->irq) {
		err = request_irq(vconfig->irq, wrhv_interrupt, IRQF_SHARED,
					dev_name(&vdev->dev), vq);
		if (err) {
			kfree(vq->buf);
			kfree(vq);
			return ERR_PTR(err);
		}

		wdev->mode |= WRHV_DEVICE_MODE_INT;
	}

	return &vq->vq;
}

static void wrhv_del_vq(struct virtqueue *vq)
{
	struct wrhv_vqconfig *vconfig = vq->priv;
	struct wrhv_virtqueue *wvq = to_wrhvvq(vq);

	if (vconfig->irq)
		free_irq(vconfig->irq, vq);
	if (!(wvq->mode & WRHV_DEVICE_VQ_NOFREE))
		kfree(wvq->buf);
	kfree(wvq);
}

static void wrhv_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list)
		wrhv_del_vq(vq);
}

int wrhv_init_device(struct wrhv_device *vdev)
{
	if (vbi_pdc_op(vdev->pdc_handle, PDC_REQUEST_IOCTL,
			PDC_IOCTL_SIO_OPEN, 0, 0, 0))
		return -1;

	if (vbi_pdc_op(vdev->pdc_handle, PDC_REQUEST_IOCTL,
			PDC_IOCTL_SIO_MODE_SET, (void *)SIO_MODE_INT,
			SIO_HW_OPTS_CLOCAL, 0))
		return -1;

	return 0;
}
EXPORT_SYMBOL_GPL(wrhv_init_device);

static int wrhv_find_vqs(struct virtio_device *vdev, unsigned nvqs,
			struct virtqueue *vqs[], vq_callback_t *callbacks[],
			const char *names[])
{
	struct wrhv_device *wdev = to_wrhvdev(vdev);
	int i;

	if (nvqs > wdev->desc->num_vq)
		return -ENOENT;

	for (i = 0; i < nvqs; i++) {
		vqs[i] = wrhv_find_vq(vdev, i, callbacks[i], names[i]);
		if (IS_ERR(vqs[i]))
			goto error;
	}

	if (wdev->ops && wdev->ops->init_dev)
		wdev->ops->init_dev(wdev);

	return 0;
error:
	wrhv_del_vqs(vdev);
	return PTR_ERR(vqs[i]);
}

static struct virtio_config_ops wrhv_config_ops = {
	.get_features		= wrhv_get_features,
	.finalize_features	= wrhv_finalize_features,
	.get			= wrhv_get,
	.set			= wrhv_set,
	.get_status		= wrhv_get_status,
	.set_status		= wrhv_set_status,
	.reset			= wrhv_reset,
	.find_vqs		= wrhv_find_vqs,
	.del_vqs		= wrhv_del_vqs,
};

/*
 * The root device for the wrhv virtio devices. This makes them appear as
 * /sys/devices/wrhv/0,1,2 not /sys/devices/0,1,2.
 */
static struct device *wrhv_root;

static int wrhv_device_sanity_check(struct wrhv_device_desc *d)
{
	if (d->num_vq > WRHV_DEVICE_MAX_VQS ||
		d->feature_len > WRHV_DEVICE_MAX_FEATURES_LEN ||
		d->config_len > WRHV_DEVICE_MAX_CONFIG_LEN) {
		pr_err("incorrect description for wrhv device type %u\n",
			d->type);
		return -1;
	}
	return 0;
}


int wrhv_add_device(struct wrhv_device_desc *d, struct wrhv_device_ops *ops)
{
	struct wrhv_device *vdev;

	if (wrhv_device_sanity_check(d))
		return -1;

	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	if (!vdev) {
		pr_err("Cannot allocate wrhv dev type %u\n", d->type);
		return -ENOMEM;
	}

	vdev->vdev.dev.parent = wrhv_root;
	vdev->vdev.id.device = d->type;
	vdev->vdev.config = &wrhv_config_ops;
	vdev->desc = d;
	vdev->ops = ops;

	vdev->idc = ops->alloc_idc(vdev);
	if (!vdev->idc)
		return -ENOMEM;

	if (register_virtio_device(&vdev->vdev)  != 0) {
		pr_err("Failed to register wrhv dev type %u\n", d->type);
		kfree(vdev);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wrhv_add_device);

static int __init wrhv_devices_init(void)
{
	wrhv_root = root_device_register("wrhv");
	if (IS_ERR(wrhv_root))
		panic("Could not register wrhv root");

	return 0;
}
postcore_initcall(wrhv_devices_init);
