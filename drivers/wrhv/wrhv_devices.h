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

#ifndef _LINUX_WRHV_DEVICES
#define _LINUX_WRHV_DEVICES

#define WRHV_DEVICE_MAX_NAME_LEN	16
#define WRHV_DEVICE_MAX_VQS		4
#define WRHV_DEVICE_MAX_FEATURES_LEN	4
#define WRHV_DEVICE_MAX_CONFIG_LEN	4

/*
 * This is how we expect the device configuration field for a virtqueue
 * to be laid out in config space.
 */
struct wrhv_vqconfig {
	/* The number of bytes for the buffer */
	__u32 num;
	/* The interrupt we get when something happens. */
	__u32 irq;
};

struct wrhv_device_desc {
	char name[WRHV_DEVICE_MAX_NAME_LEN];
	/* The device type: console, network, disk etc. Type 0 terminates. */
	__u8 type;
	/* The number of virtqueus (first in config array) */
	__u8 num_vq;
	/*
	 * The number of bytes of feature bits.  Multiply by 2: one for host
	 * features and one for Guest acknowledgements.
	 */
	__u8 feature_len;
	/* The number of bytes of the config array after virtqueues. */
	__u8 config_len;
	/* A status byte, written by the Guest. */
	__u8 status;
	struct wrhv_vqconfig vqconfig[WRHV_DEVICE_MAX_VQS];
	__u8 features[WRHV_DEVICE_MAX_FEATURES_LEN * 2];
	__u8 config[WRHV_DEVICE_MAX_CONFIG_LEN];
};

struct wrhv_device;
struct wrhv_virtqueue;

struct wrhv_device_ops {
	int (*init_dev)(struct wrhv_device *vdev);
	struct intr_device_channel_buffer *(*alloc_idc)(struct wrhv_device *vdev);
	int (*init_idc)(struct wrhv_device *vdev, unsigned int index,
			struct wrhv_virtqueue *vq);
};

#define WRHV_DEVICE_MODE_INT	1

struct wrhv_device {
	unsigned int mode;
	struct virtio_device vdev;
	struct wrhv_device_desc *desc;
	struct intr_device_channel_buffer *idc;
	vbi_pdc_handle	pdc_handle;
	struct wrhv_device_ops *ops;
};

#define to_wrhvdev(vd) container_of(vd, struct wrhv_device, vdev)

#define WRHV_DEVICE_VQ_IN	1
#define WRHV_DEVICE_VQ_OUT	2
#define WRHV_DEVICE_VQ_NOFREE	4

struct wrhv_virtqueue {
	struct virtqueue vq;
	void *buf;	/* the virtqueue idc buffer */
	size_t len;	/* the idc buffer length */
	unsigned int mode;	/* in or out */
	u32 *ph;	/* head pointer, must be consistent with ptr in idc */
	u32 *pt;	/* tail pointer, must be consistent with ptr in idc */
	void *read_buf;	/* the buffer used by guest for read */
	u32 read_len;	/* the read buffer len */
	void *data;	/* Token for callbacks */
};

#define to_wrhvvq(v) container_of(v, struct wrhv_virtqueue, vq)


extern int wrhv_add_device(struct wrhv_device_desc *d,
			struct wrhv_device_ops *ops);
extern int wrhv_init_device(struct wrhv_device *vdev);
extern int wrhv_init_vq(struct wrhv_device *vdev, unsigned int index,
			struct wrhv_virtqueue *vq);
#endif /* _LINUX_WRHV_DEVICES */
