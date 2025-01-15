/*
 * XRP driver IOCTL codes and data structures
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Alternatively you can use and distribute this file under the terms of
 * the GNU General Public License version 2 or later.
 */

#ifndef _XRP_KERNEL_DEFS_H
#define _XRP_KERNEL_DEFS_H

#define XRP_IOCTL_MAGIC 'r'
#define XRP_IOCTL_ALLOC		_IO(XRP_IOCTL_MAGIC, 1)
#define XRP_IOCTL_FREE		_IO(XRP_IOCTL_MAGIC, 2)
#define XRP_IOCTL_QUEUE		_IO(XRP_IOCTL_MAGIC, 3)
#define XRP_IOCTL_QUEUE_NS	_IO(XRP_IOCTL_MAGIC, 4)
#define XRP_IOCTL_ALLOC_SYNC_CMD	_IO(XRP_IOCTL_MAGIC, 5)
#define XRP_IOCTL_SYNC_CMD	_IO(XRP_IOCTL_MAGIC, 6)
#define XRP_IOCTL_REGISTER_CB	_IO(XRP_IOCTL_MAGIC, 7)
#define XRP_IOCTL_DEF_ALG_SET	_IO(XRP_IOCTL_MAGIC, 8)
#define XRP_IOCTL_USERDEF_ALG_SET	_IO(XRP_IOCTL_MAGIC, 9)
#define XRP_IOCTL_ALG_RESULT_GET	_IO(XRP_IOCTL_MAGIC, 10)
#define XRP_IOCTL_ALG_FLUSH	_IO(XRP_IOCTL_MAGIC, 11)
#define XRP_IOCTL_ALLOC_NSID		_IO(XRP_IOCTL_MAGIC, 12)
#define XRP_IOCTL_SYNC_CMD_PIO		_IO(XRP_IOCTL_MAGIC, 13)
#define XRP_IOCTL_DEF_ALG_SET_PIO		_IO(XRP_IOCTL_MAGIC, 14)
#define XRP_IOCTL_ALG_RESULT_GET_PIO		_IO(XRP_IOCTL_MAGIC, 15)
#define XRP_IOCTL_ALG_FLUSH_PIO		_IO(XRP_IOCTL_MAGIC, 16)

struct xrp_ioctl_alloc {
	__u32 size;
	__u32 align;
	__u64 addr;
};

enum {
	XRP_FLAG_READ = 0x1,
	XRP_FLAG_WRITE = 0x2,
	XRP_FLAG_READ_WRITE = 0x3,
};

struct xrp_ioctl_buffer {
	__u32 flags;
	__u32 size;
	__u64 addr;
};

enum {
	XRP_QUEUE_FLAG_NSID = 0x4,
	XRP_QUEUE_FLAG_PRIO = 0xff00,
	XRP_QUEUE_FLAG_PRIO_SHIFT = 8,

	XRP_QUEUE_VALID_FLAGS =
		XRP_QUEUE_FLAG_NSID |
		XRP_QUEUE_FLAG_PRIO,
};

struct xrp_ioctl_queue {
	__u32 flags;
	__u32 in_data_size;
	__u32 out_data_size;
	__u32 buffer_size;
	__u64 in_data_addr;
	__u64 out_data_addr;
	__u64 buffer_addr;
	__u64 nsid_addr;
};

struct xrp_sync_cmd {
	char input_str[64];
	char out_str[64];
	__u32 cmd_addr;
	__u32 cmd_size;
};
struct xrp_cb {
	__u32 cb_addr;
	__u32 param_addr;
};
struct xrp_alg_param {
	__u32 alg_flg;
	__u32 mask;	
};
struct xrp_output {
	__u32 output_addr;
	__u32 outsize;	
};

struct xrp_nsid {
	__u32 nsid_addr;
	__u32 nsid_size;
};
#endif
