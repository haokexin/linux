/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * DMABUF Heaps Userspace API
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2019 Linaro Ltd.
 */
#ifndef _UAPI_LINUX_DMABUF_POOL_H
#define _UAPI_LINUX_DMABUF_POOL_H

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * DOC: DMABUF Heaps Userspace API
 */

/* Valid FD_FLAGS are O_CLOEXEC, O_RDONLY, O_WRONLY, O_RDWR */
#define DMA_HEAP_VALID_FD_FLAGS (O_CLOEXEC | O_ACCMODE)

#if 0
/* Currently no heap flags */
#define DMA_HEAP_VALID_HEAP_FLAGS (0)
#else
/* heap types*/
#ifndef DMA_HEAP_IPC_NORMAL_MEM
#define DMA_HEAP_IPC_NORMAL_MEM      (00000001)     // normal memory, non-secure access
#endif

#ifndef DMA_HEAP_IPC_SECURE_MEM
#define DMA_HEAP_IPC_SECURE_MEM      (00000002)     // secure access memory
#endif

#ifndef DMA_HEAP_IPC_HIFI_MEM
#define DMA_HEAP_IPC_HIFI_MEM        (00000004)     // dmabuf for hifi, non-secure access
#endif

/* Valid HEAP_FLAGS are DMA_HEAP_IPC_NORMAL_MEM, DMA_HEAP_IPC_SECURE_MEM, DMA_HEAP_IPC_HIFI_MEM while using global dma-buf. 
 * You can only choose one from them. Default is DMA_HEAP_IPC_NORMAL_MEM.
 */
#undef DMA_HEAP_VALID_HEAP_FLAGS
#define DMA_HEAP_VALID_HEAP_FLAGS    (DMA_HEAP_IPC_NORMAL_MEM | DMA_HEAP_IPC_SECURE_MEM | DMA_HEAP_IPC_HIFI_MEM)

/* access permission on ipc secure memory */
#ifndef IPC_SECURE_MEM_NONE
#define IPC_SECURE_MEM_NONE     (00000000)      // not allowed to access
#endif  // IPC_SECURE_MEM_NONE

#ifndef IPC_SECURE_MEM_RDWR
#define IPC_SECURE_MEM_RDWR     (00000001)      // allowed to read and write
#endif  // IPC_SECURE_MEM_RDWR

#ifndef DMA_HEAP_IPC_VALID_ACCESS_FLAGS
#define DMA_HEAP_IPC_VALID_ACCESS_FLAGS (IPC_SECURE_MEM_RDWR | IPC_SECURE_MEM_NONE)
#endif  // DMA_HEAP_IPC_VALID_ACCESS_FLAGS

#endif

/**
 * struct dma_heap_allocation_data - metadata passed from userspace for
 *                                      allocations
 * @len:		size of the allocation
 * @fd:			will be populated with a fd which provides the
 *			handle to the allocated dma-buf
 * @fd_flags:		file descriptor flags used when allocating
 * @heap_flags:		flags passed to heap
 *
 * Provided by userspace as an argument to the ioctl
 */
struct dma_heap_allocation_data {
	__u64 len;
	__u32 fd;
	__u32 fd_flags;
	__u64 heap_flags;
};

/**
 * struct dma_heap_phys_data - data returned to userspace
 * @phys_start:	start of physical address
 * @length:		length of physical address
 */

struct dma_heap_phys_data {
	__u64 phys_start;
	__u32 length;
};

/**
 * struct dma_heap_phys_query - metadata passed from userspace for
 *                              physical address
 * @fd:				dma_buf fd
 * @cnt:			maximum number of struct dma_heap_phys_data to be copied
 * @phys_addrs_ptr: a pointer to dma_heap_phys_data
 * @reserved0:		reserved for future usage
 * @reserved1:		reserved for future usage
 * Provided by userspace as an argument to the ioctl
 */
struct dma_heap_phys_query {
	__u32 fd;
	__u32 cnt;
	__u64 phys_addrs_ptr;
	__u32 reserved0;
	__u32 reserved1;
};

struct dma_heap_global_fd {
	__u32 fd;
	__u64 global_fd;
	__u32 reserved0;
	__u32 reserved1;
};

struct dma_heap_import_fd_data {
	__u64 global_fd;
	__u32 fd_flags;
	__u32 fd;
	__u32 reserved0;
	__u32 reserved1;
};

struct dma_heap_secure_mem_config {
	__u64 master_id_mask;
	__u64 access_flag;
};

#define DMA_HEAP_IOC_MAGIC		'H'

/**
 * DOC: DMA_HEAP_IOCTL_ALLOC - allocate memory from pool
 *
 * Takes a dma_heap_allocation_data struct and returns it with the fd field
 * populated with the dmabuf handle of the allocation.
 */
#define DMA_HEAP_IOCTL_ALLOC	_IOWR(DMA_HEAP_IOC_MAGIC, 0x0,\
				      struct dma_heap_allocation_data)

/**
 * DOC: DMA_HEAP_GET_PHYS_ADDRS - information about physical address
 *
 * Takes an dma_heap_phys_query structure and populates information about
 * physical address.
 */
#define DMA_HEAP_GET_PHYS_ADDRS	_IOWR(DMA_HEAP_IOC_MAGIC, 0x1,\
						struct dma_heap_phys_query)

/**
 * DOC: DMA_HEAP_GET_GLOBAL_FD - get the global fd of dma_fd
 *
 * Takes an dma_heap_global_fd structure and populates global fd about
 * this dma buffer.
 */
#define DMA_HEAP_GET_GLOBAL_FD	_IOWR(DMA_HEAP_IOC_MAGIC, 0x2,\
						struct dma_heap_global_fd)

/**
 * DOC: DMA_HEAP_IOCTL_IMPORT - import global fd
 *
 * Takes an dma_heap_import_fd_data structure and populates a local dma_fd
 * to this buffer.
 */
#define DMA_HEAP_IOCTL_IMPORT	_IOWR(DMA_HEAP_IOC_MAGIC, 0x3,\
						struct dma_heap_import_fd_data)						

/**
 * DOC: DMA_HEAP_CONFIG_SECURE_MEM - Configure secure memory
 *
 * Takes an dma_heap_secure_mem_config structure and configure access permission 
 */
#define DMA_HEAP_CONFIG_SECURE_MEM _IOWR(DMA_HEAP_IOC_MAGIC, 0x4,\
						struct dma_heap_secure_mem_config)

#endif /* _UAPI_LINUX_DMABUF_POOL_H */
