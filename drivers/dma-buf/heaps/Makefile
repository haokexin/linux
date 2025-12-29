# SPDX-License-Identifier: GPL-2.0

ccflags-${CONFIG_DMABUF_MSGBX_R5MEM} += \
		-I$(srctree)/drivers/bst_ipc_msgbx/ipc_app_layer/include \
		-I$(srctree)/drivers/bst_ipc_msgbx/include \
		-I$(srctree)/drivers/bst_ipc_msgbx/ipc_trans_layer/include

obj-$(CONFIG_DMABUF_HEAPS_DEFERRED_FREE) += deferred-free-helper.o
obj-$(CONFIG_DMABUF_HEAPS_PAGE_POOL)	+= page_pool.o
obj-$(CONFIG_DMABUF_HEAPS_SYSTEM)	+= system_heap.o
obj-$(CONFIG_DMABUF_HEAPS_CMA)		+= cma_heap.o
obj-$(CONFIG_DMABUF_HEAPS_SECURE)	+= secure_heap.o
obj-$(CONFIG_DMABUF_HEAPS_CMA_BST)	+= system_heap_ipc.o
obj-$(CONFIG_DMABUF_MSGBX_R5MEM)	+= dmabuf-ipc-src-gen/r5mem_client.o dmabuf-ipc-src-gen/dmabuf_ipc_client.o
obj-$(CONFIG_DMABUF_HEAPS_SRAM)		+= sram_heap.o
