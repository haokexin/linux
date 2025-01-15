// SPDX-License-Identifier: GPL-2.0

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>

#include <linux/dma-buf.h>
#include <drm/drm.h>

#include "../../../../include/uapi/linux/dma-heap.h"

#define DEVPATH "/dev/dma_heap"

#define CTR_IDC_SHIFT           28
#define CTR_DIC_SHIFT           29

void __aarch64_inval_dcache_range (const void *base, const void *end)
{
  unsigned dcache_lsize;
  static unsigned int cache_info = 0;
  const char *address;

  if (! cache_info)
    /* CTR_EL0 [3:0] contains log2 of icache line size in words.
       CTR_EL0 [19:16] contains log2 of dcache line size in words.  */
    asm volatile ("mrs\t%0, ctr_el0":"=r" (cache_info));

 	dcache_lsize = 4 << ((cache_info >> 16) & 0xF);

    /* Make the start address of the loop cache aligned.  */
    address = (const char*) ((__UINTPTR_TYPE__) base
			     & ~ (__UINTPTR_TYPE__) (dcache_lsize - 1));

    for (; address < (const char *) end; address += dcache_lsize)
      asm volatile ("dc\tcivac, %0"
		    :
		    : "r" (address)
		    : "memory");

  asm volatile ("dsb\tsy" : : : "memory");
  asm volatile("isb" : : : "memory");
}

void __aarch64_clean_dcache_range (const void *base, const void *end)
{
  unsigned dcache_lsize;
  static unsigned int cache_info = 0;
  const char *address;

  if (! cache_info)
    /* CTR_EL0 [3:0] contains log2 of icache line size in words.
       CTR_EL0 [19:16] contains log2 of dcache line size in words.  */
    asm volatile ("mrs\t%0, ctr_el0":"=r" (cache_info));

 	dcache_lsize = 4 << ((cache_info >> 16) & 0xF);

    /* Make the start address of the loop cache aligned.  */
    address = (const char*) ((__UINTPTR_TYPE__) base
			     & ~ (__UINTPTR_TYPE__) (dcache_lsize - 1));

    for (; address < (const char *) end; address += dcache_lsize)
      asm volatile ("dc\tcvac, %0"
		    :
		    : "r" (address)
		    : "memory");

  asm volatile ("dsb\tsy" : : : "memory");
  asm volatile("isb" : : : "memory");
}

static void close_handle(int vgem_fd, uint32_t handle)
{
	struct drm_gem_close close = {
		.handle = handle,
	};

	ioctl(vgem_fd, DRM_IOCTL_GEM_CLOSE, &close);
}

static int dmabuf_heap_open(char *name)
{
	int ret, fd;
	char buf[256];

	ret = snprintf(buf, 256, "%s/%s", DEVPATH, name);
	if (ret < 0) {
		printf("snprintf failed!\n");
		return ret;
	}

	fd = open(buf, O_RDWR);
	if (fd < 0)
		printf("open %s failed!\n", buf);
	return fd;
}

static int dmabuf_heap_alloc_fdflags(int fd, size_t len, unsigned int fd_flags,
				     unsigned int heap_flags, int *dmabuf_fd)
{
	struct dma_heap_allocation_data data = {
		.len = len,
		.fd = 0,
		.fd_flags = fd_flags,
		.heap_flags = heap_flags,
	};
	int ret;

	if (!dmabuf_fd)
		return -EINVAL;

	ret = ioctl(fd, DMA_HEAP_IOCTL_ALLOC, &data);
	if (ret < 0)
		return ret;
	*dmabuf_fd = (int)data.fd;
	return ret;
}

static int dmabuf_heap_alloc(int fd, size_t len, unsigned int flags,
			     int *dmabuf_fd)
{
	return dmabuf_heap_alloc_fdflags(fd, len, O_RDWR | O_CLOEXEC, flags,
					 dmabuf_fd);
}

static void dmabuf_sync(int fd, int start_stop)
{
	struct dma_buf_sync sync = {
		.flags = start_stop | DMA_BUF_SYNC_RW,
	};
	int ret;

	ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
	if (ret)
		printf("sync failed %d\n", errno);
}

#define MEGS (1024 * 1024 * 10)

static int test_alloc_and_import(char *heap_name)
{
	int heap_fd = -1, dmabuf_fd = -1;
	char *p = NULL;
	int ret;
	int i;

	printf("Testing heap: %s\n", heap_name);

	heap_fd = dmabuf_heap_open(heap_name);
	if (heap_fd < 0)
		return -1;

	printf("Allocating %d MiB\n", MEGS/1024/1024);
	ret = dmabuf_heap_alloc(heap_fd, MEGS, 0, &dmabuf_fd);
	if (ret) {
		printf("Allocation Failed!\n");
		ret = -1;
		goto out;
	}
	else
		printf("Allocation Success!\n");

	/* mmap and write a simple pattern */

	p = mmap(NULL,
		 MEGS,
		 PROT_READ | PROT_WRITE,
		 MAP_SHARED,
		 dmabuf_fd,
		 0);
	if (p == MAP_FAILED) {
		printf("mmap() failed: %m\n");
		ret = -1;
		goto out;
	}
	printf("mmap passed p=%p\n",p);

	/*
	* if you want to sync driver cache, 
	* uncomment dmabuf_sync lines and comment the __aarch64 lines
	*/

	//dmabuf_sync(dmabuf_fd, DMA_BUF_SYNC_START); //driver cache sync start
	__aarch64_clean_dcache_range(p, p+8);
	memset(p, 0x7, 8);
	memset((char *)p + MEGS / 2, 0x8, MEGS / 2);
	//dmabuf_sync(dmabuf_fd, DMA_BUF_SYNC_END);	
	__aarch64_clean_dcache_range(p, p+8);


	/*
	* here is a read test example, after this APP has written to the buffer, 
	* we wait for another master to write some data to the buffer, 
	* then we invalidate cache to check whether the date is updated.
	*/
	usleep(100000);
	__aarch64_inval_dcache_range(p, p+8);
	for (i=0; i<8; i++)
		printf("%x ", p[i]);
	printf("\n");
	
	printf("sync passed\n");

	printf("APP: dmabuf_fd = %d, query mem blocks count\n", dmabuf_fd);

	struct dma_heap_phys_query query1 = {
			.fd = dmabuf_fd,
			.cnt = 0,
			.phys_addrs_ptr = 0
	};

	ret = ioctl(heap_fd, DMA_HEAP_GET_PHYS_ADDRS, &query1); // query cnt

	printf("APP: query.cnt = %d, query mem blocks details\n", query1.cnt);

	struct dma_heap_phys_data *phy_data = (struct dma_heap_phys_data *)
				malloc(query1.cnt * sizeof(struct dma_heap_phys_data));

	struct dma_heap_phys_query query2 = {
			.fd = dmabuf_fd,
			.cnt = query1.cnt,
			.phys_addrs_ptr = (__u64)phy_data
	};

	ret = ioctl(heap_fd, DMA_HEAP_GET_PHYS_ADDRS, &query2); // query details

	for (i = 0; i < query2.cnt; i++) {
		printf("APP: phy_data[%d].phys_start = %lx\n", i,
				(unsigned long)phy_data[i].phys_start);
		printf("APP: phy_data[%d].length = %d\n", i, phy_data[i].length);
	}

	struct dma_heap_phys_query query3 = {
			.fd = dmabuf_fd,
			.cnt = 0,
			.phys_addrs_ptr = (__u64)phy_data
	};

	printf("test an erroneous query\n");
	ret = ioctl(heap_fd, DMA_HEAP_GET_PHYS_ADDRS, &query3); // query details
	if (ret != 0)
		printf("error check passed, ret = %d\n", ret);

	ret = 0;

out:
	return ret;
}

int main(void)
{
	DIR *d;
	int ret = -1;

	d = opendir(DEVPATH);
	if (!d) {
		printf("No %s directory?\n", DEVPATH);
		return -1;
	}
	//ret = test_alloc_and_import("system");
	ret = test_alloc_and_import("coreip_pub_cma");
	//ret = test_alloc_and_import("system-uncached");
	//ret = test_alloc_and_import("reserved");
	while(1);
	closedir(d);

	return ret;
}
