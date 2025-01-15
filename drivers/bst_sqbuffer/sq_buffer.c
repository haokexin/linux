// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "sq_buffer.h"

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "ipc_hw_sem.h"
#include "mcu_id.h"

#define SQ_BUFFER_BASE_ADDR (0x4876200)
#define SQ_BUFFER_SEM_ID (15U)

sq_buffer_proxy_t *g_sq_buffer_proxy;

static void __iomem *ipc_sem_base;

static int map_phy_addr(uint64_t phy_addr, uint64_t length,
                        void __iomem **virtual_addr)
{
    uint64_t page_size = PAGE_SIZE;
    uint64_t base_addr = phy_addr & ~(page_size - 1);
    uint64_t offset = phy_addr - base_addr;

    void __iomem *pmem = ioremap(base_addr, length + offset);
    if (!pmem)
        return -ENOMEM;

    *virtual_addr = pmem + offset;
    return 0;
}

static void __aarch64_inval_dcache_range(const void *base, const void *end)
{
    unsigned dcache_lsize;
    static unsigned int cache_info = 0;
    const char *address;

    if (!cache_info)
        /* CTR_EL0 [3:0] contains log2 of icache line size in words.
           CTR_EL0 [19:16] contains log2 of dcache line size in words.  */
        asm volatile("mrs\t%0, ctr_el0" : "=r"(cache_info));

    dcache_lsize = 4 << ((cache_info >> 16) & 0xF);

    /* Make the start address of the loop cache aligned.  */
    address = (const char *)((__UINTPTR_TYPE__)base &
                             ~(__UINTPTR_TYPE__)(dcache_lsize - 1));

    for (; address < (const char *)end; address += dcache_lsize)
        asm volatile("dc\tcivac, %0" : : "r"(address) : "memory");

    asm volatile("dsb\tsy" : : : "memory");
    asm volatile("isb" : : : "memory");
}

static void __aarch64_clean_dcache_range(const void *base, const void *end)
{
    unsigned dcache_lsize;
    static unsigned int cache_info = 0;
    const char *address;

    if (!cache_info)
        /* CTR_EL0 [3:0] contains log2 of icache line size in words.
           CTR_EL0 [19:16] contains log2 of dcache line size in words.  */
        asm volatile("mrs\t%0, ctr_el0" : "=r"(cache_info));

    dcache_lsize = 4 << ((cache_info >> 16) & 0xF);

    /* Make the start address of the loop cache aligned.  */
    address = (const char *)((__UINTPTR_TYPE__)base &
                             ~(__UINTPTR_TYPE__)(dcache_lsize - 1));

    for (; address < (const char *)end; address += dcache_lsize)
        asm volatile("dc\tcvac, %0" : : "r"(address) : "memory");

    asm volatile("dsb\tsy" : : : "memory");
    asm volatile("isb" : : : "memory");
}

static int is_empty(data_SQ *sq)
{
    return sq->front == sq->rear;
}

static int sq_buffer_proxy_init(SQ_Buffer *sq)
{
    sq_buffer_proxy_t *proxy;
    uint64_t buffer_phy_addr;
    void __iomem *buffer_ptr;
    SQE *sqe;
    sqe_proxy_t *sqe_proxy;

    proxy = kzalloc(sizeof(sq_buffer_proxy_t), GFP_KERNEL);
    if (proxy == NULL) {
        return -1;
    }
    g_sq_buffer_proxy = proxy;
    proxy->sq = sq;
    for (int i = 0; i < sq->buffer_num; i++) {
        sqe = &sq->free_sq.sqe[i];
        sqe_proxy = &proxy->sqe[i];
        sqe_proxy->ref_cnt = &sqe->ref_cnt;
        sqe_proxy->index = &sqe->index;
        buffer_phy_addr = sq->buffers_phy_addr64[sqe->index];
        buffer_ptr = (void __iomem **)&sqe_proxy->buffer;
        if (map_phy_addr(buffer_phy_addr, sq->buffer_size, buffer_ptr) != 0) {
            printk("map_phy_addr failed for buffer 0x%llx\n", buffer_phy_addr);
            return -1;
        }
    }
    return 0;
}

static int sq_buffer_consume_init(uint64_t phy_addr64)
{
    SQ_Buffer *sq = NULL;
    if (map_phy_addr(phy_addr64, sizeof(SQ_Buffer), (void __iomem **)&sq) !=
        0) {
        printk("map_phy_addr for SQ_Buffer failed\n");
        return -1;
    }
    if (sq_buffer_proxy_init(sq) != 0) {
        printk("sq_buffer_proxy_init failed\n");
        return -1;
    }
    return 0;
}

sqe_proxy_t *sq_buffer_consume_get(uint8_t consumer_id)
{
    data_SQ *data_sq;
    int index;
    sqe_proxy_t *sqe;

    if (consumer_id >= SQ_BUFFER_CONSUMER_MAX) {
        return NULL;
    }
    data_sq = &g_sq_buffer_proxy->sq->data_sq[consumer_id];
    __aarch64_inval_dcache_range((char *)data_sq,
                                 (char *)data_sq + sizeof(data_SQ));
    if (is_empty(data_sq)) {
        return NULL;
    }
    index = data_sq->sqe_ptr[data_sq->front];
    sqe = &g_sq_buffer_proxy->sqe[index];
    ++(data_sq->front);
    if (data_sq->front >= (g_sq_buffer_proxy->sq->buffer_num + 1)) {
        data_sq->front = 0;
    }
    __aarch64_clean_dcache_range((char *)&data_sq->front,
                                 (char *)&data_sq->front + sizeof(int32_t));
    __aarch64_inval_dcache_range((char *)sqe->buffer,
                                 (char *)sqe->buffer +
                                     g_sq_buffer_proxy->sq->buffer_size);
    return sqe;
}
EXPORT_SYMBOL(sq_buffer_consume_get);

int sq_buffer_consume_put(sqe_proxy_t *sqe)
{
    get_sq_sem_lock(ipc_sem_base, SQ_BUFFER_SEM_ID);
    __aarch64_inval_dcache_range((char *)sqe, (char *)sqe + sizeof(SQE));
    (*sqe->ref_cnt)--;
    __aarch64_clean_dcache_range((char *)sqe, (char *)sqe + sizeof(SQE));
    release_sq_sem_lock(ipc_sem_base, SQ_BUFFER_SEM_ID);
    return 0;
}
EXPORT_SYMBOL(sq_buffer_consume_put);

static ssize_t sq_show(struct kobject *kobj, struct kobj_attribute *attr,
                       char *buf)
{
    int len = 0;
    len += sysfs_emit_at(buf, len, "SQ_Buffer size: %ld\n", sizeof(SQ_Buffer));
    len += sysfs_emit_at(buf, len, "SQ_Buffer buffer_num: %d\n",
                         g_sq_buffer_proxy->sq->buffer_num);
    len += sysfs_emit_at(buf, len, "SQ_Buffer buffer_size: %d\n",
                         g_sq_buffer_proxy->sq->buffer_size);
    for (int i = 0; i < g_sq_buffer_proxy->sq->buffer_num; i++) {
        len +=
            sysfs_emit_at(buf, len, "SQ_Buffer buffer[%d] phy addr: 0x%llx\n",
                          i, g_sq_buffer_proxy->sq->buffers_phy_addr64[i]);
    }
    return len;
}

static ssize_t sq_store(struct kobject *kobj, struct kobj_attribute *attr,
                        const char *buf, size_t count)
{
    const char *ptr = buf;
    char *end = NULL;
    int len = 0;
    int arg = 0;
    sqe_proxy_t *sqe = NULL;

    len = strlen(buf);
    arg = simple_strtol(ptr, &end, 10);
    if (arg <= 0) {
        pr_err("arg is invalid\n");
        return -EINVAL;
    }

    for (int i = 0; i < arg; i++) {
        msleep(1000);
        sqe = sq_buffer_consume_get(MCU_ADAS);
        if (sqe == NULL) {
            pr_err("sq_buffer_consume_get[%d] is NULL\n", i);
            continue;
        }
        sq_buffer_consume_put(sqe);
        printk(
            "sq_buffer_consume_get[%d] succeess, buffer phy addr is 0x%llx\n",
            i, g_sq_buffer_proxy->sq->buffers_phy_addr64[*sqe->index]);
    }
    return count;
}

static struct kobj_attribute sq_buffer_attr = __ATTR_RW(sq);

static struct kobject *sq_buffer_kobj;

static int __init sq_buffer_probe_setup_init(void)
{
    int err;
    if (sq_buffer_consume_init(SQ_BUFFER_BASE_ADDR) != 0) {
        return -1;
    }
    ipc_sem_base = ioremap(IPC_SEM_BASE_ADDR, IPC_SEM_SIZE);
    if (ipc_sem_base == NULL) {
        printk("ioremap for ipc_sem_base failed\n");
        return -1;
    }
    sq_buffer_kobj = kobject_create_and_add("sq_buffer", kernel_kobj);
    if (!sq_buffer_kobj)
        return -ENOMEM;

    err = sysfs_create_file(sq_buffer_kobj, &sq_buffer_attr.attr);
    if (err) {
        kobject_put(sq_buffer_kobj);
        return err;
    }

    return 0;
}

static void __exit sq_buffer_probe_setup_exit(void)
{
    kobject_put(sq_buffer_kobj);
    for (int i = 0; i < g_sq_buffer_proxy->sq->buffer_num; i++) {
        iounmap(g_sq_buffer_proxy->sqe[i].buffer);
    }
    iounmap(g_sq_buffer_proxy->sq);
    iounmap(ipc_sem_base);
    kfree(g_sq_buffer_proxy);
}

late_initcall(sq_buffer_probe_setup_init);
module_exit(sq_buffer_probe_setup_exit);
MODULE_LICENSE("GPL");
