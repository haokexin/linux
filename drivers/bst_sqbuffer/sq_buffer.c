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
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of_reserved_mem.h>

#include "ipc_hw_sem.h"
#include "mcu_id.h"

//#define SQ_BUFFER_FLUSH_CACHE
#define SQ_BUFFER_SEM_ID (15U)
#define SQ_BUFFER_MAGIC (0x12345678)
#define TOTAL_SIZE (64 * 1024) // 64KB

//Apply for 64KB memory, the first 32KB is used for adas2can_sq_buffer, and the last 32KB is used for can2adas_sq_buffer
#define CAN2ADAS_SQ_BUFFER_OFFSET (32*1024)
struct sq_buffer_info {
    struct device *dev;
    void __iomem *ipc_sem_base;
    struct kobject *adas2can_sq_buffer_kobj;
    sq_buffer_proxy_t *adas2can_sq_buffer_proxy;
    SQ_Buffer *adas2can_sq;
    struct kobject *can2adas_sq_buffer_kobj;
    sq_buffer_proxy_t *can2adas_sq_buffer_proxy;
    SQ_Buffer *can2adas_sq;
    uint64_t sq_buffer_base_addr;
    void* sq_buffer_base_addr_virtual;
};

static struct sq_buffer_info *g_info;

static void __iomem *ipc_sem_base;

#ifdef SQ_BUFFER_FLUSH_CACHE
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
#else
static void __aarch64_inval_dcache_range(const void *base, const void *end) {}
static void __aarch64_clean_dcache_range(const void *base, const void *end) {}
#endif

static int is_empty(data_SQ *sq)
{
    return sq->front == sq->rear;
}

int SQ_Buffer_Init(SQ_Buffer *sq, uint32_t *buffer, uint32_t buffer_size,
                   uint32_t buffer_num)
{
    free_SQ *free_sq = NULL;

    if (unlikely(buffer_num >= SQ_BUFFER_NUM_MAX)) {
        return -1;
    }

    sq->buffer_size = buffer_size;
    sq->buffer_num = buffer_num;
    free_sq = &sq->free_sq;

    for (int i = 0; i < buffer_num; ++i) {
        free_sq->sqe[i].ref_cnt = 0;
        free_sq->sqe[i].buffer = buffer[i];
        free_sq->sqe[i].index = i;
        // printk("buffer :%d, ref_cnt: %d, index: %d, addr: 0x%x\n", i, free_sq->sqe[i].ref_cnt, 
        //     free_sq->sqe[i].index, free_sq->sqe[i].buffer);
    }
    for (int i = 0; i < SQ_BUFFER_CONSUMER_MAX; ++i) {
        data_SQ *data_sq = &sq->data_sq[i];
        data_sq->front = 0;
        data_sq->rear = 0;
    }
    sq->magic = SQ_BUFFER_MAGIC;
    __aarch64_clean_dcache_range((char *)sq,
                                 (char *)sq + sizeof(SQ_Buffer));
    return 0;
}

static int sq_buffer_proxy_init(SQ_Buffer *sq, sq_buffer_proxy_t **sq_buffer_proxy)
{
    sq_buffer_proxy_t *proxy;
    uint64_t buffer_phy_addr;
    uint64_t offset;
    SQE *sqe;
    sqe_proxy_t *sqe_proxy;

    if (sq == NULL) {
        dev_err(g_info->dev, "sq is NULL\n");
        return -EINVAL;
    }
    __aarch64_inval_dcache_range((char *)sq, (char *)sq + sizeof(SQ_Buffer));
    if (sq->magic != SQ_BUFFER_MAGIC) {
        dev_err(g_info->dev, "sq->magic is invalid\n");
        return -EINVAL;
    }
    proxy = devm_kzalloc(g_info->dev, sizeof(sq_buffer_proxy_t), GFP_KERNEL);
    if (proxy == NULL) {
        dev_err(g_info->dev, "Failed to allocate proxy\n");
        return -ENOMEM;
    }
    *sq_buffer_proxy = proxy;
    for (int i = 0; i < sq->buffer_num; i++) {
        sqe = &sq->free_sq.sqe[i];
        sqe_proxy = &proxy->sqe[i];
        sqe_proxy->ref_cnt = &sqe->ref_cnt;
        sqe_proxy->index = &sqe->index;
        buffer_phy_addr = sq->buffers_phy_addr64[sqe->index];
        offset = buffer_phy_addr - g_info->sq_buffer_base_addr;
        sqe_proxy->buffer = (void __iomem *)((char *)g_info->sq_buffer_base_addr_virtual + offset);
        // printk("debug set sqe_proxy[%d]:0x%llx buffer:0x%llx by sqe:0x%llx buffer_phy_addr:0x%llx g_info->sq_buffer_base_addr:0x%llx\n",
        //         i, (uint64_t)sqe_proxy,
        //         (uint64_t)sqe_proxy->buffer,
        //         (uint64_t)sqe,
        //         buffer_phy_addr,
        //         g_info->sq_buffer_base_addr);
    }
    return 0;
}

sqe_proxy_t *sq_buffer_consume_get(uint8_t consumer_id)
{
    data_SQ *data_sq;
    int index;
    sqe_proxy_t *sqe;

    if (!g_info || consumer_id >= SQ_BUFFER_CONSUMER_MAX) {
        return NULL;
    }

    if (g_info->can2adas_sq_buffer_proxy == NULL) {
        if (sq_buffer_proxy_init(g_info->can2adas_sq, &g_info->can2adas_sq_buffer_proxy) != 0) {
            dev_err(g_info->dev, "sq_buffer_proxy_init failed\n");
            return NULL;
        }
    }

    data_sq = &g_info->can2adas_sq->data_sq[consumer_id];
    __aarch64_inval_dcache_range((char *)data_sq,
                                 (char *)data_sq + sizeof(data_SQ));
    if (is_empty(data_sq)) {
        return NULL;
    }
    index = data_sq->sqe_ptr[data_sq->front];
    sqe = &g_info->can2adas_sq_buffer_proxy->sqe[index];
    ++(data_sq->front);
    if (data_sq->front >= (g_info->can2adas_sq->buffer_num + 1)) {
        data_sq->front = 0;
    }
    __aarch64_clean_dcache_range((char *)&data_sq->front,
                                 (char *)&data_sq->front + sizeof(int32_t));
    __aarch64_inval_dcache_range((char *)sqe->buffer,
                                 (char *)sqe->buffer +
                                     g_info->can2adas_sq->buffer_size);
    return sqe;
}
EXPORT_SYMBOL(sq_buffer_consume_get);

int sq_buffer_consume_datanum(uint8_t consumer_id)
{
    data_SQ *data_sq;

    if (!g_info || consumer_id >= SQ_BUFFER_CONSUMER_MAX) {
        return 0;
    }

    if (g_info->can2adas_sq_buffer_proxy == NULL) {
        if (sq_buffer_proxy_init(g_info->can2adas_sq, &g_info->can2adas_sq_buffer_proxy) != 0) {
            dev_err(g_info->dev, "sq_buffer_proxy_init failed\n");
            return 0;
        }
    }

    data_sq = &g_info->can2adas_sq->data_sq[consumer_id];
    __aarch64_inval_dcache_range((char *)data_sq,
                                 (char *)data_sq + sizeof(data_SQ));

    return (data_sq->rear - data_sq->front + g_info->can2adas_sq->buffer_num)%(g_info->can2adas_sq->buffer_num);
}
EXPORT_SYMBOL(sq_buffer_consume_datanum);

int sq_buffer_consume_put(sqe_proxy_t *sqe)
{
    if (!g_info)
        return -EINVAL;

    if (g_info->can2adas_sq_buffer_proxy == NULL) {
        if (sq_buffer_proxy_init(g_info->can2adas_sq, &g_info->can2adas_sq_buffer_proxy) != 0) {
            dev_err(g_info->dev, "sq_buffer_proxy_init failed\n");
            return -EINVAL;
        }
    }

    get_sq_sem_lock(g_info->ipc_sem_base, SQ_BUFFER_SEM_ID);
    __aarch64_inval_dcache_range((char *)sqe, (char *)sqe + sizeof(SQE));
    (*sqe->ref_cnt)--;
    __aarch64_clean_dcache_range((char *)sqe, (char *)sqe + sizeof(SQE));
    release_sq_sem_lock(g_info->ipc_sem_base, SQ_BUFFER_SEM_ID);
    return 0;
}
EXPORT_SYMBOL(sq_buffer_consume_put);

uint64_t sq_buffer_get_base_paddr(void)
{
    if (!g_info)
        return 0;

    return g_info->sq_buffer_base_addr;
}
EXPORT_SYMBOL(sq_buffer_get_base_paddr);

uint64_t sq_buffer_get_base_vaddr(void)
{
    if (!g_info)
        return 0;

    return (uint64_t)g_info->sq_buffer_base_addr_virtual;
}

static ssize_t sq_show(struct kobject *kobj, struct kobj_attribute *attr,
                       char *buf)
{
    int len = 0;
    len += sysfs_emit_at(buf, len, "SQ_Buffer size: %ld\n", sizeof(SQ_Buffer));
    len += sysfs_emit_at(buf, len, "SQ_Buffer adas2can buffer_num: %d can2adas buffer_num:%d\n",
                         g_info->adas2can_sq->buffer_num, g_info->can2adas_sq->buffer_num);
    len += sysfs_emit_at(buf, len, "SQ_Buffer adas2can buffer_size: %d can2adas buffer_size: %d \n",
                         g_info->adas2can_sq->buffer_size, g_info->can2adas_sq->buffer_size);
    for (int i = 0; i < g_info->adas2can_sq->buffer_num; i++) {
        len +=
            sysfs_emit_at(buf, len, "SQ_Buffer adas2can_sq buffer[%d] phy addr: 0x%llx\n",
                          i, g_info->adas2can_sq->buffers_phy_addr64[i]);
    }
    for (int i = 0; i < g_info->can2adas_sq->buffer_num; i++) {
        len +=
            sysfs_emit_at(buf, len, "SQ_Buffer can2adas_sq buffer[%d] phy addr: 0x%llx\n",
                          i, g_info->can2adas_sq->buffers_phy_addr64[i]);
    }

    len += sysfs_emit_at(buf, len, "adas2can data_sq[0] front:rear %d:%d, can2adas %d:%d\n",
                    g_info->adas2can_sq->data_sq[0].front, g_info->adas2can_sq->data_sq[0].rear,
                    g_info->can2adas_sq->data_sq[0].front, g_info->can2adas_sq->data_sq[0].rear);
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
        dev_err(g_info->dev, "arg is invalid\n");
        return -EINVAL;
    }

    for (int i = 0; i < arg; i++) {
        msleep(1000);
        sqe = sq_buffer_consume_get(MCU_ADAS);
        if (sqe == NULL) {
            dev_err(g_info->dev, "sq_buffer_consume_get[%d] is NULL\n", i);
            continue;
        }
        sq_buffer_consume_put(sqe);
        dev_dbg(g_info->dev, "sq_buffer_consume_get[%d] success, buffer phy addr is 0x%llx\n",
                i, g_info->can2adas_sq->buffers_phy_addr64[*sqe->index]);
    }
    return count;
}

static struct kobj_attribute adas2can_sq_buffer_attr = __ATTR_RW(sq);
static struct kobj_attribute can2adas_sq_buffer_attr = __ATTR_RW(sq);

static int sq_buffer_probe(struct platform_device *pdev)
{
    struct sq_buffer_info *info;
    int err;

	err = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(&pdev->dev, "Could not set consistent DMA mask: %d\n", err);
		return err;
	}

    if (platform_get_drvdata(pdev)) {
        dev_warn(&pdev->dev, "sq_buffer already probed\n");
        return -EEXIST;
    }

    info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
    if (!info)
        return -ENOMEM;

    info->dev = &pdev->dev;
    g_info = info;
    platform_set_drvdata(pdev, info);


    //Apply for 64KB memory, the first 32KB is used for adas2can_sq_buffer, and the last 32KB is used for can2adas_sq_buffer
    info->sq_buffer_base_addr_virtual = dma_alloc_coherent(info->dev, TOTAL_SIZE,
		&info->sq_buffer_base_addr, GFP_KERNEL);
    if (info->sq_buffer_base_addr_virtual == NULL) {
        dev_err(info->dev, "dma_alloc_coherent for SQ_Buffer failed\n");
        return -ENOMEM;
    }
    info->adas2can_sq = info->sq_buffer_base_addr_virtual;
    info->can2adas_sq = info->sq_buffer_base_addr_virtual + CAN2ADAS_SQ_BUFFER_OFFSET;

    info->ipc_sem_base = devm_ioremap(info->dev, IPC_SEM_BASE_ADDR, IPC_SEM_SIZE);
    if (!info->ipc_sem_base) {
        dev_err(info->dev, "ioremap for ipc_sem_base failed\n");
        return -ENOMEM;
    }
    ipc_sem_base = info->ipc_sem_base;

    info->adas2can_sq_buffer_kobj = kobject_create_and_add("adas2can_sq_buffer_kobj", kernel_kobj);
    if (!info->adas2can_sq_buffer_kobj)
        return -ENOMEM;

    err = sysfs_create_file(info->adas2can_sq_buffer_kobj, &adas2can_sq_buffer_attr.attr);
    if (err) {
        kobject_put(info->adas2can_sq_buffer_kobj);
        return err;
    }
    info->can2adas_sq_buffer_kobj = kobject_create_and_add("can2adas_sq_buffer_kobj", kernel_kobj);
    if (!info->can2adas_sq_buffer_kobj)
        return -ENOMEM;

    err = sysfs_create_file(info->can2adas_sq_buffer_kobj, &can2adas_sq_buffer_attr.attr);
    if (err) {
        kobject_put(info->can2adas_sq_buffer_kobj);
        return err;
    }

    dev_info(info->dev, "sq_buffer probe success\n");
    return 0;
}

static int sq_buffer_remove(struct platform_device *pdev)
{
    struct sq_buffer_info *info = platform_get_drvdata(pdev);

    if (!info)
        return -EINVAL;

    release_sq_sem_lock(ipc_sem_base, SQ_BUFFER_SEM_ID);

    kobject_put(info->adas2can_sq_buffer_kobj);
    kobject_put(info->can2adas_sq_buffer_kobj);

    if (info->sq_buffer_base_addr_virtual) {
        dma_free_coherent(info->dev, sizeof(SQ_Buffer), info->sq_buffer_base_addr_virtual,
                          info->sq_buffer_base_addr);
    }

    return 0;
}

sqe_proxy_t *sq_buffer_produce_get(void)
{
    sq_buffer_proxy_t *proxy;
    SQ_Buffer *sq;
    sqe_proxy_t *sqe = NULL;
    if (!g_info)
        return NULL;

    if (g_info->adas2can_sq_buffer_proxy == NULL) {
        if (sq_buffer_proxy_init(g_info->adas2can_sq, &g_info->adas2can_sq_buffer_proxy) != 0) {
            dev_err(g_info->dev, "adas2_can_sq_buffer_proxy_init failed\n");
            return NULL;
        }
    }


    proxy = g_info->adas2can_sq_buffer_proxy;
    sq = g_info->adas2can_sq;
    __aarch64_inval_dcache_range((char *)proxy,
                                 (char *)proxy + sizeof(sq_buffer_proxy_t));

    // printk("debug adas2can_sq.buffer_num:%d\n", g_info->adas2can_sq->buffer_num);
    for (int i = 0; i < sq->buffer_num; i++) {
        // printk("debug sq_buffer_produce_get sqe[%d]:0x%llx \n",
        //     i, (uint64_t)(&proxy->sqe[i]));
        // printk("debug sq_buffer_produce_get proxy:0x%llx sqe[%d]:0x%llx ref_cnt:%d\n",
        //     (uint64_t)proxy, i, (uint64_t)(&proxy->sqe[i]), *proxy->sqe[i].ref_cnt);
        if (*proxy->sqe[i].ref_cnt == 0) {
            sqe = &proxy->sqe[i];
            break;
        }
    }

    return sqe;
}


int sq_buffer_produce_put(sqe_proxy_t *sqe,
                          uint16_t consumer_mask)
{
    SQ_Buffer *sq = g_info->adas2can_sq;
    get_sq_sem_lock(ipc_sem_base, SQ_BUFFER_SEM_ID);

    if ((consumer_mask & 0xFFFF) == 0) {
        release_sq_sem_lock(ipc_sem_base, SQ_BUFFER_SEM_ID);
        return -1;
    }
    (*sqe->ref_cnt)++;

    __aarch64_clean_dcache_range((char *)sqe->buffer,
                                 (char *)sqe->buffer + sq->buffer_size);
    __aarch64_clean_dcache_range((char *)sqe->ref_cnt,
                                 (char *)sqe->ref_cnt + sizeof(int));
    for (uint16_t mask = consumer_mask; mask != 0; mask &= mask - 1) {
        int index = __builtin_ctz(mask);
        data_SQ *data_sq = &sq->data_sq[index];
        data_sq->sqe_ptr[data_sq->rear] = *(sqe->index);
        mb();
        if (data_sq->rear == sq->buffer_num) {
            data_sq->rear = 0;
        } else {
            ++data_sq->rear;
        }
        __aarch64_clean_dcache_range((char *)data_sq,
                                     (char *)data_sq + sizeof(data_SQ));
    }
    release_sq_sem_lock(ipc_sem_base, SQ_BUFFER_SEM_ID);
    return 0;
}



static const struct of_device_id sqbuffer_of_match[] = {
    { .compatible = "bst,sqbuffer" },
    { }
};
MODULE_DEVICE_TABLE(of, sqbuffer_of_match);

static struct platform_driver sq_buffer_driver = {
    .probe = sq_buffer_probe,
    .remove = sq_buffer_remove,
    .driver = {
        .name = "sq-buffer",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(sqbuffer_of_match),
    },
};

module_platform_driver(sq_buffer_driver);
MODULE_LICENSE("GPL");
