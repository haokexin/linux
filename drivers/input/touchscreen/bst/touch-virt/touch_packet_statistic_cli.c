/*
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include "touch_packet_statistic_cli.h"
#include "touch_virt_bst.h"
#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
#include "touch_packet_statistic_tool.h"
#endif

/*
 * #define STATISTICS_PACKET_TIMESTAMP_TOUCH_BASEADDR(screen_inx) \
 *   (STATISTICS_PACKET_TIMESTAMP_SRAM_ADDR + (screen_inx * sizeof(struct touch_statistics_packet_cli_timestamp_t) * STATISTICS_PACKET_TIMESTAMP_GRPNUM))
 */
#define GTC_TIMESTAMP_L_REG 0x3000A098
#define GTC_TIMESTAMP_H_REG 0x3000A09C

static uint32_t pstatistics_screen_num;
static struct touch_statistics_cli_t *pstatistics;
//static struct touch_statistics_cli_t **touch_statistics = &gstatistics;
static void *gtc_timestamp_l_addr;
static void *gtc_timestamp_h_addr;

static void gtc_timestamp_init(struct device *dev)
{
    gtc_timestamp_l_addr = devm_ioremap(dev, GTC_TIMESTAMP_L_REG, 4);
    gtc_timestamp_h_addr = devm_ioremap(dev, GTC_TIMESTAMP_H_REG, 4);
    if (!gtc_timestamp_l_addr || !gtc_timestamp_h_addr)
        pr_err("Failed to remap timestamp memory!\n");
}

static void gtc_timestamp_uninit(struct device *dev)
{
    devm_iounmap(dev, gtc_timestamp_l_addr);
    devm_iounmap(dev, gtc_timestamp_h_addr);
    gtc_timestamp_l_addr = NULL;
    gtc_timestamp_h_addr = NULL;

}

static uint64_t util_get_GTC_time64(void)
{
    uint32_t high_timestamp;
    uint32_t low_timestamp;

    low_timestamp = readl(gtc_timestamp_l_addr);
    high_timestamp = readl(gtc_timestamp_h_addr);
    return (((uint64_t)high_timestamp << 32) | low_timestamp);
}

static struct touch_statistics_cli_t *statistics_packet_get_touch_statistics_by_screenid(uint32_t screen_id)
{
    int screen_inx;

    if (!pstatistics)
        return NULL;

    for (screen_inx = 0; screen_inx < pstatistics_screen_num; screen_inx++) {
        if (pstatistics[screen_inx].screen_id == screen_id) {
            return &pstatistics[screen_inx];
        }
    }

    return NULL;
}

struct touch_statistics_cli_t *statistics_packet_get_touch_statistics_cli_by_inx(int screen_inx)
{
    if (!pstatistics || screen_inx >= pstatistics_screen_num)
        return NULL;

    return &pstatistics[screen_inx];
}

#if 0
void statistics_packet_interrupt_timestamps_save(uint32_t screen_id)
{
    struct touch_statistics_cli_t *statistics = statistics_packet_get_touch_statistics_by_screenid(screen_id);

    if (!statistics)
        return;
    statistics->received_interrupt_timestamp[statistics->screen_inx] = util_get_GTC_time64();
}
#endif

void statistics_packet_loop_inc(uint32_t screen_id)
{
    struct touch_statistics_cli_t *statistics = statistics_packet_get_touch_statistics_by_screenid(screen_id);

    if (!statistics)
        return;
    if (statistics->packet_timestamp_loop >= (STATISTICS_PACKET_TIMESTAMP_GRPNUM-1))
        statistics->packet_timestamp_loop = 0;
    else
        statistics->packet_timestamp_loop++;
}
EXPORT_SYMBOL_GPL(statistics_packet_loop_inc);

void statistics_packet_timestamps_update(uint32_t screen_id, uint32_t step_inx)
{
    uint32_t step;
    struct touch_statistics_cli_t *statistics = statistics_packet_get_touch_statistics_by_screenid(screen_id);
    struct touch_statistics_packet_cli_timestamp_t *statistics_packet_timestamp;
    uint32_t loop;

    if (!statistics)
        return;

    loop = statistics->packet_timestamp_loop;
    statistics_packet_timestamp = \
        (struct touch_statistics_packet_cli_timestamp_t *)statistics->packet_timestamp_baseaddr;
    if (step_inx >= TIMESTAMP_PACKET_STEPS_NUM)
        statistics->packet_timestamp_step = 0;
    else
        statistics->packet_timestamp_step = step_inx;

    step = statistics->packet_timestamp_step;
    statistics_packet_timestamp[loop].timestamp[step] = util_get_GTC_time64();
}
EXPORT_SYMBOL_GPL(statistics_packet_timestamps_update);

#if 0
void statistics_packet_timestamps_update_by_value(uint32_t screen_id,  uint32_t step_inx, uint64_t timestamp)
{
    uint32_t step;
    struct touch_statistics_cli_t *statistics = statistics_packet_get_touch_statistics_by_screenid(screen_id);
    struct touch_statistics_packet_cli_timestamp_t *statistics_packet_timestamp;
    uint32_t loop;

    if (!statistics)
        return;

    loop = statistics->packet_timestamp_loop;
    statistics_packet_timestamp = \
        (struct touch_statistics_packet_cli_timestamp_t *)statistics->packet_timestamp_baseaddr;
    if (step_inx >= TIMESTAMP_PACKET_STEPS_NUM)
        statistics->packet_timestamp_step = 0;
    else
        statistics->packet_timestamp_step = step_inx;

    step = statistics->packet_timestamp_step;
    statistics_packet_timestamp[loop].timestamp[step] = timestamp;
}

void statistics_packet_interrupt_timestamps_update(uint32_t screen_id,  uint32_t step_inx)
{
    uint32_t step;
    struct touch_statistics_cli_t *statistics = statistics_packet_get_touch_statistics_by_screenid(screen_id);
    struct touch_statistics_packet_cli_timestamp_t *statistics_packet_timestamp;
    uint32_t loop;

    if (!statistics)
        return;

    loop = statistics->packet_timestamp_loop;
    statistics_packet_timestamp = \
        (struct touch_statistics_packet_cli_timestamp_t *)statistics->packet_timestamp_baseaddr;

    if (step_inx >= TIMESTAMP_PACKET_STEPS_NUM)
        statistics->packet_timestamp_step = 0;
    else
        statistics->packet_timestamp_step = step_inx;

    step = statistics->packet_timestamp_step;
    statistics_packet_timestamp[loop].timestamp[step] = statistics->received_interrupt_timestamp[statistics->screen_inx];
}
#endif

void statistics_packet_data_update(uint32_t screen_id, uint32_t id, uint8_t *data, uint32_t data_len)
{
    struct touch_statistics_cli_t *statistics = statistics_packet_get_touch_statistics_by_screenid(screen_id);
    struct touch_statistics_packet_cli_timestamp_t *statistics_packet_timestamp;
    uint32_t loop;

    if (!statistics)
        return;

    loop = statistics->packet_timestamp_loop;
    statistics_packet_timestamp = \
        (struct touch_statistics_packet_cli_timestamp_t *)statistics->packet_timestamp_baseaddr;

    statistics_packet_timestamp[loop].id = id;
    if (data_len > STATISTICS_PACKET_DATA_SIZE)
        data_len = STATISTICS_PACKET_DATA_SIZE;
    if (data && data_len)
        memcpy(statistics_packet_timestamp[loop].data, data, data_len);
}
EXPORT_SYMBOL_GPL(statistics_packet_data_update);

void statistics_packet_timestamp_init(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num)
{
    int i, j;
    hw_info_t* hwinfo;
    dma_addr_t dma_paddr;
    struct bst_ts_data *ts;
    struct bst_ts_data **ts_data = (struct bst_ts_data **) pdata;
    struct touch_statistics_packet_cli_timestamp_t *statistics_packet_timestamp;

    if (!ts_data || !*ts_data || !dev) 
        return;

    pstatistics = devm_kzalloc(dev, sizeof(struct touch_statistics_cli_t) * request_screen_num, GFP_KERNEL);
    if (!pstatistics) {
        pr_err("Failed to allocate memory for touch statistics\n");
        return;
    }
    pstatistics_screen_num = request_screen_num;

    gtc_timestamp_init(dev);
    // Initialize the statistics information for each screen
    for (i = 0; i < request_screen_num; i++) {
        //memset(&pstatistics[i], 0, sizeof(struct touch_statistics_cli_t));

        ts = ts_data[i];
        if (!ts)
            continue;
        hwinfo = &ts->hwinfo;

        pstatistics[i].screen_inx = i;
        pstatistics[i].screen_id = hwinfo->screen_id;
        pstatistics[i].packet_timestamp_loop = STATISTICS_PACKET_TIMESTAMP_GRPNUM;
	    pstatistics[i].packet_timestamp_memsize = \
            (sizeof(struct touch_statistics_packet_cli_timestamp_t) * STATISTICS_PACKET_TIMESTAMP_GRPNUM);

        //dma_set_mask(&ts->input_dev->dev, DMA_BIT_MASK(36));
        dma_set_coherent_mask(&ts->input_dev->dev, DMA_BIT_MASK(36));
        /* Allocate memory */
        pstatistics[i].packet_timestamp_baseaddr = dma_alloc_coherent(&ts->input_dev->dev, pstatistics[i].packet_timestamp_memsize,
		&dma_paddr, GFP_KERNEL);

        if (!pstatistics[i].packet_timestamp_baseaddr) {
            pr_err("Failed to allocate DMA memory (size: %zu)\n", (size_t) pstatistics[i].packet_timestamp_memsize);
            devm_kfree(dev, pstatistics);
            pstatistics = NULL;
            gtc_timestamp_uninit(dev);
            return;
        }
        pstatistics[i].packet_timestamp_phyaddr = dma_to_phys(&ts->input_dev->dev, dma_paddr);
        pr_info("Screen (%d-0x%x) allocated DMA memory ((vaddr: 0x%0llX, paddr: 0x%0llX size: %zu))\n", 
            i, pstatistics[i].screen_id, (uint64_t) pstatistics[i].packet_timestamp_baseaddr,
            pstatistics[i].packet_timestamp_phyaddr, (size_t) pstatistics[i].packet_timestamp_memsize);

        statistics_packet_timestamp = (struct touch_statistics_packet_cli_timestamp_t *)pstatistics[i].packet_timestamp_baseaddr;
        for (j = 0; j < STATISTICS_PACKET_TIMESTAMP_GRPNUM; j++) {
            memset(&statistics_packet_timestamp[j], 0, sizeof(struct touch_statistics_packet_cli_timestamp_t));
        }
    }

#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
    statistics_packet_timestamp_tool_init(dev, client_id, pdata, request_screen_num);
#endif

    pr_info("Touch statistics packet timestamp initialized\n");
}
EXPORT_SYMBOL_GPL(statistics_packet_timestamp_init);

void statistics_packet_timestamp_uninit(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num)
{
    int i;
    struct bst_ts_data **ts_data = (struct bst_ts_data **) pdata;
    struct touch_statistics_cli_t *statistics;

    if (!ts_data || !*ts_data || !dev)
        return;

#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
    statistics_packet_timestamp_tool_uninit(dev, client_id, pdata, request_screen_num);
#endif

    for (i = 0; i < pstatistics_screen_num; i++) {
        statistics = &pstatistics[i];
        if (!statistics->packet_timestamp_baseaddr)
            continue;
        dma_free_coherent(&ts_data[i]->input_dev->dev, statistics->packet_timestamp_memsize,
            statistics->packet_timestamp_baseaddr, statistics->packet_timestamp_phyaddr);
        pr_info("Screen (%d-0x%x) released DMA memory ((vaddr: 0x%0llX, paddr: 0x%0llX size: %zu))\n",
            i, statistics->screen_id, (uint64_t) statistics->packet_timestamp_baseaddr,
            statistics->packet_timestamp_phyaddr, (size_t) statistics->packet_timestamp_memsize);
        statistics->packet_timestamp_baseaddr = NULL;
    }

    devm_kfree(dev, pstatistics);
    pstatistics = NULL;
    gtc_timestamp_uninit(dev);

    pr_info("Touch statistics packet timestamp uninitialized\n");
}
EXPORT_SYMBOL_GPL(statistics_packet_timestamp_uninit);

MODULE_AUTHOR("Pengcheng Xue");
MODULE_DESCRIPTION("BST virtual Touchscreen Statistics Packet Timestamp Driver");
MODULE_LICENSE("GPL v2");
