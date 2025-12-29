/*
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif /* CONFIG_DEBUG_FS */
#include "touch_packet_statistic_cli.h"
#include "touch_virt_bst.h"

#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE

/* define in server */
/* The address of the touch server statistics packet timestamp */
#define STATISTICS_PACKET_SRV_STATINFO_RAM_ADDR 0x806790000 // bit32: 0x86791000 Size: 4k
#define STATISTICS_PACKET_SRV_TIMESTAMP_RAM_ADDR 0x806791000UL // bit32: 0x86791000
#define STATISTICS_PACKET_SRV_TIMESTAMP_RAM_ADDR_END  0x80688B000 // bit32: 0x8688B000 MAX: 0x8691FFFF
#define STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM 1024
#define TIMESTAMP_PACKET_SRV_NUM 8
#define STATISTICS_PACKET_SRV_DATA_SIZE 4
#define MAX_CLIENT_NUM 5

#define STATISTICS_PACKET_SRV_TIMESTAMP_TOUCH_BASEADDR(screen_inx) \
    (STATISTICS_PACKET_SRV_TIMESTAMP_RAM_ADDR + (screen_inx * sizeof(struct touch_statistics_packet_srv_timestamp_t) * STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM))

#define STR(x) #x
#define SRV_COORD_TYPE(step) TIMESTAMP_PACKET_SRV_COORD_##step
#define CLI_COORD_TYPE(step) TIMESTAMP_PACKET_CLI_COORD_##step

#define SRV_COORD_STR(step) STR(SRV_##step)
#define CLI_COORD_STR(step) STR(CLI_##step)

enum statistics_packet_srv_coord_pointinfo_timestamp_step {
    SRV_COORD_TYPE(RECEIVED_INTERRUPT) = 0, /* interrupt received */
    SRV_COORD_TYPE(DRV_PROCESSING_START), /* touch driver processing start */
    SRV_COORD_TYPE(DRV_PROCESSING_END), /* touch driver processing end */
    SRV_COORD_TYPE(QUEUING_MSGBOX_TO_SEND), /* Queuing for msgbox to send start, or touch manager processing delay time */
    SRV_COORD_TYPE(SEND_MSGBOX_TO_CLIENT), /* delay for touch manager, send msgbox start */
    SRV_COORD_TYPE(MAX)
};

/* def in client */
enum statistics_packet_cli_coord_pointinfo_timestamp_step {
    CLI_COORD_TYPE(RECV_MSGBOX_FROM_SRV) = 0, /* msgbox received from touch server */
    CLI_COORD_TYPE(SEND_EVENT_TO_INPUT_SYNC), /* delay for input sync, send event start */
    CLI_COORD_TYPE(MAX),
};

#define TIMESTAMP_PACKET_COORD_MAX (SRV_COORD_TYPE(MAX) + CLI_COORD_TYPE(MAX))

static char *statistics_packet_coord_step_str[TIMESTAMP_PACKET_COORD_MAX] = {
    SRV_COORD_STR(RECEIVED_INTERRUPT),
    SRV_COORD_STR(DRV_PROCESSING_START),
    SRV_COORD_STR(DRV_PROCESSING_END),
    SRV_COORD_STR(QUEUING_MSGBOX_TO_SEND),
    SRV_COORD_STR(SEND_MSGBOX_TO_CLIENT),
    CLI_COORD_STR(RECV_MSGBOX_FROM_SRV),
    CLI_COORD_STR(SEND_EVENT_TO_INPUT_SYNC)
};

struct touch_statistics_srv_t {
    uint32_t screen_inx;
    uint32_t screen_id;
    uint32_t packet_timestamp_baseaddr;
    uint32_t packet_timestamp_loop;
    uint32_t packet_timestamp_step;
    uint32_t resverd;
    uint64_t received_interrupt_timestamp; /* interrupt received timestamp */
};

struct msgbox_srv_sendto_cli_timestamp_t
{
    uint64_t timestamp;
    uint32_t client_id;
    uint32_t reserved;
};
struct touch_statistics_packet_srv_timestamp_t {
    uint64_t timestamp[TIMESTAMP_PACKET_SRV_NUM];
    struct msgbox_srv_sendto_cli_timestamp_t srv_sendto_cli_timestamp[MAX_CLIENT_NUM];
    uint8_t data[STATISTICS_PACKET_SRV_DATA_SIZE];
    uint32_t id;
};

static struct touch_statistics_srv_t *gstatistics_srv;
static void *statistics_packet_srv_timestamp_baseaddr[MAX_SCREEN];

/* def client */
static uint32_t pstatistics_cli_screen_num;
static uint32_t gclient_id = (~0U - 1);
static struct touch_statistics_cli_t *g_pstatistics_cli[MAX_SCREEN];
static void *statistics_packet_cli_timestamp_baseaddr[MAX_SCREEN];

/* def for the tools */
        
// DIV_ROUND_CLOSEST_ULL(value, divisor) (((value) + ((divisor) / 2)) / (divisor))

#define PMU_CPU_R5_FREQUENCY 1200000000UL
#define PMU_CNT_DIV 64
#define GTC_FREQUENCY 200000000UL
#define SECOND_TO_USECOND 1000000UL
#define GTC_REG_SUB_TO_TIME_US(reg1_value, reg2_value) (((reg2_value - reg1_value) * SECOND_TO_USECOND) / GTC_FREQUENCY)
#define GTC_REG_TO_TIME_US(reg1_value) ((reg1_value * SECOND_TO_USECOND) / GTC_FREQUENCY)

//  (((double)((reg1_value)*PMU_CNT_DIV)/PMU_CPU_R5_FREQUENCY) * SECOND_TO_USECOND)
#define PMU_REG_TO_TIME_US(reg1_value) (((reg1_value) * PMU_CNT_DIV * SECOND_TO_USECOND) / PMU_CPU_R5_FREQUENCY)

#define GMAC_TIME_TO_NS(value) (((value >> 32) *1000000000ULL) + (value & 0xFFFFFFFF))
#define GMAC_TIME_TO_US(value) (GMAC_TIME_TO_NS(value) / 1000ULL)

#define REG_TO_TIME_US GTC_REG_TO_TIME_US
//#define REG_TO_TIME_US PMU_REG_TO_TIME_US
#define REG_SUB_TIME(value1, value2) GTC_REG_SUB_TO_TIME_US(value1, value2)

#ifndef MAX
#define MAX(a, b) ((a) > (b)? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif

#define CALC_TIME_MAX_MIN(delay, max, min)  do { \
    max = MAX(delay, max); \
    if (!min) min = delay; \
    if (delay > 0) \
        min = MIN(delay, min); \
} while(0)

static struct touch_statistics_srv_t *statistics_packet_srv_get_touch_statistics_by_screenid(uint32_t screen_id)
{
    int inx;

    for (inx = 0; inx < MAX_SCREEN; inx++) {
        if (gstatistics_srv[inx].screen_id == screen_id) {
            return &gstatistics_srv[inx];
        }
    }

     return NULL;
}

static uint64_t statistics_packet_srv_get_msgbox_srv_sendto_cli_timestamp(
    struct touch_statistics_packet_srv_timestamp_t *statistics_packet_srv, uint32_t client_id)
{
    int inx;

    for (inx = 0; inx < MAX_CLIENT_NUM; inx++) {
        if (statistics_packet_srv->srv_sendto_cli_timestamp[inx].client_id == client_id) {
            return statistics_packet_srv->srv_sendto_cli_timestamp[inx].timestamp;
        }
    }

    return 0ULL;
}

#ifdef CONFIG_DEBUG_FS

#define DEF_COMMON_GET_TIMESTAMP(step, statistics_packet_srv, statistics_packet_cli, start_time, end_time) \
    do { \
        int cli_step; \
        int srv_max_index = SRV_COORD_TYPE(MAX) - 1; \
                                        \
        if (step < srv_max_index - 1) { \
                start_time = statistics_packet_srv->timestamp[step]; \
                end_time = statistics_packet_srv->timestamp[step + 1]; \
            } else if (step == srv_max_index - 1) { \
                start_time = statistics_packet_srv->timestamp[step]; \
                end_time = statistics_packet_srv_get_msgbox_srv_sendto_cli_timestamp( \
                        statistics_packet_srv, gclient_id); \
            } else if (step == srv_max_index) { \
                /* start_time = statistics_packet_srv->timestamp[srv_max_index]; */\
                start_time = \
                    statistics_packet_srv_get_msgbox_srv_sendto_cli_timestamp( \
                        statistics_packet_srv, gclient_id); \
                end_time = statistics_packet_cli->timestamp[0]; \
            } else { \
                cli_step = step - srv_max_index; \
                start_time = statistics_packet_cli->timestamp[cli_step - 1]; \
                end_time = statistics_packet_cli->timestamp[cli_step]; \
            } \
    } while(0)

static bool statistics_packet_steps_timestamp_is_valid(struct seq_file *s, int cur_loop,
    struct touch_statistics_packet_srv_timestamp_t* statistics_packet_srv,
    struct touch_statistics_packet_cli_timestamp_t* statistics_packet_cli)
{
    int step;
    int zero_timestamp_count = 0;
    uint64_t start_time, end_time;

    for (step = 0; step < TIMESTAMP_PACKET_COORD_MAX - 1; step++) {

        DEF_COMMON_GET_TIMESTAMP(step, statistics_packet_srv, statistics_packet_cli, start_time, end_time);

        if (end_time == 0) {
            zero_timestamp_count++;
        }

        if (zero_timestamp_count > 1) {
            /* invalid packet, more than one zero timestamp */
            return false;
        }
        if (end_time && (start_time >= end_time)) {
            seq_printf(s, "ERROR: packet:%d id:%d(%d) step:%d start_time(0x%llx) >= end_time(0x%llx)\n", cur_loop,
                statistics_packet_srv->id, statistics_packet_cli->id, step, start_time, end_time);
            return false;
        }
        
    }
    return true;
}

static void print_statistics_packet(struct seq_file *s,
    struct touch_statistics_packet_srv_timestamp_t* statistics_packet_srv,
    struct touch_statistics_packet_cli_timestamp_t* statistics_packet_cli,
    int start, int end)
{
    int loop, step;
    uint32_t max_delay = 20000;
    int *error_index;
    int error_index_count = 0;
    int valid_packet_count = 0;
    int invalid_packet_count = 0;
    uint64_t start_time, end_time;
    uint32_t max[TIMESTAMP_PACKET_COORD_MAX-1] = {0};
    uint32_t min[TIMESTAMP_PACKET_COORD_MAX-1] = {0};
    uint64_t total[TIMESTAMP_PACKET_COORD_MAX-1] = {0};
    uint32_t delay[TIMESTAMP_PACKET_COORD_MAX-1] = {0};
    uint64_t total_delay_avg = 0;
    uint64_t steps_delay_avg;
    uint32_t cur_steps_delay, steps_delay_max = 0, steps_delay_min = 0;
    struct touch_statistics_packet_srv_timestamp_t* temp_statistics_packet_srv;
    struct touch_statistics_packet_cli_timestamp_t* temp_statistics_packet_cli;


    if (start < 0 || start >= STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM) {
        pr_err("Invalid start %d\n", start);
        return;
    }

    error_index = kmalloc(STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM * sizeof(int), GFP_KERNEL);
    if (!error_index) {
        pr_err("Failed to allocate memory for error_index\n");
        return;
    }

    if(end >= STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM) {
        end = STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM;
    }

    for (loop = start; loop < end; loop++) {
        temp_statistics_packet_srv = statistics_packet_srv + loop;
        temp_statistics_packet_cli = statistics_packet_cli + loop;
        cur_steps_delay = 0;

        if (!statistics_packet_steps_timestamp_is_valid(s, loop,
            temp_statistics_packet_srv,
            temp_statistics_packet_cli)) {
            invalid_packet_count++;
            continue;
        }
        valid_packet_count++;

        for (step = 0; step < TIMESTAMP_PACKET_COORD_MAX - 1; step++) {

            DEF_COMMON_GET_TIMESTAMP(step, temp_statistics_packet_srv, temp_statistics_packet_cli, start_time, end_time);
            delay[step] = REG_SUB_TIME(start_time, end_time);
            cur_steps_delay += delay[step];

            CALC_TIME_MAX_MIN(delay[step], max[step], min[step]);
            total[step] += delay[step];

            if (step == 0) {
                 // print packet information and first timestamp
                seq_printf(s, "packet:%d id:%d(%d) Start %s:0x%llx",
                        loop, temp_statistics_packet_srv->id, temp_statistics_packet_cli->id,
                        statistics_packet_coord_step_str[0], temp_statistics_packet_srv->timestamp[0]);
            }
            // print each step's timestamp and delay
            seq_printf(s, " %s:0x%llx (%dus)", statistics_packet_coord_step_str[step + 1], end_time, delay[step]);
        }
    
        seq_printf(s, " TotalDelay (%dus)", cur_steps_delay);
        seq_puts(s, "\n");

        CALC_TIME_MAX_MIN(cur_steps_delay, steps_delay_max, steps_delay_min);

        if (cur_steps_delay > max_delay) {
            error_index[error_index_count++] = loop;
        }
    }

    //print all error values
    seq_printf(s, "\nERROR data (count: %d):\n", error_index_count);
    for(loop = 0; loop < error_index_count; loop++){
        temp_statistics_packet_srv = statistics_packet_srv + error_index[loop];
        temp_statistics_packet_cli = statistics_packet_cli + error_index[loop];

        // print each step's timestamp and delay
        for (step = 0; step < TIMESTAMP_PACKET_COORD_MAX - 1; step++) {

            DEF_COMMON_GET_TIMESTAMP(step, temp_statistics_packet_srv, temp_statistics_packet_cli, start_time, end_time);
            delay[step] = REG_SUB_TIME(start_time, end_time);

            if (step == 0) {
                 // print packet information and first timestamp
                seq_printf(s, "packet:%d id:%d(%d) start %s:0x%llx", error_index[loop],
                    temp_statistics_packet_srv->id, temp_statistics_packet_cli->id,
                    statistics_packet_coord_step_str[0], temp_statistics_packet_srv->timestamp[0]);
            }
            seq_printf(s, " %s:0x%llx (%dus)", statistics_packet_coord_step_str[step + 1],
                    end_time, delay[step]);
        }

        seq_puts(s, "\n");

    }

    seq_puts(s, "\n--- Summary ---\n");
    seq_printf(s, "Valid count: %d\n", valid_packet_count);
    seq_puts(s, "Max delay(us):");
    for (step = 0; step < TIMESTAMP_PACKET_COORD_MAX - 1; step++) {
        seq_printf(s, " (%d)", max[step]);
    }
    seq_printf(s, " [TotalMaxDelay (%d)]", steps_delay_max);
    seq_puts(s, "\n");

    seq_puts(s, "Min delay(us):");
    for (step = 0; step < TIMESTAMP_PACKET_COORD_MAX - 1; step++) {
        seq_printf(s, " (%d)", min[step]);
    }
    seq_printf(s, " [TotalMinDelay (%d)]", steps_delay_min);
    seq_puts(s, "\n");

    seq_puts(s, "Average delay(us):");
    for (step = 0; step < TIMESTAMP_PACKET_COORD_MAX - 1; step++) {
        steps_delay_avg = DIV_ROUND_CLOSEST_ULL(total[step], valid_packet_count);
        total_delay_avg += steps_delay_avg;
        seq_printf(s, " (%lld)", steps_delay_avg);
    }
    seq_printf(s, " [TotalAverageDelay (%lld)]", total_delay_avg);
    seq_puts(s, "\n");

    kfree(error_index);
}

static void statistics_packet_srv_timestamp_tool_print_all_info(struct seq_file *s, uint32_t cli_screen_inx)
{
    struct touch_statistics_srv_t *statistics_srv;
    struct touch_statistics_cli_t *statistics_cli = g_pstatistics_cli[cli_screen_inx];
    struct touch_statistics_packet_srv_timestamp_t *packet_srv_timestamp;
    struct touch_statistics_packet_cli_timestamp_t *packet_cli_timestamp;

    if (!statistics_cli) {
        pr_err("Failed to get statistics cli for screen %d\n", cli_screen_inx);
        return;
    }

    statistics_srv = statistics_packet_srv_get_touch_statistics_by_screenid(statistics_cli->screen_id);
    if (!statistics_srv) {
        pr_err("Failed to get statistics srv for screen %d\n", cli_screen_inx);
        return;
    }

    seq_printf(s, "\n== Touch packet timestamp tool for screen [%d] ==\n", statistics_cli->screen_inx);
    seq_printf(s, "screen_inx(srv): %d\n", statistics_srv->screen_inx);
    seq_printf(s, "screen_id: 0x%08x\n", statistics_srv->screen_id);
    seq_printf(s, "packet_timestamp_phyaddr: (r5): 0x%x (a78): 0x%lx\n",
        statistics_srv->packet_timestamp_baseaddr, STATISTICS_PACKET_SRV_TIMESTAMP_TOUCH_BASEADDR(statistics_srv->screen_inx));
    // packet_timestamp_loop of srv is next loop, -1 for the current loop
    seq_printf(s, "packet_timestamp_loop: %d\n", statistics_srv->packet_timestamp_loop - 1);
    seq_printf(s, "packet_timestamp_step: %d\n", statistics_srv->packet_timestamp_step);
    //seq_printf(s, "received_interrupt_timestamp: %lld\n", statistics_srv->received_interrupt_timestamp);
    seq_printf(s, "\nscreen_inx(cli): %d\n", statistics_cli->screen_inx);
    seq_printf(s, "packet_timestamp_phyaddr(a78): 0x%llx\n", statistics_cli->packet_timestamp_phyaddr);
    seq_printf(s, "packet_timestamp_virtaddr(a78): 0x%llx\n", (uint64_t)statistics_cli->packet_timestamp_baseaddr);
    seq_printf(s, "packet_timestamp_loop: %d\n", statistics_cli->packet_timestamp_loop);
    seq_printf(s, "packet_timestamp_step: %d\n", statistics_cli->packet_timestamp_step);

    packet_srv_timestamp = \
            (struct touch_statistics_packet_srv_timestamp_t *)statistics_packet_srv_timestamp_baseaddr[statistics_srv->screen_inx];
    
    packet_cli_timestamp = \
            (struct touch_statistics_packet_cli_timestamp_t *)statistics_packet_cli_timestamp_baseaddr[cli_screen_inx];

    print_statistics_packet(s, packet_srv_timestamp, packet_cli_timestamp,
        0, STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM);
}

static int statistics_packet_timestamp_single_screen_show(struct seq_file *s, void *unused)
{
    uint64_t cli_screen_inx = (uint64_t) s->private;

    statistics_packet_srv_timestamp_tool_print_all_info(s, (uint32_t) cli_screen_inx);
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(statistics_packet_timestamp_single_screen);

void debugfs_create_single_touch_packet_timestamp_tool(struct dentry *parent, int cli_screen_inx)
{

    debugfs_create_file("statistics_packets_timestamp", S_IRUGO, parent,
        (void *) (uint64_t)cli_screen_inx, &statistics_packet_timestamp_single_screen_fops);
}
EXPORT_SYMBOL_GPL(debugfs_create_single_touch_packet_timestamp_tool);

static int statistics_packet_timestamp_multi_screen_show(struct seq_file *s, void *unused)
{
    int i;
    //struct client_info_t *info = s->private;
    int request_screen_num = pstatistics_cli_screen_num;
    

    for (i = 0; i < request_screen_num; i++) {
        statistics_packet_srv_timestamp_tool_print_all_info(s, i);
    }
    return 0;
}
DEFINE_SHOW_ATTRIBUTE(statistics_packet_timestamp_multi_screen);

void debugfs_create_multi_touch_packet_timestamp_tool(struct dentry *parent)
{

    debugfs_create_file("statistics_packets_timestamp", S_IRUGO, parent, NULL, &statistics_packet_timestamp_multi_screen_fops);
}
EXPORT_SYMBOL_GPL(debugfs_create_multi_touch_packet_timestamp_tool);
#endif /* CONFIG_DEBUG_FS */

static int statistics_packet_srv_timestamp_tool_init(struct device *dev)
{
    int i;
    int total_size = sizeof(struct touch_statistics_packet_srv_timestamp_t) * STATISTICS_PACKET_SRV_TIMESTAMP_GRPNUM;
    uint64_t base_addr = STATISTICS_PACKET_SRV_TIMESTAMP_RAM_ADDR;

    gstatistics_srv = (struct touch_statistics_srv_t *) devm_ioremap(dev, STATISTICS_PACKET_SRV_STATINFO_RAM_ADDR,
        MAX_SCREEN * sizeof(struct touch_statistics_srv_t));
    if (!gstatistics_srv) {
        pr_err("Failed to ioremap statistics srv info\n");
        return -ENOMEM;
    }
    for (i = 0; i < MAX_SCREEN; i++) {
        base_addr = STATISTICS_PACKET_SRV_TIMESTAMP_TOUCH_BASEADDR(i);
        statistics_packet_srv_timestamp_baseaddr[i] = devm_ioremap(dev, base_addr, total_size);
        if (!statistics_packet_srv_timestamp_baseaddr[i]) {
            pr_err("Failed to ioremap statistics srv timestamp for screen %d\n", i);
            return -ENOMEM;
        }
    }
    return 0;
}

static int statistics_packet_srv_timestamp_tool_uninit(struct device *dev)
{
    int i;

    if (!gstatistics_srv) {
        pr_err("Failed to get statistics srv info\n");
        return -EINVAL;
    }
    for (i = 0; i < MAX_SCREEN; i++) {
        if (!statistics_packet_srv_timestamp_baseaddr[i]) {
            pr_err("Failed to get statistics srv timestamp for screen %d\n", i);
            return -EINVAL;
        }
        devm_iounmap(dev, statistics_packet_srv_timestamp_baseaddr[i]);
        statistics_packet_srv_timestamp_baseaddr[i] = NULL;
    }
    devm_iounmap(dev, gstatistics_srv);
    gstatistics_srv = NULL;
    return 0;
}


static int statistics_packet_cli_timestamp_tool_init(struct device *dev, uint32_t client_id, int request_screen_num)
{
    int i;

    if (request_screen_num > MAX_SCREEN) {
        pr_err("Invalid screen_num %d\n", request_screen_num);
        return -EINVAL;
    }

    pstatistics_cli_screen_num = request_screen_num;
    gclient_id = client_id;

    for (i = 0; i < request_screen_num; i++) {
        g_pstatistics_cli[i] = statistics_packet_get_touch_statistics_cli_by_inx(i);
        if (!g_pstatistics_cli[i]) {
            pr_err("Failed to get statistics cli for screen %d\n", i);
            return -EINVAL;
        }
        statistics_packet_cli_timestamp_baseaddr[i] = g_pstatistics_cli[i]->packet_timestamp_baseaddr;
        if (!statistics_packet_cli_timestamp_baseaddr[i]) {
            pr_err("Failed to get statistics cli timestamp for screen %d\n", i);
            return -EINVAL;
        }
    }

    return 0;
}

static int statistics_packet_cli_timestamp_tool_uninit(struct device *dev, uint32_t client_id, int request_screen_num)
{
    int i;

    if (request_screen_num > MAX_SCREEN) {
        pr_err("Invalid screen_num %d\n", request_screen_num);
        return -EINVAL;
    }

    if (client_id!= gclient_id) {
        pr_err("Invalid client_id %d\n", client_id);
        return -EINVAL;
    }

    for (i = 0; i < request_screen_num; i++) {
        if (!g_pstatistics_cli[i]) {
            pr_err("Failed to get statistics cli for screen %d\n", i);
            return -EINVAL;
        }
        if (statistics_packet_cli_timestamp_baseaddr[i] != g_pstatistics_cli[i]->packet_timestamp_baseaddr) {
            pr_err("Failed to get statistics cli timestamp for screen %d\n", i);
            return -EINVAL;
        }
        statistics_packet_cli_timestamp_baseaddr[i] = NULL;
        g_pstatistics_cli[i] = NULL;
    }

    return 0;
}

void statistics_packet_timestamp_tool_init(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num)
{
    int ret;

    if (!dev)
        return;

    ret = statistics_packet_srv_timestamp_tool_init(dev);
    if (ret)
        return;

    ret = statistics_packet_cli_timestamp_tool_init(dev, client_id, request_screen_num);
    if (ret)
        return;
    
    pr_info("Touch packet timestamp tool init success\n");
}

void statistics_packet_timestamp_tool_uninit(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num)
{
    int ret;

    if (!dev)
        return;

    ret = statistics_packet_srv_timestamp_tool_uninit(dev);
    if (ret)
        return;

    ret = statistics_packet_cli_timestamp_tool_uninit(dev, client_id, request_screen_num);
    if (ret)
        return;

    pr_info("Touch packet timestamp tool uninit success\n");
}
#endif /* TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE */