/*
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef __TOUCH_PACKET_STATISTIC_H__
#define __TOUCH_PACKET_STATISTIC_H__
#include <linux/kernel.h>

#if IS_ENABLED(CONFIG_TOUCHSCREEN_VIRT_BST_DEBUG_STATISTICS_DELAY_TIMESTAMPS)
#define TOUCH_STATISTICS_PACKET_TIMESTAMP_ENABLE
#endif
/**************************** Statistics packet timestamp information related function START *****************************/
#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_ENABLE
#define TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE

#define STATISTICS_PACKET_TIMESTAMP_GRPNUM 1024
#define TIMESTAMP_PACKET_STEPS_NUM TIMESTAMP_PACKET_COORD_MAX
#define STATISTICS_PACKET_DATA_SIZE 4

#define STATISTICS_PACKET_COORD_POINTINFO_TIMESTAMP_ENABLE

#ifdef STATISTICS_PACKET_COORD_POINTINFO_TIMESTAMP_ENABLE
enum statistics_packet_coord_pointinfo_timestamp_step {
    TIMESTAMP_PACKET_COORD_RECV_MSGBOX_FROM_SRV, /* recv msgbox from server */
    TIMESTAMP_PACKET_COORD_SEND_EVENT_TO_INPUT_SYNC, /* send event to input_sync */
    TIMESTAMP_PACKET_COORD_MAX, 
};
#endif

//#define STATISTICS_SAVE_PACKET_TIMESTAMP_BY_INTERRUPT(screen_id) statistics_packet_interrupt_timestamps_save(screen_id) 
//#define STATISTICS_UPDATE_PACKET_INTERRUPT_TIMESTAMP(screen_id, step_inx) statistics_packet_interrupt_timestamps_update(screen_id, step_inx)

#define STATISTICS_UPDATE_PACKET_TIMESTAMP(screen_id, step_inx) statistics_packet_timestamps_update(screen_id, step_inx)
#define STATISTICS_UPDATE_PACKET_DATA(screen_id, id, data, len) statistics_packet_data_update(screen_id, id, data, len)
#define STATISTICS_UPDATE_PACKET_LOOP(screen_id) statistics_packet_loop_inc(screen_id)
#else
//#define STATISTICS_SAVE_PACKET_TIMESTAMP_BY_INTERRUPT(screen_id)
#define STATISTICS_UPDATE_PACKET_TIMESTAMP(screen_id, step_inx)
//#define STATISTICS_UPDATE_PACKET_INTERRUPT_TIMESTAMP(screen_id, step_inx)
//#define STATISTICS_UPDATE_PACKET_TIMESTAMP_BY_VAULE(step_inx, value)
#define STATISTICS_UPDATE_PACKET_DATA(screen_id, id, data, len)
#define STATISTICS_UPDATE_PACKET_LOOP(screen_id)
#endif /* TOUCH_STATISTICS_PACKET_TIMESTAMP_ENABLE */

#ifdef STATISTICS_PACKET_COORD_POINTINFO_TIMESTAMP_ENABLE
#define STATISTICS_SAVE_PACKET_COORD_POINTINFO_TIMESTAMP_BY_INTERRUPT(screen_id) STATISTICS_SAVE_PACKET_TIMESTAMP_BY_INTERRUPT(screen_id)
#define STATISTICS_UPDATE_PACKET_COORD_INTERRUPT_TIMESTAMP(screen_id, step_inx) \
    STATISTICS_UPDATE_PACKET_INTERRUPT_TIMESTAMP(screen_id, step_inx)
#define STATISTICS_UPDATE_PACKET_COORD_TIMESTAMP(screen_id, step_inx) STATISTICS_UPDATE_PACKET_TIMESTAMP(screen_id, step_inx)
#define STATISTICS_UPDATE_PACKET_COORD_DATA(screen_id, id, data, len) STATISTICS_UPDATE_PACKET_DATA(screen_id, id, data, len)
#define STATISTICS_UPDATE_PACKET_COORD_LOOP(screen_id) STATISTICS_UPDATE_PACKET_LOOP(screen_id)
#else
#define STATISTICS_SAVE_PACKET_COORD_POINTINFO_TIMESTAMP_BY_INTERRUPT(screen_id)
#define STATISTICS_UPDATE_PACKET_COORD_INTERRUPT_TIMESTAMP(screen_id, step_inx)
#define STATISTICS_UPDATE_PACKET_COORD_TIMESTAMP(screen_id, step_inx)
#define STATISTICS_UPDATE_PACKET_COORD_DATA(screen_id, id, data, len)
#define STATISTICS_UPDATE_PACKET_COORD_LOOP(screen_id)
#endif /* STATISTICS_PACKET_COORD_POINTINFO_TIMESTAMP_ENABLE */

#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_ENABLE

// The statistics information, stored in RAM, can be obtained and parsed by A-core tools.
struct touch_statistics_cli_t {
    uint8_t screen_inx;
    uint32_t screen_id;
    uint64_t packet_timestamp_phyaddr; /* optional: physical address of packet timestamp */
    void *packet_timestamp_baseaddr; /* virtual address of packet timestamp */
    uint32_t packet_timestamp_memsize;
    uint32_t packet_timestamp_loop;
    uint32_t packet_timestamp_step;
//    uint64_t received_interrupt_timestamp[MAX_SCREEN]; /* interrupt received timestamp */
};

struct touch_statistics_packet_cli_timestamp_t {
    uint64_t timestamp[TIMESTAMP_PACKET_STEPS_NUM];
    uint8_t data[STATISTICS_PACKET_DATA_SIZE];
    uint32_t id;
};

void statistics_packet_timestamp_init(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num);
void statistics_packet_timestamp_uninit(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num);
void statistics_packet_loop_inc(uint32_t screen_id);
//void statistics_packet_interrupt_timestamps_save(uint32_t screen_id);
void statistics_packet_timestamps_update(uint32_t screen_id, uint32_t step_inx);
//void statistics_packet_interrupt_timestamps_update(uint32_t screen_id,  uint32_t step_inx);
void statistics_packet_data_update(uint32_t screen_id, uint32_t id, uint8_t *data, uint32_t data_len);

struct touch_statistics_cli_t *statistics_packet_get_touch_statistics_cli_by_inx(int screen_inx);
#endif

#endif /* __TOUCH_PACKET_STATISTIC_H__ */