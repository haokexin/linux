/*
 * SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef TOUCH_PACKET_STATISTIC_TOOL_H
#define TOUCH_PACKET_STATISTIC_TOOL_H

#include <linux/types.h>


#ifdef TOUCH_STATISTICS_PACKET_TIMESTAMP_TOOL_ENABLE
void debugfs_create_single_touch_packet_timestamp_tool(struct dentry *parent, int cli_screen_inx);
void debugfs_create_multi_touch_packet_timestamp_tool(struct dentry *parent);
void statistics_packet_timestamp_tool_init(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num);
void statistics_packet_timestamp_tool_uninit(struct device *dev, uint32_t client_id, void **pdata, int request_screen_num);
#endif

#endif /* TOUCH_PACKET_STATISTIC_TOOL_H */