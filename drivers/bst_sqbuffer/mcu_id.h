// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef _MCU_ID_H
#define _MCU_ID_H

typedef enum {
    MCU_CORE0 = 0U,
    MCU_CORE1,
    MCU_CORE2,
    MCU_CORE3,
    MCU_CORE4,
    MCU_CORE5,
    MCU_ADAS,
    MCU_IVI,
    MCU_DB,
    MCU_RT0,
    MCU_RT1,
    MCU_RT2,
    MCU_RT3,
    MCU_RT4,
    MCU_RT5
} MCU_ID;

#define MCU_CORE0_MASK (1U << MCU_CORE0)
#define MCU_CORE1_MASK (1U << MCU_CORE1)
#define MCU_CORE2_MASK (1U << MCU_CORE2)
#define MCU_CORE3_MASK (1U << MCU_CORE3)
#define MCU_CORE4_MASK (1U << MCU_CORE4)
#define MCU_CORE5_MASK (1U << MCU_CORE5)
#define MCU_ADAS_MASK (1U << MCU_ADAS)
#define MCU_IVI_MASK (1U << MCU_IVI)
#define MCU_DB_MASK (1U << MCU_DB)
#define MCU_RT0_MASK (1U << MCU_RT0)
#define MCU_RT1_MASK (1U << MCU_RT1)
#define MCU_RT2_MASK (1U << MCU_RT2)
#define MCU_RT3_MASK (1U << MCU_RT3)
#define MCU_RT4_MASK (1U << MCU_RT4)
#define MCU_RT5_MASK (1U << MCU_RT5)

#endif /* _MCU_ID_H */
