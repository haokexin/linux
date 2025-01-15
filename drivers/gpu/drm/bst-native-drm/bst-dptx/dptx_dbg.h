// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#ifndef __DPTX_DBG_H__
#define __DPTX_DBG_H__

#define DPTX_DEBUG_IRQ
#define DPTX_DEBUG_DPCD_CMDS

#define dptx_dbg(_dp, _fmt...) dev_dbg((_dp)->dev, _fmt)
#define dptx_info(_dp, _fmt...) dev_info((_dp)->dev, _fmt)
#define dptx_warn(_dp, _fmt...) dev_warn((_dp)->dev, _fmt)
#define dptx_err(_dp, _fmt...) dev_err((_dp)->dev, _fmt)

#ifdef DPTX_DEBUG_AUX
#define dptx_dbg_aux(_dp, _fmt...) dev_dbg((_dp)->dev, _fmt)
#else
#define dptx_dbg_aux(_dp, _fmt...)
#endif

#ifdef DPTX_DEBUG_IRQ
#define dptx_dbg_irq(_dp, _fmt...) dev_dbg((_dp)->dev, _fmt)
#else
#define dptx_dbg_irq(_dp, _fmt...)
#endif

#endif
