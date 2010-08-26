/*!
 * @file pch_debug.h
 * @brief Provides the macro definitions used for debugging.
 * @version 1.0.0.0
 * @section
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 */

/*
 * History:
 * Copyright (C) 2010 OKI SEMICONDUCTOR Co., LTD.
 *
 * created:
 * OKI SEMICONDUCTOR 04/14/2010
 * modified:
 *
 */

#ifndef __PCH_DEBUG_H__
#define __PCH_DEBUG_H__

#ifdef MODULE
#define PCH_LOG(level, fmt, args...) printk(level "%s:" fmt "\n",\
       THIS_MODULE->name, ##args)
#else
#define PCH_LOG(level, fmt, args...) printk(level "%s:" fmt "\n" ,\
	__FILE__, ##args)
#endif


#ifdef DEBUG
 #define PCH_DEBUG(fmt, args...) PCH_LOG(KERN_DEBUG, fmt, ##args)
#else
 #define PCH_DEBUG(fmt, args...)
#endif

#ifdef PCH_TRACE_ENABLED
 #define PCH_TRACE PCH_DEBUG
#else
 #define PCH_TRACE(fmt, args...)
#endif

#define PCH_TRACE_ENTER PCH_TRACE("Enter %s", __func__)
#define PCH_TRACE_EXIT  PCH_TRACE("Exit %s", __func__)


#endif
