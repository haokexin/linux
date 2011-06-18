/*******************************************************************************
 *
 * Copyright (c) 2009 Cavium Networks
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 ******************************************************************************/
#ifndef CNS3XXX_CONFIG_H
#define CNS3XXX_CONFIG_H

#define CNS3XXX_CPU_MIB_COUNTER
#define CNS3XXX_MAC0_MIB_COUNTER
#define CNS3XXX_MAC1_MIB_COUNTER
#define CNS3XXX_MAC2_MIB_COUNTER

#define CONFIG_CNS3XXX_NAPI
#ifdef CONFIG_CNS3XXX_NAPI
#define CNS3XXX_NAPI_WEIGHT 16
#endif

#define CNS3XXX_TX_HW_CHECKSUM
#define CNS3XXX_RX_HW_CHECKSUM

#undef CNS3XXX_SHOW_LINK_STATUS

#ifdef CNS3XXX_SHOW_LINK_STATUS
#define CNS3XXX_STATUS_ISR
#endif
#undef CNS3XXX_STATUS_ISR
#undef CNS3XXX_TEST_ONE_LEG_VLAN
#undef CNS3XXX_TX_DSCP_PROC


#define CNS3XXX_FSQF_RING0_ISR
#undef CNS3XXX_TSTC_RING0_ISR
#undef CNS3XXX_TSTC_RING1_ISR

#undef CNS3XXX_COMPARE_PACKET
#undef CONFIG_FPGA_10


#define CNS3XXX_FREE_TX_IN_RX_PATH

#define CNS3XXX_PVID_PROC
#define CNS3XXX_SARL_PROC


#undef CNS3XXX_DOUBLE_RX_RING
#undef CNS3XXX_DOUBLE_TX_RING

#define CNS3XXX_CONFIG_CHANGE_TX_RING

#ifdef CNS3XXX_DOUBLE_RX_RING
#define CNS3XXX_FSQF_RING1_ISR
#endif


#define PRINT_INFO printk

#endif
