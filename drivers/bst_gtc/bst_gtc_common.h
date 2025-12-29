// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */
#ifndef BST_GTC_H
#define BST_GTC_H

#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/miscdevice.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <net/genetlink.h>

extern struct workqueue_struct *gtc_wq;
extern struct work_struct gtc_work;

#define BST_XGMAC_IDX               0
#define BST_GTC_DRIVER_NAME         "bst_gtc"
#define BST_GTC_IOMEM_NUM           3
#define BST_GTC_MAX_SYNCBIT         63

#define BST_GTC_DEV_NAME          "gtc"
#define BST_GTC_DEV_ID_LEN        1

#define TOP_CRM_BASE_ADDR       0x30002000
#define GTC_TSGEN_CNTCR			0x00000000
#define GTC_TSGEN_CNTCVL		0x00000008
#define GTC_TSGEN_CNTCVU		0x0000000c
#define GTC_LATCH_EN_SEL		0x00000070
#define GTC_CNTREG2_LO          0x00000090
#define GTC_CNTREG2_HI          0x00000094
#define GTC_CNTREG20_LO         0x00000120
#define GTC_CNTREG20_HI         0x00000124
#define GTC_INTR_LATCH_DATA0	0x00000180
#define GTC_INTR_LATCH_DATA1	0x00000184
#define GTC_SYNCBITS_SEL		0x00000200
#define GTC_LATCH_CLR		    0x00000210
#define GTC_INTR_STATUS		    0x00000218
#define GTC_INTR_MASK           0x0000021c
#define GTC_REFCLK_DIV_PARA     0x000002d8
#define GTC_REFCLK_MUX_CTRL     0x000003a8

#define GTC_REFCLK_DIV_OFFSET   0x18
#define GTC_REFCLK_MUX_OFFSET   0x5

#define GTC_IOC_MAGIC 'g'
#define GTC_IOC_LATCH_CFG    _IOW(GTC_IOC_MAGIC, 1, int)
#define GTC_IOC_INTR_CFG     _IOW(GTC_IOC_MAGIC, 2, int)
#define GTC_IOC_MUX_CFG      _IOW(GTC_IOC_MAGIC, 3, int)
#define GTC_IOC_GET_PARM     _IOR(GTC_IOC_MAGIC, 4, struct time_sync_parm)
#define GTC_IOC_GET_FREQ     _IOR(GTC_IOC_MAGIC, 5, struct gtc_freq)
#define GTC_IOC_LOG          _IOW(GTC_IOC_MAGIC, 6, int)
#define GTC_IOC_TEST_KTIME   _IOW(GTC_IOC_MAGIC, 7, u64)
#define GTC_IOC_MAXNR    8

#define GTC_WQ_DEF_CPU      2
#define GTC_GENL_NAME       "BST_GTC_GENL"
#define GTC_GENL_VERSION    1
#define GTC_MSG_SEQ_MAX     1500
#define NLA_DATA(na)        ((void *)((char *)(na) + NLA_HDRLEN))
#define BST_GTC_MCGRP_USER_NAME     "BST_GTC_USER"

typedef struct user_msg {
    char flag;
    unsigned int data;
} user_msg_t;

/* commands */
enum {
    GTC_CMD_UNSPEC,
    GTC_CMD_SYNC_INFO,
    GTC_CMD_USER_INFO,
    __GTC_CMD_MAX,
};
#define GTC_CMD_MAX (__GTC_CMD_MAX - 1)

/* attribute */
enum {
    GTC_ATTR_UNSPEC,
    GTC_ATTR_SYNC_INFO,
    GTC_ATTR_USER_INFO,
    __GTC_ATTR_MAX,
};
#define GTC_ATTR_MAX (__GTC_ATTR_MAX - 1)

enum gtc_sync_signal {
    BST_GTC_SYNC,
    BST_SOC_XGMAC_SYNC0 = 5,
    BST_SOC_XGMAC_SYNC1,
    BST_SOC_XGMAC_SYNC2,
    BST_SOC_XGMAC_SYNC3,
    BST_MAX_SYNC = 28,
};

enum gtc_genl_multicast_groups {
	BST_GTC_MCGRP_USER_INFO,
};

struct bst_gtc {
    struct platform_device *pdev;
    struct miscdevice miscdev;
    void *addr;
    int gtc_irq;
    int mux_idx;
    int gtc_syncbit;
    spinlock_t mono_lock;
    spinlock_t ctm_lock;
    u32 freq_ns;
};

struct gtc_mux_pin_t {
    u8 reg_off;
    u32 dis_bits;
};

struct time_sync_parm {
	unsigned int latch_gtc_hicnt;
    unsigned int latch_gtc_lwcnt;
    long long phc_utc_sec;
    long phc_utc_nsec;
    unsigned int gtc_hicnt;
    unsigned int gtc_lwcnt;
};

struct gtc_freq {
	unsigned int clk_freq;
    unsigned int clk_div;
};

int bst_gtc_miscdev_init(struct bst_gtc *pbst_gtc);
void bst_gtc_miscdev_exit(struct bst_gtc *pbst_gtc);

void gtc_latch_en_sel(void __iomem *ioaddr, u32 value);
void gtc_syncbits_sel(void __iomem *ioaddr, u32 value);
void gtc_latch_clear(void __iomem *ioaddr, u32 value);
void gtc_intr_mask(void __iomem *ioaddr, u32 value);
int gtc_mux_config(void __iomem *ioaddr, u32 value);
int gtc_send_msg_to_user(struct time_sync_parm *msg, int len);
int bst_gtc_get_freq(struct gtc_freq *freq_info);

#ifdef CONFIG_BST_DWMAC_ETH
extern int bstgmac_get_synctime(unsigned int gmac_idx, long long *sec, long *nsec);
#endif
extern int scmi_read(u32 reg,u32 *val);
extern int scmi_write(u32 reg,u32 val);
#endif

