// SPDX-License-Identifier: GPL-2.0+
/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/watchdog.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include "../watchdog_core.h"
#include <uapi/linux/sched/types.h>

#ifdef CONFIG_BST_AUTOFEED_A78_WATCHDOG
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
//#include <soc/bst/bst_a1000.h>
#include <linux/percpu.h>
#include <linux/completion.h>
#endif


#include "watchdogClient.h"
// #define CLK_TREE_READY
//#define IS_SLT 1
static int smp_irq_flag = 0;

static watchdogClient_t *m_client = NULL;
static watchdogClient_data_t m_data={0};



DEFINE_PER_CPU(bool, bst_wtdog_initialized) = false;
DEFINE_PER_CPU(bool, bst_timer_initialized) = false;
#define TIMER_FEED 10

DEFINE_PER_CPU(int, bst_timer_debug) = 0;


int bst_wdt_set_timeout(struct watchdog_device *wdd, unsigned int top_ms);
int bst_wdt_ping(struct watchdog_device *wdd);

static int debug = 0x3;         // init print err & info log
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "timer test debug level 0-4:\n"
                        "\t\t BIT0 err log print\n"
                        "\t\t BIT1 info log print\n"
                        "\t\t BIT2 debug log print\n"
                        "\t\t BIT3 reg info print\n");

#define wdt_debug(fmt, ...)   \
        do {\
                if (debug & BIT(2)) {   \
                        printk("WDT_DEBUG [%s:%s:%d]: " fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__);    \
                }       \
        } while (0)

#define wdt_err(fmt, ...)     \
        do {\
                if (debug & BIT(0)) {   \
                        printk("WDT_ERR [%s:%s:%d]: " fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__);      \
                }       \
        } while (0)

#define wdt_info(fmt, ...)    \
        do {\
                if (debug & BIT(1)) {   \
                        printk("WDT_INFO [%s:%s:%d]: " fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__);     \
                }       \
        } while (0)




#define WDOG_CONTROL_REG_OFFSET             0x00
#define WDOG_CONTROL_REG_WDT_EN_MASK        0x01
#define WDOG_CONTROL_REG_RESP_MODE_MASK     0x02
#define WDOG_CONTROL_REG_PULSE_LEN_MASK     0x1c
#define WDOG_CONTROL_REG_PULSE_LEN_SHIFT    2
#define WDOG_TIMEOUT_RANGE_REG_OFFSET       0x04
#define WDOG_TIMEOUT_RANGE_TOPINIT_SHIFT    4
#define WDOG_CURRENT_COUNT_REG_OFFSET       0x08
#define WDOG_COUNTER_RESTART_REG_OFFSET     0x0c
#define WDOG_COUNTER_RESTART_KICK_VALUE     0x76
#define WDOG_EOI_REG_OFFSET                            0x14

#define BST_SEC_SAFE_REST_CTRL_OFFSET                0x28
#define BST_SEC_SAFE_REST_SEL_OFFSET                0x24
#define BST_SEC_SAFE_SW_RESET_SHIFT                        0x0
#define BST_WDT0_RST_SHIFT                                        0x0
#define BST_WDT1_RST_SHIFT                                        0x1
#define BST_WDT2_RST_SHIFT                                        0x2
#define BST_WDT3_RST_SHIFT                                        0x3

#define BST_WDT_LSP_NAME_STR                                        "lsp_wdt"
#define BST_WDT_MP4_NAME_STR                                        "soc_a78_wdt"
#define BST_WDT_MP2_NAME_STR                                        "db_a78_wdt"


enum reset_pluse_length {
        PCLK_CYCLES_2,
        PCLK_CYCLES_4,
        PCLK_CYCLES_8,
        PCLK_CYCLES_16,
        PCLK_CYCLES_32,
        PCLK_CYCLES_64,
        PCLK_CYCLES_128,
        PCLK_CYCLES_256,
};

/* The maximum TOP (timeout period) value that can be set in the watchdog. */
#define BST_WDT_MAX_TOP                9 //modify for fusa requirement, timeout <=100ms
#define BST_WDT_MAX_IVI_TOP                11 //modify for fusa requirement, timeout <=100ms
#define BST_WDT_MAX_TOP_MS             100 
#define BST_WDT_SHUTDOWN_TOP           15
#define BST_WDT_SHUTDOWN_TOP_MS        6 * 1000

#define BST_WDT_SHUTDOWN_TOP_MS_SUSPEND        1000
#define BST_WDT_TIMEOUT_IVI_MS          200 //217ms
// bool isShuttingdown = false;

#define BST_WDT_DEFAULT_SECONDS        60

#define BST_WDT_INTE_MODE    0x1

static bool nowayout = WATCHDOG_NOWAYOUT;

static int register_irq;

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct bst_wdt {
        void __iomem *regs;
        struct clk *wclk;
        struct clk *pclk;
        unsigned long rate;
        struct watchdog_device wdd;
        int virq;
        int hwirq;
        const char *name;
        /* Save/restore */
        int wdt_type;
        u32 control;
        u32 timeout;
        resource_size_t phy_base;
#define SOC_LSP_WDT 0
#define SOC_A78_WDT 1
#define DB_A78_WDT 2

#ifdef CONFIG_BST_AUTOFEED_A78_WATCHDOG
        /* a78_wdt feed on kthread */
        u32 a78wdt_num;
        struct hrtimer feed_timer;
        /* kthread completion */
        struct completion wait_done;
#endif
};

static unsigned int bst_wdt_get_timeleft(struct watchdog_device *wdd);

//static void get_wdt_base(int wdt_id, void __iomem **wdt_base);

//static void bst_wdt_reset_report_enable(int reg_bit);
static void bst_wdt_open_rstreport(struct bst_wdt *bst_wdt);

#define to_bst_wdt(wdd)        container_of(wdd, struct bst_wdt, wdd)

static inline int bst_wdt_is_enabled(struct bst_wdt *bst_wdt)
{
        return readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET) &
            WDOG_CONTROL_REG_WDT_EN_MASK;
}

static inline int bst_wdt_top_in_seconds(struct bst_wdt *bst_wdt, unsigned int top)
{
        /*
         * There are 16 possible timeout values in 0..15 where the number of
         * cycles is 2 ^ (16 + i) and the watchdog counts down.
         */
        return (1U << (16 + top)) / bst_wdt->rate;
}


static inline int bst_wdt_top_in_millisecond(struct bst_wdt *bst_wdt, unsigned int top)
{
        /*
         * There are 16 possible timeout values in 0..15 where the number of
         * cycles is 2 ^ (16 + i) and the watchdog counts down.
         */
        return (1U << (16 + top))  / (bst_wdt->rate / 1000);
}


static int bst_wdt_get_top(struct bst_wdt *bst_wdt)
{
        int top = readl(bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET) & 0xF;

        //wdt_err("%s  %d \n", __FUNCTION__, __LINE__);
        return bst_wdt_top_in_millisecond(bst_wdt, top);
}

static enum hrtimer_restart bst_a78wdt_hrtimer_func(struct hrtimer *t)
{
        struct bst_wdt *bst_fwdt = container_of(t, struct bst_wdt, feed_timer);


        ktime_t now, m_kt;

        writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_fwdt->regs +
                WDOG_COUNTER_RESTART_REG_OFFSET);
        
        m_kt = ktime_set(0, TIMER_FEED * NSEC_PER_MSEC); //10ms
        now = hrtimer_cb_get_time(t);
        hrtimer_forward(t, now, m_kt);

        /* restart */
        return HRTIMER_RESTART;
}

int set_bst_wdt_cpu_number(struct bst_wdt *bst_wdt){
	unsigned long num;
	int  ret;

	if (bst_wdt->wdt_type == SOC_A78_WDT) {
			/* over active_cpus_num watchdog shouldn't enable */
			ret = kstrtoul((const char *)(bst_wdt->name + strlen(BST_WDT_MP4_NAME_STR)), 10, &num);
			if (ret == 0)
				bst_wdt->a78wdt_num = num;
			else
				return ret;
			
			if (bst_wdt->a78wdt_num >= num_active_cpus()) {
                                wdt_debug("when active cpus number %d, the a78_wdt number should less than %d, So don't start %s\n", num_active_cpus(), num_active_cpus(), bst_wdt->name);
                                return -EINVAL;
			}
	} else if (bst_wdt->wdt_type == DB_A78_WDT) {
                ret = kstrtoul((const char *)(bst_wdt->name + strlen(BST_WDT_MP2_NAME_STR)), 10, &num);
                if ((ret == 0) && (num == 0 || num == 1)) {
                                bst_wdt->a78wdt_num = 0;
                } else if ((ret == 0) && (num == 2 || num == 3)) {
                                bst_wdt->a78wdt_num = 1;
                } else
                        return ret;
	}
	
	return  0;
}

static int start_feed_wdt_timer(struct bst_wdt *bst_wdt)
{
        ktime_t m_kt;
	struct cpumask mask,origin_mask;
        int rv;
	
		
		
        if(per_cpu(bst_timer_initialized, bst_wdt->a78wdt_num) == true) {
                
                return -1;
        }
	
        sched_getaffinity(current->pid,&origin_mask);

        /*bind hrtimer to cpu*/
        cpumask_clear(&mask);
        cpumask_set_cpu(bst_wdt->a78wdt_num, &mask);
        rv = set_cpus_allowed_ptr(current, &mask);
        if (rv)
           pr_err("Failed set set_cpus_allowed_ptr timer to CPU %u, rv: %d\n",bst_wdt->a78wdt_num, rv);
		
        hrtimer_init(&bst_wdt->feed_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
        bst_wdt->feed_timer.function = bst_a78wdt_hrtimer_func;

        /* two sec, zero nsec */
        m_kt = ktime_set(0, TIMER_FEED * NSEC_PER_MSEC); // 10ms 

        //pr_err("%s %d %d\n",__func__,__LINE__,bst_wdt->a78wdt_num);
        hrtimer_start(&bst_wdt->feed_timer, m_kt, HRTIMER_MODE_REL_PINNED);

	per_cpu(bst_timer_initialized, bst_wdt->a78wdt_num) = true;

        set_cpus_allowed_ptr(current, &origin_mask);
        
        return 0;
}

static int stop_feed_wdt_timer(struct bst_wdt *bst_wdt){
	
	if (true == per_cpu(bst_timer_initialized, bst_wdt->a78wdt_num)){
		hrtimer_cancel(&bst_wdt->feed_timer);
		per_cpu(bst_timer_initialized, bst_wdt->a78wdt_num) = false;
		
		//pr_err("%s %d %d\n",__func__,__LINE__,bst_wdt->a78wdt_num);
	}
	
	return 0;
}




int bst_wdt_ping(struct watchdog_device *wdd)
{
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
	
	writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_wdt->regs +WDOG_COUNTER_RESTART_REG_OFFSET);
    return 0;
}





int bst_wdt_set_timeout(struct watchdog_device *wdd, unsigned int top_ms)
{
        struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
        int i, top_val = BST_WDT_SHUTDOWN_TOP;

        for (i = 0; i <= BST_WDT_SHUTDOWN_TOP; ++i){
                if (bst_wdt_top_in_millisecond(bst_wdt, i) >= top_ms) {
                        top_val = i;
                        break;
                }
        }


        #if defined (CONFIG_BST_C1200_IVI)
        if(top_ms != BST_WDT_SHUTDOWN_TOP_MS)
            top_val= top_val>=BST_WDT_MAX_IVI_TOP?BST_WDT_MAX_IVI_TOP:top_val;
        #else
        if(top_ms != BST_WDT_SHUTDOWN_TOP_MS)
            top_val= top_val>=BST_WDT_MAX_TOP?BST_WDT_MAX_TOP:top_val;
        #endif
        /*
         * Set the new value in the watchdog.  Some versions of bst_wdt
         * have TOPINIT in the TIMEOUT_RANGE register (as per
         * CP_WDT_DUAL_TOP in WDT_COMP_PARAMS_1).  On those we
         * effectively get a pat of the watchdog right here.
         */
        //wdt_err("%s  %d top_val=%d top_ms=%d \n", __FUNCTION__, __LINE__,top_val,top_ms);
        writel(top_val | top_val << WDOG_TIMEOUT_RANGE_TOPINIT_SHIFT, bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
        writel(top_val, bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);
        writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_wdt->regs +WDOG_COUNTER_RESTART_REG_OFFSET);
	
	wdd->timeout = bst_wdt_top_in_millisecond(bst_wdt, top_val);
		
        return 0;
}

static int bst_wdt_set_pulse(struct watchdog_device *wdd, unsigned int rpl)
{
        struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
        u32 val = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

        //wdt_err("%s  %d \n", __FUNCTION__, __LINE__);
        val &= ~WDOG_CONTROL_REG_PULSE_LEN_MASK;
        val |= (rpl << WDOG_CONTROL_REG_PULSE_LEN_SHIFT);
        writel(val, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

        return 0;
}

static void bst_wdt_arm_system_reset(struct bst_wdt *bst_wdt)
{
        u32 val = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

        /* Enable watchdog. */
        val |= WDOG_CONTROL_REG_WDT_EN_MASK;
        writel(val, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);

        writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_wdt->regs + WDOG_COUNTER_RESTART_REG_OFFSET);
}




static int bst_wdt_init(struct watchdog_device *wdd)
{
        struct bst_wdt *bst_wdt = to_bst_wdt(wdd);

		if(per_cpu(bst_wtdog_initialized, bst_wdt->a78wdt_num) == true){
			return 0;
		}
		

		//wdt_err("%s  %d  %d\n", __FUNCTION__, __LINE__,bst_wdt->a78wdt_num);
		 
        bst_wdt_set_pulse(wdd, PCLK_CYCLES_32);
        bst_wdt_set_timeout(wdd, wdd->timeout);
        bst_wdt->timeout = bst_wdt_get_top(bst_wdt);
        bst_wdt_arm_system_reset(bst_wdt);
		
		per_cpu(bst_wtdog_initialized, bst_wdt->a78wdt_num) = true;
		
        return 0;
}


static int bst_wdt_start(struct watchdog_device *wdd){
	
	
	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
	
        if(bst_wdt->wdt_type == SOC_LSP_WDT) {
                        
                bst_wdt_init(wdd);
                start_feed_wdt_timer(bst_wdt);
	}
	
	return 0;
}

static int bst_kernel_wdt_start(struct watchdog_device *wdd)
{

	struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
        bst_wdt_init(wdd);
        start_feed_wdt_timer(bst_wdt);
	
        return 0;
}


/* Once wdt has been enabled, it can be cleared only by a system reset */
static int bst_wdt_stop(struct watchdog_device *wdd)
{
       
#ifdef CONFIG_BST_AUTOFEED_A78_WATCHDOG
        struct watchdog_core_data *wd_data = wdd->wd_data;
        struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
        // isShuttingdown = true;
        
        if (test_bit(_WDOG_HAND_STOP, &wd_data->status) == 0) {

                pr_err("%s %d\n",__func__,__LINE__);
                bst_wdt_set_timeout(wdd,BST_WDT_SHUTDOWN_TOP_MS);
        }

		stop_feed_wdt_timer(bst_wdt);
#endif
        return 0;
}

static int bst_wdt_restart(struct watchdog_device *wdd,
                           unsigned long action, void *data)
{
        struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
        // isShuttingdown = true;
        wdt_err("%s  %d \n", __FUNCTION__, __LINE__);
        //writel(0, bst_wdt->regs + WDOG_TIMEOUT_RANGE_REG_OFFSET);

        bst_wdt_set_timeout(wdd,BST_WDT_SHUTDOWN_TOP_MS);
		
        if (bst_wdt_is_enabled(bst_wdt))
                writel(WDOG_COUNTER_RESTART_KICK_VALUE, bst_wdt->regs + WDOG_COUNTER_RESTART_REG_OFFSET);
        else
                bst_wdt_arm_system_reset(bst_wdt);

        /* wait for reset to assert... */
        mdelay(500);
        return 0;
}

static unsigned int bst_wdt_get_timeleft(struct watchdog_device *wdd)
{
        struct bst_wdt *bst_wdt = to_bst_wdt(wdd);
        // wdt_err("%s  %d WDOG_CURRENT_COUNT_REG_OFFSET:%x bst_wdt->rate=%ld \n",
        // __FUNCTION__, __LINE__,readl(bst_wdt->regs + WDOG_CURRENT_COUNT_REG_OFFSET),bst_wdt->rate);
        return readl(bst_wdt->regs + WDOG_CURRENT_COUNT_REG_OFFSET) /(bst_wdt->rate / 1000);
}

static void bst_wdt_irq_clear(void *dev_id)
{
        struct bst_wdt *bst_wdt = dev_id;

        readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
}

static const struct watchdog_info bst_wdt_ident = {
        .options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
        .identity = "BST Watchdog",
};

static const struct watchdog_ops bst_wdt_ops = {
        .owner = THIS_MODULE,
        .start = bst_wdt_start,
        .stop = bst_wdt_stop,
        .ping = bst_wdt_ping,
        .set_timeout = bst_wdt_set_timeout,
        .get_timeleft = bst_wdt_get_timeleft,
        .restart = bst_wdt_restart,
};


static int bst_wdt_suspend(struct device *dev)
{
        struct bst_wdt *bst_wdt = dev_get_drvdata(dev);
        struct watchdog_device *wdd = &bst_wdt->wdd;
        
        if(bst_wdt->wdt_type == SOC_LSP_WDT){

                clk_disable_unprepare(bst_wdt->pclk);
                clk_disable_unprepare(bst_wdt->wclk);
        }
		
		
		
        bst_wdt->control = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
        bst_wdt->timeout = bst_wdt_get_top(bst_wdt);

        bst_wdt_set_timeout(wdd,BST_WDT_SHUTDOWN_TOP_MS);
		
        stop_feed_wdt_timer(bst_wdt);
        
        per_cpu(bst_wtdog_initialized, bst_wdt->a78wdt_num) = false;

        return 0;
}


 int bst_wdt_reinit(struct bst_wdt *bst_wdt)
{
    
	struct watchdog_device *wdd = &bst_wdt->wdd;

	bst_wdt_set_pulse(wdd, PCLK_CYCLES_32);
	bst_wdt_set_timeout(wdd, bst_wdt->timeout);
	writel(bst_wdt->control, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
	
	///wdt_err("%s  %d \n", __FUNCTION__, __LINE__);
	per_cpu(bst_wtdog_initialized, bst_wdt->a78wdt_num) = true;
	
    return 0;
}



static int bst_wdt_resume(struct device *dev)
{
        struct bst_wdt *bst_wdt = dev_get_drvdata(dev);
        int err;

        if(bst_wdt->wdt_type == SOC_LSP_WDT){
                err = clk_prepare_enable(bst_wdt->wclk);

                if (err)
                        return err;

                err = clk_prepare_enable(bst_wdt->pclk);
                if (err) {
                        clk_disable_unprepare(bst_wdt->wclk);
                        return err;
                }
        }

        bst_wdt_reinit(bst_wdt);
		start_feed_wdt_timer(bst_wdt);
		
        return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bst_wdt_pm_ops, bst_wdt_suspend, bst_wdt_resume);


// #define SYS_CTRL_WRITE_PROTECT  0x640314a4
#define SAFETY_WDT_RESET_INT_MASK    0x640314a4
#define SAFETY_WDT_RESET_INT_CLR    0x640314a0

static void bst_wdt_reset_report_disable(int reg_bit)
{

#ifdef CONFIG_BST_WATCHDOG_FIRMWARE_OFF

        void __iomem    *safety_intmsk;
        u32 reg_val;

        //pr_err("wdt %s, %d reg_bit %d\n", __func__, __LINE__, reg_bit);
        safety_intmsk = ioremap(SAFETY_WDT_RESET_INT_MASK, 4);
        reg_val = readl(safety_intmsk);
        reg_val |= BIT(reg_bit);

        writel(reg_val, safety_intmsk);
#endif 

}

static void bst_wdt_reset_report_enable(int reg_bit)
{

#ifdef CONFIG_BST_WATCHDOG_FIREWALL_OFF

        void __iomem    *safety_intmsk;
        void __iomem    *safety_intclr;
        u32 reg_val;

        //pr_err("wdt %s, %d reg_bit %d\n", __func__, __LINE__, reg_bit);
        safety_intmsk = ioremap(SAFETY_WDT_RESET_INT_MASK, 4);
        safety_intclr = ioremap(SAFETY_WDT_RESET_INT_CLR, 4);

        reg_val = readl(safety_intclr);
        writel((reg_val | BIT(reg_bit)) , safety_intclr);
        writel((reg_val & (~BIT(reg_bit))) , safety_intclr);
        udelay(1);
        reg_val = readl(safety_intmsk);
        reg_val &= (~BIT(reg_bit));
        writel(reg_val, safety_intmsk);
#endif

}

static void bst_wdt_open_rstreport(struct bst_wdt *bst_wdt)
{
        struct watchdog_device *wdd;
        wdd = &bst_wdt->wdd;
        //wdt_err("wdt %s, %d phy_base %llx\n", __func__, __LINE__, bst_wdt->phy_base);
        // isShuttingdown = false;
        bst_wdt_set_timeout(wdd,BST_WDT_MAX_TOP_MS);

        switch(bst_wdt->phy_base){
        case 0x32068000:
                bst_wdt_reset_report_enable(4);//4
        break;
        case 0x32069000:
                bst_wdt_reset_report_enable(5);
        break;
        case 0x3206a000:
                bst_wdt_reset_report_enable(6);
        break;
        case 0x3206b000:
                bst_wdt_reset_report_enable(7);
        break;
        case 0x3206c000:
                bst_wdt_reset_report_enable(8);
        break;
        case 0x3206d000:
                bst_wdt_reset_report_enable(9);
        break;
        case 0x3206e000:
                bst_wdt_reset_report_enable(10);
        break;
        case 0x3206f000:
                bst_wdt_reset_report_enable(11);
        break;
        case 0x33002000:
                bst_wdt_reset_report_enable(12);
        break;
        case 0x33003000:
                bst_wdt_reset_report_enable(13);
        break;
        case 0x33004000:
                bst_wdt_reset_report_enable(14);
        break;
        case 0x33005000:
                bst_wdt_reset_report_enable(15);
        break;
        }
}

static void bst_wdt_shutdown(struct platform_device *pdev)
{
        struct bst_wdt *bst_wdt = platform_get_drvdata(pdev);
        struct watchdog_device *wdd;
        wdd = &bst_wdt->wdd;
		
       //pr_err("%s,wdt phy_base %llx\n", __func__, bst_wdt->phy_base);
        // isShuttingdown = true;
        bst_wdt_set_timeout(wdd,BST_WDT_SHUTDOWN_TOP_MS);
        stop_feed_wdt_timer(bst_wdt);
        
       // pr_err("%s %d+++", __FUNCTION__, __LINE__);

        switch(bst_wdt->phy_base){
        case 0x32068000:
                bst_wdt_reset_report_disable(4);//4
        break;
        case 0x32069000:
                bst_wdt_reset_report_disable(5);
        break;
        case 0x3206a000:
                bst_wdt_reset_report_disable(6);
        break;
        case 0x3206b000:
                bst_wdt_reset_report_disable(7);
        break;
        case 0x3206c000:
                bst_wdt_reset_report_disable(8);
        break;
        case 0x3206d000:
                bst_wdt_reset_report_disable(9);
        break;
        case 0x3206e000:
                bst_wdt_reset_report_disable(10);
        break;
        case 0x3206f000:
                bst_wdt_reset_report_disable(11);
        break;
        case 0x33002000:
                bst_wdt_reset_report_disable(12);
        break;
        case 0x33003000:
                bst_wdt_reset_report_disable(13);
        break;
        case 0x33004000:
                bst_wdt_reset_report_disable(14);
        break;
        case 0x33005000:
                bst_wdt_reset_report_disable(15);
        break;
        }
}

/* LSP wdt irq handle */
static irqreturn_t bst_wdt_spi_irq_handle(int irq, void *dev_id)
{
        //struct bst_wdt *bst_wdt = dev_id;

        bst_wdt_irq_clear(dev_id);
        //wdt_err("watchdog%d: irq trigger and clear \n", bst_wdt->wdd.id);

        return IRQ_HANDLED;
}

static void wdt_percpu_irq_register(void *para)
{
        struct bst_wdt *bst_wdt = (struct bst_wdt *)para;

        //wdt_err("enable_percpu_irq cpu%d irq_num = %u \n", get_cpu(), bst_wdt->virq);
        enable_percpu_irq(bst_wdt->virq, IRQ_TYPE_NONE);
}

/* SOC_A78 wdt irq handle */
static irqreturn_t bst_wdt_ppi_irq_handle_soc(int irq, void *dev_id)
{
        //int cpu_id;
        //void __iomem    *wdt_base;
        

       // cpu_id = smp_processor_id();

        //(cpu_id, &wdt_base);

#if defined(CONFIG_C1200_SLT)
        struct bst_wdt *bst_wdt  = (struct bst_wdt * )dev_id;
        readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
#endif
        //wdt_err("wdt %s, %d cpu[%d] soc_a78 irq = %d trigger and clear \n", __func__, __LINE__, cpu_id, irq);

        return IRQ_HANDLED;
}

/* DB_A78 wdt irq handle */
static irqreturn_t bst_wdt_ppi_irq_handle_db(int irq, void *dev_id)
{
        int cpu_id;

        struct bst_wdt *bst_wdt  = (struct bst_wdt * )dev_id;
        cpu_id = smp_processor_id();

        if (cpu_id == 0) {
                readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
                readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
        } else if (cpu_id == 1) {
                readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
                readl(bst_wdt->regs + WDOG_EOI_REG_OFFSET);
        }

        return IRQ_HANDLED;
}




/*
 * get_psmid_from_safety: send block_id to safetylib get psm status,
 * this function is called by driver.
 * @block_id_in: the block_id value.
 * @block_id_out: get block_id form safetylib .
 * @psm_id_out: get psm id form safetylib.
 * @return 0 if it successes to get psm status.
 */
int get_psmid_from_safety(uint8_t block_id_in ,uint8_t *block_id_out, uint32_t *psm_id_out){
	int ret = -1;
	int i = 0;
	uint8_t blockid = 0;
        watchdog_UInt32Array4_t *psm_id=NULL;
	watchdog_ErrorEnum_t err=0;

	if(!m_client)
		return ret;

	ret = m_client->watchdog_client.fusaenable_method_sync(block_id_in,&blockid,&psm_id,&err,500,NULL);
        if(ret < 0|| err != 0){
                pr_err("%s,%d ret is %d,err = %d.", __func__, __LINE__,ret,(int)err);
                return -2;
        }

        *block_id_out = blockid;
        if(psm_id != NULL){
                for(i=0;i<4;i++)
                psm_id_out[i]=(*psm_id)[i];
        }

	return 0;
}



static int bst_wdt_drv_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct watchdog_device *wdd;
        struct bst_wdt *bst_wdt;
        struct resource *mem;
        int ret = 0;
        struct irq_desc *irq_desc;
        int cpu_online_num;
        int cpu_id;
        int i;//, j;
        u32 rmod;
        int status = 1;
        ipc_inf_version_t version = {0};

        uint8_t block_id_in;
        uint8_t block_id_out;
        uint32_t psm_id_out[4] = {0};
		

        #if defined(CONFIG_BST_C1200_ADAS)
        m_data.com_data.pid = CPU_4;
	m_data.com_data.sid = 12;
	m_data.com_data.fid = F1;
        #elif defined(CONFIG_BST_C1200_IVI)
        m_data.com_data.pid = CPU_0;
	m_data.com_data.sid = 12;
	m_data.com_data.fid = DEF;
        #else
        m_data.com_data.pid = CPUMP2_0;
	m_data.com_data.sid = 12;
	m_data.com_data.fid = DEF;
        #endif


	m_client = watchdogClient_init(&m_data);
	if (!m_client) {
		pr_err("watchdogClient_init init client fail\n");
		return ret;
	}

        
	// client start
	ret = m_client->start();
        if(ret) {
                pr_err("watchdogClient_init start failed\n");
		return ret;
        }
	// get version
	version = m_client->watchdog_client.version();

        #if defined(CONFIG_BST_C1200_DB) 
             block_id_in = 0x6d;
        #else
             block_id_in = 0x76;
        #endif

        ret = get_psmid_from_safety(block_id_in ,&block_id_out,psm_id_out);
        if(ret == 0)
        {
                #if defined(CONFIG_BST_C1200_DB)
                        status =  psm_id_out[0] & (1U<<14);
                #else
                        status = psm_id_out[0] & (1U<<4);
                #endif
        }


        cpu_online_num = num_online_cpus();
        cpu_id = smp_processor_id();

     

        bst_wdt = devm_kzalloc(dev, sizeof(*bst_wdt), GFP_KERNEL);
        if (!bst_wdt)
                return -ENOMEM;

        mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        bst_wdt->regs = devm_ioremap_resource(dev, mem);
        if (IS_ERR(bst_wdt->regs))
                return PTR_ERR(bst_wdt->regs);

        bst_wdt->phy_base = mem->start;

        bst_wdt_open_rstreport(bst_wdt);
		
        of_property_read_string(dev->of_node, "bst_wdt_name", &(bst_wdt->name));
        if (strstr(bst_wdt->name, BST_WDT_LSP_NAME_STR) != NULL) {
                bst_wdt->wdt_type = SOC_LSP_WDT;
        } else if (strstr(bst_wdt->name, BST_WDT_MP4_NAME_STR) != NULL) {
                bst_wdt->wdt_type = SOC_A78_WDT;
        } else if (strstr(bst_wdt->name, BST_WDT_MP2_NAME_STR) != NULL) {
                bst_wdt->wdt_type = DB_A78_WDT;
        }

#if 0//ndef CLK_TREE_READY
        switch (bst_wdt->wdt_type)
        {
                case 0:
                        bst_wdt->rate = 200000000;
                        break;
                case 1:
                        bst_wdt->rate = 600000000;
                        break;
                case 2:
                        bst_wdt->rate = 333333333;
                        break;
                default:
                        break;
        }
#else
	if(bst_wdt->wdt_type == DB_A78_WDT)
		bst_wdt->rate = 333333333;
	else{
        	bst_wdt->pclk = devm_clk_get(dev, "pclk");
        	if (IS_ERR(bst_wdt->pclk))
            		return PTR_ERR(bst_wdt->pclk);

        	ret = clk_prepare_enable(bst_wdt->pclk);
        	if (ret)
        	   return ret;
        	bst_wdt->wclk = devm_clk_get(dev, "wclk");
        	if (IS_ERR(bst_wdt->wclk))
            		return PTR_ERR(bst_wdt->wclk);

        	ret = clk_prepare_enable(bst_wdt->wclk);
        	if (ret)
        	   return ret;

        	bst_wdt->rate = clk_get_rate(bst_wdt->wclk);
        	if (bst_wdt->rate == 0) {
                	ret = -EINVAL;
                	goto out_disable_clk;
        	}
	}
#endif		
	set_bst_wdt_cpu_number(bst_wdt);
		
		
        bst_wdt->virq = platform_get_irq(pdev, 0);
        irq_desc = irq_to_desc(bst_wdt->virq);
        bst_wdt->hwirq = irq_desc->irq_data.hwirq;

        //wdt_info("bst_wdt->virq = %d, hwirq = %ld", bst_wdt->virq, irq_desc->irq_data.hwirq);

        if (bst_wdt->virq) {
                if (irq_desc->irq_data.hwirq > 32) {                                                            // SPI IRQ register
                        //wdt_info("SPI IRQ register \n");
                        ret = devm_request_irq(dev, bst_wdt->virq, bst_wdt_spi_irq_handle, IRQF_SHARED, pdev->name, bst_wdt);
                        if (ret < 0)
                                wdt_err("failed to request SPI IRQ \n");
                } else {                                                                                        // PPI IRQ register
                        if (bst_wdt->wdt_type == SOC_A78_WDT) {                                                 // soc_a78 wdt
                                if (smp_irq_flag == 0) {                                                        // online core register irq_handle && irq
                                        //wdt_info("PPI IRQ for percpu SOC_A78_WDT \n");

                                        ret = request_percpu_irq(bst_wdt->virq, bst_wdt_ppi_irq_handle_soc, pdev->name, bst_wdt);
                                        for (i = 0; i < cpu_online_num; i++) {
                                                smp_call_function_single(i, wdt_percpu_irq_register, (void *)bst_wdt, 1);
                                        }
                                }
                                smp_irq_flag ++;
                        } else if (bst_wdt->wdt_type == DB_A78_WDT && bst_wdt->virq != register_irq) {          // db_a78 wdt
                                       if (smp_irq_flag < 2) {                                                  // online core register irq_handle && irq
                                        register_irq = bst_wdt->virq;
                                        //wdt_info("DB_A78 WDT register irq %ld \n", irq_desc->irq_data.hwirq);
                                        //wdt_info("PPI IRQ for percpu DB_A78_WDT \n");
                                        ret = request_percpu_irq(bst_wdt->virq, bst_wdt_ppi_irq_handle_db, pdev->name, bst_wdt);
                                        for (i = 0; i < cpu_online_num; i++) {
                                                smp_call_function_single(i, wdt_percpu_irq_register, (void *)bst_wdt, 1);
                                        }
                                }
                                smp_irq_flag ++;
                        }
                        if (ret < 0)
                                wdt_err("PPI IRQ have been register \n");
                }
        }

        /* Set response mode */
        if (of_property_read_u32(dev->of_node, "response-mode", &rmod) == 0) {
                bst_wdt->control = readl(bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
                //wdt_info("WDT %s mode \n", (rmod == 1) ? "INTERRUPT" : "RESET");

                if (rmod == BST_WDT_INTE_MODE) {
                        bst_wdt->control |= WDOG_CONTROL_REG_RESP_MODE_MASK;
                } else
                        bst_wdt->control &= ~WDOG_CONTROL_REG_RESP_MODE_MASK;

                writel(bst_wdt->control, bst_wdt->regs + WDOG_CONTROL_REG_OFFSET);
        }

        wdd = &bst_wdt->wdd;
        wdd->info = &bst_wdt_ident;
        wdd->ops = &bst_wdt_ops;
        wdd->min_timeout = 1;
        wdd->max_hw_heartbeat_ms = bst_wdt_top_in_millisecond(bst_wdt, BST_WDT_MAX_TOP);
        wdd->parent = dev;

        watchdog_set_drvdata(wdd, bst_wdt);
        watchdog_set_nowayout(wdd, nowayout);
        watchdog_init_timeout(wdd, 0, dev);

        /*
         * If the watchdog is already running, use its already configured
         * timeout. Otherwise use the default or the value provided through
         * devicetree.
         */
        if (bst_wdt_is_enabled(bst_wdt)) {
                wdd->timeout = bst_wdt_get_top(bst_wdt);
                set_bit(WDOG_HW_RUNNING, &wdd->status);
        } else {
                #if defined(CONFIG_BST_C1200_IVI)
                wdd->timeout = BST_WDT_TIMEOUT_IVI_MS;
                #else
                wdd->timeout = BST_WDT_MAX_TOP_MS;
                #endif
                //watchdog_init_timeout(wdd, 0, dev);
        }

        platform_set_drvdata(pdev, bst_wdt);
        watchdog_set_restart_priority(wdd, 128);


        ret = watchdog_register_device(wdd);
        if (ret){
            goto out_disable_clk;
	}

        if(bst_wdt->wdt_type != SOC_LSP_WDT) {
                if(status)
                  bst_kernel_wdt_start(wdd);
        }
		
        return 0;

out_disable_clk:
         iounmap(bst_wdt->regs);
        if(bst_wdt->wdt_type != DB_A78_WDT){
                clk_disable_unprepare(bst_wdt->pclk);
                clk_disable_unprepare(bst_wdt->wclk);                
        }

        return ret;
}

static int bst_wdt_drv_remove(struct platform_device *pdev)
{
        struct bst_wdt *bst_wdt = platform_get_drvdata(pdev);

        iounmap(bst_wdt->regs);

        watchdog_unregister_device(&bst_wdt->wdd);
        if(bst_wdt->wdt_type != DB_A78_WDT){
                clk_disable_unprepare(bst_wdt->pclk);
                clk_disable_unprepare(bst_wdt->wclk);
        }
        return 0;
}





#ifdef CONFIG_OF
static const struct of_device_id bst_wdt_of_match[] = {
        {.compatible = "snps,dw-wdt", },
        { /* sentinel */  }
};

MODULE_DEVICE_TABLE(of, bst_wdt_of_match);
#endif

static struct platform_driver bst_wdt_driver = {
        .probe = bst_wdt_drv_probe,
        .remove = bst_wdt_drv_remove,
        .shutdown = bst_wdt_shutdown,
        .driver = {
                   .name = "bst_wdt",
                   .of_match_table = of_match_ptr(bst_wdt_of_match),
                   .pm = pm_sleep_ptr(&bst_wdt_pm_ops),
                   
        },
};

module_platform_driver(bst_wdt_driver);

MODULE_AUTHOR("Bst Ltd.");
MODULE_DESCRIPTION("BST Watchdog Driver");
MODULE_LICENSE("GPL v2");
