// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (C) 2025 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <asm-generic/signal.h>

#include "error_inject.h"

#include "msgbox/safety-client/src-gen/safetylibClient.h"
static safetylibClient_t *m_client = NULL;
static safetylibClient_data_t m_data={0};
static u8 sub_success_flag = 0;

static void on_dst_changed(bool flag, void *ext)
{
	if (flag)
		BST_SAFETYLIB_DEBUG_PRINTK("safetylib dst is online.");
	else
		BST_SAFETYLIB_LOG_PRINTK("safetylib dst is offline.");

	*((bool *)ext) = flag;
}

static int init_sessions(void)
{
	int ret = -1;
	ipc_inf_version_t version = {0};
	bool dst_avail = false;

	m_data.com_data.pid = CONFIG_FUSA_INJECT_PID;

	m_client = safetylibClient_init(&m_data);
	if (!m_client) {
		BST_SAFETYLIB_ERR_PRINTK("safetylibClient init client fail.");
		return ret;
	}

	// get version
	version = m_client->safetylib_client.version();
	BST_SAFETYLIB_DEBUG_PRINTK("safetylibClient interface version : major %d, minor %d.\n", version.major, version.minor);
	// start test_client_t
	ret = m_client->start();
	if (ret < 0) {
		BST_SAFETYLIB_ERR_PRINTK("safetylibClient start client fail.");			
		return ret;
	}

	ret = m_client->safetylib_client.register_avail_changed(on_dst_changed, &dst_avail);

	BST_SAFETYLIB_DEBUG_PRINTK("safetylib-client msgbox start.");	

	return 0;
}

static void destroy_sessions(void)
{
	int ret = 0;
	if (!m_client) 
		return;
    // stop to receive msg, must
    ret = m_client->stop();
    if (ret < 0)
    {
        BST_SAFETYLIB_ERR_PRINTK("safetylib_client stop failed,ret = %d !",ret);
		return;
    }
    ret = safetylibClient_destroy();
	if (ret)
		BST_SAFETYLIB_ERR_PRINTK("safetylib_client destroy failed,ret = %d !",ret);
    return ; 
}

static void core_cache_error_inject(void)
{
	u64 temp;
	asm volatile(
		 " mrs	   %0, errselr_el1 \n"
		 // SEL, [0]
		 // Selects which error record should be accessed.
		 // 0 Select error record 0 containing errors from L1 and L2 RAMs located on the Cortex-A78AE core.
		 // 1 Select error record 1 containing errors from L3 RAMs located on the DSU-AE.
		 " bic	   %0, %0, #(1 << 0) \n"
		 " msr	   errselr_el1, %0 \n"
		 // ERR0PFGCDNR is the Cortex-A78AE node register that generates one of the errors
		 // that are enabled in the corresponding ERR0PFGCTLR register.
		 " mov	   %0, #1000 \n"
		 " msr	   s3_0_c15_c2_2, %0 \n"
	
		 // The ERR0PFGCTLR is the Cortex-A78AE node register that enables controlled fault generation.
		 " mrs	   %0, s3_0_c15_c2_1 \n"
		 // Count down enable.
		 // The value that is held in the ERR0PFGCDNR register is transferred into the Error
		 // Generation Counter. The Error Generation Counter counts down.
		 " orr	   %0, %0, #(1 << 31) \n"
		 // Uncontainable error generation enable.
		 // An uncontainable error might be generated when the Error Generation Counter is triggered.
		 " orr	   %0, %0, #(1 << 1) \n"
		 " msr	   s3_0_c15_c2_1, %0 \n"
		 "isb\n\t"
		 : "=&r" (temp)

	);
}

static void dsu_cache_error_inject(void)
{
	u64 temp;
	asm volatile(
		 " mrs	   %0, errselr_el1 \n"
		 // SEL, [1]
		 // Selects which error record should be accessed.
		 // 0 Select error record 0 containing errors from L1 and L2 RAMs located on the Cortex-A78AE core.
		 // 1 Select error record 1 containing errors from L3 RAMs located on the DSU-AE.
		 " orr	   %0, %0, #(1 << 0) \n"
		 " msr	   errselr_el1, %0 \n"
		 // ERR0PFGCDNR is the Cortex-A78AE node register that generates one of the errors
		 // that are enabled in the corresponding ERR0PFGCTLR register.
		 " mov	   %0, #1000 \n"
		 " msr	   s3_0_c15_c2_2, %0 \n"
	
		 // The ERR0PFGCTLR is the Cortex-A78AE node register that enables controlled fault generation.
		 " mrs	   %0, s3_0_c15_c2_1 \n"
		 // Count down enable.
		 // The value that is held in the ERR0PFGCDNR register is transferred into the Error
		 // Generation Counter. The Error Generation Counter counts down.
		 " orr	   %0, %0, #(1 << 31) \n"
		 // Uncontainable error generation enable.
		 // An uncontainable error might be generated when the Error Generation Counter is triggered.
		 " orr	   %0, %0, #(1 << 1) \n"
		 " msr	   s3_0_c15_c2_1, %0 \n"
		 "isb\n\t"
		 : "=&r" (temp)

	);
}

static void inject_error_on_each_cpu(void* info)
{
	put_cpu();
	BST_SAFETYLIB_LOG_PRINTK("[%s] inject cache error on cpu%d!", __func__, get_cpu());
	core_cache_error_inject();

	return;
}

static void safetylib_fusalsm_error_inject_callback_t(
				const uint8_t block_id,
				const uint8_t lsm_id,
				void *ext,
				const ext_info_t *info)
{
	u32 i = 0;
	BST_SAFETYLIB_LOG_PRINTK("[%s] block_id:0x%x lsm_id:0x%x !", __func__, block_id, lsm_id);
	if ((block_id == CONFIG_FUSA_INJECT_BLOCK_ID) && (lsm_id == CONFIG_FUSA_INJECT_LSM_ID)) {
		dsu_cache_error_inject();

		for_each_online_cpu(i) {
			smp_call_function_single(i, inject_error_on_each_cpu, NULL, 1);
		}
	}
}

static void safetylib_fusalsm_error_inject_sub_reply(int32_t err, void *ext, const ext_info_t* info)
{
    if (err == 0) {
        sub_success_flag = 1;
        BST_SAFETYLIB_DEBUG_PRINTK("Subscribe cache ecc error inject success. uuid : %d, timestamp : %lld\n", info->uuid, info->timestamp);
        }
    else {
        BST_SAFETYLIB_ERR_PRINTK("Subscribe cache ecc error inject fail. ret is %d.\n", err);
	}

}

static int fusa_error_inject_init(void)
{
	int ret;
	
	ret = init_sessions();
	sub_success_flag = 0;
	if (ret)
		goto exit_session;

	ret = m_client->safetylib_client.fusalsm_method_sub(safetylib_fusalsm_error_inject_callback_t, NULL, NULL, safetylib_fusalsm_error_inject_sub_reply, NULL);

//	msleep(50);
	if (ret < 0) {
		BST_SAFETYLIB_ERR_PRINTK("Subscribe message fail. ret is %d sub_success_flag:%d\n", ret, sub_success_flag);
	} else {
		BST_SAFETYLIB_ERR_PRINTK("fusa inject client init success!\n");		
	}
		
	return 0;

exit_session:
	destroy_sessions();
	return ret;
}

static void fusa_error_inject_exit(void)
{
	destroy_sessions();
}

module_init(fusa_error_inject_init);
module_exit(fusa_error_inject_exit);

MODULE_DESCRIPTION("BST-C1200 fusa error inject device driver");
MODULE_LICENSE("GPL v2");
