/*
 *-----------------------------------------------------------------------------
 * Filename: msvdx_init.c
 * $Revision: 1.29 $
 *-----------------------------------------------------------------------------
 * Copyright (c) 2002-2010, Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *-----------------------------------------------------------------------------
 * Description:
 *  Initialize the MSVDX video engine.  This loads the MTX firmware and
 *  starts a MTX thread running the firmware.
 *  The host communicates with the firmware via messages. The following
 *  messages are supported:
 *  INIT               -> MTX
 *  RENDER             -> MTX
 *  DEBLOCK            -> MTX
 *  BUBBLE             -> MTX
 *  TEST1              -> MTX
 *  TEST2              -> MTX
 *  CMD_COMPLETED      <- MTX
 *  CMD_COMPLTED_BATCH <- MTX
 *  DEBLOCK_REQUIRED   <- MTX
 *  TEST_RESPONSE      <- MTX
 *  ACK                <- MTX
 *  CMD_FAILED         <- MTX
 *-----------------------------------------------------------------------------
 */

#include <io.h>
#include <pci.h>
#include <memmap.h>
#include <sched.h>

#include <igd.h>
#include <igd_errno.h>
#include <igd_init.h>

#include <context.h>
#include <intelpci.h>
#include <general.h>
#include <utils.h>
#include <msvdx.h>

#include <plb/regs.h>
#include <plb/context.h>
#include <drm/drm.h>
#include <drm_emgd_private.h>
#include <emgd_drm.h>


struct drm_device *gpDrmDevice = NULL;
static int init_msvdx_first_time = 1;

extern void send_to_mtx(igd_context_t *context, unsigned long *init_msg);
extern int process_mtx_messages(igd_context_t *context,
		unsigned long *mtx_msgs, unsigned long mtx_msg_cnt,
		unsigned long fence);

extern void populate_fence_id(igd_context_t *context, unsigned long *mtx_msgs,
		unsigned long mtx_msg_cnt);
extern int msvdx_dequeue_send(igd_context_t *context);
extern int alloc_ramdec_region(unsigned long *base_addr0, unsigned long *base_addr1,
				unsigned long size0, unsigned long size1);

static int poll_mtx_irq(igd_context_t *context);
static int reg_ready_psb(igd_context_t *context, unsigned long reg,
        unsigned long mask, unsigned long value);
static int context_count = 0;

void msvdx_reset_plb(igd_context_t *context);

static msvdx_fw_t *priv_fw = NULL;

extern unsigned long jiffies_at_last_dequeue;
extern int mtx_message_complete;


int msvdx_query_plb(igd_context_t *context,
					unsigned long *status)
{
	platform_context_plb_t *platform;

	EMGD_TRACE_ENTER;

	platform = (platform_context_plb_t *)context->platform_context;
	*status = 0;

	if (priv_fw) {
		*status |= VIDEO_STATE_FW_LOADED;
	}

	if(!platform->rendec_base0 || !platform->rendec_base1) {
		*status |= VIDEO_STATE_RENDEC_FREED;
	}

	EMGD_TRACE_EXIT;
	return 0;
}

int msvdx_status(igd_context_t *context, unsigned long *queue_status, unsigned long *mtx_msg_status)
{
	platform_context_plb_t *platform = NULL;
	platform = (platform_context_plb_t *)context->platform_context;
	if (init_msvdx_first_time) {
		*queue_status = 1;
		*mtx_msg_status = 1;
	} else {
			*queue_status = list_empty(&platform->msvdx_queue);
			*mtx_msg_status = mtx_message_complete;
	}
	return IGD_SUCCESS;
}
int msvdx_pwr_plb(
	igd_context_t *context,
	unsigned long power_state)
{
	platform_context_plb_t *platform = (platform_context_plb_t *)context->platform_context;

	/* NOTE: The MSVDX need to reset after resume */
	EMGD_TRACE_ENTER;
	if(power_state != IGD_POWERSTATE_D0){
		platform->msvdx_needs_reset = 1;
	}

	EMGD_TRACE_EXIT;
	return IGD_SUCCESS;
}

int msvdx_pre_init_plb(struct drm_device *dev)
{
    drm_emgd_priv_t *priv;
    igd_context_t *context;

	EMGD_TRACE_ENTER;

    gpDrmDevice = dev;
	priv = gpDrmDevice->dev_private;
	context = priv->context;

	context->mod_dispatch.msvdx_pwr = msvdx_pwr_plb;
	context->mod_dispatch.msvdx_status = msvdx_status;

	EMGD_TRACE_EXIT;
	return IGD_SUCCESS;
}

int msvdx_init_plb(unsigned long base0, unsigned long base1,
		           void *msvdx_fw, unsigned long msvdx_fw_size)
{
    drm_emgd_priv_t *priv;
    igd_context_t *context;
    unsigned char *mmio;
    unsigned long ram_bank;
    unsigned long bank_size;
    unsigned long current_bank;
    unsigned long address;
    unsigned long acc_control;
    unsigned long base_addr0, base_addr1, size0, size1;
    unsigned long ram_id;
    unsigned long ctrl;
    unsigned long i;
	unsigned long fw_size;
	unsigned long *fw_data;
	msvdx_fw_t *fw = NULL;
	platform_context_plb_t *platform = NULL;

	if (!priv_fw && msvdx_fw) {
		fw = (msvdx_fw_t *) msvdx_fw;
		priv_fw = kzalloc(sizeof(msvdx_fw_t), GFP_KERNEL);
		priv_fw->fw_text_size = fw->fw_text_size;
		priv_fw->fw_data_size = fw->fw_data_size;
		priv_fw->fw_version_size = fw->fw_version_size;
		priv_fw->fw_data_location = fw->fw_data_location;

		fw_size = sizeof(unsigned long) * fw->fw_text_size;
		priv_fw->fw_text = kmalloc(fw_size, GFP_KERNEL);
		memcpy(priv_fw->fw_text, (void *) ((unsigned long) msvdx_fw) +
				((unsigned long) fw->fw_text), fw_size);

		fw_size = sizeof(unsigned long) * fw->fw_data_size;
		priv_fw->fw_data = kmalloc(fw_size, GFP_KERNEL);
		memcpy(priv_fw->fw_data, (void *) ((unsigned long) msvdx_fw) +
			((unsigned long) fw->fw_data), fw_size);

		priv_fw->fw_version = kzalloc(priv_fw->fw_version_size, GFP_KERNEL);
		strcpy(priv_fw->fw_version, (char *) (((unsigned long) msvdx_fw) +
			((unsigned long) fw->fw_version)));
	} else if (!priv_fw) {
		printk(KERN_INFO "Kernel firmware is not loaded");

		return 1;
	}

    priv = gpDrmDevice->dev_private;
    context = priv->context;
    mmio = context->device_context.virt_mmadr;

	//init_msvdx_first_time = 1;
    /* Reset MSVDX engine */
    EMGD_WRITE32(0x00000100, mmio + PSB_MSVDX_CONTROL);
    reg_ready_psb(context, PSB_MSVDX_CONTROL, 0x00000100, 0);

    /*
    * Make sure the clock is on.
    *
    * Clock enable bits are 0 - 6, with each bit controlling one of the
    * clocks.  For this, make sure all the clocks are enabled.
    */
    EMGD_WRITE32(PSB_CLK_ENABLE_ALL, mmio + PSB_MSVDX_MAN_CLK_ENABLE);

    /* Set default MMU PTD to the same value used by the SGX */
    ctrl = EMGD_READ32(mmio + PSB_CR_BIF_DIR_LIST_BASE1);

    address = EMGD_READ32(mmio + PSB_CR_BIF_DIR_LIST_BASE1);
    EMGD_WRITE32(address, mmio + PSB_MSVDX_MMU_DIR_LIST_BASE0);
    EMGD_WRITE32(address, mmio + PSB_MSVDX_MMU_DIR_LIST_BASE1);
    EMGD_WRITE32(address, mmio + PSB_MSVDX_MMU_DIR_LIST_BASE2);
    EMGD_WRITE32(address, mmio + PSB_MSVDX_MMU_DIR_LIST_BASE3);


    /*
    * MMU Page size = 12
    * MMU best count = 7
    * MMU ADT TTE = 0
    * MMU TTE threshold = 12
    */
    EMGD_WRITE32(0xc070000c, mmio + PSB_MSVDX_MMU_CONTROL1);


    /* Flush the directory cache */
    ctrl = EMGD_READ32(mmio + PSB_MSVDX_MMU_CONTROL0) | 0x0C; /* Flush */
    EMGD_WRITE32(ctrl, mmio + PSB_MSVDX_MMU_CONTROL0);

    /* Enable MMU by removing all bypass bits */
    EMGD_WRITE32(0, mmio + PSB_MSVDX_MMU_CONTROL0);


    /* Set up the RENDEC.
    *   The RENDEC requires two blocks of virtual address space so those
    *   must be allocated and then the RENDEC is initialized using those
    *   address ranges.
    *
    *   RENDEC control0:
    *     bit 3     1 - search MTX_to_MTX header
    *     bit 2     1 - skip next slice
    *     bit 1     1 - flush remaining bit stream
    *     bit 0     1 - initialize RENDEC
    *
    *   RENDEC control1:
    *     bit 24:   1 - enables data to be transferred through ext. memory
    *     bit 19:18 WR burst size (0 = 32 bytes, 1 = 64 bytes, 2 = 128 bytes)
    *     bit 17:16 RD burst size (0 = 32 bytes, 1 = 64 bytes, 2 = 128 bytes)
    *     bit  7: 0 start size (zero)
    */

    size0 = RENDEC_A_SIZE;
    size1 = RENDEC_B_SIZE;

    /*
    * These allocations need to be undone when shutting down.  Where
    * should they be saved?
    */
    if (init_msvdx_first_time) {
		base_addr0 = base0;
		base_addr1 = base1;

		//printk(KERN_INFO "get the base_addr=%lx, base_addr1=%lx\n", base_addr0,base_addr1);

        /* Save the offsets so it can be freed and restored later */
        platform = (platform_context_plb_t *)context->platform_context;
        platform->rendec_base0 = base_addr0;
        platform->rendec_base1 = base_addr1;

		init_msvdx_first_time = 0;
	INIT_LIST_HEAD(&platform->msvdx_queue);
		spin_lock_init(&platform->msvdx_lock);
    } else {
        /* restore offsets. */
        platform = (platform_context_plb_t *)context->platform_context;
        base_addr0 = platform->rendec_base0;
        base_addr1 = platform->rendec_base1;

        /* Init link list */
        if(!context_count) {
			INIT_LIST_HEAD(&platform->msvdx_queue);
		}
    }

	platform->msvdx_busy = 0;
    EMGD_WRITE32(base_addr0, mmio + PSB_MSVDX_RENDEC_BASE_ADDR0);
    EMGD_WRITE32(base_addr1, mmio + PSB_MSVDX_RENDEC_BASE_ADDR1);

    EMGD_WRITE32((((size1 / 4096) << 16) | (size0 / 4096)),
            mmio + PSB_MSVDX_RENDEC_BUFFER_SIZE);

    /* Rendec setup:
    *   Start size = 0
    *   Burst size R = 4 words
    *   Burst size W = 4 words
    *   External memory enabled
    *   Stream End = 0
    *   Slice mode = 0
    *   DEC disable = 0
    */
    EMGD_WRITE32(0x01050000, mmio + PSB_MSVDX_RENDEC_CONTROL1);

    EMGD_WRITE32(0x00101010, mmio + PSB_MSVDX_RENDEC_CONTEXT0);
    EMGD_WRITE32(0x00101010, mmio + PSB_MSVDX_RENDEC_CONTEXT1);
    EMGD_WRITE32(0x00101010, mmio + PSB_MSVDX_RENDEC_CONTEXT2);
    EMGD_WRITE32(0x00101010, mmio + PSB_MSVDX_RENDEC_CONTEXT3);
    EMGD_WRITE32(0x00101010, mmio + PSB_MSVDX_RENDEC_CONTEXT4);
    EMGD_WRITE32(0x00101010, mmio + PSB_MSVDX_RENDEC_CONTEXT5);

    EMGD_WRITE32(0x00000001, mmio + PSB_MSVDX_RENDEC_CONTROL0);


    /* Start Firmware Load process */

    /* Reset the MTX */
    EMGD_WRITE32(0x00000001, mmio + PSB_MSVDX_MTX_SOFT_RESET);

	/* Reset the counter that looks for MSVDX getting into a bad state */
	jiffies_at_last_dequeue = 0;

    /*
    * Should this check the core revision and only do this if it is
    * a specific version or range of versions?
    *
    * Stepping prior to D0, need to set COMMS_OFFSET_FLAGS to 0
    * Stepping D0 should set MSVDX_DEVICE_NODE_FLAGS_DEFAULT_D0 (0x222)
    * Stepping D1 should set MSVDX_DEVICE_NODE_FLAGS_DEFAULT_D1 (0x220)
    */

    /* If POULSBO_D1 or later use MSVDX_DEVICE_NODE_FLAGS_MMU_HW_INVALIDATION */
    EMGD_WRITE32(MSVDX_DEVICE_NODE_FLAGS_DEFAULT,
            mmio + PSB_MSVDX_COMMS_OFFSET_FLAGS);
    /* Else EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_OFFSET_FLAGS); */

    /* Initialize communication control */
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_MSG_COUNTER);
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_SIGNATURE);
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_TO_HOST_RD_INDEX);
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_TO_HOST_WRT_INDEX);
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_TO_MTX_RD_INDEX);
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_TO_MTX_WRT_INDEX);
    EMGD_WRITE32(0x00, mmio + PSB_MSVDX_COMMS_FW_STATUS);

    /*
    * Get the ram bank size
    *   The banks size seems to be a 4 bit value in the MTX debug register.
    *   Where this is documented other than the UMG code is not clear.
    */
    ram_bank = EMGD_READ32(mmio + PSB_MSVDX_MTX_RAM_BANK);
    bank_size = (ram_bank & 0x000f0000) >> 16;
    bank_size = (1 << (bank_size + 2));

    /* Firmware version? */
    printk(KERN_INFO "Firmware version is %s\n", priv_fw->fw_version);

    /* Save RAM access control register */
    acc_control = EMGD_READ32(mmio + PSB_MSVDX_MTX_RAM_ACCESS_CONTROL);

    /* Loop writing text/code to core memory */
    current_bank = ~0L;
    address = PC_START_ADDRESS - MTX_CODE_BASE;

	fw_data = priv_fw->fw_text;
	fw_size = priv_fw->fw_text_size;

    for (i = 0; i < fw_size; i++) {
        /* Wait for MCMSTAT to become be idle 1 */
        if (reg_ready_psb(context, PSB_MSVDX_MTX_RAM_ACCESS_STATUS,
                    0xffffffff, 0x00000001) == 0) {
            ram_id = MTX_CORE_CODE_MEM + (address / bank_size);
            if (ram_id != current_bank) {
                /*
                * bits 20:27    - ram bank (CODE_BASE | DATA_BASE)
                * bits  2:19    - address
                * bit   1       - enable auto increment addressing mode
                */
                ctrl = (ram_id << 20) | (((address >> 2) & 0x000ffffc) << 2) |
                    0x02;
                EMGD_WRITE32(ctrl, mmio + PSB_MSVDX_MTX_RAM_ACCESS_CONTROL);

                current_bank = ram_id;
                /* Wait for MCMSTAT to become be idle 1 */
                reg_ready_psb(context, PSB_MSVDX_MTX_RAM_ACCESS_STATUS,
                        0xffffffff, 0x00000001);
            }

            address +=  4;
            EMGD_WRITE32(fw_data[i],
                    mmio + PSB_MSVDX_MTX_RAM_ACCESS_DATA_TRANSFER);
        } else {
            printk(KERN_ERR "Timeout waiting for MCMSTAT to be idle\n");
        }
    }

    /* verify firmware upload. */
    current_bank = ~0L;
    address = PC_START_ADDRESS - MTX_CODE_BASE;

    for (i = 0; i < fw_size; i++) {
        if (reg_ready_psb(context, PSB_MSVDX_MTX_RAM_ACCESS_STATUS,
                    0xffffffff, 0x00000001) == 0) {
            ram_id = MTX_CORE_CODE_MEM + (address / bank_size);
            if (ram_id != current_bank) {
                /*
                * bits 20:27    - ram bank (CODE_BASE | DATA_BASE)
                * bits  2:19    - address
                * bit   1       - enable auto increment addressing mode
                */
                ctrl = (ram_id << 20) | (((address >> 2) & 0x000ffffc) << 2) |
                    0x03;
                EMGD_WRITE32(ctrl, mmio + PSB_MSVDX_MTX_RAM_ACCESS_CONTROL);
                current_bank = ram_id;
                reg_ready_psb(context, PSB_MSVDX_MTX_RAM_ACCESS_STATUS,
                        0xffffffff, 0x00000001);
            }

            address +=  4;
            if (EMGD_READ32(mmio + PSB_MSVDX_MTX_RAM_ACCESS_DATA_TRANSFER) !=
                    fw_data[i]) {
                printk(KERN_ERR "Verify Error at index %ld\n", i);
            }
        } else {
            printk(KERN_ERR "Timeout waiting for MCMSTAT to be idle while verifying\n");
        }
    }

	fw_data = priv_fw->fw_data;
	fw_size = priv_fw->fw_data_size;

    /* Loop writing data to core memory */
    current_bank = ~0L;
    address = priv_fw->fw_data_location - MTX_DATA_BASE;

    for (i = 0; i < fw_size; i++) {
        if (reg_ready_psb(context, PSB_MSVDX_MTX_RAM_ACCESS_STATUS,
                    0xffffffff, 0x00000001) == 0) {
            ram_id = MTX_CORE_DATA_MEM + (address / bank_size);
            if (ram_id != current_bank) {
                /*
                * bits 20:27    - ram bank (CODE_BASE | DATA_BASE)
                * bits  2:19    - address
                * bit   1       - enable auto increment addressing mode
                */
                ctrl = (ram_id << 20) | (((address >> 2) & 0x000ffffc) << 2) |
                    0x02;
                EMGD_WRITE32(ctrl, mmio + PSB_MSVDX_MTX_RAM_ACCESS_CONTROL);
                current_bank = ram_id;
                reg_ready_psb(context, PSB_MSVDX_MTX_RAM_ACCESS_STATUS,
                        0xffffffff, 0x00000001);
            }

            address +=  4;
            EMGD_WRITE32(fw_data[i],
                    mmio + PSB_MSVDX_MTX_RAM_ACCESS_DATA_TRANSFER);
        } else {
            printk(KERN_ERR "Timeout waiting for MCMSTAT to be idle - data segment\n");
        }
    }

    /* Restore the RAM access control register */
    EMGD_WRITE32(acc_control, mmio + PSB_MSVDX_MTX_RAM_ACCESS_CONTROL);

    /* Start the firmware thread running */
    EMGD_WRITE32(PC_START_ADDRESS, mmio + PSB_MSVDX_MTX_REGISTER_READ_WRITE_DATA);
    EMGD_WRITE32(MTX_PC, mmio + PSB_MSVDX_MTX_REGISTER_READ_WRITE_REQUEST);
    reg_ready_psb(context, PSB_MSVDX_MTX_REGISTER_READ_WRITE_REQUEST,
            0x80000000, 0x80000000);

    /* Enable the MTX */
    printk(KERN_INFO "Enabling MTX 0x%x\n", EMGD_READ32(mmio + PSB_MSVDX_MTX_ENABLE));
    EMGD_WRITE32(MSVDX_MTX_ENABLE_MTX_ENABLE_MASK, mmio + PSB_MSVDX_MTX_ENABLE);
    printk(KERN_INFO "Enabled MTX 0x%x\n", EMGD_READ32(mmio + PSB_MSVDX_MTX_ENABLE));

    /*
    * Wait for signature value to be written.
    *
    * This is how the firmware thread notifies us that it is running.
    */
    if (reg_ready_psb(context, PSB_MSVDX_COMMS_SIGNATURE, 0xffffffff,
                0xA5A5A5A5)){
        /* Error initializing firmware.... */
        EMGD_DEBUG("Error, no MSVDX COMMS Signature");
        return 0; /* FIXME: return an error code */
    }
    printk(KERN_INFO "MSVDX COMMS Signature OK\n");

    /* Locate message buffers */
    platform->mtx_buf_size = EMGD_READ32(mmio+PSB_MSVDX_COMMS_TO_MTX_BUF_SIZE) & 0xFFFF;
    platform->host_buf_size = EMGD_READ32(mmio+PSB_MSVDX_COMMS_TO_HOST_BUF_SIZE) & 0xFFFF;
    platform->mtx_buf_offset = MSVDX_BASE + (EMGD_READ32(mmio+PSB_MSVDX_COMMS_TO_MTX_BUF_SIZE) >> 16) + 0x2000;
    platform->host_buf_offset = MSVDX_BASE + (EMGD_READ32(mmio+PSB_MSVDX_COMMS_TO_HOST_BUF_SIZE) >> 16) + 0x2000;

    platform->sequence = 1;
    platform->mtx_submitted = 0;

    /* Send initialization message to firmware, newer versions don't */
    if (0) {
        unsigned long init_msg[2];

        init_msg[0] = 8 | (0x80 << 8);

        /* physical address of the PD shared by SGX/MSVDX */
        init_msg[1] = EMGD_READ32(mmio + 0x40c84);

        send_to_mtx(context, init_msg);

        /* Check response from MTX firmware */
        poll_mtx_irq(context);
    }

    /* Clear the firmware buffer, this is mostly to make debugging easier */
    if (1) {
        unsigned long i;

        for (i = 0; i < platform->mtx_buf_size; i++) {
            EMGD_WRITE32(0, mmio + platform->mtx_buf_offset + (i <<2));
        }
        for (i = 0; i < platform->host_buf_size; i++) {
            EMGD_WRITE32(0, mmio + platform->host_buf_offset + (i <<2));
        }
    }


    /* Enable minimal clocks */
    EMGD_WRITE32(PSB_CLK_ENABLE_MIN, mmio + PSB_MSVDX_MAN_CLK_ENABLE);

    /* Enable MTX interrupts to host */
    EMGD_WRITE32(1<<14, mmio + PSB_MSVDX_HOST_INTERRUPT_ENABLE);


    /* Are we done? */
    EMGD_TRACE_EXIT;
    return 0; /* Successfully initialized the MTX firmware */
}


int msvdx_uninit_plb(igd_context_t *context)
{
	EMGD_TRACE_ENTER;

	if(!context_count) {
		msvdx_reset_plb(context);
	}
	EMGD_TRACE_EXIT;
	return 0;
}


int msvdx_close_context(igd_context_t *context)
{
	EMGD_TRACE_ENTER;

	if(context_count) {
		context_count -= 1;
	} else {
		EMGD_TRACE_EXIT;
		return 1;
	}

	EMGD_TRACE_EXIT;
	return 0;
}

int msvdx_create_context(igd_context_t *context)
{
	EMGD_TRACE_ENTER;

	context_count +=1;

	EMGD_TRACE_EXIT;
	return 0;
}

int process_video_decode_plb(igd_context_t *context, unsigned long offset, void *virt_addr, unsigned long *fence_id)
{
	unsigned long *mtx_buf;
    unsigned long *mtx_msgs;
    unsigned long mtx_offset;
    unsigned long mtx_msg_cnt;
    unsigned long irq_flags;
	int ret = 0;
    platform_context_plb_t *platform;

    EMGD_TRACE_ENTER;


    platform = (platform_context_plb_t *)context->platform_context;

	mtx_buf = (unsigned long *) virt_addr;
    mtx_offset = mtx_buf[0];
    mtx_msg_cnt = mtx_buf[1];

//	printk(KERN_INFO "process_video_decode_plb where buf=%lx, offset=%lx, cnt=%lx\n",
//				mtx_buf, offset, mtx_msg_cnt);
    if (mtx_msg_cnt > 0x20) {
        printk(KERN_ERR "Message count too big at %ld\n", mtx_msg_cnt);
        return -EINVAL;
    }

    mtx_msgs = mtx_buf + (mtx_offset / sizeof (unsigned long));
	if (mtx_msg_cnt > 0) {
	//if ((mtx_buf[0] != 0x8) || (mtx_buf[2] != 0x8504)) {

		spin_lock_irqsave(&platform->msvdx_lock, irq_flags);

		if (!platform->msvdx_busy) {

			platform->msvdx_busy = 1;
			spin_unlock_irqrestore(&platform->msvdx_lock, irq_flags);


			if (platform->msvdx_needs_reset) {
				msvdx_reset_plb(context);
				msvdx_init_plb(0, 0, NULL, 0);
				jiffies_at_last_dequeue = 0;
			}
			// Send message buffer to MSVDX Firmware

			populate_fence_id(context, mtx_msgs, mtx_msg_cnt);
			ret = process_mtx_messages(context, mtx_msgs, mtx_msg_cnt, platform->msvdx_fence);

			if (ret) {
				ret = -EINVAL;

			}
		} else {
			struct msvdx_cmd_queue *msvdx_cmd;

			spin_unlock_irqrestore(&platform->msvdx_lock, irq_flags);

			msvdx_cmd = kzalloc(sizeof(struct msvdx_cmd_queue), GFP_KERNEL);
			if (msvdx_cmd == NULL) {
				printk(KERN_ERR "MSVDXQUE: Out of memory\n");
				return -ENOMEM;
			}

			populate_fence_id(context, mtx_msgs, mtx_msg_cnt);
			msvdx_cmd->cmd = mtx_msgs;
			msvdx_cmd->cmd_size = mtx_msg_cnt;
			/* If more than 1000 msec (1 second or 1000 jiffies) passes since
			 * the last time a video cmd has been decoded, MSVDX may be hung
			 * and needing to be reset.
			 */
			if ((jiffies_at_last_dequeue != 0) &&
				((jiffies - jiffies_at_last_dequeue) > 1000)) {
				printk(KERN_ERR "Video decode hardware appears to be hung; "
					"resetting\n");
				platform->msvdx_needs_reset = 1;
			}
			if (platform->msvdx_needs_reset) {
				msvdx_reset_plb(context);
				msvdx_init_plb(0, 0, NULL, 0);
				platform->msvdx_busy = 0;
				jiffies_at_last_dequeue = 0;
			}

			spin_lock_irqsave(&platform->msvdx_lock, irq_flags);
			list_add_tail(&msvdx_cmd->head, &platform->msvdx_queue);
			if (!platform->msvdx_busy) {
				platform->msvdx_busy = 1;
				msvdx_dequeue_send(context);
			}

			spin_unlock_irqrestore(&platform->msvdx_lock, irq_flags);

		}
		*fence_id = platform->msvdx_fence;
	} else {
		/* return the fence id even there is no messages to process.
		 * Used this for context id.
		 */
		*fence_id = platform->msvdx_fence;
	}

	return ret;
}

int msvdx_get_fence_id(igd_context_t *context, unsigned long *fence_id)
{
	int ret = 0;
    platform_context_plb_t *platform;

    platform = (platform_context_plb_t *)context->platform_context;

	*fence_id = platform->mtx_completed;

	return ret;
}

int msvdx_flush_tlb(igd_context_t *context)
{
	unsigned char *mmio = context->device_context.virt_mmadr;
	unsigned long msvdx_mmu;
	msvdx_mmu = EMGD_READ32(mmio + PSB_MSVDX_MMU_CONTROL0);
	msvdx_mmu &= 0xFFFFFFF0;
	msvdx_mmu |= 0x0C; 	/* MMU_INVALDC + MMU_FLUSH */
	EMGD_WRITE32(msvdx_mmu, mmio + PSB_MSVDX_MMU_CONTROL0);

	msvdx_mmu = EMGD_READ32(mmio + PSB_MSVDX_MMU_CONTROL0);
	msvdx_mmu &= 0xFFFFFF00;
	EMGD_WRITE32(msvdx_mmu, mmio + PSB_MSVDX_MMU_CONTROL0);
	EMGD_READ32(mmio + PSB_MSVDX_MMU_CONTROL0);

	return 0;

}


/*
 * Resets the MSVDX engine via the soft reset control.
 *
 * This function is exported.
 */
void msvdx_reset_plb(igd_context_t *context)
{
    unsigned char *mmio = context->device_context.virt_mmadr;
    platform_context_plb_t *platform;

    EMGD_TRACE_ENTER;

    platform = (platform_context_plb_t *)context->platform_context;

    /* Reset MSVDX engine */
    EMGD_WRITE32(0x11111100, mmio + PSB_MSVDX_CONTROL);
    reg_ready_psb(context, PSB_MSVDX_CONTROL, 0x00000100, 0);

    /* Clear interrupt and clear pending interrupts */
    EMGD_WRITE32(0, mmio + PSB_MSVDX_HOST_INTERRUPT_ENABLE);
    EMGD_WRITE32(0xffffffff, mmio + PSB_MSVDX_INTERRUPT_CLEAR);

    /* Mark the engine as being reset */
    platform->msvdx_needs_reset = 0;
}


/*
 * When shuting down, need to reset the MSVDX engine too.
 */
int msvdx_shutdown_plb(igd_context_t *context)
{
	platform_context_plb_t *platform;

	EMGD_TRACE_ENTER;
	platform = (platform_context_plb_t *)context->platform_context;

	/* Reset MSVDX engine */
	msvdx_reset_plb(context);

	/* Free RENDEC memory allocations */
	platform->rendec_base0 = 0;
	platform->rendec_base1 = 0;
	init_msvdx_first_time = 1;

	EMGD_TRACE_EXIT;
	return 0;
}

static int reg_ready_psb(igd_context_t *context,
        unsigned long reg,
        unsigned long mask,
        unsigned long value)
{
    unsigned char *mmio = context->device_context.virt_mmadr;
    unsigned long status;
    int poll_cnt = 1000;

    while (poll_cnt) {
        status = EMGD_READ32(mmio + reg);
        if ((status & mask) == value) {
            return 0;
        }
        poll_cnt--;
        OS_SLEEP(100);
    }

    /* Timeout waiting for RAM ACCESS ready */
    EMGD_DEBUG("TIMEOUT: Got 0x%08lx while waiting for 0x%08lx", status, value);
    return 1;
}


static int poll_mtx_irq(igd_context_t *context)
{
    unsigned char *mmio = context->device_context.virt_mmadr;
    int ret;
    unsigned long mtx_int;

    EMGD_TRACE_ENTER;
    mtx_int = (1 << 14);

    ret = reg_ready_psb(context, PSB_MSVDX_INTERRUPT_STATUS, mtx_int, mtx_int);
    if (ret) {
        /* Timeout waiting on interrupt status */
        return ret;
    }

    /* Clear the interrupt */
    EMGD_WRITE32(mtx_int, mmio + PSB_MSVDX_INTERRUPT_CLEAR);

    return ret;
}

