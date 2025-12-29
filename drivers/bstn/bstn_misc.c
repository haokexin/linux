// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bstn_misc.c
 * @brief   This file is the source code file of misc device interface of BSTN
 *          driver. It contains function definitions of ioctl callbacks and
 *          initialization of the misc device.
 */

#include "bstn.h"

#define BSTN_DEV_ID_LEN 5

/*******************************************************************************
 * BSTN MISC IOCTL INTERFACE
 ******************************************************************************/
/*
 * @func    bstn_ioctl_buf_alloc
 * @brief   This function allocates a continuous memory buffer requested by the
 *          user.
 * @params  filp - the file pointer to the misc device
 *          pbstn - the pointer to the BSTN device
 *          pbuffer - the user pointer to the buffer metadata prefilled with
 *          requested size
 * @return  0 - success
 *          error code - failure
 */
static int bstn_ioctl_buf_alloc(struct file *filp, struct bstn_device *pbstn,
				struct bsnn_buffer __user *pbuffer)
{
	struct bsnn_buffer buffer;
	int ret;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);

	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}
	if (buffer.size == 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "zero size buffer!");
		return -EINVAL;
	}

	ret = bsnn_buffer_alloc(filp, pbstn, &buffer);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "user buffer allcoation failed!");
		return ret;
	}

	if (copy_to_user(pbuffer, &buffer, sizeof(buffer))) {
		bsnn_buffer_free(filp, pbstn, &buffer);
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_to_user failed!");
		return ret;
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*
 * @func    bstn_ioctl_buf_free
 * @brief   This function frees the continuous memory buffer specified in the
 *          user request.
 * @params  pbstn - the pointer to the BSTN device
 *          pbuffer - the user pointer to the metadata of the buffer to be freed
 * @return  0 - success
 *          error code - failure
 */
static int bstn_ioctl_buf_free(struct file *filp, struct bstn_device *pbstn,
			       struct bsnn_buffer __user *pbuffer)
{
	struct bsnn_buffer buffer;
	int ret;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);

	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("buffer->handle: %px", buffer.handle);

	ret = bsnn_buffer_free(filp, pbstn, &buffer);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "user buffer free failed!");
		return ret;
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*
 * @func    bstn_ioctl_buf_sync
 * @brief   This function synchronizes the specified continuous memory buffer by
 *          flushing its cache.
 * @params  pbstn - the pointer to the BSTN device
 *          pbuffer - the user pointer to the metadata of the buffer to be
 *          synchronized
 * @return  0 - success
 *          error code - failure
 */
static int bstn_ioctl_buf_sync(struct file *filp, struct bstn_device *pbstn,
			       struct bsnn_buffer __user *pbuffer)
{
	int ret;
	struct bsnn_buffer buffer;
	dma_addr_t dma_addr;
	struct bstn_mem_manager *pmman;
	struct device *pdev = &pbstn->pdev->dev;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);

	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BSTN_DEV_ERR(pdev, "copy_from_user failed");
		return -EFAULT;
	}

	pmman = &pbstn->mem_manager;

	if (pmman->enable_smmu) {
		ret = bstn_dma_buf_flush(filp, pbstn, &buffer);
		if (ret != 0) {
			BSTN_DEV_ERR(pdev, "dma buf flush failed!");
			return -EFAULT;
		}
		return 0;
	} else {
		dma_addr = bus_to_dma(buffer.baddr);
		dma_sync_single_for_device(pmman->pdev, dma_addr, buffer.size,
					   DMA_TO_DEVICE);
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*
 * @func    bstn_ioctl_buf_invalidate
 * @brief   This function synchronizes the specified continuous memory buffer by
 *          invalidate cache so CPU can get data same to memory for the following
 *          read.
 * @params  pbstn - the pointer to the BSTN device
 *          pbuffer - the user pointer to the metadata of the buffer to be
 *          synchronized
 * @return  0 - success
 *          error code - failure
 */
static int bstn_ioctl_buf_invalidate(struct file *filp,
				     struct bstn_device *pbstn,
				     struct bsnn_buffer __user *pbuffer)
{
	int ret;
	struct bsnn_buffer buffer;
	dma_addr_t dma_addr;
	struct bstn_mem_manager *pmman;
	struct device *pdev = &pbstn->pdev->dev;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);

	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BSTN_DEV_ERR(pdev, "copy_from_user failed");
		return -EFAULT;
	}

	pmman = &pbstn->mem_manager;

	if (pmman->enable_smmu) {
		ret = bstn_dma_buf_invalidate(filp, pbstn, &buffer);
		if (ret != 0) {
			BSTN_DEV_ERR(pdev, "dma buf invalidate failed!");
			return -EFAULT;
		}
		return 0;
	} else {
		dma_addr = bus_to_dma(buffer.baddr);
		dma_sync_single_for_cpu(pmman->pdev, dma_addr, buffer.size,
					DMA_FROM_DEVICE);
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*
 * @func    bstn_ioctl_buf_sync_offset
 * @brief   This function synchronizes the specified continuous memory buffer by
 *          flushing its cache.
 * @params  pbstn - the pointer to the BSTN device
 *          pbuffer - the user pointer to the metadata of the buffer to be
 *          synchronized
 * @return  0 - success
 *          error code - failure
 */
static int bstn_ioctl_buf_sync_offset(struct file *filp,
				      struct bstn_device *pbstn,
				      struct bstnpu_mem_sync __user *pbuffer)
{
	int ret;
	struct bstnpu_mem_sync buffer;
	dma_addr_t dma_addr;
	struct bstn_mem_manager *pmman;
	struct device *pdev = &pbstn->pdev->dev;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);

	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BSTN_DEV_ERR(pdev, "copy_from_user failed");
		return -EFAULT;
	}

	pmman = &pbstn->mem_manager;

	if (pmman->enable_smmu) {
		ret = bstn_dma_buf_sync(filp, pbstn, &buffer);
		if (ret != 0) {
			BSTN_DEV_ERR(pdev, "dma buf flush failed!");
			return -EFAULT;
		}
		return 0;
	} else {
		dma_addr = bus_to_dma(buffer.baddr);
		if (buffer.flags & BSTN_MEM_SYNC_TO_DEVICE) {
			dma_sync_single_for_device(pmman->pdev,
						   dma_addr + buffer.offset,
						   buffer.size, DMA_TO_DEVICE);
		}
		if (buffer.flags & BSTN_MEM_SYNC_FROM_DEVICE) {
			dma_sync_single_for_cpu(pmman->pdev,
						dma_addr + buffer.offset,
						buffer.size, DMA_FROM_DEVICE);
		}
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*
 * @func    bstn_ioctl_msg_send
 * @brief   This function sends the BSTN message to the DSP. It is a blocking
 *          function which has to wait for the response from the DSP or timeout.
 * @params  pbstn - the pointer to the BSTN device
 *          pexchange - the user pointer to the preallocated message exchange
 *          structure
 * @return  0 - success
 *          error code  - failure
 */
static int bstn_ioctl_msg_send(struct bstn_device *pbstn,
			       struct bsnn_msg_exchange __user *pexchange)
{
	int ret;
	struct bsnn_msg_exchange exchange;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pexchange);
	if (copy_from_user(&exchange.target_net, &pexchange->target_net,
			   sizeof(exchange.target_net))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user exchange pointer!");
		return -EFAULT;
	}
	BSTN_TRACE_PRINTK("target net %d", exchange.target_net);
	if (copy_from_user(&exchange.req, &pexchange->req,
			   sizeof(exchange.req))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user exchange pointer!");
		return -EFAULT;
	}

	ret = bstn_msg_exchange(pbstn, &exchange);
	// error code
	if (ret < 0) {
		BSTN_TRACE_PRINTK("bstn_msg_exchange error: %d", ret);
		return ret;
	}
	// time out
	else if (ret == 0) {
		BSTN_STAGE_PRINTK(
			"bstn_msg_exchange time out, it'll reset NET DSP when next time driver gets opened");
		bstn_soft_reset = 1;
		return -ENOMSG;
	}

	if (copy_to_user(&pexchange->rsp, &exchange.rsp,
			 sizeof(exchange.rsp))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user response pointer!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*
 * @func    bstn_ioctl_ver_get
 * @brief   This function gets versions and release dates of the BSTN kernel
 *          driver and runtime firmware.
 *          pinfo - the user pointer to the version information structure
 * @return  0 - success
 *          error code  - failure
 */
static int bstn_ioctl_ver_get(struct bstn_device *pbstn,
			      struct bstn_ver_info __user *pinfo)
{
	struct bstn_ver_info info;

	BSTN_TRACE_PRINTK("enter, user ptr: %px", pinfo);

	info.driver_ver_major = BSTN_VER_MAJOR;
	info.driver_ver_minor = BSTN_VER_MINOR;
	info.driver_ver_patch = BSTN_VER_PATCH;
	info.driver_release_date = BSTN_RELEASE_DATE;
	info.driver_release_month = BSTN_RELEASE_MONTH;
	info.driver_release_year = BSTN_RELEASE_YEAR;
	info.fw_ver_major = pbstn->fw_manager.ver_major;
	info.fw_ver_minor = pbstn->fw_manager.ver_minor;
	info.fw_ver_patch = pbstn->fw_manager.ver_patch;
	info.fw_release_date = pbstn->fw_manager.release_date;
	info.fw_release_month = pbstn->fw_manager.release_month;
	info.fw_release_year = pbstn->fw_manager.release_year;

	if (copy_to_user(pinfo, &info, sizeof(info))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user response pointer!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

/*!
 * @brief           This function imports a dma-buf.
 *
 * @param[in]       pbstn      The bstn driver
 * @param[in,out]   buf        The user pointer to the imported dma-buf information
 *
 * @return          0 - success
 *                  Error code - failure
 */
static int bstn_ioctl_dma_buf_import(struct file *filp,
				     struct bstn_device *pbstn,
				     struct bstn_dma_buf __user *buf)
{
	struct bstn_dma_buf buffer;
	int ret;

	BSTN_TRACE_PRINTK("enter import");

	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("fd: %d", buffer.fd);
	ret = bstn_dma_buf_import(filp, pbstn, &buffer);
	if (ret != 0) {
		return ret;
	} else {
		BSTN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);
	}

	ret = copy_to_user(buf, &buffer, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_to_user failed!");
		bstn_dma_buf_return(filp, pbstn, &buffer);
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit import");
	return ret;
}

/*!
 * @brief       This function returns a dma-buf.
 *
 * @param[in]   pbstn  The bstn driver
 * @param[in]   buf            The user pointer to the returned dma-buf information
 *
 * @return      0 - success
 *              Error code - failure
 */
static int bstn_ioctl_dma_buf_return(struct file *filp,
				     struct bstn_device *pbstn,
				     struct bstn_dma_buf __user *buf)
{
	struct bstn_dma_buf buffer;
	int ret;

	BSTN_TRACE_PRINTK("enter return, user ptr: %px", buf);

	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}
	BSTN_TRACE_PRINTK("fd: %d", buffer.fd);

	ret = bstn_dma_buf_return(filp, pbstn, &buffer);

	BSTN_TRACE_PRINTK("exit return");
	return ret;
}

static int bstn_ioctl_cma_buf_import(struct file *filp,
				     struct bstn_device *pbstn,
				     struct bstn_cma_buf __user *buf)
{
	struct bstn_cma_buf buffer;
	int ret;

	BSTN_TRACE_PRINTK("enter import");
	if (!pbstn->mem_manager.enable_smmu) {
		buf->bus_addr = phys_to_bus(buf->pa);
		BSTN_TRACE_PRINTK("exit import, smmu off");
		return 0;
	}

	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("pa: 0x%llx", buffer.pa);
	ret = bstn_cma_buf_import(filp, pbstn, &buffer);
	if (ret != 0) {
		return ret;
	} else {
		BSTN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);
	}

	ret = copy_to_user(buf, &buffer, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_to_user failed!");
		bstn_cma_buf_return(filp, pbstn, &buffer);
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit import");
	return ret;
}

static int bstn_ioctl_cma_buf_return(struct file *filp,
				     struct bstn_device *pbstn,
				     struct bstn_cma_buf __user *buf)
{
	struct bstn_cma_buf buffer;
	int ret;

	BSTN_TRACE_PRINTK("enter return, user ptr: %px", buf);
	if (!pbstn->mem_manager.enable_smmu) {
		return 0;
	}

	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}
	BSTN_TRACE_PRINTK("pa: 0x%llx", buffer.pa);

	ret = bstn_cma_buf_return(filp, pbstn, &buffer);

	BSTN_TRACE_PRINTK("exit return");
	return ret;
}

/*!
 * @brief           This function imports a dma-buf.
 * @param[in]       pbstn      The bstn driver
 * @param[in,out]   buf                The user pointer to the imported dma-buf information
 * @return          0 - success
 *                  Error code - failure
 */
static int bstn_ioctl_dma_buf_export(struct file *filp,
				     struct bstn_device *pbstn,
				     struct bstn_dma_buf __user *buf)
{
	int ret;
	struct bstn_dma_buf buffer;
	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_from_user failed!");
		return -EFAULT;
	}

	ret = bstn_dma_buf_export(filp, pbstn, &buffer);
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "dma buf export failed!");
		return -EFAULT;
	}
	BSTN_TRACE_PRINTK("exp fd: %d", buffer.fd);

	ret = copy_to_user(buf, &buffer, sizeof(buffer));
	if (ret != 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "copy_to_user failed!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit export");

	return ret;
}

/*
 * @func    bstn_ioctl_perf_get
 * @brief   This function gets the DAG cycle count from NETs.
 *          pinfo - the user pointer to the version information structure
 * @return  0 - success
 *          error code  - failure
 */
static int bstn_ioctl_perf_get(struct bstn_device *pbstn,
			       struct bstn_perf_info __user *pinfo)
{
	struct bstn_perf_info info = { 0 };
	// void __iomem *net_core_glb = NULL;

	BSTN_TRACE_PRINTK("enter ioctl perf get, user ptr: %px", pinfo);
#if 0
	// NET TOP
	net_core_glb = ioremap(BST_NET_CORE_GLOBAL_REG, 0x30);
	info.dag_cnt_top = readl_relaxed(net_core_glb + BST_NET_PERF_CNT_0);
	// NET LITE
	net_core_glb =
	    ioremap(BST_NET_CORE_GLOBAL_REG + BST_NET_LITE_OFFSET, 0x30);
	info.dag_cnt_lite = readl_relaxed(net_core_glb + BST_NET_PERF_CNT_0);
#endif
	if (copy_to_user(pinfo, &info, sizeof(info))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user response pointer!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit ioctl perf get");
	return 0;
}

/*
 * @func    bstn_ioctl_asic_type_get
 * @brief   This function gets the ASIC chip type..
 *          pinfo - the user pointer to the ASIC chip type string
 * @return  0 - success
 *          error code  - failure
 */
static int bstn_ioctl_asic_type_get(struct bstn_device *pbstn,
				    char __user *pinfo)
{
	char info[32] = { 0 }; // be sure the size is big enough to hold type

	BSTN_TRACE_PRINTK("enter %s, user ptr: %px", __func__, pinfo);

	strscpy(info, "C1200", sizeof(info));

	if (copy_to_user(pinfo, &info, sizeof(info))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user response pointer!");
		return -EFAULT;
	}

	BSTN_TRACE_PRINTK("exit");
	return 0;
}

#if 0 //unused
/*
 * @func    bstn_ioctl_bister_start
 * @brief   This function gets the flag for the bister thread to start
 *          pinfo - the user pointer to the start flag
 * @return  0 - success
 *          error code  - failure
 */
static int bstn_ioctl_bister_start(struct bstn_device *pbstn,
				   int32_t __user * pstart)
{
	int32_t *bist_start = &pbstn->msg_manager.bist_thread_start, ret;
	if (copy_from_user(bist_start, pstart, sizeof(*bist_start))) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "invalid user bist thread start flag!");
		return -EFAULT;
	}
	if (pbstn->msg_manager.msg_sw_bister_task != NULL) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "bist thread is running");
	} else if (*bist_start) {
		ret = bstn_msg_start_sw_bist_thread(pbstn);
		if (ret != 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "start sw bist failed error: %d", ret);
		}
		BSTN_TRACE_PRINTK("ioctl bister start");
	}

	return 0;
}
#endif

/*
 * @func    bstn_ioctl
 * @brief   This function is the top-level BSTN ioctl interface.
 * @params  filp - file pointer to the misc device
 *          cmd - ioctl command
 *          arg - pointer to the argument of the ioctl command
 * @return  0 - success
 *          error code - failure
 */
static long bstn_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int ret = 0;
	struct bstn_device *pbstn;

	BSTN_TRACE_PRINTK("enter, cmd: 0x%x", cmd);

	// check the device driver
	if (filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EINVAL;
	}
	if (filp->private_data == NULL) {
		printk(KERN_ERR "not BSTN device!");
		return -EFAULT;
	}
	pbstn = container_of(filp->private_data, struct bstn_device, miscdev);
	if (pbstn->state != BSTN_ONLINE) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "invalid device state!");
		return -EPERM;
	}

	switch (cmd) {
	case BSTN_IOCTL_BUF_ALLOC:
		ret = bstn_ioctl_buf_alloc(filp, pbstn, (void __user *)args);
		break;
	case BSTN_IOCTL_BUF_FREE:
		ret = bstn_ioctl_buf_free(filp, pbstn, (void __user *)args);
		break;
	case BSTN_IOCTL_BUF_SYNC:
		ret = bstn_ioctl_buf_sync(filp, pbstn, (void __user *)args);
		break;
	case BSTN_IOCTL_BUF_INVALIDATE:
		ret = bstn_ioctl_buf_invalidate(filp, pbstn,
						(void __user *)args);
		break;
	case BSTN_IOCTL_BUF_SYNC_OFFSET:
		ret = bstn_ioctl_buf_sync_offset(filp, pbstn,
						 (void __user *)args);
		break;
	case BSTN_IOCTL_CMA_BUF_IMPORT:
		ret = bstn_ioctl_cma_buf_import(filp, pbstn,
						(void __user *)args);
		break;
	case BSTN_IOCTL_CMA_BUF_RETURN:
		ret = bstn_ioctl_cma_buf_return(filp, pbstn,
						(void __user *)args);
		break;
	case BSTN_IOCTL_DMA_BUF_IMPORT:
		ret = bstn_ioctl_dma_buf_import(filp, pbstn,
						(void __user *)args);
		break;
	case BSTN_IOCTL_DMA_BUF_RETURN:
		ret = bstn_ioctl_dma_buf_return(filp, pbstn,
						(void __user *)args);
		break;
	case BSTN_IOCTL_DMA_BUF_EXPORT:
		ret = bstn_ioctl_dma_buf_export(filp, pbstn,
						(void __user *)args);
		break;
	case BSTN_IOCTL_MSG_SEND:
		ret = bstn_ioctl_msg_send(pbstn, (void __user *)args);
		break;
	case BSTN_IOCTL_VER_GET:
		ret = bstn_ioctl_ver_get(pbstn, (void __user *)args);
		break;
	case BSTN_IOCTL_PERF_GET:
		ret = bstn_ioctl_perf_get(pbstn, (void __user *)args);
		break;
	case BSTN_IOCTL_ASIC_TYPE_GET:
		ret = bstn_ioctl_asic_type_get(pbstn, (void __user *)args);
		break;
	default:
		BSTN_DEV_ERR(&pbstn->pdev->dev, "invalid ioctl cmd 0x%x!", cmd);
		ret = -EINVAL;
		break;
	}

	BSTN_TRACE_PRINTK("exit, ret: %d", ret);
	return ret;
}

/*******************************************************************************
 * BSTN MISC FILE INTERFACE
 ******************************************************************************/
/*
 * @func    bstn_open
 * @brief   This is the open callback function of the BSTN Misc Device.
 * @params  inode - the inode pointer of the BSTN misc device
 *          filp - the file pointer of the BSTN misc device
 * @return  0 - success
 *          error code - failure
 */
static int bstn_open(struct inode *inode, struct file *filp)
{
	struct bstn_device *pbstn;
	int ret = 0;

	BSTN_TRACE_PRINTK("enter");

	if (inode == NULL || filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EINVAL;
	}
	if (filp->private_data == NULL) {
		printk(KERN_ERR "not BSTN device!");
		return -EFAULT;
	}
	pbstn = container_of(filp->private_data, struct bstn_device, miscdev);

	mutex_lock(&pbstn->mutex);

	// clear msg before soft reset
	if (bstn_soft_reset) {
		bstn_firmware_stall(pbstn);
		bstn_msg_manager_exit(pbstn);
		bstn_fw_manager_unmap(pbstn);
	}

	// setup fw
	if (pbstn->state != BSTN_ONLINE || bstn_soft_reset) {
		struct bstn_memblock *block;
		struct bstn_rt_setup_info *info;
		struct bstn_rt_setup_rsp *rsp;
		struct bsnn_msg_exchange exchange_msg = { 0 };

		// prepare for fw
		ret = bstn_fw_manager_map(pbstn);
		if (ret < 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "bstn_fw_manager_map failed, ret %d",
				     ret);
			goto bstn_fw_load_boot_failed;
		}
		BSTN_STAGE_PRINTK("bstn_fw_manager_map OK");

		// init bstn message manager
		ret = bstn_msg_manager_init(pbstn);
		if (ret < 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "bstn_msg_manager_init failed, ret %d",
				     ret);
			goto bstn_fw_load_boot_failed;
		}
		BSTN_STAGE_PRINTK("bstn_msg_manager_init OK");

		bstn_soft_reset = 0;

		/* boot_done  && main_os,  skip */
		/* boot_done  && !main_os, skip */
		if(pbstn->fw_manager.fw_boot_done) {
			BSTN_STAGE_PRINTK("bstn firmware booted, don't boot again.");
		}
		else if(pbstn->fw_manager.main_os) { /* !boot_done && main_os,  boot  */
			BSTN_STAGE_PRINTK("main os booting fw");
			ret = bstn_firmware_load(pbstn);
			if (ret < 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
						 "Failed to load firmware: %d", ret);
				goto bstn_fw_load_boot_failed;
			} else {
				/*if (wdt_config_flag == false) {
				   wdt_bstn_init();
				   wdt_bstn_config(WDT_BST_BSTN_ID, WDT_PING_TIME_DEFAULT);
				   wdt_config_flag = true;
				   } */
			}
			BSTN_STAGE_PRINTK("main os load bstn firmware done");

			bstn_firmware_boot(pbstn);
			// wait for firmware to be started
			if (!bstn_msg_is_bootdone(pbstn)) {
				ret = -ETIME;
				BSTN_DEV_ERR(&pbstn->pdev->dev,
						 "Failed to boot firmware: %d", ret);
				goto bstn_fw_load_boot_failed;
			}

			bstn_fw_set_boot_flag(pbstn);
			BSTN_STAGE_PRINTK("main os boot bstn firmware done");
		} else { /* !boot_done && !main_os, wait boot done */
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "assert error branch, bstn_fw_manager_map handle this");
			goto bstn_fw_load_boot_failed;
		}
		BSTN_STAGE_PRINTK("bstn firmware load & boot done.");

		//get psmid enabled status from safety
		bstn_msg_psm_enabled_status(pbstn);

		// allocate memory block for init message data
		block = pbstn->mem_manager.ops->alloc(
			pbstn,
			sizeof(struct bstn_rt_setup_info) +
				sizeof(struct bstn_rt_setup_rsp),
			0, 0);
		if (block == NULL) {
			ret = -ENOMEM;
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "Failed to alloc rt setup info: %d", ret);
			goto bstn_fw_load_boot_failed;
		}

		info = block->kern_addr;
		info->assigned_mem =
			dma_to_bus(pbstn->fw_manager.assigned_mem->dma_addr);
		info->assigned_mem_size = pbstn->fw_manager.assigned_mem->size;
		BSTN_STAGE_PRINTK(
			"info->assigned_mem %x info->assigned_mem_size %x",
			info->assigned_mem, info->assigned_mem_size);

		rsp = (void *)(info + 1);
		info->rsp_addr = dma_to_bus(block->dma_addr +
					    sizeof(struct bstn_rt_setup_info));
		BSTN_STAGE_PRINTK("info->rsp_addr %x", info->rsp_addr);

		exchange_msg.req.opcode = RT_CMD_INIT;
		exchange_msg.req.pdata = dma_to_bus(block->dma_addr);

		// send init message
		ret = bstn_msg_exchange(pbstn, &exchange_msg);
		if (ret >= 0) {
			if (exchange_msg.rsp.status != RT_STATUS_SUCCESS) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "failure status from fw");
				ret = -EFAULT;
			} else {
				pbstn->fw_manager.release_year =
					rsp->release_date % 10000;
				pbstn->fw_manager.release_date =
					(rsp->release_date % 1000000) / 10000;
				pbstn->fw_manager.release_month =
					rsp->release_date / 1000000;
				pbstn->fw_manager.ver_major = rsp->ver_major;
				pbstn->fw_manager.ver_minor = rsp->ver_minor;
				pbstn->fw_manager.ver_patch = rsp->ver_patch;
				BSTN_STAGE_PRINTK(
					"firmware v%d.%d.%d released on %02d/%02d/%04d",
					pbstn->fw_manager.ver_major,
					pbstn->fw_manager.ver_minor,
					pbstn->fw_manager.ver_patch,
					pbstn->fw_manager.release_month,
					pbstn->fw_manager.release_date,
					pbstn->fw_manager.release_year);
				ret = 0;
			}
		} else {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "bstn_msg_exchange timeout");
			ret = -ENOMSG;
		}

		pbstn->mem_manager.ops->free(block);

		if (ret < 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "bstn fw load/boot failed, ret %d", ret);
			goto bstn_fw_load_boot_failed;
		}
		BSTN_STAGE_PRINTK("bstn fw load & boot OK");
		pbstn->state = BSTN_ONLINE;
#if 0
		if (bstn_fw_debug) {
			exchange_msg.req.opcode = RT_CMD_DEBUG;
			exchange_msg.req.pdata = bstn_fw_debug;

			ret = bstn_msg_exchange(pbstn, &exchange_msg);
			// error code
			if (ret < 0) {
				BSTN_DEV_ERR(&pbstn->pdev->dev,
					     "fw debug bstn_msg_exchange error: %d",
					     ret);
				return ret;
			}
			// time out
			else if (ret == 0) {
				BSTN_STAGE_PRINTK
				    ("fw debug bstn_msg_exchange time out, it'll reset "
				     "NET DSP when next time driver gets opened");
				bstn_soft_reset = 1;
				return -ENOMSG;
			}
		}
		if (pbstn->msg_manager.bist_thread_start) {
			// set up sw bist thread while the initial value of bist_thread_start is 1
			/*ret = bstn_msg_start_sw_bist_thread(pbstn);
			   if (ret != 0) {
			   BSTN_DEV_ERR(&pbstn->pdev->dev, "start sw bist failed error: %d", ret);
			   goto bstn_fw_load_boot_failed;
			   }
			   BSTN_STAGE_PRINTK("start sw bist ok"); */
		}
#endif
		if (bstn_fw_profiling) {
			exchange_msg.req.opcode = RT_CMD_PROFILING;
			exchange_msg.req.pdata = bstn_fw_profiling;

			ret = bstn_msg_exchange(pbstn, &exchange_msg);
			// error code
			if (ret < 0) {
				BSTN_DEV_ERR(
					&pbstn->pdev->dev,
					"fw profiling bstn_msg_exchange error: %d",
					ret);
				return ret;
			}
			// time out
			else if (ret == 0) {
				BSTN_STAGE_PRINTK(
					"fw profiling bstn_msg_exchange time out, it'll "
					"reset NET DSP when next time driver gets opened");
				bstn_soft_reset = 1;
				return -ENOMSG;
			}
		}
	}

	ret = bstn_mem_ctx_add(pbstn, filp);
	if (ret < 0) {
		BSTN_DEV_ERR(&pbstn->pdev->dev,
			     "bstn_mem_ctx_add failed, ret %d", ret);
		goto bstn_fw_load_boot_failed;
	}

	mutex_unlock(&pbstn->mutex);

	BSTN_TRACE_PRINTK("exit");
	return ret;

bstn_fw_load_boot_failed:
	BSTN_DEV_ERR(&pbstn->pdev->dev, "bstn open failed, ret %d", ret);

	bstn_fw_rt_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_fw_rt_exit OK");
	bstn_msg_manager_exit(pbstn);
	BSTN_STAGE_PRINTK("bstn_msg_manager_exit OK");
	pbstn->state = BSTN_ERROR;

	mutex_unlock(&pbstn->mutex);
	return ret;
}

/*
 * @func    bstn_close
 * @brief   This is the close callback function of the BSTN Misc Device.
 * @params  inode - the inode pointer of the BSTN misc device
 *          filp - the file pointer of the BSTN misc device
 * @return  0 - success
 *          error code - failure
 */
static int bstn_close(struct inode *inode, struct file *filp)
{
	struct bstn_device *pbstn;
	int ret = 0;

	BSTN_TRACE_PRINTK("enter");

	if (inode == NULL || filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EINVAL;
	}
	if (filp->private_data == NULL) {
		printk(KERN_ERR "not BSTN device!");
		return -EFAULT;
	}
	pbstn = container_of(filp->private_data, struct bstn_device, miscdev);

	mutex_lock(&pbstn->mutex);
	if (pbstn->state != BSTN_ONLINE) {
		BSTN_DEV_ERR(&pbstn->pdev->dev, "invalid device state!");
		ret = -EPERM;
	} else {
		ret = bstn_mem_ctx_del(pbstn, filp);
		if (ret < 0) {
			BSTN_DEV_ERR(&pbstn->pdev->dev,
				     "bstn_mem_ctx_del failed, ret %d", ret);
		}
	}

	mutex_unlock(&pbstn->mutex);

	BSTN_TRACE_PRINTK("exit");
	return ret;
}

/*
 * @func    bstn_mmap
 * @brief   This is the mmap callback function of the BSTN Misc Device.
 * @params  inode - the inode pointer of the BSTN misc device
 *          vma - the vma pointer of the target vm area
 * @return  0 for success and error code otherwise
 */
static int bstn_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = 0;

	if (filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EINVAL;
	}
	if (vma == NULL) {
		printk(KERN_ERR "invalid vma area");
		return -EINVAL;
	}

	BSTN_TRACE_PRINTK(
		"enter, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);

	//map as cacheable memory into userspace
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			      vma->vm_end - vma->vm_start, vma->vm_page_prot);

	BSTN_TRACE_PRINTK("exit, ret: %d", ret);
	return ret;
}

/*******************************************************************************
 * BSTN MISC Initialization
 ******************************************************************************/
static const struct file_operations bstn_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = bstn_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bstn_ioctl,
#endif
	.mmap = bstn_mmap,
	.open = bstn_open,
	.release = bstn_close,
};

/*
 * @func    bstn_misc_init
 * @brief   This is the initialization function of the BSTN Misc Device.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bstn_misc_init(struct bstn_device *pbstn)
{
	int ret;
	char dev_name[sizeof(BSTN_DRIVER_NAME) + BSTN_DEV_ID_LEN];

	sprintf(dev_name, "bstn%d", pbstn->id);
	BSTN_TRACE_PRINTK("bstn struct ptr: %px", pbstn);
	BSTN_TRACE_PRINTK("probe device name: %s", dev_name);

	//init & register bstn miscdev
	pbstn->miscdev.minor = MISC_DYNAMIC_MINOR;
	pbstn->miscdev.fops = &bstn_fops;
	pbstn->miscdev.name =
		devm_kstrdup(&pbstn->pdev->dev, dev_name, GFP_KERNEL);
	pbstn->miscdev.nodename =
		devm_kstrdup(&pbstn->pdev->dev, dev_name, GFP_KERNEL);

	ret = misc_register(&pbstn->miscdev);
	return ret;
}

/*
 * @func    bstn_misc_exit
 * @brief   This is the cleanup function of the BSTN Misc Device. It garantees
 *          that the misc device cannot be deregistered when there are still
 *          opened file handles.
 * @params  pbstn - the pointer to the BSTN device
 * @return  0 - success
 *          error code - failure
 */
int bstn_misc_exit(struct bstn_device *pbstn)
{
	int ret = 0;

	mutex_lock(&pbstn->mutex);
	pbstn->state = pbstn->state == BSTN_ONLINE ? BSTN_OFFLINE :
						     pbstn->state;
	mutex_unlock(&pbstn->mutex);

	// this is mutually exlusive with open due to misc implementation
	misc_deregister(&pbstn->miscdev);
	BSTN_STAGE_PRINTK("misc_deregister OK");

	return ret;
}
