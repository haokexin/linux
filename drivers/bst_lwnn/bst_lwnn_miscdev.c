// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn: Linux device driver for Black Sesame Technologies Computer Vision IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_misc.c
 * @brief   This file is the source code file of misc device interface of bst_lwnn
 *          driver. It contains function definitions of ioctl callbacks,
 *          initialization and exit of the misc device.
 */

#include "bst_lwnn.h"
#include "bst_lwnn_mem_manager.h"

/*******************************************************************************
 * bst_lwnn MISC IOCTL INTERFACE
 ******************************************************************************/
/*!
 * @brief           This function allocates a continuous memory buffer requested
 *                  by the user.
 * @param[in]       filp The misc device file descriptor
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[in,out]   ubuffer The user pointer to the buffer information
 * @return          0 - success
 *                  Error code - failure
 */
static int bst_lwnn_ioctl_buf_alloc(struct file *filp,
				    struct bst_lwnn *pbst_lwnn,
				    struct bst_lwnn_user_buffer __user *ubuffer)
{
	struct bst_lwnn_user_buffer buffer;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", ubuffer);

	ret = copy_from_user(&buffer, ubuffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}
	if (buffer.size == 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "zero size buffer!");
		return -EINVAL;
	}

	ret = lwnn_buffer_alloc(filp, pbst_lwnn, &buffer);
	if (ret != 0) {
		return ret;
	}

	ret = copy_to_user(ubuffer, &buffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		lwnn_buffer_free(filp, pbst_lwnn, &buffer);
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief       This function synchronizes the specified continuous memory
 *              buffer by flushing its cache.
 * @param[in]   filp The misc device file descriptor
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   ubuffer The user pointer to the synchronized user buffer
 *              inforamtion
 * @return      0 - success
 *              Error code - failure
 */
static int bst_lwnn_ioctl_buf_flush(struct file *filp,
				    struct bst_lwnn *pbst_lwnn,
				    struct bst_lwnn_user_buffer __user *pbuffer)
{
	int ret;
	struct bst_lwnn_user_buffer buffer;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);
	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BST_LWNN_DEV_ERR(pdev, "copy_from_user failed");
		return -EFAULT;
	}

	if (pmman->enable_smmu) {
		ret = bst_lwnn_dma_buf_flush(filp, pbst_lwnn, &buffer);
		if (ret != 0) {
			BST_LWNN_DEV_ERR(pdev, "dma buf flush failed!");
			return -EFAULT;
		}
		return 0;
	} else {
		dma_sync_single_for_device(pbst_lwnn->mem_manager.pdev,
					   bus_to_dma(buffer.bus_addr),
					   buffer.size, DMA_TO_DEVICE);
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return 0;
}

/*!
 * @brief       This function synchronizes the specified continuous memory
 *              buffer by invalidating its cache.
 * @param[in]   filp The misc device file descriptor
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   pbuffer The user pointer to the synchronized user buffer
 *              inforamtion
 * @return      0 - success
 *              Error code - failure
 */
static int
bst_lwnn_ioctl_buf_invalidate(struct file *filp, struct bst_lwnn *pbst_lwnn,
			      struct bst_lwnn_user_buffer __user *pbuffer)
{
	int ret;
	struct bst_lwnn_user_buffer buffer;
	struct bst_lwnn_mem_manager *pmman;
	struct device *pdev;

	pmman = &pbst_lwnn->mem_manager;
	pdev = &pbst_lwnn->pdev->dev;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", pbuffer);
	if (copy_from_user(&buffer, pbuffer, sizeof(buffer))) {
		BST_LWNN_DEV_ERR(pdev, "copy_from_user failed");
		return -EFAULT;
	}

	if (pmman->enable_smmu) {
		ret = bst_lwnn_dma_buf_invalidate(filp, pbst_lwnn, &buffer);
		if (ret != 0) {
			BST_LWNN_DEV_ERR(pdev, "dma buf invalidate failed!");
			return -EFAULT;
		}
		return 0;
	} else {
		dma_sync_single_for_cpu(pbst_lwnn->mem_manager.pdev,
					bus_to_dma(buffer.bus_addr),
					buffer.size, DMA_FROM_DEVICE);
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return 0;
}

/*!
 * @brief       This function frees the allocated continuous memory buffer.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   ubuffer The user pointer to the freed user buffer information
 * @return      0 - success
 *              Error code - failure
 */
static int bst_lwnn_ioctl_buf_free(struct file *filp,
				   struct bst_lwnn *pbst_lwnn,
				   struct bst_lwnn_user_buffer __user *ubuffer)
{
	struct bst_lwnn_user_buffer buffer;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", ubuffer);

	ret = copy_from_user(&buffer, ubuffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}
	BST_LWNN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);

	ret = lwnn_buffer_free(filp, pbst_lwnn, &buffer);
	if (ret != 0) {
		BST_LWNN_STAGE_PRINTK("buffer free failed, ret=%d", ret);
		return ret;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

static int
bst_lwnn_ioctl_cma_buf_import(struct file *filp, struct bst_lwnn *pbst_lwnn,
			      struct bst_lwnn_cma_buf __user *ubuffer)
{
	struct bst_lwnn_cma_buf buffer;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", ubuffer);
	if (!pbst_lwnn->mem_manager.enable_smmu) {
		ubuffer->bus_addr = phys_to_bus(ubuffer->pa);
		BST_LWNN_TRACE_PRINTK("exit import, smmu off");
		return 0;
	}

	ret = copy_from_user(&buffer, ubuffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}

	ret = bst_lwnn_cma_buf_import(filp, pbst_lwnn, &buffer);
	if (ret != 0) {
		return ret;
	} else {
		BST_LWNN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);
	}

	ret = copy_to_user(ubuffer, &buffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		bst_lwnn_cma_buf_return(filp, pbst_lwnn, &buffer);
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

static int
bst_lwnn_ioctl_cma_buf_return(struct file *filp, struct bst_lwnn *pbst_lwnn,
			      struct bst_lwnn_cma_buf __user *ubuffer)
{
	struct bst_lwnn_cma_buf buffer;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", ubuffer);
	if (!pbst_lwnn->mem_manager.enable_smmu) {
		return 0;
	}

	ret = copy_from_user(&buffer, ubuffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}
	BST_LWNN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);

	ret = bst_lwnn_cma_buf_return(filp, pbst_lwnn, &buffer);

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief           This function imports a dma-buf.
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[in,out]   ubuffer The user pointer to the imported dma-buf information
 * @return          0 - success
 *                  Error code - failure
 */
static int
bst_lwnn_ioctl_dma_buf_import(struct file *filp, struct bst_lwnn *pbst_lwnn,
			      struct bst_lwnn_dma_buf __user *ubuffer)
{
	struct bst_lwnn_dma_buf buffer;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", ubuffer);

	ret = copy_from_user(&buffer, ubuffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}

	ret = bst_lwnn_dma_buf_import(filp, pbst_lwnn, &buffer);
	if (ret != 0) {
		return ret;
	} else {
		BST_LWNN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);
	}

	ret = copy_to_user(ubuffer, &buffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		bst_lwnn_dma_buf_return(filp, pbst_lwnn, &buffer);
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief           This function export a dma-buf.
 * @param[in]       pbst_lwnn      The bst_lwnn driver
 * @param[in,out]   buf                The user pointer to the exported dma-buf information
 * @return          0 - success
 *                  Error code - failure
 */
static int bst_lwnn_ioctl_dma_buf_export(struct file *filp,
					 struct bst_lwnn *pbst_lwnn,
					 struct bst_lwnn_dma_buf __user *buf)
{
	int ret;
	struct bst_lwnn_dma_buf buffer;
	ret = copy_from_user(&buffer, buf, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}

	ret = bst_lwnn_dma_buf_export(filp, pbst_lwnn, &buffer);
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "dma buf export failed!");
		return -EFAULT;
	}
	BST_LWNN_TRACE_PRINTK("exp fd: %d", buffer.fd);

	ret = copy_to_user(buf, &buffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit export");
	return ret;
}

/*!
 * @brief       This function returns a dma-buf.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @param[in]   ubuffer The user pointer to the returned dma-buf information
 * @return      0 - success
 *              Error code - failure
 */
static int
bst_lwnn_ioctl_dma_buf_return(struct file *filp, struct bst_lwnn *pbst_lwnn,
			      struct bst_lwnn_dma_buf __user *ubuffer)
{
	struct bst_lwnn_dma_buf buffer;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", ubuffer);

	ret = copy_from_user(&buffer, ubuffer, sizeof(buffer));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}
	BST_LWNN_TRACE_PRINTK("buffer addr: 0x%x", buffer.bus_addr);

	ret = bst_lwnn_dma_buf_return(filp, pbst_lwnn, &buffer);

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief           This function sends the bst_lwnn message to the DSP. It is a
 *                  nonblocking function which returns immediately after the
 *                  request is added into the work list of the worker thread of
 *                  one DSP.
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[in,out]   pxchg The user pointer to the preallocated message xchg
 *                  structure
 * @return          0 - success
 *                  Error code  - failure
 */
static int bst_lwnn_ioctl_msg_xchg(struct bst_lwnn *pbst_lwnn,
				   struct bst_lwnn_msg_xchg __user *pmsg_xchg)
{
	int ret;
	struct bst_lwnn_msg_xchg msg_xchg;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", pmsg_xchg);

	ret = copy_from_user(&msg_xchg, pmsg_xchg, sizeof(msg_xchg));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "copy_from_user failed!");
		return -EFAULT;
	}

	ret = bst_lwnn_msg_xchg(pbst_lwnn, &msg_xchg);
	if (ret != 0) {
		return ret;
	}

	ret = copy_to_user(&pmsg_xchg->rsp, &msg_xchg.rsp,
			   sizeof(msg_xchg.rsp));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief           This function gets the DSP status information of the LWNN
 *                  driver.
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[out]      pdsp_info The user pointer to the preallocated DSP
 *                  information structure
 * @return          0 - success
 *                  Error code  - failure
 */
static int
bst_lwnn_ioctl_dsp_info_get(struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_ver_info __user *pdsp_info)
{
	int ret, i;
	struct bst_lwnn_dsp_info dsp_info;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", pdsp_info);

	dsp_info.dsp_num = pbst_lwnn->dsp_num;
	/*
	   Even if we lock the dsp_online here, TOCTOU issues can still happen in
	   userland. Also because the operation here is only read, there is no need
	   to carefully guard with any synchronization primitive.
	 */
	for (i = 0; i < dsp_info.dsp_num; i++) {
		dsp_info.dsp_online[i] = pbst_lwnn->dsp_online[i];
		dsp_info.dsp_indices[i] = pbst_lwnn->dsp_indices[i];
	}

	ret = copy_to_user(pdsp_info, &dsp_info, sizeof(dsp_info));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief           This function gets the version information of the LWNN
 *                  driver, including firmware.
 * @param[in]       pbst_lwnn The bst_lwnn driver
 * @param[out]      pver_info The user pointer to the preallocated version
 *                  information structure
 * @return          0 - success
 *                  Error code  - failure
 */
static int
bst_lwnn_ioctl_ver_info_get(struct bst_lwnn *pbst_lwnn,
			    struct bst_lwnn_ver_info __user *pver_info)
{
	int ret;
	struct bst_lwnn_ver_info ver_info;

	BST_LWNN_TRACE_PRINTK("enter, user ptr: %px", pver_info);

	ver_info.drv.ver_major = BST_LWNN_VER_MAJOR;
	ver_info.drv.ver_minor = BST_LWNN_VER_MINOR;
	ver_info.drv.ver_patch = BST_LWNN_VER_PATCH;
	ver_info.drv.release_month = BST_LWNN_RELEASE_MONTH;
	ver_info.drv.release_date = BST_LWNN_RELEASE_DATE;
	ver_info.drv.release_year = BST_LWNN_RELEASE_YEAR;
	ver_info.fw = pbst_lwnn->fw_manager.ver_info;

	ret = copy_to_user(pver_info, &ver_info, sizeof(ver_info));
	if (ret != 0) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "copy_to_user failed!");
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief       This function is a wrapper bst_lwnn ioctl interface.
 * @param[in]   filp The file pointer to the misc device
 * @param[in]   cmd The ioctl command
 * @param[in]   arg The pointer to the argument of the ioctl command
 * @return      0 - success
 *              Error code - failure
 */
static long bst_lwnn_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long args)
{
	int ret = 0;
	struct bst_lwnn *pbst_lwnn;

	BST_LWNN_TRACE_PRINTK("%s, cmd: 0x%x", "enter", cmd);

	// check the device driver
	if (filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EFAULT;
	}
	if (filp->private_data == NULL) {
		printk(KERN_ERR "not bst_lwnn device!");
		return -EFAULT;
	}
	if (!try_module_get(THIS_MODULE)) {
		printk(KERN_ERR "try_module_get failed!");
		return -EFAULT;
	}

	pbst_lwnn = container_of(filp->private_data, struct bst_lwnn, miscdev);
	/* BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev, "build:%s", __TIME__); */
	mutex_lock(&pbst_lwnn->mutex);
	if (pbst_lwnn->state != BST_LWNN_ONLINE) {
		mutex_unlock(&pbst_lwnn->mutex);
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid device state %d", pbst_lwnn->state);
		return -EFAULT;
	}
	mutex_unlock(&pbst_lwnn->mutex);

	switch (cmd) {
	case BST_LWNN_IOCTL_BUF_ALLOC:
		ret = bst_lwnn_ioctl_buf_alloc(filp, pbst_lwnn,
					       (void __user *)args);
		break;
	case BST_LWNN_IOCTL_BUF_FREE:
		ret = bst_lwnn_ioctl_buf_free(filp, pbst_lwnn,
					      (void __user *)args);
		break;
	case BST_LWNN_IOCTL_BUF_FLUSH:
		ret = bst_lwnn_ioctl_buf_flush(filp, pbst_lwnn,
					       (void __user *)args);
		break;
	case BST_LWNN_IOCTL_BUF_INVALIDATE:
		ret = bst_lwnn_ioctl_buf_invalidate(filp, pbst_lwnn,
						    (void __user *)args);
		break;
	case BST_LWNN_IOCTL_CMA_BUF_IMPORT:
		ret = bst_lwnn_ioctl_cma_buf_import(filp, pbst_lwnn,
						    (void __user *)args);
		break;
	case BST_LWNN_IOCTL_CMA_BUF_RETURN:
		ret = bst_lwnn_ioctl_cma_buf_return(filp, pbst_lwnn,
						    (void __user *)args);
		break;
	case BST_LWNN_IOCTL_DMA_BUF_IMPORT:
		ret = bst_lwnn_ioctl_dma_buf_import(filp, pbst_lwnn,
						    (void __user *)args);
		break;
	case BST_LWNN_IOCTL_DMA_BUF_EXPORT:
		ret = bst_lwnn_ioctl_dma_buf_export(filp, pbst_lwnn,
						    (void __user *)args);
		break;
	case BST_LWNN_IOCTL_DMA_BUF_RETURN:
		ret = bst_lwnn_ioctl_dma_buf_return(filp, pbst_lwnn,
						    (void __user *)args);
		break;
	case BST_LWNN_IOCTL_MSG_XCHG:
		ret = bst_lwnn_ioctl_msg_xchg(pbst_lwnn, (void __user *)args);
		break;
	case BST_LWNN_IOCTL_DSP_INFO_GET:
		ret = bst_lwnn_ioctl_dsp_info_get(pbst_lwnn,
						  (void __user *)args);
		break;
	case BST_LWNN_IOCTL_VER_INFO_GET:
		ret = bst_lwnn_ioctl_ver_info_get(pbst_lwnn,
						  (void __user *)args);
		break;
	default:
		ret = -EINVAL;
	}

	module_put(THIS_MODULE);
	BST_LWNN_TRACE_PRINTK("%s, ret: %d", "exit", ret);
	return ret;
}

/*******************************************************************************
 * bst_lwnn MISC FILE INTERFACE
 ******************************************************************************/
/*!
 * @brief       This is the open callback function of the bst_lwnn Misc Device.
 * @param[in]   inode The inode pointer of the bst_lwnn misc device
 * @param[in]   filp The file pointer of the bst_lwnn misc device
 * @return      0 - success
 *              Error code - failure
 */
static int bst_lwnn_open(struct inode *inode, struct file *filp)
{
	struct bst_lwnn *pbst_lwnn;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter");

	if (inode == NULL || filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EFAULT;
	}
	if (filp->private_data == NULL) {
		printk(KERN_ERR "not bst_lwnn device!");
		return -EFAULT;
	}

	pbst_lwnn = container_of(filp->private_data, struct bst_lwnn, miscdev);

	mutex_lock(&pbst_lwnn->mutex);
	if (pbst_lwnn->state == BST_LWNN_ONLINE) {
		ret = 0;
	} else if (pbst_lwnn->state == BST_LWNN_INIT) {
		// init bst_lwnn message manager
		ret = bst_lwnn_msg_manager_init(pbst_lwnn);
		if (ret < 0) {
			BST_LWNN_DEV_ERR(
				&pbst_lwnn->pdev->dev,
				"bst_lwnn_msg_manager_init all failed");
			ret = -EFAULT;
		}
		BST_LWNN_STAGE_PRINTK("bst_lwnn_msg_manager_init OK");

		//for safety
		bst_lwnn_msg_psm_enabled_status(pbst_lwnn);

		//setup rt fw
		ret = bst_lwnn_fw_rt_setup(pbst_lwnn);
		bst_lwnn_msg_manager_cleanup(pbst_lwnn);
		bst_lwnn_fw_manager_cleanup(pbst_lwnn);
		if (ret != 0) {
			pbst_lwnn->state = BST_LWNN_ERROR;
			BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
					 "bst_lwnn_fw_rt_setup all failed");
			bst_lwnn_mem_manager_exit(pbst_lwnn);
			BST_LWNN_STAGE_PRINTK("bst_lwnn_mem_manager_exit OK");
		} else {
			pbst_lwnn->state = BST_LWNN_ONLINE;
			BST_LWNN_STAGE_PRINTK("bst_lwnn_fw_rt_setup OK");
		}
	} else {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid device state %d", pbst_lwnn->state);
		ret = -EFAULT;
	}

	if (ret == 0) {
		ret = bst_lwnn_mem_ctx_add(pbst_lwnn, filp);
	}
	mutex_unlock(&pbst_lwnn->mutex);

	if (ret == 0) {
		if (!try_module_get(THIS_MODULE)) {
			printk(KERN_ERR "try_module_get failed!");
			ret = -EFAULT;
		}
	}
	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief       This is the close callback function of the bst_lwnn Misc Device.
 * @param[in]   inode The inode pointer of the bst_lwnn misc device
 * @param[in]   filp The file pointer of the bst_lwnn misc device
 * @return      0 - success
 *              Error code - failure
 */
static int bst_lwnn_close(struct inode *inode, struct file *filp)
{
	struct bst_lwnn *pbst_lwnn;
	int ret;

	BST_LWNN_TRACE_PRINTK("enter");

	if (inode == NULL || filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EFAULT;
	}
	if (filp->private_data == NULL) {
		printk(KERN_ERR "not bst_lwnn device!");
		return -EFAULT;
	}
	pbst_lwnn = container_of(filp->private_data, struct bst_lwnn, miscdev);

	mutex_lock(&pbst_lwnn->mutex);
	if (pbst_lwnn->state != BST_LWNN_ONLINE) {
		BST_LWNN_DEV_ERR(&pbst_lwnn->pdev->dev,
				 "invalid device state %d", pbst_lwnn->state);
		ret = -EFAULT;
	} else {
		ret = bst_lwnn_mem_ctx_del(pbst_lwnn, filp);
	}
	mutex_unlock(&pbst_lwnn->mutex);

	module_put(THIS_MODULE);
	BST_LWNN_TRACE_PRINTK("exit");
	return ret;
}

/*!
 * @brief       This is the mmap callback function of the bst_lwnn Misc Device.
 * @param[in]   inode The inode pointer of the bst_lwnn misc device
 * @param[in]   vma The pointer of the target vm area
 * @return      0 - success
 *              Error code - failure
 */
static int bst_lwnn_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret = 0;

	if (filp == NULL) {
		printk(KERN_ERR "invalid file!");
		return -EFAULT;
	}
	if (vma == NULL) {
		printk(KERN_ERR "invalid vma area");
		return -EFAULT;
	}

	BST_LWNN_TRACE_PRINTK(
		"enter, vm_start: 0x%lx, vm_end: 0x%lx, vm_pgoff: 0x%lx",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);

	//map as cacheable memory into userspace
	ret = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			      vma->vm_end - vma->vm_start, vma->vm_page_prot);

	BST_LWNN_TRACE_PRINTK("exit, ret: %d", ret);
	return ret;
}

/*******************************************************************************
 * bst_lwnn MISC Initialization
 ******************************************************************************/
static const struct file_operations bst_lwnn_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = bst_lwnn_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bst_lwnn_ioctl,
#endif
	.mmap = bst_lwnn_mmap,
	.open = bst_lwnn_open,
	.release = bst_lwnn_close,
};

/*!
 * @brief       This is the initialization function of the bst_lwnn misc device.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
int bst_lwnn_miscdev_init(struct bst_lwnn *pbst_lwnn)
{
	int i;
	int ret;

	// init & register bst_lwnn miscdev
	pbst_lwnn->miscdev.minor = MISC_DYNAMIC_MINOR;
	pbst_lwnn->miscdev.fops = &bst_lwnn_fops;
	pbst_lwnn->miscdev.name = devm_kstrdup(
		&pbst_lwnn->pdev->dev, BST_LWNN_DRIVER_NAME, GFP_KERNEL);
	pbst_lwnn->miscdev.nodename = devm_kstrdup(
		&pbst_lwnn->pdev->dev, BST_LWNN_DRIVER_NAME, GFP_KERNEL);

	ret = misc_register(&pbst_lwnn->miscdev);
	if (ret != 0) {
		for (i = 0; i < pbst_lwnn->dsp_num; i++) {
			pbst_lwnn->dsp_online[i] = 0;
		}
	}
	return ret;
}

/*!
 * @brief       This is the exit function of the bst_lwnn misc device.
 * @param[in]   pbst_lwnn The bst_lwnn driver
 * @return      0 - success
 *              Error code - failure
 */
void bst_lwnn_miscdev_exit(struct bst_lwnn *pbst_lwnn)
{
	//this is mutually exlusive with open due to misc implementation
	misc_deregister(&pbst_lwnn->miscdev);
	BST_LWNN_STAGE_PRINTK("misc_deregister OK");
	return;
}
