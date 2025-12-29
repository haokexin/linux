// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 *
 */
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include "bst_drm_dev.h"
#include "bst_md_csr.h"
#include "bst_disp_conn.h"
#include <drm/drm_print.h>

struct dpu_addr_info {
	u32 reg;
	u32 dpu_id;
};

#define MEDIANOC_SERVER_ADDR (0x30230000)
#define QoSGenerator_DISP0_DMA   (0x3000)
#define QoSGenerator_DISP0_P0    (0x3080)
#define QoSGenerator_DISP0_P1    (0x3100)
#define QoSGenerator_DISP0_TCU1  (0x3180)
#define QoSGenerator_DISP0_TCU   (0x3200)
#define QoSGenerator_DISP1_DMA   (0x3280)
#define QoSGenerator_DISP1_P0    (0x3300)
#define QoSGenerator_DISP1_P1    (0x3380)
#define QoSGenerator_DISP1_TCU1  (0x3400)
#define QoSGenerator_DISP1_TCU   (0x3480)
#define QoSGenerator_DISP_SP_DMA  (0x3500)
#define QoSGenerator_DISP_SP_P0   (0x3580)
#define QoSGenerator_DISP_SP_TCU1 (0x3600)
#define QoSGenerator_DISP_SP_TCU  (0x3680)
#define QoS_PRIORITY_REG          (0x08)
#define QoS_MODE_REG              (0x0C)
#define QoS_BANDWIDTH_REG         (0x10)
#define QoS_SATURATION_REG        (0x14)
#define QoS_EXTCONTROL_REG        (0x1C)
#define QoS_MODE_FIXED            (0x0)
#define QoS_MODE_LIMITER          (0x1)
#define QoS_MODE_BYPASS           (0x2)
#define QoS_MODE_REGULATOR        (0x3)

const static struct dpu_addr_info dpu_id_map[] = {
	{0x28000000, 0}, {0x28800000, 1}, {0x29000000, 2}
};

struct meida_noc_qos {
	u32 priority;
	u32 socket_qos_en;
	u32 qos_mode;
};

const static struct meida_noc_qos disp_pipe_qos_map[] = {
	[0] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-0 pipe-0 */
	[1] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-0 pipe-1 */
	[2] = {0x7, 0x0, QoS_MODE_FIXED}, /* display-1 pipe-0 */
	[3] = {0x7, 0x0, QoS_MODE_FIXED}, /* display-1 pipe-1 */
	[4] = {0x7, 0x0, QoS_MODE_FIXED}, /* display-2 pipe-0 */
};

const static struct meida_noc_qos disp_top_qos_map[] = {
	[0] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-0 TCU  */
	[1] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-0 TCU1 */
	[2] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-0 DMA  */
	[3] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-1 TCU  */
	[4] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-1 TCU1 */
	[5] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-1 DMA  */
	[6] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-2 TCU  */
	[7] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-2 TCU1 */
	[8] = {0x5, 0x0, QoS_MODE_FIXED}, /* display-2 DMA  */
};


static inline u32 build_disp_topo_value(u32 disp_id, u32 pipe_id)
{
    return (disp_id << 1) | (pipe_id << 0);
}

static inline void __iomem * dev_to_mdnoc_qos_base(struct device *dev)
{
    struct bst_dev *mdev = dev_to_mdev(dev);
    return mdev->mdnoc_qos_base;
}

static u32 bst_mdnoc_qos_read(void __iomem * addr, u32 reg)
{
    return readl(addr + reg);
}

static void bst_mdnoc_qos_write(void __iomem * addr, u32 reg, u32 val)
{
    writel(val, addr + reg);
}

static void update_media_qos(void __iomem * blk_addr, const struct meida_noc_qos* qos)
{
	u32 reg_val = 0;

	reg_val = bst_mdnoc_qos_read(blk_addr, QoS_PRIORITY_REG);
	reg_val &= ~(0x7<<0);
	reg_val |= (qos->priority << 0);
	reg_val &= ~(0x7<<8);
	reg_val |= (qos->priority << 8);
	bst_mdnoc_qos_write(blk_addr, QoS_PRIORITY_REG, reg_val);

	reg_val = bst_mdnoc_qos_read(blk_addr, QoS_MODE_REG);
	reg_val &= ~(0x3<<0);
	reg_val |= (qos->qos_mode << 0);
	bst_mdnoc_qos_write((void*)blk_addr, QoS_MODE_REG, reg_val);

	reg_val = bst_mdnoc_qos_read(blk_addr, QoS_EXTCONTROL_REG);
	reg_val &= ~(0x1<<0);
	reg_val |= (qos->socket_qos_en << 0);
	bst_mdnoc_qos_write(blk_addr, QoS_EXTCONTROL_REG, reg_val);
}

static int set_meida_noc_qos_by_id(struct device *dev, u32 dpu_id, u32 pipe_id,
	bool bypass_kernel)
{
	u32 topo_id = build_disp_topo_value(dpu_id, pipe_id);
	void __iomem *blk_addr = 0;
	void __iomem *base = dev_to_mdnoc_qos_base(dev);
	const struct meida_noc_qos* pipe_qos = &disp_pipe_qos_map[topo_id];
	const struct meida_noc_qos* tcu_qos = &disp_top_qos_map[dpu_id * 3 + 0];
	const struct meida_noc_qos* tcu1_qos = &disp_top_qos_map[dpu_id * 3 + 1];
	const struct meida_noc_qos* dma_qos = &disp_top_qos_map[dpu_id * 3 + 2];

	if (bypass_kernel) return 0;

	switch(topo_id){
		case 0:
		blk_addr = base + QoSGenerator_DISP0_P0;
		update_media_qos(blk_addr, pipe_qos);
		break;
		case 1:
		blk_addr = base + QoSGenerator_DISP0_P1;
		update_media_qos(blk_addr, pipe_qos);
		break;
		case 2:
		blk_addr = base + QoSGenerator_DISP1_P0;
		update_media_qos(blk_addr, pipe_qos);
		break;
		case 3:
		blk_addr = base + QoSGenerator_DISP1_P1;
		update_media_qos(blk_addr, pipe_qos);
		break;
		case 4:
		blk_addr = base + QoSGenerator_DISP_SP_P0;
		update_media_qos(blk_addr, pipe_qos);
		break;
		default:
		DRM_ERROR("not support topo_id=%d for mdnoc disp qos\n", topo_id);
		return -EINVAL;
	}

	switch(dpu_id){
		case 0:
		blk_addr = base + QoSGenerator_DISP0_DMA;
		update_media_qos(blk_addr, dma_qos);
		blk_addr = base + QoSGenerator_DISP0_TCU1;
		update_media_qos(blk_addr, tcu1_qos);
		blk_addr = base + QoSGenerator_DISP0_TCU;
		update_media_qos(blk_addr, tcu_qos);
		break;
		case 1:
		blk_addr = base + QoSGenerator_DISP1_DMA;
		update_media_qos(blk_addr, dma_qos);
		blk_addr = base + QoSGenerator_DISP1_TCU1;
		update_media_qos(blk_addr, tcu1_qos);
		blk_addr = base + QoSGenerator_DISP1_TCU;
		update_media_qos(blk_addr, tcu_qos);
		break;
		case 2:
		blk_addr = base + QoSGenerator_DISP_SP_DMA;
		update_media_qos(blk_addr, dma_qos);
		blk_addr = base + QoSGenerator_DISP_SP_TCU1;
		update_media_qos(blk_addr, tcu1_qos);
		blk_addr = base + QoSGenerator_DISP_SP_TCU;
		update_media_qos(blk_addr, dma_qos);
		break;
		default:
		DRM_ERROR("not support dpu_id=%d for mdnoc disp qos\n", dpu_id);
		return -EINVAL;
	}

	DRM_INFO("display topo_id=%d, \
			  dpu_id=%d, pipe=%d, \
			  PIPE_QoS: priority-%d, \
			  mode-%d, socket_qos_en-%d\n", \
		topo_id, dpu_id, pipe_id, \
		pipe_qos->priority, \
		pipe_qos->qos_mode, \
		pipe_qos->socket_qos_en);

	return 0;
}

static inline int dt_node_get_dpu_id_by_reg(struct device_node *node, unsigned int *dpu_id)
{
	struct resource res;
	int ret;
	int i;

	ret = of_address_to_resource(node, 0, &res);
	if (ret)
		return ret;

	for (i = 0; i < sizeof(dpu_id_map)/sizeof(dpu_id_map[0]); i++) {
		if (dpu_id_map[i].reg == res.start) {
			*dpu_id = dpu_id_map[i].dpu_id;
			return 0;
		}
	}

	return -EINVAL;
}

int bst_get_remote_dpu_connection(struct device_node *endpoint,
		struct bst_dpu_connection *conn)
{
	struct device_node *remote, *rremote, *remote_port, *remote_pipeline, *remote_parent;
	struct platform_device *pdev;
	int ret;

	if (!endpoint || !conn)
		return -EINVAL;

	remote = of_graph_get_remote_endpoint(endpoint);

	rremote = of_graph_get_remote_endpoint(remote);
	if (endpoint != rremote) {
		of_node_put(rremote);
		of_node_put(remote);
		return -EINVAL; 
	}
	of_node_put(rremote);

	ret = of_property_read_u32(remote, "reg", &conn->port.link_id);
	if (ret) {
		of_node_put(remote);
		return ret;
	}

	remote_port = of_get_next_parent(remote);
	remote_pipeline = of_get_next_parent(remote_port);
	ret = of_property_read_u32(remote_pipeline, "reg", &conn->port.pipeline_id);
	if (ret) {
		of_node_put(remote_pipeline);
		of_node_put(remote);
		return ret;
	}

	remote_parent = of_get_next_parent(remote_pipeline);
	ret = dt_node_get_dpu_id_by_reg(remote_parent, &conn->port.dpu_id);
	if (ret) {
		of_node_put(remote_parent);
		return ret;
	}

	pdev = of_find_device_by_node(remote_parent);
	of_node_put(remote_parent);
	if (!pdev)
		return -EINVAL;

	ret = set_meida_noc_qos_by_id(&pdev->dev,
		conn->port.dpu_id, conn->port.pipeline_id,
		BST_BYPASS_KERNEL_QOS);
	if (ret) {
		of_node_put(remote_parent);
		return ret;
	}

	conn->host = &pdev->dev;
	return ret;
}
EXPORT_SYMBOL(bst_get_remote_dpu_connection);

int bst_get_remote_dpu_connection_by_port(struct device *dev,
		int port, struct bst_dpu_connection *conn)
{
	struct device_node *endpoint_node;
	int ret;

	endpoint_node = of_graph_get_endpoint_by_regs(dev->of_node, port, -1);
	ret = bst_get_remote_dpu_connection(endpoint_node, conn);
	of_node_put(endpoint_node);
	return ret;
}
EXPORT_SYMBOL(bst_get_remote_dpu_connection_by_port);

void bst_select_dpu_output_to_vout(struct bst_dpu_connection *conn)
{
	//printk("%s dpu:%d,pipe:%d,link:%d\n", __func__, conn->port.dpu_id, conn->port.pipeline_id, conn->port.link_id);
	bst_disp_vout_mux_sel(conn->host, conn->port.dpu_id, conn->port.pipeline_id, conn->port.link_id);
}
EXPORT_SYMBOL(bst_select_dpu_output_to_vout);

void bst_select_dpu_output_to_edp(struct bst_dpu_connection *conn)
{
	bst_disp_edp_mux_sel(conn->host, conn->port.dpu_id, conn->port.pipeline_id, conn->port.link_id);
}
EXPORT_SYMBOL(bst_select_dpu_output_to_edp);

void bst_select_dpu_output_to_lvds(struct bst_dpu_connection *conn, unsigned int lvds_n)
{
	bst_disp_lvds_mux_sel(conn->host, lvds_n, conn->port.dpu_id, conn->port.pipeline_id, conn->port.link_id);
}
EXPORT_SYMBOL(bst_select_dpu_output_to_lvds);

void bst_select_dpu_output_to_dsi(struct bst_dpu_connection *conn, unsigned int dsi_n)
{
	bst_disp_dsi_mux_sel(conn->host, dsi_n, conn->port.dpu_id, conn->port.pipeline_id, conn->port.link_id);
}
EXPORT_SYMBOL(bst_select_dpu_output_to_dsi);
