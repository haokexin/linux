// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/string.h>

#include "isp_proto_base.h"

void setup_init_parti(tSoneInit *pInit, uint32_t dma_base_low,
		      uint32_t dma_base_high, char *platform, char *env,
		      uint32_t trace_mask, char *core_name, char short_name)
{
	struct parti_init_control *pCtrl = &pInit->ctrl;
	struct parti_init_syscfg *pCfg = &pInit->cfg;

	memset(pInit, 0, sizeof(tSoneInit));
	strscpy(pCtrl->magic_sone, SONE_MAGIC_SONE, sizeof(pCtrl->magic_sone));
	strscpy(pCtrl->magic_init, SONE_MAGIC_INIT, sizeof(pCtrl->magic_init));
	pCtrl->api_version = API_VERSION;
	pCtrl->doc_version = DOC_VERSION;
	pCtrl->reserve0[0] = SW_VERSION;
	pCtrl->base_addr = dma_base_low;
	pCtrl->base_addr_high = dma_base_high;

	pCtrl->init_roda0_offset_div32 =
		(unsigned char)(((void *)&pInit->rodata_resv[0] -
				 (void *)pInit) >>
				5);
	pCtrl->rodata_targ_addr_div4 = 0;

	strscpy(pCfg->corecfg.platform, platform,
		sizeof(pCfg->corecfg.platform));
	strscpy(pCfg->corecfg.host_env, env, sizeof(pCfg->corecfg.host_env));
	pCfg->corecfg.trace_mask = trace_mask;

	strscpy(pCfg->vendor.corp_name, SONE_CORP_NAME,
		SONE_MIN(sizeof(SONE_CORP_NAME),
			 sizeof(pCfg->vendor.corp_name)));
	strscpy(pCfg->vendor.IP_name, core_name,
		SONE_MIN(sizeof(core_name), sizeof(pCfg->vendor.IP_name)));
	pCfg->vendor.short_name = short_name;
}

void setup_cmdp_parti(tSoneInit *pInit, tSoneCmdp *pCmdp,
		      uint32_t cmdp_dma_base)
{
	int i;
	struct parti_init_control *pCtrl = &pInit->ctrl;
	struct parti_cmdp_ep *pEp = &pCmdp->ch[1].ep;

	pEp->ep_num = 0;
	pCmdp->hdr.chnum = 1;

	strscpy(pCmdp->hdr.magic, SONE_MAGIC_CMDP, sizeof(pCmdp->hdr.magic));
	strscpy(pCmdp->ch[0].cqueue.magic, SONE_MAGIC_CMDP_C0,
		sizeof(pCmdp->ch[0].cqueue.magic));
	strscpy(pCmdp->ch[1].cqueue.magic, SONE_MAGIC_CMDP_M0,
		sizeof(pCmdp->ch[1].cqueue.magic));

	for (i = 0; i < SONE_MAX_CHNUM; i++) {
		pCtrl->cmdp_base[i] = (uint8_t *)&pCmdp->ch[i] -
				      (uint8_t *)pCmdp + cmdp_dma_base;

		pCmdp->ch[0].cqueue.ctrl_cmd[i].p0.total_num_minus1 =
			SONE_MEDIA_QSIZE - 1;
		pCmdp->ch[1].cqueue.ctrl_cmd[i].p0.total_num_minus1 =
			SONE_MEDIA_QSIZE - 1;
	}
}

void setup_slab_parti(tSoneInit *pInit, void *pSlab, uint32_t slab_dma_base)
{
	struct parti_init_control *pCtrl;

	pCtrl = &pInit->ctrl;
	pCtrl->slab_base = slab_dma_base;
}
