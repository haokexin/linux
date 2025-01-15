// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/coreip/proto_api_common.h>

#define VSP_SONE_MEDIA_QSIZE 32

void setup_init_parti(tSoneInit *pInit, uint32_t base, uint32_t base_high,
		      char *platform, char *env, uint32_t traceMask,
		      char *core_name, char short_name)
{
	struct parti_init_control *pCtrl = &pInit->ctrl;
	struct parti_init_syscfg *pCfg = &pInit->cfg;

	memset(pInit, 0, sizeof(tSoneInit));
	strncpy(pCtrl->magic_sone, SONE_MAGIC_SONE, 4);
	strncpy(pCtrl->magic_init, SONE_MAGIC_INIT, 4);
	pCtrl->api_version = API_VERSION;
	pCtrl->doc_version = DOC_VERSION;

	pCtrl->reserve0[0] = SW_VERSION;
	pCtrl->base_addr = base;
	pCtrl->base_addr_high = base_high;

	// TODO:: overflow, should be correct
	pCtrl->init_roda0_offset_div32 =
		(unsigned char)(((void *)&pInit->rodata_resv[0] -
				 (void *)pInit) >>
				5);
	pCtrl->rodata_targ_addr_div4 = 0;

	strncpy(pCfg->vendor.corp_name, SONE_CORP_NAME,
		SONE_MIN(sizeof(SONE_CORP_NAME),
			 sizeof(pCfg->vendor.corp_name)));
	strncpy(pCfg->vendor.IP_name, core_name,
		SONE_MIN(sizeof(core_name), sizeof(pCfg->vendor.IP_name)));
	pCfg->vendor.short_name = short_name;

	strncpy(pCfg->corecfg.platform, platform, 4);
	strncpy(pCfg->corecfg.host_env, env, 4);
}

void setup_cmdp_parti(tSoneInit *pInit, tSoneCmdp *pCmdp,
		      uint32_t cmdp_phy_base)
{
	struct parti_init_control *pCtrl = &pInit->ctrl;
	struct parti_cmdp_ep *pEp = &pCmdp->ch[1].ep;
	int i;

	pEp->ep_num = 0;
	pCmdp->hdr.chnum = 1;

	strncpy(pCmdp->hdr.magic, SONE_MAGIC_CMDP, 4);
	strncpy(pCmdp->ch[0].cqueue.magic, SONE_MAGIC_CMDP_C0, 4);
	strncpy(pCmdp->ch[1].cqueue.magic, SONE_MAGIC_CMDP_M0, 4);

	for (i = 0; i < SONE_MAX_CHNUM; i++) {
		pCtrl->cmdp_base[i] = 0x80000000 + (uint8_t *)&pCmdp->ch[i] -
				      (uint8_t *)pCmdp + cmdp_phy_base;
		pr_info("%s: cmdp_base %d addr: 0x%08X", __func__, i,
			pCtrl->cmdp_base[i]);

		pCmdp->ch[0].cqueue.ctrl_cmd[i].p0.total_num_minus1 =
			VSP_SONE_MEDIA_QSIZE - 1;
		pCmdp->ch[1].cqueue.ctrl_cmd[i].p0.total_num_minus1 =
			VSP_SONE_MEDIA_QSIZE - 1;
	}
}
EXPORT_SYMBOL(setup_cmdp_parti);

void setup_slab_parti(tSoneInit *pInit, void *pSlab, uint32_t slab_phy_base)
{
	struct parti_init_control *pCtrl = &pInit->ctrl;

	pCtrl->slab_base = 0x80000000 + slab_phy_base;
	pr_info("%s: slab_base addr: 0x%08X\n", __func__, pCtrl->slab_base);
}
EXPORT_SYMBOL(setup_slab_parti);

void setup_fbuf_parti(tSoneInit *pInit, void *pFbuf, uint32_t fbuf_phy_base,
		      char *name)
{
	struct parti_init_control *pCtrl = &pInit->ctrl;

	pCtrl->fbuf_num = 1;
	pCtrl->fbuf_table[0] = fbuf_phy_base;
}
EXPORT_SYMBOL(setup_fbuf_parti);
