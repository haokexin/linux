/*
 * Created by: Jason Wessel <jason.wessel@windriver.com>
 *
 * Copyright (c) 2009 Wind River Systems, Inc.  All Rights Reserved.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifdef CONFIG_VT
#include <linux/kgdb.h>
#include <linux/console.h>
#include <linux/vt_kern.h>
#include <linux/selection.h>
#include <linux/kdb.h>
#include "kdb/kdb_private.h"

static int dbg_orig_vc_mode;
static int saved_fg_con;
static int saved_last_con;
static int saved_want_con;

void dbg_pre_vt_hook(void)
{
	struct vc_data *vc = vc_cons[fg_console].d;
	saved_fg_con = fg_console;
	saved_last_con = last_console;
	saved_want_con = want_console;
	dbg_orig_vc_mode = vc->vc_mode;
	vc->vc_mode = KD_TEXT;
	console_blanked = 0;
	vc->vc_sw->con_blank(vc, 0, 1);
	vc->vc_sw->con_set_palette(vc, color_table);
#ifdef CONFIG_KGDB_KDB
	/* Set the initial LINES variable if it is not already set */
	if (vc->vc_rows < 999) {
		int linecount;
		char lns[4];
		const char *setargs[3] = {
			"set",
			"LINES",
			lns,
		};
		if (kdbgetintenv(setargs[0], &linecount)) {
			snprintf(lns, 4, "%i", vc->vc_rows);
			kdb_set(2, setargs);
		}
	}
#endif /* CONFIG_KGDB_KDB */
}
EXPORT_SYMBOL_GPL(dbg_pre_vt_hook);

void dbg_post_vt_hook(void)
{
	fg_console = saved_fg_con;
	last_console = saved_last_con;
	want_console = saved_want_con;
	vc_cons[fg_console].d->vc_mode = dbg_orig_vc_mode;
}
EXPORT_SYMBOL_GPL(dbg_post_vt_hook);
#endif /* CONFIG_VT */
