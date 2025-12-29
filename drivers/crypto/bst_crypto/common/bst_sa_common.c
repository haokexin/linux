
#include "bst_sa_common.h"
#include <linux/printk.h>


#define MAX_HEX_LINE_LEN 2048

int bst_debug = 0;
int bst_debug_level = 6;
int bst_sec_sa_hfe_enable = 0;
int bst_sec_sa_ske_enable = 0;
int bst_sec_sa_pke_enable = 0;

module_param(bst_debug, int, 0644);
module_param(bst_debug_level, int, 0644);

MODULE_PARM_DESC(bst_debug, "Enable or disable debug");
MODULE_PARM_DESC(bst_debug_level, "Set debug verbosity level");

EXPORT_SYMBOL_GPL(bst_debug);
EXPORT_SYMBOL_GPL(bst_debug_level);
EXPORT_SYMBOL_GPL(bst_sec_sa_hfe_enable);
EXPORT_SYMBOL_GPL(bst_sec_sa_ske_enable);
EXPORT_SYMBOL_GPL(bst_sec_sa_pke_enable);


void sa_enbale_change_hfe(void);
void sa_enbale_change_ske(void);
void sa_enbale_change_pke(void);

static int bst_sec_sa_set_hfe_enable(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (bst_sec_sa_hfe_enable)
		pr_info("BST SEC SA: enabling hfe algorithms\n");
	else
		pr_info("BST SEC SA: disabling hfe algorithms\n");

	sa_enbale_change_hfe();

	return 0;
}

static int bst_sec_sa_set_ske_enable(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (bst_sec_sa_ske_enable)
		pr_info("BST SEC SA: enabling ske algorithms\n");
	else
		pr_info("BST SEC SA: disabling ske algorithms\n");

	sa_enbale_change_ske();

	return 0;
}

static int bst_sec_sa_set_pke_enable(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (bst_sec_sa_pke_enable)
		pr_info("BST SEC SA: enabling pke algorithms\n");
	else
		pr_info("BST SEC SA: disabling pke algorithms\n");

	sa_enbale_change_pke();

	return 0;
}

static const struct kernel_param_ops bst_sec_sa_hfe_enable_ops = {
	.set = bst_sec_sa_set_hfe_enable,
	.get = param_get_int,
};

static const struct kernel_param_ops bst_sec_sa_ske_enable_ops = {
	.set = bst_sec_sa_set_ske_enable,
	.get = param_get_int,
};

static const struct kernel_param_ops bst_sec_sa_pke_enable_ops = {
	.set = bst_sec_sa_set_pke_enable,
	.get = param_get_int,
};


void __attribute__((unused)) printHex_old(const char *prompt,const uint8_t *data, uint32_t len)
{
    char *buf;
    uint32_t i, pos = 0;

	if(bst_debug == 0){
		return;
	}

    buf = kmalloc(MAX_HEX_LINE_LEN, GFP_KERNEL);
    if (!buf)
        return;

    pos += scnprintf(buf + pos, MAX_HEX_LINE_LEN - pos, "%s len %d - ", prompt, len);
	if( len != 0 ){
        pos += scnprintf(buf + pos, MAX_HEX_LINE_LEN - pos, "%02x", data[0]);
	}
    for (i = 1; i < len && pos < MAX_HEX_LINE_LEN - 4; i++) {
        pos += scnprintf(buf + pos, MAX_HEX_LINE_LEN - pos, ":%02x", data[i]);
    }

    bst_dbg(3, "%s\n", buf);

    bst_kfree(buf);
}

void __attribute__((unused)) printHex(const char *prompt, const uint8_t *data, uint32_t len)
{
    if (bst_debug == 0 || !data || len == 0)
        return;
    
    pr_info("%s len %u:\n", prompt, len);
    
    print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, data, len, true);
}

module_param_cb(bst_sec_sa_hfe_enable, &bst_sec_sa_hfe_enable_ops, &bst_sec_sa_hfe_enable, 0644);
MODULE_PARM_DESC(bst_sec_sa_hfe_enable, "Enable BST SEC STANDALONE hfe algorithms dynamically (0=off, 1=on)");
module_param_cb(bst_sec_sa_ske_enable, &bst_sec_sa_ske_enable_ops, &bst_sec_sa_ske_enable, 0644);
MODULE_PARM_DESC(bst_sec_sa_ske_enable, "Enable BST SEC STANDALONE ske algorithms dynamically (0=off, 1=on)");
module_param_cb(bst_sec_sa_pke_enable, &bst_sec_sa_pke_enable_ops, &bst_sec_sa_pke_enable, 0644);
MODULE_PARM_DESC(bst_sec_sa_pke_enable, "Enable BST SEC STANDALONE pke algorithms dynamically (0=off, 1=on)");

