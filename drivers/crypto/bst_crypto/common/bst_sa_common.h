
#ifndef _BST_SA_COMMON_H_
#define _BST_SA_COMMON_H_

#include <linux/module.h>
#include <linux/kernel.h>

#define bst_kfree(buf) if(buf != NULL){ \
						kfree(buf); \
						buf = NULL; \
					}

extern int bst_debug;
extern int bst_debug_level;
extern int bst_sec_sa_hfe_enable;
extern int bst_sec_sa_ske_enable;
extern int bst_sec_sa_pke_enable;

#define bst_dbg(level, fmt, ...) \
	do { \
		if (bst_debug && (level) <= bst_debug_level) \
			printk(KERN_INFO "[BST_SA] " fmt, ##__VA_ARGS__); \
	} while (0)

#endif

void __attribute__((unused)) printHex(const char *prompt,const uint8_t *data, uint32_t len);