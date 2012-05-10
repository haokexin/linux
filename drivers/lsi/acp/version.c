/*
 * version.c
 *
 * Copyright (C) 2010 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  021_2_6.17	 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <asm/bitops.h>

#include "../common/debug.h"
#include "../common/version.h"
#include "ncr.h"

#undef TLBERRORCOUNTER
/*#define TLBERRORCOUNTER*/
#ifdef TLBERRORCOUNTER
unsigned long dtlb_misses = 0;
unsigned long itlb_misses = 0;
#endif

extern int ubootenv_get(const char *, char *);

/*
  MODULE
*/

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Information about the Agere APP3xx");

/*
 */

unsigned agere_app3xx_revision;
EXPORT_SYMBOL(agere_app3xx_revision);

/*
  ----------------------------------------------------------------------
  timer_test_
*/

#ifdef TIMER_TEST

static void
timer_test_(void)
{
	const int first_timer_ = 0;
	const int last_timer_ = 7;
	int timer_;
	unsigned long timer_base_[] = {
		TIMER0_BASE, TIMER1_BASE, TIMER2_BASE, TIMER3_BASE,
		TIMER4_BASE, TIMER5_BASE, TIMER6_BASE, TIMER7_BASE
	};

	{
		const int number_of_reads_ = 200;
		unsigned long time_[number_of_reads_];
		int index_;

		for (index_ = 0; index_ < number_of_reads_; ++index_)
			time_[index_] = ~(readl((TIMER0_BASE + TIMER_n_VALUE)));

		for (index_ = 0; index_ < number_of_reads_; ++index_) {
			printk(KERN_INFO "%03d : 0x%08x\n", index_, time_[index_]);

			if (0 < index_) {
				if (time_[index_] <= time_[(index_ - 1)]) {
					printk(KERN_ERR "WHAT!\n");
				}
			}
		}
	}

	for (timer_ = first_timer_; timer_ <= last_timer_; ++timer_) {
		unsigned long base_ = timer_base_[timer_];

		printk(KERN_INFO "load value=0x%x control=0x%x background load=0x%x\n",
			readl((base_ + TIMER_n_LOAD)),
			readl((base_ + TIMER_n_CONTROL)),
			readl((base_ + TIMER_n_BACKGROUND_LOAD)));
	}
}

#endif /* TIMER_TEST */

#ifdef SHMEM_TEST

/*
  ----------------------------------------------------------------------
  shmem_test
*/

void
shmem_test(void)
{
	void __iomem *cached_shmem;
	void __iomem *uncached_shmem;
	unsigned i;
	unsigned long start_time;

	uncached_shmem = ioremap_nocache(APP3K_PHYS_SHMEM_BASE, PAGE_SZ);
	cached_shmem = ioremap_cached(APP3K_PHYS_SHMEM_BASE + PAGE_SZ,
				       APP_SHMEM_SIZE - PAGE_SZ);

	if ((void __iomem *) 0 == cached_shmem) {
		printk(KERN_ERR "Unable to map shmem as cached IO\n");
		return;
	}

	printk("cached_shmem=0x%08p shmem=0x%08x uncached_shmem=0x%08p\n",
		cached_shmem, APP_SHMEM_BASE, uncached_shmem);

#if 0
	i = 0;
	start_time = jiffies;
	while (jiffies == start_time)
		;
	start_time = jiffies;
	while (jiffies == start_time) {

		unsigned j;
		unsigned char * dest = (unsigned char *) APP_SHMEM_BASE;

		for (j = 0; j < APP_SHMEM_SIZE; ++j, ++dest) {
			*dest = j;
		}

		++i;
	}

	printk("%d iterations uncached\n", i);

	i = 0;
	start_time = jiffies;
	while (jiffies == start_time)
		;
	start_time = jiffies;
	while (jiffies == start_time) {
		unsigned j;
		unsigned char * dest = (unsigned char *) cached_shmem;

		for (j = 0; j < APP_SHMEM_SIZE; ++j, ++dest) {
			*dest = j;
		}

		++i;
	}

	printk(KERN_INFO "%d iterations cached\n", i);
#endif

	return;
}

#endif /* SHMEM_TEST */

#ifdef TLBERRORCOUNTER

static int
tlberror_create_string(char *buffer)
{
	int length = 0;

	length = sprintf(buffer,
			 "Data TLB Errors: %d\n"
			 "Instruction TLB Errors: %d\n",
			 dtlb_misses, itlb_misses);

	return length;
}

/*
  ----------------------------------------------------------------------
  tlberror_read_proc
*/

static int
tlberror_read_proc(char *page, char **start, off_t offset,
		   int count, int *eof, void *data)
{
	int length = 0;

	length = tlberror_create_string(page);
	*eof = 1;

	return length;
}

#endif /* TLBERRORCOUNTER */

/*
  ------------------------------------------------------------------------------
  is_asic
*/

int
is_asic(void)
{
#ifdef CONFIG_ACPISS
	return 0;
#else
#if 1
	unsigned long nca_config;

	if (0 == ncr_read(NCP_REGION_ID(0x16, 0xff), 0x10, 4, &nca_config)) {
		return (0 == (nca_config & 0x80000000));
	}

	return -1;
#else
	printk("%s:%s:%d - Writing 0x%x to 0x%x (NCA=0x%x)\n",
		__FILE__, __FUNCTION__, __LINE_,
		value, address, NCA); /* ZZZ */
	return 0;
#endif
#endif
}

/*
  ----------------------------------------------------------------------
  info_create_string
*/

static int
info_create_string(char *buffer)
{
	int length = 0;
	char uboot_version2[80];
	char uboot_version3[80];

#ifdef CONFIG_ACPISS
	sprintf(uboot_version2, "Unknown");
	sprintf(uboot_version3, "Unknown");
#else
	if (0 != ubootenv_get("version2", uboot_version2)) {
		sprintf(uboot_version2, "Unknown");
	}

	if (0 != ubootenv_get("version3", uboot_version3)) {
		sprintf(uboot_version3, "Unknown");
	}
#endif

	length = sprintf(buffer,
			 "LSI ACP Info\n" \
			 "LSI Version %s\n" \
			 "Platform: %s\n" \
			 "U-Boot Version (2nd Stage) %s\n" \
			 "U-Boot Version (3rd Stage) %s\n",
			 ACP_VERSION, (is_asic() ? "ASIC" : "FPGA"),
			 uboot_version2, uboot_version3);

	/* that's all */
	return length;

}

/*
  ----------------------------------------------------------------------
  info_read_proc
*/

static int
info_read_proc(char *page, char **start, off_t offset,
		int count, int *eof, void *data)
{
	int length_ = 0;

#ifdef SHMEM_TEST
	shmem_test();
#endif /* SHMEM_TEST */

#ifdef TIMER_TEST
	timer_test_();
#endif /* TIMER_TEST */

#if !defined(SHMEM_TEST) && !defined(TIMER_TEST)
	length_ = info_create_string(page);
#endif
	*eof = 1;

	return length_;
}

/*
  ----------------------------------------------------------------------
  info_module_init
*/

int __init
info_module_init(void)
{

	char buffer_[256];

	create_proc_read_entry("driver/version", 0, NULL, info_read_proc, NULL);
	info_create_string(buffer_);
	printk("%s", buffer_);

#ifdef TLBERRORCOUNTER
	create_proc_read_entry("driver/tlberrors",
			       0, NULL, tlberror_read_proc, NULL);
#endif

	return 0;

}

module_init(info_module_init);

/*
  ----------------------------------------------------------------------
  info_module_exit
*/

void __exit
info_module_exit(void)
{
#ifdef TLBERRORCOUNTER
	remove_proc_entry("driver/tlberrors", NULL);
#endif
	remove_proc_entry("driver/version", NULL);

	return;
}

module_exit(info_module_exit);
