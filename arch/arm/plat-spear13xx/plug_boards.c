/*
 * arch/arm/mach-spear13xx/plug_boards_common.c
 *
 * SPEAr plug boards common source file
 *
 * Copyright (C) 2011-12 ST Microelectronics
 * Viresh Kumar <viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/*
 * This file includes all the common function to be used by all the plug
 * board files.
 * Plug boards allow the different standard test and debug procedures on the
 * same physical interface without the need to develop a specific board and use
 * a different manufacturer’s chips. They are connected to the main board
 * through small high speed shielded connectors to avoid quality degradation of
 * the signals. Each plug has the interface connectors on different positions to
 * prevent any insertions errors.
 *
 * The philosophy picked for maintaining them in software is as follows. Firstly
 * device arrays will be prepared in respective evb file and these will be
 * passed to plug board init routine, if plug boards are passed from bootargs.
 * This routine may:
 * - override the padmux settings done earlier by evb board
 * - skip registeration of some devices passed from evb_init()
 * - add new amba/plat devs, etc devices
 * - If multiple boards are passed from bootargs, then boards will be
 *   initialized in the order they are mentioned in bootargs.
 *
 * Passing param from bootargs
 * ---------------------------
 * "pb=" must be used to pass plug boards request from bootargs. This variable
 * can contain string values mentioned above in board descriptions"
 *
 * More than one board can be requested by passing ',' separated board list, eg:
 * bootargs: console=ttyAMA0,115200 pb=rgmii,hdmi_tx,cam0
 */


#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <plat/plug_board.h>
#include <mach/generic.h>
#include <mach/hardware.h>

/* string specifying which plug boards are requested */
static char __initdata spear_plug_board[MAX_REQ_PB] = {'\0', };

static int __init spear_pb_select(char *boards)
{
	strcpy(spear_plug_board, boards);

	return 0;
}
__setup("pb=", spear_pb_select);

static bool skip_device(struct list_head *pb_list, void *dev,
		enum skip_device_type skip)
{
	struct plug_board *pb;
	int i;

	list_for_each_entry(pb, pb_list, node) {
		switch (skip) {
		case SKIP_PLAT_DEVICE:
			for (i = 0; i < pb->rm_pcnt; i++) {
				if (dev == pb->rm_pdevs[i]) {
					pr_debug("%s: skip %s.%d\n",
						pb->name, pb->rm_pdevs[i]->name,
						pb->rm_pdevs[i]->id == -1 ? 0 :
						pb->rm_pdevs[i]->id);
					return true;
				}
			}
			break;
		case SKIP_AMBA_DEVICE:
			for (i = 0; i < pb->rm_acnt; i++) {
				if (dev == pb->rm_adevs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
						pb->rm_adevs[i]->dev.init_name);
					return true;
				}
			}
			break;
		case SKIP_SPI_DEVICE:
			for (i = 0; i < pb->rm_spi_cnt; i++) {
				if (dev == pb->rm_spi_devs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
						pb->rm_spi_devs[i]->modalias);
					return true;
				}
			}
			break;
		case SKIP_I2C_DEVICE:
			for (i = 0; i < pb->rm_i2c_cnt; i++) {
				if (dev == pb->rm_i2c_devs[i]) {
					pr_debug("%s: skip %s\n", pb->name,
					pb->rm_i2c_devs[i]->board->type);
					return true;
				}
			}
			break;

		default:
			return false;
		}
	}

	return false;
}

bool spear_pb_present(void)
{
	if (spear_plug_board[0] != '\0')
		return true;
	else
		return false;
}

char *get_spear_pb(void)
{
	static char *str = spear_plug_board;

	return strsep((char **)&str, ",");
}

int __init spear_pb_init(struct plug_board_info *pb_info,
		int (*make_pb_list)(struct list_head *pb_list))
{
	struct platform_device **pdevs;
	struct amba_device **adevs;
	struct plug_board *pb;
	struct spi_board_info **spi_devs;
	struct i2c_dev_info **i2c_devs;
	int ret, i;
	u8 pcnt, acnt, spi_cnt, i2c_cnt;
	LIST_HEAD(pb_list);

	if (!pb_info)
		return -EINVAL;

	pr_debug("%s: Plug board string passed from bootargs: %s\n", __func__,
			spear_plug_board);

	pdevs = pb_info->pdevs;
	pcnt = pb_info->pcnt;
	adevs = pb_info->adevs;
	acnt = pb_info->acnt;
	spi_devs = pb_info->spi_devs;
	spi_cnt = pb_info->spi_cnt;
	i2c_devs = pb_info->i2c_devs;
	i2c_cnt = pb_info->i2c_cnt;

	ret = (*make_pb_list)(&pb_list);
	if (ret) {
		pr_err("Error creating pb_list: %d\n", ret);
		return ret;
	}

	/* Call board specific init routine */
	list_for_each_entry(pb, &pb_list, node) {
		pr_debug("%s: Initializing plug board\n", pb->name);

		if (pb->pb_init)
			pb->pb_init();
	}

	list_for_each_entry(pb, &pb_list, node) {
		if (!pb->pmx_cnt)
			continue;

		ret = pmx_devs_enable(pb->pmx_devs, pb->pmx_cnt);
		if (ret)
			pr_err("padmux: Failed adding pmx devs: %d\n", ret);

		pr_debug("%s: Added %d pmx devices\n", pb->name, pb->pmx_cnt);
	}

	/* Add SPI Devices passed from evb.c */
	for (i = 0; i < spi_cnt; i++) {
		if (skip_device(&pb_list, spi_devs[i], SKIP_SPI_DEVICE))
			continue;

		spi_register_board_info(spi_devs[i], 1);
	}

	/* Add SPI Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_spi_cnt; i++)
			spi_register_board_info(pb->add_spi_devs[i], 1);

		pr_debug("%s: Added %d SPI devices\n",
				pb->name, pb->add_spi_cnt);
	}

	/* Add I2C Devices passed from evb.c */
	for (i = 0; i < i2c_cnt; i++) {
		if (skip_device(&pb_list, i2c_devs[i], SKIP_I2C_DEVICE))
			continue;

		i2c_register_board_info(i2c_devs[i]->busnum,
				i2c_devs[i]->board, 1);
	}

	/* Add I2C Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_i2c_cnt; i++)
			i2c_register_board_info(pb->add_i2c_devs[i]->busnum,
				pb->add_i2c_devs[i]->board, 1);

		pr_debug("%s: Added %d I2C devices\n",
				pb->name, pb->add_i2c_cnt);
	}

	/* Add Amba Devices passed from evb.c */
	for (i = 0; i < acnt; i++) {
		if (skip_device(&pb_list, adevs[i], SKIP_AMBA_DEVICE))
			continue;

		amba_device_register(adevs[i], &iomem_resource);
	}

	/* Add Amba Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_acnt; i++)
			amba_device_register(pb->add_adevs[i], &iomem_resource);

		pr_debug("%s: Added %d amba devices\n", pb->name, pb->add_acnt);
	}

	/* Add Platform Devices passed from evb.c */
	for (i = 0; i < pcnt; i++) {
		if (skip_device(&pb_list, pdevs[i], SKIP_PLAT_DEVICE))
			continue;

		platform_device_register(pdevs[i]);
	}

	/* Add Platform Devices requested by plug boards */
	list_for_each_entry(pb, &pb_list, node) {
		for (i = 0; i < pb->add_pcnt; i++)
			platform_device_register(pb->add_pdevs[i]);

		pr_debug("%s: Added %d plat devices\n", pb->name, pb->add_pcnt);
	}

	list_for_each_entry(pb, &pb_list, node)
		kfree(pb);

	return 0;
}
