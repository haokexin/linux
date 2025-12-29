// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file 	bstn_misc.h
 * @brief 	This file is the header file of misc device interface of BSTN
 *			driver. It contains the declarations of misc device initialization
 *			and cleanup functions.
 */

#ifndef BSTN_MISC_H
#define BSTN_MISC_H

#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>

int bstn_misc_init(struct bstn_device *pbstn);
int bstn_misc_exit(struct bstn_device *pbstn);

#endif
