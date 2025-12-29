// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * BSTN: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file 	bstn_sysfile.h
 * @brief 	This file is the header file of sysfs file interface of BSTN
 *			driver. It contains the declarations of misc device initialization
 *			and cleanup functions.
 */

#ifndef BSTN_SYSFILE_H
#define BSTN_SYSFILE_H

#include <linux/kobject.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>

int bstn_sysfile_init(struct bstn_device *pbstn);
void bstn_sysfile_exit(struct bstn_device *pbstn);

#endif
