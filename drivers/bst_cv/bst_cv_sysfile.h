/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * BST_CV: Linux device driver for Black Sesame Technologies Neural Network IP
 * @author AI Tools Team, BST Ltd.
 *
 * @file 	bst_cv_sysfile.h
 * @brief This file is the header file of sysfs file interface of BST_CV
 *			  driver. It contains the declarations of misc device initialization
 *			  and cleanup functions.
 */

#ifndef BST_CV_SYSFILE_H
#define BST_CV_SYSFILE_H

#include <linux/kobject.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <asm/mman.h>

int bst_cv_sysfile_init(struct bst_cv *pbst_cv);
void bst_cv_sysfile_exit(struct bst_cv *pbst_cv);

#endif /* BST_CV_SYSFILE_H */
