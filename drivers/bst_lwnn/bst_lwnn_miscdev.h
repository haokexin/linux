// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * Copyright (c) 2024 Black Sesame Technologies
 */

/*!
 * bst_lwnn: Linux device driver for Black Sesame Technologies Computer Vision IP
 * @author: AI Tools Team, BST Ltd.
 *
 * @file    bst_lwnn_misc.h
 * @brief   This file is the header file of misc device interface of bst_lwnn
 *          driver. It contains the declarations of misc device initialization
 *          and exit functions.
 */

#ifndef BST_LWNN_MISCDEV_H
#define BST_LWNN_MISCDEV_H

int bst_lwnn_miscdev_init(struct bst_lwnn *pbst_lwnn);
void bst_lwnn_miscdev_exit(struct bst_lwnn *pbst_lwnn);

#endif
