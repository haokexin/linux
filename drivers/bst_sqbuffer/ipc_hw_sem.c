// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#include "ipc_hw_sem.h"

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>

#define SHM_SEM_ID (15U)
#define SEM_BANK_ID0 (0U)
#define MST_ID_LOCK (1U)

uint32_t get_sq_sem_lock(void __iomem *ipc_sem_base, uint8_t sem_id)
{
    uint32_t data;
    UN_REG_SEM sem_reg;
    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = sem_id;
    sem_reg.bit.bank_id = SEM_BANK_ID0;
    sem_reg.bit.mst_id = MST_ID_LOCK;

    do {
        data = readl_relaxed(ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));
    } while (data != 0);

    return data;
}

uint32_t release_sq_sem_lock(void __iomem *ipc_sem_base, uint8_t sem_id)
{
    UN_REG_SEM sem_reg;

    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = sem_id;
    sem_reg.bit.bank_id = SEM_BANK_ID0;
    sem_reg.bit.mst_id = MST_ID_LOCK;

    writel_relaxed(1, ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));

    return 0;
}