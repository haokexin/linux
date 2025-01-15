// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef IPC_HW_SEM_H
#define IPC_HW_SEM_H

#include <linux/workqueue.h>

#define IPC_SEM_BASE_ADDR (0x30100000)
#define IPC_SEM_SIZE (0x100000)

union un_reg_sem {
    struct {
        u32 res : 2;
        u32 sem_id : 4;
        u32 intr : 1;
        u32 bank_id : 2;
        u32 mst_id : 4;
        u32 base_addr : 19;
    } bit;
    u32 data;
};
#define UN_REG_SEM union un_reg_sem

uint32_t get_sq_sem_lock(void __iomem *ipc_sem_base, uint8_t sem_id);
uint32_t release_sq_sem_lock(void __iomem *ipc_sem_base, uint8_t sem_id);

#endif  // IPC_HW_SEM_H
