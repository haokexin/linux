
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include "semaphore.h"


#define SEM_BANK_ID0 (0U)
#define MST_ID_SAFETY_LOCK (4U)
#define SCMI_SEM_ID (1U)

static void __iomem *scmi_ipc_sem_base;

uint32_t scmi_get_sem_lock(void)
{
    uint32_t data;
    UN_REG_SEM sem_reg;

    if(scmi_ipc_sem_base == NULL){
        return -1;
    }

    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = SCMI_SEM_ID;
    sem_reg.bit.bank_id = SEM_BANK_ID0;
    sem_reg.bit.mst_id = MST_ID_SAFETY_LOCK;

    do {
        
        data = readl_relaxed(scmi_ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));

    } while (data != 0);


    return data;
}


uint32_t scmi_release_sem_lock(void)
{
    UN_REG_SEM sem_reg;

    if(scmi_ipc_sem_base == NULL){
        return -1;
    }


    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = SCMI_SEM_ID;
    sem_reg.bit.bank_id = SEM_BANK_ID0;
    sem_reg.bit.mst_id = MST_ID_SAFETY_LOCK;

    writel_relaxed(1, scmi_ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));

    return 0;
}


int scmi_sem_lock_init(void){

    scmi_ipc_sem_base = ioremap(IPC_SEM_BASE_ADDR, IPC_SEM_SIZE);
    if (scmi_ipc_sem_base == NULL) {
        printk("ioremap for ipc_sem_base failed\n");
        return -1;
    }

    return 0;
}