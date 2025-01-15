#ifndef __SEMAPHORE_H__
#define __SEMAPHORE_H__

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

uint32_t scmi_get_sem_lock(void);
uint32_t scmi_release_sem_lock(void);

int scmi_sem_lock_init(void);
#endif