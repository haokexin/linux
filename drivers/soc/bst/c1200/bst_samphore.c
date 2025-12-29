
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/delay.h> 
#include <linux/jiffies.h>  
#include <linux/errno.h>

#include <linux/bst_samphore.h>

#define IPC_SEM_BASE_ADDR (0x30100000)
#define IPC_SEM_SIZE (0x100000)
#define SEM_BANK_ID0 (0U)


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




int get_sem_lock(struct bst_samphore * samphore)
{
    union un_reg_sem sem_reg;
    unsigned int data;

    if(samphore == NULL || samphore->ipc_sem_base == NULL){
        return -1;
    }

    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = samphore->msg_id;
    sem_reg.bit.bank_id = SEM_BANK_ID0;
    sem_reg.bit.mst_id = samphore->mst_id;

    do {
        
        data = readl_relaxed(samphore->ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));

    } while (data != 0);

    return 0;
}
EXPORT_SYMBOL(get_sem_lock);

/**
 * read_register_with_timeout 
 * @samphore: bst_samphore
 * @timeout_ms: TIME MS
 *
 * return：
 *   success:data
 *   fail:mst_id
 */
int get_sem_lock_with_timeout(struct bst_samphore * samphore,unsigned int timeout_ms)
{
    union un_reg_sem sem_reg;
    unsigned int data;
    unsigned long timeout_jiffies = msecs_to_jiffies(timeout_ms);
    unsigned long start_jiffies = jiffies;

    if(samphore == NULL || samphore->ipc_sem_base == NULL){
        return -1;
    }

    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = samphore->msg_id;
    sem_reg.bit.bank_id = samphore->bank_id;
    sem_reg.bit.mst_id = samphore->mst_id;

    do {
        data = readl_relaxed(samphore->ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));
        if (time_after(jiffies, start_jiffies + timeout_jiffies)) {
            return data;
        }

    } while (data != 0);

    return 0;
}
EXPORT_SYMBOL(get_sem_lock_with_timeout);

int release_bst_sem_lock(struct bst_samphore * samphore)
{
   union un_reg_sem sem_reg;

    if(samphore == NULL || samphore->ipc_sem_base == NULL){
        return -1;
    }

    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = samphore->msg_id;
    sem_reg.bit.bank_id = samphore->bank_id;
    sem_reg.bit.mst_id = samphore->mst_id;

    writel_relaxed(1, samphore->ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));

    return 0;
}
EXPORT_SYMBOL(release_bst_sem_lock);

int release_sem_lock(struct bst_samphore * samphore)
{
   union un_reg_sem sem_reg;

    if(samphore == NULL || samphore->ipc_sem_base == NULL){
        return -1;
    }

    sem_reg.data = IPC_SEM_BASE_ADDR;
    sem_reg.bit.sem_id = samphore->msg_id;
    sem_reg.bit.bank_id = SEM_BANK_ID0;
    sem_reg.bit.mst_id = samphore->mst_id;

    writel_relaxed(1, samphore->ipc_sem_base + (sem_reg.data - IPC_SEM_BASE_ADDR));

    return 0;
}
EXPORT_SYMBOL(release_sem_lock);


int samphore_lock_remove(struct bst_samphore * samphore){

    if(samphore == NULL || samphore->ipc_sem_base == NULL){
        return -1;
    }

    iounmap(samphore->ipc_sem_base);
    kfree(samphore);

    return 0;
}
EXPORT_SYMBOL(samphore_lock_remove);


struct bst_samphore *  samphore_lock_init(enum MST_ID mst_id,enum MSG_ID msg_id){
    struct bst_samphore * samphore = NULL;

    samphore = kzalloc(sizeof(struct bst_samphore), GFP_KERNEL);
	if (!samphore)
		return NULL;

    samphore->mst_id = mst_id;
    samphore->msg_id = msg_id;

    samphore->ipc_sem_base =  ioremap(IPC_SEM_BASE_ADDR, IPC_SEM_SIZE);
    if (samphore->ipc_sem_base == NULL) {
       kfree(samphore);
        return NULL;
    }

    return samphore;
}
EXPORT_SYMBOL(samphore_lock_init);

struct bst_samphore * bst_semaphore_init(enum MST_ID mst_id,enum BANK_ID bank_id,enum MSG_ID msg_id){
    struct bst_samphore * samphore = NULL;

    samphore = kzalloc(sizeof(struct bst_samphore), GFP_KERNEL);
	if (!samphore)
		return NULL;

    samphore->mst_id = mst_id;
    samphore->msg_id = msg_id;
    samphore->bank_id = bank_id;
    
    samphore->ipc_sem_base =  ioremap(IPC_SEM_BASE_ADDR, IPC_SEM_SIZE);
    if (samphore->ipc_sem_base == NULL) {
       kfree(samphore);
        return NULL;
    }

    return samphore;
}
EXPORT_SYMBOL(bst_semaphore_init);