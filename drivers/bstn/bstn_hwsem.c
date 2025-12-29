#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>

#include "bstn_hwsem.h"

#define _BSTN_HWSEM_BASE_ADDR_  (0x30100000)
#define _BSTN_HWSEM_SIZE_       (0x100000)

#define _BSTN_SEM_ID_           (0)
#define _BSTN_BANK_ID_          (1)
#define _BSTN_MST_ID_           (1)

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

struct bstn_hwsem *bstn_hwsem_init(void)
{
    struct bstn_hwsem *hwsem = kzalloc(sizeof(struct bstn_hwsem), GFP_KERNEL);
    if (!hwsem)
    {
        return NULL;
    }

    hwsem->sem_base = ioremap(_BSTN_HWSEM_BASE_ADDR_, _BSTN_HWSEM_SIZE_);
    if (NULL == hwsem->sem_base)
    {
        kfree(hwsem);
        return NULL;
    }

    return hwsem;
}

int bstn_hwsem_uninit(struct bstn_hwsem *hwsem)
{
    if(NULL == hwsem)
    {
        return -1;
    }

    if(NULL == hwsem->sem_base)
    {
        kfree(hwsem);
        return -2;
    }

    bstn_hwsem_release(hwsem);

    iounmap(hwsem->sem_base);
    kfree(hwsem);

    return 0;
}

int bstn_hwsem_get(struct bstn_hwsem *hwsem, uint32_t *val)
{
    union un_reg_sem sem_reg = {0};
    uint32_t data = 0;

    if((NULL == hwsem) || (NULL == hwsem->sem_base))
    {
        return -1;
    }

    sem_reg.data        = _BSTN_HWSEM_BASE_ADDR_;
    sem_reg.bit.sem_id  = _BSTN_SEM_ID_;
    sem_reg.bit.bank_id = _BSTN_BANK_ID_;
    sem_reg.bit.mst_id  = _BSTN_MST_ID_;

    data = readl_relaxed(hwsem->sem_base + (sem_reg.data - _BSTN_HWSEM_BASE_ADDR_));
    if(NULL != val)
    {
        *val = data;
    }

    if(0 == data)  /* get lock success */
    {
        return 1;
    }
    else           /* get lock failled */
    {
        return 0;
    }
}

int32_t bstn_hwsem_release(struct bstn_hwsem *hwsem)
{
    union un_reg_sem sem_reg = {0};
    if((NULL == hwsem) || (NULL == hwsem->sem_base))
    {
        return -1;
    }

    sem_reg.data        = _BSTN_HWSEM_BASE_ADDR_;
    sem_reg.bit.sem_id  = _BSTN_SEM_ID_;
    sem_reg.bit.bank_id = _BSTN_BANK_ID_;
    sem_reg.bit.mst_id  = _BSTN_MST_ID_;

    writel_relaxed(1, hwsem->sem_base + (sem_reg.data - _BSTN_HWSEM_BASE_ADDR_));
    return 0;
}

