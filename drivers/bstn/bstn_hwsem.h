#ifndef _BSTN_HWSEM_H_
#define _BSTN_HWSEM_H_

struct bstn_hwsem {
    void __iomem *sem_base;
};

struct bstn_hwsem *bstn_hwsem_init(void);
int bstn_hwsem_uninit(struct bstn_hwsem *hwsem);

int bstn_hwsem_get(struct bstn_hwsem *hwsem, uint32_t *val);
int bstn_hwsem_release(struct bstn_hwsem *hwsem);

#endif
