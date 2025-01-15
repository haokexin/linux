#ifndef IPC_APP_LAYER_CLIENT_UTILITIES_H
#define IPC_APP_LAYER_CLIENT_UTILITIES_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <bst/ipc_app_common.h>

typedef struct
{
    volatile bool busy;
    void *cb;
    void *ext;
    struct semaphore sem;
} callback_registration_t;

static inline int32_t init_registry(callback_registration_t *reg)
{
    if (!reg)
        return -1;
    reg->busy = false;
    reg->cb = NULL;
    reg->ext = NULL;
    sema_init(&reg->sem, 0);
    return 0;
}

static inline int32_t init_registry_list(callback_registration_t* list, uint32_t size)
{
    int32_t ret = 0;
    int32_t i;

    for(i = 0; i < size; ++i)
    {
        ret += init_registry(&list[i]);
    }
    return ret;
}

static inline callback_registration_t* take_registry(callback_registration_t* list, volatile _Atomic uint8_t *index, uint8_t *out) {
    uint8_t curr_idx = *index;
    int32_t ret = 0;
    do
    {
        if (list[curr_idx].busy)
            return NULL;
    }while(!__atomic_compare_exchange_n(index, &curr_idx, ((curr_idx + 1)%IPC_TOKEN_NUM), false, __ATOMIC_RELEASE, __ATOMIC_RELAXED));
    list[curr_idx].busy = true;
    if (out)
        *out = curr_idx;
    return &list[curr_idx];
}

static inline int32_t add_registry(callback_registration_t *reg, void *cb, void *ext)
{
    if (!reg)
        return -1;

    reg->cb = cb;
    reg->ext = ext;
    reg->sem.count = 0;
    // atomic_set((atomic_t *)&reg->sem.count, 0);
    return 0;
}

static inline void clear_registry(callback_registration_t *reg)
{
    if (!reg)
        return;
    reg->busy = false;
    // reg->sem.count = 0;
}

static inline int32_t wait_on_registry(callback_registration_t *reg)
{
    int32_t ret = 0;
    if (!reg)
        return -1;

    ret = down_interruptible(&reg->sem);
    return ret;
}

static inline int32_t timedwait_on_registry(callback_registration_t *reg, int64_t timeout_ms)
{
    int32_t ret = 0;
    if (!reg || timeout_ms <= 0)
        return -1;

    ret = down_timeout(&reg->sem, msecs_to_jiffies(timeout_ms));
    return ret;
}

static inline int32_t notify_callback_registry(callback_registration_t *reg)
{
    if (!reg)
        return -1;
    if (reg->busy)
        up(&reg->sem);
    return 0;
}

static inline void destroy_registry(callback_registration_t *reg)
{
    if (!reg)
        return;
    reg->cb = NULL;
    reg->ext = NULL;
}

static inline void destroy_registry_list(callback_registration_t* list, uint32_t size)
{
	int32_t i;
    for(i = 0; i < size; ++i)
    {
        destroy_registry(&list[i]);
    }
}

#endif
