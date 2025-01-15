#ifndef IPC_APP_LAYER_SERVER_UTILITIES_H
#define IPC_APP_LAYER_SERVER_UTILITIES_H

#include <bst/bstipc_cfg.h>

#define MAX_SUBSCRIPTION 34U

typedef struct
{
    uint8_t pid;
    uint8_t fid;
    uint8_t sid;
} broadcast_reg_entry;

typedef struct
{
    broadcast_reg_entry entries[MAX_SUBSCRIPTION];
    int32_t start;
    int32_t end;
} broadcast_registry;

// inline functions for registration
static inline int32_t add_registration(broadcast_registry *regs, uint8_t pid, uint8_t fid, uint8_t sid)
{
    int32_t index = 0, insert_index = -1;
    if (!regs || pid == 0)
        return -1;
    // find if already exists.
    broadcast_reg_entry *entry = regs->entries;
    for (index = 0; index < MAX_SUBSCRIPTION; ++index, ++entry)
    {
        if (entry->pid == pid && entry->fid == fid && entry->sid == sid)
            return 0;
        if (insert_index < 0 && entry->pid == 0)
            insert_index = index;
    }
    if (insert_index < 0)
        return -1;
    // insert the entry.
    entry = &regs->entries[insert_index];
    entry->pid = pid;
    entry->fid = fid;
    entry->sid = sid;
    if (index < regs->start)
        regs->start = index;
    if (index >= regs->end)
        regs->end = index + 1;
    return 0;
}

static inline int32_t remove_registration(broadcast_registry *regs, uint8_t pid, uint8_t fid, uint8_t sid)
{
    int32_t index = 0;
    if (!regs || pid == 0)
        return -1;
    broadcast_reg_entry *entry = regs->entries;
    for (index = regs->start, entry += regs->start; index < regs->end; ++index, ++entry)
    {
        if (entry->pid == pid && entry->fid == fid && entry->sid == sid)
        {
            entry->pid = 0;
            entry->fid = 0;
            entry->sid = 0;
            return 0;
        }
    }
    return -1;
}

#endif
