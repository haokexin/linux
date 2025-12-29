#ifndef IPC_LOCKFREE_LIST_MIMO_H
#define IPC_LOCKFREE_LIST_MIMO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
// #include <stdio.h>

#ifdef __LP64__
typedef uintptr_t atomic_stamped_ptr_t;

static atomic_stamped_ptr_t null_atomic_stamped_ptr = 0;
static inline atomic_stamped_ptr_t atomic_stamped_ptr_make(void* ptr, uint16_t stamp)
{
    return ((atomic_stamped_ptr_t)stamp << 48) | (uintptr_t)ptr;
}

static inline void* atomic_stamped_ptr_get_ptr(atomic_stamped_ptr_t stamped_ptr)
{
    return (void*)(stamped_ptr & 0xFFFFFFFFFFFFULL);
}

static inline uint16_t atomic_stamped_ptr_get_stamp(atomic_stamped_ptr_t stamped_ptr)
{
    return (stamped_ptr >> 48) & 0xFF;
}

static inline bool atomic_stamped_ptr_eq(atomic_stamped_ptr_t a, atomic_stamped_ptr_t b) {
    return a == b;
}

static inline atomic_stamped_ptr_t atomic_stamped_ptr_inc(atomic_stamped_ptr_t stamped_ptr)
{
    uint16_t stamp = (stamped_ptr >> 48) & 0xFF;
    return ((atomic_stamped_ptr_t)(++stamp) << 48) | (stamped_ptr & 0xFFFFFFFFFFFFULL);
}

static inline bool atomic_stamped_ptr_compare_exchange(volatile atomic_stamped_ptr_t *dest, atomic_stamped_ptr_t *expected, atomic_stamped_ptr_t desired)
{
    return __atomic_compare_exchange_n(dest, expected, desired, true, __ATOMIC_RELEASE, __ATOMIC_RELAXED);
}

static inline void atomic_stamped_ptr_load(volatile atomic_stamped_ptr_t *dest, atomic_stamped_ptr_t *expected)
{
    *expected = __atomic_load_n(dest, __ATOMIC_ACQUIRE);
}

static inline void atomic_stamped_ptr_store(volatile atomic_stamped_ptr_t *dest, atomic_stamped_ptr_t desired)
{
    __atomic_store_n(dest, desired, __ATOMIC_RELEASE);
}

#else

typedef struct _stamp_ptr
{
    void* ptr;
    uint16_t stamp;
    uint16_t res;
} atomic_stamped_ptr_t;

static atomic_stamped_ptr_t null_atomic_stamped_ptr = {.ptr=((void *)0), .stamp=0};

static inline atomic_stamped_ptr_t atomic_stamped_ptr_make(void* ptr, uint16_t stamp)
{
    atomic_stamped_ptr_t ret = {.ptr=ptr, .stamp=stamp};
    return ret;
}

static inline void* atomic_stamped_ptr_get_ptr(atomic_stamped_ptr_t stamped_ptr)
{
    return stamped_ptr.ptr;
}

static inline uint16_t atomic_stamped_ptr_get_stamp(atomic_stamped_ptr_t stamped_ptr)
{
    return stamped_ptr.stamp;
}

static inline bool atomic_stamped_ptr_eq(atomic_stamped_ptr_t a, atomic_stamped_ptr_t b) {
    return a.ptr == b.ptr && a.stamp == b.stamp;
}

static inline atomic_stamped_ptr_t atomic_stamped_ptr_inc(atomic_stamped_ptr_t stamped_ptr)
{
    ++stamped_ptr.stamp;
    return stamped_ptr;
}

static inline bool atomic_stamped_ptr_compare_exchange(_Atomic volatile atomic_stamped_ptr_t *dest,
    atomic_stamped_ptr_t *expected, atomic_stamped_ptr_t desired)
{
    return __atomic_compare_exchange(dest, expected, &desired, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED);
}

static inline void atomic_stamped_ptr_load(_Atomic volatile atomic_stamped_ptr_t *dest, atomic_stamped_ptr_t *expected)
{
    __atomic_load(dest, expected, __ATOMIC_RELAXED);
}

static inline void atomic_stamped_ptr_store(_Atomic volatile atomic_stamped_ptr_t *dest, atomic_stamped_ptr_t desired)
{
    __atomic_store(dest, &desired, __ATOMIC_RELEASE);
}

#endif

typedef struct _lflist_node
{
    void* data;
    __attribute__ ((aligned(8))) atomic_stamped_ptr_t next;
    __attribute__ ((aligned(8))) atomic_stamped_ptr_t me;
} lflist_node;

typedef struct
{
    __attribute__ ((aligned(64))) atomic_stamped_ptr_t volatile head;
    __attribute__ ((aligned(64))) atomic_stamped_ptr_t volatile tail;
    lflist_node head_node;
} lflist;

static inline void lflist_init_node(lflist_node* node, void* data)
{
    if (!node || !data)
        return;
    node->data = data;
    node->next = null_atomic_stamped_ptr;
    node->me = atomic_stamped_ptr_make(node, 0);
}

static inline void lflist_init(lflist *list) 
{
    if (!list)
        return;

    list->head_node.next = null_atomic_stamped_ptr;
    atomic_stamped_ptr_t head_node = atomic_stamped_ptr_make(&list->head_node, 0);
    list->head = head_node;
    list->tail = head_node;
}

static inline void lflist_enqueue(lflist* list, lflist_node* node) {
    atomic_stamped_ptr_t last = null_atomic_stamped_ptr;
    atomic_stamped_ptr_t next = null_atomic_stamped_ptr;
    if (!list || !node)
        return;

    while (true)
    {
        atomic_stamped_ptr_load(&list->tail, &last);
        atomic_stamped_ptr_load(&((lflist_node*)atomic_stamped_ptr_get_ptr(last))->next, &next);

        if (atomic_stamped_ptr_eq(last, list->tail))
        {
            if (!atomic_stamped_ptr_get_ptr(next))
            {
                if (atomic_stamped_ptr_compare_exchange(&((lflist_node*)atomic_stamped_ptr_get_ptr(last))->next, &next, node->me))
                {
                    atomic_stamped_ptr_compare_exchange(&list->tail, &last, node->me);
                    return;
                }
            }
            else
            {
                atomic_stamped_ptr_compare_exchange(&list->tail, &last, next);
            }
        }
    }
}

static inline lflist_node* lflist_dequeue(lflist* list) {
    void* data = ((void *)0);
    atomic_stamped_ptr_t first = null_atomic_stamped_ptr;
    atomic_stamped_ptr_t last = null_atomic_stamped_ptr;
    atomic_stamped_ptr_t next = null_atomic_stamped_ptr;
    lflist_node* first_node = ((void *)0);
    lflist_node* next_node = ((void *)0);
    if (!list)
        return NULL;
    while (true)
    {
        atomic_stamped_ptr_load(&list->head, &first);
        atomic_stamped_ptr_load(&list->tail, &last);
        first_node = (lflist_node*)atomic_stamped_ptr_get_ptr(first);
        if (!first_node)
            continue;

        atomic_stamped_ptr_load(&((lflist_node*)atomic_stamped_ptr_get_ptr(first))->next, &next);
        if (atomic_stamped_ptr_eq(first, list->head))
        {
            if (atomic_stamped_ptr_eq(first, last))
            {
                if (!atomic_stamped_ptr_get_ptr(next))
                    return NULL;
                atomic_stamped_ptr_compare_exchange(&list->tail, &last, next);
            }
            else
            {
                next_node = (lflist_node*)atomic_stamped_ptr_get_ptr(next);
                if (next_node)
                    data = next_node->data;
                if (atomic_stamped_ptr_compare_exchange(&list->head, &first, next))
                {
                    lflist_node* ret = atomic_stamped_ptr_get_ptr(first);
                    first = atomic_stamped_ptr_inc(first);
                    ret->me = first;
                    ret->data = data;
                    ret->next = atomic_stamped_ptr_make((void *)0, atomic_stamped_ptr_get_stamp(first));
                    return ret;
                }
            }
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif