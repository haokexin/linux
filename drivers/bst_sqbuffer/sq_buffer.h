// SPDX-License-Identifier: (GPL-2.0 OR MIT)

/*
 *  Copyright (C) 2024 Black Sesame Technologies. All Rights Reserved.
 */

#ifndef SQ_BUFFER_H
#define SQ_BUFFER_H

#include <linux/types.h>

#define SQ_BUFFER_NUM_MAX 64
#define SQ_BUFFER_CONSUMER_MAX 16

typedef struct SQE {
    int32_t ref_cnt;
    int32_t index;
    uint32_t buffer;  // 32-bit physical address of the buffer
} SQE;

typedef struct free_SQ {
    SQE sqe[SQ_BUFFER_NUM_MAX];
} free_SQ;

typedef struct data_SQ {
    int32_t front;
    int32_t rear;
    int32_t sqe_ptr[SQ_BUFFER_NUM_MAX];  // Save the index of SQE in free_SQ
} data_SQ;

typedef struct SQ_Buffer {
    int32_t buffer_size;
    int32_t buffer_num;
    free_SQ free_sq;
    data_SQ data_sq[SQ_BUFFER_CONSUMER_MAX];
    uint64_t buffers_phy_addr64[SQ_BUFFER_NUM_MAX];
    uint32_t magic;
} __attribute__((aligned(4))) SQ_Buffer;

typedef struct sqe_t {
    int32_t *index;
    int32_t *ref_cnt;
    void *buffer;  // Virtual address of the buffer
} sqe_proxy_t;

typedef struct sq_buffer_proxy_t {
    sqe_proxy_t sqe[SQ_BUFFER_NUM_MAX];
} sq_buffer_proxy_t;


/**
 * @brief Init sq buffer
 * @param sq Pointer to the SQ_Buffer
 * @param buffer buffer array in smmu address
 * @param buffer_size Buffer's size
 * @param buffer_num Buffer's number
 * @return 0 if successful, -1 if failed
 */
int SQ_Buffer_Init(SQ_Buffer *sq, uint32_t *buffer,
        uint32_t buffer_size, uint32_t buffer_num);

/**
 * @brief Get a SQE from the consumer proxy structure
 * @param proxy Pointer to the consumer proxy structure
 * @param consumer_id Consumer ID
 * @return Return the pointer to the SQE proxy structure, if there is no
 * available SQE, return NULL
 */
sqe_proxy_t *sq_buffer_consume_get(uint8_t consumer_id);

/**
 * @brief Return the SQE to the consumer proxy structure
 * @param proxy Pointer to the consumer proxy structure
 * @param sqe Pointer to the SQE proxy structure
 * @return 0 if successful, -1 if failed
 */
int sq_buffer_consume_put(sqe_proxy_t *sqe);

/**
 * @brief Get a free SQE from the producer proxy structure
 * @return Return the pointer to a free SQE proxy structure, if there is no
 * free SQE, return NULL
 */
sqe_proxy_t *sq_buffer_produce_get(void);

/**
 * @brief Put SQE back into the producer proxy structure and set the consumer mask
 * @param sqe Pointer to the SQE proxy structure
 * @param consumer_mask consumer mask
 * @return 0 if successful, -1 if failed
 */
int sq_buffer_produce_put(sqe_proxy_t *sqe,
                          uint16_t consumer_mask);


/**
 * @brief Get the base physical address of the SQ_Buffer
 * @return The base physical address of the SQ_Buffer
 */
uint64_t sq_buffer_get_base_paddr(void);

/**
 * @brief Get the base virtual address of the SQ_Buffer
 * @return The base virtual address of the SQ_Buffer
 */
uint64_t sq_buffer_get_base_vaddr(void);

int sq_buffer_consume_datanum(uint8_t consumer_id);

#endif  // SQ_BUFFER_H
