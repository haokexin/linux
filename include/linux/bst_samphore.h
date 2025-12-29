#ifndef __BST_SAMPHORE_H__
#define __BST_SAMPHORE_H__


enum MSG_ID {
    MSG_ID_0,
    MSG_ID_1,
    MSG_ID_2,
    MSG_ID_3,
    MSG_ID_4,
    MSG_ID_5,
    MSG_ID_6,
    MSG_ID_7,
    MSG_ID_8,
    MSG_ID_9,
    MSG_ID_10,
    MSG_ID_11,
    MSG_ID_12,
    MSG_ID_13,
    MSG_ID_14,
    MSG_ID_15,
};

enum MST_ID {
    MST_MP4 = 1,
    MST_MP2,
    MST_SEC,
    MST_SAFETY,
    MST_REALTIME,
    MST_SWITCH,
    MST_SOC_DMA,
    MST_ISP_TV,
    MST_CV_DSP,
    MST_NET_DSP,
    MSG_HIFI_DSP,
};

enum BANK_ID {
    BANK_ID0,
    BANK_ID1,
};




struct bst_samphore{
    enum MST_ID mst_id;
    enum MSG_ID msg_id;
    enum BANK_ID bank_id;
    void __iomem *ipc_sem_base;
};


struct bst_samphore *  samphore_lock_init(enum MST_ID mst_id,enum MSG_ID msg_id);
int samphore_lock_remove(struct bst_samphore * samphore);
int release_sem_lock(struct bst_samphore * samphore);
int get_sem_lock(struct bst_samphore * samphore);
int get_sem_lock_with_timeout(struct bst_samphore * samphore,unsigned int timeout_ms);
int release_bst_sem_lock(struct bst_samphore * samphore);
struct bst_samphore * bst_semaphore_init(enum MST_ID mst_id,enum BANK_ID bank_id,enum MSG_ID msg_id);
#endif