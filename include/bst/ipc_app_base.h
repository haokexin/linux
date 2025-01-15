/* SPDX-License-Identifier: GPL-2.0 */

#ifdef CONFIG_MSGBOX_BAREMETAL
struct task_struct *start_trans_server_test(void);
struct task_struct *start_trans_client_test(void);
#else
static inline struct task_struct *start_trans_server_test(void)
{
	return NULL;
}

static inline struct task_struct *start_trans_client_test(void)
{
	return NULL;
}
#endif

#ifdef CONFIG_MSGBOX_DEBUG_TEST
struct task_struct *start_client_test(void *data);
struct task_struct *start_server_test(void *data);
#else
static inline struct task_struct *start_client_test(void)
{
	return NULL;
}

static inline struct task_struct *start_server_test(void)
{
	return NULL;
}
#endif

