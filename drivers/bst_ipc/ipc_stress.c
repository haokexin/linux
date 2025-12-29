#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <bst/ipc_interface.h>
#include <linux/kfifo.h>



static DEFINE_RAW_SPINLOCK(stress_lock);
#define STRESS_TEST_CMD 2
uint32_t is_start = 0;
uint32_t cycle_sync = 0;
#define SEND_MAX_TIMEOUT 2000

#define MAX_MSG_SIZE 64 
typedef struct ipc_stress_ {
    int32_t ipc_session;
    struct task_struct *task_sigal_sync;
    struct task_struct *task_sigal_async;
    struct task_struct *task_method_sync;
    struct task_struct *task_method_async;
    struct task_struct *task_recv;
    struct task_struct *task_reply;
    uint32_t signal_sync_token;
    uint32_t signal_async_token;
    uint32_t method_sync_token;
     uint32_t method_async_token;
    uint32_t recv_method_token;
    uint32_t recv_signal_token;
    uint32_t recv_sync_reply_token;
    uint32_t recv_async_reply_token;
    struct completion reply_complete;
   	DECLARE_KFIFO(kfifo, ipc_msg, MAX_MSG_SIZE);

} ipc_stress_s;




ipc_stress_s * ipc_stress = NULL;
int is_failed = 0;
static unsigned int soc_time = 1000;
module_param(soc_time, uint, 0644);
MODULE_PARM_DESC(soc_time, "delay time");


static int ipc_signal_sync_task(void *data){
    ipc_msg msg;
    int ret = 0;
    ipc_stress_s * ipc_stress = (ipc_stress_s *)data;
    if(!ipc_stress){
        pr_err("ipc_signal_sync_task ipc_stress null\n");
        return -1;
    }


    while(true) {

        if (kthread_should_stop()){
			break;
        }

        if(is_failed == 1){
            usleep_range(soc_time*1000,soc_time*1000);
            continue;
        }

        raw_spin_lock(&stress_lock);
        msg.data = (ipc_stress->signal_sync_token++) & 0x7FFFFFFF;

        raw_spin_unlock(&stress_lock);

        msg.cmd = STRESS_TEST_CMD<<4;
        msg.type = IPC_MSG_TYPE_SIGNAL;

        ret =  ipc_send_sync(ipc_stress->ipc_session, &msg);
        if(ret ){
            is_failed = 1;
            ipc_stress->signal_sync_token--;
        }

       usleep_range(soc_time,soc_time);
    }

    return 0;
}


static int ipc_signal_async_task(void *data){

    ipc_msg msg;
    ipc_stress_s * ipc_stress = (ipc_stress_s *)data;
    int ret = 0;
    if(!ipc_stress){
        pr_err("ipc_signal_sync_task ipc_stress null\n");
        return -1;
    }

    while(true) {

        if (kthread_should_stop()){
			break;
        }

         if(is_failed == 1){
            usleep_range(soc_time*1000,soc_time*1000);
            continue;
        }

        raw_spin_lock(&stress_lock);
        msg.data = (ipc_stress->signal_async_token++) | (0x1 << 31);
        raw_spin_unlock(&stress_lock);

        msg.cmd = STRESS_TEST_CMD<<4;
        msg.type = IPC_MSG_TYPE_SIGNAL;

        ret =  ipc_send(ipc_stress->ipc_session, &msg,SEND_MAX_TIMEOUT,0);
        if(ret){

              pr_err("async signal last token:%u\n",ipc_stress->signal_async_token);

            ipc_stress->signal_async_token--;
            is_failed = 1;
          
        }

        usleep_range(soc_time,soc_time);
    }

    return 0;
}



extern unsigned int get_last_token(void);

static int ipc_method_sync_task(void *data){

    ipc_msg msg;
    int ret;
    ipc_stress_s * ipc_stress = (ipc_stress_s *)data;
    if(!ipc_stress){
        pr_err("ipc_method_sync_task ipc_stress null\n");
        return -1;
    }

    while(true) {

        if (kthread_should_stop()){
			break;
        }

         if(is_failed == 1){
            usleep_range(soc_time*1000,soc_time*1000);
            continue;
        }


        msg.cmd =  STRESS_TEST_CMD<<4;
        msg.type = IPC_MSG_TYPE_METHOD;

        raw_spin_lock(&stress_lock);
        msg.data = (ipc_stress->method_sync_token++)&0x7FFFFFFF;
        raw_spin_unlock(&stress_lock);


        ret = ipc_send_sync(ipc_stress->ipc_session, &msg);
        if(ret){
            ipc_stress->method_sync_token--;
            is_failed = 1;
            pr_err("sync last token\n");
        }
        
        usleep_range(soc_time,soc_time);
    }

    return 0;
}




static int ipc_method_async_task(void *data){

    ipc_msg msg;
    int ret;

    ipc_stress_s * ipc_stress = (ipc_stress_s *)data;
    if(!ipc_stress){
        pr_err("ipc_method_sync_task ipc_stress null\n");
        return -1;
    }

    while(true) {

        if (kthread_should_stop()){
			break;
        }

         if(is_failed == 1){
            usleep_range(soc_time*1000,soc_time*1000);
            continue;
        }


        raw_spin_lock(&stress_lock);
        msg.data = (ipc_stress->method_async_token++) | (0x1 << 31);
        raw_spin_unlock(&stress_lock);

        msg.cmd = STRESS_TEST_CMD<<4;
        msg.type = IPC_MSG_TYPE_METHOD;

        ret =ipc_send(ipc_stress->ipc_session, &msg,SEND_MAX_TIMEOUT,1);
        if(ret) {
            pr_err("async signal last token:%u\n",ipc_stress->signal_async_token);
           ipc_stress->method_async_token--;
           is_failed = 1;
        }

        usleep_range(soc_time,soc_time);
    }

    return 0;
}




static int ipc_reply_task(void *data){

    ipc_msg msg;
    ipc_stress_s * ipc_stress = (ipc_stress_s *)data;
    if(!ipc_stress){
        pr_err("ipc_method_sync_task ipc_stress null\n");
        return -1;
    }

    while(true) {

        if (kthread_should_stop()){
			break;
        }


        wait_for_completion_interruptible(&ipc_stress->reply_complete);
        
        raw_spin_lock(&stress_lock);
        cycle_sync = cycle_sync==1 ? 0: 1;
        raw_spin_unlock(&stress_lock);

    	while (kfifo_out(&ipc_stress->kfifo, &msg, 1)) {

            msg.type = IPC_MSG_TYPE_REPLY;
            msg.cmd = STRESS_TEST_CMD<<4;
            if(cycle_sync == 1){
                ipc_send_sync(ipc_stress->ipc_session, &msg);
            }else{
                ipc_send(ipc_stress->ipc_session, &msg,SEND_MAX_TIMEOUT,1);
            }
		}
       

        usleep_range(soc_time,soc_time);
     }

    return 0;
}



static int ipc_recv_reply_task(void *data){

    ipc_msg msg;
    int ret;
    ipc_stress_s * ipc_stress = (ipc_stress_s *)data;
    if(!ipc_stress){
        pr_err("ipc_method_sync_task ipc_stress null\n");
        return -1;
    }

   

    while(true) {

        if (kthread_should_stop()){
			break;
        }

       ret= ipc_recv(ipc_stress->ipc_session, &msg, 5000); //1000ms
       switch(ret){
            case IPC_RECV_ERR_TIMEOUT:{
                continue;
            }
            case IPC_RECV_ERR:
            case IPC_RECV_ERR_INVALID_PARAM:
            case IPC_RECV_ERR_GET_MSG_FAIL:{
                printk("ipc recieve failed\n");
                break;
            }
            default:{
                if(msg.type == IPC_MSG_TYPE_SIGNAL){

                    if(ipc_stress->recv_signal_token !=  msg.data){
                        pr_err("ipc recieve signal safety token %d  linux token:%d failed\n",msg.data,ipc_stress->recv_signal_token);
                    }

                    ipc_stress->recv_signal_token++;
                    if(ipc_stress->recv_signal_token%300000 == 0){
                        pr_err("ipc recieve signal from safety %d\n",ipc_stress->recv_signal_token);
                    }
                }

                if(msg.type == IPC_MSG_TYPE_REPLY){

                    if((msg.data>>31) == 0x1){

                        if(ipc_stress->recv_async_reply_token !=  (msg.data & 0x7FFFFFFF)){
                            pr_err("ipc recieve reply safety token %d  linux token:%d failed\n",msg.data,ipc_stress->recv_async_reply_token);
                        }

                        ipc_stress->recv_async_reply_token++;

                        //printk("ipc recieve reply from safety %d\n",ipc_stress->recv_reply_token);

                        if(ipc_stress->recv_async_reply_token%300000 == 0){
                            pr_err("ipc recieve async method reply from safety %d\n",ipc_stress->recv_async_reply_token);
                        }

                    }
                    else{
                        
                        if(ipc_stress->recv_sync_reply_token !=  (msg.data & 0x7FFFFFFF)){
                            pr_err("ipc recieve reply safety token %d  linux token:%d failed\n",msg.data,ipc_stress->recv_sync_reply_token);
                        }

                        ipc_stress->recv_sync_reply_token++;

                        //printk("ipc recieve reply from safety %d\n",ipc_stress->recv_reply_token);

                        if(ipc_stress->recv_sync_reply_token%300000 == 0){
                            pr_err("ipc recieve sync method reply from safety %d\n",ipc_stress->recv_sync_reply_token);
                        }

                    }
                   
                }

                if(msg.type == IPC_MSG_TYPE_METHOD){

                     if(ipc_stress->recv_method_token !=  msg.data){
                        pr_err("ipc recieve method safety token %d  linux token:%d failed\n",msg.data,ipc_stress->recv_method_token);
                    }

                    ipc_stress->recv_method_token++;

                    //printk("ipc recieve method from safety %d\n",ipc_stress->recv_method_token);


                    if (!kfifo_put(&ipc_stress->kfifo, msg)) {

                         //printk("ipc recieve method from safety %d\n",ipc_stress->recv_method_token);

                        return -ENOSPC;
                    }


                    if(ipc_stress->recv_method_token%300000 == 0){
                        pr_err("ipc recieve method from safety %d\n",ipc_stress->recv_method_token);
                    }

                    complete(&ipc_stress->reply_complete);
                }

                continue;
            }
       }

    }

    return 0;
}




int32_t ipc_stress_start(void){

    int ret = 0;
    ipc_msg msg;


    #ifdef CONFIG_BST_C1200_IVI
	uint32_t cpu_id = IPC_CORE_ARM0;
	#endif

	#ifdef CONFIG_BST_C1200_ADAS
	uint32_t cpu_id = IPC_CORE_ARM2;
	#endif

	#ifdef CONFIG_BST_C1200_DB
	uint32_t cpu_id = IPC_CORE_DB0;
	#endif


    if(is_start == 1){
        pr_err("stress started\n");
        return 0;
    }



    pr_err("delay time :%dus\n",soc_time);
    
    ipc_stress = kmalloc(sizeof(struct ipc_stress_), GFP_KERNEL);
    if(!ipc_stress){
        pr_err("kmalloc failed\n");
        return -ENOMEM;
    }

    ipc_stress->signal_sync_token = 0;
    ipc_stress->signal_async_token = 0;
    ipc_stress->method_sync_token = 0;
    ipc_stress->method_async_token = 0;
    ipc_stress->recv_method_token = 0;
    ipc_stress->recv_signal_token = 0;
    ipc_stress->recv_sync_reply_token = 0;
    ipc_stress->recv_async_reply_token = 0;

    INIT_KFIFO(ipc_stress->kfifo);
	
    ipc_stress->ipc_session =  ipc_init(IPC_CORE_SAFE,cpu_id,NULL);
    if(ipc_stress->ipc_session < 0){
        pr_err("ipc_init failed: %d\n",ipc_stress->ipc_session);
        ret = IPC_INIT_ERR_SESSION;
        goto err_free;
    }


    ipc_signal_subscribe(ipc_stress->ipc_session, 32);
    ipc_method_register(ipc_stress->ipc_session, 31);
    init_completion(&ipc_stress->reply_complete);

    ipc_stress->task_recv = kthread_run(ipc_recv_reply_task, ipc_stress, "task_reply");
	if (IS_ERR(ipc_stress->task_reply)) {
		pr_err("task_reply: unable to create kernel thread: %ld\n",PTR_ERR(ipc_stress->task_reply));
		goto err_close;
	}



    raw_spin_lock(&stress_lock);
    msg.data = (ipc_stress->signal_sync_token++) & 0x7FFFFFFF;
    raw_spin_unlock(&stress_lock);

    msg.cmd = (STRESS_TEST_CMD<<4) | 0x2;
    msg.type = IPC_MSG_TYPE_SIGNAL;

    ipc_send_sync(ipc_stress->ipc_session, &msg);



    mdelay(5);

    ipc_stress->task_sigal_sync = kthread_run(ipc_signal_sync_task, ipc_stress, "signal_sync");
	if (IS_ERR(ipc_stress->task_sigal_sync)) {
		pr_err("task_sigal_sync: unable to create kernel thread: %ld\n",PTR_ERR(ipc_stress->task_sigal_sync));
		goto err_stop_reply_recv;
	}


    ipc_stress->task_sigal_async = kthread_run(ipc_signal_async_task, ipc_stress, "signal_async");
	if (IS_ERR(ipc_stress->task_sigal_async)) {
		pr_err("task_sigal_async: unable to create kernel thread: %ld\n",PTR_ERR(ipc_stress->task_sigal_async));
		goto err_stop_signal_sync;
	}


    ipc_stress->task_method_sync = kthread_run(ipc_method_sync_task, ipc_stress, "method_sync");
	if (IS_ERR(ipc_stress->task_method_sync)) {
		pr_err("task_sigal_async: unable to create kernel thread: %ld\n",PTR_ERR(ipc_stress->task_method_sync));
		goto err_stop_signal_async;
	}


    ipc_stress->task_method_async = kthread_run(ipc_method_async_task, ipc_stress, "method_async");
	if (IS_ERR(ipc_stress->task_method_async)) {
		pr_err("task_method_async: unable to create kernel thread: %ld\n",PTR_ERR(ipc_stress->task_method_async));
		goto err_stop_method_sync;
	}



    ipc_stress->task_reply = kthread_run(ipc_reply_task, ipc_stress, "method_async");
	if (IS_ERR(ipc_stress->task_reply)) {
		pr_err("task_reply: unable to create kernel thread: %ld\n",PTR_ERR(ipc_stress->task_reply));
		goto err_stop_method_async;
	}

    is_start = 1;


    pr_err("ipc stress insmod success\n");

    return 0;


err_stop_method_async:
    kthread_stop( ipc_stress->task_method_async);

err_stop_method_sync:
    kthread_stop( ipc_stress->task_method_sync);


err_stop_signal_async:
    kthread_stop( ipc_stress->task_sigal_async);


err_stop_signal_sync:
    kthread_stop( ipc_stress->task_sigal_sync);

err_stop_reply_recv:
    kthread_stop( ipc_stress->task_recv);

err_close:
    ipc_close(ipc_stress->ipc_session);

err_free:
    kfree(ipc_stress);
    return ret;
}

EXPORT_SYMBOL(ipc_stress_start);



int32_t ipc_stress_stop(void){

    ipc_msg msg;
    int count = 5;
    if(is_start == 0 || !ipc_stress){
        pr_err("ipc stress not start\n");
        return 0;
    }

    is_start = 0;


    kthread_stop( ipc_stress->task_method_async);
    kthread_stop( ipc_stress->task_method_sync);
    kthread_stop( ipc_stress->task_sigal_async);
    kthread_stop( ipc_stress->task_sigal_sync);

    mdelay(100);

    while(count--){
        raw_spin_lock(&stress_lock);
        msg.data = (ipc_stress->signal_sync_token++) & 0x7FFFFFFF;
        raw_spin_unlock(&stress_lock);

        msg.cmd = (STRESS_TEST_CMD<<4) | 0x1;
        msg.type = IPC_MSG_TYPE_SIGNAL;

        ipc_send_sync(ipc_stress->ipc_session, &msg);
    }

    mdelay(50);

    kthread_stop( ipc_stress->task_reply);
    kthread_stop( ipc_stress->task_recv);

    mdelay(50);

    ipc_close(ipc_stress->ipc_session);
    kfree(ipc_stress);
    ipc_stress = NULL;

    pr_err("ipc stress stop success\n");
    return 0;
}


EXPORT_SYMBOL(ipc_stress_stop);


static int ipc_ops_set(void *data, u64 val){

    if(val == 0){
        ipc_stress_stop();
    }else{
        ipc_stress_start();
    }

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(debugfs_stress_fops, NULL, ipc_ops_set, "0x%016llx\n");

void ipc_stress_init(void){

    struct dentry		*debugfs_dir;

    debugfs_dir = debugfs_create_dir("ipc_stress",NULL);
    if(debugfs_dir){
        debugfs_create_file("stress", 0444, debugfs_dir,NULL,&debugfs_stress_fops);
    }

}

EXPORT_SYMBOL_GPL(ipc_stress_init);








MODULE_AUTHOR("bst.ai");
MODULE_DESCRIPTION("ipc stress test");
MODULE_LICENSE("GPL v2"); 