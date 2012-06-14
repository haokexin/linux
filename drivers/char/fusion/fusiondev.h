/*
 *	Fusion Kernel Module
 *
 *	(c) Copyright 2002-2003  Convergence GmbH
 *
 *      Written by Denis Oliver Kropp <dok@directfb.org>
 *
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#ifndef __FUSIONDEV_H__
#define __FUSIONDEV_H__

#include <linux/proc_fs.h>

#include "entries.h"
#include "list.h"

#define FUSION_ASSERT(exp)    if (!(exp)) BUG()
#define FUSION_ASSUME(exp)    if (!(exp)) printk( KERN_ERR "fusiondev: assumption '" #exp "' failed!\n" )

struct __Fusion_FusionDev {
     int                    refs;
     int                    index;

     struct semaphore       enter_lock;
     int                    enter_ok;
     wait_queue_head_t      enter_wait;

     unsigned long          shared_area;

     struct proc_dir_entry *proc_dir;

     struct {
          int property_lease_purchase;
          int property_cede;

          int reactor_attach;
          int reactor_detach;
          int reactor_dispatch;

          int ref_up;
          int ref_down;

          int skirmish_prevail_swoop;
          int skirmish_dismiss;
          int skirmish_wait;
          int skirmish_notify;

          int shmpool_attach;
          int shmpool_detach;
     } stat;

     struct {
          int                ids;
          FusionLink        *list;
          struct semaphore   lock;
     } call;

     struct {
          int                last_id;
          FusionLink        *list;
          struct semaphore   lock;
          wait_queue_head_t  wait;
     } fusionee;

     FusionEntries  properties;
     FusionEntries  reactor;
     FusionEntries  ref;
     FusionEntries  shmpool;
     FusionEntries  skirmish;
};

/*
 * Special version of interruptible_sleep_on() that unlocks the mutex
 * after adding the entry to the queue (just before schedule).
 */
void fusion_sleep_on (wait_queue_head_t *q,
                      struct semaphore  *lock,
                      signed long       *timeout_ms);

#endif
