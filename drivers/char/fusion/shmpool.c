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

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/sched.h>

#include <linux/fusion.h>

#include "fusiondev.h"
#include "fusionee.h"
#include "list.h"
#include "shmpool.h"


#define SHM_BASE    0x20010000     /* virtual base address */
#define SHM_SIZE    0x1FFEF000     /* size of virtual address space */


typedef struct {
     FusionLink         link;
     unsigned long      next_base;
} AddrEntry;


typedef struct {
     FusionLink         link;

     FusionID           fusion_id;

     int                count;     /* number of attach calls */
} SHMPoolNode;

typedef struct {
     FusionEntry        entry;

     int                max_size;

     void              *addr_base;
     int                size;

     AddrEntry         *addr_entry;

     FusionLink        *nodes;

     int                dispatch_count;
} FusionSHMPool;

/******************************************************************************/

static SHMPoolNode *get_node      ( FusionSHMPool *shmpool,
                                    FusionID       fusion_id );

static void         remove_node   ( FusionSHMPool *shmpool,
                                    FusionID       fusion_id );

static int          fork_node     ( FusionSHMPool *shmpool,
                                    FusionID       fusion_id,
                                    FusionID       from_id );

static void         free_all_nodes( FusionSHMPool *shmpool );

/******************************************************************************/


static DECLARE_MUTEX (addr_lock);
static FusionLink    *addr_entries;
static unsigned long  addr_base = SHM_BASE;

/******************************************************************************/

static AddrEntry *
add_addr_entry( unsigned long next_base )
{
     AddrEntry *entry = kmalloc( sizeof(AddrEntry), GFP_KERNEL );

     entry->next_base = next_base;

     fusion_list_prepend( &addr_entries, &entry->link );

     return entry;
}

/******************************************************************************/

static int
fusion_shmpool_construct( FusionEntry *entry,
                          void        *ctx,
                          void        *create_ctx )
{
     FusionSHMPool    *shmpool = (FusionSHMPool*) entry;
     FusionSHMPoolNew *poolnew = create_ctx;

     down( &addr_lock );

     if (addr_base + poolnew->max_size >= SHM_BASE + SHM_SIZE) {
          up( &addr_lock );
          printk( KERN_WARNING "%s: virtual address space exhausted! (FIXME)\n", __FUNCTION__ );
          return -ENOSPC;
     }

     shmpool->max_size  = poolnew->max_size;
     shmpool->addr_base = poolnew->addr_base = (void*) addr_base;

     addr_base += PAGE_ALIGN(poolnew->max_size) + PAGE_SIZE; /* fence page */

     shmpool->addr_entry = add_addr_entry( addr_base );

     up( &addr_lock );

     return 0;
}

static void
fusion_shmpool_destruct( FusionEntry *entry,
                         void        *ctx )
{
     AddrEntry     *addr_entry;
     FusionSHMPool *shmpool = (FusionSHMPool*) entry;

     free_all_nodes( shmpool );


     down( &addr_lock );

     fusion_list_remove( &addr_entries, &shmpool->addr_entry->link );


     /*
      * free trailing address space
      */

     addr_base = SHM_BASE;

     fusion_list_foreach (addr_entry, addr_entries) {
          if (addr_entry->next_base > addr_base)
               addr_base = addr_entry->next_base;
     }

     up( &addr_lock );
}

static int
fusion_shmpool_print( FusionEntry *entry,
                      void        *ctx,
                      char        *buf )
{
     int            num     = 0;
     FusionSHMPool *shmpool = (FusionSHMPool*) entry;
     FusionLink    *node    = shmpool->nodes;

     fusion_list_foreach (node, shmpool->nodes) {
          num++;
     }

     return sprintf( buf, "0x%p [0x%x] - 0x%x, %dx dispatch, %d nodes\n",
                     shmpool->addr_base, shmpool->max_size, shmpool->size,
                     shmpool->dispatch_count, num );
}


FUSION_ENTRY_CLASS( FusionSHMPool, shmpool, fusion_shmpool_construct,
                    fusion_shmpool_destruct, fusion_shmpool_print )

/******************************************************************************/

int
fusion_shmpool_init (FusionDev *dev)
{
     fusion_entries_init( &dev->shmpool, &shmpool_class, dev );

     create_proc_read_entry( "shmpools", 0, dev->proc_dir,
                             fusion_entries_read_proc, &dev->shmpool );

     return 0;
}

void
fusion_shmpool_deinit (FusionDev *dev)
{
     remove_proc_entry ("shmpools", dev->proc_dir);

     fusion_entries_deinit( &dev->shmpool );
}

/******************************************************************************/

int
fusion_shmpool_new (FusionDev        *dev,
                    FusionSHMPoolNew *pool)
{
     if (pool->max_size <= 0)
          return -EINVAL;

     return fusion_entry_create( &dev->shmpool, &pool->pool_id, pool );
}

int
fusion_shmpool_attach (FusionDev           *dev,
                       FusionSHMPoolAttach *attach,
                       FusionID             fusion_id)
{
     int            ret;
     SHMPoolNode   *node;
     FusionSHMPool *shmpool;

     ret = fusion_shmpool_lock( &dev->shmpool, attach->pool_id, false, &shmpool );
     if (ret)
          return ret;

     dev->stat.shmpool_attach++;

     node = get_node (shmpool, fusion_id);
     if (!node) {
          node = kmalloc (sizeof(SHMPoolNode), GFP_KERNEL);
          if (!node) {
               fusion_shmpool_unlock( shmpool );
               return -ENOMEM;
          }

          node->fusion_id = fusion_id;
          node->count     = 1;

          fusion_list_prepend (&shmpool->nodes, &node->link);
     }
     else
          node->count++;

     attach->addr_base = shmpool->addr_base;
     attach->size      = shmpool->size;

     fusion_shmpool_unlock( shmpool );

     return 0;
}

int
fusion_shmpool_detach (FusionDev *dev, int id, FusionID fusion_id)
{
     int            ret;
     SHMPoolNode   *node;
     FusionSHMPool *shmpool;

     ret = fusion_shmpool_lock( &dev->shmpool, id, false, &shmpool );
     if (ret)
          return ret;

     dev->stat.shmpool_detach++;

     node = get_node (shmpool, fusion_id);
     if (!node) {
          fusion_shmpool_unlock( shmpool );
          return -EIO;
     }

     if (! --node->count) {
          fusion_list_remove (&shmpool->nodes, &node->link);
          kfree (node);
     }

     fusion_shmpool_unlock( shmpool );

     return 0;
}

int
fusion_shmpool_dispatch( FusionDev             *dev,
                         FusionSHMPoolDispatch *dispatch,
                         FusionID               fusion_id )
{
     int                   ret;
     FusionLink           *l;
     FusionSHMPool        *shmpool;
     FusionSHMPoolMessage  message;

     if (dispatch->size <= 0)
          return -EINVAL;

     ret = fusion_shmpool_lock( &dev->shmpool, dispatch->pool_id, false, &shmpool );
     if (ret)
          return ret;

     message.type = FSMT_REMAP;
     message.size = dispatch->size;

     shmpool->dispatch_count++;

     shmpool->size = dispatch->size;

     fusion_list_foreach (l, shmpool->nodes) {
          SHMPoolNode *node = (SHMPoolNode *) l;

          if (node->fusion_id == fusion_id)
               continue;

          fusionee_send_message (dev, fusion_id, node->fusion_id, FMT_SHMPOOL,
                                 shmpool->entry.id, sizeof(message), &message);
     }

     fusion_shmpool_unlock( shmpool );

     return 0;
}

int
fusion_shmpool_destroy (FusionDev *dev, int id)
{
     return fusion_entry_destroy( &dev->shmpool, id );
}

void
fusion_shmpool_detach_all (FusionDev *dev, FusionID fusion_id)
{
     FusionLink *l;

     down (&dev->shmpool.lock);

     fusion_list_foreach (l, dev->shmpool.list) {
          FusionSHMPool *shmpool = (FusionSHMPool *) l;

          remove_node (shmpool, fusion_id);
     }

     up (&dev->shmpool.lock);
}

int
fusion_shmpool_fork_all( FusionDev *dev,
                         FusionID   fusion_id,
                         FusionID   from_id )
{
     FusionLink *l;
     int         ret = 0;

     down (&dev->shmpool.lock);

     fusion_list_foreach (l, dev->shmpool.list) {
          FusionSHMPool *shmpool = (FusionSHMPool *) l;

          ret = fork_node( shmpool, fusion_id, from_id );
          if (ret)
               break;
     }

     up (&dev->shmpool.lock);

     return ret;
}

/******************************************************************************/

static SHMPoolNode *
get_node (FusionSHMPool *shmpool,
          FusionID       fusion_id)
{
     SHMPoolNode *node;

     fusion_list_foreach (node, shmpool->nodes) {
          if (node->fusion_id == fusion_id)
               return node;
     }

     return NULL;
}

static void
remove_node (FusionSHMPool *shmpool, FusionID fusion_id)
{
     SHMPoolNode *node;

     down (&shmpool->entry.lock);

     fusion_list_foreach (node, shmpool->nodes) {
          if (node->fusion_id == fusion_id) {
               fusion_list_remove (&shmpool->nodes, &node->link);
               break;
          }
     }

     up (&shmpool->entry.lock);
}

static int
fork_node (FusionSHMPool *shmpool, FusionID fusion_id, FusionID from_id)
{
     int          ret = 0;
     SHMPoolNode *node;

     down (&shmpool->entry.lock);

     fusion_list_foreach (node, shmpool->nodes) {
          if (node->fusion_id == from_id) {
               SHMPoolNode *new_node;

               new_node = kmalloc (sizeof(SHMPoolNode), GFP_KERNEL);
               if (!new_node) {
                    ret = -ENOMEM;
                    break;
               }

               new_node->fusion_id = fusion_id;
               new_node->count     = node->count;

               fusion_list_prepend (&shmpool->nodes, &new_node->link);

               break;
          }
     }

     up (&shmpool->entry.lock);

     return ret;
}

static void
free_all_nodes (FusionSHMPool *shmpool)

{
     FusionLink  *n;
     SHMPoolNode *node;

     fusion_list_foreach_safe (node, n, shmpool->nodes) {
          kfree (node);
     }

     shmpool->nodes = NULL;
}
