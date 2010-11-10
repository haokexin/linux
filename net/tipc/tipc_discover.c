/*
 * net/tipc/tipc_discover.c: TIPC neighbor discovery code
 * 
 * Copyright (c) 2003-2006, Ericsson AB
 * Copyright (c) 2005-2008, 2010, Wind River Systems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "tipc_core.h"
#include "tipc_dbg.h"
#include "tipc_link.h"
#include "tipc_net.h"
#include "tipc_discover.h"
#include "tipc_cfgsrv.h"
#include "tipc_port.h"
#include "tipc_name_table.h"

#define TIPC_DISC_INIT 125	/* min delay during bearer start up */
#define TIPC_DISC_FAST 1000	/* normal delay if bearer has no links */
#define TIPC_DISC_SLOW 60000	/* normal delay if bearer has links */
#define TIPC_DISC_INACTIVE 0xffffffff	/* There is no timer */


/** 
 * disc_init_msg - initialize a link setup message
 * @type: message type (request or response)
 * @dest_domain: network domain of node(s) which should respond to message
 * @b_ptr: ptr to bearer issuing message
 */

static struct sk_buff *disc_init_msg(u32 type, u32 dest_domain,
				     struct bearer *b_ptr)
{
	struct sk_buff *buf = buf_acquire(INT_H_SIZE);
	struct tipc_msg *msg;
	u32 sig;

	if (buf) {
		msg = buf_msg(buf);
		tipc_msg_init(msg, LINK_CONFIG, type, INT_H_SIZE, dest_domain);
		msg_set_non_seq(msg, 1);
		sig = tipc_random & 0xffff;
		msg_set_node_sig(msg, (sig ? sig : 1));
		msg_set_node_flags(msg, NF_MULTICLUSTER);
		msg_set_dest_domain(msg, dest_domain);
		msg_set_bc_netid(msg, tipc_net_id);
		if (b_ptr->media->addr2msg(&b_ptr->publ.addr, &msg->hdr[5])) {
			buf_discard(buf);
			buf = NULL;
		}
	}
	return buf;
}

/**
 * disc_dupl_alert - issue node address duplication alert
 * @b_ptr: pointer to bearer detecting duplication
 * @node_addr: duplicated node address
 * @media_addr: media address advertised by duplicated node
 */

static void disc_dupl_alert(struct bearer *b_ptr, u32 node_addr, 
			    struct tipc_media_addr *media_addr)
{
#ifdef CONFIG_TIPC_SYSTEM_MSGS
	char node_addr_str[16];
	char media_addr_str[64];
	struct print_buf pb;

	tipc_addr_string_fill(node_addr_str, node_addr);
	tipc_printbuf_init(&pb, media_addr_str, sizeof(media_addr_str));
	tipc_media_addr_printf(&pb, media_addr);
	tipc_printbuf_validate(&pb);
	warn("Duplicate %s using %s seen on <%s>\n",
	     node_addr_str, media_addr_str, b_ptr->publ.name);
#endif
}

/**
 * tipc_disc_recv_msg - handle incoming link setup message (request or response)
 * @buf: buffer containing message
 * @b_ptr: bearer that message arrived on
 */

void tipc_disc_recv_msg(struct sk_buff *buf, struct bearer *b_ptr)
{
	struct link *link;
	struct tipc_media_addr media_addr;
        struct sk_buff *rbuf;
	struct tipc_msg *msg = buf_msg(buf);
	u32 dest = msg_dest_domain(msg);
	u32 orig = msg_prevnode(msg);
	u32 net_id = msg_bc_netid(msg);
	u32 type = msg_type(msg);
	u32 signature = msg_node_sig(msg);
	u32 node_flags = msg_node_flags(msg);
	struct tipc_node *n_ptr;
	int addr_mismatch;
        int link_fully_up;

	b_ptr->media->msg2addr(&media_addr, &msg->hdr[5]);
	msg_dbg(msg, "RECV:");
	buf_discard(buf);

	/* Validate network address of requesting node */

	if (net_id != tipc_net_id)
		return;

#ifdef CONFIG_TIPC_UNICLUSTER_FRIENDLY
	if ((node_flags & NF_MULTICLUSTER) == 0 && !in_own_cluster(orig))
		return;
#else
	if ((node_flags & NF_MULTICLUSTER) == 0)
		return;
#endif

	if (!tipc_addr_domain_valid(dest))
		return;
	if (!tipc_addr_node_valid(orig))
		return;

	if (orig == tipc_own_addr) {
		if (memcmp(&media_addr, &b_ptr->publ.addr, sizeof(media_addr)))
			disc_dupl_alert(b_ptr, tipc_own_addr, &media_addr);
		return;
	}

	if (!tipc_in_scope(dest, tipc_own_addr))
		return;
	if (!tipc_in_scope(b_ptr->disc_obj->domain, orig))
		return;

        /* We can accept discovery messages from requesting node */

        n_ptr = tipc_net_find_node(orig);
        if (n_ptr == NULL) {
                n_ptr = tipc_node_create(orig);
		if (n_ptr == NULL)
			return;
        }
        tipc_node_lock(n_ptr);

	/* Don't talk to neighbor during cleanup after last session */

	if (n_ptr->cleanup_required) {
		tipc_node_unlock(n_ptr);                
		return;
	}

	/* Prepare to validate requesting node's signature and media address */

	link = n_ptr->links[b_ptr->identity];
	addr_mismatch = (link != NULL) &&
		memcmp(&link->media_addr, &media_addr, sizeof(media_addr));

	/*
	 * Ensure discovery message's signature is correct
	 *
	 * If signature is incorrect and there is no working link to the node,
	 * accept the new signature but invalidate all existing links to the
	 * node so they won't re-activate without a new discovery message.
	 *
	 * If signature is incorrect and the requested link to the node is
	 * working, accept the new signature. (This is an instance of delayed
	 * rediscovery, where a link endpoint was able to re-establish contact
	 * with its peer endpoint on a node that rebooted before receiving a
	 * discovery message from that node.)
	 *
	 * If signature is incorrect and there is a working link to the node
	 * that is not the requested link, reject the request (must be from
	 * a duplicate node).
	 */

	if (signature != n_ptr->signature) {
		if (n_ptr->working_links == 0) {
			struct link *curr_link;
			int i;

			for (i = 0; i < TIPC_MAX_BEARERS; i++) {
				curr_link = n_ptr->links[i];
				if (curr_link) {
					memset(&curr_link->media_addr, 0, 
					       sizeof(media_addr));
					tipc_link_reset(curr_link);
				}
			}
			addr_mismatch = (link != NULL);
		} else if (tipc_link_is_up(link) && !addr_mismatch) {
			/* delayed rediscovery */
		} else {
			disc_dupl_alert(b_ptr, orig, &media_addr);
			tipc_node_unlock(n_ptr);                
			return;
		}
		n_ptr->signature = signature;
	}

	/*
	 * Ensure discovery message's media address is correct
	 *
	 * If media address doesn't match and the link is working, reject the
	 * request (must be from a duplicate node).
	 *
	 * If media address doesn't match and the link is not working, accept
	 * the new media address and reset the link to ensure it starts up
	 * cleanly.
	 */

	if (addr_mismatch) {
		if (tipc_link_is_up(link)) {
			disc_dupl_alert(b_ptr, orig, &media_addr);
			tipc_node_unlock(n_ptr);
			return;
		} else {
			memcpy(&link->media_addr, &media_addr,
			       sizeof(media_addr));
			tipc_link_reset(link);
		}
	}

	/* Create a link endpoint for this bearer, if necessary */

	if (link == NULL) {
#ifndef CONFIG_TIPC_MULTIPLE_LINKS
		if (n_ptr->link_cnt > 0) {
			char node_addr_str[16];

			tipc_addr_string_fill(node_addr_str, orig);
			warn("Ignoring request for second link to node %s\n",
			     node_addr_str);
			tipc_node_unlock(n_ptr);
			return;
		}
#endif
		link = tipc_link_create(b_ptr, orig, &media_addr);
		if (link == NULL) {
			warn("Memory squeeze; Failed to create link\n");
			tipc_node_unlock(n_ptr);
			return;
		}
	}

	/* Accept discovery message & send response, if necessary */

	n_ptr->flags = node_flags;
        link_fully_up = link_working_working(link);

	if ((type == DSC_REQ_MSG) && !link_fully_up && !b_ptr->publ.blocked) {
		rbuf = disc_init_msg(DSC_RESP_MSG, orig, b_ptr);
		if (rbuf != NULL) {
			msg_dbg(buf_msg(rbuf), "SEND:");
			tipc_bearer_send(b_ptr, rbuf, &media_addr);
			buf_discard(rbuf);
		}
	}

	tipc_node_unlock(n_ptr);
}

/**
 * tipc_disc_deactivate - deactivate discoverer searching
 * @d_ptr: ptr to discoverer structure
 */

void tipc_disc_deactivate(struct discoverer *d_ptr)
{
        k_cancel_timer(&d_ptr->timer);
        d_ptr->timer_intv = TIPC_DISC_INACTIVE;
} 

/**
 * tipc_disc_update - update frequency of periodic link setup requests
 * @d_ptr: ptr to discovery structure
 * 
 * Reinitiates discovery process if discoverer has no associated nodes
 * and is either not currently searching or is searching at the slow rate
 */

void tipc_disc_update(struct discoverer *d_ptr) 
{
        if (d_ptr->num_nodes == 0) {
		if ((d_ptr->timer_intv == TIPC_DISC_INACTIVE) ||
		    (d_ptr->timer_intv > TIPC_DISC_FAST)) {
			d_ptr->timer_intv = TIPC_DISC_INIT;
			k_start_timer(&d_ptr->timer, d_ptr->timer_intv);
		}
	}
} 

/**
 * tipc_disc_send_msg - send discovery request message
 * @d_ptr: ptr to discoverer structure
 */

void tipc_disc_send_msg(struct discoverer *d_ptr)
{
        if (!d_ptr->bearer->publ.blocked) {
		msg_dbg(buf_msg(d_ptr->buf), "SEND:");
                tipc_bearer_send(d_ptr->bearer, d_ptr->buf, &d_ptr->dest);
	}
}

/**
 * disc_timeout - send a periodic discovery request
 * @d_ptr: ptr to discoverer structure
 * 
 * Called whenever a link setup request timer associated with a bearer expires.
 */

static void disc_timeout(struct discoverer *d_ptr) 
{
        struct bearer *b_ptr = d_ptr->bearer;
	int max_delay;

	spin_lock_bh(&b_ptr->publ.lock);

	/* See if discovery object can be deactivated */

	if ((tipc_node(d_ptr->domain) != 0) && (d_ptr->num_nodes != 0)) {
		d_ptr->timer_intv = TIPC_DISC_INACTIVE;
		goto exit;
	}

	/* 
	 * Send discovery message, then update discovery timer
	 *
	 * Keep doubling time between requests until limit is reached;
	 * hold at fast polling rate if don't have any associated nodes,
	 * otherwise hold at slow polling rate
	 */

        tipc_disc_send_msg(d_ptr);

        d_ptr->timer_intv *= 2;
	max_delay = (d_ptr->num_nodes == 0) ? TIPC_DISC_FAST : TIPC_DISC_SLOW;
        if (d_ptr->timer_intv > max_delay)
                d_ptr->timer_intv = max_delay;

	k_start_timer(&d_ptr->timer, d_ptr->timer_intv);
exit:
	spin_unlock_bh(&b_ptr->publ.lock);
}

/**
 * tipc_disc_create - start sending periodic discovery requests
 * @b_ptr: ptr to bearer issuing requests
 * @dest: destination address for discovery message
 * @domain: network domain of node(s) to be discovered
 * 
 * Returns 1 if successful, otherwise 0.
 *
 * 'tipc_net_lock' must be write-locked by caller on entry
 */

int tipc_disc_create(struct bearer *b_ptr, struct tipc_media_addr *dest,
                     u32 domain)
{
	struct discoverer *d_ptr;

	d_ptr = kmalloc(sizeof(*d_ptr), GFP_ATOMIC);
	if (!d_ptr)
		return 0;

	d_ptr->buf = disc_init_msg(DSC_REQ_MSG, domain, b_ptr);
	if (!d_ptr->buf) {
		kfree(d_ptr);
		return 0;
	}

	b_ptr->disc_obj = d_ptr;
	d_ptr->bearer = b_ptr;
	memcpy(&d_ptr->dest, dest, sizeof(*dest));
        d_ptr->domain = domain;
	d_ptr->num_nodes = 0;
	d_ptr->timer_intv = TIPC_DISC_INIT;
	k_init_timer(&d_ptr->timer, (Handler)disc_timeout, (unsigned long)d_ptr);
        k_start_timer(&d_ptr->timer, d_ptr->timer_intv);
	tipc_disc_send_msg(d_ptr);
	return 1;
} 

/**
 * tipc_disc_delete - stop sending periodic link setup requests
 * @disc: ptr to link request structure
 * Timer must be cancelled or expired before doing this call
 */

void tipc_disc_delete(struct discoverer *d_ptr) 
{
	if (!d_ptr)
		return;

	k_term_timer(&d_ptr->timer);
	buf_discard(d_ptr->buf);
	kfree(d_ptr);
}

