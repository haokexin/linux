/*
 * port.c - hypervisor vbi port library
 *
 * Copyright (c) 2011 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#include <linux/string.h>
#include <vbi/vbi.h>
#include <vbi/private.h>

/*

DESCRIPTION

This module implements the inter-VB communication services through queuing
ports. The APIs in this module are non-blocking. ERROR is returned when sending
a message and the queue is fulll or when receiving a message and the queue
is empty

*/

#ifdef DEBUG
#define DEBUGM(x) x
#else
#define DEBUGM(x)
#endif

/*******************************************************************************
*
* vbi_port_id_get - return the port id given his name
*
* The name is case sensitive
*
* RETURNS: OK if the port name was found.
*	   ERROR otherwise.
*
* ERRNO: N/A
*
* SEE ALSO: vbi_port_send(), vbi_port_receive().
*/
int32_t vbi_port_id_get(char *name,  uint32_t *vb_port_id)
{
	uint32_t i;
	int32_t ret_val = -1;

	/* go through all ports config to find a matching name */
	for (i = 0; i < VBI_CONFIG_ADDR_GET()->num_ports; i++) {
		if (strncmp((const char *)name,
			(const char *)(&(VBI_CONFIG_ADDR_GET()->port_config[i].name)),
			VB_NAMELEN) == 0) {
				*vb_port_id =
				VBI_CONFIG_ADDR_GET()->port_config[i].vb_port_id;
				ret_val = 0;
				break;
			}
	}
	return ret_val;
}

/*******************************************************************************
*
* vbi_port_send - send a message to a queuing port
*
* This routine copies the message to send to the source port's message buffer,
* if space is available, and makes a hypercall.
*
* RETURNS: OK if message could be copied to the source port message queue.
*	   ERROR otherwise.
*
* ERRNO: N/A
*
* SEE ALSO: vbi_port_receive(), vbi_port_id_get().
*/

int32_t vbi_port_send(uint32_t vb_port_id, const char *message, size_t length)
{
	int32_t status = 0;
	uint8_t *s_buf;
	struct vb_port_status *port_status;
	struct vb_port_config *port_config;

	/* make sure the port id is valid and it is a source port and that the
	 * message is not too long for this port
	 */

	if ((vb_port_id >= VBI_CONFIG_ADDR_GET()->num_ports) ||
		(VBI_CONFIG_ADDR_GET()->port_config[vb_port_id].direction != SOURCE) ||
		(VBI_CONFIG_ADDR_GET()->port_config[vb_port_id].max_msg_size < length)) {
		return -1;
	}

	/* lock interrupt when manipulating port status fields */
	vbi_vcore_irq_lock();

	/* make sure port is not full */
	if (VBI_CONFIG_ADDR_GET()->vb_status->port_status[vb_port_id].num_msg ==
		VBI_CONFIG_ADDR_GET()->port_config[vb_port_id].max_num_msgs) {
		vbi_vcore_irq_unlock();
		return -1;
	}

	port_status =
		&(VBI_CONFIG_ADDR_GET()->vb_status->port_status[vb_port_id]);

	DEBUGM(printk(KERN_INFO
		"vbi_port_send: port_status.s_buf %p\n", port_status->s_buf);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_send: port_status.msg_len_array %p\n",
		port_status->msg_len_array);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_send: port_status.write_index %d\n",
		port_status->write_index);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_send: port_status.read_index %d\n",
		port_status->read_index);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_send: port_status.num_msg %d\n",
		port_status->num_msg);)

	port_config = &(VBI_CONFIG_ADDR_GET()->port_config[vb_port_id]);

	/* get the pointer to the buffer where the message should be copied */
	s_buf = port_status->s_buf +
		(port_status->write_index * port_status->mem_align_msg_size);
	DEBUGM(KERN_INFO printk("vbi_port_send: s_buf %p\n", s_buf);)

	/* copy the message in the queue */
	memcpy((void *)message, (void *)s_buf, length);

	/* make a hypercall to notify HV that there is a new message
	 * in the queue
	 */
	status = vbi_port_op(PORT_SEND, vb_port_id, length);

	vbi_vcore_irq_unlock();

	return status;
}

/*******************************************************************************
*
* vbi_port_receive - receive a message from a queuing port
*
* This routine copies to the provided buffer a message from the port's queue,
* if any.
*
* RETURNS: OK if a message has been received,
	   ERROR otherwise.
*
* ERRNO: N/A
*
* SEE ALSO: vbi_port_send(), vbi_port_id_get()
*/

int32_t vbi_port_receive(uint32_t vb_port_id, char *buffer, size_t *length)
{

	uint8_t *s_buf;
	uint32_t msg_len;
	int32_t	status;
	struct vb_port_status *port_status;
	struct vb_port_config *port_config;

	/* make sure the port id is valid and that itis a destination port */
	if ((vb_port_id >= VBI_CONFIG_ADDR_GET()->num_ports) ||
		(VBI_CONFIG_ADDR_GET()->port_config[vb_port_id].direction !=
		DESTINATION)) {
		return -1;
	}

	/* lock interrupt when manipulating port status fields */
	vbi_vcore_irq_lock();

	/* verify if there is any mssage in the destination queue */
	if (VBI_CONFIG_ADDR_GET()->vb_status->port_status[vb_port_id].num_msg
		 == 0) {
		vbi_vcore_irq_unlock();
		return -1;
	}

	port_status =
		&(VBI_CONFIG_ADDR_GET()->vb_status->port_status[vb_port_id]);

	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: port_status.s_buf %p\n",
		port_status->s_buf);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: port_status.msg_len_array %p\n",
		port_status->msg_len_array);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: port_status.write_index %d\n",
		port_status->write_index);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: port_status.read_index %d\n",
		port_status->read_index);)
	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: port_status.num_msg %d\n",
		port_status->num_msg);)

	port_config = &(VBI_CONFIG_ADDR_GET()->port_config[vb_port_id]);

	/* get the pointer to the buffer where the message to be read reside */
	s_buf = port_status->s_buf +
		(port_status->read_index * port_status->mem_align_msg_size);
	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: s_buf %p\n", s_buf);)

	/* get the length of the message to be received */
	msg_len = port_status->msg_len_array[port_status->read_index];
	DEBUGM(printk(KERN_INFO
		"vbi_port_receive: msg_len %d\n", msg_len);)

	/* copy the message to the buffer provided */
	memcpy((void *)s_buf, (void *)buffer, msg_len);

	/* make a hypercall to notify HV that a message has been read */
	status = vbi_port_op((int)PORT_BUF_RELEASE, (int)vb_port_id, 0);

	vbi_vcore_irq_unlock();

	/* return the message length */
	*length = msg_len;

	return status;
}
