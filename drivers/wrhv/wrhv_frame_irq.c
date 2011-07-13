/*
 *  Copyright (C) 2011 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wrhv.h>
#include <vbi/vbi.h>
#include <vbi/errors.h>

#define START_FRAME_IRQ_NAME	"start"
#define END_FRAME_IRQ_NAME	"end"

/* a simple start of frame interrupt handler */
static irqreturn_t start_frame_interrupt(int irq, void *p)
{
	return IRQ_HANDLED;
}

/* a simple end of frame interrupt handler */
static irqreturn_t end_frame_interrupt(int irq, void *p)
{
	return IRQ_HANDLED;
}

/* certifiable hypervisor support start and end frame transition interrupts.
   User can make use of this interrupt to maintain house keeping related tasks.
*/
static int __init frame_transition_test_init(void)
{
	int rc;
	int start_irq;
	int end_irq;

	printk(KERN_INFO "Initialize frame transition interrupt test");

	/* scheduled transition vbi, the schedule name "init" is
	 * defined in the scheduler section of wrhvConfig.xml
	 */
	rc = vbi_sched_transition("init", SCHEDULER_TRANSITION_MAJOR,
		VBI_VCORE_ID_GET());

	if (rc) {
		printk(KERN_WARNING
			"Failed to schedule transition %s core id=%d\n", "init",
			 VBI_VCORE_ID_GET());
	}

	start_irq = vbi_find_irq(START_FRAME_IRQ_NAME, 1);
	if (start_irq != VBI_INVALID_IRQ) {
		/* request start frame interrupt handler */
		rc = request_irq(start_irq, start_frame_interrupt,
				IRQF_SHARED, "start_frame_interrupt",
				start_frame_interrupt);
		if (rc) {
			printk(KERN_WARNING 
				"Failed to request start frame irq at %d\n", start_irq);
		}
	}

	end_irq = vbi_find_irq(END_FRAME_IRQ_NAME, 1);
	if (end_irq != VBI_INVALID_IRQ) {
		/* request end frame interrupt handler */
		rc = request_irq(end_irq, end_frame_interrupt,
				IRQF_SHARED, "end_frame_interrupt",
				end_frame_interrupt);
		if (rc) {
			printk(KERN_WARNING 
				"Failed to request start frame irq at %d\n", end_irq);
		}
	}

	return rc;
}
module_init(frame_transition_test_init);
