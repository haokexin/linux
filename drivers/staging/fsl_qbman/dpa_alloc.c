/* Copyright 2009-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dpa_sys.h"

/* Qman and Bman APIs are front-ends to the common code; */

static DECLARE_DPA_ALLOC(bpalloc);
static DECLARE_DPA_ALLOC(fqalloc);

int bman_alloc_bpid_range(u32 *result, u32 count, u32 align, int partial)
{
	return dpa_alloc_new(&bpalloc, result, count, align, partial);
}
EXPORT_SYMBOL(bman_alloc_bpid_range);

void bman_release_bpid_range(u32 bpid, u32 count)
{
	dpa_alloc_free(&bpalloc, bpid, count);
}
EXPORT_SYMBOL(bman_release_bpid_range);

int qman_alloc_fqid_range(u32 *result, u32 count, u32 align, int partial)
{
	return dpa_alloc_new(&fqalloc, result, count, align, partial);
}
EXPORT_SYMBOL(qman_alloc_fqid_range);

void qman_release_fqid_range(u32 fqid, u32 count)
{
	dpa_alloc_free(&fqalloc, fqid, count);
}
EXPORT_SYMBOL(qman_release_fqid_range);

/* The rest is the common backend to the Qman and Bman allocators */

/* The allocator is a (possibly-empty) list of these; */
struct alloc_node {
	struct list_head list;
	u32 base;
	u32 num;
};

/* #define DPA_ALLOC_DEBUG */

#ifdef DPA_ALLOC_DEBUG
#define DPRINT pr_info
static void DUMP(struct dpa_alloc *alloc)
{
	int off = 0;
	char buf[256];
	struct alloc_node *p;
	list_for_each_entry(p, &alloc->list, list) {
		if (off < 255)
			off += snprintf(buf + off, 255-off, "{%d,%d}",
				p->base, p->base + p->num - 1);
	}
	pr_info("%s\n", buf);
}
#else
#define DPRINT(x...)	do { ; } while (0)
#define DUMP(a)		do { ; } while (0)
#endif

int dpa_alloc_new(struct dpa_alloc *alloc, u32 *result, u32 count, u32 align,
		  int partial)
{
	struct alloc_node *i = NULL, *next_best = NULL;
	u32 base, next_best_base = 0, num = 0, next_best_num = 0;
	struct alloc_node *margin_left, *margin_right;

	*result = (u32)-1;
	DPRINT("alloc_range(%d,%d,%d)\n", count, align, partial);
	DUMP(alloc);
	/* If 'align' is 0, it should behave as though it was 1 */
	if (!align)
		align = 1;
	margin_left = kmalloc(sizeof(*margin_left), GFP_KERNEL);
	if (!margin_left)
		goto err;
	margin_right = kmalloc(sizeof(*margin_right), GFP_KERNEL);
	if (!margin_right) {
		kfree(margin_left);
		goto err;
	}
	spin_lock_irq(&alloc->lock);
	list_for_each_entry(i, &alloc->list, list) {
		base = (i->base + align - 1) / align;
		base *= align;
		if ((base - i->base) >= i->num)
			/* alignment is impossible, regardless of count */
			continue;
		num = i->num - (base - i->base);
		if (num >= count) {
			/* this one will do nicely */
			num = count;
			goto done;
		}
		if (num > next_best_num) {
			next_best = i;
			next_best_base = base;
			next_best_num = num;
		}
	}
	if (partial && next_best) {
		i = next_best;
		base = next_best_base;
		num = next_best_num;
	} else
		i = NULL;
done:
	if (i) {
		if (base != i->base) {
			margin_left->base = i->base;
			margin_left->num = base - i->base;
			list_add_tail(&margin_left->list, &i->list);
		} else
			kfree(margin_left);
		if ((base + num) < (i->base + i->num)) {
			margin_right->base = base + num;
			margin_right->num = (i->base + i->num) -
						(base + num);
			list_add(&margin_right->list, &i->list);
		} else
			kfree(margin_right);
		list_del(&i->list);
		kfree(i);
		*result = base;
	}
	spin_unlock_irq(&alloc->lock);
err:
	DPRINT("returning %d\n", i ? num : -ENOMEM);
	DUMP(alloc);
	return i ? (int)num : -ENOMEM;
}

/* Allocate the list node using GFP_ATOMIC, because we *really* want to avoid
 * forcing error-handling on to users in the deallocation path. */
void dpa_alloc_free(struct dpa_alloc *alloc, u32 fqid, u32 count)
{
	struct alloc_node *i, *node = kmalloc(sizeof(*node), GFP_ATOMIC);
	BUG_ON(!node);
	DPRINT("release_range(%d,%d)\n", fqid, count);
	DUMP(alloc);
	BUG_ON(!count);
	spin_lock_irq(&alloc->lock);
	node->base = fqid;
	node->num = count;
	list_for_each_entry(i, &alloc->list, list) {
		if (i->base >= node->base) {
			/* BUG_ON(any overlapping) */
			BUG_ON(i->base < (node->base + node->num));
			list_add_tail(&node->list, &i->list);
			goto done;
		}
	}
	list_add_tail(&node->list, &alloc->list);
done:
	/* Merge to the left */
	i = list_entry(node->list.prev, struct alloc_node, list);
	if (node->list.prev != &alloc->list) {
		BUG_ON((i->base + i->num) > node->base);
		if ((i->base + i->num) == node->base) {
			node->base = i->base;
			node->num += i->num;
			list_del(&i->list);
			kfree(i);
		}
	}
	/* Merge to the right */
	i = list_entry(node->list.next, struct alloc_node, list);
	if (node->list.next != &alloc->list) {
		BUG_ON((node->base + node->num) > i->base);
		if ((node->base + node->num) == i->base) {
			node->num += i->num;
			list_del(&i->list);
			kfree(i);
		}
	}
	spin_unlock_irq(&alloc->lock);
	DUMP(alloc);
}
