// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "utils/mausb_ring_buffer.h"

#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/log2.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "hpal/mausb_events.h"
#include "utils/mausb_logs.h"

static void mausb_cleanup_ring_buffer_event(struct mausb_event *event)
{
	mausb_pr_debug("event=%d", event->type);
	switch (event->type) {
	case MAUSB_EVENT_TYPE_SEND_DATA_MSG:
		mausb_cleanup_send_data_msg_event(event);
		break;
	case MAUSB_EVENT_TYPE_RECEIVED_DATA_MSG:
		mausb_cleanup_received_data_msg_event(event);
		break;
	case MAUSB_EVENT_TYPE_DELETE_DATA_TRANSFER:
		mausb_cleanup_delete_data_transfer_event(event);
		break;
	case MAUSB_EVENT_TYPE_NONE:
		break;
	default:
		mausb_pr_warn("Unknown event type");
		break;
	}
}

static int mausb_get_page_order(unsigned int num_of_elems,
				unsigned int elem_size)
{
	unsigned int num_of_pages = DIV_ROUND_UP(
					num_of_elems * elem_size, PAGE_SIZE);
	unsigned int order = ilog2(num_of_pages) +
			     (is_power_of_2(num_of_pages) ? 0 : 1);
	return order;
}

int mausb_ring_buffer_init(struct mausb_ring_buffer *ring)
{
	int page_order = mausb_get_page_order(2 * MAUSB_RING_BUFFER_SIZE,
					sizeof(struct mausb_event));
	ring->to_user_buffer =
		(struct mausb_event *)__get_free_pages(GFP_KERNEL, page_order);
	if (!ring->to_user_buffer)
		return -ENOMEM;
	ring->from_user_buffer = ring->to_user_buffer + MAUSB_RING_BUFFER_SIZE;
	ring->head = 0;
	ring->tail = 0;
	ring->current_from_user = 0;
	spin_lock_init(&ring->lock);

	return 0;
}

int mausb_ring_buffer_put(struct mausb_ring_buffer *ring,
			  struct mausb_event *event)
{
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);
	if (CIRC_SPACE(ring->head, ring->tail, MAUSB_RING_BUFFER_SIZE) < 1) {
		spin_unlock_irqrestore(&ring->lock, flags);
		return -ENOSPC;
	}
	memcpy(ring->to_user_buffer + ring->head, event, sizeof(*event));
	mausb_pr_debug("HEAD=%d, TAIL=%d", ring->head, ring->tail);
	ring->head = (ring->head + 1) & (MAUSB_RING_BUFFER_SIZE - 1);
	mausb_pr_debug("HEAD=%d, TAIL=%d", ring->head, ring->tail);
	spin_unlock_irqrestore(&ring->lock, flags);
	return 0;
}

static int mausb_ring_buffer_get(struct mausb_ring_buffer *ring,
				struct mausb_event *event)
{
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);
	if (CIRC_CNT(ring->head, ring->tail, MAUSB_RING_BUFFER_SIZE) < 1) {
		spin_unlock_irqrestore(&ring->lock, flags);
		return -ENOSPC;
	}
	memcpy(event, ring->to_user_buffer + ring->tail, sizeof(*event));
	mausb_pr_debug("HEAD=%d, TAIL=%d", ring->head, ring->tail);
	ring->tail = (ring->tail + 1) & (MAUSB_RING_BUFFER_SIZE - 1);
	mausb_pr_debug("HEAD=%d, TAIL=%d", ring->head, ring->tail);
	spin_unlock_irqrestore(&ring->lock, flags);
	return 0;
}

void mausb_ring_buffer_cleanup(struct mausb_ring_buffer *ring)
{
	struct mausb_event event;

	mausb_pr_info("");
	while (mausb_ring_buffer_get(ring, &event) == 0)
		mausb_cleanup_ring_buffer_event(&event);
}

void mausb_ring_buffer_destroy(struct mausb_ring_buffer *ring)
{
	int page_order = mausb_get_page_order(2 * MAUSB_RING_BUFFER_SIZE,
					      sizeof(struct mausb_event));
	if (ring && ring->to_user_buffer)
		free_pages((unsigned long)ring->to_user_buffer, page_order);
}

int mausb_ring_buffer_move_tail(struct mausb_ring_buffer *ring, uint32_t count)
{
	unsigned long flags;

	spin_lock_irqsave(&ring->lock, flags);
	if (CIRC_CNT(ring->head, ring->tail, MAUSB_RING_BUFFER_SIZE) < count) {
		spin_unlock_irqrestore(&ring->lock, flags);
		return -ENOSPC;
	}
	mausb_pr_debug("old HEAD=%d, TAIL=%d", ring->head, ring->tail);
	ring->tail = (ring->tail + count) & (MAUSB_RING_BUFFER_SIZE - 1);
	mausb_pr_debug("new HEAD=%d, TAIL=%d", ring->head, ring->tail);
	spin_unlock_irqrestore(&ring->lock, flags);
	return 0;
}
