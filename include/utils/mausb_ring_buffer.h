/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef ___MAUSB_UTILS_MAUSB_RING_BUFFER_H__
#define ___MAUSB_UTILS_MAUSB_RING_BUFFER_H__

#include <linux/spinlock.h>
#include <linux/log2.h>

#include "common/mausb_event.h"
#include "utils/mausb_logs.h"

struct mausb_ring_buffer {
	atomic_t mausb_ring_events;
	atomic_t mausb_completed_user_events;

	struct mausb_event *to_user_buffer;
	int		head;
	int		tail;
	spinlock_t	lock; /* Protect ring buffer */
	u64		id;

	struct mausb_event *from_user_buffer;
	int current_from_user;

	struct list_head list_entry;
	bool buffer_full;
};

int mausb_ring_buffer_init(struct mausb_ring_buffer *ring);
int mausb_ring_buffer_put(struct mausb_ring_buffer *ring,
			  struct mausb_event *event);
int mausb_ring_buffer_move_tail(struct mausb_ring_buffer *ring, u32 count);
void mausb_ring_buffer_cleanup(struct mausb_ring_buffer *ring);
void mausb_ring_buffer_destroy(struct mausb_ring_buffer *ring);
void mausb_cleanup_ring_buffer_event(struct mausb_event *event);
void mausb_disconect_event_unsafe(struct mausb_ring_buffer *ring,
				  uint8_t madev_addr);

static inline unsigned int mausb_get_page_order(unsigned int num_of_elems,
						unsigned int elem_size)
{
	unsigned int num_of_pages = DIV_ROUND_UP(num_of_elems * elem_size,
						 PAGE_SIZE);
	unsigned int order = (unsigned int)ilog2(num_of_pages) +
					(is_power_of_2(num_of_pages) ? 0 : 1);
	return order;
}

static inline
struct mausb_event *mausb_ring_current_from_user(struct mausb_ring_buffer *ring)
{
	return ring->from_user_buffer + ring->current_from_user;
}

static inline void mausb_ring_next_from_user(struct mausb_ring_buffer *ring)
{
	ring->current_from_user = (ring->current_from_user + 1) &
				  (MAUSB_RING_BUFFER_SIZE - 1);
}

#endif /* ___MAUSB_UTILS_MAUSB_RING_BUFFER_H__ */
