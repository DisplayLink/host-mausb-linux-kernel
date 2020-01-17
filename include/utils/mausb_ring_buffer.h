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

#include "common/mausb_event.h"
#include "utils/mausb_logs.h"

struct mausb_ring_buffer {
	atomic_t mausb_ring_events;
	atomic_t mausb_completed_user_events;

	struct mausb_event *to_user_buffer;
	int		head;
	int		tail;
	spinlock_t	lock;
	uint64_t	id;

	struct mausb_event *from_user_buffer;
	int current_from_user;

	struct list_head list_entry;
};

int mausb_ring_buffer_init(struct mausb_ring_buffer *ring);
int mausb_ring_buffer_put(struct mausb_ring_buffer *ring,
			  struct mausb_event *event);
int mausb_ring_buffer_move_tail(struct mausb_ring_buffer *ring, uint32_t count);

static inline struct mausb_event *mausb_ring_current_from_user(
						struct mausb_ring_buffer *ring)
{
	return ring->from_user_buffer + ring->current_from_user;
}

static inline void mausb_ring_next_from_user(struct mausb_ring_buffer *ring)
{
	ring->current_from_user = (ring->current_from_user + 1) &
				  (MAUSB_RING_BUFFER_SIZE - 1);
}

void mausb_ring_buffer_cleanup(struct mausb_ring_buffer *ring);
void mausb_ring_buffer_destroy(struct mausb_ring_buffer *ring);

#endif /* ___MAUSB_UTILS_MAUSB_RING_BUFFER_H__ */
