/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_UTILS_MAUSB_MMAP_H__
#define __MAUSB_UTILS_MAUSB_MMAP_H__

#include "utils/mausb_ring_buffer.h"

int mausb_create_dev(void);

void mausb_cleanup_dev(int device_created);
void mausb_notify_completed_user_events(struct mausb_ring_buffer *ring_buffer);
void mausb_notify_ring_events(struct mausb_ring_buffer *ring_buffer);
void mausb_stop_ring_events(void);

#endif /* __MAUSB_UTILS_MAUSB_MMAP_H__ */
