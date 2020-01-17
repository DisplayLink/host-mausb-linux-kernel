/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_NETWORK_CALLBACKS_H__
#define __MAUSB_HPAL_NETWORK_CALLBACKS_H__

#include <linux/workqueue.h>

#include "link/mausb_ip_link.h"

/* generic callback by default */
void mausb_ip_callback(void *ctx, enum mausb_channel channel,
		       enum mausb_link_action action, int status, void *data);

#endif /* __MAUSB_HPAL_NETWORK_CALLBACKS_H__ */
