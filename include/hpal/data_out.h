/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_DATA_OUT_H__
#define __MAUSB_HPAL_DATA_OUT_H__

#include <linux/types.h>

#include "hpal/mausb_events.h"
#include "utils/mausb_data_iterator.h"

int mausb_send_out_data_msg(struct mausb_device *dev, struct mausb_event *event,
			    struct mausb_urb_ctx *urb_ctx);
int mausb_receive_out_data(struct mausb_device *dev, struct mausb_event *event,
			   struct mausb_urb_ctx *urb_ctx);

#endif /* __MAUSB_HPAL_DATA_OUT_H__ */
