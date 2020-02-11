/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_DATA_IN_H__
#define __MAUSB_HPAL_DATA_IN_H__

#include "hpal/hpal.h"

int mausb_send_in_data_msg(struct mausb_device *dev, struct mausb_event *event);
void mausb_receive_in_data(struct mausb_event *event,
			   struct mausb_urb_ctx *urb_ctx);

#endif /* __MAUSB_HPAL_DATA_IN_H__ */
