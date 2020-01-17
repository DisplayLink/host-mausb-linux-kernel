/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_ISOCH_IN_H__
#define __MAUSB_HPAL_ISOCH_IN_H__

#include "hpal/hpal.h"

#define MAUSB_ISOCH_IN_KVEC_NUM 3

int mausb_send_isoch_in_msg(struct mausb_device *dev,
			    struct mausb_event *event);
int mausb_receive_isoch_in_data(struct mausb_device *dev,
				struct mausb_event *event,
				struct mausb_urb_ctx *urb_ctx);

#endif /* __MAUSB_HPAL_ISOCH_IN_H__ */
