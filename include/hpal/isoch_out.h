/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_ISOCH_OUT_H__
#define __MAUSB_HPAL_ISOCH_OUT_H__

#include "common/mausb_event.h"
#include "hpal/hpal.h"

int mausb_send_isoch_out_msg(struct mausb_device *ma_dev,
			     struct mausb_event *mausb_event,
			     struct mausb_urb_ctx *urb_ctx);
int mausb_receive_isoch_out(struct mausb_event *event);

#endif /* __MAUSB_HPAL_ISOCH_OUT_H__ */
