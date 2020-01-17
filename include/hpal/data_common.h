/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_DATA_COMMON_H__
#define __MAUSB_HPAL_DATA_COMMON_H__

#include "hpal/hpal.h"

int mausb_send_data(struct mausb_device *dev, enum mausb_channel channel_num,
		    struct mausb_kvec_data_wrapper *data);

int mausb_send_transfer_ack(struct mausb_device *dev,
			    struct mausb_event *event);

int mausb_send_data_msg(struct mausb_device *dev, struct mausb_event *event);

int mausb_receive_data_msg(struct mausb_device *dev, struct mausb_event *event);

static inline bool mausb_ctrl_transfer(struct ma_usb_hdr_common *hdr)
{
	return (hdr->data.t_flags & MA_USB_DATA_TFLAGS_TRANSFER_TYPE_MASK) ==
		MA_USB_DATA_TFLAGS_TRANSFER_TYPE_CTRL;
}

static inline bool mausb_isoch_transfer(struct ma_usb_hdr_common *hdr)
{
	return (hdr->data.t_flags & MA_USB_DATA_TFLAGS_TRANSFER_TYPE_MASK) ==
		MA_USB_DATA_TFLAGS_TRANSFER_TYPE_ISOCH;
}

static inline bool mausb_ctrl_data_event(struct mausb_event *event)
{
	return event->data.transfer_type ==
		MA_USB_DATA_TFLAGS_TRANSFER_TYPE_CTRL;
}

static inline bool mausb_isoch_data_event(struct mausb_event *event)
{
	return event->data.transfer_type ==
		MA_USB_DATA_TFLAGS_TRANSFER_TYPE_ISOCH;
}

/* usb to mausb transfer type */
static inline uint8_t mausb_transfer_type_from_usb(
					struct usb_endpoint_descriptor *epd)
{
	return (uint8_t)usb_endpoint_type(epd) << 3;
}

static inline uint8_t mausb_transfer_type_from_hdr(
						struct ma_usb_hdr_common *hdr)
{
	return hdr->data.t_flags & MA_USB_DATA_TFLAGS_TRANSFER_TYPE_MASK;
}

static inline enum mausb_channel mausb_transfer_type_to_channel(
						uint8_t transfer_type)
{
	return transfer_type >> 3;
}

#endif /* __MAUSB_HPAL_DATA_COMMON_H__ */
