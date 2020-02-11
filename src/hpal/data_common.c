// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hpal/data_common.h"

#include "hpal/data_in.h"
#include "hpal/data_out.h"
#include "hpal/isoch_in.h"
#include "hpal/isoch_out.h"
#include "utils/mausb_logs.h"

static inline
struct mausb_ip_ctx *mausb_get_data_channel(struct mausb_device *ma_dev,
					    enum mausb_channel channel)
{
	if (channel >= MAUSB_CHANNEL_MAP_LENGTH)
		return NULL;

	return ma_dev->channel_map[channel];
}

int mausb_send_data(struct mausb_device *dev, enum mausb_channel channel_num,
		    struct mausb_kvec_data_wrapper *data)
{
	struct mausb_ip_ctx *channel = mausb_get_data_channel(dev, channel_num);
	int status = 0;

	if (!channel)
		return -ECHRNG;

	status = mausb_ip_send(channel, data);

	if (status < 0) {
		mausb_pr_err("Send failed. Disconnecting... status=%d", status);
		queue_work(dev->workq, &dev->socket_disconnect_work);
		queue_work(dev->workq, &dev->hcd_disconnect_work);
	}

	return status;
}

int mausb_send_transfer_ack(struct mausb_device *dev, struct mausb_event *event)
{
	struct ma_usb_hdr_common *ack_hdr;
	struct kvec kvec;
	struct mausb_kvec_data_wrapper data_to_send;
	enum mausb_channel channel;

	ack_hdr = (struct ma_usb_hdr_common *)(&event->data.hdr_ack);

	data_to_send.kvec	    = &kvec;
	data_to_send.kvec->iov_base = ack_hdr;
	data_to_send.kvec->iov_len  = ack_hdr->length;
	data_to_send.kvec_num	    = 1;
	data_to_send.length	    = ack_hdr->length;

	channel = mausb_transfer_type_to_channel(event->data.transfer_type);
	return mausb_send_data(dev, channel, &data_to_send);
}

int mausb_send_data_msg(struct mausb_device *dev, struct mausb_event *event)
{
	struct mausb_urb_ctx *urb_ctx;
	int status = 0;

	if (event->status != 0) {
		mausb_pr_err("Event %d failed with status %d",
			     event->type, event->status);
		mausb_complete_urb(event);
		return event->status;
	}

	urb_ctx = mausb_find_urb_in_tree((struct urb *)event->data.urb);

	if (!urb_ctx) {
		/* Transfer will be deleted from dequeue task */
		mausb_pr_warn("Urb is already cancelled for event=%d",
			      event->type);
		return status;
	}

	if (mausb_isoch_data_event(event)) {
		if (event->data.direction == MAUSB_DATA_MSG_DIRECTION_IN)
			status = mausb_send_isoch_in_msg(dev, event);
		else
			status = mausb_send_isoch_out_msg(dev, event, urb_ctx);
	} else {
		if (event->data.direction == MAUSB_DATA_MSG_DIRECTION_IN)
			status = mausb_send_in_data_msg(dev, event);
		else
			status = mausb_send_out_data_msg(dev, event, urb_ctx);
	}

	return status;
}

int mausb_receive_data_msg(struct mausb_device *dev, struct mausb_event *event)
{
	int status = 0;
	struct mausb_urb_ctx *urb_ctx;

	mausb_pr_debug("Direction=%d", event->data.direction);

	if (!mausb_isoch_data_event(event)) {
		status = mausb_send_transfer_ack(dev, event);
		if (status < 0) {
			mausb_pr_warn("Sending acknowledgment failed");
			goto cleanup;
		}
	}

	urb_ctx = mausb_find_urb_in_tree((struct urb *)event->data.urb);

	if (!urb_ctx) {
		/* Transfer will be deleted from dequeue task */
		mausb_pr_warn("Urb is already cancelled");
		goto cleanup;
	}

	if (mausb_isoch_data_event(event)) {
		if (event->data.direction == MAUSB_DATA_MSG_DIRECTION_IN)
			status = mausb_receive_isoch_in_data(dev, event,
							     urb_ctx);
		else
			status = mausb_receive_isoch_out(event);
	} else {
		if (event->data.direction == MAUSB_DATA_MSG_DIRECTION_IN)
			mausb_receive_in_data(event, urb_ctx);
		else
			mausb_receive_out_data(event, urb_ctx);
	}

cleanup:
	mausb_release_event_resources(event);
	return status;
}
