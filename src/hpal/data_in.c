// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hpal/data_in.h"

#include "hcd/hub.h"
#include "hpal/data_common.h"
#include "hpal/mausb_events.h"
#include "utils/mausb_data_iterator.h"
#include "utils/mausb_logs.h"

int mausb_send_in_data_msg(struct mausb_device *dev, struct mausb_event *event)
{
	int status = 0;
	struct mausb_kvec_data_wrapper data_to_send;
	struct kvec kvec[2];
	struct urb *urb   = (struct urb *)(event->data.urb);
	bool setup_packet = (usb_endpoint_xfer_control(&urb->ep->desc) &&
			     urb->setup_packet);
	uint32_t kvec_num = setup_packet ? 2 : 1;

	data_to_send.kvec_num	= kvec_num;
	data_to_send.length	= MAUSB_TRANSFER_HDR_SIZE +
					(setup_packet ?
						MAUSB_CONTROL_SETUP_SIZE :
						0);

	/* Prepare transfer header kvec */
	kvec[0].iov_base = event->data.hdr;
	kvec[0].iov_len  = MAUSB_TRANSFER_HDR_SIZE;

	/* Prepare setup packet kvec */
	if (setup_packet) {
		kvec[1].iov_base = urb->setup_packet;
		kvec[1].iov_len  = MAUSB_CONTROL_SETUP_SIZE;
	}
	data_to_send.kvec = kvec;

	status = mausb_send_data(dev, mausb_transfer_type_to_channel(
						event->data.transfer_type),
				&data_to_send);

	return status;
}

int mausb_receive_in_data(struct mausb_device *dev, struct mausb_event *event,
			  struct mausb_urb_ctx *urb_ctx)
{
	struct urb *urb = urb_ctx->urb;
	struct mausb_data_iter *iterator     = &urb_ctx->iterator;
	struct ma_usb_hdr_common *common_hdr =
			(struct ma_usb_hdr_common *)event->data.recv_buf;
	uint16_t payload_size = common_hdr->length - MAUSB_TRANSFER_HDR_SIZE;
	uint32_t data_written = 0;
	int status = 0;

	mausb_pr_debug("");

	data_written = mausb_data_iterator_write(iterator, shift_ptr(common_hdr,
						 MAUSB_TRANSFER_HDR_SIZE),
						 payload_size);

	mausb_pr_debug("data_written=%d, payload_size=%d", data_written,
		       payload_size);
	event->data.rem_transfer_size -= data_written;

	if (event->data.transfer_eot) {
		mausb_pr_debug("transfer_size=%d, rem_transfer_size=%d, status=%d",
			       event->data.transfer_size,
			       event->data.rem_transfer_size, event->status);
		mausb_complete_request(urb, event->data.transfer_size -
				       event->data.rem_transfer_size,
				       event->status);
	}

	return status;
}
