// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include <hpal/data_out.h>

#include <linux/slab.h>

#include "hcd/hub.h"
#include "hpal/data_common.h"
#include "utils/mausb_data_iterator.h"
#include "utils/mausb_logs.h"

static int mausb_add_data_chunk(void *buffer, u32 buffer_size,
			 struct list_head *chunks_list)
{
	struct mausb_payload_chunk *data_chunk;

	data_chunk = kzalloc(sizeof(*data_chunk), GFP_KERNEL);
	if (!data_chunk)
		return -ENOMEM;

	/* Initialize data chunk for MAUSB header and add it to chunks list */
	INIT_LIST_HEAD(&data_chunk->list_entry);

	data_chunk->kvec.iov_base = buffer;
	data_chunk->kvec.iov_len  = buffer_size;
	list_add_tail(&data_chunk->list_entry, chunks_list);
	return 0;
}

static int mausb_init_data_wrapper(struct mausb_kvec_data_wrapper *data,
				   struct list_head *chunks_list,
				   uint32_t num_of_data_chunks)
{
	struct mausb_payload_chunk *data_chunk = NULL,
				   *tmp = NULL;
	uint32_t current_kvec = 0;

	data->length = 0;
	data->kvec =
	    kcalloc(num_of_data_chunks, sizeof(struct kvec), GFP_KERNEL);
	if (!data->kvec)
		return -ENOMEM;

	list_for_each_entry_safe(data_chunk, tmp, chunks_list, list_entry) {
		data->kvec[current_kvec].iov_base =
			data_chunk->kvec.iov_base;
		data->kvec[current_kvec].iov_len  =
		    data_chunk->kvec.iov_len;
		++data->kvec_num;
		data->length += data_chunk->kvec.iov_len;
		++current_kvec;
	}
	return 0;
}

static int mausb_init_header_data_chunk(struct ma_usb_hdr_common *common_hdr,
					struct list_head *chunks_list,
					uint32_t *num_of_data_chunks)
{
	int status = mausb_add_data_chunk(common_hdr,
					  MAUSB_TRANSFER_HDR_SIZE,
					  chunks_list);
	/* Success */
	if (!status)
		++(*num_of_data_chunks);

	return status;
}

static int mausb_init_control_data_chunk(struct mausb_event *event,
				  struct list_head *chunks_list,
				  uint32_t *num_of_data_chunks)
{
	int status = 0;

	if (!event->data.first_control_packet)
		goto l_return;

	status = mausb_add_data_chunk(
				((struct urb *)event->data.urb)->setup_packet,
				MAUSB_CONTROL_SETUP_SIZE, chunks_list);
	/* Success */
	if (!status)
		++(*num_of_data_chunks);

l_return:
	return status;
}

static void mausb_cleanup_chunks_list(struct list_head *chunks_list)
{
	struct mausb_payload_chunk *data_chunk = NULL, *tmp = NULL;

	list_for_each_entry_safe(data_chunk, tmp, chunks_list, list_entry) {
		list_del(&data_chunk->list_entry);
		kfree(data_chunk);
	}
}

static int mausb_prepare_transfer_packet(
		struct mausb_kvec_data_wrapper *wrapper,
				  struct mausb_event *event,
				  struct mausb_data_iter *iterator)
{
	uint32_t num_of_data_chunks	    = 0;
	uint32_t num_of_payload_data_chunks = 0;
	uint32_t payload_data_size	    = 0;
	struct list_head chunks_list;
	struct list_head payload_data_chunks;
	int status = 0;

	INIT_LIST_HEAD(&chunks_list);

	/* Initialize data chunk for MAUSB header and add it to chunks list */
	if (mausb_init_header_data_chunk(
				(struct ma_usb_hdr_common *)event->data.hdr,
				&chunks_list, &num_of_data_chunks) < 0) {
		status = -ENOMEM;
		goto l_cleanup_data_chunks;
	}

	/*
	 * Initialize data chunk for MAUSB control setup packet and
	 * add it to chunks list
	 */
	if (mausb_init_control_data_chunk(event, &chunks_list,
					  &num_of_data_chunks) < 0) {
		status = -ENOMEM;
		goto l_cleanup_data_chunks;
	}

	/* Get data chunks for data payload to send */
	INIT_LIST_HEAD(&payload_data_chunks);
	payload_data_size =
			((struct ma_usb_hdr_common *)event->data.hdr)->length -
			MAUSB_TRANSFER_HDR_SIZE -
			(event->data.first_control_packet ?
				MAUSB_CONTROL_SETUP_SIZE : 0);

	if (mausb_data_iterator_read(iterator, payload_data_size,
				     &payload_data_chunks,
				     &num_of_payload_data_chunks) < 0) {
		status = -ENOMEM;
		goto l_cleanup_data_chunks;
	}

	list_splice_tail(&payload_data_chunks, &chunks_list);
	num_of_data_chunks += num_of_payload_data_chunks;

	/* Map all data chunks to data wrapper */
	if (mausb_init_data_wrapper(wrapper, &chunks_list,
				    num_of_data_chunks) < 0) {
		status = -ENOMEM;
		goto l_cleanup_data_chunks;
	}

l_cleanup_data_chunks: /* Cleanup all allocated data chunk in case of error */
	mausb_cleanup_chunks_list(&chunks_list);
	return status;
}

int mausb_send_out_data_msg(struct mausb_device *dev, struct mausb_event *event,
			    struct mausb_urb_ctx *urb_ctx)
{
	int status = 0;
	struct mausb_kvec_data_wrapper data;

	status = mausb_prepare_transfer_packet(&data, event,
					       &urb_ctx->iterator);

	if (status < 0) {
		mausb_pr_err("Failed to prepare transfer packet");
		return status;
	}

	status = mausb_send_data(dev, mausb_transfer_type_to_channel(
				 event->data.transfer_type), &data);

	kfree(data.kvec);

	return status;
}

int mausb_receive_out_data(struct mausb_device *dev, struct mausb_event *event,
			   struct mausb_urb_ctx *urb_ctx)
{
	struct urb *urb = urb_ctx->urb;
	int status = 0;

	mausb_pr_debug("transfer_size=%d, rem_transfer_size=%d, status=%d",
		       event->data.transfer_size, event->data.rem_transfer_size,
		       event->status);

	if (event->data.transfer_eot) {
		mausb_complete_request(urb, urb->transfer_buffer_length -
				       event->data.rem_transfer_size,
				       event->status);
	}

	return status;
}
