// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hpal/isoch_out.h"

#include <linux/slab.h>

#include "hcd/hub.h"
#include "hpal/data_common.h"
#include "hpal/hpal.h"
#include "utils/mausb_data_iterator.h"
#include "utils/mausb_logs.h"

static struct ma_usb_hdr_common *__mausb_create_isoch_out_transfer_packet(
					struct mausb_event *event,
					struct mausb_urb_ctx *urb_ctx,
					uint16_t payload_size, uint32_t seq_n,
					uint32_t start_of_segments,
					uint32_t number_of_segments)
{
	struct ma_usb_hdr_common		 *hdr;
	struct ma_usb_hdr_isochtransfer		 *hdr_isochtransfer;
	struct ma_usb_hdr_isochdatablock_std	 *isoc_header_std;
	struct ma_usb_hdr_isochtransfer_optional *hdr_opt_isochtransfer;
	struct urb *urb = (struct urb *) event->data.urb;
	void *isoc_headers = NULL;
	uint16_t length = 0;
	uint16_t i;
	unsigned long block_length;
	uint32_t number_of_packets = (uint32_t)event->data.isoch_seg_num;
	uint16_t size_of_request;

	size_of_request = MAUSB_ISOCH_TRANSFER_HDR_SIZE +
			  ISOCH_STANDARD_FORMAT_SIZE * number_of_segments;

	hdr = kzalloc(size_of_request, GFP_KERNEL);
	if (unlikely(!hdr)) {
		mausb_pr_alert("Header allocation failed");
		return NULL;
	}

	hdr->version	  = MA_USB_HDR_VERSION_1_0;
	hdr->ssid	  = event->data.mausb_ssid;
	hdr->flags	  = MA_USB_HDR_FLAGS_HOST;
	hdr->dev_addr	  = event->data.mausb_address;
	hdr->handle.epv	  = event->data.ep_handle;
	hdr->data.status  = MA_USB_HDR_STATUS_NO_ERROR;
	hdr->data.eps	  = MAUSB_TRANSFER_RESERVED;
	hdr->data.t_flags = usb_endpoint_type(&urb->ep->desc) << 3;

	isoc_headers = shift_ptr(hdr, MAUSB_ISOCH_TRANSFER_HDR_SIZE);

	for (i = (uint16_t)start_of_segments;
	     i < number_of_segments + start_of_segments; ++i) {
		block_length = i < number_of_packets - 1 ?
			urb->iso_frame_desc[i + 1].offset -
			urb->iso_frame_desc[i].offset :
			mausb_data_iterator_length(&urb_ctx->iterator) -
			urb->iso_frame_desc[i].offset;

		urb->iso_frame_desc[i].status = MA_USB_HDR_STATUS_UNSUCCESSFUL;
		isoc_header_std = (struct ma_usb_hdr_isochdatablock_std *)
				   shift_ptr(isoc_headers,
					(uint64_t)ISOCH_STANDARD_FORMAT_SIZE *
					(i - start_of_segments));
		isoc_header_std->block_length	 = (uint16_t)block_length;
		isoc_header_std->segment_number  = i;
		isoc_header_std->s_flags	 = 0;
		isoc_header_std->segment_length  = (uint16_t)block_length;
		isoc_header_std->fragment_offset = 0;
	}

	length = (ISOCH_STANDARD_FORMAT_SIZE * number_of_segments)
		 + MAUSB_ISOCH_TRANSFER_HDR_SIZE;

	hdr->flags		   |= MA_USB_HDR_FLAGS_TIMESTAMP;
	hdr->type		    = MA_USB_HDR_TYPE_DATA_REQ(ISOCHTRANSFER);
	hdr->data.headers	    = (uint16_t)number_of_segments;
	hdr->data.i_flags	    = MA_USB_DATA_IFLAGS_HDR_FMT_STD |
				      MA_USB_DATA_IFLAGS_ASAP;
	hdr_isochtransfer	    = mausb_get_isochtransfer_hdr(hdr);
	hdr_opt_isochtransfer	    = mausb_hdr_isochtransfer_optional_hdr(hdr);
	hdr_isochtransfer->req_id   = event->data.req_id;
	hdr_isochtransfer->seq_n    = seq_n;
	hdr_isochtransfer->segments = number_of_packets;

	hdr_isochtransfer->presentation_time = MA_USB_TRANSFER_RESERVED;

	hdr_opt_isochtransfer->timestamp = MA_USB_TRANSFER_RESERVED;
	hdr_opt_isochtransfer->mtd	 = MA_USB_TRANSFER_RESERVED;

	hdr->length = length + payload_size;

	return hdr;
}

static int mausb_add_data_chunk(void *buffer, uint32_t buffer_size,
				struct list_head *chunks_list)
{
	struct mausb_payload_chunk *data_chunk = NULL;

	data_chunk = kzalloc(sizeof(*data_chunk), GFP_KERNEL);
	if (unlikely(!data_chunk)) {
		mausb_pr_alert("Data chunk allocation failed");
		return -ENOMEM;
	}

	/* Initialize data chunk for MAUSB header and add it to chunks list */
	INIT_LIST_HEAD(&data_chunk->list_entry);
	data_chunk->kvec.iov_base = buffer;
	data_chunk->kvec.iov_len  = buffer_size;
	list_add_tail(&data_chunk->list_entry, chunks_list);
	return 0;
}

static int mausb_init_header_data_chunk(struct ma_usb_hdr_common *common_hdr,
					struct list_head *chunks_list,
					uint32_t *num_of_data_chunks,
					uint32_t num_of_packets)
{
	uint32_t header_size = MAUSB_ISOCH_TRANSFER_HDR_SIZE +
			       ISOCH_STANDARD_FORMAT_SIZE * num_of_packets;
	int status;

	status = mausb_add_data_chunk(common_hdr, header_size, chunks_list);

	/* Success */
	if (!status)
		++(*num_of_data_chunks);

	return status;
}

static int mausb_init_data_wrapper(struct mausb_kvec_data_wrapper *data,
				   struct list_head *chunks_list,
				   uint32_t num_of_data_chunks)
{
	struct mausb_payload_chunk *data_chunk = NULL, *tmp = NULL;
	uint32_t current_kvec = 0;

	data->length = 0;
	data->kvec = kcalloc(num_of_data_chunks, sizeof(struct kvec),
			     GFP_KERNEL);
	if (!data->kvec)
		return -ENOMEM;

	list_for_each_entry_safe(data_chunk, tmp, chunks_list, list_entry) {
		data->kvec[current_kvec].iov_base =
			data_chunk->kvec.iov_base;
		data->kvec[current_kvec].iov_len =
		    data_chunk->kvec.iov_len;
		++data->kvec_num;
		data->length += data_chunk->kvec.iov_len;
		++current_kvec;
	}
	return 0;
}

static void mausb_cleanup_chunks_list(struct list_head *chunks_list)
{
	struct mausb_payload_chunk *data_chunk = NULL, *tmp = NULL;

	list_for_each_entry_safe(data_chunk, tmp, chunks_list, list_entry) {
		list_del(&data_chunk->list_entry);
		kfree(data_chunk);
	}
}

static int mausb_prepare_isoch_out_transfer_packet(
			struct ma_usb_hdr_common *hdr,
			struct mausb_event *event,
			struct mausb_urb_ctx *urb_ctx,
			struct mausb_kvec_data_wrapper *result_data_wrapper)
{
	uint32_t num_of_data_chunks	    = 0;
	uint32_t num_of_payload_data_chunks = 0;
	uint32_t payload_data_size	    = 0;
	uint32_t size_of_header		    = MAUSB_ISOCH_TRANSFER_HDR_SIZE +
					      ISOCH_STANDARD_FORMAT_SIZE *
					      event->data.isoch_seg_num;
	struct list_head chunks_list;
	struct list_head payload_data_chunks;
	int status = 0;

	INIT_LIST_HEAD(&chunks_list);

	/* Initialize data chunk for MAUSB header and add it to chunks list */
	if (mausb_init_header_data_chunk(hdr, &chunks_list, &num_of_data_chunks,
					 event->data.isoch_seg_num) < 0){
		status = -ENOMEM;
			goto l_cleanup_data_chunks;
	}

	/* Get data chunks for data payload to send */
	INIT_LIST_HEAD(&payload_data_chunks);
	payload_data_size = hdr->length - size_of_header;

	if (mausb_data_iterator_read(&urb_ctx->iterator, payload_data_size,
				     &payload_data_chunks,
				     &num_of_payload_data_chunks) < 0){
		mausb_pr_err("Data iterator read failed");
		status = -ENOMEM;
		goto l_cleanup_data_chunks;
	}

	list_splice_tail(&payload_data_chunks, &chunks_list);
	num_of_data_chunks += num_of_payload_data_chunks;

	/* Map all data chunks to data wrapper */
	if (mausb_init_data_wrapper(result_data_wrapper, &chunks_list,
				    num_of_data_chunks) < 0) {
		mausb_pr_err("Data wrapper init failed");
		status = -ENOMEM;
		goto l_cleanup_data_chunks;
	}

l_cleanup_data_chunks:
	mausb_cleanup_chunks_list(&chunks_list);
	return status;
}

static int mausb_create_and_send_isoch_transfer_req(struct mausb_device *dev,
			struct mausb_event *event,
			struct mausb_urb_ctx *urb_ctx, uint32_t *seq_n,
			uint32_t payload_size, uint32_t start_of_segments,
			uint32_t number_of_segments)
{
	struct ma_usb_hdr_common *hdr;
	struct mausb_kvec_data_wrapper data_to_send;
	int status;

	hdr = __mausb_create_isoch_out_transfer_packet(event, urb_ctx,
						       payload_size, *seq_n,
						       start_of_segments,
						       number_of_segments);
	if (!hdr) {
		mausb_pr_alert("Isoch transfer packet alloc failed");
		return -ENOMEM;
	}
	*seq_n = (*seq_n + 1) % (MA_USB_TRANSFER_SEQN_MAX + 1);

	status = mausb_prepare_isoch_out_transfer_packet(hdr, event, urb_ctx,
							 &data_to_send);
	if (status < 0) {
		mausb_pr_alert("Failed to prepare transfer packet");
		kfree(hdr);
		return status;
	}

	status = mausb_send_data(dev, mausb_transfer_type_to_channel(
						event->data.transfer_type),
				&data_to_send);

	kfree(hdr);
	kfree(data_to_send.kvec);

	return status;
}

static inline int __mausb_send_isoch_out_packet(struct mausb_device *dev,
			struct mausb_event *event,
			struct mausb_urb_ctx *urb_ctx, uint32_t *seq_n,
			uint32_t *starting_segments, uint32_t *rem_transfer_buf,
			uint32_t *payload_size, int index)
{
	int status = mausb_create_and_send_isoch_transfer_req(dev, event,
					urb_ctx, seq_n, *payload_size,
					*starting_segments,
					index - *starting_segments);

	if (status < 0) {
		mausb_pr_err("ISOCH transfer request create and send failed");
		return status;
	}
	*starting_segments = index;
	*rem_transfer_buf  = MAX_ISOCH_ASAP_PACKET_SIZE;
	*payload_size	   = 0;

	return 0;
}

int mausb_send_isoch_out_msg(struct mausb_device *ma_dev,
			     struct mausb_event *mausb_event,
			     struct mausb_urb_ctx *urb_ctx)
{
	uint32_t   starting_segments = 0;
	uint32_t   rem_transfer_buf  = MAX_ISOCH_ASAP_PACKET_SIZE;
	struct urb *urb = (struct urb *)mausb_event->data.urb;
	uint32_t number_of_packets = urb->number_of_packets;
	uint32_t payload_size	   = 0;
	uint32_t chunk_size	   = 0;
	uint32_t seq_n		   = 0;
	int status = 0;
	int i	   = 0;

	for (i = 0; i < number_of_packets; ++i) {
		if (i < number_of_packets - 1)
			chunk_size = urb->iso_frame_desc[i + 1].offset -
					urb->iso_frame_desc[i].offset;
		else
			chunk_size =
				mausb_data_iterator_length(&urb_ctx->iterator) -
						urb->iso_frame_desc[i].offset;

		if (chunk_size + ISOCH_STANDARD_FORMAT_SIZE >
							rem_transfer_buf) {
			if (payload_size == 0) {
				mausb_pr_warn("Fragmentation");
			} else {
				status = __mausb_send_isoch_out_packet(ma_dev,
						mausb_event, urb_ctx, &seq_n,
						&starting_segments,
						&rem_transfer_buf,
						&payload_size, i);
				if (status < 0)
					return status;
				i--;
				continue;
			}
		} else {
			rem_transfer_buf -= chunk_size +
					     ISOCH_STANDARD_FORMAT_SIZE;
			payload_size += chunk_size;
		}

		if (i == number_of_packets - 1 || rem_transfer_buf == 0) {

			status = __mausb_send_isoch_out_packet(ma_dev,
					mausb_event, urb_ctx, &seq_n,
					&starting_segments,
					&rem_transfer_buf, &payload_size,
					i + 1);
			if (status < 0)
				return status;
		}
	}
	return 0;
}

int mausb_receive_isoch_out(struct mausb_device *dev, struct mausb_event *event,
			    struct mausb_urb_ctx *urb_ctx)
{
	struct urb *urb = (struct urb *)event->data.urb;
	int status = 0;
	uint16_t i;

	mausb_pr_debug("transfer_size=%d, rem_transfer_size=%d, status=%d",
		       event->data.transfer_size, event->data.rem_transfer_size,
		       event->status);

	for (i = 0; i < urb->number_of_packets; ++i)
		urb->iso_frame_desc[i].status = event->status;

	mausb_complete_request(urb, event->data.payload_size, event->status);

	return status;
}
