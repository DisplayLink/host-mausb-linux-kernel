// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hpal/isoch_in.h"

#include <linux/slab.h>
#include <linux/uio.h>

#include "hcd/hub.h"
#include "hpal/data_common.h"
#include "hpal/hpal.h"
#include "hpal/mausb_events.h"
#include "utils/mausb_data_iterator.h"
#include "utils/mausb_logs.h"

static inline u32
__mausb_isoch_prepare_read_size_block(struct ma_usb_hdr_isochreadsizeblock_std *
				      isoch_readsize_block, struct urb *urb)
{
	u32 i;
	u32 number_of_packets = (u32)urb->number_of_packets;

	if (number_of_packets == 0)
		return 0;

	isoch_readsize_block->service_intervals  = number_of_packets;
	isoch_readsize_block->max_segment_length =
					(u32)urb->iso_frame_desc[0].length;

	for (i = 0; i < number_of_packets; ++i) {
		urb->iso_frame_desc[i].status = 0;
		urb->iso_frame_desc[i].actual_length = 0;
	}

	return sizeof(struct ma_usb_hdr_isochreadsizeblock_std);
}

int mausb_send_isoch_in_msg(struct mausb_device *dev, struct mausb_event *event)
{
	u32 read_size_block_length = 0;
	struct mausb_kvec_data_wrapper data_to_send;
	struct kvec kvec[MAUSB_ISOCH_IN_KVEC_NUM];
	struct ma_usb_hdr_isochtransfer_optional opt_isoch_hdr;
	struct ma_usb_hdr_isochreadsizeblock_std isoch_readsize_block;
	struct ma_usb_hdr_common *hdr =
				(struct ma_usb_hdr_common *)event->data.hdr;
	struct urb *urb = (struct urb *)event->data.urb;
	enum mausb_channel channel;

	data_to_send.kvec_num	= 0;
	data_to_send.length	= 0;

	/* Prepare transfer header kvec */
	kvec[0].iov_base     = event->data.hdr;
	kvec[0].iov_len	     = MAUSB_TRANSFER_HDR_SIZE;
	data_to_send.length += (u32)kvec[0].iov_len;
	data_to_send.kvec_num++;

	/* Prepare optional header kvec */
	opt_isoch_hdr.timestamp = MA_USB_TRANSFER_RESERVED;
	opt_isoch_hdr.mtd	= MA_USB_TRANSFER_RESERVED;

	kvec[1].iov_base     = &opt_isoch_hdr;
	kvec[1].iov_len	     = sizeof(struct ma_usb_hdr_isochtransfer_optional);
	data_to_send.length += (u32)kvec[1].iov_len;
	data_to_send.kvec_num++;

	/* Prepare read size blocks */
	read_size_block_length =
		__mausb_isoch_prepare_read_size_block(&isoch_readsize_block,
						      urb);
	if (read_size_block_length > 0) {
		kvec[2].iov_base     = &isoch_readsize_block;
		kvec[2].iov_len	     = read_size_block_length;
		data_to_send.length += (u32)kvec[2].iov_len;
		data_to_send.kvec_num++;
	}

	hdr->length = (u16)data_to_send.length;
	data_to_send.kvec = kvec;

	channel = mausb_transfer_type_to_channel(event->data.transfer_type);
	return mausb_send_data(dev, channel, &data_to_send);
}

static void __mausb_process_in_isoch_short_resp(struct mausb_event *event,
						struct ma_usb_hdr_common *hdr,
						struct mausb_urb_ctx *urb_ctx)
{
	u8 opt_hdr_shift = (hdr->flags & MA_USB_HDR_FLAGS_TIMESTAMP) ?
			   sizeof(struct ma_usb_hdr_isochtransfer_optional) : 0;
	struct ma_usb_hdr_isochdatablock_short *data_block_hdr =
			(struct ma_usb_hdr_isochdatablock_short *)
			shift_ptr(mausb_hdr_isochtransfer_optional_hdr(hdr),
				  opt_hdr_shift);
	u8 *isoch_data = shift_ptr(data_block_hdr, hdr->data.headers *
				   sizeof(*data_block_hdr));
	u8 *end_of_packet = shift_ptr(hdr, hdr->length);
	struct urb *urb = urb_ctx->urb;
	int i;

	if (isoch_data >= end_of_packet) {
		mausb_pr_err("Bad header data. Data start pointer after end of packet: ep_handle=%#x",
			     event->data.ep_handle);
		return;
	}

	for (i = 0; i < hdr->data.headers; ++i) {
		u16 seg_num  = data_block_hdr[i].segment_number;
		u16 seg_size = data_block_hdr[i].block_length;

		if (seg_num >= urb->number_of_packets) {
			mausb_pr_err("Too many segments: ep_handle=%#x, seg_num=%d, urb.number_of_packets=%d",
				     event->data.ep_handle, seg_num,
				     urb->number_of_packets);
			break;
		}

		if (seg_size > urb->iso_frame_desc[seg_num].length) {
			mausb_pr_err("Block to long for segment: ep_handle=%#x",
				     event->data.ep_handle);
			break;
		}

		if (shift_ptr(isoch_data, seg_size) > end_of_packet) {
			mausb_pr_err("End of segment after enf of packet: ep_handle=%#x",
				     event->data.ep_handle);
			break;
		}

		mausb_reset_data_iterator(&urb_ctx->iterator);
		mausb_data_iterator_seek(&urb_ctx->iterator,
					 urb->iso_frame_desc[seg_num].offset);
		mausb_data_iterator_write(&urb_ctx->iterator, isoch_data,
					  seg_size);

		isoch_data = shift_ptr(isoch_data, seg_size);

		urb->iso_frame_desc[seg_num].actual_length = seg_size;
		urb->iso_frame_desc[seg_num].status = 0;
	}
}

static void __mausb_process_in_isoch_std_resp(struct mausb_event *event,
					      struct ma_usb_hdr_common *hdr,
					      struct mausb_urb_ctx *urb_ctx)
{
	u8 opt_hdr_shift = (hdr->flags & MA_USB_HDR_FLAGS_TIMESTAMP) ?
			   sizeof(struct ma_usb_hdr_isochtransfer_optional) : 0;
	struct ma_usb_hdr_isochdatablock_std *data_block_hdr =
		(struct ma_usb_hdr_isochdatablock_std *)
		shift_ptr(mausb_hdr_isochtransfer_optional_hdr(hdr),
			  opt_hdr_shift);
	u8 *isoch_data =
		shift_ptr(data_block_hdr, hdr->data.headers *
			  sizeof(struct ma_usb_hdr_isochdatablock_std));
	u8 *end_of_packet = shift_ptr(hdr, hdr->length);
	struct urb *urb = (struct urb *)event->data.urb;
	int i;

	if (isoch_data >= end_of_packet) {
		mausb_pr_err("Bad header data. Data start pointer after end of packet: ep_handle=%#x",
			     event->data.ep_handle);
		return;
	}

	for (i = 0; i < hdr->data.headers; ++i) {
		u16 seg_num   = data_block_hdr[i].segment_number;
		u16 seg_len   = data_block_hdr[i].segment_length;
		u16 block_len = data_block_hdr[i].block_length;

		if (seg_num >= urb->number_of_packets) {
			mausb_pr_err("Too many segments: ep_handle=%#x, seg_num=%d, number_of_packets=%d",
				     event->data.ep_handle, seg_num,
				     urb->number_of_packets);
			break;
		}

		if (block_len > urb->iso_frame_desc[seg_num].length -
			     urb->iso_frame_desc[seg_num].actual_length) {
			mausb_pr_err("Block too long for segment: ep_handle=%#x",
				     event->data.ep_handle);
			break;
		}

		if (shift_ptr(isoch_data, block_len) >
				       end_of_packet) {
			mausb_pr_err("End of fragment after end of packet: ep_handle=%#x",
				     event->data.ep_handle);
			break;
		}

		mausb_reset_data_iterator(&urb_ctx->iterator);
		mausb_data_iterator_seek(&urb_ctx->iterator,
					 urb->iso_frame_desc[seg_num].offset +
					 data_block_hdr[i].fragment_offset);
		mausb_data_iterator_write(&urb_ctx->iterator,
					  isoch_data, block_len);
		isoch_data = shift_ptr(isoch_data, block_len);

		urb->iso_frame_desc[seg_num].actual_length += block_len;

		if (urb->iso_frame_desc[seg_num].actual_length == seg_len)
			urb->iso_frame_desc[seg_num].status = 0;
	}
}

int mausb_receive_isoch_in_data(struct mausb_device *dev,
				struct mausb_event *event,
				struct mausb_urb_ctx *urb_ctx)
{
	struct ma_usb_hdr_common *common_hdr =
			(struct ma_usb_hdr_common *)event->data.recv_buf;
	struct ma_usb_hdr_transfer *transfer_hdr =
					mausb_get_data_transfer_hdr(common_hdr);

	if (!(common_hdr->data.i_flags & MA_USB_DATA_IFLAGS_FMT_MASK)) {
		/* Short ISO headers response */
		__mausb_process_in_isoch_short_resp(event, common_hdr, urb_ctx);
	} else if ((common_hdr->data.i_flags & MA_USB_DATA_IFLAGS_FMT_MASK) &
		MA_USB_DATA_IFLAGS_HDR_FMT_STD) {
		/* Standard ISO headers response */
		__mausb_process_in_isoch_std_resp(event, common_hdr, urb_ctx);
	} else if ((common_hdr->data.i_flags & MA_USB_DATA_IFLAGS_FMT_MASK) &
		MA_USB_DATA_IFLAGS_HDR_FMT_LONG) {
		/* Long ISO headers response */
		mausb_pr_warn("Long isoc headers in response: ep_handle=%#x, req_id=%#x",
			      event->data.ep_handle, transfer_hdr->req_id);
	} else {
		/* Error */
		mausb_pr_err("Isoc header error in response: ep_handle=%#x, req_id=%#x",
			     event->data.ep_handle, transfer_hdr->req_id);
	}

	return 0;
}
