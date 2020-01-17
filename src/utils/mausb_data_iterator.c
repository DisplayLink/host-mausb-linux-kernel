// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "utils/mausb_data_iterator.h"

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uio.h>

#include "hpal/hpal.h"
#include "utils/mausb_logs.h"

static int mausb_read_virtual_buffer(struct mausb_data_iter *iterator,
				     uint32_t byte_num,
				     struct list_head *data_chunks_list,
				     uint32_t *data_chunks_num)
{
	uint32_t rem_data	= 0;
	uint32_t bytes_to_read	= 0;
	struct mausb_payload_chunk *data_chunk = NULL;

	(*data_chunks_num) = 0;

	if (!data_chunks_list)
		return -EINVAL;

	INIT_LIST_HEAD(data_chunks_list);
	rem_data      = iterator->length - iterator->offset;
	bytes_to_read = min(byte_num, rem_data);

	if (bytes_to_read == 0)
		return 0;

	data_chunk = kzalloc(sizeof(*data_chunk), GFP_KERNEL);

	if (!data_chunk)
		return -ENOMEM;

	++(*data_chunks_num);

	data_chunk->kvec.iov_base = (uint8_t *) (iterator->buffer) +
		iterator->offset;
	data_chunk->kvec.iov_len = bytes_to_read;
	iterator->offset += bytes_to_read;

	list_add_tail(&data_chunk->list_entry, data_chunks_list);
	return 0;
}

static int mausb_read_scatterlist_buffer(struct mausb_data_iter *iterator,
					 uint32_t byte_num,
					 struct list_head *data_chunks_list,
					 uint32_t *data_chunks_num)
{
	uint32_t current_sg_read_num = 0;
	struct mausb_payload_chunk *data_chunk = NULL;

	(*data_chunks_num) = 0;

	if (!data_chunks_list)
		return -EINVAL;

	INIT_LIST_HEAD(data_chunks_list);
	while (byte_num) {

		if (iterator->sg_iter.consumed == iterator->sg_iter.length) {
			if (!sg_miter_next(&iterator->sg_iter))
				break;
			iterator->sg_iter.consumed = 0;
		}

		data_chunk = kzalloc(sizeof(*data_chunk), GFP_KERNEL);
		if (!data_chunk) {
			sg_miter_stop(&iterator->sg_iter);
			return -ENOMEM;
		}

		current_sg_read_num = min((size_t) byte_num,
			iterator->sg_iter.length - iterator->sg_iter.consumed);

		data_chunk->kvec.iov_base = (uint8_t *) iterator->sg_iter.addr +
					     iterator->sg_iter.consumed;
		data_chunk->kvec.iov_len  = current_sg_read_num;

		++(*data_chunks_num);
		list_add_tail(&data_chunk->list_entry, data_chunks_list);

		byte_num -= current_sg_read_num;
		iterator->sg_iter.consumed += current_sg_read_num;
		data_chunk = NULL;
	}

	return 0;
}

static uint32_t mausb_write_virtual_buffer(struct mausb_data_iter *iterator,
					   void *buffer, uint32_t size)
{

	uint32_t rem_space   = 0;
	uint32_t write_count = 0;

	if (!buffer || !size)
		return write_count;

	rem_space   = iterator->length - iterator->offset;
	write_count = min(size, rem_space);

	if (write_count > 0) {
		void *location = shift_ptr(iterator->buffer, iterator->offset);

		memcpy(location, buffer, write_count);
		iterator->offset += write_count;
	}

	return write_count;
}

static uint32_t mausb_write_scatterlist_buffer(struct mausb_data_iter *iterator,
					       void *buffer, uint32_t size)
{
	uint32_t current_sg_rem_space = 0;
	uint32_t count		      = 0;
	uint32_t total_count	      = 0;
	void *location = NULL;

	if (!buffer || !size)
		return count;

	while (size) {

		if (iterator->sg_iter.consumed >= iterator->sg_iter.length) {
			if (!sg_miter_next(&iterator->sg_iter))
				break;
			iterator->sg_iter.consumed = 0;
		}

		current_sg_rem_space = iterator->sg_iter.length -
		    iterator->sg_iter.consumed;

		count = min(size, current_sg_rem_space);
		total_count += count;

		location = shift_ptr(iterator->sg_iter.addr,
				     iterator->sg_iter.consumed);

		memcpy(location, buffer, count);

		buffer = shift_ptr(buffer, count);
		size -= count;
		iterator->sg_iter.consumed += count;
	}

	return total_count;

}

int mausb_data_iterator_read(struct mausb_data_iter *iterator,
			     uint32_t byte_num,
			     struct list_head *data_chunks_list,
			     uint32_t *data_chunks_num)
{
	if (iterator->buffer)
		return mausb_read_virtual_buffer(iterator, byte_num,
						 data_chunks_list,
						 data_chunks_num);
	else
		return mausb_read_scatterlist_buffer(iterator, byte_num,
						     data_chunks_list,
						     data_chunks_num);

}

uint32_t mausb_data_iterator_write(struct mausb_data_iter *iterator,
				   void *buffer, uint32_t length)
{
	if (iterator->buffer)
		return mausb_write_virtual_buffer(iterator, buffer, length);
	else
		return mausb_write_scatterlist_buffer(iterator, buffer, length);

}

static inline void mausb_seek_virtual_buffer(struct mausb_data_iter *iterator,
					     uint32_t seek_delta)
{
	iterator->offset += min(seek_delta, iterator->length -
					    iterator->offset);
}

static void mausb_seek_scatterlist_buffer(struct mausb_data_iter *iterator,
					  uint32_t seek_delta)
{
	uint32_t rem_bytes_in_current_scatter = 0;

	while (seek_delta) {
		rem_bytes_in_current_scatter =
		    iterator->sg_iter.length - iterator->sg_iter.consumed;
		if (rem_bytes_in_current_scatter <= seek_delta) {
			iterator->sg_iter.consumed +=
			    rem_bytes_in_current_scatter;
			seek_delta -= rem_bytes_in_current_scatter;
			if (!sg_miter_next(&iterator->sg_iter))
				break;
			iterator->sg_iter.consumed = 0;
		} else {
			iterator->sg_iter.consumed += seek_delta;
			break;
		}
	}
}

void mausb_data_iterator_seek(struct mausb_data_iter *iterator,
			      uint32_t seek_delta)
{
	if (iterator->buffer)
		mausb_seek_virtual_buffer(iterator, seek_delta);
	else
		mausb_seek_scatterlist_buffer(iterator, seek_delta);
}

static void mausb_calculate_buffer_length(struct mausb_data_iter *iterator)
{
	/* Calculate buffer length */
	if (iterator->buffer_len > 0) {
		/* Transfer_buffer_length is populated */
		iterator->length = iterator->buffer_len;
	} else if (iterator->sg && iterator->num_sgs != 0) {
		/* Transfer_buffer_length is not populated */
		sg_miter_start(&iterator->sg_iter, iterator->sg,
				iterator->num_sgs, iterator->flags);
		while (sg_miter_next(&iterator->sg_iter))
			iterator->length += iterator->sg_iter.length;
		sg_miter_stop(&iterator->sg_iter);
	} else {
		iterator->length = 0;
	}
}

void mausb_init_data_iterator(struct mausb_data_iter *iterator, void *buffer,
			      uint32_t buffer_len, struct scatterlist *sg,
			      uint32_t num_sgs, bool direction)
{
	iterator->offset = 0;
	iterator->buffer     = buffer;
	iterator->buffer_len = buffer_len;
	iterator->length     = 0;
	iterator->sg	     = sg;
	iterator->num_sgs    = num_sgs;
	iterator->sg_started = 0;

	mausb_calculate_buffer_length(iterator);

	if (!buffer && sg && num_sgs != 0) {
		/* Scatterlist provided */
		iterator->flags = direction ? SG_MITER_TO_SG : SG_MITER_FROM_SG;
		sg_miter_start(&iterator->sg_iter, sg, num_sgs,
			iterator->flags);
		iterator->sg_started = 1;
	}
}

void mausb_uninit_data_iterator(struct mausb_data_iter *iterator)
{
	iterator->offset     = 0;
	iterator->length     = 0;
	iterator->buffer     = NULL;
	iterator->buffer_len = 0;

	if (iterator->sg_started)
		sg_miter_stop(&iterator->sg_iter);

	iterator->sg_started = 0;
}

void mausb_reset_data_iterator(struct mausb_data_iter *iterator)
{
	iterator->offset = 0;
	if (iterator->sg_started) {
		sg_miter_stop(&iterator->sg_iter);
		iterator->sg_started = 0;
	}

	if (!iterator->buffer && iterator->sg && iterator->num_sgs != 0) {
		sg_miter_start(&iterator->sg_iter, iterator->sg,
			iterator->num_sgs, iterator->flags);
		iterator->sg_started = 1;
	}

}

uint32_t mausb_data_iterator_length(struct mausb_data_iter *iterator)
{
	return iterator->length;
}
