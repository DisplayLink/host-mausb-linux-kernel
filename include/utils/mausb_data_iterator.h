/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_UTILS_MAUSB_DATA_ITERATOR_H__
#define __MAUSB_UTILS_MAUSB_DATA_ITERATOR_H__

#include <linux/list.h>
#include <linux/scatterlist.h>
#include <linux/uio.h>

struct mausb_data_iter {
	u32 length;

	void *buffer;
	u32  buffer_len;
	u32  offset;

	struct scatterlist	*sg;
	struct sg_mapping_iter	sg_iter;
	bool		sg_started;
	unsigned int	num_sgs;
	unsigned int	flags;
};

struct mausb_payload_chunk {
	struct list_head list_entry;
	struct kvec	 kvec;
};

int mausb_data_iterator_read(struct mausb_data_iter *iterator,
			     u32 byte_num,
			     struct list_head *data_chunks_list,
			     u32 *data_chunks_num);

u32 mausb_data_iterator_length(struct mausb_data_iter *iterator);
u32 mausb_data_iterator_write(struct mausb_data_iter *iterator, void *buffer,
			      u32 length);

void mausb_init_data_iterator(struct mausb_data_iter *iterator,
			      void *buffer, u32 buffer_len,
			      struct scatterlist *sg, unsigned int num_sgs,
			      bool direction);
void mausb_reset_data_iterator(struct mausb_data_iter *iterator);
void mausb_uninit_data_iterator(struct mausb_data_iter *iterator);
void mausb_data_iterator_seek(struct mausb_data_iter *iterator, u32 seek_delta);

#endif /* __MAUSB_UTILS_MAUSB_DATA_ITERATOR_H__ */
