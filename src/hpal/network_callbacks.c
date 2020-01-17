// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hpal/network_callbacks.h"

#include <linux/uio.h>
#include <linux/slab.h>

#include "hpal/hpal.h"
#include "hpal/mausb_events.h"
#include "utils/mausb_logs.h"

static void mausb_init_ip_ctx_helper(struct mausb_device *dev,
				     struct mausb_ip_ctx **ip_ctx,
				     uint16_t port,
				     enum mausb_channel channel)
{
	int status;

	status = mausb_init_ip_ctx(ip_ctx, dev->net_ns,
				   dev->dev_addr.Ip.Address.ip4, port, dev,
				   mausb_ip_callback, channel);
	if (status < 0) {
		mausb_pr_err("Init ip context failed with error=%d", status);
		queue_work(dev->workq, &dev->socket_disconnect_work);
		return;
	}

	dev->channel_map[channel] = *ip_ctx;
	mausb_ip_connect_async(*ip_ctx);
}

static void mausb_connect_callback(struct mausb_device *dev, enum mausb_channel
				channel, int status)
{
	mausb_pr_info("Connect callback for channel=%d with status=%d",
		      channel, status);

	if (status < 0) {
		queue_work(dev->workq, &dev->socket_disconnect_work);
		return;
	}

	if (channel == MAUSB_MGMT_CHANNEL) {
		if (dev->dev_addr.Ip.Port.control == 0) {
			dev->channel_map[MAUSB_CTRL_CHANNEL] =
				dev->mgmt_channel;
			channel = MAUSB_CTRL_CHANNEL;
		} else {
			mausb_init_ip_ctx_helper(dev, &dev->ctrl_channel,
					dev->dev_addr.Ip.Port.control,
					MAUSB_CTRL_CHANNEL);
			return;
		}
	}

	if (channel == MAUSB_CTRL_CHANNEL) {
		if (dev->dev_addr.Ip.Port.bulk == 0) {
			dev->channel_map[MAUSB_BULK_CHANNEL] =
				dev->channel_map[MAUSB_CTRL_CHANNEL];
			channel = MAUSB_BULK_CHANNEL;
		} else {
			mausb_init_ip_ctx_helper(dev, &dev->bulk_channel,
					dev->dev_addr.Ip.Port.bulk,
					MAUSB_BULK_CHANNEL);
			return;
		}
	}

	if (channel == MAUSB_BULK_CHANNEL) {
		if (dev->dev_addr.Ip.Port.isochronous == 0) {
			/* if there is no isoch port use tcp for it */
			dev->channel_map[MAUSB_ISOCH_CHANNEL] =
				dev->channel_map[MAUSB_BULK_CHANNEL];
			channel = MAUSB_ISOCH_CHANNEL;
		} else {
			mausb_init_ip_ctx_helper(dev, &dev->isoch_channel,
					dev->dev_addr.Ip.Port.isochronous,
					MAUSB_ISOCH_CHANNEL);
			return;
		}
	}

	if (channel == MAUSB_ISOCH_CHANNEL) {
		dev->channel_map[MAUSB_INTR_CHANNEL] =
				dev->channel_map[MAUSB_CTRL_CHANNEL];
		mausb_on_madev_connected(dev);
	}
}

static void mausb_handle_connect_event(struct mausb_device *dev,
				       enum mausb_channel channel, int status,
				       void *data)
{
	mausb_connect_callback(dev, channel, status);
}

static void mausb_handle_receive_event(struct mausb_device *dev,
				       enum mausb_channel channel, int status,
				       void *data)
{
	struct mausb_event event;

	event.madev_addr = dev->madev_addr;

	if (status <= 0) {
		mausb_pr_err("Receive event error status=%d", status);
		queue_work(dev->workq, &dev->socket_disconnect_work);
		queue_work(dev->workq, &dev->hcd_disconnect_work);
		return;
	}

	mausb_reset_connection_timer(dev);

	status = mausb_msg_received_event(&event,
					  (struct ma_usb_hdr_common *)data,
					  channel);

	if (status == 0)
		status = mausb_enqueue_event_to_user(dev, &event);

	if (status < 0) {
		mausb_pr_err("Failed to enqueue, status=%d", status);
		queue_work(dev->workq, &dev->socket_disconnect_work);
		queue_work(dev->workq, &dev->hcd_disconnect_work);
		return;
	}
}

void mausb_ip_callback(void *ctx, enum mausb_channel channel,
		       enum mausb_link_action action, int status, void *data)
{
	struct mausb_device *dev = (struct mausb_device *)ctx;

	switch (action) {
	case MAUSB_LINK_CONNECT:
		mausb_handle_connect_event(dev, channel, status, data);
		break;
	case MAUSB_LINK_SEND:
		/*
		 * Currently there is nothing to do, as send operation is
		 * synchronous
		 */
		break;
	case MAUSB_LINK_RECV:
		mausb_handle_receive_event(dev, channel, status, data);
		break;
	case MAUSB_LINK_DISCONNECT:
		/*
		 * Currently there is nothing to do, as disconnect operation is
		 * synchronous
		 */
		break;
	default:
		mausb_pr_warn("Unknown network action");
	}
}
