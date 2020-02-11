// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hpal/hpal.h"

#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uio.h>
#include <linux/version.h>

#include "common/mausb_event.h"
#include "hcd/hub.h"
#include "hcd/vhcd.h"
#include "hpal/mausb_events.h"
#include "hpal/data_common.h"
#include "hpal/network_callbacks.h"
#include "link/mausb_ip_link.h"
#include "utils/mausb_logs.h"
#include "utils/mausb_mmap.h"
#include "utils/mausb_ring_buffer.h"
#include "utils/mausb_data_iterator.h"
#include "utils/mausb_version.h"

#define MAUSB_DELETE_MADEV_TIMEOUT_MS 3000

struct mss mss;

static int mausb_start_connection_timer(struct mausb_device *dev);
static int mausb_power_state_cb(struct notifier_block *nb, unsigned long action,
				void *data);
static void mausb_signal_empty_mss(void);
static void mausb_remove_madev_from_list(u8 madev_addr);
static void mausb_execute_urb_dequeue(struct work_struct *dequeue_work);
static int mausb_start_heartbeat_timer(void);

static inline struct mausb_urb_ctx *__mausb_find_urb_in_tree(struct urb *urb)
{
	struct rb_node *node = mhcd->mausb_urbs.rb_node;

	while (node) {
		struct mausb_urb_ctx *urb_ctx =
		    rb_entry(node, struct mausb_urb_ctx, rb_node);

		if (urb < urb_ctx->urb)
			node = urb_ctx->rb_node.rb_left;
		else if (urb > urb_ctx->urb)
			node = urb_ctx->rb_node.rb_right;
		else
			return urb_ctx;
	}
	return NULL;
}

struct mausb_urb_ctx *mausb_find_urb_in_tree(struct urb *urb)
{
	unsigned long flags = 0;
	struct mausb_urb_ctx *urb_ctx;

	spin_lock_irqsave(&mhcd->lock, flags);
	urb_ctx =  __mausb_find_urb_in_tree(urb);
	spin_unlock_irqrestore(&mhcd->lock, flags);

	return urb_ctx;
}

static int mausb_insert_urb_ctx_in_tree(struct mausb_urb_ctx *urb_ctx)
{
	struct rb_node **new_node = &mhcd->mausb_urbs.rb_node;
	struct rb_node *parent = NULL;
	struct mausb_urb_ctx *current_urb = NULL;

	while (*new_node) {
		parent = *new_node;
		current_urb = rb_entry(*new_node, struct mausb_urb_ctx,
				       rb_node);

		if (urb_ctx->urb < current_urb->urb)
			new_node = &((*new_node)->rb_left);
		else if (urb_ctx->urb > current_urb->urb)
			new_node = &((*new_node)->rb_right);
		else
			return -EEXIST;
	}
	rb_link_node(&urb_ctx->rb_node, parent, new_node);
	rb_insert_color(&urb_ctx->rb_node, &mhcd->mausb_urbs);
	return 0;
}

static void mausb_delete_urb_ctx_from_tree(struct mausb_urb_ctx *urb_ctx)
{
	rb_erase(&urb_ctx->rb_node, &mhcd->mausb_urbs);
}

static struct mausb_urb_ctx *mausb_create_urb_ctx(struct urb *urb, int *status)
{
	struct mausb_urb_ctx *urb_ctx = NULL;

	if (!urb) {
		mausb_pr_err("Urb is NULL");
		*status = -EINVAL;
		return NULL;
	}

	urb_ctx = kzalloc(sizeof(*urb_ctx), GFP_ATOMIC);
	if (!urb_ctx) {
		*status = -ENOMEM;
		return NULL;
	}

	urb_ctx->urb = urb;
	INIT_WORK(&urb_ctx->work, mausb_execute_urb_dequeue);

	return urb_ctx;
}

int mausb_insert_urb_in_tree(struct urb *urb, bool link_urb_to_ep)
{
	unsigned long flags;
	int status = 0;

	struct mausb_urb_ctx *urb_ctx = mausb_create_urb_ctx(urb, &status);

	if (!urb_ctx)
		return status;

	spin_lock_irqsave(&mhcd->lock, flags);

	if (link_urb_to_ep) {
		status = usb_hcd_link_urb_to_ep(urb->hcpriv, urb);
		if (status) {
			spin_unlock_irqrestore(&mhcd->lock, flags);
			mausb_pr_err("Error %d while linking urb to hcd_endpoint",
				     status);
			kfree(urb_ctx);
			return status;
		}
	}

	if (mausb_insert_urb_ctx_in_tree(urb_ctx)) {
		kfree(urb_ctx);
		if (link_urb_to_ep)
			usb_hcd_unlink_urb_from_ep(urb->hcpriv, urb);
		spin_unlock_irqrestore(&mhcd->lock, flags);
		mausb_pr_err("Urb_ctx insertion failed");
		return -EEXIST;
	}

	mausb_init_data_iterator(&urb_ctx->iterator, urb->transfer_buffer,
				 urb->transfer_buffer_length, urb->sg,
				 (unsigned int)urb->num_sgs,
				 usb_urb_dir_in(urb));

	spin_unlock_irqrestore(&mhcd->lock, flags);

	return 0;
}

static bool mausb_return_urb_ctx_to_tree(struct mausb_urb_ctx *urb_ctx,
					 bool link_urb_to_ep)
{
	unsigned long flags;
	int status;

	if (!urb_ctx)
		return false;

	spin_lock_irqsave(&mhcd->lock, flags);
	if (link_urb_to_ep) {
		status = usb_hcd_link_urb_to_ep(urb_ctx->urb->hcpriv,
						urb_ctx->urb);
		if (status) {
			spin_unlock_irqrestore(&mhcd->lock, flags);
			mausb_pr_err("Error %d while linking urb to hcd_endpoint",
				     status);
			return false;
		}
	}

	if (mausb_insert_urb_ctx_in_tree(urb_ctx)) {
		if (link_urb_to_ep)
			usb_hcd_unlink_urb_from_ep(urb_ctx->urb->hcpriv,
						   urb_ctx->urb);
		spin_unlock_irqrestore(&mhcd->lock, flags);
		mausb_pr_err("Urb_ctx insertion failed");
		return false;
	}

	spin_unlock_irqrestore(&mhcd->lock, flags);

	return true;
}

static void mausb_complete_urbs_from_tree(void)
{
	struct mausb_urb_ctx *urb_ctx = NULL;
	struct urb	     *current_urb = NULL;
	struct rb_node	     *current_node = NULL;
	unsigned long flags;
	int status = 0;
	int ret;

	mausb_pr_debug("Completing all urbs from tree");

	spin_lock_irqsave(&mhcd->lock, flags);

	while ((current_node = rb_first(&mhcd->mausb_urbs))) {
		urb_ctx = rb_entry(current_node, struct mausb_urb_ctx, rb_node);

		current_urb = urb_ctx->urb;
		mausb_delete_urb_ctx_from_tree(urb_ctx);
		mausb_uninit_data_iterator(&urb_ctx->iterator);
		kfree(urb_ctx);

		ret = usb_hcd_check_unlink_urb(current_urb->hcpriv,
					       current_urb, status);
		if (ret == -EIDRM)
			mausb_pr_warn("Urb=%p is already unlinked",
				      current_urb);
		else
			usb_hcd_unlink_urb_from_ep(current_urb->hcpriv,
						   current_urb);

		spin_unlock_irqrestore(&mhcd->lock, flags);

		/* Prepare urb for completion */
		mausb_pr_debug("Completing urb=%p", current_urb);

		current_urb->status	   = -EPROTO;
		current_urb->actual_length = 0;
		atomic_dec(&current_urb->use_count);
		usb_hcd_giveback_urb(current_urb->hcpriv, current_urb,
				     current_urb->status);

		spin_lock_irqsave(&mhcd->lock, flags);
	}

	spin_unlock_irqrestore(&mhcd->lock, flags);

	mausb_pr_debug("Completed all urbs from tree");
}

/*After this function call only valid thing to do with urb is to give it back*/
struct mausb_urb_ctx *mausb_unlink_and_delete_urb_from_tree(struct urb *urb,
							    int status)
{
	struct mausb_urb_ctx *urb_ctx = NULL;
	unsigned long flags;
	int ret;

	if (!urb) {
		mausb_pr_warn("Urb is NULL");
		return NULL;
	}

	spin_lock_irqsave(&mhcd->lock, flags);

	urb_ctx = __mausb_find_urb_in_tree(urb);

	if (!urb_ctx) {
		mausb_pr_warn("Urb=%p not in tree", urb);
		spin_unlock_irqrestore(&mhcd->lock, flags);
		return NULL;
	}

	ret = usb_hcd_check_unlink_urb(urb->hcpriv, urb, status);

	if (ret == -EIDRM)
		mausb_pr_warn("Urb=%p is already unlinked", urb);
	else
		usb_hcd_unlink_urb_from_ep(urb->hcpriv, urb);

	mausb_delete_urb_ctx_from_tree(urb_ctx);

	spin_unlock_irqrestore(&mhcd->lock, flags);

	mausb_pr_debug("Urb=%p is removed from tree", urb);

	return urb_ctx;
}

void mausb_release_event_resources(struct mausb_event *event)
{
	struct ma_usb_hdr_common *receive_buffer = (struct ma_usb_hdr_common *)
						    event->data.recv_buf;

	kfree(receive_buffer);
}

static void mausb_iterator_reset(struct mausb_device *dev,
				 struct mausb_event *event)
{
	struct urb	     *urb = (struct urb *)event->data.urb;
	struct mausb_urb_ctx *urb_ctx;

	urb_ctx = mausb_find_urb_in_tree(urb);

	if (urb_ctx)
		mausb_reset_data_iterator(&urb_ctx->iterator);
}

static void mausb_iterator_seek(struct mausb_device *dev,
				struct mausb_event *event)
{
	struct urb	     *urb = (struct urb *)event->data.urb;
	struct mausb_urb_ctx *urb_ctx;

	urb_ctx = mausb_find_urb_in_tree(urb);

	if (urb_ctx)
		mausb_data_iterator_seek(&urb_ctx->iterator,
					 event->data.iterator_seek_delta);
}

void mausb_complete_urb(struct mausb_event *event)
{
	struct urb *urb = (struct urb *)event->data.urb;

	mausb_pr_debug("transfer_size=%d, rem_transfer_size=%d, status=%d",
		       event->data.transfer_size,
		       event->data.rem_transfer_size, event->status);
	mausb_complete_request(urb,
			       event->data.transfer_size -
			       event->data.rem_transfer_size,
			       event->status);
}

static void mausb_delete_ma_dev(struct mausb_device *dev,
				struct mausb_event *event)
{
	mausb_signal_event(dev, event, event->mgmt.delete_ma_dev.event_id);
}

static void mausb_process_user_finished(struct mausb_device *dev,
					struct mausb_event *event)
{
	complete(&dev->user_finished_event);
}

static int mausb_send_mgmt_msg(struct mausb_device *dev,
			       struct mausb_event *event)
{
	struct mausb_kvec_data_wrapper wrapper;
	struct kvec kvec;
	struct ma_usb_hdr_common *hdr;
	int status;

	hdr = (struct ma_usb_hdr_common *)event->mgmt.mgmt_hdr.hdr;

	mausb_pr_info("event=%d, type=%d", event->type, hdr->type);

	kvec.iov_base	 = hdr;
	kvec.iov_len	 = hdr->length;
	wrapper.kvec	 = &kvec;
	wrapper.kvec_num = 1;
	wrapper.length	 = hdr->length;

	status = mausb_ip_send(dev->mgmt_channel, &wrapper);
	if (status < 0) {
		mausb_pr_err("Send failed. Disconnecting... status=%d", status);
		queue_work(dev->workq, &dev->socket_disconnect_work);
		queue_work(dev->workq, &dev->hcd_disconnect_work);
	}

	return status;
}

static int mausb_get_first_free_port_number(u16 *port_number)
{
	(*port_number) = 0;
	while ((mhcd->connected_ports & (1 << *port_number)) != 0 &&
	       *port_number < NUMBER_OF_PORTS)
		++(*port_number);

	if (*port_number == NUMBER_OF_PORTS)
		return -EINVAL;

	mhcd->connected_ports |= (1 << *port_number);

	return 0;
}

static inline void mausb_port_has_changed_event(struct mausb_device *dev,
						struct mausb_event *event)
{
	int status;
	u16 port_number;
	unsigned long flags = 0;

	spin_lock_irqsave(&mhcd->lock, flags);

	status = mausb_get_first_free_port_number(&port_number);
	if (status < 0) {
		spin_unlock_irqrestore(&mhcd->lock, flags);
		mausb_pr_err("There is no free port, schedule delete ma_dev");
		queue_work(dev->workq, &dev->socket_disconnect_work);
		return;
	}

	spin_unlock_irqrestore(&mhcd->lock, flags);

	dev->dev_type	   = event->port_changed.dev_type;
	dev->dev_speed	   = event->port_changed.dev_speed;
	dev->lse	   = event->port_changed.lse;
	dev->dev_connected = 1;
	dev->port_number   = port_number;

	mausb_port_has_changed(event->port_changed.dev_type,
			       event->port_changed.dev_speed, dev);

	if ((enum mausb_device_type)event->port_changed.dev_type == USB30HUB)
		mausb_port_has_changed(USB20HUB, HIGH_SPEED, dev);
}

static void mausb_complete_timeout_event(struct mausb_device *dev,
					 struct mausb_event *event)
{
	mausb_pr_debug("Event type=%d, event_id=%llu", event->type,
		       event->mgmt.mgmt_req_timedout.event_id);
	mausb_signal_event(dev, event, event->mgmt.mgmt_req_timedout.event_id);
}

static void mausb_process_event(struct mausb_device *dev,
				struct mausb_event *event)
{
	mausb_pr_debug("Event type=%d", event->type);

	switch (event->type) {
	case MAUSB_EVENT_TYPE_USB_DEV_HANDLE:
		mausb_usbdevhandle_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_EP_HANDLE:
		mausb_ephandle_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_EP_HANDLE_ACTIVATE:
		mausb_epactivate_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_EP_HANDLE_INACTIVATE:
		mausb_epinactivate_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_EP_HANDLE_RESET:
		mausb_epreset_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_EP_HANDLE_DELETE:
		mausb_epdelete_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_MODIFY_EP0:
		mausb_modifyep0_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_SET_USB_DEV_ADDRESS:
		mausb_setusbdevaddress_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_UPDATE_DEV:
		mausb_updatedev_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_USB_DEV_RESET:
		mausb_usbdevreset_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_CANCEL_TRANSFER:
		mausb_canceltransfer_event_from_user(dev, event);
		break;
	case MAUSB_EVENT_TYPE_PORT_CHANGED:
		mausb_port_has_changed_event(dev, event);
		break;
	case MAUSB_EVENT_TYPE_PING:
		mausb_send_mgmt_msg(dev, event);
		break;
	case MAUSB_EVENT_TYPE_SEND_MGMT_MSG:
		mausb_send_mgmt_msg(dev, event);
		break;
	case MAUSB_EVENT_TYPE_SEND_DATA_MSG:
		mausb_send_data_msg(dev, event);
		break;
	case MAUSB_EVENT_TYPE_RECEIVED_DATA_MSG:
		mausb_receive_data_msg(dev, event);
		break;
	case MAUSB_EVENT_TYPE_URB_COMPLETE:
		mausb_complete_urb(event);
		break;
	case MAUSB_EVENT_TYPE_SEND_ACK:
		mausb_send_transfer_ack(dev, event);
		mausb_release_event_resources(event);
		break;
	case MAUSB_EVENT_TYPE_ITERATOR_RESET:
		mausb_iterator_reset(dev, event);
		break;
	case MAUSB_EVENT_TYPE_ITERATOR_SEEK:
		mausb_iterator_seek(dev, event);
		break;
	case MAUSB_EVENT_TYPE_DELETE_MA_DEV:
		mausb_delete_ma_dev(dev, event);
		break;
	case MAUSB_EVENT_TYPE_USER_FINISHED:
		mausb_process_user_finished(dev, event);
		break;
	case MAUSB_EVENT_TYPE_RELEASE_EVENT_RESOURCES:
		mausb_release_event_resources(event);
		break;
	case MAUSB_EVENT_TYPE_NONE:
		mausb_release_event_resources(event);
		break;
	case MAUSB_EVENT_TYPE_MGMT_REQUEST_TIMED_OUT:
		mausb_complete_timeout_event(dev, event);
		break;
	default:
		break;
	}

	mausb_notify_completed_user_events(dev->ring_buffer);
}

static void mausb_hpal_kernel_work(struct work_struct *work)
{
	struct mausb_device *dev = container_of(work, struct mausb_device,
						work);
	struct mausb_event *event;
	int status;
	u16 i;
	u16 events;
	u16 completed_events;
	unsigned long flags;
	struct mausb_ring_buffer *dev_mausb_ring = dev->ring_buffer;

	spin_lock_irqsave(&dev->num_of_user_events_lock, flags);
	events = dev->num_of_user_events;
	completed_events = dev->num_of_completed_events;
	dev->num_of_user_events = 0;
	dev->num_of_completed_events = 0;
	spin_unlock_irqrestore(&dev->num_of_user_events_lock, flags);

	status = mausb_ring_buffer_move_tail(dev_mausb_ring, completed_events);
	if (status < 0) {
		mausb_pr_err("Dequeue failed, status=%d", status);
		kref_put(&dev->refcount, mausb_release_ma_dev_async);
		return;
	}

	for (i = 0; i < events; ++i) {
		event = mausb_ring_current_from_user(dev_mausb_ring);
		mausb_ring_next_from_user(dev_mausb_ring);
		mausb_process_event(dev, event);
	}
}

static void mausb_socket_disconnect_event(struct work_struct *work)
{
	struct mausb_device *dev = container_of(work, struct mausb_device,
						socket_disconnect_work);
	struct mausb_event event;
	int status;

	mausb_pr_info("madev_addr=%d", dev->madev_addr);

	mausb_ip_disconnect(dev->ctrl_channel);
	mausb_destroy_ip_ctx(dev->ctrl_channel);
	dev->ctrl_channel = NULL;

	mausb_ip_disconnect(dev->bulk_channel);
	mausb_destroy_ip_ctx(dev->bulk_channel);
	dev->bulk_channel = NULL;

	mausb_ip_disconnect(dev->isoch_channel);
	mausb_destroy_ip_ctx(dev->isoch_channel);
	dev->isoch_channel = NULL;

	if (dev->mgmt_channel) {
		memset(&event, 0, sizeof(event));
		event.type = MAUSB_EVENT_TYPE_NETWORK_DISCONNECTED;
		event.data.device_id = dev->id;

		status = mausb_enqueue_event_to_user(dev, &event);

		mausb_pr_info("Sending notification to user that network is disconnected status=%d",
			      status);

		mausb_pr_info("Releasing MAUSB device ref");
		kref_put(&dev->refcount, mausb_release_ma_dev_async);
	}

	mausb_ip_disconnect(dev->mgmt_channel);
	mausb_destroy_ip_ctx(dev->mgmt_channel);
	dev->mgmt_channel = NULL;

	memset(dev->channel_map, 0, sizeof(dev->channel_map));
}

static void mausb_disconnect_ma_dev(struct mausb_device *dev)
{
	mausb_pr_info("Disconnecting MAUSB device madev_addr=%d",
		      dev->madev_addr);

	if (!dev->dev_connected) {
		mausb_pr_warn("MAUSB device is not connected");
		kref_put(&dev->refcount, mausb_release_ma_dev_async);
		return;
	}
	mausb_hcd_disconnect(dev->port_number, dev->dev_type, dev->dev_speed);

	if (dev->dev_type == USB30HUB)
		mausb_hcd_disconnect(dev->port_number, USB20HUB, HIGH_SPEED);
}

static void mausb_hcd_disconnect_event(struct work_struct *work)
{
	struct mausb_device *ma_dev = container_of(work, struct mausb_device,
						   hcd_disconnect_work);

	mausb_disconnect_ma_dev(ma_dev);
}

static void mausb_delete_madev(struct work_struct *work)
{
	struct mausb_device *dev = container_of(work, struct mausb_device,
						madev_delete_work);
	struct mausb_event	event;
	struct completion	completion;
	struct completion	*user_event;
	struct mausb_completion mausb_completion;
	long status;
	unsigned long timeout = msecs_to_jiffies(MAUSB_DELETE_MADEV_TIMEOUT_MS);

	mausb_pr_info("Deleting MAUSB device madev_addr=%d", dev->madev_addr);

	del_timer_sync(&dev->connection_timer);

	/* Client IS responsive */
	if (!atomic_read(&dev->unresponsive_client)) {
		memset(&event, 0, sizeof(event));
		event.type = MAUSB_EVENT_TYPE_DELETE_MA_DEV;
		event.mgmt.delete_ma_dev.device_id = dev->id;
		event.mgmt.delete_ma_dev.event_id  = mausb_event_id(dev);

		init_completion(&completion);
		mausb_completion.completion_event = &completion;
		mausb_completion.event_id = event.mgmt.delete_ma_dev.event_id;
		mausb_completion.mausb_event = &event;

		mausb_insert_event(dev, &mausb_completion);

		status = mausb_enqueue_event_to_user(dev, &event);
		if (status < 0) {
			mausb_remove_event(dev, &mausb_completion);
			mausb_pr_err("Ring buffer full, enqueue failed");
			return;
		}
		mausb_pr_debug("Deleting MAUSB device...");

		status = wait_for_completion_interruptible_timeout(&completion,
								   timeout);

		mausb_pr_debug("Deleting MAUSB device event finished with %ld",
			       status);

		mausb_remove_event(dev, &mausb_completion);

		user_event = &dev->user_finished_event;

		status = wait_for_completion_interruptible_timeout(user_event,
								   timeout);
		mausb_pr_info("User event finished with %ld", status);
	}

	flush_workqueue(dev->workq);
	destroy_workqueue(dev->workq);

	mausb_clear_hcd_madev(dev->port_number);

	mausb_ring_buffer_cleanup(dev->ring_buffer);
	mausb_ring_buffer_destroy(dev->ring_buffer);

	mausb_remove_madev_from_list(dev->madev_addr);

	put_net(dev->net_ns);

	kfree(dev->ring_buffer);
	kfree(dev);
	mausb_signal_empty_mss();

	mausb_pr_info("MAUSB device deleted. Version=%s", MAUSB_DRIVER_VERSION);
}

static void mausb_ping_work(struct work_struct *work)
{
	struct mausb_device *dev = container_of(work, struct mausb_device,
						ping_work);
	int status = 0;

	if (mausb_start_connection_timer(dev) < 0) {
		mausb_pr_err("Device disconnecting due to session timeout madev_addr=%d",
			     dev->madev_addr);
		queue_work(dev->workq, &dev->socket_disconnect_work);
		queue_work(dev->workq, &dev->hcd_disconnect_work);
		return;
	}

	status = mausb_ping_event_to_user(dev);

	if (status < 0) {
		mausb_pr_err("Ring buffer full");
		return;
	}
}

static void mausb_heartbeat_work(struct work_struct *work)
{
	struct mausb_device *dev = container_of(work, struct mausb_device,
						heartbeat_work);

	mausb_pr_err("Device disconnecting - app is unresponsive");
	atomic_set(&dev->unresponsive_client, 1);
	mausb_complete_urbs_from_tree();
	queue_work(dev->workq, &dev->socket_disconnect_work);
	queue_work(dev->workq, &dev->hcd_disconnect_work);
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void mausb_connection_timer_func(struct timer_list *timer)
{
	struct mausb_device *dev = container_of(timer, struct mausb_device,
						connection_timer);
#else
static void mausb_connection_timer_func(unsigned long data)
{
	struct mausb_device *dev = (struct mausb_device *)data;
#endif

	queue_work(dev->workq, &dev->ping_work);
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
static void mausb_heartbeat_timer_func(struct timer_list *timer)
{
#else
static void mausb_heartbeat_timer_func(unsigned long data)
{
#endif
	unsigned long flags = 0;
	struct mausb_device *dev = NULL;

	if (mausb_start_heartbeat_timer() < 0) {
		mausb_pr_err("Devices disconnecting - app is unresponsive");
		spin_lock_irqsave(&mss.lock, flags);

		/* Reset connected clients */
		mss.client_connected = false;
		mss.missed_heartbeats = 0;

		list_for_each_entry(dev, &mss.madev_list, list_entry) {
			mausb_pr_debug("Enqueue heartbeat_work madev_addr=%x",
				       dev->madev_addr);
			queue_work(dev->workq, &dev->heartbeat_work);
		}

		complete(&mss.client_stopped);
		spin_unlock_irqrestore(&mss.lock, flags);
	}
}

static struct mausb_device *mausb_create_madev(struct mausb_device_address
					dev_addr, u8 madev_address,
					int *status)
{
	struct mausb_device *dev;
	unsigned long flags = 0;
	char workq_name[16];
	struct workqueue_struct *workq;

	memset(workq_name, 0, sizeof(workq_name));
	sprintf(workq_name, "%x", madev_address);
	strcat(workq_name, "_madev_workq");

	mausb_pr_debug("madev_workq_name = %s", workq_name);

	workq = alloc_ordered_workqueue(workq_name, WQ_MEM_RECLAIM);
	if (!workq) {
		mausb_pr_alert("Could not allocate workqueue!");
		*status = -ENOMEM;
		return NULL;
	}

	spin_lock_irqsave(&mss.lock, flags);

	if (mss.deinit_in_progress) {
		spin_unlock_irqrestore(&mss.lock, flags);
		mausb_pr_alert("Device creating failed - mss deinit in progress");
		flush_workqueue(workq);
		destroy_workqueue(workq);
		*status = -ESHUTDOWN;
		return NULL;
	}

	dev = mausb_get_dev_from_addr_unsafe(madev_address);
	if (dev) {
		spin_unlock_irqrestore(&mss.lock, flags);
		mausb_pr_debug("MAUSB device already connected, madev_address=%x",
			       madev_address);
		flush_workqueue(workq);
		destroy_workqueue(workq);
		*status = -EEXIST;
		return NULL;
	}

	dev = kzalloc(sizeof(*dev), GFP_ATOMIC);

	if (!dev) {
		spin_unlock_irqrestore(&mss.lock, flags);
		mausb_pr_alert("Could not allocate MAUSB device!");
		flush_workqueue(workq);
		destroy_workqueue(workq);
		*status = -ENOMEM;
		return NULL;
	}

	mausb_pr_info("Create MAUSB device. Version=%s", MAUSB_DRIVER_VERSION);

	dev->workq = workq;

	INIT_WORK(&dev->work, mausb_hpal_kernel_work);
	INIT_WORK(&dev->socket_disconnect_work, mausb_socket_disconnect_event);
	INIT_WORK(&dev->hcd_disconnect_work, mausb_hcd_disconnect_event);
	INIT_WORK(&dev->madev_delete_work, mausb_delete_madev);
	INIT_WORK(&dev->ping_work, mausb_ping_work);
	INIT_WORK(&dev->heartbeat_work, mausb_heartbeat_work);

	kref_init(&dev->refcount);

	dev->event_id = 0;
	spin_lock_init(&dev->event_id_lock);

	INIT_LIST_HEAD(&dev->completion_events);
	spin_lock_init(&dev->completion_events_lock);
	spin_lock_init(&dev->num_of_user_events_lock);
	spin_lock_init(&dev->connection_timer_lock);

	init_completion(&dev->user_finished_event);
	atomic_set(&dev->unresponsive_client, 0);

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	timer_setup(&dev->connection_timer, mausb_connection_timer_func, 0);
#else
	setup_timer(&dev->connection_timer, mausb_connection_timer_func,
		    (unsigned long)dev);
#endif

	dev->dev_addr = dev_addr;
	dev->madev_addr = madev_address;
	dev->net_ns = get_net(current->nsproxy->net_ns);

	if (!list_empty(&mss.available_ring_buffers)) {
		dev->ring_buffer = container_of(mss.available_ring_buffers.next,
						struct mausb_ring_buffer,
						list_entry);
		list_del(mss.available_ring_buffers.next);
	} else {
		mausb_pr_alert("Ring buffer for mausb device is not availbale!");
	}

	list_add_tail(&dev->list_entry, &mss.madev_list);

	reinit_completion(&mss.empty);

	spin_unlock_irqrestore(&mss.lock, flags);

	return dev;
}

void mausb_release_ma_dev_async(struct kref *kref)
{
	struct mausb_device *dev = container_of(kref, struct mausb_device,
						refcount);

	mausb_pr_info("Scheduling work for MAUSB device to be deleted");

	schedule_work(&dev->madev_delete_work);
}

int mausb_initiate_dev_connection(struct mausb_device_address dev_addr,
				  u8 madev_address)
{
	int error = 0;
	struct mausb_device *dev;
	unsigned long flags = 0;

	spin_lock_irqsave(&mss.lock, flags);
	dev = mausb_get_dev_from_addr_unsafe(madev_address);
	spin_unlock_irqrestore(&mss.lock, flags);

	if (dev) {
		mausb_pr_debug("MAUSB device already connected, madev_address=%x",
			       madev_address);
		return -EEXIST;
	}

	dev = mausb_create_madev(dev_addr, madev_address, &error);

	if (!dev)
		return error;

	mausb_pr_info("New MAUSB device created madev_addr=%d", madev_address);

	error = mausb_init_ip_ctx(&dev->mgmt_channel, dev->net_ns,
				  dev->dev_addr.ip.address.ip4,
				  dev->dev_addr.ip.port.management, dev,
				  mausb_ip_callback, MAUSB_MGMT_CHANNEL);
	if (error) {
		mausb_pr_err("Mgmt ip context init failed: error=%d", error);
		kref_put(&dev->refcount, mausb_release_ma_dev_async);
		return error;
	}

	mausb_ip_connect_async(dev->mgmt_channel);

	return 0;
}

void mausb_on_madev_connected(struct mausb_device *dev)
{
	struct mausb_event mausb_event;

	mausb_dev_reset_req_event(&mausb_event);
	mausb_enqueue_event_to_user(dev, &mausb_event);
}

int mausb_enqueue_event_from_user(u8 madev_addr, u16 num_of_events,
				  u16 num_of_completed)
{
	unsigned long flags;
	struct mausb_device *dev;

	spin_lock_irqsave(&mss.lock, flags);
	dev = mausb_get_dev_from_addr_unsafe(madev_addr);

	if (!dev) {
		spin_unlock_irqrestore(&mss.lock, flags);
		return -EINVAL;
	}

	spin_lock_irqsave(&dev->num_of_user_events_lock, flags);
	dev->num_of_user_events += num_of_events;
	dev->num_of_completed_events += num_of_completed;
	spin_unlock_irqrestore(&dev->num_of_user_events_lock, flags);
	queue_work(dev->workq, &dev->work);
	spin_unlock_irqrestore(&mss.lock, flags);

	return 0;
}

int mausb_enqueue_event_to_user(struct mausb_device *dev,
				struct mausb_event *event)
{
	int status;

	event->madev_addr = dev->madev_addr;
	status = mausb_ring_buffer_put(dev->ring_buffer, event);
	if (status < 0) {
		mausb_pr_err("Ring buffer operation failed");
		mausb_cleanup_ring_buffer_event(event);
		return status;
	}

	mausb_notify_ring_events(dev->ring_buffer);
	mausb_pr_debug("User-space notification sent.");

	return 0;
}

int mausb_data_req_enqueue_event(struct mausb_device *dev, u16 ep_handle,
				 struct urb *request)
{
	int status;
	struct mausb_event mausb_event;

	mausb_event.type   = MAUSB_EVENT_TYPE_SEND_DATA_MSG;
	mausb_event.status = 0;

	mausb_event.data.transfer_type =
		mausb_transfer_type_from_usb(&request->ep->desc);
	mausb_event.data.device_id	= dev->id;
	mausb_event.data.ep_handle	= ep_handle;
	mausb_event.data.urb		= (u64)request;
	mausb_event.data.setup_packet	=
		(usb_endpoint_xfer_control(&request->ep->desc) &&
			request->setup_packet);
	mausb_event.data.transfer_size	= request->transfer_buffer_length;
	mausb_event.data.direction	= (usb_urb_dir_in(request) ?
						MAUSB_DATA_MSG_DIRECTION_IN :
						MAUSB_DATA_MSG_DIRECTION_OUT);
	mausb_event.data.transfer_size +=
		((mausb_event.data.direction == MAUSB_DATA_MSG_DIRECTION_OUT &&
			mausb_event.data.setup_packet) ?
				MAUSB_CONTROL_SETUP_SIZE : 0);
	mausb_event.data.rem_transfer_size = mausb_event.data.transfer_size;
	mausb_event.data.transfer_flags	   = request->transfer_flags;
	mausb_event.data.transfer_eot	   = false;
	mausb_event.data.isoch_seg_num	   = (u32)request->number_of_packets;
	mausb_event.data.recv_buf	   = 0;
	mausb_event.data.payload_size	   =
		(usb_endpoint_xfer_isoc(&request->ep->desc) &&
		 usb_endpoint_dir_out(&request->ep->desc)) ?
		(request->iso_frame_desc[request->number_of_packets - 1]
								.offset +
		 request->iso_frame_desc[request->number_of_packets - 1]
								.length) : 0;

	if (mausb_event.data.setup_packet) {
		memcpy(mausb_event.data.hdr_ack, request->setup_packet,
		       MAUSB_CONTROL_SETUP_SIZE);
		memcpy(shift_ptr(mausb_event.data.hdr_ack,
				 MAUSB_CONTROL_SETUP_SIZE),
		       &request->dev->route, sizeof(request->dev->route));
	}

	status = mausb_enqueue_event_to_user(dev, &mausb_event);
	if (status < 0)
		mausb_pr_err("Failed to enqueue event to user-space ep_handle=%#x, status=%d",
			     mausb_event.data.ep_handle, status);

	return status;
}

void mausb_complete_request(struct urb *urb, u32 actual_length, int status)
{
	mausb_hcd_urb_complete(urb, actual_length, status);
}

int mausb_signal_event(struct mausb_device *dev,
		       struct mausb_event *event, u64 event_id)
{
	unsigned long flags;
	struct mausb_completion *mausb_completion;

	spin_lock_irqsave(&dev->completion_events_lock, flags);
	list_for_each_entry(mausb_completion, &dev->completion_events,
			    list_entry) {
		if (mausb_completion->event_id == event_id) {
			memcpy(mausb_completion->mausb_event, event,
			       sizeof(*event));
			complete(mausb_completion->completion_event);
			spin_unlock_irqrestore(&dev->completion_events_lock,
					       flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dev->completion_events_lock, flags);

	return -ETIMEDOUT;
}

static int mausb_start_connection_timer(struct mausb_device *dev)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&dev->connection_timer_lock, flags);

	if (++dev->receive_failures_num > MAUSB_MAX_RECEIVE_FAILURES) {
		mausb_pr_err("Missed more than %d ping responses",
			     MAUSB_MAX_RECEIVE_FAILURES);
		spin_unlock_irqrestore(&dev->connection_timer_lock, flags);
		return -ETIMEDOUT;
	}

	mod_timer(&dev->connection_timer, jiffies + msecs_to_jiffies(1000));

	spin_unlock_irqrestore(&dev->connection_timer_lock, flags);

	return 0;
}

void mausb_reset_connection_timer(struct mausb_device *dev)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&dev->connection_timer_lock, flags);
	dev->receive_failures_num = 0;

	mod_timer(&dev->connection_timer, jiffies + msecs_to_jiffies(1000));

	spin_unlock_irqrestore(&dev->connection_timer_lock, flags);
}

static int mausb_start_heartbeat_timer(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&mss.lock, flags);
	if (++mss.missed_heartbeats > MAUSB_MAX_MISSED_HEARTBEATS) {
		mausb_pr_err("Missed more than %d heartbeats",
			     MAUSB_MAX_MISSED_HEARTBEATS);
		spin_unlock_irqrestore(&mss.lock, flags);
		return -ETIMEDOUT;
	}

	spin_unlock_irqrestore(&mss.lock, flags);
	mod_timer(&mss.heartbeat_timer,
		  jiffies + msecs_to_jiffies(MAUSB_HEARTBEAT_TIMEOUT_MS));

	return 0;
}

void mausb_reset_heartbeat_cnt(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&mss.lock, flags);
	mss.missed_heartbeats = 0;
	spin_unlock_irqrestore(&mss.lock, flags);
}

static void mausb_execute_urb_dequeue(struct work_struct *dequeue_work)
{
	struct mausb_urb_ctx *urb_ctx =
			container_of(dequeue_work, struct mausb_urb_ctx, work);
	struct urb		  *urb = urb_ctx->urb;
	struct mausb_endpoint_ctx *ep_ctx;
	struct mausb_device	  *ma_dev;
	struct mausb_event	  mausb_event;
	int status = 0;

	ep_ctx = urb->ep->hcpriv;
	ma_dev = ep_ctx->ma_dev;

	if (atomic_read(&ma_dev->unresponsive_client)) {
		mausb_pr_err("Client is not responsive anymore - finish urb immediately urb=%p, ep_handle=%#x, dev_handle=%#x",
			     urb, ep_ctx->ep_handle, ep_ctx->dev_handle);
		goto complete_urb;
	}

	mausb_pr_debug("urb=%p, ep_handle=%#x, dev_handle=%#x",
		       urb, ep_ctx->ep_handle, ep_ctx->dev_handle);

	if (!usb_endpoint_xfer_isoc(&urb->ep->desc)) {
		status = mausb_canceltransfer_event_to_user(ep_ctx->ma_dev,
							    ep_ctx->dev_handle,
							    ep_ctx->ep_handle,
							    (u64)urb);
		if (status < 0) {
			mausb_pr_err("Failed to enqueue cancel transfer to user");
			goto complete_urb;
		}
	}

	memset(&mausb_event, 0, sizeof(mausb_event));

	mausb_event.type   = MAUSB_EVENT_TYPE_DELETE_DATA_TRANSFER;
	mausb_event.status = 0;

	mausb_event.data.transfer_type =
				mausb_transfer_type_from_usb(&urb->ep->desc);
	mausb_event.data.device_id     = ma_dev->id;
	mausb_event.data.ep_handle     = ep_ctx->ep_handle;
	mausb_event.data.urb	       = (u64)urb;
	mausb_event.data.direction     = (usb_urb_dir_in(urb) ?
						MAUSB_DATA_MSG_DIRECTION_IN :
						MAUSB_DATA_MSG_DIRECTION_OUT);

	status = mausb_enqueue_event_to_user(ep_ctx->ma_dev, &mausb_event);
	if (status < 0) {
		mausb_pr_alert("Failed to enqueue event to user-space ep_handle=%#x, status=%d",
			       mausb_event.data.ep_handle, status);
		goto complete_urb;
	}

	if (!mausb_return_urb_ctx_to_tree(urb_ctx, false)) {
		mausb_pr_alert("Failed to insert in tree urb=%p ep_handle=%#x, status=%d",
			       urb, mausb_event.data.ep_handle, status);
		goto complete_urb;
	}

	return;

complete_urb:

	/* Deallocate urb_ctx */
	mausb_uninit_data_iterator(&urb_ctx->iterator);
	kfree(urb_ctx);

	urb->status	   = -EPROTO;
	urb->actual_length = 0;
	atomic_dec(&urb->use_count);
	usb_hcd_giveback_urb(urb->hcpriv, urb, urb->status);
}

void mausb_initialize_mss(void)
{
	spin_lock_init(&mss.lock);
	INIT_LIST_HEAD(&mss.madev_list);
	INIT_LIST_HEAD(&mss.available_ring_buffers);

	init_completion(&mss.empty);
	complete(&mss.empty);
	init_completion(&mss.rings_events.mausb_ring_has_events);
	atomic_set(&mss.rings_events.mausb_stop_reading_ring_events, 0);
	mss.deinit_in_progress	= false;
	mss.ring_buffer_id	= 0;
	mss.client_connected = false;
	mss.missed_heartbeats = 0;
	init_completion(&mss.client_stopped);
	atomic_set(&mss.num_of_transitions_to_sleep, 0);

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	timer_setup(&mss.heartbeat_timer, mausb_heartbeat_timer_func, 0);
#else
	setup_timer(&mss.heartbeat_timer, mausb_heartbeat_timer_func, 0);
#endif
}

void mausb_deinitialize_mss(void)
{
	struct mausb_device *dev = NULL;
	unsigned long flags = 0;
	unsigned long timeout =
			msecs_to_jiffies(MAUSB_CLIENT_STOPPED_TIMEOUT_MS);

	spin_lock_irqsave(&mss.lock, flags);

	mss.deinit_in_progress = true;

	list_for_each_entry(dev, &mss.madev_list, list_entry) {
		mausb_pr_debug("Enqueue mausb_hcd_disconnect_work madev_addr=%x",
			       dev->madev_addr);
		queue_work(dev->workq, &dev->hcd_disconnect_work);
	}

	spin_unlock_irqrestore(&mss.lock, flags);

	wait_for_completion(&mss.empty);
	mausb_pr_debug("Waiting for completion on disconnect_event ended");
	mausb_stop_ring_events();

	timeout = wait_for_completion_timeout(&mss.client_stopped, timeout);
	mausb_pr_info("Remaining time after waiting for stopping client %ld",
		      timeout);
}

int mausb_register_power_state_listener(void)
{
	mausb_pr_info("Registering power states listener");

	mhcd->power_state_listener.notifier_call = mausb_power_state_cb;
	return register_pm_notifier(&mhcd->power_state_listener);
}

void mausb_unregister_power_state_listener(void)
{
	mausb_pr_info("Un-registering power states listener");

	unregister_pm_notifier(&mhcd->power_state_listener);
}

static int mausb_power_state_cb(struct notifier_block *nb, unsigned long action,
				void *data)
{
	unsigned long flags = 0;
	struct mausb_device *dev = NULL;

	mausb_pr_info("Power state callback action = %ld", action);
	if (action == PM_SUSPEND_PREPARE || action == PM_HIBERNATION_PREPARE) {
		/* Stop heartbeat timer */
		del_timer_sync(&mss.heartbeat_timer);
		mausb_pr_info("Saving state before sleep");
		spin_lock_irqsave(&mss.lock, flags);
		if (!list_empty(&mss.madev_list))
			atomic_inc(&mss.num_of_transitions_to_sleep);

		list_for_each_entry(dev, &mss.madev_list, list_entry) {
			mausb_pr_info("Enqueue heartbeat_work madev_addr=%x",
				      dev->madev_addr);
			queue_work(dev->workq, &dev->heartbeat_work);
		}

		spin_unlock_irqrestore(&mss.lock, flags);
	} else if (action == PM_POST_SUSPEND || action == PM_POST_HIBERNATION) {
		mausb_reset_heartbeat_cnt();
		/* Start hearbeat timer */
		mod_timer(&mss.heartbeat_timer, jiffies +
			  msecs_to_jiffies(MAUSB_HEARTBEAT_TIMEOUT_MS));
	}
	return NOTIFY_OK;
}

struct mausb_device *mausb_get_dev_from_addr_unsafe(u8 madev_addr)
{
	struct mausb_device *dev = NULL;

	list_for_each_entry(dev, &mss.madev_list, list_entry) {
		if (dev->madev_addr == madev_addr)
			return dev;
	}

	return NULL;
}

static void mausb_remove_madev_from_list(u8 madev_addr)
{
	unsigned long flags = 0;
	struct mausb_device *ma_dev, *tmp = NULL;

	spin_lock_irqsave(&mss.lock, flags);

	list_for_each_entry_safe(ma_dev, tmp, &mss.madev_list, list_entry) {
		if (ma_dev->madev_addr == madev_addr) {
			list_del(&ma_dev->list_entry);
			break;
		}
	}

	if (list_empty(&mss.madev_list))
		reinit_completion(&mss.rings_events.mausb_ring_has_events);

	spin_unlock_irqrestore(&mss.lock, flags);
}

static void mausb_signal_empty_mss(void)
{
	unsigned long flags = 0;

	spin_lock_irqsave(&mss.lock, flags);
	if (list_empty(&mss.madev_list))
		complete(&mss.empty);
	spin_unlock_irqrestore(&mss.lock, flags);
}
