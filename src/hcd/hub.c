// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hcd/hub.h"

#include <linux/version.h>

#include "hcd/vhcd.h"
#include "hpal/hpal.h"
#include "hpal/mausb_events.h"
#include "utils/mausb_logs.h"

static void mausb_get_hub_descriptor(struct usb_hcd *hcd, uint16_t typeReq,
				     uint16_t wValue, uint16_t wIndex,
				     char *buff, uint16_t wLength);
static void mausb_set_port_feature(struct usb_hcd *hcd, uint16_t typeReq,
				   uint16_t wValue, uint16_t wIndex, char *buff,
				   uint16_t wLength);
static void mausb_get_port_status(struct usb_hcd *hcd, uint16_t typeReq,
				  uint16_t wValue, uint16_t wIndex, char *buff,
				  uint16_t wLength);
static void mausb_clear_port_feature(struct usb_hcd *hcd, uint16_t typeReq,
				     uint16_t wValue, uint16_t wIndex,
				     char *buff, uint16_t wLength);
static void mausb_get_hub_status(struct usb_hcd *hcd, uint16_t typeReq,
				 uint16_t wValue, uint16_t wIndex, char *buff,
				 uint16_t wLength);
static int mausb_add_endpoint(struct usb_hcd *hcd, struct usb_device *dev,
		struct usb_host_endpoint *endpoint);
static int mausb_address_device(struct usb_hcd *hcd, struct usb_device *dev);
static int mausb_alloc_dev(struct usb_hcd *hcd, struct usb_device *dev);
static int mausb_check_bandwidth(struct usb_hcd *hcd, struct usb_device *dev);
static int mausb_drop_endpoint(struct usb_hcd *hcd, struct usb_device *dev,
			struct usb_host_endpoint *endpoint);
static int mausb_enable_device(struct usb_hcd *hcd, struct usb_device *dev);
static void mausb_endpoint_disable(struct usb_hcd *hcd,
			struct usb_host_endpoint *endpoint);
static void mausb_endpoint_reset(struct usb_hcd *hcd,
			  struct usb_host_endpoint *endpoint);
static void mausb_free_dev(struct usb_hcd *hcd, struct usb_device *dev);
static int mausb_hcd_bus_resume(struct usb_hcd *hcd);
static int mausb_hcd_bus_suspend(struct usb_hcd *hcd);
static int mausb_hcd_get_frame_number(struct usb_hcd *hcd);
static int mausb_hcd_hub_control(struct usb_hcd *hcd, uint16_t typeReq,
				uint16_t wValue, uint16_t wIndex, char *buff,
				uint16_t wLength);
static int mausb_hcd_hub_status(struct usb_hcd *hcd, char *buff);
static int mausb_hcd_reset(struct usb_hcd *hcd);
static int mausb_hcd_start(struct usb_hcd *hcd);
static void mausb_hcd_stop(struct usb_hcd *hcd);
static int mausb_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
				int status);
static int mausb_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
				gfp_t mem_flags);
static int mausb_hub_update_device(struct usb_hcd *hcd, struct usb_device *dev,
				struct usb_tt *tt, gfp_t mem_flags);
static void mausb_reset_bandwidth(struct usb_hcd *hcd, struct usb_device *dev);
static int mausb_reset_device(struct usb_hcd *hcd, struct usb_device *dev);
static int mausb_update_device(struct usb_hcd *hcd, struct usb_device *dev);

static void mausb_print_urb(struct urb *request)
{
	mausb_pr_debug("URB: urb=%p, ep_handle=%#x, packet_num=%d, setup_dma=%lld, is_setup_packet=%d, is_ep=%d, is_sg=%d, num_sgs=%d, num_mapped_sgs=%d, status=%d, is_transfer_buffer=%d, transfer_buffer_length=%d, is_transfer_dma=%d, transfer_flags=%d, is_hcpriv=%d",
		request,
		((struct mausb_endpoint_ctx *)request->ep->hcpriv)->ep_handle,
		request->number_of_packets, request->setup_dma,
		request->setup_packet != 0, request->ep != 0, request->sg != 0,
		request->num_sgs, request->num_mapped_sgs, request->status,
		request->transfer_buffer != 0, request->transfer_buffer_length,
		request->transfer_dma != 0, request->transfer_flags,
		request->ep ? request->ep->hcpriv != 0 : 0);
}

static const struct hc_driver mausb_hc_driver = {
	.description  =  driver_name,
	.product_desc = driver_name,
	.flags	      = HCD_USB3 | HCD_SHARED,

	.hcd_priv_size = sizeof(struct hub_ctx),

	.reset = mausb_hcd_reset,
	.start = mausb_hcd_start,
	.stop  = mausb_hcd_stop,

	.urb_enqueue = mausb_hcd_urb_enqueue,
	.urb_dequeue = mausb_hcd_urb_dequeue,

	.get_frame_number = mausb_hcd_get_frame_number,

	.hub_status_data   = mausb_hcd_hub_status,
	.hub_control	   = mausb_hcd_hub_control,
	.update_hub_device = mausb_hub_update_device,
	.bus_suspend	   = mausb_hcd_bus_suspend,
	.bus_resume	   = mausb_hcd_bus_resume,

	.alloc_dev	= mausb_alloc_dev,
	.free_dev	= mausb_free_dev,
	.enable_device	= mausb_enable_device,
	.update_device	= mausb_update_device,
	.reset_device	= mausb_reset_device,

	.add_endpoint	  = mausb_add_endpoint,
	.drop_endpoint	  = mausb_drop_endpoint,
	.check_bandwidth  = mausb_check_bandwidth,
	.reset_bandwidth  = mausb_reset_bandwidth,
	.address_device   = mausb_address_device,
	.endpoint_disable = mausb_endpoint_disable,
	.endpoint_reset	  = mausb_endpoint_reset,

#ifdef ISOCH_CALLBACKS
	.sec_event_ring_setup		= mausb_sec_event_ring_setup,
	.sec_event_ring_cleanup		= mausb_sec_event_ring_cleanup,
	.get_sec_event_ring_phys_addr	= mausb_get_sec_event_ring_phys_addr,
	.get_xfer_ring_phys_addr	= mausb_get_xfer_ring_phys_addr,
	.get_core_id			= mausb_get_core_id
#endif /* ISOCH_CALLBACKS */

};

static struct {
	struct usb_bos_descriptor    bos;
	struct usb_ss_cap_descriptor ss_cap;
} usb3_bos_desc = {
	.bos = {
		.bLength	 = USB_DT_BOS_SIZE,
		.bDescriptorType = USB_DT_BOS,
		.wTotalLength	 = cpu_to_le16(sizeof(usb3_bos_desc)),
		.bNumDeviceCaps	 = 1
	},
	.ss_cap = {
		.bLength		= USB_DT_USB_SS_CAP_SIZE,
		.bDescriptorType	= USB_DT_DEVICE_CAPABILITY,
		.bDevCapabilityType	= USB_SS_CAP_TYPE,
		.wSpeedSupported	= cpu_to_le16(USB_5GBPS_OPERATION),
		.bFunctionalitySupport	= ilog2(USB_5GBPS_OPERATION)
	}
};

static int8_t get_root_hub_port_number(struct usb_device *dev)
{
	struct usb_device *first_hub_device = dev;

	if (!dev->parent) {
		mausb_pr_info("Trying to get roothub port number");
		return -1;
	}

	while (first_hub_device->parent->parent)
		first_hub_device = first_hub_device->parent;

	return first_hub_device->portnum - 1;
}

static int usb_to_mausb_device_speed(uint8_t speed)
{
	switch (speed) {
	case USB_SPEED_LOW:
		return MA_USB_SPEED_LOW_SPEED;
	case USB_SPEED_FULL:
		return MA_USB_SPEED_FULL_SPEED;
	case USB_SPEED_WIRELESS:
	case USB_SPEED_HIGH:
		return MA_USB_SPEED_HIGH_SPEED;
	case USB_SPEED_SUPER:
		return MA_USB_SPEED_SUPER_SPEED;
#if KERNEL_VERSION(4, 6, 0) <= LINUX_VERSION_CODE
	case USB_SPEED_SUPER_PLUS:
		return MA_USB_SPEED_SUPER_SPEED_PLUS;
#endif /* #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0) */
	default:
		return -EINVAL;
	}
}

static struct mausb_usb_device_ctx *mausb_find_usb_device(struct mausb_dev
							*mdevs, void *dev_addr)
{
	struct rb_node *node = mdevs->usb_devices.rb_node;

	while (node) {
		struct mausb_usb_device_ctx *usb_device =
		    rb_entry(node, struct mausb_usb_device_ctx, rb_node);

		if (dev_addr < usb_device->dev_addr)
			node = usb_device->rb_node.rb_left;
		else if (dev_addr > usb_device->dev_addr)
			node = usb_device->rb_node.rb_right;
		else
			return usb_device;
	}
	return NULL;
}

static int mausb_insert_usb_device(struct mausb_dev *mdevs,
			    struct mausb_usb_device_ctx *usb_device)
{
	struct rb_node **new_node = &(mdevs->usb_devices.rb_node),
		       *parent = NULL;
	struct mausb_usb_device_ctx *current_usb_device = NULL;

	while (*new_node) {
		parent = *new_node;
		current_usb_device = rb_entry(*new_node,
					      struct mausb_usb_device_ctx,
					      rb_node);

		if (usb_device->dev_addr < current_usb_device->dev_addr)
			new_node = &((*new_node)->rb_left);
		else if (usb_device->dev_addr > current_usb_device->dev_addr)
			new_node = &((*new_node)->rb_right);
		else
			return -EEXIST;
	}
	rb_link_node(&usb_device->rb_node, parent, new_node);
	rb_insert_color(&usb_device->rb_node, &mdevs->usb_devices);
	return 0;
}

void mausb_port_has_changed(const enum mausb_device_type device_type,
			    const enum mausb_device_speed device_speed,
			    void *ma_dev)
{
	struct usb_hcd *hcd;
	unsigned long flags = 0;
	struct mausb_device *dev = ma_dev;
	uint16_t port_number = dev->port_number;

	spin_lock_irqsave(&mhcd->lock, flags);

	if (device_type == USB20HUB || device_speed < SUPER_SPEED) {
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
		    USB_PORT_STAT_CONNECTION | (1 <<
						USB_PORT_FEAT_C_CONNECTION);

		if (device_speed == LOW_SPEED) {
			mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
			    MAUSB_PORT_20_STATUS_LOW_SPEED;
			mhcd->hcd_hs_ctx->ma_devs[port_number].dev_speed =
			    LOW_SPEED;
		} else if (device_speed == HIGH_SPEED) {
			mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
			    MAUSB_PORT_20_STATUS_HIGH_SPEED;
			mhcd->hcd_hs_ctx->ma_devs[port_number].dev_speed =
			    HIGH_SPEED;
		}

		hcd = mhcd->hcd_hs_ctx->hcd;
		mhcd->hcd_hs_ctx->ma_devs[port_number].ma_dev = ma_dev;
	} else {
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status |=
		    USB_PORT_STAT_CONNECTION | (1 <<
						USB_PORT_FEAT_C_CONNECTION);
		mhcd->hcd_ss_ctx->ma_devs[port_number].dev_speed = SUPER_SPEED;

		hcd = mhcd->hcd_ss_ctx->hcd;
		mhcd->hcd_ss_ctx->ma_devs[port_number].ma_dev = ma_dev;
	}
	spin_unlock_irqrestore(&mhcd->lock, flags);

	usb_hcd_poll_rh_status(hcd);
}

void mausb_hcd_disconnect(const uint16_t port_number,
			  const enum mausb_device_type device_type,
			  const enum mausb_device_speed device_speed)
{
	struct usb_hcd *hcd;
	unsigned long flags;

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_err("port number out of range, port_number=%x",
			     port_number);
		return;
	}

	spin_lock_irqsave(&mhcd->lock, flags);

	if (device_type == USB20HUB || device_speed < SUPER_SPEED) {
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_CONNECTION);
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_ENABLE);
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
			(1 << USB_PORT_FEAT_C_CONNECTION);
		hcd = mhcd->hcd_hs_ctx->hcd;
	} else {
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_CONNECTION);
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_ENABLE);
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status |=
			(1 << USB_PORT_FEAT_C_CONNECTION);
		hcd = mhcd->hcd_ss_ctx->hcd;
	}

	spin_unlock_irqrestore(&mhcd->lock, flags);
	if (!hcd)
		return;

	usb_hcd_poll_rh_status(hcd);
}

static int mausb_hcd_get_frame_number(struct usb_hcd *hcd)
{
	return 0;
}

static int mausb_hcd_reset(struct usb_hcd *hcd)
{
	if (usb_hcd_is_primary_hcd(hcd)) {
		hcd->speed = HCD_USB2;
		hcd->self.root_hub->speed = USB_SPEED_HIGH;
	} else {
		hcd->speed = HCD_USB3;
		hcd->self.root_hub->speed = USB_SPEED_SUPER;
	}
	hcd->self.no_sg_constraint = 1;
	hcd->self.sg_tablesize = ~0;

	return 0;
}

static int mausb_hcd_start(struct usb_hcd *hcd)
{
	mausb_pr_info("");

	hcd->power_budget = 0;
	hcd->uses_new_polling = 1;
	return 0;
}

static void mausb_hcd_stop(struct usb_hcd *hcd)
{
	mausb_pr_debug("Not implemented");
}

static int mausb_hcd_hub_status(struct usb_hcd *hcd, char *buff)
{
	int retval;
	int changed;
	int i;
	struct hub_ctx *hub;
	unsigned long flags;

	hub = (struct hub_ctx *)hcd->hcd_priv;

	retval  = DIV_ROUND_UP(NUMBER_OF_PORTS + 1, 8);
	changed = 0;

	memset(buff, 0, retval);

	spin_lock_irqsave(&mhcd->lock, flags);

	if (!HCD_HW_ACCESSIBLE(hcd)) {
		mausb_pr_info("hcd not accessible, hcd speed=%d", hcd->speed);
		spin_unlock_irqrestore(&mhcd->lock, flags);
		return 0;
	}

	for (i = 0; i < NUMBER_OF_PORTS; ++i) {
		if ((hub->ma_devs[i].port_status & PORT_C_MASK)) {
			buff[(i + 1) / 8] |= 1 << (i + 1) % 8;
			changed = 1;
		}
	}

	mausb_pr_info("Usb %d.0 : changed=%d, retval=%d",
			(hcd->speed == HCD_USB2) ? 2 : 3, changed, retval);


	if ((hcd->state == HC_STATE_SUSPENDED) && (changed == 1)) {
		mausb_pr_info("hcd state is suspended");
		usb_hcd_resume_root_hub(hcd);
	}

	spin_unlock_irqrestore(&mhcd->lock, flags);

	return changed ? retval : 0;
}

static int mausb_hcd_bus_resume(struct usb_hcd *hcd)
{
	unsigned long flags;

	mausb_pr_info("");

	spin_lock_irqsave(&mhcd->lock, flags);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		mausb_pr_info("hcd not accessible, hcd speed=%d", hcd->speed);
		spin_unlock_irqrestore(&mhcd->lock, flags);
		return -ESHUTDOWN;
	}
	hcd->state = HC_STATE_RUNNING;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	return 0;
}

static int mausb_hcd_bus_suspend(struct usb_hcd *hcd)
{
	unsigned long flags;

	mausb_pr_info("");

	spin_lock_irqsave(&mhcd->lock, flags);
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	return 0;
}

static int mausb_hcd_hub_control(struct usb_hcd *hcd, uint16_t typeReq,
				uint16_t wValue, uint16_t wIndex, char *buff,
				uint16_t wLength)
{
	int retval;
	struct mausb_hcd *mhcd;
	struct hub_ctx	 *hub;
	unsigned long	 flags;
	bool invalid_rhport = false;

	hub  = (struct hub_ctx *)hcd->hcd_priv;
	mhcd = hub->mhcd;

	wIndex = ((__u8) (wIndex & 0x00ff));
	retval = 0;

	if (wIndex < 1 || wIndex > NUMBER_OF_PORTS)
		invalid_rhport = true;

	mausb_pr_info("TypeReq=%d", typeReq);

	spin_lock_irqsave(&mhcd->lock, flags);

	if (!HCD_HW_ACCESSIBLE(hcd)) {
		mausb_pr_info("hcd not accessible, hcd speed=%d", hcd->speed);
		spin_unlock_irqrestore(&mhcd->lock, flags);
		return -ETIMEDOUT;
	}

	switch (typeReq) {
	case ClearHubFeature:
		break;
	case ClearPortFeature:
		if (invalid_rhport)
			goto invalid_port;

		mausb_clear_port_feature(hcd, typeReq, wValue, wIndex, buff,
					 wLength);
		break;
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		memcpy(buff, &usb3_bos_desc, sizeof(usb3_bos_desc));
		retval = sizeof(usb3_bos_desc);
		break;
	case GetHubDescriptor:
		mausb_get_hub_descriptor(hcd, typeReq, wValue, wIndex, buff,
					 wLength);
		break;
	case GetHubStatus:
		mausb_get_hub_status(hcd, typeReq, wValue, wIndex, buff,
				     wLength);
		break;
	case GetPortStatus:
		if (invalid_rhport)
			goto invalid_port;

		mausb_get_port_status(hcd, typeReq, wValue, wIndex, buff,
				      wLength);
		break;
	case SetHubFeature:
		retval = -EPIPE;
		break;
	case SetPortFeature:
		if (invalid_rhport)
			goto invalid_port;

		mausb_set_port_feature(hcd, typeReq, wValue, wIndex, buff,
				       wLength);
		break;
	default:
		retval = -EPIPE;
	}

invalid_port:
	spin_unlock_irqrestore(&mhcd->lock, flags);
	return retval;
}

static int mausb_validate_urb(struct urb *urb)
{
	if (!urb->ep->hcpriv) {
		mausb_pr_err("urb->ep->hcpriv is NULL");
		return -EINVAL;
	}

	if (!urb->ep->enabled) {
		mausb_pr_err("Endpoint not enabled");
		return -EINVAL;
	}
	return 0;
}

static int mausb_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
				gfp_t mem_flags)
{
	struct mausb_endpoint_ctx *endpoint_ctx;
	struct mausb_device	  *ma_dev;
	struct mausb_urb_ctx	  *urb_ctx;
	int status = 0;

	if (mausb_validate_urb(urb) < 0) {
		mausb_pr_err("Hpal urb enqueue failed");
		return -EPROTO;
	}

	endpoint_ctx = urb->ep->hcpriv;
	ma_dev = endpoint_ctx->ma_dev;

	if (atomic_read(&ma_dev->unresponsive_client)) {
		mausb_pr_err("Client is not responsive anymore - finish urb immediately");
		return -EHOSTDOWN;
	}

	urb->hcpriv = hcd;

	mausb_pr_debug("ep_handle=%#x, dev_handle=%#x, urb_reject=%d",
			endpoint_ctx->ep_handle, endpoint_ctx->dev_handle,
			atomic_read(&urb->reject));

	status = mausb_insert_urb_in_tree(urb, true);
	if (status) {
		mausb_pr_err("Hpal urb enqueue failed");
		return status;
	}

	atomic_inc(&urb->use_count);

	mausb_print_urb(urb);

	/*
	 * Masking URB_SHORT_NOT_OK flag as SCSI driver is adding it where it
	 * should not, so it is breaking the USB drive on the linux
	 */
	urb->transfer_flags &= ~URB_SHORT_NOT_OK;

	status = mausb_data_req_enqueue_event(ma_dev, endpoint_ctx->ep_handle,
					      urb);

	if (status < 0) {
		urb_ctx = mausb_unlink_and_delete_urb_from_tree(urb, status);
		atomic_dec(&urb->use_count);
		if (urb_ctx) {
			mausb_uninit_data_iterator(&urb_ctx->iterator);
			kfree(urb_ctx);
		}
	}

	return status;
}

static int mausb_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
				int status)
{
	struct mausb_endpoint_ctx *endpoint_ctx;
	struct mausb_device	  *ma_dev;
	struct mausb_urb_ctx	  *urb_ctx;

	mausb_pr_info("Urb=%p", urb);

	urb_ctx = mausb_unlink_and_delete_urb_from_tree(urb, status);
	if (!urb_ctx) {
		mausb_pr_warn("Urb=%p is not in tree", urb);
		return 0;
	}

	endpoint_ctx = urb->ep->hcpriv;
	ma_dev	     = endpoint_ctx->ma_dev;

	queue_work(ma_dev->workq, &urb_ctx->work);

	return 0;
}

void mausb_hcd_urb_complete(struct urb *urb, uint32_t actual_length, int status)
{
	struct mausb_endpoint_ctx *endpoint_ctx;
	struct mausb_urb_ctx *urb_ctx =
		mausb_unlink_and_delete_urb_from_tree(urb, status);

	if (urb_ctx) {

		mausb_uninit_data_iterator(&urb_ctx->iterator);
		kfree(urb_ctx);

		endpoint_ctx	   = urb->ep->hcpriv;
		urb->status	   = status;
		urb->actual_length = actual_length;

		if (endpoint_ctx)
			mausb_pr_debug("ep_handle=%#x; urb->status=%d; urb->act_len=%d, dev_handle=%#x",
					endpoint_ctx->ep_handle, urb->status,
					urb->actual_length,
					endpoint_ctx->dev_handle);

		atomic_dec(&urb->use_count);
		usb_hcd_giveback_urb(urb->hcpriv, urb, urb->status);
		return;
	}
}

int mausb_probe(struct device *dev)
{
	struct mausb_hcd *mausb_hcd;
	struct usb_hcd	 *hcd_ss;
	struct usb_hcd	 *hcd_hs;
	int ret;

	mausb_hcd = dev_get_drvdata(dev);
	spin_lock_init(&mausb_hcd->lock);

	hcd_hs = usb_create_hcd(&mausb_hc_driver, dev, dev_name(dev));
	if (!hcd_hs) {
		mausb_pr_err("usb_create_hcd failed");
		return -ENOMEM;
	}
	hcd_hs->has_tt = 1;
	mausb_hcd->hcd_hs_ctx	    = (struct hub_ctx *)hcd_hs->hcd_priv;
	mausb_hcd->hcd_hs_ctx->mhcd = mausb_hcd;
	mausb_hcd->hcd_hs_ctx->hcd  = hcd_hs;
	memset(mausb_hcd->hcd_hs_ctx->ma_devs, 0,
	       sizeof(struct mausb_dev) * NUMBER_OF_PORTS);

	ret = usb_add_hcd(hcd_hs, 0, 0);
	if (ret) {
		mausb_pr_err("usb_add_hcd failed");
		goto put_hcd_hs;
	}

	hcd_ss = usb_create_shared_hcd(&mausb_hc_driver, dev, dev_name(dev),
				       hcd_hs);
	if (!hcd_ss) {
		mausb_pr_err("usb_create_shared_hcd failed");
		ret = -ENOMEM;
		goto remove_hcd_hs;
	}
	mausb_hcd->hcd_ss_ctx	    = (struct hub_ctx *)hcd_ss->hcd_priv;
	mausb_hcd->hcd_ss_ctx->mhcd = mausb_hcd;
	mausb_hcd->hcd_ss_ctx->hcd  = hcd_ss;

	memset(mausb_hcd->hcd_ss_ctx->ma_devs, 0,
	       sizeof(struct mausb_dev) * NUMBER_OF_PORTS);

	ret = usb_add_hcd(hcd_ss, 0, 0);
	if (ret) {
		mausb_pr_err("usb_add_hcd failed");
		goto put_hcd_ss;
	}

	return ret;

put_hcd_ss:
	usb_put_hcd(hcd_ss);
remove_hcd_hs:
	usb_remove_hcd(hcd_hs);
put_hcd_hs:
	usb_put_hcd(hcd_hs);
	mausb_hcd->hcd_hs_ctx = NULL;
	mausb_hcd->hcd_ss_ctx = NULL;
	return ret;
}

static void mausb_get_hub_descriptor(struct usb_hcd *hcd, uint16_t typeReq,
				     uint16_t wValue, uint16_t wIndex,
				     char *buff, uint16_t wLength)
{
	int width;
	struct usb_hub_descriptor *desc = (struct usb_hub_descriptor *)buff;

	memset(desc, 0, sizeof(struct usb_hub_descriptor));

	if (hcd->speed == HCD_USB3) {
		desc->bDescriptorType	   = USB_DT_SS_HUB;
		desc->bDescLength	   = 12;
		desc->wHubCharacteristics  =
		    cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
		desc->bNbrPorts		   = NUMBER_OF_PORTS;
		desc->u.ss.bHubHdrDecLat   = 0x04;
		desc->u.ss.DeviceRemovable = 0xffff;
	} else {
		desc->bDescriptorType	  = USB_DT_HUB;
		desc->wHubCharacteristics =
		    cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
		desc->bNbrPorts		  = NUMBER_OF_PORTS;
		width			  = desc->bNbrPorts / 8 + 1;
		desc->bDescLength	  = USB_DT_HUB_NONVAR_SIZE + 2 * width;

		memset(&desc->u.hs.DeviceRemovable[0], 0, width);
		memset(&desc->u.hs.DeviceRemovable[width], 0xff, width);
	}
}

static void mausb_set_port_feature(struct usb_hcd *hcd, uint16_t typeReq,
				   uint16_t wValue, uint16_t wIndex, char *buff,
				   uint16_t wLength)
{
	struct hub_ctx *hub = (struct hub_ctx *)hcd->hcd_priv;

	wIndex = ((__u8) (wIndex & 0x00ff));

	switch (wValue) {
	case USB_PORT_FEAT_LINK_STATE:
		mausb_pr_info("USB_PORT_FEAT_LINK_STATE");
		if (hcd->speed == HCD_USB3) {
			if ((hub->ma_devs[wIndex - 1].port_status &
			     USB_SS_PORT_STAT_POWER) != 0) {
				hub->ma_devs[wIndex - 1].port_status |=
				    (1 << wValue);
			}
		} else {
			if ((hub->ma_devs[wIndex - 1].port_status &
			     USB_PORT_STAT_POWER) != 0) {
				hub->ma_devs[wIndex - 1].port_status |=
				    (1 << wValue);
			}
		}
		break;
	case USB_PORT_FEAT_U1_TIMEOUT:
	case USB_PORT_FEAT_U2_TIMEOUT:
		break;
	case USB_PORT_FEAT_SUSPEND:
		break;
	case USB_PORT_FEAT_POWER:
		mausb_pr_info("USB_PORT_FEAT_POWER");

		if (hcd->speed == HCD_USB3) {
			hub->ma_devs[wIndex - 1].port_status |=
			    USB_SS_PORT_STAT_POWER;
		} else {
			hub->ma_devs[wIndex - 1].port_status |=
			    USB_PORT_STAT_POWER;
		}
		break;
	case USB_PORT_FEAT_BH_PORT_RESET:
		mausb_pr_info("USB_PORT_FEAT_BH_PORT_RESET");
		/* fall through */
	case USB_PORT_FEAT_RESET:
		mausb_pr_info("USB_PORT_FEAT_RESET hcd->speed=%d", hcd->speed);

		if (hcd->speed == HCD_USB3) {
			hub->ma_devs[wIndex - 1].port_status = 0;
			hub->ma_devs[wIndex - 1].port_status =
			    (USB_SS_PORT_STAT_POWER |
			     USB_PORT_STAT_CONNECTION | USB_PORT_STAT_RESET);
		} else if (hub->ma_devs[wIndex -
					1].port_status & USB_PORT_STAT_ENABLE) {
			hub->ma_devs[wIndex - 1].port_status &=
			    ~(USB_PORT_STAT_ENABLE |
			      USB_PORT_STAT_LOW_SPEED |
			      USB_PORT_STAT_HIGH_SPEED);
		}
		/* fall through */
	default:
		mausb_pr_info("Default wValue=%d", wValue);

		if (hcd->speed == HCD_USB3) {
			if ((hub->ma_devs[wIndex - 1].port_status &
			     USB_SS_PORT_STAT_POWER) != 0) {
				hub->ma_devs[wIndex - 1].port_status |=
				    (1 << wValue);
			}
		} else {
			if ((hub->ma_devs[wIndex - 1].port_status &
			     USB_PORT_STAT_POWER) != 0) {
				hub->ma_devs[wIndex - 1].port_status |=
				    (1 << wValue);
			}
		}
	}
}

static void mausb_get_port_status(struct usb_hcd *hcd, uint16_t typeReq,
				  uint16_t wValue, uint16_t wIndex, char *buff,
				  uint16_t wLength)
{
	uint8_t dev_speed   = 0;
	struct hub_ctx *hub = (struct hub_ctx *)hcd->hcd_priv;

	wIndex = ((__u8) (wIndex & 0x00ff));

	if ((hub->ma_devs[wIndex - 1].port_status &
				(1 << USB_PORT_FEAT_RESET)) != 0) {
		mausb_pr_info("Finished reset hcd->speed=%d", hcd->speed);

		dev_speed = hub->ma_devs[wIndex - 1].dev_speed;
		switch (dev_speed) {
		case LOW_SPEED:
			hub->ma_devs[wIndex - 1].port_status |=
			    USB_PORT_STAT_LOW_SPEED;
			break;
		case HIGH_SPEED:
			hub->ma_devs[wIndex - 1].port_status |=
			    USB_PORT_STAT_HIGH_SPEED;
			break;
		}

		hub->ma_devs[wIndex - 1].port_status |=
		    (1 << USB_PORT_FEAT_C_RESET) | USB_PORT_STAT_ENABLE;
		hub->ma_devs[wIndex - 1].port_status &=
		    ~(1 << USB_PORT_FEAT_RESET);

	}

	((__le16 *) buff)[0] =
	    cpu_to_le16(hub->ma_devs[wIndex - 1].port_status);
	((__le16 *) buff)[1] =
	    cpu_to_le16(hub->ma_devs[wIndex - 1].port_status >> 16);

	mausb_pr_info("port_status=%d", hub->ma_devs[wIndex - 1].port_status);
}

static void mausb_clear_port_feature(struct usb_hcd *hcd, uint16_t typeReq,
				     uint16_t wValue, uint16_t wIndex,
				     char *buff, uint16_t wLength)
{
	struct hub_ctx *hub = (struct hub_ctx *)hcd->hcd_priv;

	wIndex = ((__u8) (wIndex & 0x00ff));

	switch (wValue) {
	case USB_PORT_FEAT_SUSPEND:
		break;
	case USB_PORT_FEAT_POWER:
		mausb_pr_info("USB_PORT_FEAT_POWER");

		if (hcd->speed == HCD_USB3) {
			hub->ma_devs[wIndex - 1].port_status &=
			    ~USB_SS_PORT_STAT_POWER;
		} else {
			hub->ma_devs[wIndex - 1].port_status &=
			    ~USB_PORT_STAT_POWER;
		}
		break;
	case USB_PORT_FEAT_RESET:

	case USB_PORT_FEAT_C_RESET:

	default:
		mausb_pr_info("Default wValue: %d", wValue);

		hub->ma_devs[wIndex - 1].port_status &= ~(1 << wValue);
	}
}

static void mausb_get_hub_status(struct usb_hcd *hcd, uint16_t typeReq,
				 uint16_t wValue, uint16_t wIndex, char *buff,
				 uint16_t wLength)
{
	mausb_pr_info("");
	*(uint32_t *) buff = cpu_to_le32(0);
}

static int mausb_alloc_dev(struct usb_hcd *hcd, struct usb_device *dev)
{
	mausb_pr_info("Usb device=%p", dev);

	return 1;
}

static void mausb_free_dev(struct usb_hcd *hcd, struct usb_device *dev)
{
	int8_t  port_number = get_root_hub_port_number(dev);
	int32_t dev_handle;
	int	status = 0;
	unsigned long	 flags;
	struct hub_ctx   *hub  = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_dev	    *mdev = NULL;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
	struct mausb_endpoint_ctx   *endpoint_ctx = dev->ep0.hcpriv;

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return;
	}

	mdev  = &hub->ma_devs[port_number];

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return;
	}

	usb_device_ctx = mausb_find_usb_device(mdev, dev);
	if (!usb_device_ctx) {
		mausb_pr_warn("device_ctx is not found");
		return;
	}

	dev_handle = usb_device_ctx->dev_handle;

	if (atomic_read(&ma_dev->unresponsive_client)) {
		mausb_pr_err("Client is not responsive anymore - free usbdevice immediately");
		dev->ep0.hcpriv = NULL;
		kfree(endpoint_ctx);
		goto l_free_dev;
	}

	if (endpoint_ctx) {
		status = mausb_epinactivate_event_to_user(ma_dev, dev_handle,
						endpoint_ctx->ep_handle);

		mausb_pr_info("epinactivate request ep_handle=%#x, dev_handle=%#x, status=%d",
			       endpoint_ctx->ep_handle, dev_handle, status);

		status = mausb_epdelete_event_to_user(ma_dev, dev_handle,
						      endpoint_ctx->ep_handle);

		if (status < 0)
			mausb_pr_warn("ep_handle_del request failed for ep0: ep_handle=%#x, dev_handle=%#x",
					endpoint_ctx->ep_handle, dev_handle);
		dev->ep0.hcpriv = NULL;
		kfree(endpoint_ctx);

	} else {
		mausb_pr_warn("endpoint_ctx is NULL: dev_handle=%#x",
			dev_handle);
	}

	if (dev_handle != DEV_HANDLE_NOT_ASSIGNED) {
		status = mausb_usbdevdisconnect_event_to_user(ma_dev,
							      dev_handle);

		if (status < 0)
			mausb_pr_warn("usb_dev_disconnect req failed for dev_handle=%#x",
				      dev_handle);
	}

l_free_dev:
	if (atomic_sub_and_test(1, &ma_dev->num_of_usb_devices)) {
		mausb_pr_info("All usb devices destroyed - proceed with disconnecting");
		queue_work(ma_dev->workq, &ma_dev->socket_disconnect_work);
	}

	rb_erase(&usb_device_ctx->rb_node, &mdev->usb_devices);
	mausb_pr_info("USB device deleted device=%p", usb_device_ctx->dev_addr);
	kfree(usb_device_ctx);

	mausb_pr_info("kref_put");
	if (kref_put(&ma_dev->refcount, mausb_release_ma_dev_async))
		mausb_clear_hcd_madev(port_number);
}

static int mausb_device_assign_address(struct mausb_device *dev,
		struct mausb_usb_device_ctx *usb_device_ctx)
{
	int status = 0;

	status = mausb_setusbdevaddress_event_to_user(dev,
				usb_device_ctx->dev_handle, RESPONSE_TIMEOUT);

	mausb_pr_info("dev_handle=%#x, status=%d", usb_device_ctx->dev_handle,
						   status);
	usb_device_ctx->addressed = (status == 0);

	return status;
}

static struct mausb_usb_device_ctx *mausb_alloc_device_ctx(struct hub_ctx *hub,
						struct usb_device *dev,
						struct mausb_device *ma_dev,
						uint16_t port_number,
						int *status)
{
	struct mausb_usb_device_ctx *usb_device_ctx = NULL;

	mausb_pr_info("");

	usb_device_ctx = kzalloc(sizeof(*usb_device_ctx), GFP_ATOMIC);
	if (unlikely(!usb_device_ctx)) {
		mausb_pr_err("Allocation failed");
		*status = -ENOMEM;
		return NULL;
	}

	usb_device_ctx->dev_addr   = dev;
	usb_device_ctx->dev_handle = DEV_HANDLE_NOT_ASSIGNED;
	usb_device_ctx->addressed  = false;

	if (mausb_insert_usb_device(&hub->ma_devs[port_number],
				    usb_device_ctx)) {
		mausb_pr_warn("device_ctx already exists");
		kfree(usb_device_ctx);
		*status = -EEXIST;
		return NULL;
	}

	kref_get(&ma_dev->refcount);
	mausb_pr_info("New USB device added device=%p",
		      usb_device_ctx->dev_addr);
	atomic_inc(&ma_dev->num_of_usb_devices);

	return usb_device_ctx;
}

/*
 * For usb 2.0 logitech camera called multiple times, once during
 * enumeration of device and later after mausb_reset_device.
 */
static int mausb_address_device(struct usb_hcd *hcd, struct usb_device *dev)
{
	int8_t	port_number = 0;
	int	status	    = 0;
	unsigned long	flags;
	struct hub_ctx	*hub = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
	struct mausb_endpoint_ctx   *endpoint_ctx;

	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return -EINVAL;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx =
	    mausb_find_usb_device(&hub->ma_devs[port_number], dev);

	if (!usb_device_ctx) {
		usb_device_ctx = mausb_alloc_device_ctx(hub, dev, ma_dev,
							port_number, &status);

		if (!usb_device_ctx)
			return status;
	}

	mausb_pr_info("dev_handle=%#x, dev_speed=%#x",
		usb_device_ctx->dev_handle, dev->speed);

	if (dev->speed >= USB_SPEED_SUPER)
		mausb_pr_info("USB 3.0");
	else
		mausb_pr_info("USB 2.0");


	if (usb_device_ctx->dev_handle == DEV_HANDLE_NOT_ASSIGNED) {
		status = mausb_enable_device(hcd, dev);
		if (status < 0)
			return status;
	}

	if (!usb_device_ctx->addressed) {
		status = mausb_device_assign_address(ma_dev, usb_device_ctx);
		if (status < 0)
			return status;
	}

	endpoint_ctx = dev->ep0.hcpriv;
	if (!endpoint_ctx) {
		mausb_pr_err("endpoint_ctx is NULL: dev_handle=%#x",
			     usb_device_ctx->dev_handle);
		return -EINVAL;
	}

	/*
	 * Fix to support usb 2.0 logitech camera. If endoint handle of usb 2.0
	 * device is already modified, do not modify it again.
	 */
	if (dev->speed < USB_SPEED_SUPER && endpoint_ctx->ep_handle != 0)
		return 0;

	status = mausb_modifyep0_event_to_user(ma_dev,
					       usb_device_ctx->dev_handle,
					       &endpoint_ctx->ep_handle,
					       dev->ep0.desc.wMaxPacketSize);

	mausb_pr_info("modify ep0 response, ep_handle=%#x, dev_handle=%#x, status=%d",
		       endpoint_ctx->ep_handle, endpoint_ctx->dev_handle,
		       status);

	return status;
}

static int mausb_add_endpoint(struct usb_hcd *hcd, struct usb_device *dev,
			struct usb_host_endpoint *endpoint)
{
	int    status	   = 0;
	int8_t port_number = 0;
	struct ma_usb_ephandlereq_desc_ss  descriptor_ss;
	struct ma_usb_ephandlereq_desc_std descriptor;
	struct hub_ctx		    *hub = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
	struct mausb_endpoint_ctx   *endpoint_ctx;
	unsigned long flags;

	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return 0;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx =
		mausb_find_usb_device(&hub->ma_devs[port_number], dev);

	if (!usb_device_ctx) {
		mausb_pr_warn("Device not found");
		return -ENODEV;
	}

	endpoint_ctx = kzalloc(sizeof(*endpoint_ctx), GFP_ATOMIC);
	if (unlikely(!endpoint_ctx)) {
		mausb_pr_alert("Failed to alloc endpoint_ctx");
		return -ENOMEM;
	}

	endpoint_ctx->dev_handle	= usb_device_ctx->dev_handle;
	endpoint_ctx->usb_device_ctx	= usb_device_ctx;
	endpoint_ctx->ma_dev		= ma_dev;
	endpoint->hcpriv		= endpoint_ctx;

	if (dev->speed >= USB_SPEED_SUPER) {
		mausb_init_superspeed_ep_descriptor(&descriptor_ss,
						    &endpoint->desc,
						    &endpoint->ss_ep_comp);
		status = mausb_ephandle_event_to_user(ma_dev,
					usb_device_ctx->dev_handle,
					sizeof(descriptor_ss),
					&descriptor_ss,
					&endpoint_ctx->ep_handle);

	} else {
		mausb_init_standard_ep_descriptor(&descriptor, &endpoint->desc);
		status = mausb_ephandle_event_to_user(ma_dev,
					usb_device_ctx->dev_handle,
					sizeof(descriptor),
					&descriptor,
					&endpoint_ctx->ep_handle);
	}

	if (status < 0) {
		mausb_pr_err("ep_handle_request failed dev_handle=%#x",
			     usb_device_ctx->dev_handle);
		endpoint->hcpriv = NULL;
		kfree(endpoint_ctx);
		return status;
	}

	mausb_pr_info("Endpoint added ep_handle=%#x, dev_handle=%#x",
		      endpoint_ctx->ep_handle, endpoint_ctx->dev_handle);

	return 0;
}

static int mausb_drop_endpoint(struct usb_hcd *hcd, struct usb_device *dev,
			struct usb_host_endpoint *endpoint)
{
	int8_t	port_number = 0;
	int	status	    = 0,
		retries	    = 0;
	struct hub_ctx		    *hub = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
	struct mausb_endpoint_ctx   *endpoint_ctx = endpoint->hcpriv;
	unsigned long flags;

	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return -EINVAL;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx =
	    mausb_find_usb_device(&hub->ma_devs[port_number], dev);

	if (!endpoint_ctx) {
		mausb_pr_err("Endpoint context doesn't exist");
		return 0;
	}
	if (!usb_device_ctx) {
		mausb_pr_err("Usb device context doesn't exist");
		return 0;
	}

	mausb_pr_info("Start dropping ep_handle=%#x, dev_handle=%#x",
		      endpoint_ctx->ep_handle, endpoint_ctx->dev_handle);

	if (atomic_read(&ma_dev->unresponsive_client)) {
		mausb_pr_err("Client is not responsive anymore - drop endpoint immediately");
		endpoint->hcpriv = NULL;
		kfree(endpoint_ctx);
		return status;
	}

	status = mausb_epinactivate_event_to_user(ma_dev,
						  usb_device_ctx->dev_handle,
						  endpoint_ctx->ep_handle);

	mausb_pr_info("epinactivate request ep_handle=%#x, dev_handle=%#x, status=%d",
		       endpoint_ctx->ep_handle, endpoint_ctx->dev_handle,
		       status);

	while (true) {
		status = mausb_epdelete_event_to_user(ma_dev,
						usb_device_ctx->dev_handle,
						endpoint_ctx->ep_handle);

		mausb_pr_info("ep_handle_delete_request, ep_handle=%#x, dev_handle=%#x, status=%d",
			      endpoint_ctx->ep_handle, endpoint_ctx->dev_handle,
			      status);
		/* sleep for 10 ms to give device some time */
		if (status == -EBUSY) {
			if (++retries < MAUSB_BUSY_RETRIES_COUNT)
				usleep_range(MAUSB_BUSY_TIMEOUT_MIN,
					MAUSB_BUSY_TIMEOUT_MAX);
			else
				return status;
		} else {
			break;
		}
	}

	mausb_pr_info("Endpoint dropped ep_handle=%#x, dev_handle=%#x",
		      endpoint_ctx->ep_handle, endpoint_ctx->dev_handle);

	endpoint->hcpriv = NULL;
	kfree(endpoint_ctx);
	return status;
}

static int mausb_device_assign_dev_handle(struct usb_hcd *hcd, struct usb_device
				*dev,
				struct hub_ctx *hub,
				struct mausb_device *ma_dev,
				struct mausb_usb_device_ctx *usb_device_ctx)
{
	int8_t port_number = get_root_hub_port_number(dev);
	int status	   = 0;
	int dev_speed	   = 0;
	uint16_t hub_dev_handle		    = 0;
	uint16_t parent_hs_hub_dev_handle   = 0;
	uint16_t parent_hs_hub_port	    = 0;
	struct usb_device *first_hub_device = dev;
	struct mausb_usb_device_ctx	   *hub_device_ctx;
	struct mausb_endpoint_ctx	   *endpoint_ctx;
	struct ma_usb_ephandlereq_desc_std descriptor;

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return -EINVAL;
	}

	while (first_hub_device->parent->parent)
		first_hub_device = first_hub_device->parent;

	hub_device_ctx = mausb_find_usb_device(&hub->ma_devs[port_number],
					       first_hub_device);

	if (hub_device_ctx)
		hub_dev_handle = hub_device_ctx->dev_handle;

	if ((dev->speed == USB_SPEED_LOW || dev->speed == USB_SPEED_FULL)
	    && first_hub_device->speed == USB_SPEED_HIGH) {
		parent_hs_hub_dev_handle =
			mausb_find_usb_device(&hub->ma_devs[port_number],
					      dev->parent)->dev_handle;
		parent_hs_hub_port = dev->parent->portnum;
	}

	dev_speed = usb_to_mausb_device_speed(dev->speed);
	mausb_pr_info("start... mausb_devspeed=%d, route=%#x, port_number=%d",
		      dev_speed, dev->route, port_number);

	if (unlikely(dev_speed == -EINVAL)) {
		mausb_pr_err("bad dev_speed");
		return -EINVAL;
	}

	status = mausb_usbdevhandle_event_to_user(ma_dev, (uint8_t) dev_speed,
						  dev->route, hub_dev_handle,
						  parent_hs_hub_dev_handle,
						  parent_hs_hub_port, 0,
						  ma_dev->lse,
						  &usb_device_ctx->dev_handle);

	mausb_pr_info("mausb_usbdevhandle_event status=%d", status);

	if (status < 0)
		return status;

	mausb_pr_info("Get ref dev_handle=%#x", usb_device_ctx->dev_handle);

	endpoint_ctx = kzalloc(sizeof(*endpoint_ctx), GFP_ATOMIC);
	if (unlikely(!endpoint_ctx)) {
		mausb_pr_alert("Failed to allocate endpoint_ctx");
		return -ENOMEM;
	}

	endpoint_ctx->dev_handle     = usb_device_ctx->dev_handle;
	endpoint_ctx->ma_dev	     = ma_dev;
	endpoint_ctx->usb_device_ctx = usb_device_ctx;
	dev->ep0.hcpriv		     = endpoint_ctx;

	mausb_init_standard_ep_descriptor(&descriptor, &dev->ep0.desc);

	status = mausb_ephandle_event_to_user(ma_dev,
					      usb_device_ctx->dev_handle,
					      sizeof(descriptor),
					      &descriptor,
					      &endpoint_ctx->ep_handle);

	mausb_pr_info("mausb_ephandle_event ep_handle=%#x, dev_handle=%#x, status=%d",
		      endpoint_ctx->ep_handle, endpoint_ctx->dev_handle,
		      status);

	if (status < 0) {
		dev->ep0.hcpriv = NULL;
		kfree(endpoint_ctx);
		return status;
	}

	return 0;
}

/*
 * For usb 2.0 logitech camera called multiple times, once during enumeration
 * of device and later after mausb_reset_device. In latter case it is
 * required to address the device again in order for ep0 to work properly.
 */
static int mausb_enable_device(struct usb_hcd *hcd, struct usb_device *dev)
{
	int8_t port_number;
	struct hub_ctx		    *hub = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
	unsigned long flags;
	int status = 0;

	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return -EINVAL;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx =
	    mausb_find_usb_device(&hub->ma_devs[port_number], dev);

	if (!usb_device_ctx) {
		usb_device_ctx = mausb_alloc_device_ctx(hub, dev, ma_dev,
							port_number, &status);

		if (!usb_device_ctx)
			return status;
	}

	mausb_pr_info("Device assigned and addressed usb_device_ctx=%p",
		      usb_device_ctx);

	if (usb_device_ctx->dev_handle == DEV_HANDLE_NOT_ASSIGNED)
		return mausb_device_assign_dev_handle(hcd, dev, hub, ma_dev,
						      usb_device_ctx);

	/*
	 * Fix for usb 2.0 logitech camera
	 */
	if (!usb_device_ctx->addressed)
		return mausb_device_assign_address(ma_dev, usb_device_ctx);

	mausb_pr_info("Device assigned and addressed dev_handle=%#x",
		      usb_device_ctx->dev_handle);
	return 0;
}

static int mausb_is_hub_device(struct usb_device *dev)
{
	return dev->descriptor.bDeviceClass == 0x09;
}

static int mausb_update_device(struct usb_hcd *hcd, struct usb_device *dev)
{
	int8_t	port_number = 0;
	int	status	    = 0;
	struct hub_ctx		    *hub = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_device	    *ma_dev = NULL;
	struct mausb_usb_device_ctx *usb_device_ctx = NULL;
	unsigned long flags;

	if (mausb_is_hub_device(dev)) {
		mausb_pr_warn("Device is hub");
		return 0;
	}

	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return -EINVAL;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx =
	    mausb_find_usb_device(&hub->ma_devs[port_number], dev);
	if (!usb_device_ctx) {
		mausb_pr_warn("Device not found");
		return -ENODEV;
	}

	status = mausb_updatedev_event_to_user(ma_dev,
					       usb_device_ctx->dev_handle,
					       0, 0, 0, 0, 0, 0,
					       &dev->descriptor);

	mausb_pr_info("Finished dev_handle=%#x, status=%d",
			usb_device_ctx->dev_handle, status);

	return status;
}

static int mausb_hub_update_device(struct usb_hcd *hcd, struct usb_device *dev,
				struct usb_tt *tt, gfp_t mem_flags)
{
	int8_t	port_number = 0;
	int	status	    = 0;
	unsigned long flags;
	uint16_t max_exit_latency = 0;
	uint8_t  number_of_ports = dev->maxchild;
	uint8_t  mtt = 0;
	uint8_t  ttt = 0;
	uint8_t  integrated_hub_latency = 0;
	struct hub_ctx		    *hub = (struct hub_ctx *)hcd->hcd_priv;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;

	if (dev->speed == USB_SPEED_HIGH) {
		mtt = tt->multi == 0 ? 1 : 0;
		ttt = tt->think_time;
	}
	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return 0;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx = mausb_find_usb_device(&hub->ma_devs[port_number],
					       dev);

	if (!usb_device_ctx) {
		mausb_pr_err("USB device not found");
		return -ENODEV;
	}

#if KERNEL_VERSION(4, 3, 5) <= LINUX_VERSION_CODE
	if (dev->usb3_lpm_u1_enabled)
		max_exit_latency = dev->u1_params.mel;
	else if (dev->usb3_lpm_u2_enabled)
		max_exit_latency = dev->u2_params.mel;
#endif

	status = mausb_updatedev_event_to_user(ma_dev,
					       usb_device_ctx->dev_handle,
					       max_exit_latency, 1,
					       number_of_ports, mtt, ttt,
					       integrated_hub_latency,
					       &dev->descriptor);

	mausb_pr_info("Finished dev_handle=%#x, status=%d",
			usb_device_ctx->dev_handle, status);

	return status;
}

static int mausb_check_bandwidth(struct usb_hcd *hcd, struct usb_device *dev)
{
	mausb_pr_debug("Not implemented");
	return 0;
}

static void mausb_reset_bandwidth(struct usb_hcd *hcd, struct usb_device *dev)
{
	mausb_pr_debug("Not implemented");
}

static void mausb_endpoint_disable(struct usb_hcd *hcd,
				struct usb_host_endpoint *endpoint)
{
	mausb_pr_debug("Not implemented");
}

static void mausb_endpoint_reset(struct usb_hcd *hcd,
				struct usb_host_endpoint *endpoint)
{
	int status = 0;
	int is_control,
	    epnum,
	    is_out;
	unsigned long flags;
	uint16_t dev_handle;
	uint8_t	 tsp;
	int8_t	 port_number;
	struct hub_ctx		    *hub;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
	struct usb_device	    *dev;
	struct mausb_endpoint_ctx   *endpoint_ctx;
	struct ma_usb_ephandlereq_desc_ss  descriptor_ss;
	struct ma_usb_ephandlereq_desc_std descriptor;

	endpoint_ctx = endpoint->hcpriv;
	if (!endpoint_ctx) {
		mausb_pr_err("ep->hcpriv is NULL");
		return;
	}

	usb_device_ctx	= endpoint_ctx->usb_device_ctx;
	dev_handle	= usb_device_ctx->dev_handle;
	dev		= usb_device_ctx->dev_addr;

	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return;
	}
	hub = (struct hub_ctx *)hcd->hcd_priv;

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return;
	}

	is_control = usb_endpoint_xfer_control(&endpoint->desc);
	epnum = usb_endpoint_num(&endpoint->desc);
	is_out = usb_endpoint_dir_out(&endpoint->desc);
	tsp = is_out ? dev->toggle[1] : dev->toggle[0];

	status = mausb_epreset_event_to_user(ma_dev, dev_handle,
					     endpoint_ctx->ep_handle, tsp);

	mausb_pr_info("ep_reset status=%d, ep_handle=%#x, dev_handle=%#x",
		      status, endpoint_ctx->ep_handle, dev_handle);

	if (status < 0)
		return;

	if (status != EUCLEAN) {
		if (!tsp) {
			usb_settoggle(dev, epnum, is_out, 0);
			if (is_control)
				usb_settoggle(dev, epnum, !is_out, 0);
		}

		status = mausb_epactivate_event_to_user(ma_dev, dev_handle,
						endpoint_ctx->ep_handle);

		mausb_pr_err("ep_activate failed, status=%d, ep_handle=%#x, dev_handle=%#x",
			     status, endpoint_ctx->ep_handle, dev_handle);

		return;
	}

	if (tsp)
		return;

	status = mausb_epinactivate_event_to_user(ma_dev, dev_handle,
						  endpoint_ctx->ep_handle);

	mausb_pr_info("ep_inactivate status=%d, ep_handle=%#x, dev_handle=%#x",
		      status, endpoint_ctx->ep_handle, dev_handle);

	if (status < 0)
		return;

	status = mausb_epdelete_event_to_user(ma_dev, dev_handle,
					      endpoint_ctx->ep_handle);

	mausb_pr_info("ep_handle_delete status=%d, ep_handle=%#x, dev_handle=%#x",
		      status, endpoint_ctx->ep_handle, dev_handle);

	if (status < 0)
		return;

	if (dev->speed >= USB_SPEED_SUPER) {
		mausb_init_superspeed_ep_descriptor(&descriptor_ss,
						    &endpoint->desc,
						    &endpoint->ss_ep_comp);
		status = mausb_ephandle_event_to_user(ma_dev, dev_handle,
						      sizeof(descriptor_ss),
						      &descriptor_ss,
						      &endpoint_ctx->ep_handle);
	} else {
		mausb_init_standard_ep_descriptor(&descriptor, &endpoint->desc);
		status = mausb_ephandle_event_to_user(ma_dev, dev_handle,
						      sizeof(descriptor),
						      &descriptor,
						      &endpoint_ctx->ep_handle);
	}

	mausb_pr_info("ep_handle request status=%d, ep_handle=%#x, dev_handle=%#x",
			status, endpoint_ctx->ep_handle, dev_handle);
}

/*
 * For usb 2.0 logitech camera called multiple times,
 * followed by either mausb_enable_device or mausb_address_device.
 * Resets device to non-addressed state.
 */
static int mausb_reset_device(struct usb_hcd *hcd, struct usb_device *dev)
{
	int8_t	 port_number = 0;
	uint16_t dev_handle;
	int status = 0;
	unsigned long flags;
	struct hub_ctx		    *hub;
	struct mausb_device	    *ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;

	hub = (struct hub_ctx *)hcd->hcd_priv;
	port_number = get_root_hub_port_number(dev);

	if (port_number < 0 || port_number >= NUMBER_OF_PORTS) {
		mausb_pr_info("port_number out of range, port_number=%x",
			      port_number);
		return -EINVAL;
	}

	spin_lock_irqsave(&mhcd->lock, flags);
	ma_dev = hub->ma_devs[port_number].ma_dev;
	spin_unlock_irqrestore(&mhcd->lock, flags);

	if (!ma_dev) {
		mausb_pr_err("MAUSB device not found on port_number=%d",
			     port_number);
		return -ENODEV;
	}

	usb_device_ctx =
	    mausb_find_usb_device(&hub->ma_devs[port_number], dev);

	if (!usb_device_ctx ||
		usb_device_ctx->dev_handle == DEV_HANDLE_NOT_ASSIGNED)
		return 0;

	dev_handle = usb_device_ctx->dev_handle;

	status = mausb_usbdevreset_event_to_user(ma_dev, dev_handle);

	mausb_pr_info("usb_dev_reset dev_handle=%#x, status=%d",
		      dev_handle, status);

	if (status == 0)
		usb_device_ctx->addressed = false;

	return status;
}

#ifdef ISOCH_CALLBACKS
int mausb_sec_event_ring_setup(struct usb_hcd *hcd, unsigned int intr_num)
{
	mausb_pr_debug("");
	return 0;
}

int mausb_sec_event_ring_cleanup(struct usb_hcd *hcd, unsigned int intr_num)
{
	mausb_pr_debug("");
	return 0;
}

phys_addr_t mausb_get_sec_event_ring_phys_addr(struct usb_hcd *hcd,
					       unsigned int intr_num,
					       dma_addr_t *dma)
{
	mausb_pr_debug("");
	return 0;
}

phys_addr_t mausb_get_xfer_ring_phys_addr(struct usb_hcd *hcd,
					  struct usb_device *udev,
					  struct usb_host_endpoint *ep,
					  dma_addr_t *dma)
{
	mausb_pr_debug("");
	return 0;
}

int mausb_get_core_id(struct usb_hcd *hcd)
{
	mausb_pr_debug("");
	return 0;
}
#endif /* ISOCH_CALLBACKS */

void mausb_clear_hcd_madev(uint16_t port_number)
{
	unsigned long flags = 0;

	if (port_number >= NUMBER_OF_PORTS) {
		mausb_pr_err("port_number out of range, port_number=%x",
			     port_number);
		return;
	}

	spin_lock_irqsave(&mhcd->lock, flags);

	memset(&mhcd->hcd_hs_ctx->ma_devs[port_number], 0,
	       sizeof(struct mausb_dev));
	memset(&mhcd->hcd_ss_ctx->ma_devs[port_number], 0,
	       sizeof(struct mausb_dev));

	mhcd->connected_ports &= ~(1 << port_number);

	mhcd->hcd_hs_ctx->ma_devs[port_number].port_status =
							USB_PORT_STAT_POWER;
	mhcd->hcd_ss_ctx->ma_devs[port_number].port_status =
							USB_SS_PORT_STAT_POWER;

	spin_unlock_irqrestore(&mhcd->lock, flags);
}
