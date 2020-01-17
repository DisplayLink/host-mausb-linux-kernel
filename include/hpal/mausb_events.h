/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HPAL_MAUSB_EVENTS_H__
#define __MAUSB_HPAL_MAUSB_EVENTS_H__

#include "common/mausb_event.h"
#include "hpal/hpal.h"
#include "link/mausb_ip_link.h"

#define MANAGEMENT_EVENT_TIMEOUT 3000

int mausb_msg_received_event(struct mausb_event *event,
			     struct ma_usb_hdr_common *hdr,
			     enum mausb_channel channel);
int mausb_usbdevhandle_event_to_user(struct mausb_device *dev,
				     uint8_t device_speed,
				     uint32_t route_string,
				     uint16_t hub_dev_handle,
				     uint16_t parent_hs_hub_dev_handle,
				     uint16_t parent_hs_hub_port, uint16_t mtt,
				     uint8_t lse, int32_t *usb_dev_handle);
int mausb_usbdevhandle_event_from_user(struct mausb_device *dev,
				       struct mausb_event *event);
void mausb_init_standard_ep_descriptor(
				struct ma_usb_ephandlereq_desc_std *std_desc,
				struct usb_endpoint_descriptor *usb_std_desc);
void mausb_init_superspeed_ep_descriptor(
				struct ma_usb_ephandlereq_desc_ss *ss_desc,
				struct usb_endpoint_descriptor *usb_std_desc,
				struct usb_ss_ep_comp_descriptor *usb_ss_desc);
int mausb_ephandle_event_to_user(struct mausb_device *dev,
				 uint16_t device_handle,
				 uint16_t descriptor_size, void *descriptor,
				 uint16_t *ep_handle);
int mausb_ephandle_event_from_user(struct mausb_device *dev,
				   struct mausb_event *event);
int mausb_epactivate_event_to_user(struct mausb_device *dev,
				   uint16_t device_handle, uint16_t ep_handle);
int mausb_epactivate_event_from_user(struct mausb_device *dev,
				     struct mausb_event *event);
int mausb_epinactivate_event_to_user(struct mausb_device *dev,
				     uint16_t device_handle,
				     uint16_t ep_handle);
int mausb_epinactivate_event_from_user(struct mausb_device *dev,
				       struct mausb_event *event);
int mausb_epreset_event_to_user(struct mausb_device *dev,
				uint16_t device_handle, uint16_t ep_handle,
				uint8_t tsp_flag);
int mausb_epreset_event_from_user(struct mausb_device *dev,
				  struct mausb_event *event);
int mausb_epdelete_event_to_user(struct mausb_device *dev,
				 uint16_t device_handle, uint16_t ep_handle);
int mausb_epdelete_event_from_user(struct mausb_device *dev,
				   struct mausb_event *event);
int mausb_modifyep0_event_to_user(struct mausb_device *dev,
				  uint16_t device_handle, uint16_t *ep_handle,
				  uint16_t max_packet_size);
int mausb_modifyep0_event_from_user(struct mausb_device *dev,
				    struct mausb_event *event);
int mausb_setusbdevaddress_event_to_user(struct mausb_device *dev,
					 uint16_t device_handle,
					 uint16_t response_timeout);
int mausb_setusbdevaddress_event_from_user(struct mausb_device *dev,
					   struct mausb_event *event);
int mausb_updatedev_event_to_user(struct mausb_device *dev,
				  uint16_t device_handle,
				  uint16_t max_exit_latency, uint8_t hub,
				  uint8_t number_of_ports, uint8_t mtt,
				  uint8_t ttt, uint8_t integrated_hub_latency,
				  struct usb_device_descriptor *dev_descriptor);
int mausb_updatedev_event_from_user(struct mausb_device *dev,
				    struct mausb_event *event);
int mausb_usbdevdisconnect_event_to_user(struct mausb_device *dev,
					 uint16_t dev_handle);
int mausb_ping_event_to_user(struct mausb_device *dev);
int mausb_usbdevreset_event_to_user(struct mausb_device *dev,
				    uint16_t device_handle);
int mausb_usbdevreset_event_from_user(struct mausb_device *dev,
				      struct mausb_event *event);
int mausb_canceltransfer_event_to_user(struct mausb_device *dev,
				       uint16_t device_handle,
				       uint16_t ep_handle, uint64_t urb);
int mausb_canceltransfer_event_from_user(struct mausb_device *dev,
					 struct mausb_event *event);

void mausb_dev_reset_req_event(struct mausb_event *event);
void mausb_cleanup_send_data_msg_event(struct mausb_event *event);
void mausb_cleanup_received_data_msg_event(struct mausb_event *event);
void mausb_cleanup_delete_data_transfer_event(struct mausb_event *event);

#endif /* __MAUSB_HPAL_MAUSB_EVENTS_H__ */
