/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_COMMON_MAUSB_EVENT_H__
#define __MAUSB_COMMON_MAUSB_EVENT_H__

#include <common/ma_usb.h>

#define MAUSB_MAX_NUM_OF_MA_DEVS			15
#define MAUSB_RING_BUFFER_SIZE				1024
#define MAUSB_MAX_DATA_IN_REQ_SIZE			28

#define MAUSB_EVENT_TYPE_DEV_RESET			1u
#define MAUSB_EVENT_TYPE_USB_DEV_HANDLE			2u
#define MAUSB_EVENT_TYPE_EP_HANDLE			3u
#define MAUSB_EVENT_TYPE_EP_HANDLE_ACTIVATE		4u
#define MAUSB_EVENT_TYPE_EP_HANDLE_INACTIVATE		5u
#define MAUSB_EVENT_TYPE_EP_HANDLE_RESET		6u
#define MAUSB_EVENT_TYPE_EP_HANDLE_DELETE		7u
#define MAUSB_EVENT_TYPE_MODIFY_EP0			8u
#define MAUSB_EVENT_TYPE_SET_USB_DEV_ADDRESS		9u
#define MAUSB_EVENT_TYPE_UPDATE_DEV			10u
#define MAUSB_EVENT_TYPE_USB_DEV_DISCONNECT		11u
#define MAUSB_EVENT_TYPE_PING				12u
#define MAUSB_EVENT_TYPE_DEV_DISCONNECT			13u
#define MAUSB_EVENT_TYPE_USB_DEV_RESET			14u
#define MAUSB_EVENT_TYPE_CANCEL_TRANSFER		15u

#define MAUSB_EVENT_TYPE_PORT_CHANGED			80u
#define MAUSB_EVENT_TYPE_SEND_MGMT_MSG			81u
#define MAUSB_EVENT_TYPE_SEND_DATA_MSG			82u
#define MAUSB_EVENT_TYPE_RECEIVED_MGMT_MSG		83u
#define MAUSB_EVENT_TYPE_RECEIVED_DATA_MSG		84u
#define MAUSB_EVENT_TYPE_URB_COMPLETE			85u
#define MAUSB_EVENT_TYPE_SEND_ACK			86u
#define MAUSB_EVENT_TYPE_ITERATOR_RESET			87u
#define MAUSB_EVENT_TYPE_ITERATOR_SEEK			88u
#define MAUSB_EVENT_TYPE_DELETE_DATA_TRANSFER		89u
#define MAUSB_EVENT_TYPE_DELETE_MA_DEV			90u
#define MAUSB_EVENT_TYPE_USER_FINISHED			91u
#define MAUSB_EVENT_TYPE_RELEASE_EVENT_RESOURCES	92u
#define MAUSB_EVENT_TYPE_NETWORK_DISCONNECTED		93u
#define MAUSB_EVENT_TYPE_MGMT_REQUEST_TIMED_OUT		94u

#define MAUSB_EVENT_TYPE_NONE				255u

#define MAUSB_DATA_MSG_DIRECTION_OUT			0
#define MAUSB_DATA_MSG_DIRECTION_IN			1
#define MAUSB_DATA_MSG_CONTROL MAUSB_DATA_MSG_DIRECTION_OUT

struct mausb_devhandle {
	uint64_t event_id;
	uint32_t route_string;
	uint16_t hub_dev_handle;
	uint16_t parent_hs_hub_dev_handle;
	uint16_t parent_hs_hub_port;
	uint16_t mtt;
	/* dev_handle assigned in user */
	uint16_t dev_handle;
	uint8_t  device_speed;
	uint8_t  lse;
};

struct mausb_ephandle {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t descriptor_size;
	/* ep_handle assigned in user */
	uint16_t ep_handle;
	char	 descriptor[sizeof(struct ma_usb_ephandlereq_desc_ss)];
};

struct mausb_epactivate {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t ep_handle;
};

struct mausb_epinactivate {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t ep_handle;
};

struct mausb_epreset {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t ep_handle;
	uint8_t  tsp;
};

struct mausb_epdelete {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t ep_handle;
};

struct mausb_updatedev {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t max_exit_latency;
	struct ma_usb_updatedevreq_desc update_descriptor;
	uint8_t  hub;
	uint8_t  number_of_ports;
	uint8_t  mtt;
	uint8_t  ttt;
	uint8_t  integrated_hub_latency;
};

struct mausb_usbdevreset {
	uint64_t event_id;
	uint16_t device_handle;
};

struct mausb_modifyep0 {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t ep_handle;
	uint16_t max_packet_size;
};

struct mausb_setusbdevaddress {
	uint64_t event_id;
	uint16_t device_handle;
	uint16_t response_timeout;
};

struct mausb_usbdevdisconnect {
	uint16_t device_handle;
};

struct mausb_canceltransfer {
	uint64_t urb;
	uint16_t device_handle;
	uint16_t ep_handle;
};

struct mausb_mgmt_hdr {
	__aligned(4) char hdr[MAUSB_MAX_MGMT_SIZE];
};

struct mausb_mgmt_req_timedout {
	uint64_t event_id;
};

struct mausb_delete_ma_dev {
	uint64_t event_id;
	uint16_t device_id;
};

/* TODO split mgmt_event to generic send mgmt req and specific requests */
struct mausb_mgmt_event {
	union {
		struct mausb_devhandle		dev_handle;
		struct mausb_ephandle		ep_handle;
		struct mausb_epactivate		ep_activate;
		struct mausb_epinactivate	ep_inactivate;
		struct mausb_epreset		ep_reset;
		struct mausb_epdelete		ep_delete;
		struct mausb_modifyep0		modify_ep0;
		struct mausb_setusbdevaddress	set_usb_dev_address;
		struct mausb_updatedev		update_dev;
		struct mausb_usbdevreset	usb_dev_reset;
		struct mausb_usbdevdisconnect	usb_dev_disconnect;
		struct mausb_canceltransfer	cancel_transfer;
		struct mausb_mgmt_hdr		mgmt_hdr;
		struct mausb_mgmt_req_timedout	mgmt_req_timedout;
		struct mausb_delete_ma_dev	delete_ma_dev;
	};
};

struct mausb_data_event {
	uint64_t urb;
	uint64_t recv_buf;
	uint32_t iterator_seek_delta;
	uint32_t transfer_size;
	uint32_t rem_transfer_size;
	uint32_t transfer_flags;
	uint32_t isoch_seg_num;
	uint32_t req_id;
	uint32_t payload_size;
	int32_t  status;

	__aligned(4) char hdr[MAUSB_TRANSFER_HDR_SIZE];
	__aligned(4) char hdr_ack[MAUSB_TRANSFER_HDR_SIZE];

	uint16_t device_id;
	uint16_t ep_handle;
	uint16_t packet_size;
	uint8_t  setup_packet;
	uint8_t  direction;
	uint8_t  transfer_type;
	uint8_t  first_control_packet;
	uint8_t  transfer_eot;
	uint8_t  mausb_address;
	uint8_t  mausb_ssid;
};

struct mausb_port_changed_event {
	uint8_t port;
	uint8_t dev_type;
	uint8_t dev_speed;
	uint8_t lse;
};

struct mausb_event {
	union {
		struct mausb_mgmt_event		mgmt;
		struct mausb_data_event		data;
		struct mausb_port_changed_event port_changed;
	};
	int32_t status;
	uint8_t type;
	uint8_t madev_addr;
};

struct mausb_events_notification {
	uint16_t num_of_events;
	uint16_t num_of_completed_events;
	uint8_t  madev_addr;
};

#endif /* __MAUSB_COMMON_MAUSB_EVENT_H__ */
