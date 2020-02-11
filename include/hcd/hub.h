/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HCD_HUB_H__
#define __MAUSB_HCD_HUB_H__

#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "utils/mausb_data_iterator.h"

#define PORT_C_MASK \
		((USB_PORT_STAT_C_CONNECTION \
		| USB_PORT_STAT_C_ENABLE \
		| USB_PORT_STAT_C_SUSPEND \
		| USB_PORT_STAT_C_OVERCURRENT \
		| USB_PORT_STAT_C_RESET) << 16)

#define MAUSB_PORT_20_STATUS_CONNECT         0x0001
#define MAUSB_PORT_20_STATUS_ENABLE          0x0002
#define MAUSB_PORT_20_STATUS_SUSPEND         0x0004
#define MAUSB_PORT_20_STATUS_OVER_CURRENT    0x0008
#define MAUSB_PORT_20_STATUS_RESET           0x0010
#define MAUSB_PORT_20_STATUS_POWER           0x0100
#define MAUSB_PORT_20_STATUS_LOW_SPEED       0x0200
#define MAUSB_PORT_20_STATUS_HIGH_SPEED      0x0400

#define MAUSB_CHANGE_PORT_20_STATUS_CONNECT  0x010000
#define MAUSB_CHANGE_PORT_20_STATUS_RESET    0x100000

/* USB 3.2 specification chapter 10.16.2.6.1 table 10-13 page 440 */
#define MAUSB_PORT_30_STATUS_CONNECT              0x0001
#define MAUSB_PORT_30_STATUS_ENABLE               0x0002
#define MAUSB_PORT_30_STATUS_OVER_CURRENT         0x0008
#define MAUSB_PORT_30_STATUS_RESET                0x0010
#define MAUSB_PORT_30_LINK_STATE_U0               0x0000
#define MAUSB_PORT_30_LINK_STATE_U1               0x0020
#define MAUSB_PORT_30_LINK_STATE_U2               0x0040
#define MAUSB_PORT_30_LINK_STATE_U3               0x0060
#define MAUSB_PORT_30_LINK_STATE_DISABLED         0x0080
#define MAUSB_PORT_30_LINK_STATE_RX_DETECT        0x00A0
#define MAUSB_PORT_30_LINK_STATE_INACTIVE         0x00C0
#define MAUSB_PORT_30_LINK_STATE_POLLING          0x00E0
#define MAUSB_PORT_30_LINK_STATE_RECOVERY         0x0100
#define MAUSB_PORT_30_LINK_STATE_HOT_RESET        0x0120
#define MAUSB_PORT_30_LINK_STATE_COMPLIANCE_MODE  0x0140
#define MAUSB_PORT_30_LINK_STATE_LOOPBACK         0x0160
#define MAUSB_PORT_30_STATUS_POWER                0x0200
#define MAUSB_PORT_30_STATUS_SUPER_SPEED          0x0400
#define MAUSB_PORT_30_CLEAR_LINK_STATE            0xFE1F

/* USB 3.2 specification chapter 10.16.2.6.2 table 10-14 page 443 */
#define MAUSB_CHANGE_PORT_30_STATUS_CONNECT              0x010000
#define MAUSB_CHANGE_PORT_30_STATUS_OVER_CURRENT         0x080000
#define MAUSB_CHANGE_PORT_30_STATUS_RESET                0x100000
#define MAUSB_CHANGE_PORT_30_BH_STATUS_RESET             0x200000
#define MAUSB_CHANGE_PORT_30_LINK_STATE                  0x400000
#define MAUSB_CHANGE_PORT_30_CONFIG_ERROR                0x800000

/* USB 3.2 specification chapter 10.16.2.4 table 10-10 page 438 */
#define MAUSB_HUB_30_POWER_GOOD              0x00
#define MAUSB_HUB_30_LOCAL_POWER_SOURCE_LOST 0x01
#define MAUSB_HUB_30_OVER_CURRENT            0x02

/* USB 3.2 specification chapter 10.16.2.4 table 10-11 page 438 */
#define MAUSB_CHANGE_HUB_30_LOCAL_POWER_SOURCE_LOST  0x10000
#define MAUSB_CHANGE_HUB_30_OVER_CURRENT             0x20000

#define DEV_HANDLE_NOT_ASSIGNED	-1

struct mausb_usb_device_ctx {
	s32		dev_handle;
	bool		addressed;
	void		*dev_addr;
	struct rb_node	rb_node;
};

struct mausb_endpoint_ctx {
	u16	ep_handle;
	u16	dev_handle;
	void	*ma_dev;
	struct mausb_usb_device_ctx *usb_device_ctx;
};

struct mausb_urb_ctx {
	struct urb		*urb;
	struct mausb_data_iter	iterator;
	struct rb_node		rb_node;
	struct work_struct	work;
};

int mausb_probe(struct device *dev);
void mausb_hcd_urb_complete(struct urb *urb, u32 actual_length, int status);

#ifdef ISOCH_CALLBACKS
int mausb_sec_event_ring_setup(struct usb_hcd *hcd, unsigned int intr_num);
int mausb_sec_event_ring_cleanup(struct usb_hcd *hcd, unsigned int intr_num);
phys_addr_t mausb_get_sec_event_ring_phys_addr(struct usb_hcd *hcd,
					       unsigned int intr_num,
					       dma_addr_t *dma);
phys_addr_t mausb_get_xfer_ring_phys_addr(struct usb_hcd *hcd,
					  struct usb_device *udev,
					  struct usb_host_endpoint *ep,
					  dma_addr_t *dma);
int mausb_get_core_id(struct usb_hcd *hcd);
#endif /* ISOCH_CALLBACKS */

void mausb_clear_hcd_madev(u16 port_number);

#endif /* __MAUSB_HCD_HUB_H__ */
