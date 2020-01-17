/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_HCD_VHCD_H__
#define __MAUSB_HCD_VHCD_H__

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "utils/mausb_ring_buffer.h"

#define DEVICE_NAME "mausb_host_hcd_dev"
#define CLASS_NAME "mausb"

#define NUMBER_OF_PORTS		15

#define MAX_USB_DEVICE_DEPTH	6

#define RESPONSE_TIMEOUT	5000

struct mausb_hcd {
	spinlock_t	lock;
	struct device	*pdev;
	uint16_t	connected_ports;

	struct rb_root	mausb_urbs;
	struct hub_ctx	*hcd_ss_ctx;
	struct hub_ctx	*hcd_hs_ctx;
	struct notifier_block power_state_listener;
};

struct mausb_dev {
	uint32_t	port_status;
	struct rb_root	usb_devices;
	uint8_t		dev_speed;
	void		*ma_dev;
};

struct hub_ctx {
	struct mausb_hcd *mhcd;
	struct usb_hcd	 *hcd;
	struct mausb_dev ma_devs[NUMBER_OF_PORTS];
};

enum mausb_device_type {
	USBDEVICE = 0,
	USB20HUB  = 1,
	USB30HUB  = 2
};

enum mausb_device_speed {
	LOW_SPEED	 = 0,
	FULL_SPEED	 = 1,
	HIGH_SPEED	 = 2,
	SUPER_SPEED	 = 3,
	SUPER_SPEED_PLUS = 4
};

extern struct mausb_hcd *mhcd;

int mausb_init_hcd(void);
void mausb_deinit_hcd(void);

void mausb_port_has_changed(const enum mausb_device_type device_type,
			    const enum mausb_device_speed device_speed,
			    void *ma_dev);
void mausb_hcd_disconnect(const uint16_t port_number,
			  const enum mausb_device_type device_type,
			  const enum mausb_device_speed device_speed);

#endif /* __MAUSB_HCD_VHCD_H__ */
