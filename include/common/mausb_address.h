/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_COMMON_MAUSB_ADDRESS_H__
#define __MAUSB_COMMON_MAUSB_ADDRESS_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <types.h>
#endif /* __KERNEL__ */

#define ADDR_LEN 16

struct mausb_device_address {
	u8 link_type;
	struct {
		union {
			char	ip4[ADDR_LEN];
			u8	ip6[ADDR_LEN];
		} address;
		u8 number_of_ports;
		struct {
			u16 management;
			u16 control;
			u16 bulk;
			u16 interrupt;
			u16 isochronous;
		} port;
	} ip;
};

#endif /* __MAUSB_COMMON_MAUSB_ADDRESS_H__ */
