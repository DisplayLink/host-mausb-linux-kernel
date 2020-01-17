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
	uint8_t linkType;
	struct {
		union {
			char	ip4[ADDR_LEN];
			uint8_t	ip6[ADDR_LEN];
		} Address;
		uint8_t numberOfPorts;
		struct {
			uint16_t management;
			uint16_t control;
			uint16_t bulk;
			uint16_t interrupt;
			uint16_t isochronous;
		} Port;
	} Ip;
};

#endif /* __MAUSB_COMMON_MAUSB_ADDRESS_H__ */
