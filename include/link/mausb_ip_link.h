/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_LINK_MAUSB_IP_LINK_H__
#define __MAUSB_LINK_MAUSB_IP_LINK_H__

#include <linux/inet.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <net/net_namespace.h>

#define MAUSB_LINK_BUFF_SIZE	16777216
#define MAUSB_LINK_TOS_LEVEL_EF 0xB8

enum mausb_link_action {
	MAUSB_LINK_CONNECT	= 0,
	MAUSB_LINK_DISCONNECT	= 1,
	MAUSB_LINK_RECV		= 2,
	MAUSB_LINK_SEND		= 3
};

enum mausb_channel {
	MAUSB_CTRL_CHANNEL  = 0,
	MAUSB_ISOCH_CHANNEL = 1,
	MAUSB_BULK_CHANNEL  = 2,
	MAUSB_INTR_CHANNEL  = 3,
	MAUSB_MGMT_CHANNEL  = 4
};

struct mausb_kvec_data_wrapper {
	struct kvec *kvec;
	uint32_t    kvec_num;
	uint32_t    length;
};

struct mausb_ip_recv_ctx {
	uint16_t left;
	uint16_t received;
	char	 *buffer;
	char	 common_hdr[12]
		 __aligned(4);
};

struct mausb_ip_ctx {
	struct socket *client_socket;
	struct net    *net_ns;
	char	      ip_addr[INET6_ADDRSTRLEN];
	uint16_t      port;
	bool	      udp;
	/* IPV6 support */

	/* Queues to schedule rx work */
	struct workqueue_struct	*recv_workq;
	struct workqueue_struct	*connect_workq;
	struct work_struct	recv_work;
	struct work_struct	connect_work;
	/* recv buffer */
	struct mausb_ip_recv_ctx recv_ctx;

	enum mausb_channel channel;
	void *ctx;
	/* callback should store task into hpal queue */
	void (*fn_callback)(void *ctx, enum mausb_channel channel,
			    enum mausb_link_action act, int status, void *data);
};

int mausb_init_ip_ctx(struct mausb_ip_ctx **ip_ctx,
		      struct net *net_ns,
		      char ip_addr[INET6_ADDRSTRLEN],
		      uint16_t port,
		      void *ctx,
		      void (*ctx_callback)(void *ctx,
					   enum mausb_channel channel,
					   enum mausb_link_action act,
					   int status, void *data),
		      enum mausb_channel channel);
int mausb_ip_disconnect(struct mausb_ip_ctx *ip_ctx);
int mausb_ip_send(struct mausb_ip_ctx *ip_ctx,
		  struct mausb_kvec_data_wrapper *wrapper);

void mausb_destroy_ip_ctx(struct mausb_ip_ctx *ip_ctx);
void mausb_ip_connect_async(struct mausb_ip_ctx *ip_ctx);

#endif /* __MAUSB_LINK_MAUSB_IP_LINK_H__ */
