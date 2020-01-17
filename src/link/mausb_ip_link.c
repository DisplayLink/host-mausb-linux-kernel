// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "link/mausb_ip_link.h"

#include <linux/in.h>
#include <linux/inet.h>
#include <linux/jiffies.h>
#include <linux/net.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/socket.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <net/tcp.h>
#include <net/sock.h>

#include "utils/mausb_logs.h"

static void __mausb_ip_connect(struct work_struct *work);
static int __mausb_ip_recv(struct mausb_ip_ctx *ip_ctx);
static void __mausb_ip_recv_work(struct work_struct *work);
static inline void __mausb_ip_recv_ctx_clear(struct mausb_ip_recv_ctx
					     *recv_ctx);
static inline void __mausb_ip_recv_ctx_free(struct mausb_ip_recv_ctx
					    *recv_ctx);

int mausb_init_ip_ctx(struct mausb_ip_ctx **ip_ctx,
		      struct net *net_ns,
		      char ip_addr[INET6_ADDRSTRLEN],
		      uint16_t port, void *context,
		      void (*fn_callback)(void *ctx, enum mausb_channel channel,
					  enum mausb_link_action act,
					  int status, void *data),
		      enum mausb_channel channel)
{
	struct mausb_ip_ctx *ctx;
	*ip_ctx = kzalloc(sizeof(**ip_ctx), GFP_ATOMIC);
	if (unlikely(*ip_ctx == NULL)) {
		mausb_pr_alert("ip context allocation failed");
		return -ENOMEM;
	}
	ctx = *ip_ctx;
	ctx->client_socket = NULL;
	__mausb_ip_recv_ctx_clear(&ctx->recv_ctx);
	/* something safer than strcpy */
	strcpy(ctx->ip_addr, ip_addr);
	ctx->port = port;
	ctx->net_ns = net_ns;

	if (channel == MAUSB_ISOCH_CHANNEL)
		ctx->udp = true;

	ctx->connect_workq = alloc_ordered_workqueue("connect_workq",
						     WQ_MEM_RECLAIM);
	if (!ctx->connect_workq) {
		mausb_pr_alert("connect_workq alloc failed");
		kfree(ctx);
		return -ENOMEM;
	}

	ctx->recv_workq =
	    alloc_ordered_workqueue("recv_workq", WQ_MEM_RECLAIM);
	if (!ctx->recv_workq) {
		mausb_pr_alert("send_recv_workq alloc failed");
		destroy_workqueue(ctx->connect_workq);
		kfree(ctx);
		return -ENOMEM;
	}

	INIT_WORK(&ctx->connect_work, __mausb_ip_connect);
	INIT_WORK(&ctx->recv_work, __mausb_ip_recv_work);

	ctx->channel	 = channel;
	ctx->ctx	 = context;
	ctx->fn_callback = fn_callback;

	return 0;
}

void mausb_destroy_ip_ctx(struct mausb_ip_ctx *ip_ctx)
{
	if (!ip_ctx)
		return;

	if (ip_ctx->connect_workq) {
		flush_workqueue(ip_ctx->connect_workq);
		destroy_workqueue(ip_ctx->connect_workq);
	}

	if (ip_ctx->recv_workq) {
		flush_workqueue(ip_ctx->recv_workq);
		destroy_workqueue(ip_ctx->recv_workq);
	}
	if (ip_ctx->client_socket)
		sock_release(ip_ctx->client_socket);
	__mausb_ip_recv_ctx_free(&ip_ctx->recv_ctx);

	kfree(ip_ctx);
}

static void __mausb_ip_set_options(struct socket *sock, bool udp)
{
	uint32_t optval = 0;
	unsigned int optlen = sizeof(optval);
	int status = 0;
	struct timeval timeo = {.tv_sec = 0, .tv_usec = 500000U };
	struct timeval send_timeo = {.tv_sec = 1, .tv_usec = 0 };

	if (!udp) {
		optval = 1;
		status = kernel_setsockopt(sock, IPPROTO_TCP, TCP_NODELAY,
					  (char *)&optval, optlen);
		if (status < 0)
			mausb_pr_warn("Failed to set tcp no delay option: status=%d",
				status);
	}
#if KERNEL_VERSION(5, 1, 0) <= LINUX_VERSION_CODE
	status = kernel_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO_NEW,
				  (char *)&timeo, sizeof(timeo));
#else
	status = kernel_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
				  (char *)&timeo, sizeof(timeo));
#endif

	if (status < 0)
		mausb_pr_warn("Failed to set recv timeout option: status=%d",
			status);

#if KERNEL_VERSION(5, 1, 0) <= LINUX_VERSION_CODE
	status = kernel_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO_NEW,
				  (char *)&send_timeo, sizeof(send_timeo));
#else
	status = kernel_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO,
				  (char *)&send_timeo, sizeof(send_timeo));
#endif

	if (status < 0)
		mausb_pr_warn("Failed to set snd timeout option: status=%d",
			      status);

	optval = MAUSB_LINK_BUFF_SIZE;
	status  = kernel_setsockopt(sock, SOL_SOCKET, SO_RCVBUF,
				   (char *) &optval, optlen);
	if (status < 0)
		mausb_pr_warn("Failed to set recv buffer size: status=%d",
			      status);

	optval = MAUSB_LINK_BUFF_SIZE;
	status  = kernel_setsockopt(sock, SOL_SOCKET, SO_SNDBUF,
				   (char *) &optval, optlen);
	if (status < 0)
		mausb_pr_warn("Failed to set send buffer size: status=%d",
			      status);

	optval = MAUSB_LINK_TOS_LEVEL_EF;
	status  = kernel_setsockopt(sock, IPPROTO_IP, IP_TOS,
				   (char *) &optval, optlen);
	if (status < 0)
		mausb_pr_warn("Failed to set QOS: status=%d", status);

}

static void __mausb_ip_connect(struct work_struct *work)
{
	struct sockaddr_in sockaddr;
	int status = 0;

	struct mausb_ip_ctx *ip_ctx = container_of(work, struct mausb_ip_ctx,
						   connect_work);

	if (!ip_ctx->udp) {
#if KERNEL_VERSION(4, 2, 0) <= LINUX_VERSION_CODE
		status = sock_create_kern(ip_ctx->net_ns, AF_INET, SOCK_STREAM,
					  IPPROTO_TCP, &ip_ctx->client_socket);
#else
		status = sock_create(AF_INET, SOCK_STREAM, IPPROTO_TCP,
				     &ip_ctx->client_socket);
#endif /* #if KERNEL_VERSION(4, 2, 0) <= LINUX_VERSION_CODE */
		if (status < 0) {
			mausb_pr_err("Failed to create socket: status=%d",
				     status);
			goto callback;
		}
	} else {
#if KERNEL_VERSION(4, 2, 0) <= LINUX_VERSION_CODE
		status = sock_create_kern(ip_ctx->net_ns, AF_INET, SOCK_DGRAM,
					  IPPROTO_UDP, &ip_ctx->client_socket);
#else
		status = sock_create(AF_INET, SOCK_DGRAM, IPPROTO_UDP,
				     &ip_ctx->client_socket);
#endif /* #if KERNEL_VERSION(4, 2, 0) <= LINUX_VERSION_CODE */
		if (status < 0) {
			mausb_pr_err("Failed to create socket: status=%d",
				     status);
			goto callback;
		}
	}

	memset(&sockaddr, 0, sizeof(sockaddr));
	sockaddr.sin_family	 = AF_INET;
	sockaddr.sin_port	 = htons(ip_ctx->port);
	sockaddr.sin_addr.s_addr = in_aton(ip_ctx->ip_addr);

	__mausb_ip_set_options((struct socket *)ip_ctx->client_socket,
			       ip_ctx->udp);

	status = kernel_connect(ip_ctx->client_socket,
				(struct sockaddr *)&sockaddr, sizeof(sockaddr),
				O_RDWR);
	if (status < 0) {
		mausb_pr_err("Failed to connect to host %s:%d, status=%d",
			     ip_ctx->ip_addr, ip_ctx->port, status);
		goto clear_socket;
	}

	queue_work(ip_ctx->recv_workq, &ip_ctx->recv_work);
	mausb_pr_info("Conected to host %s:%d, status=%d", ip_ctx->ip_addr,
							   ip_ctx->port,
							   status);

	goto callback;

clear_socket:
	sock_release(ip_ctx->client_socket);
	ip_ctx->client_socket = NULL;
callback:
	ip_ctx->fn_callback(ip_ctx->ctx, ip_ctx->channel, MAUSB_LINK_CONNECT,
			    status, NULL);
}

void mausb_ip_connect_async(struct mausb_ip_ctx *ip_ctx)
{
	queue_work(ip_ctx->connect_workq, &ip_ctx->connect_work);
}

int mausb_ip_disconnect(struct mausb_ip_ctx *ip_ctx)
{
	int status = 0;

	if (ip_ctx && ip_ctx->client_socket)
		status = kernel_sock_shutdown(ip_ctx->client_socket, SHUT_RDWR);
	return status;
}

int mausb_ip_send(struct mausb_ip_ctx *ip_ctx,
		  struct mausb_kvec_data_wrapper *wrapper)
{
	struct msghdr msghd;
	int status;

	if (!ip_ctx) {
		mausb_pr_alert("Socket ctx is NULL!");
		return -EINVAL;
	}

	memset(&msghd, 0, sizeof(msghd));
	msghd.msg_flags = MSG_WAITALL;

	status = kernel_sendmsg(ip_ctx->client_socket, &msghd, wrapper->kvec,
				wrapper->kvec_num, wrapper->length);

	return status;
}

static inline void __mausb_ip_recv_ctx_clear(struct mausb_ip_recv_ctx *recv_ctx)
{
	recv_ctx->buffer   = NULL;
	recv_ctx->left	   = 0;
	recv_ctx->received = 0;
}

static inline void __mausb_ip_recv_ctx_free(struct mausb_ip_recv_ctx *recv_ctx)
{
	kfree(recv_ctx->buffer);
	__mausb_ip_recv_ctx_clear(recv_ctx);
}

static int __mausb_ip_recv(struct mausb_ip_ctx *ip_ctx)
{
	struct msghdr msghd;
	struct kvec vec;
	int  status;
	bool peek = true;
	unsigned int optval = 1;

	/* receive with timeout of 0.5s */
	while (true) {
		memset(&msghd, 0, sizeof(msghd));
		if (peek) {
			vec.iov_base = ip_ctx->recv_ctx.common_hdr;
			vec.iov_len  = sizeof(ip_ctx->recv_ctx.common_hdr);
			msghd.msg_flags = MSG_PEEK;
		} else {
			vec.iov_base =
			    ip_ctx->recv_ctx.buffer +
			    ip_ctx->recv_ctx.received;
			vec.iov_len = ip_ctx->recv_ctx.left;
			msghd.msg_flags = MSG_WAITALL;
		}

		if (!ip_ctx->udp) {
			status = kernel_setsockopt(
					(struct socket *)ip_ctx->client_socket,
					IPPROTO_TCP, TCP_QUICKACK,
					(char *) &optval, sizeof(optval));
			if (status != 0) {
				mausb_pr_warn("Setting TCP_QUICKACK failed: %s:%d, status=%d",
					      ip_ctx->ip_addr, ip_ctx->port,
					      status);
			}
		}

		status =
		    kernel_recvmsg((struct socket *)ip_ctx->client_socket,
				   &msghd, &vec, 1, vec.iov_len,
				   msghd.msg_flags);

		if (status == -EAGAIN) {
			return status;
		} else if (status <= 0) {
			mausb_pr_warn("kernel_recvmsg host %s:%d, status=%d",
				      ip_ctx->ip_addr, ip_ctx->port, status);

			__mausb_ip_recv_ctx_free(&ip_ctx->recv_ctx);
			ip_ctx->fn_callback(ip_ctx->ctx, ip_ctx->channel,
					    MAUSB_LINK_RECV, status, NULL);
			return status;

		} else {
			mausb_pr_debug("kernel_recvmsg host %s:%d, status=%d",
				       ip_ctx->ip_addr, ip_ctx->port, status);
		}

		if (peek) {
			if (status < sizeof(ip_ctx->recv_ctx.common_hdr))
				return -EAGAIN;
			/* length field of mausb_common_hdr */
			ip_ctx->recv_ctx.left =
			    *(uint16_t *) (&ip_ctx->recv_ctx.common_hdr[2]);
			ip_ctx->recv_ctx.received = 0;
			ip_ctx->recv_ctx.buffer	  =
			    kzalloc(ip_ctx->recv_ctx.left, GFP_KERNEL);
			peek = false;
			if (!ip_ctx->recv_ctx.buffer) {
				ip_ctx->fn_callback(ip_ctx->ctx,
						    ip_ctx->channel,
						    MAUSB_LINK_RECV,
						    -ENOMEM, NULL);
				return -ENOMEM;
			}
		} else {
			if (status < ip_ctx->recv_ctx.left) {
				ip_ctx->recv_ctx.left -= status;
				ip_ctx->recv_ctx.received += status;
			} else {
				ip_ctx->fn_callback(ip_ctx->ctx,
						    ip_ctx->channel,
						    MAUSB_LINK_RECV, status,
						    ip_ctx->recv_ctx.buffer);
				__mausb_ip_recv_ctx_clear(&ip_ctx->recv_ctx);
				peek = true;
			}
		}
	}

	return status;
}

static void __mausb_ip_recv_work(struct work_struct *work)
{
	struct mausb_ip_ctx *ip_ctx = container_of(work, struct mausb_ip_ctx,
						   recv_work);
	int status = __mausb_ip_recv(ip_ctx);

	if (status <= 0 && status != -EAGAIN)
		return;

	queue_work(ip_ctx->recv_workq, &ip_ctx->recv_work);
}
