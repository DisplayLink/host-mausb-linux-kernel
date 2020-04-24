/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 */
#ifndef __MAUSB_UTILS_H__
#define __MAUSB_UTILS_H__

#include "hpal.h"

#if defined(MAUSB_NO_LOGS)
#define mausb_pr_logs(...)
#else
#include <linux/printk.h>
#include <linux/sched.h>
#define mausb_pr_logs(level_str, level, log, ...)\
	pr_##level_str("MAUSB " #level " [%x:%x] [%s] " log "\n",\
	current->pid, current->tgid, __func__, ##__VA_ARGS__)
#endif /* MAUSB_NO_LOGS */

#define mausb_pr_alert(...) mausb_pr_logs(alert, 1, ##__VA_ARGS__)

#define mausb_pr_err(...) mausb_pr_logs(err, 2, ##__VA_ARGS__)

#define mausb_pr_warn(...) mausb_pr_logs(warn, 3, ##__VA_ARGS__)

#define mausb_pr_info(...) mausb_pr_logs(info, 4, ##__VA_ARGS__)

#if defined(MAUSB_LOG_VERBOSE)
	#define mausb_pr_debug(...) mausb_pr_logs(debug, 5, ##__VA_ARGS__)
#else
	#define mausb_pr_debug(...)
#endif /* defined(MAUSB_LOG_VERBOSE) */

#define MAUSB_STRINGIFY2(x) #x
#define MAUSB_STRINGIFY(x) MAUSB_STRINGIFY2(x)

#define MAUSB_DRIVER_VERSION MAUSB_STRINGIFY(1.3.0.0.781d5ee5)

int mausb_create_dev(void);
void mausb_cleanup_dev(int device_created);
void mausb_notify_completed_user_events(struct mausb_ring_buffer *ring_buffer);
void mausb_notify_ring_events(struct mausb_ring_buffer *ring_buffer);
void mausb_stop_ring_events(void);

#endif /* __MAUSB_UTILS_H__ */
