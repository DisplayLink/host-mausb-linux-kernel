/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __MAUSB_UTILS_MAUSB_LOGS_H__
#define __MAUSB_UTILS_MAUSB_LOGS_H__

#ifdef MAUSB_WITH_LOGS
#include <linux/sched.h>
#define mausb_pr_logs(level_str, level, log, ...)\
	pr_##level_str("MAUSB " #level " [%x:%x] [%s] " log "\n",\
	current->pid, current->tgid, __func__, ##__VA_ARGS__)
#else
#define mausb_pr_logs(...)
#endif /* MAUSB_WITH_LOGS */

#define mausb_pr_alert(...) mausb_pr_logs(alert, 1, ##__VA_ARGS__)

#define mausb_pr_err(...) mausb_pr_logs(err, 2, ##__VA_ARGS__)

#define mausb_pr_warn(...) mausb_pr_logs(warn, 3, ##__VA_ARGS__)

#define mausb_pr_info(...)  mausb_pr_logs(info, 4, ##__VA_ARGS__)

#if defined(MAUSB_LOG_VERBOSE)
	#define mausb_pr_debug(...) mausb_pr_logs(debug, 5, ##__VA_ARGS__)
#else
	#define mausb_pr_debug(...)
#endif /* defined(MAUSB_LOG_VERBOSE) */

#endif /* __MAUSB_UTILS_MAUSB_LOGS_H__ */
