// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 */
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/net.h>

#include "hcd.h"
#include "hpal.h"
#include "mausb_address.h"
#include "utils.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("DisplayLink (UK) Ltd.");
MODULE_VERSION(MAUSB_DRIVER_VERSION);

static struct mausb_device_address	device_address;
static int				mausb_device_disconnect_param;
static u16				madev_addr;
static u8				mausb_client_connect_param;
static u8				mausb_client_disconnect_param;

static int mausb_client_connect(const char *value,
				const struct kernel_param *kp)
{
	unsigned long flags = 0;

	mausb_pr_info("Version=%s", MAUSB_DRIVER_VERSION);

	spin_lock_irqsave(&mss.lock, flags);
	if (mss.client_connected) {
		mausb_pr_err("MA-USB client is already connected");
		spin_unlock_irqrestore(&mss.lock, flags);
		return -EEXIST;
	}
	/* Save heartbeat client information */
	mss.client_connected = true;
	mss.missed_heartbeats = 0;
	reinit_completion(&mss.client_stopped);
	spin_unlock_irqrestore(&mss.lock, flags);
	/* Start hearbeat timer */
	mod_timer(&mss.heartbeat_timer,
		  jiffies + msecs_to_jiffies(MAUSB_HEARTBEAT_TIMEOUT_MS));

	return 0;
}

static int mausb_client_disconnect(const char *value,
				   const struct kernel_param *kp)
{
	unsigned long flags = 0;
	struct mausb_device *dev = NULL;

	mausb_pr_info("Version=%s", MAUSB_DRIVER_VERSION);

	spin_lock_irqsave(&mss.lock, flags);
	if (!mss.client_connected) {
		mausb_pr_err("MA-USB client is not connected");
		spin_unlock_irqrestore(&mss.lock, flags);
		return -ENODEV;
	}

	spin_unlock_irqrestore(&mss.lock, flags);

	/* Stop heartbeat timer */
	del_timer_sync(&mss.heartbeat_timer);

	/* Clear heartbeat client information */
	spin_lock_irqsave(&mss.lock, flags);
	mss.client_connected = false;
	mss.missed_heartbeats = 0;
	list_for_each_entry(dev, &mss.madev_list, list_entry) {
		mausb_pr_debug("Enqueue heartbeat_work madev_addr=%x",
			       dev->madev_addr);
		queue_work(dev->workq, &dev->heartbeat_work);
	}
	complete(&mss.client_stopped);
	spin_unlock_irqrestore(&mss.lock, flags);

	return 0;
}

static int mausb_device_connect(const char *value,
				const struct kernel_param *kp)
{
	int status = 0;

	mausb_pr_info("Version=%s", MAUSB_DRIVER_VERSION);

	if (strlen(value) <= INET_ADDRSTRLEN) {
		strcpy(device_address.ip.address.ip4, value);
		/* Add list of already connected devices */
	} else if (strlen(value) <= INET6_ADDRSTRLEN) {
		/* Logic for ip6 */
	} else {
		mausb_pr_err("Invalid IP format");
		return 0;
	}
	status = mausb_initiate_dev_connection(device_address, madev_addr);
	memset(&device_address, 0, sizeof(device_address));

	return status;
}

static int mausb_device_disconnect(const char *value,
				   const struct kernel_param *kp)
{
	u8 dev_address = 0;
	int status = 0;
	unsigned long flags = 0;
	struct mausb_device *dev = NULL;

	mausb_pr_info("Version=%s", MAUSB_DRIVER_VERSION);

	status = kstrtou8(value, 0, &dev_address);
	if (status < 0)
		return -EINVAL;

	spin_lock_irqsave(&mss.lock, flags);

	dev = mausb_get_dev_from_addr_unsafe(dev_address);
	if (dev)
		queue_work(dev->workq, &dev->hcd_disconnect_work);

	spin_unlock_irqrestore(&mss.lock, flags);

	return 0;
}

static const struct kernel_param_ops mausb_device_connect_ops = {
	.set = mausb_device_connect
};

static const struct kernel_param_ops mausb_device_disconnect_ops = {
	.set = mausb_device_disconnect
};

static const struct kernel_param_ops mausb_client_connect_ops = {
	.set = mausb_client_connect
};

static const struct kernel_param_ops mausb_client_disconnect_ops = {
	.set = mausb_client_disconnect
};

module_param_named(mgmt, device_address.ip.port.management, ushort, 0664);
MODULE_PARM_DESC(mgmt, "MA-USB management port");
module_param_named(ctrl, device_address.ip.port.control, ushort, 0664);
MODULE_PARM_DESC(ctrl, "MA-USB control port");
module_param_named(bulk, device_address.ip.port.bulk, ushort, 0664);
MODULE_PARM_DESC(bulk, "MA-USB bulk port");
module_param_named(isoch, device_address.ip.port.isochronous, ushort, 0664);
MODULE_PARM_DESC(isoch, "MA-USB isochronous port");
module_param_named(madev_addr, madev_addr, ushort, 0664);
MODULE_PARM_DESC(madev_addr, "MA-USB device address");

module_param_cb(client_connect, &mausb_client_connect_ops,
		&mausb_client_connect_param, 0664);
module_param_cb(client_disconnect, &mausb_client_disconnect_ops,
		&mausb_client_disconnect_param, 0664);
module_param_cb(ip, &mausb_device_connect_ops,
		device_address.ip.address.ip4, 0664);
module_param_cb(disconnect, &mausb_device_disconnect_ops,
		&mausb_device_disconnect_param, 0664);

static int mausb_host_init(void)
{
	int status;

	mausb_pr_info("Module load. Version=%s", MAUSB_DRIVER_VERSION);
	status = mausb_init_hcd();
	if (status < 0)
		goto cleanup;

	status = mausb_register_power_state_listener();
	if (status < 0)
		goto cleanup_hcd;

	status = mausb_create_dev();
	if (status < 0)
		goto unregister_power_state_listener;

	mausb_initialize_mss();

	return 0;

unregister_power_state_listener:
	mausb_unregister_power_state_listener();
cleanup_hcd:
	mausb_deinit_hcd();
cleanup:
	mausb_pr_alert("Failed to load MAUSB module!");
	return status;
}

static void mausb_host_exit(void)
{
	mausb_pr_info("Module unloading started...");
	mausb_unregister_power_state_listener();
	mausb_deinitialize_mss();
	mausb_deinit_hcd();
	mausb_cleanup_dev(1);
	mausb_pr_info("Module unloaded. Version=%s", MAUSB_DRIVER_VERSION);
}

module_init(mausb_host_init);
module_exit(mausb_host_exit);
