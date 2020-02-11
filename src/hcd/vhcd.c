// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */
#include "hcd/vhcd.h"

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "hcd/hub.h"
#include "hpal/hpal.h"
#include "utils/mausb_logs.h"

static int mausb_open(struct inode *inode, struct file *file);
static int mausb_release(struct inode *inode, struct file *file);
static ssize_t mausb_read(struct file *file, char __user *buffer, size_t length,
			  loff_t *offset);
static ssize_t mausb_write(struct file *file, const char __user *buffer,
			   size_t length, loff_t *offset);
static long mausb_ioctl(struct file *file, unsigned int ioctl_func,
			unsigned long ioctl_buffer);
static int mausb_bus_probe(struct device *dev);
static int mausb_bus_remove(struct device *dev);
static int mausb_bus_match(struct device *dev, struct device_driver *drv);

static const struct file_operations mausb_fops = {
	.open		= mausb_open,
	.release	= mausb_release,
	.read		= mausb_read,
	.write		= mausb_write,
	.unlocked_ioctl	= mausb_ioctl
};

static unsigned int major;
static unsigned int minor = 1;
static dev_t devt;
static struct device *device;

struct mausb_hcd	*mhcd;
static struct class	*mausb_class;
static struct bus_type	mausb_bus_type = {
	.name	= DEVICE_NAME,
	.match	= mausb_bus_match,
	.probe	= mausb_bus_probe,
	.remove	= mausb_bus_remove,
};

static struct device_driver mausb_driver = {
	.name	= DEVICE_NAME,
	.bus	= &mausb_bus_type,
	.owner	= THIS_MODULE,
};

static void mausb_remove(void)
{
	struct usb_hcd *hcd, *shared_hcd;

	hcd	   = mhcd->hcd_hs_ctx->hcd;
	shared_hcd = mhcd->hcd_ss_ctx->hcd;

	if (shared_hcd) {
		usb_remove_hcd(shared_hcd);
		usb_put_hcd(shared_hcd);
		mhcd->hcd_ss_ctx = NULL;
	}

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	mhcd->hcd_hs_ctx = NULL;
}

static int mausb_bus_probe(struct device *dev)
{
	return mausb_probe(dev);
}

static int mausb_bus_remove(struct device *dev)
{
	return 0;
}

static int mausb_bus_match(struct device *dev, struct device_driver *drv)
{
	if (strncmp(dev->bus->name, drv->name, strlen(drv->name)))
		return 0;
	else
		return 1;
}

static int mausb_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mausb_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mausb_read(struct file *file, char __user *buffer, size_t length,
			  loff_t *offset)
{
	return 0;
}

static ssize_t mausb_write(struct file *file, const char __user *buffer,
			   size_t length, loff_t *offset)
{
	return 0;
}

static long mausb_ioctl(struct file *file, unsigned int ioctl_func,
			unsigned long ioctl_buffer)
{
	return 0;
}

int mausb_init_hcd(void)
{
	int retval;

	retval = register_chrdev(0, DEVICE_NAME, &mausb_fops);
	if (retval < 0) {
		mausb_pr_err("Register_chrdev failed");
		return retval;
	}

	major = (unsigned int)retval;
	retval = bus_register(&mausb_bus_type);
	if (retval) {
		mausb_pr_err("Bus_register failed %d", retval);
		goto bus_register_error;
	}

	mausb_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mausb_class)) {
		mausb_pr_err("Class_create failed %ld", PTR_ERR(mausb_class));
		goto class_error;
	}

	retval = driver_register(&mausb_driver);
	if (retval) {
		mausb_pr_err("Driver_register failed");
		goto driver_register_error;
	}

	mhcd = kzalloc(sizeof(*mhcd), GFP_ATOMIC);
	if (!mhcd) {
		retval = -ENOMEM;
		goto mausb_hcd_alloc_failed;
	}

	devt = MKDEV(major, minor);
	device = device_create(mausb_class, NULL, devt, mhcd, DEVICE_NAME);

	if (IS_ERR(device)) {
		mausb_pr_err("Device_create failed %ld", PTR_ERR(device));
		goto device_create_error;
	}

	device->driver = &mausb_driver;

	retval = mausb_probe(device);

	if (retval) {
		mausb_pr_err("Mausb_probe failed");
		goto mausb_probe_failed;
	}

	return retval;
mausb_probe_failed:
	device_destroy(mausb_class, devt);
device_create_error:
	kfree(mhcd);
	mhcd = NULL;
mausb_hcd_alloc_failed:
	driver_unregister(&mausb_driver);
driver_register_error:
	class_destroy(mausb_class);
class_error:
	bus_unregister(&mausb_bus_type);
bus_register_error:
	unregister_chrdev(major, DEVICE_NAME);

	return retval;
}

void mausb_deinit_hcd(void)
{
	mausb_pr_info("Start");

	if (mhcd) {
		mausb_remove();
		device_destroy(mausb_class, devt);
		driver_unregister(&mausb_driver);
		class_destroy(mausb_class);
		bus_unregister(&mausb_bus_type);
		unregister_chrdev(major, DEVICE_NAME);
	}

	mausb_pr_info("Finish");
}

void mausb_port_has_changed(const enum mausb_device_type device_type,
			    const enum mausb_device_speed device_speed,
			    void *ma_dev)
{
	struct usb_hcd *hcd;
	unsigned long flags = 0;
	struct mausb_device *dev = ma_dev;
	u16 port_number = dev->port_number;

	spin_lock_irqsave(&mhcd->lock, flags);

	if (device_type == USB20HUB || device_speed < SUPER_SPEED) {
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
		    USB_PORT_STAT_CONNECTION | (1 <<
						USB_PORT_FEAT_C_CONNECTION);

		if (device_speed == LOW_SPEED) {
			mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
			    MAUSB_PORT_20_STATUS_LOW_SPEED;
			mhcd->hcd_hs_ctx->ma_devs[port_number].dev_speed =
			    LOW_SPEED;
		} else if (device_speed == HIGH_SPEED) {
			mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
			    MAUSB_PORT_20_STATUS_HIGH_SPEED;
			mhcd->hcd_hs_ctx->ma_devs[port_number].dev_speed =
			    HIGH_SPEED;
		}

		hcd = mhcd->hcd_hs_ctx->hcd;
		mhcd->hcd_hs_ctx->ma_devs[port_number].ma_dev = ma_dev;
	} else {
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status |=
		    USB_PORT_STAT_CONNECTION | (1 <<
						USB_PORT_FEAT_C_CONNECTION);
		mhcd->hcd_ss_ctx->ma_devs[port_number].dev_speed = SUPER_SPEED;

		hcd = mhcd->hcd_ss_ctx->hcd;
		mhcd->hcd_ss_ctx->ma_devs[port_number].ma_dev = ma_dev;
	}
	spin_unlock_irqrestore(&mhcd->lock, flags);

	usb_hcd_poll_rh_status(hcd);
}

void mausb_hcd_disconnect(const u16 port_number,
			  const enum mausb_device_type device_type,
			  const enum mausb_device_speed device_speed)
{
	struct usb_hcd *hcd;
	unsigned long flags = 0;

	if (port_number >= NUMBER_OF_PORTS) {
		mausb_pr_err("port number out of range, port_number=%x",
			     port_number);
		return;
	}

	spin_lock_irqsave(&mhcd->lock, flags);

	if (device_type == USB20HUB || device_speed < SUPER_SPEED) {
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_CONNECTION);
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_ENABLE);
		mhcd->hcd_hs_ctx->ma_devs[port_number].port_status |=
			(1 << USB_PORT_FEAT_C_CONNECTION);
		hcd = mhcd->hcd_hs_ctx->hcd;
	} else {
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_CONNECTION);
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status &=
			~(USB_PORT_STAT_ENABLE);
		mhcd->hcd_ss_ctx->ma_devs[port_number].port_status |=
			(1 << USB_PORT_FEAT_C_CONNECTION);
		hcd = mhcd->hcd_ss_ctx->hcd;
	}

	spin_unlock_irqrestore(&mhcd->lock, flags);
	if (!hcd)
		return;

	usb_hcd_poll_rh_status(hcd);
}
