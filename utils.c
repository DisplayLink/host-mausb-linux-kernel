// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 - 2020 DisplayLink (UK) Ltd.
 */
#include "utils.h"

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "mausb_driver_status.h"

#define MAUSB_KERNEL_DEV_NAME "mausb_host"
#define MAUSB_READ_DEVICE_TIMEOUT_MS 500

static dev_t mausb_major_kernel;
static struct cdev  mausb_kernel_dev;
static struct class *mausb_kernel_class;

static void mausb_vm_open(struct vm_area_struct *vma)
{
	mausb_pr_debug("");
}

static void mausb_vm_close(struct vm_area_struct *vma)
{
	struct mausb_ring_buffer *buffer = NULL, *next = NULL;
	unsigned long flags = 0;
	u64 ring_buffer_id = *(u64 *)(vma->vm_private_data);

	mausb_pr_info("Releasing ring buffer with id: %llu", ring_buffer_id);
	spin_lock_irqsave(&mss.lock, flags);
	list_for_each_entry_safe(buffer, next, &mss.available_ring_buffers,
				 list_entry) {
		if (buffer->id == ring_buffer_id) {
			list_del(&buffer->list_entry);
			mausb_ring_buffer_destroy(buffer);
			kfree(buffer);
			break;
		}
	}
	spin_unlock_irqrestore(&mss.lock, flags);
}

/* mausb_vm_fault is called the first time a memory area is accessed which is
 * not in memory
 */
#if KERNEL_VERSION(4, 17, 0) <= LINUX_VERSION_CODE
static vm_fault_t mausb_vm_fault(struct vm_fault *vmf)
#elif (KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE &&\
	KERNEL_VERSION(4, 17, 0) > LINUX_VERSION_CODE)
static int mausb_vm_fault(struct vm_fault *vmf)
#else
static int mausb_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#endif
{
	mausb_pr_debug("");
	return 0;
}

static const struct vm_operations_struct mausb_vm_ops = {
	.open  = mausb_vm_open,
	.close = mausb_vm_close,
	.fault = mausb_vm_fault,
};

static int mausb_file_open(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static int mausb_file_close(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	filp->private_data = NULL;

	return 0;
}

static ssize_t mausb_file_read(struct file *filp, char __user *user_buffer,
			       size_t size, loff_t *offset)
{
	ssize_t num_of_bytes_to_read = MAUSB_MAX_NUM_OF_MA_DEVS *
				       sizeof(struct mausb_events_notification);
	unsigned long num_of_bytes_not_copied;
	int completed_events;
	int ring_events;
	struct mausb_ring_buffer *ring_buffer;
	struct mausb_device	 *dev;
	struct completion	 *ring_has_events;
	u8 current_device = 0;
	s8 fail_ret_val;
	unsigned long flags;
	unsigned long timeout;
	long status;

	/* Reset heartbeat timer events */
	mausb_reset_heartbeat_cnt();

	if ((ssize_t)size != num_of_bytes_to_read) {
		mausb_pr_alert("Different expected bytes to read (%ld) from actual size (%ld)",
			       num_of_bytes_to_read, size);
		fail_ret_val = MAUSB_DRIVER_BAD_READ_BUFFER_SIZE;
		if (copy_to_user(user_buffer, &fail_ret_val,
				 sizeof(fail_ret_val)) != 0) {
			mausb_pr_warn("Failed to set error code.");
		}
		return MAUSB_DRIVER_READ_ERROR;
	}

	/* If suspend/hibernate happened delete all devices */
	if (atomic_xchg(&mss.num_of_transitions_to_sleep, 0)) {
		mausb_pr_alert("Suspend system event detected");
		fail_ret_val = MAUSB_DRIVER_SYSTEM_SUSPENDED;
		if (copy_to_user(user_buffer, &fail_ret_val,
				 sizeof(fail_ret_val)) != 0) {
			mausb_pr_warn("Failed to set error code.");
		}
		return MAUSB_DRIVER_READ_ERROR;
	}

	ring_has_events = &mss.rings_events.mausb_ring_has_events;
	timeout = msecs_to_jiffies(MAUSB_READ_DEVICE_TIMEOUT_MS);
	status = wait_for_completion_interruptible_timeout(ring_has_events,
							   timeout);
	reinit_completion(ring_has_events);

	if (atomic_read(&mss.rings_events.mausb_stop_reading_ring_events)) {
		mausb_pr_alert("Ring events stopped");
		fail_ret_val = MAUSB_DRIVER_RING_EVENTS_STOPPED;
		if (copy_to_user(user_buffer, &fail_ret_val,
				 sizeof(fail_ret_val)) != 0) {
			mausb_pr_warn("Failed to set error code.");
		}
		return MAUSB_DRIVER_READ_ERROR;
	}

	/* There are no new events - waiting for events hit timeout */
	if (status == 0)
		return MAUSB_DRIVER_READ_TIMEOUT;

	spin_lock_irqsave(&mss.lock, flags);

	list_for_each_entry(dev, &mss.madev_list, list_entry) {
		mss.events[current_device].madev_addr = dev->madev_addr;
		ring_buffer = dev->ring_buffer;
		ring_events = atomic_xchg(&ring_buffer->mausb_ring_events, 0);
		completed_events =
			atomic_xchg(&ring_buffer->mausb_completed_user_events,
				    0);
		mss.events[current_device].num_of_events = (u16)ring_events;
		mss.events[current_device].num_of_completed_events =
				(u16)completed_events;
		if (++current_device == MAUSB_MAX_NUM_OF_MA_DEVS)
			break;
	}

	spin_unlock_irqrestore(&mss.lock, flags);

	num_of_bytes_to_read =
		(ssize_t)(current_device *
			  sizeof(struct mausb_events_notification));
	num_of_bytes_not_copied =
		copy_to_user(user_buffer, &mss.events,
			     (unsigned long)num_of_bytes_to_read);

	mausb_pr_debug("num_of_bytes_not_copied %ld, num_of_bytes_to_read %ld",
		       num_of_bytes_not_copied, num_of_bytes_to_read);

	if (num_of_bytes_not_copied) {
		fail_ret_val = MAUSB_DRIVER_COPY_TO_BUFFER_FAILED;
		if (copy_to_user(user_buffer, &fail_ret_val,
				 sizeof(fail_ret_val)) != 0) {
			mausb_pr_warn("Failed to set error code.");
		}
		return MAUSB_DRIVER_READ_ERROR;
	}

	return num_of_bytes_to_read;
}

static ssize_t mausb_file_write(struct file *filp, const char __user *buffer,
				size_t size, loff_t *offset)
{
	ssize_t num_of_bytes_to_write =
				sizeof(struct mausb_events_notification);
	struct mausb_events_notification notification;
	unsigned long flags;
	struct mausb_device *dev;

	if (size != (size_t)num_of_bytes_to_write) {
		mausb_pr_alert("Different expected bytes to write (%ld) from actual size (%ld)",
			       num_of_bytes_to_write, size);
		return MAUSB_DRIVER_WRITE_ERROR;
	}

	if (copy_from_user(&notification, buffer, size))
		return MAUSB_DRIVER_WRITE_ERROR;

	spin_lock_irqsave(&mss.lock, flags);
	dev = mausb_get_dev_from_addr_unsafe(notification.madev_addr);

	if (!dev) {
		spin_unlock_irqrestore(&mss.lock, flags);
		return 0;
	}

	spin_lock_irqsave(&dev->num_of_user_events_lock, flags);
	dev->num_of_user_events += notification.num_of_events;
	dev->num_of_completed_events += notification.num_of_completed_events;
	spin_unlock_irqrestore(&dev->num_of_user_events_lock, flags);

	queue_work(dev->workq, &dev->work);
	spin_unlock_irqrestore(&mss.lock, flags);

	return num_of_bytes_to_write;
}

static inline unsigned long mausb_ring_buffer_length(void)
{
	int page_order = mausb_get_page_order(2 * MAUSB_RING_BUFFER_SIZE,
					      sizeof(struct mausb_event));
	return PAGE_SIZE << page_order;
}

static int mausb_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret;
	struct page *page = NULL;
	unsigned long flags = 0;
	struct mausb_ring_buffer *ring_buffer = kzalloc(sizeof(*ring_buffer),
							GFP_KERNEL);
	if (!ring_buffer)
		return -ENOMEM;

	ret = mausb_ring_buffer_init(ring_buffer);
	if (ret < 0) {
		mausb_pr_err("Ring buffer init failed");
		goto release_ring_buffer;
	}

	vma->vm_private_data = kzalloc(sizeof(ring_buffer->id), GFP_KERNEL);
	if (!vma->vm_private_data) {
		ret = -ENOMEM;
		goto release_ring_buffer;
	}

	filp->private_data = vma->vm_private_data;

	if (size > mausb_ring_buffer_length()) {
		mausb_pr_err("Invalid memory size to map");
		ret = -EINVAL;
		goto release_ring_buffer;
	}

	vma->vm_ops = &mausb_vm_ops;
	mausb_vm_open(vma);

	page = virt_to_page(ring_buffer->to_user_buffer);
	ret = remap_pfn_range(vma, vma->vm_start, page_to_pfn(page), size,
			      vma->vm_page_prot);
	if (ret < 0) {
		mausb_pr_err("Could not map the address area");
		goto release_ring_buffer;
	}

	spin_lock_irqsave(&mss.lock, flags);
	ring_buffer->id = mss.ring_buffer_id++;
	*(u64 *)(vma->vm_private_data) = ring_buffer->id;
	list_add_tail(&ring_buffer->list_entry, &mss.available_ring_buffers);
	mausb_pr_info("Allocated ring buffer with id: %llu", ring_buffer->id);
	spin_unlock_irqrestore(&mss.lock, flags);

	return 0;

release_ring_buffer:
	mausb_ring_buffer_destroy(ring_buffer);
	kfree(ring_buffer);
	return ret;
}

static const struct file_operations mausb_file_ops = {
	.open	 = mausb_file_open,
	.release = mausb_file_close,
	.read	 = mausb_file_read,
	.write   = mausb_file_write,
	.mmap	 = mausb_mmap,
};

int mausb_create_dev(void)
{
	int device_created = 0;
	int status = alloc_chrdev_region(&mausb_major_kernel, 0, 1,
					 MAUSB_KERNEL_DEV_NAME "_proc");
	if (status)
		goto cleanup;

	mausb_kernel_class = class_create(THIS_MODULE,
					  MAUSB_KERNEL_DEV_NAME "_sys");
	if (!mausb_kernel_class) {
		status = -ENOMEM;
		goto cleanup;
	}

	if (!device_create(mausb_kernel_class, NULL, mausb_major_kernel, NULL,
			   MAUSB_KERNEL_DEV_NAME "_dev")) {
		status = -ENOMEM;
		goto cleanup;
	}
	device_created = 1;
	cdev_init(&mausb_kernel_dev, &mausb_file_ops);
	status = cdev_add(&mausb_kernel_dev, mausb_major_kernel, 1);
	if (status)
		goto cleanup;
	return 0;
cleanup:
	mausb_cleanup_dev(device_created);
	return status;
}

void mausb_cleanup_dev(int device_created)
{
	if (device_created) {
		device_destroy(mausb_kernel_class, mausb_major_kernel);
		cdev_del(&mausb_kernel_dev);
	}

	if (mausb_kernel_class)
		class_destroy(mausb_kernel_class);

	unregister_chrdev_region(mausb_major_kernel, 1);
}

void mausb_notify_completed_user_events(struct mausb_ring_buffer *ring_buffer)
{
	int completed;

	completed =
		atomic_inc_return(&ring_buffer->mausb_completed_user_events);
	mausb_pr_debug("mausb_completed_user_events INCREMENTED %d", completed);
	if (completed > MAUSB_RING_BUFFER_SIZE / 16)
		complete(&mss.rings_events.mausb_ring_has_events);
}

void mausb_notify_ring_events(struct mausb_ring_buffer *ring_buffer)
{
	int events;

	events = atomic_inc_return(&ring_buffer->mausb_ring_events);
	if (events == 1)
		complete(&mss.rings_events.mausb_ring_has_events);
}

void mausb_stop_ring_events(void)
{
	atomic_set(&mss.rings_events.mausb_stop_reading_ring_events, 1);
	complete(&mss.rings_events.mausb_ring_has_events);
}
