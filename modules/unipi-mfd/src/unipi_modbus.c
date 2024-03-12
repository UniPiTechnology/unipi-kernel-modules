/*
 * UniPi PLC modbus channel driver - Copyright (C) 2018 UniPi Technology
 * 
 * Author: Tomas Knot <tomasknot@gmail.com>
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

//#include <linux/completion.h>
//#include <linux/cpufreq.h>
#include <linux/version.h>
#include <linux/poll.h>
#include <linux/wait.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,3,0)
#include <linux/filelock.h>
#endif

#include "unipi_common.h"
#include "unipi_modbus.h"
//#include "unipi_spi.h"


#if UNIPI_MODBUS_DETAILED_DEBUG > 1
# define unipi_modbus_trace_1(f, args...)	printk(f, ##args)
#else
# define unipi_modbus_trace_1(f, args...)
#endif

#if UNIPI_MODBUS_DETAILED_DEBUG > 0
# define unipi_modbus_trace(f, args...)	printk(f, ##args)
#else
# define unipi_modbus_trace(f, args...)
#endif

#define unipi_modbus_error(f, args...)	printk(f, ##args)


int				unipi_modbus_major;
struct class	*unipi_modbus_class;

enum unipi_modbus_status {
	UNIPI_MODBUS_STATUS_IDLE,
	UNIPI_MODBUS_STATUS_INOP,
	UNIPI_MODBUS_STATUS_DATA,
};

struct unipi_modbus_file_data
{
	struct mutex         lock;
	struct unipi_channel *channel;
	wait_queue_head_t    wait;
	enum unipi_modbus_status status;
	int					is_locked;
	u32					cookie;
	u8					recv_buf[UNIPI_MODBUS_BUFFER_MAX];
	u8					send_buf[UNIPI_MODBUS_BUFFER_MAX];
	int					result;
};

struct unipi_channel* unipi_modbus_dev_by_address(u8 modbus_address)
{
	struct device *dev;
	struct unipi_channel *channel;
	
	dev = class_find_device_by_devt(unipi_modbus_class, 
	                      MKDEV(unipi_modbus_major, modbus_address));
	if (dev) {
		channel = (struct unipi_channel*) dev_get_drvdata(dev);
		put_device(dev);
		return channel;
 	}
	return NULL;
}
EXPORT_SYMBOL_GPL(unipi_modbus_dev_by_address);

struct device* unipi_modbus_classdev_register(struct unipi_channel *channel, u8 address)
{
	struct device* modbus_dev;

	if (unipi_modbus_dev_by_address(address)) {
		dev_err(channel->dev,"Duplicit modbus-address (%d). Channel inaccessible!", address);
		return NULL;
	}

	modbus_dev = device_create(unipi_modbus_class, channel->dev, MKDEV(unipi_modbus_major, address),
	                           channel, UNIPI_MODBUS_DEVICE_NAME_T, address);
	if (IS_ERR(modbus_dev)) {
		dev_err(channel->dev,"Failed to create modbus device (%d). Channel inaccessible!", address);
		return NULL;
	}
	return modbus_dev;
}
EXPORT_SYMBOL_GPL(unipi_modbus_classdev_register);

void unipi_modbus_classdev_unregister(struct device *dev)
{
	device_unregister(dev);
}
EXPORT_SYMBOL_GPL(unipi_modbus_classdev_unregister);


/*****************************************************************
 *  Modbus like Interface via /dev/unipimodbus%d
 */
int unipi_modbus_open (struct inode *inode_p, struct file *file_p)
{
	struct unipi_modbus_file_data *private_data;
	int modbus_address = iminor(inode_p);
	private_data = kzalloc(sizeof(*private_data), GFP_ATOMIC);
	mutex_init(&private_data->lock);

	private_data->channel = unipi_modbus_dev_by_address(modbus_address);
	/* private_data->spi_dev = (struct spi_device*) inode_p->i_private; */
	if (private_data->channel == NULL || get_device(private_data->channel->dev) == NULL) {
		kfree(private_data);
		return -ENODEV;
	}
	if (private_data->channel->dev->driver) {
		if (private_data->channel->dev->driver->owner)
			try_module_get(private_data->channel->dev->driver->owner);
	}
	init_waitqueue_head(&private_data->wait);
	file_p->private_data = private_data;
	return 0;
}

int unipi_modbus_release (struct inode *inode_p, struct file *file_p)
{
	struct unipi_modbus_file_data *private_data;
	if (file_p == NULL) {
		return -1;
	}
	private_data = (struct unipi_modbus_file_data*)file_p->private_data;
	if (private_data->is_locked) {
		private_data->channel->op->unlock_op(private_data->channel->proto_self);
	}
	put_device(private_data->channel->dev);
	if (private_data->channel->dev->driver) {
		if (private_data->channel->dev->driver->owner)
			module_put(private_data->channel->dev->driver->owner);
	}

	kfree(private_data);
	file_p->private_data = NULL;
	return 0;
}


static __poll_t unipi_modbus_poll(struct file *file, poll_table *wait)
{
	struct unipi_modbus_file_data* private_data = (struct unipi_modbus_file_data*) file->private_data;
	//struct serio_raw_client *client = file->private_data;
	//struct serio_raw *serio_raw = client->serio_raw;
	__poll_t mask = 0;

	poll_wait(file, &private_data->wait, wait);
	if (private_data->status != UNIPI_MODBUS_STATUS_INOP)
		mask = EPOLLOUT | EPOLLWRNORM;
	else if (private_data->status == UNIPI_MODBUS_STATUS_DATA)
		mask = EPOLLIN | EPOLLRDNORM | EPOLLOUT | EPOLLWRNORM;

	//mask = serio_raw->dead ? EPOLLHUP | EPOLLERR : EPOLLOUT | EPOLLWRNORM;

	return mask;
}

ssize_t unipi_modbus_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{
	s32 result = 0;
	loff_t dummy_offset = 0;
	struct unipi_modbus_file_data* private_data;

	unipi_modbus_trace(KERN_INFO "unipi-modbus: CDEV Read %zu, offset=%d\n", len,
					   (int)*offset);
	/*
	if (buffer == NULL) return -EINVAL; // Invalid read buffer
	if (len == 0) return 0; // Empty read
	if (len > UNIPI_MODBUS_BUFFER_MAX) return -EMSGSIZE;
	*/
	if (file_p == NULL || file_p->private_data == NULL) {
		printk(KERN_DEBUG "unipi-modbus: Bad file descriptor\n");
		return -EINVAL;
	}
	private_data = (struct unipi_modbus_file_data*) file_p->private_data;

	mutex_lock(&(private_data->lock));
	if (private_data->is_locked) {
		if ((buffer != NULL) && (len > 0))
			result = simple_read_from_buffer(buffer, len, &dummy_offset, private_data->recv_buf,
		                                 UNIPI_MODBUS_BUFFER_MAX);
		goto unlock;
	}
	if (private_data->status == UNIPI_MODBUS_STATUS_INOP) {
		result = -EAGAIN;
		goto unlock;
	}
	if (private_data->status == UNIPI_MODBUS_STATUS_IDLE) {
		result = 0;
		goto unlock;
	}
	if ((buffer != NULL) && (len > 0) && (private_data->result >=0)){
		result = simple_read_from_buffer(buffer, len, &dummy_offset, private_data->recv_buf,
		                                 UNIPI_MODBUS_BUFFER_MAX);
		if (result < 0) private_data->result = result;
	}
	result = private_data->result;
	private_data->status = UNIPI_MODBUS_STATUS_IDLE;
unlock:
	mutex_unlock(&(private_data->lock));
	return result;
}

void unipi_modbus_op_callback(void* cb_data, int result)
{
	struct unipi_modbus_file_data *private_data = (struct unipi_modbus_file_data*) cb_data;
	if (cb_data) {
		private_data->result = result;
		private_data->status = UNIPI_MODBUS_STATUS_DATA;
		wake_up_interruptible(&private_data->wait);
	}
}

ssize_t unipi_modbus_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
    loff_t dummy_offset = 0;
	struct unipi_modbus_file_data *private_data;
	struct unipi_channel *channel;
	u8 op, count;
	int reg;
	int ret;

	unipi_modbus_trace(KERN_INFO "unipi-modbus: CDEV Write len:%zu\n", len);
	if ((buffer == NULL) || (len == 0)) {
		return 0; // Void write
	}

    if ((len > UNIPI_MODBUS_BUFFER_MAX) || (len < UNIPI_MODBUS_HEADER_SIZE))
		return -EMSGSIZE;

    if (file_p == NULL || file_p->private_data == NULL) {
    	printk(KERN_DEBUG "unipi-modbus: Bad file descriptor\n");
    	return -EINVAL;
    }
    private_data = (struct unipi_modbus_file_data*) file_p->private_data;

	channel = private_data->channel;

	mutex_lock(&(private_data->lock));
	if (private_data->is_locked) {
		/* Firmware op */
		private_data->recv_buf[0] = 0;
		simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
							   &dummy_offset, buffer, len);
		ret = channel->op->firmware_sync(channel->proto_self, private_data->send_buf, private_data->recv_buf, 
		                            len, private_data->cookie);
		goto unlock;
	}
	if (private_data->status == UNIPI_MODBUS_STATUS_INOP) {
		ret = -EBUSY;
		goto unlock;
	}
	private_data->status = UNIPI_MODBUS_STATUS_INOP;
	private_data->recv_buf[0] = 0;
	op = buffer[0]; count = buffer[1]; reg = buffer[2] | (buffer[3] << 8);
	//printk("op=%d count=%d reg=%d", op, count, reg);
	switch (op) {
		case UNIPI_MODBUS_OP_READREG:
			ret = unipi_read_regs_async(channel, reg, count, private_data->recv_buf,
										private_data, unipi_modbus_op_callback);
			break;
		case UNIPI_MODBUS_OP_READBIT:
			ret = unipi_read_bits_async(channel, reg, count, private_data->recv_buf,
										private_data, unipi_modbus_op_callback);
			break;
		case UNIPI_MODBUS_OP_WRITEREG:
			simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
								&dummy_offset, buffer+UNIPI_MODBUS_HEADER_SIZE,
								len-UNIPI_MODBUS_HEADER_SIZE);
			ret = unipi_write_regs_async(channel, reg, count, private_data->send_buf,
										private_data, unipi_modbus_op_callback);
			break;
		case UNIPI_MODBUS_OP_WRITEBITS:
			simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
							   &dummy_offset, buffer+UNIPI_MODBUS_HEADER_SIZE,
							   len-UNIPI_MODBUS_HEADER_SIZE);
			ret = unipi_write_bits_async(channel, reg, count, private_data->send_buf,
	                                     private_data, unipi_modbus_op_callback);
			break;
		default:
			ret = -ENOENT;
	}
	//printk("op=%d count=%d reg=%d ret=%d", op, count, reg, ret);
	if (ret != 0) {
		private_data->status = UNIPI_MODBUS_STATUS_IDLE;
		goto unlock;
	}
	if (file_p->f_flags & O_NONBLOCK) {
		goto unlock;
	}
	wait_event_interruptible(private_data->wait,
                             (private_data->status!=UNIPI_MODBUS_STATUS_INOP));
	if (op==UNIPI_MODBUS_OP_WRITEBITS && count==1) 
		ret = (private_data->result == 0) ? 1 : private_data->result;
	else
		/* OK / Bad register / error in comm */
		ret = (private_data->result >= count)? count : private_data->result;
unlock:
	mutex_unlock(&private_data->lock);
	unipi_modbus_trace(KERN_INFO "unipi-modbus: CDEV Write ret:%d\n", ret);
	return ret;
}


int unipi_modbus_lock (struct file *file_p, int cmd, struct file_lock *flock)
{
	/* cmd set_of(F_OFD_GETLK, F_OFD_SETLK, F_OFD_SETLKW) */
	struct unipi_modbus_file_data *private_data;

	if (flock->fl_type == F_UNLCK) {
		// Unlock device
		private_data = (struct unipi_modbus_file_data*) file_p->private_data;
		mutex_lock(&(private_data->lock));
		private_data->channel->op->unlock_op(private_data->channel->proto_self);
		private_data->cookie = 0;
		private_data->is_locked = 0;
		mutex_unlock(&(private_data->lock));

	} else if ((flock->fl_type == F_WRLCK)||(flock->fl_type == F_RDLCK)||(flock->fl_type == F_EXLCK)) {
		// Lock device
		private_data = (struct unipi_modbus_file_data*) file_p->private_data;
		mutex_lock(&(private_data->lock));
		private_data->cookie = private_data->channel->op->lock_op(private_data->channel->proto_self);
		private_data->is_locked = private_data->cookie != 0;
		mutex_unlock(&(private_data->lock));
		if (!private_data->is_locked) return -ETXTBSY;
	}
	return 0;
}

struct file_operations file_ops =
{
	.open 				= unipi_modbus_open,
	.read 				= unipi_modbus_read,
	.write 				= unipi_modbus_write,
	.poll 				= unipi_modbus_poll,
	.release 			= unipi_modbus_release,
	.flock 				= unipi_modbus_lock,
	.lock 				= unipi_modbus_lock,
	.owner				= THIS_MODULE
};


int __init unipi_modbus_init(void)
{
	int major, rc = 0;

	// Character device registration
	major = register_chrdev(0, UNIPI_MODBUS_DEVICE_NAME, &file_ops);
	if (major < 0){
		printk(KERN_ALERT "unipi-modbus: Failed to register chrdev\n");
		return major;
	}

	// Character class registration
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,3,0)
	unipi_modbus_class = class_create(THIS_MODULE, UNIPI_MODBUS_DEVICE_CLASS);
#else
	unipi_modbus_class = class_create(UNIPI_MODBUS_DEVICE_CLASS);
#endif
	if (IS_ERR(unipi_modbus_class)) {
		printk(KERN_ALERT "unipi-modbus: Failed to register device class\n");
		rc = PTR_ERR(unipi_modbus_class);
		goto err_class;
	}

	unipi_modbus_major = major;
	return 0;

err_class:
	unregister_chrdev(major, UNIPI_MODBUS_DEVICE_NAME);
	return rc;
}

void __exit unipi_modbus_exit(void)
{
	class_destroy(unipi_modbus_class); 
	unregister_chrdev(unipi_modbus_major, UNIPI_MODBUS_DEVICE_NAME);
}

module_init(unipi_modbus_init);
module_exit(unipi_modbus_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi SPI Modbus Character Device Driver");
