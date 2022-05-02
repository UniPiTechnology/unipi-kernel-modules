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

#include "unipi_common.h"
#include "unipi_modbus.h"
#include "unipi_spi.h"


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


struct unipi_modbus_file_data
{
	//struct spi_device** spi_device;
	struct mutex 		lock;
	struct spi_device*	spi_dev;
	int					is_locked;
	u32					cookie;
	u8					recv_buf[UNIPI_MODBUS_BUFFER_MAX];
	u8					send_buf[UNIPI_MODBUS_BUFFER_MAX];
	int					result;
};


/*
int unipi_modbus_match_modbus_address(struct device *dev, const void *modbus_address)
{
	struct spi_device* spi_dev = dev_get_drvdata(dev);
	struct unipi_spi_device* n_spi = (struct unipi_spi_device*) spi_get_drvdata(spi_dev);
	return n_spi->mfd.modbus_address == *((u8*)modbus_address);
}
*/

struct spi_device * unipi_modbus_dev_by_address(u8 modbus_address)
{
	struct device *dev;
	struct spi_device *spi_dev;
	
	//dev = class_find_device(unipi_modbus_class, NULL, &modbus_address, unipi_modbus_match_modbus_address);
	dev = class_find_device_by_devt(unipi_modbus_class, MKDEV(unipi_modbus_major, modbus_address));
	if (dev) {
		spi_dev = (struct spi_device*) dev_get_drvdata(dev);
		put_device(dev);
		return spi_dev;
 	}
	return NULL;
}
EXPORT_SYMBOL_GPL(unipi_modbus_dev_by_address);

struct device* unipi_modbus_classdev_register(struct spi_device *spi_dev, u8 address)
{
	struct device* modbus_dev;

	if (unipi_modbus_dev_by_address(address)) {
		dev_err(&spi_dev->dev,"Duplicit modbus-address (%d). Channel inaccessible!", address);
		return NULL;
	}

	modbus_dev = device_create(unipi_modbus_class, &spi_dev->dev, MKDEV(unipi_modbus_major, address),
	                           spi_dev, UNIPI_MODBUS_DEVICE_NAME_T, address);
	if (IS_ERR(modbus_dev)) {
		dev_err(&spi_dev->dev,"Failed to create modbus device (%d). Channel inaccessible!", address);
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

	private_data->spi_dev = unipi_modbus_dev_by_address(modbus_address);
	/* private_data->spi_dev = (struct spi_device*) inode_p->i_private; */
	if (private_data->spi_dev == NULL) {
		kfree(private_data);
		return -ENODEV;
	}
	printk("open %s", dev_name(&private_data->spi_dev->dev));
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
	kfree(private_data);
	file_p->private_data = NULL;
	return 0;
}

ssize_t unipi_modbus_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{
	s32 result = 0;
    loff_t dummy_offset = 0;
	struct unipi_modbus_file_data* private_data;

	if (buffer == NULL) return -EINVAL; // Invalid read buffer
	if (len == 0) return 0; // Empty read
	if (len > UNIPI_MODBUS_BUFFER_MAX) return -EMSGSIZE;
	if (file_p == NULL || file_p->private_data == NULL) {
		printk(KERN_DEBUG "unipi-modbus: Bad file descriptor\n");
		return -EINVAL;
	}
	private_data = (struct unipi_modbus_file_data*) file_p->private_data;

	mutex_lock(&(private_data->lock));
	unipi_modbus_trace(KERN_INFO "unipi-modbus: CDEV Read %zu, offset=%d\n", len,
					   (int)*offset);

	result = simple_read_from_buffer(buffer, len, &dummy_offset, private_data->recv_buf,
	                                 UNIPI_MODBUS_BUFFER_MAX);
	mutex_unlock(&(private_data->lock));
	return result;
}

struct unipi_modbus_cb_data
{
	struct completion* done;
	int		result;
};

void unipi_modbus_op_callback(void* cb_data, int result, u8* recv)
{
	struct unipi_modbus_cb_data* unipi_cb_data = (struct unipi_modbus_cb_data*) cb_data;
	if (cb_data) {
		unipi_cb_data->result = result;
		complete(unipi_cb_data->done);
	}
}

ssize_t unipi_modbus_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
    loff_t dummy_offset = 0;
	struct unipi_modbus_file_data *private_data;
	struct spi_device *spi;
	struct unipi_modbus_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;
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

    spi = private_data->spi_dev;
	spi_dev_get(spi);

	mutex_lock(&(private_data->lock));
    private_data->recv_buf[0] = 0;
#if UNIPI_MODBUS_DETAILED_DEBUG > 1
	// clear receive buffer content
	memset(private_data->recv_buf, 0, UNIPI_MODBUS_BUFFER_MAX);
#endif
	if (private_data->is_locked) {
		/* Firmware op */
		simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
							   &dummy_offset, buffer, len);
		ret = unipi_spi_firmware_op(spi, private_data->send_buf, private_data->recv_buf, 
		                            len, private_data->cookie);
	} else {
		op = buffer[0]; count = buffer[1]; reg = buffer[2] | (buffer[3] << 8);
		printk("op=%d count=%d reg=%d", op, count, reg);
		init_completion(&done);
		cb_data.done = &done;
		switch (op) {
			case UNIPI_MODBUS_OP_READREG:
				ret = unipi_spi_read_regs_async(spi, reg, count, private_data->recv_buf,
												&cb_data, unipi_modbus_op_callback);
				break;
			case UNIPI_MODBUS_OP_READBIT:
				ret = unipi_spi_read_bits_async(spi, reg, count, private_data->recv_buf,
												&cb_data, unipi_modbus_op_callback);
				break;
			case UNIPI_MODBUS_OP_WRITEREG:
				simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
									&dummy_offset, buffer+UNIPI_MODBUS_HEADER_SIZE,
									len-UNIPI_MODBUS_HEADER_SIZE);
				ret = unipi_spi_write_regs_async(spi, reg, count, private_data->send_buf,
												&cb_data, unipi_modbus_op_callback);
				break;
			case UNIPI_MODBUS_OP_WRITEBITS:
				simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
									   &dummy_offset, buffer+UNIPI_MODBUS_HEADER_SIZE,
									   len-UNIPI_MODBUS_HEADER_SIZE);
				ret = unipi_spi_write_bits_async(spi, reg, count, private_data->send_buf,
			                                     &cb_data, unipi_modbus_op_callback);
				break;
			default:
				ret = -ENOENT;
		}
		printk("op=%d count=%d reg=%d ret=%d", op, count, reg, ret);
		if (ret == 0) {
			wait_for_completion(&done);
			printk("after cb=%d\n", cb_data.result);
			if (op==UNIPI_MODBUS_OP_WRITEBITS && count==1) 
				ret = (cb_data.result == 0) ? 1 : cb_data.result;
			else
				/* OK / Bad register / error in comm */
				ret =(cb_data.result >= count)? count : cb_data.result;
				//ret =(cb_data.result >= count)? 0 : (cb_data.result >= 0)? -ENOENT: cb_data.result;
		}
	}
	spi_dev_put(spi);
	mutex_unlock(&private_data->lock);
	return ret;
	//return (ret < 0) ? ret : len;
}


int unipi_modbus_lock (struct file *file_p, int cmd, struct file_lock *flock)
{
	/* cmd set_of(F_OFD_GETLK, F_OFD_SETLK, F_OFD_SETLKW) */
	struct unipi_modbus_file_data *private_data;

	if (flock->fl_type == F_UNLCK) {
		// Unlock device
		private_data = (struct unipi_modbus_file_data*) file_p->private_data;
		mutex_lock(&(private_data->lock));
		unipi_spi_unlock(private_data->spi_dev);
		private_data->cookie = 0;
		private_data->is_locked = 0;
		mutex_unlock(&(private_data->lock));

	} else if ((flock->fl_type == F_WRLCK)||(flock->fl_type == F_RDLCK)||(flock->fl_type == F_EXLCK)) {
		// Lock device
		private_data = (struct unipi_modbus_file_data*) file_p->private_data;
		mutex_lock(&(private_data->lock));
		private_data->cookie = unipi_spi_lock(private_data->spi_dev);
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
	.release 			= unipi_modbus_release,
	.flock 				= unipi_modbus_lock,
	.lock 				= unipi_modbus_lock,
	.owner				= THIS_MODULE
};


int __init unipi_modbus_init(void)
{
    int major, rc = 0;
	//if (unipi_modbus_major >= 0) return 0;

	// Character device registration
	major = register_chrdev(0, UNIPI_MODBUS_DEVICE_NAME, &file_ops);
	if (major < 0){
	   printk(KERN_ALERT "unipi-modbus: Failed to register chrdev\n");
	   return major;
	}

	// Character class registration
	unipi_modbus_class = class_create(THIS_MODULE, UNIPI_MODBUS_DEVICE_CLASS);
	if (IS_ERR(unipi_modbus_class)) {
		printk(KERN_ALERT "unipi-modbus: Failed to register device class\n");
		rc = PTR_ERR(unipi_modbus_class);
		goto err_class;
	}

    unipi_modbus_major = major;
	//printk(KERN_INFO "unipi-modbus: Modbus module loaded\n");
	return 0;

err_class:
	unregister_chrdev(major, UNIPI_MODBUS_DEVICE_NAME);
	return rc;
}

void __exit unipi_modbus_exit(void)
{
	//unipi_spi_trace(KERN_INFO "UNIPISPI: Open Counter is %d\n", neuronspi_cdrv.open_counter);
    //if (unipi_modbus_major < 0) return;

	class_destroy(unipi_modbus_class); 
	unregister_chrdev(unipi_modbus_major, UNIPI_MODBUS_DEVICE_NAME);
	//printk(KERN_INFO "unipi-modbus: Modbus module unloaded\n");
}

/*
module_init(unipi_modbus_init);
module_exit(unipi_modbus_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi SPI Modbus Character Device Driver");
*/