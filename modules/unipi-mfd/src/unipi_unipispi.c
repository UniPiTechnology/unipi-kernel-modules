/*
 * UniPi PLC modbus common channel driver - Copyright (C) 2018 UniPi Technology
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

#include <linux/version.h>

#include "unipi_common.h"
#include "unipi_spi.h"
#include "unipi_modbus.h"

#if UNIPI_MODBUS_DETAILED_DEBUG > 1
# define unipi_unipispi_trace_1(f, args...)	printk(f, ##args)
#else
# define unipi_unipispi_trace_1(f, args...)
#endif

#if UNIPI_MODBUS_DETAILED_DEBUG > 0
# define unipi_unipispi_trace(f, args...)	printk(f, ##args)
#else
# define unipi_unipispi_trace(f, args...)
#endif


#define UNIPI_UNIPISPI_DEVICE_NAME		"unipispi"
#define UNIPI_UNIPISPI_DEVICE_CLASS		"unipispi"
#define UNIPI_UNIPISPI_OPTION_HEADER_LEN  10


int				unipi_unipispi_major;
struct class	*unipi_unipispi_class;
struct device	*unipi_unipispi_dev;
u32				open_counter = 0;



struct unipi_unipispi_file_data
{
	struct mutex 		lock;
	u32			        message_len;
	u8					reservation;
	int					cookie;
	u8					device_index;
	u8					has_first_message;
	u8					recv_buf[UNIPI_MODBUS_BUFFER_MAX];
	u8					send_buf[UNIPI_MODBUS_BUFFER_MAX];
};


/*****************************************************************
 *  Modbus like Interface via /dev/unipispi 
 */
int unipi_unipispi_open (struct inode *inode_p, struct file *file_p)
{
	struct unipi_unipispi_file_data *private_data;
	open_counter += 1;
	private_data = kzalloc(sizeof(*private_data), GFP_ATOMIC);
	mutex_init(&private_data->lock);
	file_p->private_data = private_data;
	return 0;
}

int unipi_unipispi_release (struct inode *inode_p, struct file *file_p)
{
	struct unipi_unipispi_file_data *private_data;
	private_data = (struct unipi_unipispi_file_data*)file_p->private_data;
	kfree(private_data);
	file_p->private_data = NULL;
	open_counter -= 1;
	return 0;
}

const u8 dummy_idle_message[UNIPI_SPI_FIRST_MESSAGE_LENGTH] = {0xfa, 0x00, 0x55, 0x0e, 0xb6, 0x0a };

ssize_t unipi_unipispi_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{
	//spi_dev_put

	s32 result = 0;
    loff_t dummy_offset = 0;
	struct unipi_unipispi_file_data* private_data;

	if (buffer == NULL) return -EINVAL; // Invalid read buffer
	if (len == 0) return 0; // Empty read
	if (len > 4095) return -EMSGSIZE;
	if (file_p == NULL || file_p->private_data == NULL) {
		printk(KERN_DEBUG "unipi-unipispi: Bad file descriptor\n");
		return -EINVAL;
	}
	private_data = (struct unipi_unipispi_file_data*) file_p->private_data;

    mutex_lock(&(private_data->lock));
    unipi_unipispi_trace(KERN_INFO "unipi-unipispi: CDEV Read %zu, msglen=%d offset=%d\n", len,
                    private_data->message_len, (int)*offset);
            
	if (private_data->has_first_message & UNIPISPI_OP_MODE_SEND_HEADER) {
		result = simple_read_from_buffer(buffer, len, &dummy_offset, dummy_idle_message,
		                                 UNIPI_SPI_FIRST_MESSAGE_LENGTH);
		if (result >=0) {
			dummy_offset = 0;
			result = simple_read_from_buffer(buffer+result, len - result, &dummy_offset,
									private_data->recv_buf, private_data->message_len);
        }
	} else {
   		result = simple_read_from_buffer(buffer, len, &dummy_offset, private_data->recv_buf,
		                                 private_data->message_len);
	}
	if (result >= 0) {
		if (result + UNIPI_UNIPISPI_OPTION_HEADER_LEN <= len) result = len;
	}

	mutex_unlock(&(private_data->lock));
    unipi_unipispi_trace(KERN_INFO "unipi-unipispi: CDEV Read ret=%d data=%16ph\n", result, private_data->recv_buf);
	return result;
}


ssize_t unipi_unipispi_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
	u8 device_index = 0;
	u8  reservation, send_header;
    loff_t dummy_offset = 0;
    size_t datalen;
	struct unipi_unipispi_file_data *private_data;
	struct spi_device *spi_dev;
	//struct unipi_spi_device *n_spi;
	int ret;

	unipi_unipispi_trace(KERN_INFO "unipi-unipispi: CDEV Write len:%zu dev=%d\n", len, buffer[0]);
	if ((buffer == NULL) || (len == 0)) {
		return 0; // Void write
	}

    if ((len > 4095) || (len < UNIPI_UNIPISPI_OPTION_HEADER_LEN + UNIPI_SPI_FIRST_MESSAGE_LENGTH))
		return -EMSGSIZE;

    if (file_p == NULL || file_p->private_data == NULL) {
    	printk(KERN_DEBUG "unipi-unipispi: Bad file descriptor\n");
    	return -EINVAL;
    }
    private_data = (struct unipi_unipispi_file_data*) file_p->private_data;

    // Read packet header and initialise private data (dependent on each other)
    device_index = buffer[0];
    send_header  = buffer[3];
    //frequency  = (buffer[4] << 8 | buffer[5]) * 1000;
   	//delay      = buffer[6];
    reservation  = buffer[7]; // Device reservation

    buffer += UNIPI_UNIPISPI_OPTION_HEADER_LEN;
    datalen = len - UNIPI_UNIPISPI_OPTION_HEADER_LEN;
    
	/* correction for old arm_spi implementation - started from zero */
	if (device_index < 3) device_index++;

    spi_dev = unipi_modbus_dev_by_address(device_index);
    if (spi_dev == NULL) 
		return -ENODEV;

	if (reservation == 255) {
		// Unlock device
		unipi_spi_unlock(spi_dev);
		reservation = 0;
		private_data->reservation = 0;
	} else if (reservation != private_data->reservation) {
		if (private_data->reservation == 0) {
			private_data->cookie = unipi_spi_lock(spi_dev);
			if (private_data->cookie == 0)
				return -ETXTBSY;
			private_data->reservation = reservation;
		} else {
			return -ETXTBSY;
		}
	}

	mutex_lock(&(private_data->lock));
	private_data->device_index = device_index;
	private_data->has_first_message = send_header;
    private_data->recv_buf[0] = 0;
#if UNIPI_MODBUS_DETAILED_DEBUG > 1
    // clear receive buffer content
    memset(private_data->recv_buf, 0, UNIPI_MODBUS_BUFFER_MAX );
#endif
	if (reservation != 0) { //(!(send_header & UNIPISPI_OP_MODE_SEND_HEADER)) {
		/* Firmware op */
		private_data->message_len = datalen-2; // correction for crc
		simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX,
		                       &dummy_offset, buffer, datalen);
		unipi_spi_firmware_op(spi_dev, private_data->send_buf, private_data->recv_buf, 
		                      private_data->message_len, private_data->cookie);
		mutex_unlock(&private_data->lock);
		return len;
	}

	private_data->message_len = datalen - UNIPI_SPI_FIRST_MESSAGE_LENGTH;
	if (private_data->message_len <= 0) {
		private_data->message_len = UNIPI_SPI_MESSAGE_HEADER_LENGTH;
		simple_write_to_buffer(private_data->send_buf, UNIPI_SPI_FIRST_MESSAGE_LENGTH,
		                       &dummy_offset, buffer, datalen);
	} else {
		simple_write_to_buffer(private_data->send_buf, UNIPI_MODBUS_BUFFER_MAX, &dummy_offset, 
		                       buffer+UNIPI_SPI_FIRST_MESSAGE_LENGTH, private_data->message_len);
		/* Fix missing info */
		if (private_data->send_buf[0]==UNIPI_MODBUS_OP_READREG) {
			/* fix regcount */
			private_data->send_buf[1] = (private_data->message_len-UNIPI_SPI_MESSAGE_HEADER_LENGTH)/2-1;
		}
	}
	if (private_data->send_buf[0] == UNIPI_MODBUS_OP_READSTR) {
		// op read string is not allowed here
		ret = -EACCES;
	} else {
		ret = unipi_spi_send_op(spi_dev, private_data->send_buf, private_data->recv_buf, private_data->message_len);
	}
	mutex_unlock(&private_data->lock);
	return (ret < 0) ? ret : len;
}


struct file_operations file_ops =
{
	.open 				= unipi_unipispi_open,
	.read 				= unipi_unipispi_read,
	.write 				= unipi_unipispi_write,
	.release 			= unipi_unipispi_release,
	.owner				= THIS_MODULE
};


static int __init unipi_unipispi_init(void)
{
    int major, rc = 0;

	// Character device registration
	major = register_chrdev(0, UNIPI_UNIPISPI_DEVICE_NAME, &file_ops);
	if (major < 0){
		printk(KERN_ALERT "unipi-unipispi: Failed to register unipispi chrdev\n");
		return major;
	}

	// Character class registration
	unipi_unipispi_class = class_create(THIS_MODULE, UNIPI_UNIPISPI_DEVICE_CLASS);
	if (IS_ERR(unipi_unipispi_class)) {
		printk(KERN_ALERT "unipi-unipispi: Failed to register device class\n");
		rc = PTR_ERR(unipi_unipispi_class);
		goto err_class;
	}

	// Device driver registration
	unipi_unipispi_dev = device_create(unipi_unipispi_class, NULL, MKDEV(major, 0),
	                                 NULL, UNIPI_UNIPISPI_DEVICE_NAME);
	if (IS_ERR(unipi_unipispi_dev)) {
        printk(KERN_ALERT "unipi-unipispi: Failed to create the device\n");
        rc = PTR_ERR(unipi_unipispi_dev);
		goto err_dev;
	}
    unipi_unipispi_major = major;
	return 0;

err_dev:
	class_destroy(unipi_unipispi_class);
err_class:
	unregister_chrdev(major, UNIPI_UNIPISPI_DEVICE_NAME);
	return rc;
}

static void __exit unipi_unipispi_exit(void)
{
	device_unregister(unipi_unipispi_dev);
	class_destroy(unipi_unipispi_class); 
	unregister_chrdev(unipi_unipispi_major, UNIPI_UNIPISPI_DEVICE_NAME);
}


module_init(unipi_unipispi_init);
module_exit(unipi_unipispi_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Modbus Common Access Character Device Driver");
