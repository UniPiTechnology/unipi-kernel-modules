/*
 * UniPi Neuron tty serial driver - Copyright (C) 2017 UniPi Tech
nologies
 * Author: Tomas Knot <tomasknot@gmail.com>
 *
 *  Based on the SC16IS7xx driver by Jon Ringle <jringle@gridpoint.com>,
 *  which was in turn based on max310x.c, by Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/************
 * Includes *
 ************/

#include "unipi_common.h"
#include "unipi_sysfs.h"
#include "unipi_uart.h"
#include "unipi_platform.h"
#include "unipi_gpio.h"
#include "unipi_iio.h"
#include "unipi_misc.h"
#include "unipi_spi.h"
#include "unipi_tty.h"

/********************
 * Data Definitions *
 ********************/

MODULE_DEVICE_TABLE(of, neuronspi_id_match);

struct spi_driver neuronspi_spi_driver =
{
	.driver =
	{
		.name			= NEURON_DRIVER_NAME,
		.of_match_table	= of_match_ptr(neuronspi_id_match)
	},
	.probe				= neuronspi_spi_probe,
	.remove				= neuronspi_spi_remove,
};

struct file_operations file_ops =
{
	.open 				= neuronspi_open,
	.read 				= neuronspi_read,
	.write 				= neuronspi_write,
	.release 			= neuronspi_release,
	.owner				= THIS_MODULE
};

struct neuronspi_char_driver neuronspi_cdrv =
{
	.dev = NULL
};

struct mutex neuronspi_master_mutex;
struct mutex unipi_inv_speed_mutex;
struct spinlock *neuronspi_spi_w_spinlock;
int neuronspi_model_id = -1;
struct spi_device *neuronspi_s_dev[NEURONSPI_MAX_DEVS];
struct task_struct *neuronspi_invalidate_thread;

static u8 neuronspi_spi_w_flag = 1;
static u8 neuronspi_probe_count = 0;
static struct spinlock *neuronspi_probe_spinlock;
static struct sched_param neuronspi_sched_param = { .sched_priority = MAX_RT_PRIO / 2 };

/************************
 * Non-static Functions *
 ************************/

int neuronspi_open (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (neuronspi_s_dev == NULL || file_p == NULL || inode_p == NULL) {
		return -1;
	}
	neuronspi_cdrv.open_counter += 1;
	f_internal_data = kzalloc(sizeof(*f_internal_data), GFP_ATOMIC);
	f_internal_data->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	f_internal_data->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	f_internal_data->spi_device = neuronspi_s_dev;
	mutex_init(&f_internal_data->lock);
	file_p->private_data = f_internal_data;
	return 0;
}

int neuronspi_release (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (file_p == NULL) {
		return -1;
	}
	f_internal_data = (struct neuronspi_file_data*)file_p->private_data;
	f_internal_data->spi_device = NULL;
	kfree(f_internal_data->recv_buf);
	f_internal_data->recv_buf = NULL;
	kfree(f_internal_data->send_buf);
	f_internal_data->send_buf = NULL;
	kfree(f_internal_data);
	file_p->private_data = NULL;
	neuronspi_cdrv.open_counter -= 1;
	return 0;
}

ssize_t neuronspi_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{

	s32 result = 0;
	u8 device_index = 0;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	// Sanity checking
	if (neuronspi_cdrv.open_counter == 0) {
		neuronspi_cdrv.open_counter = 1;
	}
	if (buffer == NULL) return -7; // Invalid read buffer
    if (len == 0) return result; // Empty read
    if (len > 4095) return -EMSGSIZE;
    if (file_p == NULL) {
    	printk(KERN_DEBUG "NEURONSPI: File Pointer is NULL\n");
    	return -8;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_DEBUG "NEURONSPI: No Private Data\n");
    	return -1;	// No private data
    }
    private_data = (struct neuronspi_file_data*) file_p->private_data;
    if (private_data == NULL) return -4;
    device_index = private_data->send_buf[0];
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    if (spi_driver_data == NULL) return -2;
    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data == NULL) return -2;
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if (driver_data->first_probe_reply[0] == 0) return -3; // No device present
    mutex_lock(&(private_data->lock));
    if (private_data->recv_buf == NULL) {
    	mutex_unlock(&(private_data->lock));
    	return -10;
    }
#if NEURONSPI_DETAILED_DEBUG > 0
    printk(KERN_INFO "NEURONSPI: Device read %d DEV:%s%d DRV:%s%d\n", private_data->message_len, (spi_driver_data->dev.of_node->name),
    		(spi_driver_data->chip_select), (driver_data->spi_driver->driver.name), (device_index));
#endif
    if ((((s32)len) == private_data->message_len + 10)) {
    	memcpy(buffer, private_data->recv_buf, len);
    	result = len;
    } else if (private_data->message_len == 0) {
    	mutex_unlock(&(private_data->lock));
    	return -9;
    } else if (len <= private_data->message_len) {
    	result = simple_read_from_buffer(buffer, len, offset, private_data->recv_buf, len);
    } else {
    	mutex_unlock(&(private_data->lock));
    	return -9;
    }
    memset(private_data->recv_buf, 0, NEURONSPI_BUFFER_MAX);
	mutex_unlock(&(private_data->lock));
	return result;
}



ssize_t neuronspi_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
	u8 device_index = 0;
	s32 result = 0;
	u32 frequency = NEURONSPI_COMMON_FREQ;
	s32 transmit_len = len - NEURONSPI_HEADER_LENGTH;
	s32 send_header = 0;
	s32 delay = 25;
	unsigned long flags;
	struct neuronspi_file_data* private_data;
	struct spi_device* spi_driver_data;
	struct neuronspi_driver_data* driver_data;
	// Sanity checking
	if (neuronspi_cdrv.open_counter == 0) {
		neuronspi_cdrv.open_counter = 1;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: LENGTH:%d\n", len);
#endif
	if (buffer == NULL) {
		return 0; // Void write
	}
    if (len == 0) {
    	return result; // Empty write
    }
    if (len > 4095) return -EMSGSIZE;
    if (file_p == NULL) {
    	return -12;
    }
    if (file_p->private_data == NULL) {
    	printk(KERN_DEBUG "NEURONSPI: No Private Data\n");
    	return -1;	// No private data
    }
    // Read packet header and initialise private data (dependent on each other)
    device_index = buffer[0];
    if (device_index > NEURONSPI_MAX_DEVS - 1) return -2;
    private_data = (struct neuronspi_file_data*) file_p->private_data;
    spi_driver_data = private_data->spi_device[device_index];	// Get private (driver) data from FP
    if (spi_driver_data == NULL) return -2;
    driver_data = spi_get_drvdata(spi_driver_data);
    if (driver_data == NULL) return -2;
    if (driver_data->spi_driver == NULL) return -2;	// Invalid private data
    if (driver_data->first_probe_reply[0] == 0) return -3; // Device not present
    send_header = buffer[3];
    if (buffer[4]) {	// Frequency setting
    	frequency = (buffer[4] << 8 | buffer[5]) * 1000;
    }
    if (buffer[6]) {	// Delay setting
    	delay = buffer[6];
    }
    if (buffer[7]) {	// Device reservation
    	if (buffer[7] == 255) { // Unlock device
    		driver_data->reserved_device = 0;
    	} else if ((driver_data->reserved_device) && buffer[7] != driver_data->reserved_device) {
    		return -4;				// Another device reserved
    	} else if (!driver_data->reserved_device) {
    		driver_data->reserved_device = buffer[7];	// Reserve the device
    	}
#ifdef STRICT_RESERVING
    } else if (driver_data->reserved_device) {
    	return -5;			// Device reserved
    }
    if (driver_data->slower_model && frequency > NEURONSPI_SLOWER_FREQ) {
    	frequency = NEURONSPI_SLOWER_FREQ;
    }
#else
	} else if (device_index == (driver_data->reserved_device - 1)) {
		return -5;			// Device reserved
	}
	if (driver_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#endif
    mutex_lock(&(private_data->lock));
    memset(private_data->send_buf, 0, NEURONSPI_BUFFER_MAX );
    memcpy(private_data->send_buf, buffer, len);
    memset(private_data->recv_buf, 0, NEURONSPI_BUFFER_MAX );
    private_data->message_len = transmit_len;
    spin_lock_irqsave(neuronspi_spi_w_spinlock, flags);
    neuronspi_spi_w_flag = 1;
    spin_unlock_irqrestore(neuronspi_spi_w_spinlock, flags);
    neuronspi_spi_send_message(spi_driver_data, &private_data->send_buf[NEURONSPI_HEADER_LENGTH], private_data->recv_buf,
    		transmit_len, frequency, delay, send_header, buffer[7]);
    mutex_unlock(&private_data->lock);
    return len;
}

s32 neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, u8 length, u8 uart_index)
{
	u8 *message_buf;
	u8 *recv_buf;
	s32 transmit_len, i;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	u16 crc1, crc2;
	u32 frequency = NEURONSPI_COMMON_FREQ;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART SPI Write, dev:%d, len:%d\n", uart_index, length);
#endif
	if (length == 0) {
		return -1;
	}
	if (length == 1) {
		transmit_len = 6;
		message_buf = kzalloc(transmit_len, GFP_ATOMIC);
		memcpy(message_buf, NEURONSPI_SPI_UART_SHORT_MESSAGE, NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN);
		message_buf[1] = send_buf[0];
		message_buf[3] = uart_index;
		crc1 = neuronspi_spi_crc(message_buf, 4, 0);
		memcpy(&message_buf[4], &crc1, 2);
	} else {
		transmit_len = 6 + length + 2;
		message_buf = kzalloc(transmit_len, GFP_ATOMIC);
		memcpy(message_buf, NEURONSPI_SPI_UART_LONG_MESSAGE, NEURONSPI_SPI_UART_LONG_MESSAGE_LEN);
		message_buf[1] = length;
		message_buf[3] = uart_index;
		crc1 = neuronspi_spi_crc(message_buf, 4, 0);
		memcpy(&message_buf[4], &crc1, 2);
		for (i = 0; i < length; i++) {
			message_buf[6 + i] = send_buf[i];
		}
		crc2 = neuronspi_spi_crc(&message_buf[6], length, crc1);
		memcpy(&message_buf[6+length], &crc2, 2);
	}
	recv_buf = kzalloc(transmit_len, GFP_ATOMIC);
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi, message_buf, recv_buf, transmit_len, frequency, 65, 1, 0);
	}
	kfree(message_buf);
	kfree(recv_buf);
	return 0;
}


void neuronspi_spi_uart_read(struct spi_device* spi, u8 *send_buf, u8 *recv_buf, s32 len, u8 uart_index)
{
	s32 transmit_len;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	u16 crc1, crc2;
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART SPI Read, cs:%d, len:%d\n", uart_index, len);
#endif
	if (len <= 2) {
		memcpy(send_buf, NEURONSPI_SPI_UART_READ_MESSAGE, NEURONSPI_SPI_UART_READ_MESSAGE_LEN);
		transmit_len = NEURONSPI_SPI_UART_READ_MESSAGE_LEN;
	} else {
		memcpy(send_buf, NEURONSPI_SPI_UART_READ_MESSAGE, NEURONSPI_SPI_UART_READ_MESSAGE_LEN);
		if (len < 100) {
			len = (len * 2) + 1;
		} else {
			len = 201;
		}
		transmit_len = 5 + len + 6;	// Header (-1 for the byte there) + 4 bytes in second part + 2 bytes of CRC
		send_buf[1] = len + 3;	// Length of second part (len + 4 - 1)

		crc1 = neuronspi_spi_crc(send_buf, 4, 0);
		memcpy(&send_buf[4], &crc1, 2);
		send_buf[7] = len;
		crc2 = neuronspi_spi_crc(&send_buf[6], len + 3, crc1);
		memcpy(&send_buf[len + 9], &crc2, 2);
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: UART Device Read len:%d %100ph\n", transmit_len, send_buf);
#endif
	}
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi, send_buf, recv_buf, transmit_len, frequency, 65, 1, 0);
	}
}

void neuronspi_spi_set_irqs(struct spi_device* spi_dev, u16 to)
{
	u8 *message_buf;
	u8 *recv_buf;
	u16 crc1, crc2;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI IRQ Set, Dev-CS:%d, to:%d\n", spi_dev->chip_select, to);
#endif
	message_buf = kzalloc(NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, GFP_ATOMIC);
	recv_buf = kzalloc(NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, GFP_ATOMIC);
	memcpy(message_buf, NEURONSPI_SPI_IRQ_SET_MESSAGE, NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, frequency, 65, 1, 0);
	}
	kfree(message_buf);
	kfree(recv_buf);
}

void neuronspi_spi_uart_set_cflag(struct spi_device* spi_dev, u8 port, u32 to)
{
	u8 *message_buf;
	u8 *recv_buf;
	u16 crc1, crc2;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS Set, Dev-CS:%d, to:%x\n", spi_dev->chip_select, to);
#endif
	message_buf = kzalloc(NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, GFP_ATOMIC);
	recv_buf = kzalloc(NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, GFP_ATOMIC);
	memcpy(message_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	memcpy(&message_buf[10], &to, 4);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 65, 1, 0);
	}
	kfree(message_buf);
	kfree(recv_buf);
}

void neuronspi_spi_uart_set_ldisc(struct spi_device* spi_dev, u8 port, u8 to)
{
	u8 *message_buf;
	u8 *recv_buf;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 1
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS Set, Dev-CS:%d, to:%x\n", spi_dev->chip_select, to);
#endif
	neuronspi_spi_compose_single_register_write(503, &message_buf, &recv_buf, to);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 35, 1, 0);
	}
	kfree(message_buf);
	kfree(recv_buf);
}

u8 neuronspi_spi_uart_get_ldisc(struct spi_device* spi_dev, u8 port)
{
	u8 *message_buf;
	u8 *recv_buf;
	u8 outp;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
	neuronspi_spi_compose_single_register_read(503, &message_buf, &recv_buf);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 35, 1, 0);
	}
	outp = recv_buf[MODBUS_FIRST_DATA_BYTE + 1];
#if NEURONSPI_DETAILED_DEBUG > 1
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS GET, Dev-CS:%d, to:%x\n", spi_dev->chip_select, outp);
#endif
	kfree(message_buf);
	kfree(recv_buf);
	return outp;
}


/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */
void neuronspi_spi_iio_sec_ai_read_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_l);
	sec_ai_val_m = ((((u32)sec_ai_val_h) << 25) | (((u32)sec_ai_val_l) << 9)) >> 16;
	sec_ai_exp = (sec_ai_val_h & 0x7F80) >> 7;

	*val = sec_ai_val_m | 0x00010000;
	if (142 - ((int)sec_ai_exp) <= 0) {
		*val = (*val << (((int)sec_ai_exp) - 142)) * 1000;
		*val2 = 1;
	} else {
		*val = *val * 1000;
		*val2 = 2 << (142 - sec_ai_exp);
	}

}

void neuronspi_spi_iio_sec_ai_read_current(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;

	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_l);
	sec_ai_val_m = ((((u32)sec_ai_val_h) << 25) | (((u32)sec_ai_val_l) << 9)) >> 16;
	sec_ai_exp = (sec_ai_val_h & 0x7F80) >> 7;
	*val = sec_ai_val_m | 0x00010000;
	if (142 - ((int)sec_ai_exp) <= 0) {
		*val2 = 1;
		*val = *val << (((int)sec_ai_exp) - 142);
	} else {
		*val2 = 2 << (142 - sec_ai_exp);
	}
}

void neuronspi_spi_iio_sec_ai_read_resistance(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_l);
	sec_ai_val_m = ((((u32)sec_ai_val_h) << 25) | (((u32)sec_ai_val_l) << 9)) >> 16;
	sec_ai_exp = (sec_ai_val_h & 0x7F80) >> 7;
	*val = sec_ai_val_m | 0x00010000;
	if (142 - ((int)sec_ai_exp) <= 0) {
		*val2 = 1;
		*val = *val << (((int)sec_ai_exp) - 142);
	} else {
		*val2 = 2 << (142 - sec_ai_exp);
	}
}

void neuronspi_spi_iio_sec_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_true_val = (val * 2) / 5;
	if (val > 10000) val = 10000;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg + ao_data->index, sec_true_val);
}

/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */
void neuronspi_spi_iio_stm_ai_read_voltage(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(iio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_ai_val = 0;
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_v_err = 0;
	u32 stm_v_off = 0;
	u64 stm_true_val = 0;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_val_reg, &stm_ai_val);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_vol_err, &stm_v_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_vol_off, &stm_v_off);
	stm_true_ref = ((u64)stm_v_int_ref) * 99000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = stm_true_ref * ((u64)(stm_ai_val * 1000));
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 4096);
	stm_true_val *= (10000 + stm_v_err);
	stm_true_val += stm_v_off;
	do_div(stm_true_val, 10000);
	*val = stm_true_val;
}

void neuronspi_spi_iio_stm_ai_read_current(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_ai_val = 0;
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_i_err = 0;
	u32 stm_i_off = 0;
	u64 stm_true_val = 0;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_val_reg, &stm_ai_val);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_curr_err, &stm_i_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_curr_off, &stm_i_off);
	stm_true_ref = ((u64)stm_v_int_ref) * 330000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = stm_true_ref * ((u64)(stm_ai_val * 1000));
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 4096);
	stm_true_val *= (10000 + stm_i_err);
	stm_true_val += stm_i_off;
	do_div(stm_true_val, 10000);
	*val = stm_true_val;
	*val2 = 1000;
}

void neuronspi_spi_iio_stm_ao_read_resistance(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_aio_val = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_aio_val_reg, &stm_aio_val);
	*val = stm_aio_val;
	*val2 = 10;
}


void neuronspi_spi_iio_stm_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_v_err = 0;
	u32 stm_v_off = 0;
	u64 stm_true_val = val;
	u64 stm_true_val_fraction = val2 / 100;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_err, &stm_v_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_off, &stm_v_off);
	stm_true_ref = ((u64)stm_v_int_ref) * (99000 + stm_v_err) * 1000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = ((stm_true_val * 10000) + (stm_true_val_fraction) - stm_v_off) * 4095;
	do_div(stm_true_ref, stm_v_inp_ref);
	stm_v_inp_ref = stm_true_ref;
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 10000);
	if (stm_true_val > 4095) stm_true_val = 4095;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg, (unsigned int) stm_true_val);
}

void neuronspi_spi_iio_stm_ao_set_current(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_i_err = 0;
	u32 stm_i_off = 0;
	u64 stm_true_val = val;
	u64 stm_true_val_fraction = val2 / 100;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_curr_err, &stm_i_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_curr_off, &stm_i_off);
	stm_true_ref = ((u64)stm_v_int_ref) * (330000 + stm_i_err) * 100;
	stm_v_inp_ref = stm_v_inp_ref * 1000;
	stm_true_val = (((stm_true_val * 10000) + (stm_true_val_fraction)) - stm_i_off) * 4095;
	do_div(stm_true_ref, stm_v_inp_ref);
	stm_v_inp_ref = stm_true_ref;
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 10);
	if (stm_true_val > 4095) stm_true_val = 4095;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg, (unsigned int)stm_true_val);
}

int neuronspi_spi_send_message(struct spi_device* spi_dev, u8 *send_buf, u8 *recv_buf, s32 len, s32 freq, s32 delay, s32 send_header, u8 lock_val)
{
	s32 i = 0;
	int ret_code = 0;
	u16 recv_crc1 = 0;
	u16 recv_crc2 = 0;
	u16 packet_crc = 0;
	s32 trans_count = (len / NEURONSPI_MAX_TX) + 3;	// number of transmissions
	struct spi_message *s_msg;
	struct neuronspi_driver_data *d_data;
    struct spi_transfer* s_trans;
	mutex_lock(&neuronspi_master_mutex);
    d_data = spi_get_drvdata(spi_dev);
	if (d_data != NULL && d_data->reserved_device && lock_val != d_data->reserved_device) {
		memset(&recv_buf, 0, len);
	} else {
		s_trans = kzalloc(sizeof(struct spi_transfer) * trans_count, GFP_ATOMIC);
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI Master Write, len:%d,\n %100ph\n", len, send_buf);
#endif
		if (!send_header) {
			trans_count -= 1;	// one less transmission as the header is omitted
		}
		s_msg = kmalloc(sizeof(struct spi_message), GFP_ATOMIC);
		spi_message_init(s_msg);
		for (i = 0; i < trans_count; i++) {
			memset(&(s_trans[i]), 0, sizeof(s_trans[i]));
			s_trans[i].delay_usecs = 0;
			s_trans[i].bits_per_word = 8;
			s_trans[i].speed_hz = freq;
			if (i == 0) {
				s_trans[i].delay_usecs = NEURONSPI_EDGE_DELAY;
			} else if (i == 1) {
				s_trans[i].tx_buf = send_buf;
				s_trans[i].rx_buf = recv_buf;
				if (send_header) {
					s_trans[i].delay_usecs = delay;
					s_trans[i].len = NEURONSPI_FIRST_MESSAGE_LENGTH;
				} else {
					// If len is more than NEURONSPI_MAX_TX * i, then chunk len is NEURONSPI_MAX_TX, otherwise it's the remainder
					s_trans[i].len = (len - (NEURONSPI_MAX_TX * i)) > 0 ? NEURONSPI_MAX_TX : len;
				}
			} else if (i == trans_count - 1) {
				if (send_header) {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].len = ((NEURONSPI_MAX_TX * (i - 1)) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 2)));
				} else {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].len = ((NEURONSPI_MAX_TX * i) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 1)));
				}
				s_trans[i].delay_usecs = NEURONSPI_LAST_TRANSFER_DELAY;
				// If len is more than NEURONSPI_MAX_TX * i (+ optionally header), then chunk len is NEURONSPI_MAX_TX (+ optionally header),
				// otherwise it's the remainder
			} else {
				if (send_header) {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_FIRST_MESSAGE_LENGTH + (NEURONSPI_MAX_TX * (i - 2))]);
					s_trans[i].len = ((NEURONSPI_MAX_TX * (i - 1)) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 2)));
				} else {
					s_trans[i].tx_buf = &(send_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].rx_buf = &(recv_buf[NEURONSPI_MAX_TX * (i - 1)]);
					s_trans[i].len = ((NEURONSPI_MAX_TX * i) - len) < 0 ? NEURONSPI_MAX_TX : (len - (NEURONSPI_MAX_TX * (i - 1)));
				}
				// If len is more than NEURONSPI_MAX_TX * i (+ optionally header), then chunk len is NEURONSPI_MAX_TX (+ optionally header),
				// otherwise it's the remainder
			}
			spi_message_add_tail(&(s_trans[i]), s_msg);
		}
		spi_sync(spi_dev, s_msg);
		for (i = 0; i < trans_count; i++) {
			spi_transfer_del(&(s_trans[i]));
		}
	    kfree(s_trans);
	    kfree(s_msg);
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI Master Read - %d:\n\t%100ph\n\t%100ph\n\t%100ph\n\t%100ph\n", len,recv_buf, &recv_buf[64],
				&recv_buf[128], &recv_buf[192]);
#endif
	}
    if (d_data == NULL || (d_data != NULL && !d_data->reserved_device)) {
		recv_crc1 = neuronspi_spi_crc(recv_buf, 4, 0);
		memcpy(&packet_crc, &recv_buf[4], 2);
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI CRC1: %x\t COMPUTED CRC1:%x\n", packet_crc, recv_crc1);
#endif
		if (recv_crc1 == packet_crc) {
		// Signal the UART to issue character reads
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI CRC1 Correct");
#endif
			if (d_data && recv_buf[0] == 0x41) {
				d_data->uart_buf[0] = recv_buf[3];
#if NEURONSPI_DETAILED_DEBUG > 0
				printk(KERN_INFO "NEURONSPI: Reading UART data for device %d\n", d_data->neuron_index);
#endif
				for (i = 0; i < d_data->uart_data->p_count; i++) {
					if (d_data->uart_data->p[i].dev_index == d_data->neuron_index) {
						neuronspi_uart_handle_rx(&d_data->uart_data->p[i], 1, 1);
					}
				}
				if (!(d_data->uart_read) && (d_data->uart_count)) {
					d_data->uart_read = recv_buf[2];
					for (i = 0; i < d_data->uart_data->p_count; i++) {
#if NEURONSPI_DETAILED_DEBUG > 0
					printk(KERN_INFO "NEURONSPI: UART Buffer:%d, UART Local Port Count:%d, UART Global Port Count:%d\n", d_data->uart_read,
							d_data->uart_count,  d_data->uart_data->p_count);
#endif
						if (d_data->uart_data->p[i].dev_index == d_data->neuron_index && !d_data->reserved_device) {
							kthread_queue_work(&d_data->uart_data->kworker, &d_data->uart_data->p[i].rx_work);
						}
					}
				}
			}
		}
#if NEURONSPI_DETAILED_DEBUG > 0
		else {
			printk(KERN_INFO "NEURONSPI: SPI CRC1 Not Correct");
		}
#endif
		recv_crc2 = neuronspi_spi_crc(&recv_buf[6], len - 8, recv_crc1);
		memcpy(&packet_crc, &recv_buf[len - 2], 2);
#if NEURONSPI_DETAILED_DEBUG > 1
		printk(KERN_INFO "NEURONSPI: SPI CRC2: %x\t COMPUTED CRC2:%x\n", packet_crc, recv_crc2);
#endif
		if (recv_crc2 != packet_crc) {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_INFO "NEURONSPI: SPI CRC2 Not Correct");
#endif
			recv_buf[0] = 0;
			ret_code = 1;
		}
    }
    mutex_unlock(&neuronspi_master_mutex);
    return ret_code;
}



irqreturn_t neuronspi_spi_irq(s32 irq, void *dev_id)
{
	s32 i;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	struct neuronspi_uart_data *u_data;
	spi = (struct spi_device *)dev_id;
	d_data = spi_get_drvdata(spi);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI IRQ\n");
#endif
	if (d_data->uart_count) {
		u_data = d_data->uart_data;
		for (i = 0; i < u_data->p_count; i++) {
			if (u_data->p[i].dev_index == d_data->neuron_index) {
				kthread_queue_work(&u_data->kworker, &u_data->p[i].irq_work);
			}

		}
	}
	return IRQ_HANDLED;
}

void neuronspi_spi_led_set_brightness(struct spi_device* spi_dev, enum led_brightness brightness, int id)
{
	u8 *message_buf;
	u8 *recv_buf;
	u16 crc1;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI LED Set, Dev-CS:%d, led id:%d\n", spi_dev->chip_select, id);
#endif
	message_buf = kzalloc(NEURONSPI_SPI_LED_SET_MESSAGE_LEN, GFP_ATOMIC);
	recv_buf = kzalloc(NEURONSPI_SPI_LED_SET_MESSAGE_LEN, GFP_ATOMIC);
	memcpy(message_buf, NEURONSPI_SPI_LED_SET_MESSAGE, NEURONSPI_SPI_LED_SET_MESSAGE_LEN);
	if (d_data->features != NULL) {
		message_buf[2] = d_data->features->di_count + d_data->features->do_count + d_data->features->ro_count + id;
	} else {
		message_buf[2] += id;
	}
	if (brightness > 0) {
		message_buf[1] = 0x01;
	} else {
		message_buf[1] = 0x00;
	}
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_LED_SET_MESSAGE_LEN, frequency, 25, 0, 0);
	}
	kfree(message_buf);
	kfree(recv_buf);
}

int neuronspi_spi_gpio_di_get(struct spi_device* spi_dev, u32 id)
{
	u8 *recv_buf;
	bool ret = 0;
	u32 offset = id / 16;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	recv_buf = kzalloc(4, GFP_ATOMIC);
	regmap_read(d_data->reg_map, d_data->regstart_table->di_val_reg + offset, (void*)recv_buf);
	if (*recv_buf & (0x1 << offset)) {
		ret = 1;
	}
	kfree(recv_buf);
	return ret;
}

int neuronspi_spi_gpio_do_set(struct spi_device* spi_dev, u32 id, int value)
{
	u32 current_value = 0;
	bool ret = 0;
	u32 offset = id / 16;
	u16 off_val = value << (id % 16);
	u16 mask = ~(1 << (id % 16));
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	regmap_read(d_data->reg_map, d_data->regstart_table->do_val_reg + offset, &current_value);
	current_value&= mask;
	current_value|= off_val;
	regmap_write(d_data->reg_map, d_data->regstart_table->do_val_reg + offset, current_value);
	return ret;
}

int neuronspi_spi_gpio_ro_set(struct spi_device* spi_dev, u32 id, int value)
{
	u32 current_value = 0;
	bool ret = 0;
	u32 offset = id / 16;
	u16 off_val = value << (id % 16);
	u16 mask = ~(1 << (id % 16));
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	regmap_read(d_data->reg_map, d_data->regstart_table->ro_val_reg + offset, &current_value);
	current_value&= mask;
	current_value|= off_val;
	regmap_write(d_data->reg_map, d_data->regstart_table->ro_val_reg + offset, current_value);
	return ret;
}



s32 neuronspi_spi_probe(struct spi_device *spi)
{
	const struct neuronspi_devtype *devtype;
	struct neuronspi_driver_data *n_spi;
	s32 ret, i, no_irq = 0;
	u8 uart_count = 0;
	unsigned long flags;
	n_spi = kzalloc(sizeof *n_spi, GFP_ATOMIC);
	spin_lock_irqsave(neuronspi_probe_spinlock, flags);
	neuronspi_probe_count++;
	spin_unlock_irqrestore(neuronspi_probe_spinlock, flags);
	if (!n_spi)
		return -ENOMEM;
	printk(KERN_INFO "NEURONSPI: Neuronspi Probe Started\n");
	if (n_spi == NULL || spi == NULL) {
		return -8;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Chip Max Hz-%d\n",spi->master->max_speed_hz);
#endif
	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	spi->mode		= spi->mode ? spi->mode : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? spi->max_speed_hz : 12000000;
	ret = spi_setup(spi);
	n_spi->neuron_index = spi->chip_select - 1;
	n_spi->reserved_device = 0;

	if (neuron_plc_dev == NULL) {
		neuron_plc_dev = platform_device_alloc("unipi_plc", -1);
		neuron_plc_dev->dev.groups = neuron_plc_attr_groups;
		platform_device_add(neuron_plc_dev);
	}

	if (ret)
		return ret;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Chip Max Hz-%d %d\n", spi->master->max_speed_hz, spi->max_speed_hz);
#endif
	if (spi->dev.of_node) {
		const struct of_device_id *of_id =
			of_match_device(neuronspi_id_match, &spi->dev);
		if (!of_id) {
			printk(KERN_DEBUG "NEURONSPI: Probe %s does not match a device!\n", *(&spi->dev.of_node->full_name));
			return -ENODEV;
		}
		of_property_read_u32_array(spi->dev.of_node, "neuron-board-index", &(n_spi->neuron_index), 1);
		of_property_read_u32_array(spi->dev.of_node, "neuron-probe-always-succeeds", &(n_spi->probe_always_succeeds), 1);
		devtype = (struct neuronspi_devtype *)of_id->data;
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "DEVICE TREE NODE FOUND %d\n", n_spi->neuron_index);
#endif
	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);
		devtype = (struct neuronspi_devtype *)id_entry->driver_data;
	}


	kthread_init_worker(&n_spi->primary_worker);

	n_spi->primary_worker_task = kthread_run(kthread_worker_fn, &n_spi->primary_worker, "neuronspi");
	if (IS_ERR(n_spi->primary_worker_task )) {
		ret = PTR_ERR(n_spi->primary_worker_task);
		return ret;
	}
	sched_setscheduler(n_spi->primary_worker_task, SCHED_FIFO, &neuronspi_sched_param);


	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	n_spi->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	n_spi->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_ATOMIC);
	n_spi->first_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_ATOMIC);	// allocate space for initial probe
	n_spi->second_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_ATOMIC); // allocate space for uart probe
	n_spi->lower_board_id = 0xFF;
	n_spi->upper_board_id = 0xFF;
	n_spi->combination_id = 0xFF;
	n_spi->spi_driver = &neuronspi_spi_driver;

	memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1, 0);

	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point
	i = 0;
	do {
		memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
		memset(n_spi->first_probe_reply, 0, NEURONSPI_PROBE_MESSAGE_LEN);
		neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1, 0);
		i++;
	} while (n_spi->first_probe_reply[0] == 0x41 && i < 1000);	// UART messages should be ignored

	if (n_spi->first_probe_reply[0] != 0) { 	// CRC error sets the first byte to 0
		uart_count = n_spi->first_probe_reply[14] & 0x0f;
		for (i = 0; i < NEURONSPI_BOARDTABLE_LEN; i++) {
			if (n_spi->first_probe_reply[19] == NEURONSPI_BOARDTABLE[i].lower_board_id) {
				if (n_spi->combination_id == 0xFF && NEURONSPI_BOARDTABLE[i].upper_board_id == 0) {
					n_spi->combination_id = NEURONSPI_BOARDTABLE[i].index;
				}
				if (n_spi->lower_board_id == 0xFF) {
					n_spi->lower_board_id = n_spi->first_probe_reply[17];
				}
				if (n_spi->first_probe_reply[17] == NEURONSPI_BOARDTABLE[i].index) {
					n_spi->combination_id = n_spi->first_probe_reply[17];
					n_spi->upper_board_id = NEURONSPI_BOARDTABLE[i].upper_board_id;
				}
			}
		}
	} else if (!n_spi->probe_always_succeeds) {
		ret = -ENODEV;
		kfree(n_spi);
		printk(KERN_INFO "NEURONSPI: Probe did not detect a valid Neuron device on CS %d\n", spi->chip_select);
		return ret;
	}

	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		n_spi->features = kzalloc(sizeof(struct neuronspi_board_features), GFP_ATOMIC);
		n_spi->features = &(NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->features);
	} else {
		n_spi->features = NULL;
	}

	n_spi->ideal_frequency = NEURONSPI_COMMON_FREQ;
	for (i = 0; i < NEURONSPI_SLOWER_MODELS_LEN; i++) {
		if (NEURONSPI_SLOWER_MODELS[i] == (n_spi->first_probe_reply[19] << 8 | n_spi->first_probe_reply[18])) {
			n_spi->slower_model = 1;
			n_spi->ideal_frequency = NEURONSPI_SLOWER_FREQ;
		}
	}
	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		printk(KERN_INFO "NEURONSPI: Probe detected Neuron Board %s (L:%x U:%x C:%x) Index: %d Fw: v%d.%d on CS %d, \
Uart count: %d - reg1000: %x, reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				n_spi->lower_board_id, n_spi->upper_board_id, n_spi->combination_id,  n_spi->neuron_index,
				n_spi->first_probe_reply[11],  n_spi->first_probe_reply[10], spi->chip_select, uart_count,
				n_spi->first_probe_reply[11] << 8 | n_spi->first_probe_reply[10],
				n_spi->first_probe_reply[13] << 8 | n_spi->first_probe_reply[12], n_spi->first_probe_reply[15] << 8 | n_spi->first_probe_reply[14],
				n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16], n_spi->first_probe_reply[19] << 8 | n_spi->first_probe_reply[18]);
	} else if (n_spi->lower_board_id != 0xFF) {
		printk(KERN_INFO "NEURONSPI: Probe detected Neuron Board L:%x C:??? Index: %d Fw: v%d.%d on CS %d, Uart count: %d - reg1000: %x, \
reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",
				n_spi->lower_board_id, n_spi->neuron_index, n_spi->first_probe_reply[11],  n_spi->first_probe_reply[10],
				spi->chip_select, uart_count, n_spi->first_probe_reply[11] << 8 | n_spi->first_probe_reply[10],
				n_spi->first_probe_reply[13] << 8 | n_spi->first_probe_reply[12], n_spi->first_probe_reply[15] << 8 | n_spi->first_probe_reply[14],
				n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16], n_spi->first_probe_reply[19] << 8 | n_spi->first_probe_reply[18]);
	} else {
		printk(KERN_INFO "NEURONSPI: Probe detected Neuron Board L:??? C:??? Index: %d Fw: v%d.%d on CS %d, Uart count: %d - reg1000: %x, \
reg1001: %x, reg1002: %x, reg1003: %x, reg1004: %x\n",
				n_spi->neuron_index, n_spi->first_probe_reply[11],  n_spi->first_probe_reply[10], spi->chip_select, uart_count,
				n_spi->first_probe_reply[11] << 8 | n_spi->first_probe_reply[10], n_spi->first_probe_reply[13] << 8 | n_spi->first_probe_reply[12],
				n_spi->first_probe_reply[15] << 8 | n_spi->first_probe_reply[14], n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16],
				n_spi->first_probe_reply[19] << 8 | n_spi->first_probe_reply[18]);
	}
	if (n_spi->combination_id != 0xFF) {
		printk(KERN_INFO "NEURONSPI: Neuron device %s on CS %d uses SPI communication freq. %d Hz\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				spi->chip_select, n_spi->ideal_frequency);
	}
	n_spi->reg_map = regmap_init(&(spi->dev), &neuronspi_regmap_bus, spi, &neuronspi_regmap_config_default);
	spin_lock_init(&n_spi->sysfs_regmap_lock);
	if (n_spi->features) {
		n_spi->regstart_table = kzalloc(sizeof(struct neuronspi_board_regstart_table), 1);
		neuronspi_create_reg_starts(n_spi->regstart_table, NEURONSPI_BOARDTABLE[n_spi->combination_id].definition);
	} else {
		n_spi->regstart_table = NULL;
	}


	// Check for user-configurable LED devices
	if (n_spi->features && n_spi->features->led_count > 0) {
		printk(KERN_INFO "NEURONSPI: LED model %s with %d LEDs detected at CS: %d\n",
				NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
				n_spi->features->led_count, spi->chip_select);
		n_spi->led_driver = kzalloc(sizeof(struct neuronspi_led_driver) * n_spi->features->led_count, GFP_ATOMIC);
		for (i = 0; i < n_spi->features->led_count; i++) {
			kthread_init_work(&(n_spi->led_driver[i].led_work), neuronspi_led_proc);
		}
	}


	if (uart_count && neuronspi_uart == NULL) {	// Register UART if not registered
		neuronspi_uart = kzalloc(sizeof(struct uart_driver), GFP_ATOMIC);
		neuronspi_uart->owner		= THIS_MODULE;
		neuronspi_uart->dev_name	= "ttyNS";
		neuronspi_uart->driver_name = "ttyNS";
		neuronspi_uart->nr	= NEURONSPI_MAX_UART;
		ret = uart_register_driver(neuronspi_uart);
		if (ret) {
			printk(KERN_ERR "NEURONSPI:Failed to register the neuronspi uart driver, ERR:%d\n", ret);
		} else {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_DEBUG "NEURONSPI: UART driver registered successfully!\n");
#endif
		}
		if (neuronspi_uart_glob_data != NULL) {
			printk(KERN_ERR "NEURONSPI:Uart data already allocated!\n");
		} else {
			neuronspi_uart_glob_data = kzalloc(sizeof(struct neuronspi_uart_data), GFP_ATOMIC);
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_DEBUG "NEURONSPI: UART driver data allocated successfully!\n");
#endif
		}

	}
	if (neuronspi_uart_glob_data != NULL) {
		n_spi->uart_data = neuronspi_uart_glob_data;
	}
	n_spi->char_driver = &neuronspi_cdrv;
	if (uart_count) {
		n_spi->serial_driver = neuronspi_uart;
	} else {
		n_spi->serial_driver = NULL;
	}
	n_spi->uart_count = uart_count;

#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: CHIP SELECT %d\n", spi->chip_select);
#endif
	spin_lock_irqsave(neuronspi_probe_spinlock, flags);
	neuronspi_s_dev[n_spi->neuron_index] = spi;
	spi_set_drvdata(neuronspi_s_dev[n_spi->neuron_index], n_spi);
	if (neuronspi_probe_count == NEURONSPI_MAX_DEVS) {
		neuronspi_model_id = neuronspi_find_model_id(neuronspi_probe_count);
	}
	spin_unlock_irqrestore(neuronspi_probe_spinlock, flags);
	if (neuronspi_model_id != -1) {
		printk(KERN_INFO "NEURONSPI: Detected Neuron board combination corresponding to %s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].model_name);
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: SPI IRQ: %d", spi->irq);
#endif
	strcpy(n_spi->platform_name, "io_group0");
	n_spi->platform_name[8] = n_spi->neuron_index + '1';
	n_spi->board_device = platform_device_alloc(n_spi->platform_name, -1);
	n_spi->board_device->dev.parent = &(neuron_plc_dev->dev);
	if (n_spi->combination_id != 0xFF) {
		n_spi->board_device->dev.groups = neuron_board_attr_groups;
	}
	n_spi->board_device->dev.driver = &neuronspi_spi_driver.driver;
	platform_device_add(n_spi->board_device);
	platform_set_drvdata(n_spi->board_device, n_spi);

	if (!(neuronspi_cdrv.major_number)) { // Register character device if it doesn't exist
		ret = char_register_driver();
		if (ret) {
			printk(KERN_ERR "NEURONSPI: Failed to register the neuronspi character driver, ERR:%d\n", ret);
		}
	}

	if (n_spi->features) {
		if (n_spi->features->led_count) {
			for (i = 0; i < n_spi->features->led_count; i++) {
				strcpy(n_spi->led_driver[i].name, "neuron:green:uled-x1");
				if (i < 9) {
					n_spi->led_driver[i].name[19] = i + '1';
				} else {
					n_spi->led_driver[i].name[19] = i - 9 + 'a';
				}
				// Initialise the rest of the structure
				n_spi->led_driver[i].id = i;
				n_spi->led_driver[i].brightness = LED_OFF;
				n_spi->led_driver[i].spi = spi;
				spin_lock_init(&n_spi->led_driver[i].lock);
				n_spi->led_driver[i].ldev.name = n_spi->led_driver[i].name;
				n_spi->led_driver[i].ldev.brightness = n_spi->led_driver[i].brightness;
				n_spi->led_driver[i].ldev.brightness_set = neuronspi_led_set_brightness;
				led_classdev_register(&(n_spi->board_device->dev), &(n_spi->led_driver[i].ldev));
			}
		}
#ifdef CONFIG_GPIOLIB
		if (n_spi->features->di_count) {
			n_spi->di_driver = kzalloc(sizeof(struct neuronspi_di_driver*) * n_spi->features->di_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->di_count; i++) {
				n_spi->di_driver[i] = kzalloc(sizeof(struct neuronspi_di_driver), GFP_ATOMIC);
				strcpy(n_spi->di_driver[i]->name, "di_0_00");
				n_spi->di_driver[i]->name[3] = n_spi->neuron_index + '1';
				n_spi->di_driver[i]->name[5] = ((i + 1) / 10) + '0';
				n_spi->di_driver[i]->name[6] = ((i + 1) % 10) + '0';
				n_spi->di_driver[i]->di_index = i;
				n_spi->di_driver[i]->spi = spi;
				n_spi->di_driver[i]->plat_dev = platform_device_alloc(n_spi->di_driver[i]->name, -1);
				n_spi->di_driver[i]->plat_dev->dev.parent = &(n_spi->board_device->dev);
				n_spi->di_driver[i]->plat_dev->dev.groups = neuron_gpio_di_attr_groups;
				n_spi->di_driver[i]->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
				platform_device_add(n_spi->di_driver[i]->plat_dev);
				platform_set_drvdata(n_spi->di_driver[i]->plat_dev, n_spi->di_driver[i]);
				n_spi->di_driver[i]->gpio_c.owner = THIS_MODULE;
				n_spi->di_driver[i]->gpio_c.parent = &(n_spi->di_driver[i]->plat_dev->dev);
				n_spi->di_driver[i]->gpio_c.label = "neuron_di";
				n_spi->di_driver[i]->gpio_c.can_sleep = 1;
				n_spi->di_driver[i]->gpio_c.ngpio = 1;
				n_spi->di_driver[i]->gpio_c.base = -1;
				n_spi->di_driver[i]->gpio_c.direction_input = neuronspi_gpio_di_direction_input;
				n_spi->di_driver[i]->gpio_c.get = neuronspi_gpio_di_get;
				gpiochip_add_data(&n_spi->di_driver[i]->gpio_c, n_spi->di_driver[i]);
			}
		}

		if (n_spi->features->do_count) {
			n_spi->do_driver = kzalloc(sizeof(struct neuronspi_do_driver*) * n_spi->features->do_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->do_count; i++) {
				n_spi->do_driver[i] = kzalloc(sizeof(struct neuronspi_do_driver), GFP_ATOMIC);
				strcpy(n_spi->do_driver[i]->name, "do_0_00");
				n_spi->do_driver[i]->name[3] = n_spi->neuron_index + '1';
				n_spi->do_driver[i]->name[5] = ((i + 1) / 10) + '0';
				n_spi->do_driver[i]->name[6] = ((i + 1) % 10) + '0';
				n_spi->do_driver[i]->do_index = i;
				n_spi->do_driver[i]->spi = spi;
				n_spi->do_driver[i]->plat_dev = platform_device_alloc(n_spi->do_driver[i]->name, -1);
				n_spi->do_driver[i]->plat_dev->dev.parent = &(n_spi->board_device->dev);
				n_spi->do_driver[i]->plat_dev->dev.groups = neuron_gpio_do_attr_groups;
				n_spi->do_driver[i]->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
				platform_device_add(n_spi->do_driver[i]->plat_dev);
				platform_set_drvdata(n_spi->do_driver[i]->plat_dev, n_spi->do_driver[i]);
				n_spi->do_driver[i]->gpio_c.owner = THIS_MODULE;
				n_spi->do_driver[i]->gpio_c.parent = &(n_spi->do_driver[i]->plat_dev->dev);
				n_spi->do_driver[i]->gpio_c.label = "neuron_do";
				n_spi->do_driver[i]->gpio_c.can_sleep = 1;
				n_spi->do_driver[i]->gpio_c.ngpio = 1;
				n_spi->do_driver[i]->gpio_c.base = -1;
				n_spi->do_driver[i]->gpio_c.direction_output = neuronspi_gpio_do_direction_output;
				n_spi->do_driver[i]->gpio_c.set = neuronspi_gpio_do_set;
				gpiochip_add_data(&n_spi->do_driver[i]->gpio_c, n_spi->do_driver[i]);
			}
		}

		if (n_spi->features->ro_count) {
			n_spi->ro_driver = kzalloc(sizeof(struct neuronspi_ro_driver*) * n_spi->features->ro_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->ro_count; i++) {
				n_spi->ro_driver[i] = kzalloc(sizeof(struct neuronspi_ro_driver), GFP_ATOMIC);
				strcpy(n_spi->ro_driver[i]->name, "ro_0_00");
				n_spi->ro_driver[i]->name[3] = n_spi->neuron_index + '1';
				n_spi->ro_driver[i]->name[5] = ((i + 1) / 10) + '0';
				n_spi->ro_driver[i]->name[6] = ((i + 1) % 10) + '0';
				n_spi->ro_driver[i]->ro_index = i;
				n_spi->ro_driver[i]->spi = spi;
				n_spi->ro_driver[i]->plat_dev = platform_device_alloc(n_spi->ro_driver[i]->name, -1);
				n_spi->ro_driver[i]->plat_dev->dev.parent = &(n_spi->board_device->dev);
				n_spi->ro_driver[i]->plat_dev->dev.groups = neuron_gpio_ro_attr_groups;
				n_spi->ro_driver[i]->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
				platform_device_add(n_spi->ro_driver[i]->plat_dev);
				platform_set_drvdata(n_spi->ro_driver[i]->plat_dev, n_spi->ro_driver[i]);
				n_spi->ro_driver[i]->gpio_c.owner = THIS_MODULE;
				n_spi->ro_driver[i]->gpio_c.parent = &(n_spi->ro_driver[i]->plat_dev->dev);
				n_spi->ro_driver[i]->gpio_c.label = "neuron_ro";
				n_spi->ro_driver[i]->gpio_c.can_sleep = 1;
				n_spi->ro_driver[i]->gpio_c.ngpio = 1;
				n_spi->ro_driver[i]->gpio_c.base = -1;
				n_spi->ro_driver[i]->gpio_c.direction_output = neuronspi_gpio_ro_direction_output;
				n_spi->ro_driver[i]->gpio_c.set = neuronspi_gpio_ro_set;
				gpiochip_add_data(&n_spi->ro_driver[i]->gpio_c, n_spi->ro_driver[i]);
			}
		}
#endif
		if (n_spi->features->stm_ai_count) {
			n_spi->stm_ai_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ai_driver))->parent = spi;
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ai_driver))->index = 0;
			n_spi->stm_ai_driver->modes = INDIO_DIRECT_MODE;
			n_spi->stm_ai_driver->currentmode = INDIO_DIRECT_MODE;
			n_spi->stm_ai_driver->name = "ai_type_a";
			n_spi->stm_ai_driver->dev.parent = &(n_spi->board_device->dev);
			dev_set_name(&n_spi->stm_ai_driver->dev, "ai_%d_1",	(int)n_spi->neuron_index + 1);
			n_spi->stm_ai_driver->num_channels = 2;
			n_spi->stm_ai_driver->channels = neuronspi_stm_ai_chan_spec;
			n_spi->stm_ai_driver->info = &neuronspi_stm_ai_info;
			iio_device_register(n_spi->stm_ai_driver);
		}
		if (n_spi->features->stm_ao_count) {
			n_spi->stm_ao_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ao_driver))->parent = spi;
			((struct neuronspi_analog_data*)iio_priv(n_spi->stm_ao_driver))->index = 0;
			n_spi->stm_ao_driver->modes = INDIO_DIRECT_MODE;
			n_spi->stm_ao_driver->currentmode = INDIO_DIRECT_MODE;
			n_spi->stm_ao_driver->name = "ao_type_a";
			n_spi->stm_ao_driver->dev.parent = &(n_spi->board_device->dev);
			dev_set_name(&n_spi->stm_ao_driver->dev, "ao_%d_1",	(int)n_spi->neuron_index + 1);
			n_spi->stm_ao_driver->num_channels = 3;
			n_spi->stm_ao_driver->channels = neuronspi_stm_ao_chan_spec;
			n_spi->stm_ao_driver->info = &neuronspi_stm_ao_info;
			iio_device_register(n_spi->stm_ao_driver);
		}
		if (n_spi->features->sec_ai_count) {
			n_spi->sec_ai_driver = kzalloc(sizeof(struct neuronspi_analog_data*) * n_spi->features->sec_ai_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->sec_ai_count; i++) {
				n_spi->sec_ai_driver[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ai_driver[i]))->parent = spi;
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ai_driver[i]))->index = i;
				n_spi->sec_ai_driver[i]->modes = INDIO_DIRECT_MODE;
				n_spi->sec_ai_driver[i]->currentmode = INDIO_DIRECT_MODE;
				n_spi->sec_ai_driver[i]->name = "ai_type_b";
				n_spi->sec_ai_driver[i]->dev.parent = &(n_spi->board_device->dev);
				dev_set_name(&n_spi->sec_ai_driver[i]->dev, "ai_%d_%d",	(int)n_spi->neuron_index + 1, (int)i + 1);
				n_spi->sec_ai_driver[i]->num_channels = 3;
				n_spi->sec_ai_driver[i]->channels = neuronspi_sec_ai_chan_spec;
				n_spi->sec_ai_driver[i]->info = &neuronspi_sec_ai_info;
				iio_device_register(n_spi->sec_ai_driver[i]);
			}
		}
		if (n_spi->features->sec_ao_count) {
			n_spi->sec_ao_driver = kzalloc(sizeof(struct neuronspi_analog_data*) * n_spi->features->sec_ao_count, GFP_ATOMIC);
			for (i = 0; i < n_spi->features->sec_ao_count; i++) {
				n_spi->sec_ao_driver[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ao_driver[i]))->parent = spi;
				((struct neuronspi_analog_data*)iio_priv(n_spi->sec_ao_driver[i]))->index = i;
				n_spi->sec_ao_driver[i]->modes = INDIO_DIRECT_MODE;
				n_spi->sec_ao_driver[i]->currentmode = INDIO_DIRECT_MODE;
				n_spi->sec_ao_driver[i]->name = "ao_type_b";
				n_spi->sec_ao_driver[i]->dev.parent = &(n_spi->board_device->dev);
				dev_set_name(&n_spi->sec_ao_driver[i]->dev, "ao_%d_%d",	(int)n_spi->neuron_index + 1, (int)i + 1);
				n_spi->sec_ao_driver[i]->num_channels = 1;
				n_spi->sec_ao_driver[i]->channels = neuronspi_sec_ao_chan_spec;
				n_spi->sec_ao_driver[i]->info = &neuronspi_sec_ao_info;
				iio_device_register(n_spi->sec_ao_driver[i]);
			}
		}
	}

	if (uart_count) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: UART registration 1\n");
#endif
		n_spi->uart_buf = kzalloc(NEURONSPI_FIFO_SIZE, GFP_ATOMIC);
		neuronspi_uart_probe(spi, n_spi->neuron_index);
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: UART PROBE MCTRL:%d\n", neuronspi_spi_uart_get_cflag(spi, 0));
#endif
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART registration\n");
#endif
	neuronspi_spi_set_irqs(spi, 0x5);
	for (i = 0; i < NEURONSPI_NO_INTERRUPT_MODELS_LEN; i++) {
		if (NEURONSPI_NO_INTERRUPT_MODELS[i] == (n_spi->first_probe_reply[17] << 8 | n_spi->first_probe_reply[16])) {
			no_irq = 1;
		}
	}

	n_spi->poll_thread = NULL;
	if (!no_irq) {
		n_spi->no_irq = 0;
		ret = devm_request_irq(&(spi->dev), spi->irq, neuronspi_spi_irq, 0x81, dev_name(&(spi->dev)), spi);
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: IRQ registration, ret:%d\n", ret);
#endif
	} else {
		n_spi->no_irq = 1;
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: NO IRQ ON THIS MODEL !!\n");
#endif
	}

	return ret;
}

u32 neuronspi_spi_uart_get_cflag(struct spi_device* spi_dev, u8 port)
{
	u8 *message_buf;
	u8 *recv_buf;
	u16 crc1, crc2, ret;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	s32 frequency = NEURONSPI_COMMON_FREQ;
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
	message_buf = kzalloc(NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, GFP_ATOMIC);
	recv_buf = kzalloc(NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, GFP_ATOMIC);
	memcpy(message_buf, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, frequency, 65, 1, 0);
	}
	ret = ((u32*)recv_buf)[5];
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS Get, Dev-CS:%d, val:%x\n", spi_dev->chip_select, ret);
#endif
	kfree(message_buf);
	kfree(recv_buf);
	return ret;
}

s32 neuronspi_spi_remove(struct spi_device *spi)
{
	int i;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	if (n_spi) {
		if (n_spi->led_driver) {
			for (i = 0; i < n_spi->features->led_count; i++) {
				led_classdev_unregister(&(n_spi->led_driver[i].ldev));
			}
			kthread_flush_worker(&n_spi->primary_worker);
			kfree(n_spi->led_driver);
			n_spi->led_driver = NULL;
		}
		printk(KERN_INFO "NEURONSPI: LED DRIVER UNREGISTERED\n");
		if (n_spi->di_driver) {
			for (i = 0; i < n_spi->features->di_count; i++) {
				gpiochip_remove(&n_spi->di_driver[i]->gpio_c);
				platform_set_drvdata(n_spi->di_driver[i]->plat_dev, 0);
				platform_device_unregister(n_spi->di_driver[i]->plat_dev);
				kfree(n_spi->di_driver[i]);
			}
			kfree(n_spi->di_driver);
		}
		if (n_spi->do_driver) {
			for (i = 0; i < n_spi->features->do_count; i++) {
				gpiochip_remove(&n_spi->do_driver[i]->gpio_c);
				platform_set_drvdata(n_spi->do_driver[i]->plat_dev, 0);
				platform_device_unregister(n_spi->do_driver[i]->plat_dev);
				kfree(n_spi->do_driver[i]);
			}
			kfree(n_spi->do_driver);
		}
		if (n_spi->ro_driver) {
			for (i = 0; i < n_spi->features->ro_count; i++) {
				gpiochip_remove(&n_spi->ro_driver[i]->gpio_c);
				platform_set_drvdata(n_spi->ro_driver[i]->plat_dev, 0);
				platform_device_unregister(n_spi->ro_driver[i]->plat_dev);
				kfree(n_spi->ro_driver[i]);
			}
			kfree(n_spi->ro_driver);
		}
		printk(KERN_INFO "NEURONSPI: GPIO DRIVER UNREGISTERED\n");
		if (n_spi->stm_ai_driver) {
			iio_device_unregister(n_spi->stm_ai_driver);
		}
		if (n_spi->stm_ao_driver) {
			iio_device_unregister(n_spi->stm_ao_driver);
		}
		if (n_spi->sec_ai_driver) {
			for (i = 0; i < n_spi->features->sec_ai_count; i++) {
				iio_device_unregister(n_spi->sec_ai_driver[i]);
			}
			kfree(n_spi->sec_ai_driver);
			n_spi->sec_ai_driver = NULL;
		}
		if (n_spi->sec_ao_driver) {
			for (i = 0; i < n_spi->features->sec_ao_count; i++) {
				iio_device_unregister(n_spi->sec_ao_driver[i]);
			}
			kfree(n_spi->sec_ao_driver);
			n_spi->sec_ao_driver = NULL;
		}
		printk(KERN_INFO "NEURONSPI: IIO DRIVER UNREGISTERED\n");
		if (n_spi->send_buf) {
			kfree(n_spi->send_buf);
			n_spi->send_buf = NULL;
		}
		if (n_spi->recv_buf) {
			kfree(n_spi->recv_buf);
			n_spi->recv_buf = NULL;
		}
		if (n_spi->uart_buf) {
			kfree(n_spi->uart_buf);
			n_spi->uart_buf = NULL;
		}
		printk(KERN_INFO "NEURONSPI: SPI/UART DRIVER UNREGISTERED\n");
		if (n_spi->board_device) {
			platform_set_drvdata(n_spi->board_device, 0);
			platform_device_unregister(n_spi->board_device);
		}
		kfree(n_spi);
	}
	return 0;
}

s32 char_register_driver(void)
{
	s32 ret = 0;

	// Character device registration
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Initialising Character Device\n");
#endif
	neuronspi_cdrv.major_number = register_chrdev(0, NEURON_DEVICE_NAME, &file_ops);
	if (neuronspi_cdrv.major_number < 0){
	   printk(KERN_ALERT "NEURONSPI: failed to register a major number\n");
	   return neuronspi_cdrv.major_number;
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: registered correctly with major number %d\n", neuronspi_cdrv.major_number);
#endif

	// Character class registration
	neuronspi_cdrv.driver_class = class_create(THIS_MODULE, NEURON_DEVICE_CLASS);
	if (IS_ERR(neuronspi_cdrv.driver_class)) {
		unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);
		printk(KERN_ALERT "NEURONSPI: Failed to register device class\n");
		return PTR_ERR(neuronspi_cdrv.driver_class);
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: device class registered correctly\n");
#endif

	// Device driver registration
	neuronspi_cdrv.dev = device_create_with_groups(neuronspi_cdrv.driver_class, &(neuron_plc_dev->dev), MKDEV(neuronspi_cdrv.major_number, 0), NULL, neuron_plc_attr_groups, NEURON_DEVICE_NAME);
	if (IS_ERR(neuronspi_cdrv.dev)) {
		class_destroy(neuronspi_cdrv.driver_class);
	    	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);
	    	printk(KERN_ALERT "NEURONSPI: Failed to create the device\n");
	    	return PTR_ERR(neuronspi_cdrv.dev);
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: device class created correctly\n");
#endif
	return ret;
}

s32 char_unregister_driver(void)
{
	device_destroy(neuronspi_cdrv.driver_class, MKDEV(neuronspi_cdrv.major_number, 0));     // Destroy the device
	class_unregister(neuronspi_cdrv.driver_class);                          				// Unregister the class
	class_destroy(neuronspi_cdrv.driver_class);                             				// Destroy the class
	unregister_chrdev(neuronspi_cdrv.major_number, NEURON_DEVICE_NAME);             			// Unregister the major number
	printk(KERN_INFO "NEURONSPI: Device unloaded successfully\n");
	return 0;
}

/*********************
 * Final definitions *
 *********************/

MODULE_ALIAS("spi:neuronspi");

static s32 __init neuronspi_init(void)
{
	s32 ret = 0;
	neuronspi_spi_w_spinlock = kzalloc(sizeof(struct spinlock), GFP_ATOMIC);
	spin_lock_init(neuronspi_spi_w_spinlock);
	neuronspi_probe_spinlock = kzalloc(sizeof(struct spinlock), GFP_ATOMIC);
	spin_lock_init(neuronspi_probe_spinlock);
	mutex_init(&neuronspi_master_mutex);
	mutex_init(&unipi_inv_speed_mutex);
	memset(&neuronspi_s_dev, 0, sizeof(neuronspi_s_dev));
	ret = spi_register_driver(&neuronspi_spi_driver);
	if (ret < 0) {
		printk(KERN_ERR "NEURONSPI: Failed to init neuronspi spi --> %d\n", ret);
		return ret;
	} else {
#ifdef NEURONSPI_MAJOR_VERSIONSTRING
		printk(KERN_INFO "NEURONSPI: SPI Driver Registered, Major Version: %s\n", NEURONSPI_MAJOR_VERSIONSTRING);
#else
		printk(KERN_INFO "NEURONSPI: SPI Driver Registered\n");
#endif
	}
	neuronspi_invalidate_thread = kthread_create(neuronspi_regmap_invalidate, NULL, "neuronspi_inv");
	if (neuronspi_invalidate_thread != NULL) {
		wake_up_process(neuronspi_invalidate_thread);
	}
	neuronspi_tty_init();
	return ret;
}

module_init(neuronspi_init);

static void __exit neuronspi_exit(void)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: Open Counter is %d\n", neuronspi_cdrv.open_counter);
#endif
	if (neuronspi_invalidate_thread) {
		kthread_stop(neuronspi_invalidate_thread);
	}
	char_unregister_driver();
	if (neuronspi_uart) {
		neuronspi_uart_remove(neuronspi_uart_glob_data);
		uart_unregister_driver(neuronspi_uart);
		kfree(neuronspi_uart);
		kfree(neuronspi_uart_glob_data);
	}
	spi_unregister_driver(&neuronspi_spi_driver);
	if (neuron_plc_dev) {
		platform_device_unregister(neuron_plc_dev);
	}
	kfree(neuronspi_spi_w_spinlock);
	printk(KERN_INFO "NEURONSPI: SPI Driver Unregistered\n");
}
module_exit(neuronspi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_DESCRIPTION("UniPi PLC Driver");
