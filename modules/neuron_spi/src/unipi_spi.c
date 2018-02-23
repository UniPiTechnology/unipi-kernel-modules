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

#include "unipi_spi.h"

/***********************
 * End of Data section *
 ***********************/

static int neuronspi_open (struct inode *inode_p, struct file *file_p)
{
	struct neuronspi_file_data *f_internal_data;
	if (neuronspi_s_dev == NULL || file_p == NULL || inode_p == NULL) {
		return -1;
	}
	neuronspi_cdrv.open_counter += 1;
	f_internal_data = kzalloc(sizeof(*f_internal_data), GFP_KERNEL);
	f_internal_data->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	f_internal_data->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	f_internal_data->spi_device = neuronspi_s_dev;
	mutex_init(&f_internal_data->lock);
	file_p->private_data = f_internal_data;
	return 0;
}

static int neuronspi_release (struct inode *inode_p, struct file *file_p)
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

static ssize_t neuronspi_read (struct file *file_p, char *buffer, size_t len, loff_t *offset)
{

	s32 result = 0;
	u8 device_index = 0;
	u32 frequency = NEURONSPI_COMMON_FREQ;
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
    if (driver_data->slower_model) {
    	frequency = NEURONSPI_SLOWER_FREQ;
    }
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



static ssize_t neuronspi_write (struct file *file_p, const char *buffer, size_t len, loff_t *w_offset)
{
	u8 device_index = 0;
	s32 result = 0;
	u32 frequency = NEURONSPI_COMMON_FREQ;
	s32 transmit_len = len - NEURONSPI_HEADER_LENGTH;
	s32 send_header = 0;
	s32 delay = 25;
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
    if (driver_data->slower_model) {
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
    spin_lock(neuronspi_spi_w_spinlock);
    neuronspi_spi_w_flag = 1;
    spin_unlock(neuronspi_spi_w_spinlock);
    neuronspi_spi_send_message(spi_driver_data, &private_data->send_buf[NEURONSPI_HEADER_LENGTH], private_data->recv_buf,
    		transmit_len, frequency, delay, send_header);
    mutex_unlock(&private_data->lock);
    return len;
}

static s32 neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, u8 length, u8 uart_index)
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
		message_buf = kzalloc(transmit_len, GFP_KERNEL);
		memcpy(message_buf, NEURONSPI_SPI_UART_SHORT_MESSAGE, NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN);
		message_buf[1] = send_buf[0];
		message_buf[3] = uart_index;
		crc1 = neuronspi_spi_crc(message_buf, 4, 0);
		memcpy(&message_buf[4], &crc1, 2);
	} else {
		transmit_len = 6 + length + 2;
		message_buf = kzalloc(transmit_len, GFP_KERNEL);
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
	recv_buf = kzalloc(transmit_len, GFP_KERNEL);
	if (d_data->slower_model) {
		frequency = NEURONSPI_SLOWER_FREQ;
	}
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi, message_buf, recv_buf, transmit_len, frequency, 65, 1);
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
		neuronspi_spi_send_message(spi, send_buf, recv_buf, transmit_len, frequency, 65, 1);
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
	message_buf = kzalloc(NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_IRQ_SET_MESSAGE, NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN, frequency, 65, 1);
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
	message_buf = kzalloc(NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	memcpy(&message_buf[10], &to, 4);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 65, 1);
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
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 35, 1);
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
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN, frequency, 35, 1);
	}
	outp = recv_buf[MODBUS_FIRST_DATA_BYTE + 1];
#if NEURONSPI_DETAILED_DEBUG > 1
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS GET, Dev-CS:%d, to:%x\n", spi_dev->chip_select, outp);
#endif
	kfree(message_buf);
	kfree(recv_buf);
	return outp;
}

int neuronspi_gpio_di_direction_input(struct gpio_chip *chip, unsigned offset) {
	return 0;
}
int neuronspi_gpio_di_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return -EINVAL;
}
int	neuronspi_gpio_di_get(struct gpio_chip *chip, unsigned offset) {
	struct neuronspi_di_driver *n_di = gpiochip_get_data(chip);
	struct spi_device *spi = n_di->spi;
	return neuronspi_spi_gpio_di_get(spi, n_di->di_index);
}

int neuronspi_gpio_do_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return 0;
}
void neuronspi_gpio_do_set(struct gpio_chip *chip, unsigned offset, int value) {
	struct neuronspi_do_driver *n_do = gpiochip_get_data(chip);
	struct spi_device *spi = n_do->spi;
	neuronspi_spi_gpio_do_set(spi, n_do->do_index, value);
}

int neuronspi_gpio_ro_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return 0;
}
void neuronspi_gpio_ro_set(struct gpio_chip *chip, unsigned offset, int value) {
	struct neuronspi_ro_driver *n_ro = gpiochip_get_data(chip);
	struct spi_device *spi = n_ro->spi;
	neuronspi_spi_gpio_ro_set(spi, n_ro->ro_index, value);
}

int neuronspi_iio_stm_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask) {
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg, &ai_data->mode);
	switch(ai_data->mode) {
	case 0: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_stm_ai_read_voltage(indio_dev, ch, val, val2, mask);
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 1: {
		if (ch->type == IIO_CURRENT) {
			neuronspi_spi_iio_stm_ai_read_current(indio_dev, ch, val, val2, mask);
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_stm_ao_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, &ao_data->mode);
	switch(ao_data->mode) {
	case 3: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_stm_ao_read_resistance(indio_dev, ch, val, val2, mask);
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_stm_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, &ao_data->mode);
	switch(ao_data->mode) {
	case 0: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_stm_ao_set_voltage(indio_dev, ch, val, val2, mask);
			return 0;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 1: {
		if (ch->type == IIO_CURRENT) {
			neuronspi_spi_iio_stm_ao_set_current(indio_dev, ch, val, val2, mask);
			return 0;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_sec_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg, &ai_data->mode);
	switch(ai_data->mode) {
	case 0: {
		return -EINVAL;
		break;
	}
	case 1: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_sec_ai_read_voltage(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 2: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_sec_ai_read_voltage(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 3: {
		if (ch->type == IIO_CURRENT) {
			neuronspi_spi_iio_sec_ai_read_current(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 4: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_sec_ai_read_resistance(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 5: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_sec_ai_read_resistance(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_sec_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	if (ch->type == IIO_VOLTAGE) {
		neuronspi_spi_iio_stm_ao_set_voltage(indio_dev, ch, val, val2, mask);
		return 0;
	} else {
		return -EINVAL;
	}
}

/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */
void neuronspi_spi_iio_sec_ai_read_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;

	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_l);
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
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;

	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_l);
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
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_l);
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
	struct neuronspi_sec_ao_data *ao_data = iio_priv(indio_dev);
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
	struct neuronspi_stm_ai_data *ai_data = iio_priv(iio_dev);
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
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
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
}

void neuronspi_spi_iio_stm_ao_read_resistance(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_aio_val = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_aio_val_reg, &stm_aio_val);
	*val = stm_aio_val;
}


void neuronspi_spi_iio_stm_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_v_err = 0;
	u32 stm_v_off = 0;

	u64 stm_true_val = val;
	u64 stm_true_ref = 0;

	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_err, &stm_v_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_off, &stm_v_off);

	stm_true_ref = ((u64)stm_v_int_ref) * (99000 + stm_v_err) * 1000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = ((stm_true_val * 10000) - stm_v_off) * 4095;
	do_div(stm_true_ref, stm_v_inp_ref);
	stm_v_inp_ref = stm_true_ref;
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 10000);

	if (stm_true_val > 4095) stm_true_val = 4095;

	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg, (unsigned int) stm_true_val);
}

void neuronspi_spi_iio_stm_ao_set_current(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_i_err = 0;
	u32 stm_i_off = 0;
	u64 stm_true_val = 0;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_curr_err, &stm_i_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_curr_off, &stm_i_off);
	stm_true_ref = ((u64)stm_v_int_ref) * (330000 + stm_i_err);
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = (stm_true_val * 10000) - stm_i_off;
	do_div(stm_true_ref, stm_v_inp_ref);
	do_div(stm_true_ref, 4095);
	stm_v_inp_ref = stm_true_ref;
	do_div(stm_true_val, stm_v_inp_ref);
	if (stm_true_val > 4095) stm_true_val = 4095;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg, (unsigned int)stm_true_val);
}

static ssize_t neuronspi_iio_show_primary_ai_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}
static ssize_t neuronspi_iio_store_primary_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg, val);
	}
err_end:
	return count;
}
static ssize_t neuronspi_iio_show_primary_ao_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}
static ssize_t neuronspi_iio_store_primary_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, val);
	}
err_end:
	return count;
}
static ssize_t neuronspi_iio_show_secondary_ai_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}
static ssize_t neuronspi_iio_store_secondary_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, val);
	}
err_end:
	return count;
}
static ssize_t neuronspi_iio_show_secondary_ao_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ao_mode_reg + ao_data->index, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}
static ssize_t neuronspi_iio_store_secondary_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ao_mode_reg + ao_data->index, val);
	}
err_end:
	return count;
}
/*
static s32 neuronspi_spi_watchdog(void *data)
{
	s32 *cycle = (s32 *) data;
	while (!kthread_should_stop()) {
		msleep(*cycle);
		spin_lock(neuronspi_spi_w_spinlock);
		if (neuronspi_spi_w_flag == 0) {
			panic_timeout = -1;
			panic("SPI Watchdog Failure\n");
		} else {
			neuronspi_spi_w_flag = 0;
		}
		spin_unlock(neuronspi_spi_w_spinlock);
	}
	return 0;
}
*/

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
	message_buf = kzalloc(NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN);
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	crc2 = neuronspi_spi_crc(&message_buf[6], NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN - 8, crc1);
	memcpy(&message_buf[NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN - 2], &crc2, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN, frequency, 65, 1);
	}
	ret = ((u32*)recv_buf)[5];
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: SPI TERMIOS Get, Dev-CS:%d, val:%x\n", spi_dev->chip_select, ret);
#endif
	kfree(message_buf);
	kfree(recv_buf);
	return ret;
}


void neuronspi_spi_send_message(struct spi_device* spi_dev, u8 *send_buf, u8 *recv_buf, s32 len, s32 freq, s32 delay, s32 send_header)
{
	s32 i = 0;
	u16 recv_crc1 = 0;
	u16 recv_crc2 = 0;
	u16 packet_crc = 0;
	s32 trans_count = (len / NEURONSPI_MAX_TX) + 3;	// number of transmissions
	struct spi_message s_msg;
	struct neuronspi_driver_data *d_data;
    struct spi_transfer* s_trans;
	mutex_lock(&neuronspi_master_mutex);
    s_trans = kzalloc(sizeof(struct spi_transfer) * trans_count, GFP_KERNEL);
#if NEURONSPI_DETAILED_DEBUG > 1
    printk(KERN_INFO "NEURONSPI: SPI Master Write, len:%d,\n %100ph\n", len, send_buf);
#endif
	if (!send_header) {
		trans_count -= 1;	// one less transmission as the header is omitted
	}
    spi_message_init(&s_msg);
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
    	spi_message_add_tail(&(s_trans[i]), &s_msg);
    }
    spi_sync(spi_dev, &s_msg);
    for (i = 0; i < trans_count; i++) {
    	spi_transfer_del(&(s_trans[i]));
    }
#if NEURONSPI_DETAILED_DEBUG > 1
    printk(KERN_INFO "NEURONSPI: SPI Master Read - %d:\n\t%100ph\n\t%100ph\n\t%100ph\n\t%100ph\n", len,recv_buf, &recv_buf[64],
    		&recv_buf[128], &recv_buf[192]);
#endif
    d_data = spi_get_drvdata(spi_dev);
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
		}
    }

    mutex_unlock(&neuronspi_master_mutex);
    kfree(s_trans);
}

static void neuronspi_uart_fifo_read(struct uart_port *port, u32 rxlen)
{
	struct neuronspi_port *s = to_neuronspi_port(port,port);
	struct neuronspi_driver_data *d_data = spi_get_drvdata(neuronspi_s_dev[s->dev_index]);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: FIFO Read len:%d\n", rxlen);
#endif
    memcpy(s->buf, d_data->uart_buf, rxlen);
}

static void neuronspi_uart_fifo_write(struct neuronspi_port *port, u8 to_send)
{
	s32 i;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: FIFO Write to_send:%d\n", to_send);
#endif
	for (i = 0; i < to_send; i++) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_INFO "NEURONSPI: UART Char Send: %x\n", port->buf[i]);
#endif
	}
    neuronspi_spi_uart_write(neuronspi_s_dev[port->dev_index], port->buf, to_send, port->dev_port);
}

static s32 neuronspi_uart_alloc_line(void)
{
	s32 i;
	BUILD_BUG_ON(NEURONSPI_MAX_DEVS > BITS_PER_LONG);

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++)
		if (!test_and_set_bit(i, &neuronspi_lines))
			break;

	return i;
}

static void neuronspi_uart_power(struct uart_port *port, s32 on)
{
    /* Do nothing */
}

static void neuronspi_uart_handle_rx(struct neuronspi_port *port, u32 rxlen, u32 iir)
{
	u32 ch, flag, bytes_read, i;
	while (rxlen) {

		neuronspi_uart_fifo_read(&port->port, rxlen);
		bytes_read = rxlen;

		port->port.icount.rx++;
		flag = TTY_NORMAL;

		for (i = 0; i < bytes_read; ++i) {
#if NEURONSPI_DETAILED_DEBUG > 0
			printk(KERN_INFO "NEURONSPI: UART Insert Char:%x\n", port->buf[i]);
#endif
			ch = port->buf[i];
			if (uart_handle_sysrq_char(port, ch))
				continue;

			uart_insert_char(&port->port, 0, 0, ch, flag);
		}
		rxlen -= bytes_read;
	}

	tty_flip_buffer_push(&port->port.state->port);
}

static void neuronspi_uart_handle_tx(struct neuronspi_port *port)
{
	u32 txlen, to_send, i;
	struct spi_device *spi;
	struct neuronspi_driver_data *d_data;
	struct circ_buf *xmit;

	spi = neuronspi_s_dev[port->dev_index];
	d_data = spi_get_drvdata(spi);
	xmit = &port->port.state->xmit;

	if (unlikely(port->port.x_char)) {
		neuronspi_spi_uart_write(spi, &port->port.x_char, 1, port->dev_port);
		port->port.icount.tx++;
		port->port.x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&port->port)) {
		return;
	}


	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
		txlen = NEURONSPI_FIFO_SIZE;
		to_send = (to_send > txlen) ? txlen : to_send;

		/* Add data to send */
		port->port.icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			port->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}

		neuronspi_uart_fifo_write(port, to_send);
	}


	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&port->port);
}

static void neuronspi_uart_handle_irq(struct neuronspi_uart_data *uart_data, s32 portno)
{
	struct neuronspi_port *n_port = &uart_data->p[portno];
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	u8 *send_buf = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_KERNEL);
	u8 *recv_buf = kzalloc(NEURONSPI_UART_PROBE_MESSAGE_LEN, GFP_KERNEL);
	memcpy(send_buf, NEURONSPI_UART_PROBE_MESSAGE, NEURONSPI_UART_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, send_buf, recv_buf, NEURONSPI_UART_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);
	kfree(send_buf);
	kfree(recv_buf);
}

static void neuronspi_uart_ist(struct kthread_work *ws)
{
	struct neuronspi_port *p = to_neuronspi_port(ws, irq_work);
	neuronspi_uart_handle_irq(p->parent, p->line);
}

static irqreturn_t neuronspi_spi_irq(s32 irq, void *dev_id)
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

static void neuronspi_uart_tx_proc(struct kthread_work *ws)
{
	struct neuronspi_port *port = to_neuronspi_port(ws, tx_work);

	if ((port->port.rs485.flags & SER_RS485_ENABLED) &&
	    (port->port.rs485.delay_rts_before_send > 0)) {
		msleep(port->port.rs485.delay_rts_before_send);
	}

	neuronspi_uart_handle_tx(port);
}


static void neuronspi_led_proc(struct kthread_work *ws)
{
	struct neuronspi_led_driver *led = to_led_driver(ws, led_work);
	printk("NEURONSPI: BRIGHT id:%d\n", led->id);
	neuronspi_spi_led_set_brightness(led->spi, led->brightness, led->id);
}

static void neuronspi_uart_rx_proc(struct kthread_work *ws)
{
	s32 end_flag = 0;
	s32 read_count = 0;
	struct neuronspi_port *n_port = to_neuronspi_port(ws, rx_work);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);

	u8 *send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	u8 *recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);

	mutex_lock(&neuronspi_master_mutex);
	read_count = d_data->uart_read;
	mutex_unlock(&neuronspi_master_mutex);

	while (!end_flag) {
		memset(recv_buf, 0, NEURONSPI_BUFFER_MAX);
		neuronspi_spi_uart_read(spi, send_buf, recv_buf, read_count, n_port->dev_port);
		if (recv_buf[6] == 0x65 && recv_buf[7] > 0) {
			mutex_lock(&neuronspi_master_mutex);
			memcpy(&d_data->uart_buf[0], &recv_buf[10], recv_buf[7]);
			neuronspi_uart_handle_rx(n_port, recv_buf[7], 1);
			read_count = recv_buf[9];
			mutex_unlock(&neuronspi_master_mutex);
		} else if (recv_buf[0] != 0x41) {
			mutex_lock(&neuronspi_master_mutex);
			d_data->uart_read = 0;
			end_flag = 1;
			mutex_unlock(&neuronspi_master_mutex);
		}
	}
	kfree(recv_buf);
	kfree(send_buf);
}


static void neuronspi_uart_stop_tx(struct uart_port *port)
{
    // ToDo : create new opcode / coil?
}

static void neuronspi_uart_stop_rx(struct uart_port *port)
{
    // ToDo : create new opcode / coil?
}

static void neuronspi_uart_start_tx(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port,port);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: Start TX\n");
#endif
	kthread_queue_work(&n_port->parent->kworker, &n_port->tx_work);
}

static s32 neuronspi_uart_poll(void *data)
{
	struct neuronspi_driver_data *d_data = (struct neuronspi_driver_data*) data;
	struct neuronspi_uart_data *u_data;
	s32 i;
	while (!kthread_should_stop()) {
		usleep_range(2000,8000);
		if (d_data->uart_count) {
			u_data = d_data->uart_data;
			for (i = 0; i < u_data->p_count; i++) {
				if (u_data->p[i].dev_index == d_data->neuron_index) {
					kthread_queue_work(&u_data->kworker, &u_data->p[i].irq_work);
				}
			}
		}
	}
	return 0;
}

static s32 neuronspi_regmap_invalidate(void *data)
{
	int i;
	int freq_cnt = 0;
	while (!kthread_should_stop()) {
		usleep_range(15000,25000);
		if (freq_cnt == 450001) freq_cnt = 0;
		for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
			if (neuronspi_s_dev[i] != NULL) {
				struct neuronspi_driver_data *d_data = spi_get_drvdata(neuronspi_s_dev[i]);
				if (d_data->combination_id == 0xFF) {
					continue;
				}
				neuronspi_regmap_invalidate_device(d_data->reg_map, NEURONSPI_BOARDTABLE[d_data->combination_id].definition, freq_cnt);
			}
		}
		freq_cnt++;
	}
	return 0;
}

static void neuronspi_regmap_invalidate_device(struct regmap *reg_map, struct neuronspi_board_combination *device_def, u32 period_counter)
{
	int block_start, block_len, block_counter, current_period, period_len;
	int i;
	block_start = -1;
	block_len = -1;
	block_counter = 0;
	period_len = 1;

	for (i = 0; i < device_def->block_count; i++) {
		if (block_start == -1) {
			block_start = device_def->blocks[i];
			block_counter = 0;
		} else if (block_len == -1) {
			block_len = device_def->blocks[i];
		} else if ((block_counter != block_len)) {
			current_period = device_def->blocks[i] & 0x00FF0000;
			if ((block_counter + 1 != block_len) && ((device_def->blocks[i + 1] & 0x00FF0000) == current_period)) {
				period_len++;
			} else {
				switch (current_period) {
				case NEURONSPI_REGFLAG_ACC_AFAP: {
					regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					break;
				}
				case NEURONSPI_REGFLAG_ACC_10HZ: {
					if ((period_counter) % 5) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_1HZ: {
					if ((period_counter % 50) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_6SEC: {
					if ((period_counter % 300) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_1MIN: {
					if ((period_counter % 3000) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				case NEURONSPI_REGFLAG_ACC_15MIN: {
					if ((period_counter % 45000) == 0) {
						regcache_drop_region(reg_map, block_start + block_counter - period_len + 1, block_start + block_counter);
					}
					break;
				}
				default:
					break;
				}
				period_len = 1;
			}
			block_counter++;
		}
		if (block_counter == block_len) {
			block_counter = 0;
			block_start = -1;
			block_len = -1;
		}
	}
}

static u32 neuronspi_uart_tx_empty(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "NEURONSPI: UART TX Empty\n");
#endif
	return TIOCSER_TEMT;
}

static u32 neuronspi_uart_get_mctrl(struct uart_port *port)
{
	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART MCTRL Get\n");
#endif
	return TIOCM_DSR | TIOCM_CAR;
}

static void neuronspi_uart_set_mctrl(struct uart_port *port, u32 mctrl) {}

int	neuronspi_uart_ioctl (struct uart_port *port, unsigned int ioctl_code, unsigned long ioctl_arg)
{
	switch (ioctl_code) {
	case TIOCSETD: {
		printk(KERN_INFO "NEURONSPI: IOCTL TIOCSETD\n");
		return 1;
	}
	case 0x5480: {
		printk(KERN_INFO "NEURONSPI: IOCTL 0x5480\n");
		return 1;
	}
	case 0x5481: {
		printk(KERN_INFO "NEURONSPI: IOCTL 0x5481\n");
		return 1;
	}
	default: {
		return 0;
	}
	}
}

static void neuronspi_uart_set_parmrk(struct uart_port *port, int to)
{
	struct neuronspi_port *n_port;
	n_port = to_neuronspi_port(port, port);
}

static void neuronspi_uart_set_ldisc(struct uart_port *port, struct ktermios *kterm)
{
	struct neuronspi_port *n_port;
	n_port = to_neuronspi_port(port, port);
	if (kterm->c_line == N_PROFIBUS_FDL) {
		printk(KERN_INFO "NEURONSPI: PROFIBUS discipline set/n");
	}
	return;
}

static void neuronspi_uart_break_ctl(struct uart_port *port, int break_state)
{

}

static void neuronspi_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	s32 baud;
	struct neuronspi_port *n_port;
	n_port = to_neuronspi_port(port, port);
	if (old && old->c_iflag && old->c_iflag != termios->c_iflag) {
		printk(KERN_INFO "NEURONSPI: c_iflag termios:%d\n", termios->c_iflag);
	}
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: TERMIOS Set, p:%d, c_cflag:%x\n", port->line, termios->c_cflag);
#endif
	neuronspi_spi_uart_set_cflag(neuronspi_s_dev[n_port->dev_index], n_port->dev_port, termios->c_cflag);
	if (old && termios && (old->c_iflag & PARMRK) != (termios->c_iflag & PARMRK)) {
		if (termios->c_iflag & PARMRK) {
			neuronspi_uart_set_parmrk(port, 1); 		// TODO: Re-enable line discipline once finished
		} else {
			neuronspi_uart_set_parmrk(port, 0);
		}
	}
	if (old && termios && old->c_line != termios->c_line) {
		if (termios->c_line == N_PROFIBUS_FDL) {
			printk(KERN_INFO "NEURONSPI: Line Discipline change/n");
			neuronspi_uart_set_ldisc(port, termios);
		}
	}

	baud = uart_get_baud_rate(port, termios, old, 2400, 115200);
	uart_update_timeout(port, termios->c_cflag, baud);
}

static s32 neuronspi_uart_config_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	port->rs485 = *rs485;
	return 0;
}

// Initialise the module
static s32 neuronspi_uart_startup(struct uart_port *port)
{
	struct neuronspi_port *n_port = to_neuronspi_port(port, port);
	struct spi_device *spi = neuronspi_s_dev[n_port->dev_index];
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi);
	neuronspi_spi_set_irqs(spi, 0x5);
	if (d_data->poll_thread != NULL) {
		wake_up_process(d_data->poll_thread);
	} else if (d_data->no_irq) {
		d_data->poll_thread = kthread_create(neuronspi_uart_poll, (void *)d_data, "UART_poll_thread");
	}
	neuronspi_uart_power(port, 1);
	// TODO: /* Reset FIFOs*/
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: UART Startup\n");
#endif
	return 0;
}

static void neuronspi_uart_shutdown(struct uart_port *port)
{
    neuronspi_uart_power(port, 0);
}

static const char* neuronspi_uart_type(struct uart_port *port)
{
	return port->type == PORT_NEURONSPI ? "NEURONSPI_NAME" : NULL;
}

static s32 neuronspi_uart_request_port(struct uart_port *port)
{
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_DEBUG "NEURONSPI: Requested port %d\n", port->line);
#endif
	return 0;
}

static void neuronspi_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_NEURONSPI;
}

static s32 neuronspi_uart_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_NEURONSPI))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void neuronspi_uart_pm(struct uart_port *port, u32 state, u32 oldstate)
{
	neuronspi_uart_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

static void neuronspi_uart_null_void(struct uart_port *port)
{
	/* Do nothing */
}

static s32 neuronspi_uart_probe(struct spi_device* dev, u8 device_index)
{
	struct neuronspi_driver_data* driver_data = spi_get_drvdata(dev);
	struct sched_param sched_param = { .sched_priority = MAX_RT_PRIO / 2 };
	s32 i, j, ret, new_uart_count;
	struct neuronspi_uart_data *uart_data = driver_data->uart_data;

	if (uart_data->p == NULL) {
		uart_data->p = kzalloc(sizeof(struct neuronspi_port[NEURONSPI_MAX_UART]), GFP_KERNEL);
		for (i = 0; i < NEURONSPI_MAX_UART; i++) {
			uart_data->p[i].parent = uart_data;
		}
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: Allocated port structure for %d potential UART devices\n", NEURONSPI_MAX_UART);
#endif
	}

	new_uart_count = driver_data->uart_count + uart_data->p_count;

	// Initialise port data
	for (i = uart_data->p_count; i < new_uart_count; i++) {
		uart_data->p[i].dev_index = device_index;
		uart_data->p[i].dev_port = i - uart_data->p_count;
		uart_data->p[i].line		= i;
		uart_data->p[i].port.dev	= &(dev->dev);
		uart_data->p[i].port.irq	= dev->irq;
		uart_data->p[i].port.type	= PORT_NEURONSPI;
		uart_data->p[i].port.fifosize	= NEURONSPI_FIFO_SIZE;
		uart_data->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		uart_data->p[i].port.iotype	= UPIO_PORT;
		uart_data->p[i].port.uartclk	= 9600;
		uart_data->p[i].port.rs485_config = neuronspi_uart_config_rs485;
		uart_data->p[i].port.ops	= &neuronspi_uart_ops;
		uart_data->p[i].port.line	= neuronspi_uart_alloc_line();
		spin_lock_init(&uart_data->p[i].port.lock);
		if (uart_data->p[i].port.line >= NEURONSPI_MAX_DEVS) {
			ret = -ENOMEM;
		}
		kthread_init_work(&(uart_data->p[i].tx_work), neuronspi_uart_tx_proc);
		kthread_init_work(&(uart_data->p[i].rx_work), neuronspi_uart_rx_proc);
		kthread_init_work(&(uart_data->p[i].irq_work), neuronspi_uart_ist);
		uart_add_one_port(driver_data->serial_driver, &uart_data->p[i].port);
		printk(KERN_INFO "NEURONSPI: Added UART port %d\n", i);
	}

	// For ports on multiple SPI devices renumber the ports to correspond to SPI chip-select numbering
	if (uart_data->p_count) {
		// First remove all existing ports
		for (i = 0; i < new_uart_count; i++) {
			uart_remove_one_port(driver_data->serial_driver, &uart_data->p[i].port);
			clear_bit(uart_data->p[i].port.line, &neuronspi_lines);
			kthread_flush_worker(&uart_data->kworker);
		}
		// Now add the ports in the correct order
		for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
			if (neuronspi_s_dev[i] != NULL) {
				driver_data = spi_get_drvdata(neuronspi_s_dev[i]);
#if NEURONSPI_DETAILED_DEBUG > 0
				printk(KERN_DEBUG "NEURONSPI: Renumber not NULL %d UC:%d\n", i, driver_data->uart_count);
#endif
				if (driver_data->uart_count) {
					for (j = 0; j < new_uart_count; j++) {
						if (uart_data->p[j].dev_index == i) {
							uart_data->p[j].port.dev	= &(neuronspi_s_dev[i]->dev);
							uart_data->p[j].port.irq	= neuronspi_s_dev[i]->irq;
							uart_data->p[j].port.type	= PORT_NEURONSPI;
							uart_data->p[j].port.fifosize	= NEURONSPI_FIFO_SIZE;
							uart_data->p[j].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
							uart_data->p[j].port.iotype	= UPIO_PORT;
							uart_data->p[j].port.uartclk	= 9800;
							uart_data->p[j].port.rs485_config = neuronspi_uart_config_rs485;
							uart_data->p[j].port.ops	= &neuronspi_uart_ops;
							uart_data->p[j].port.line	= neuronspi_uart_alloc_line();
							uart_add_one_port(driver_data->serial_driver, &uart_data->p[j].port);
#if NEURONSPI_DETAILED_DEBUG > 0
							printk(KERN_DEBUG "NEURONSPI: Added UART port %d\n", j);
#endif
						}
					}
				}
			}
		}
	}

	uart_data->p_count = new_uart_count;
	if (uart_data->kworker_task == NULL) {
#if NEURONSPI_DETAILED_DEBUG > 0
		printk(KERN_DEBUG "NEURONSPI: KWorker Task is NULL\n");
#endif

		kthread_init_worker(&uart_data->kworker);

		uart_data->kworker_task = kthread_run(kthread_worker_fn, &uart_data->kworker,
						  "neuronspi");
		if (IS_ERR(uart_data->kworker_task)) {
			ret = PTR_ERR(uart_data->kworker_task);
		}
		sched_setscheduler(uart_data->kworker_task, SCHED_FIFO, &sched_param);
	}
	return ret;
}

static s32 neuronspi_uart_remove(struct neuronspi_uart_data *u_data)
{
	struct neuronspi_driver_data *d_data;
	struct spi_device *spi;
	s32 i;

	for (i = 0; i < NEURONSPI_MAX_DEVS; i++) {
		if (!(neuronspi_s_dev[i] == NULL)) {
			spi = neuronspi_s_dev[i];
			d_data = spi_get_drvdata(spi);
			if (d_data->poll_thread != NULL) {
				kthread_stop(d_data->poll_thread);
			}
		}
	}
	for (i = 0; i < u_data->p_count; i++) {
		uart_remove_one_port(neuronspi_uart, &u_data->p[i].port);
		clear_bit(u_data->p[i].port.line, &neuronspi_lines);
		neuronspi_uart_power(&u_data->p[i].port, 0);
	}

	kthread_flush_worker(&u_data->kworker);
	return 0;
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
	message_buf = kzalloc(NEURONSPI_SPI_LED_SET_MESSAGE_LEN, GFP_KERNEL);
	recv_buf = kzalloc(NEURONSPI_SPI_LED_SET_MESSAGE_LEN, GFP_KERNEL);
	memcpy(message_buf, NEURONSPI_SPI_LED_SET_MESSAGE, NEURONSPI_SPI_LED_SET_MESSAGE_LEN);
	message_buf[2] += id;
	if (brightness > 0) {
		message_buf[1] = 0x01;
	} else {
		message_buf[1] = 0x00;
	}
	crc1 = neuronspi_spi_crc(message_buf, 4, 0);
	memcpy(&message_buf[4], &crc1, 2);
	if (!d_data->reserved_device) {
		neuronspi_spi_send_message(spi_dev, message_buf, recv_buf, NEURONSPI_SPI_LED_SET_MESSAGE_LEN, frequency, 25, 0);
	}
	printk(KERN_INFO "NEURONSPI: Brightness set to %d on led %d\n", brightness, id);
	kfree(message_buf);
	kfree(recv_buf);
	recv_buf = kzalloc(8, GFP_KERNEL);
	printk(KERN_INFO "NEURONSPI: REGMAP TEST: %d\n", regmap_bulk_read(d_data->reg_map, 1000, (void*)recv_buf, 4));
	printk(KERN_INFO "NEURONSPI: REGMAP TEST OUT: %d %d %d %d\n", recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3]);
	kfree(recv_buf);
	recv_buf = kzalloc(8, GFP_KERNEL);
	printk(KERN_INFO "NEURONSPI: REGMAP TEST: %d\n", regmap_bulk_read(d_data->reg_map, 1000, (void*)recv_buf, 4));
	printk(KERN_INFO "NEURONSPI: REGMAP TEST OUT: %d %d %d %d\n", recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3]);
	kfree(recv_buf);
	recv_buf = kzalloc(8, GFP_KERNEL);
	printk(KERN_INFO "NEURONSPI: REGMAP TEST: %d\n", regmap_bulk_read(d_data->reg_map, 1000, (void*)recv_buf, 4));
	printk(KERN_INFO "NEURONSPI: REGMAP TEST OUT: %d %d %d %d\n", recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3]);
	kfree(recv_buf);
}

int neuronspi_spi_gpio_di_get(struct spi_device* spi_dev, u32 id)
{
	u8 *recv_buf;
	bool ret = 0;
	u32 offset = id / 16;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	recv_buf = kzalloc(4, GFP_KERNEL);
	printk(KERN_INFO "NEURONSPI: REGMAP TEST: %d\n", regmap_read(d_data->reg_map, d_data->regstart_table->di_val_reg + offset, (void*)recv_buf));
	if (*recv_buf & (0x1 << offset)) {
		ret = 1;
	}
	kfree(recv_buf);
	printk(KERN_INFO "NEURONSPI: GPIO DI %d get %d\n", id, ret);
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
	printk(KERN_INFO "NEURONSPI: GPIO DO %d set %d\n", id, value);
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
	printk(KERN_INFO "NEURONSPI: GPIO RO %d set %d\n", id, value);
	return ret;
}


static void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness)
{
	struct neuronspi_led_driver *led = container_of(ldev, struct neuronspi_led_driver, ldev);
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(led->spi);
	spin_lock(&led->lock);
	led->brightness = brightness;
	kthread_queue_work(&n_spi->primary_worker, &led->led_work);
	spin_unlock(&led->lock);
}

int neuronspi_regmap_hw_read(void *context, const void *reg_buf, size_t reg_size, void *val_buf, size_t val_size) {
	const u16 *mb_reg_buf = reg_buf;
	u16 *mb_val_buf = val_buf;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	u8 *inp_buf;
	u8 *outp_buf;
	int i, write_length;
	int block_counter = 0;
	if (context == NULL) {
		return 0;
	}
	spi = context;
	n_spi = spi_get_drvdata(spi);
	if (n_spi == NULL) {
		return 0;
	}
	for (i = 0; i < (reg_size / 2); i++) {
		// Check for continuity and read the largest possible continuous block
		if (block_counter == ((reg_size / 2) - 1) || ((mb_reg_buf[i] + 1) != mb_reg_buf[i + 1])) {
			write_length = neuronspi_spi_compose_multiple_register_read(block_counter + 1, mb_reg_buf[i - block_counter], &inp_buf, &outp_buf);
			neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 125, 1);
			memcpy(&mb_val_buf[i - block_counter], &outp_buf[NEURONSPI_HEADER_LENGTH], (block_counter + 1) * 2);
			kfree(inp_buf);
			kfree(outp_buf);
			block_counter = 0;
		} else {
			block_counter++;
		}
	}
	printk(KERN_INFO "NEURONSPI: RM_READ %d %x %d %x\n", reg_size, mb_reg_buf[0], val_size, mb_val_buf[0]);
	return 0;
}

int neuronspi_regmap_hw_reg_read(void *context, unsigned int reg, unsigned int *val) {
	struct spi_device *spi = context;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u8 *inp_buf;
	u8 *outp_buf;
	int write_length;
	printk(KERN_INFO "NEURONSPI: RM_REG_READ\n");
	write_length = neuronspi_spi_compose_single_register_read(reg, &inp_buf, &outp_buf);
	neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1);
	memcpy(val, &outp_buf[NEURONSPI_HEADER_LENGTH], sizeof(u16));
	kfree(inp_buf);
	kfree(outp_buf);
	return 0;
}

int neuronspi_regmap_hw_write(void *context, const void *data, size_t count) {
	BUG_ON(count < 1);
	return neuronspi_regmap_hw_gather_write(context, data, 1, data + 1, count - 1);
}

int neuronspi_regmap_hw_reg_write(void *context, unsigned int reg, unsigned int val) {
	struct spi_device *spi = context;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u8 *inp_buf;
	u8 *outp_buf;
	int write_length;
	write_length = neuronspi_spi_compose_single_register_write(reg, &inp_buf, &outp_buf, (val >> 8));
	printk(KERN_INFO "HW_REG_WRITE l:%d, r:%d, v:%d\n", write_length, reg, (val >> 8));
	neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1);
	memcpy(&val, &outp_buf[NEURONSPI_HEADER_LENGTH], sizeof(u16));
	kfree(inp_buf);
	kfree(outp_buf);
	return 0;
}

int neuronspi_regmap_hw_gather_write(void *context, const void *reg, size_t reg_size, const void *val, size_t val_size) {
	u16 *mb_reg_buf = (u16*)reg;
	u32 *mb_val_buf = (u32*)val;
	struct spi_device *spi = context;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u8 *inp_buf;
	u8 *outp_buf;
	int i, write_length;
	int block_counter = 0;
	printk(KERN_INFO "HW_REG_GATHER_WRITE:%d, %d, %x, %x\n", val_size, reg_size, mb_reg_buf[0], mb_val_buf[0]);
	if (reg_size == 1) {
		neuronspi_regmap_hw_reg_write(context,mb_reg_buf[0],mb_val_buf[0]);
	} else {
		for (i = 0; i < reg_size; i++) {
			// Swap endianness
			cpu_to_be16s((u16*)&(mb_val_buf[i]));
			// Check for continuity and read the largest possible continuous block
			if (block_counter == (reg_size - 1) || ((mb_reg_buf[i] + 1) != mb_reg_buf[i + 1]))  {
				write_length = neuronspi_spi_compose_multiple_register_write(block_counter, mb_reg_buf[i - block_counter], &inp_buf, &outp_buf,
						                                                     (u8*)(&mb_val_buf[i - block_counter]));
				neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 125, 1);
				block_counter = 0;
				kfree(inp_buf);
				kfree(outp_buf);
			} else {
				block_counter++;
			}
		}
	}
	return 0;
}

static int neuronspi_create_reg_starts(struct neuronspi_board_regstart_table *out_table, struct neuronspi_board_combination *board) {
	if (board->features.di_count > 0) {
		out_table->di_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DI_READ);
		out_table->di_counter_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DI_COUNTER_UPPER);
		out_table->di_deboun_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DI_DEBOUNCE);
		out_table->di_direct_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DS_ENABLE);
		out_table->di_polar_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DS_POLARITY);
		out_table->di_toggle_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DS_TOGGLE);
	}
	if (board->features.do_count > 0) {
		out_table->do_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DO_RW);
		out_table->do_pwm_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_PWM_DUTY);
		out_table->do_pwm_c_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_PWM_CYCLE);
		out_table->do_pwm_ps_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_PWM_PRESCALE);
	}
	if (board->features.led_count > 0) {
		out_table->led_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_LED_RW);
	}
	if (board->features.light_count > 0) {
		// TODO: Fill in light bus registers
	}
	if (board->features.ro_count > 0) {
		out_table->ro_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_DO_RW);
	}
	if (board->features.sec_ai_count > 0) {
		out_table->sec_ai_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_VER2_READ_LOWER);
		out_table->sec_ai_mode_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_VER2_MODE);
	}
	if (board->features.sec_ao_count > 0) {
		out_table->sec_ao_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_VER2_RW);
	}
	if (board->features.stm_ao_count > 0) {
		out_table->stm_ao_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN);
		out_table->stm_ao_mode_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_MODE);
		out_table->stm_ao_vol_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_V_ERR);
		out_table->stm_ao_vol_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_V_OFF);
		out_table->stm_ao_curr_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_I_ERR);
		out_table->stm_ao_curr_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AO_BRAIN_I_OFF);
	}
	if (board->features.stm_ai_count > 0) {
		out_table->stm_ai_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN);
		out_table->stm_ai_mode_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_MODE);
		out_table->stm_ai_vol_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_V_ERR);
		out_table->stm_ai_vol_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_V_OFF);
		out_table->stm_ai_curr_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_I_ERR);
		out_table->stm_ai_curr_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AI_BRAIN_I_OFF);
		out_table->stm_aio_val_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AIO_BRAIN);
		out_table->stm_aio_vol_err = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AIO_BRAIN_ERR);
		out_table->stm_aio_vol_off = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_AIO_BRAIN_OFF);
		out_table->vref_inp = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_V_REF_INP);
	}
	out_table->uart_queue_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_TX_QUEUE_LEN);
	out_table->uart_conf_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_RS485_CONFIG);
	out_table->vref_int = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_V_REF_INT);
	out_table->wd_timeout_reg = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_MWD_TO);
	out_table->sys_serial_num = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_SERIAL_NR_LOWER);
	out_table->sys_sw_ver = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_SW_VER);
	out_table->sys_hw_ver = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_HW_VER);
	out_table->sys_hw_flash_ver = neuronspi_find_reg_start(board, NEURONSPI_REGFUN_FLASH_HW_VER);
	return 0;
}

static s32 neuronspi_find_reg_start(struct neuronspi_board_combination *board, u16 regfun) {
	int i;
	int block_start = -1;
	int block_len = -1;
	int block_counter = 0;
	for (i = 0; i < board->block_count; i++) {
		if (block_start == -1) {
			block_start = board->blocks[i];
		} else if (block_len == -1) {
			block_len = board->blocks[i];
		} else if ((board->blocks[i] & 0xFFFF) == regfun) {
			//printk(KERN_INFO "NEURONSPI: Reg Start Fun: %x RegS: %d", regfun, block_start + block_counter);
			return block_start + block_counter;
		} else {
			block_counter++;
		}
		if (block_counter == block_len) {
			block_counter = 0;
			block_start = -1;
			block_len = -1;
		}
	}
	return -1;
}

static s32 neuronspi_find_model_id(u32 probe_count)
{
	struct neuronspi_driver_data *n_spi;
	int i,j, ret = -1;
	u8 *inv = kzalloc(sizeof(*inv) * NEURONSPI_MODELTABLE_LEN, GFP_KERNEL);
	for (i = 0; i < probe_count; i++) {
		if (neuronspi_s_dev[i]) {
			n_spi = spi_get_drvdata(neuronspi_s_dev[i]);
			for (j = 0; j < NEURONSPI_MODELTABLE_LEN; j++) {
				if (i + 1 > NEURONSPI_MODELTABLE[j].combination_count) {
					inv[j] = 1;
				} else if (NEURONSPI_MODELTABLE[j].combinations[i].combination_board_id != n_spi->combination_id) {
					inv[j] = 1;
				}
			}
		} else {
			for (j = 0; j < NEURONSPI_MODELTABLE_LEN; j++) {
				if (i + 1 < NEURONSPI_MODELTABLE[j].combination_count) {
					inv[j] = 1;
				}
			}
		}
	}
	for (i = 0; i < NEURONSPI_MODELTABLE_LEN; i++) {
		if (inv[i] != 1) {
			ret = i;
			break;
		}
	}
	kfree(inv);
	return ret;
}

static ssize_t neuronspi_show_model(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	if (neuronspi_model_id != -1) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].model_name);
	}
	return ret;
}

static ssize_t neuronspi_show_eeprom(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	if (neuronspi_model_id != -1) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].eeprom_name);
	}
	return ret;
}



static ssize_t neuronspi_spi_show_serial(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val[2] = {0, 0};
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_serial_num, val);
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_serial_num + 1, &(val[1]));
		ret = scnprintf(buf, 255, "%d\n", val[0]);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_hw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_hw_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x\n", (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_hw_flash_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_hw_flash_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x\n", (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_sw_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x%x\n", (val & 0xF00) >> 8, (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_uart_queue_length(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->regstart_table->uart_queue_reg) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->uart_queue_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_uart_config(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->uart_conf_reg, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_uart_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->uart_conf_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_watchdog_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->wd_val_reg, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_watchdog_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->wd_val_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_watchdog_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->wd_timeout_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_watchdog_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->wd_timeout_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_show_pwm_presc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_ps_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_pwm_presc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_ps_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_show_pwm_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_c_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_pwm_freq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_c_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_show_pwm_cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_reg + n_do->do_index, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_pwm_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_reg + n_do->do_index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_di_show_counter(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + (2 * n_di->di_index), &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_di_store_counter(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + (2 * n_di->di_index), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_di_show_debounce(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_deboun_reg + n_di->di_index, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_di_store_debounce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_deboun_reg + n_di->di_index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_di_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_val_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_do_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->do_count > n_do->do_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), &val);
		val &= 0x1 << (n_do->do_index % 15);
		val = val >> (n_do->do_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_do_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->do_count > n_do->do_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), &val);
		val &= ~(0x1 << (n_do->do_index % 15));
		val |= inp << (n_do->do_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_ro_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ro_count > n_ro->ro_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), &val);
		val &= 0x1 << (n_ro->ro_index % 15);
		val = val >> (n_ro->ro_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_ro_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ro_count > n_ro->ro_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), &val);
		val &= ~(0x1 << (n_ro->ro_index % 15));
		val |= inp << (n_ro->ro_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), val);
	}
err_end:
	return count;
}


static ssize_t neuronspi_spi_gpio_show_ds_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ds_toggle(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ds_polarity(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_ds_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_store_ds_toggle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_store_ds_polarity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_show_regmap(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->reg_map) {
		spin_lock(&n_spi->sysfs_regmap_lock);
		regmap_read(n_spi->reg_map, n_spi->sysfs_regmap_target, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
		spin_unlock(&n_spi->sysfs_regmap_lock);
	}
	return ret;
}

static ssize_t neuronspi_store_regmap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->reg_map && val < 65536) {
		spin_lock(&n_spi->sysfs_regmap_lock);
		n_spi->sysfs_regmap_target = val;
		spin_unlock(&n_spi->sysfs_regmap_lock);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_board(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != -1 && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_lboard_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != -1 && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%d\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->lower_board_id);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_uboard_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != -1 && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%d\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->upper_board_id);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_do_prefix(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi->features && n_spi->features->do_count > 0 && n_spi->do_driver) {
		ret = scnprintf(buf, 255, "%s_%d\n", n_spi->do_driver[n_do->do_index]->gpio_c.label, n_spi->neuron_index + 1);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_di_prefix(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi->features && n_spi->features->di_count > 0 && n_spi->di_driver) {
		ret = scnprintf(buf, 255, "%s_%d\n", n_spi->di_driver[n_di->di_index]->gpio_c.label, n_spi->neuron_index + 1);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ro_prefix(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi->features && n_spi->features->ro_count > 0 && n_spi->ro_driver) {
		ret = scnprintf(buf, 255, "%s_%d\n", n_spi->ro_driver[n_ro->ro_index]->gpio_c.label, n_spi->neuron_index + 1);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_do_base(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi->features && n_spi->features->do_count > 0 && n_spi->do_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->do_driver[n_do->do_index]->gpio_c.base);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_di_base(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi->features && n_spi->features->di_count > 0 && n_spi->di_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->di_driver[n_di->di_index]->gpio_c.base);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ro_base(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi->features && n_spi->features->ro_count > 0 && n_spi->ro_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->ro_driver[n_ro->ro_index]->gpio_c.base);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_do_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi->features && n_spi->features->do_count > 0 && n_spi->do_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->do_driver[n_do->do_index]->gpio_c.ngpio);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_di_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi->features && n_spi->features->di_count > 0 && n_spi->di_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->di_driver[n_di->di_index]->gpio_c.ngpio);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ro_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi->features && n_spi->features->ro_count > 0 && n_spi->ro_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->ro_driver[n_ro->ro_index]->gpio_c.ngpio);
	}
	return ret;
}

static s32 neuronspi_spi_probe(struct spi_device *spi)
{
	const struct neuronspi_devtype *devtype;
	struct sched_param sched_param = { .sched_priority = MAX_RT_PRIO / 2 };

	struct neuronspi_driver_data *n_spi;

	s32 ret, i, no_irq = 0;
	u8 uart_count = 0;
	n_spi = kzalloc(sizeof *n_spi, GFP_KERNEL);
	spin_lock(&neuronspi_probe_spinlock);
	neuronspi_probe_count++;
	spin_unlock(&neuronspi_probe_spinlock);
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
		//print_device_tree_node(spi->dev.of_node->parent, 0);
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
	sched_setscheduler(n_spi->primary_worker_task, SCHED_FIFO, &sched_param);


	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	n_spi->recv_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	n_spi->send_buf = kzalloc(NEURONSPI_BUFFER_MAX, GFP_KERNEL);
	n_spi->first_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_KERNEL);	// allocate space for initial probe
	n_spi->second_probe_reply = kzalloc(NEURONSPI_PROBE_MESSAGE_LEN, GFP_KERNEL); // allocate space for uart probe
	n_spi->lower_board_id = 0xFF;
	n_spi->upper_board_id = 0xFF;
	n_spi->combination_id = 0xFF;
	n_spi->spi_driver = &neuronspi_spi_driver;

	memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
	neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);

	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point
	i = 0;
	do {
		memcpy(n_spi->send_buf, &NEURONSPI_PROBE_MESSAGE, NEURONSPI_PROBE_MESSAGE_LEN);
		memset(n_spi->first_probe_reply, 0, NEURONSPI_PROBE_MESSAGE_LEN);
		neuronspi_spi_send_message(spi, n_spi->send_buf, n_spi->first_probe_reply, NEURONSPI_PROBE_MESSAGE_LEN, NEURONSPI_DEFAULT_FREQ, 25, 1);
		i++;
		if (i > 1000) {	// Sanity check to prevent looping if we get constant UART/0x41 messages
			ret = -4;
			kfree(n_spi);
			printk(KERN_INFO "NEURONSPI: Probe did not detect a valid Neuron device on CS %d\n", spi->chip_select);
			return ret;
		}
	} while (n_spi->first_probe_reply[0] == 0x41);	// UART messages should be ignored

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
	} else {
		ret = -5;
		kfree(n_spi);
		printk(KERN_INFO "NEURONSPI: Probe did not detect a valid Neuron device on CS %d\n", spi->chip_select);
		return ret;
	}

	if (n_spi->lower_board_id != 0xFF && n_spi->combination_id != 0xFF) {
		n_spi->features = kzalloc(sizeof(struct neuronspi_board_features), GFP_KERNEL);
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
	printk(KERN_INFO "NEURONSPI: Neuron device %s on CS %d uses SPI communication freq. %d Hz\n",
			NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name,
			spi->chip_select, n_spi->ideal_frequency);

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
		n_spi->led_driver = kzalloc(sizeof(struct neuronspi_led_driver) * n_spi->features->led_count, GFP_KERNEL);
		for (i = 0; i < n_spi->features->led_count; i++) {
			kthread_init_work(&(n_spi->led_driver[i].led_work), neuronspi_led_proc);
		}

	}


	if (uart_count && neuronspi_uart == NULL) {	// Register UART if not registered
		neuronspi_uart = kzalloc(sizeof(struct uart_driver), GFP_KERNEL);
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
			neuronspi_uart_glob_data = kzalloc(sizeof(struct neuronspi_uart_data), GFP_KERNEL);
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
	spin_lock(&neuronspi_probe_spinlock);
	neuronspi_s_dev[n_spi->neuron_index] = spi;
	spi_set_drvdata(neuronspi_s_dev[n_spi->neuron_index], n_spi);
	if (neuronspi_probe_count == NEURONSPI_MAX_DEVS) {
		printk(KERN_INFO "NEURONSPI: MAXPROBECOUNT\n");
		neuronspi_model_id = neuronspi_find_model_id(neuronspi_probe_count);
	}
	spin_unlock(&neuronspi_probe_spinlock);
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
	n_spi->board_device->dev.groups = neuron_board_attr_groups;
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
				n_spi->led_driver[i].name[19] = i + '1';
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
			n_spi->di_driver = kzalloc(sizeof(struct neuronspi_di_driver*) * n_spi->features->di_count, GFP_KERNEL);
			for (i = 0; i < n_spi->features->di_count; i++) {
				n_spi->di_driver[i] = kzalloc(sizeof(struct neuronspi_di_driver), GFP_KERNEL);
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
			n_spi->do_driver = kzalloc(sizeof(struct neuronspi_do_driver*) * n_spi->features->do_count, GFP_KERNEL);
			for (i = 0; i < n_spi->features->do_count; i++) {
				n_spi->do_driver[i] = kzalloc(sizeof(struct neuronspi_do_driver), GFP_KERNEL);
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
				n_spi->do_driver[i]->gpio_c.ngpio = n_spi->features->do_count;
				n_spi->do_driver[i]->gpio_c.base = -1;
				n_spi->do_driver[i]->gpio_c.direction_output = neuronspi_gpio_do_direction_output;
				n_spi->do_driver[i]->gpio_c.set = neuronspi_gpio_do_set;
				gpiochip_add_data(&n_spi->do_driver[i]->gpio_c, n_spi->do_driver[i]);
			}
		}

		if (n_spi->features->ro_count) {
			n_spi->ro_driver = kzalloc(sizeof(struct neuronspi_ro_driver*) * n_spi->features->ro_count, GFP_KERNEL);
			for (i = 0; i < n_spi->features->ro_count; i++) {
				n_spi->ro_driver[i] = kzalloc(sizeof(struct neuronspi_ro_driver), GFP_KERNEL);
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
				n_spi->ro_driver[i]->gpio_c.ngpio = n_spi->features->ro_count;
				n_spi->ro_driver[i]->gpio_c.base = -1;
				n_spi->ro_driver[i]->gpio_c.direction_output = neuronspi_gpio_ro_direction_output;
				n_spi->ro_driver[i]->gpio_c.set = neuronspi_gpio_ro_set;
				gpiochip_add_data(&n_spi->ro_driver[i]->gpio_c, n_spi->ro_driver[i]);
			}
		}

#endif
		if (n_spi->features->stm_ai_count) {
			n_spi->stm_ai_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_stm_ai_data));
			((struct neuronspi_stm_ai_data*)iio_priv(n_spi->stm_ai_driver))->parent = spi;
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
			n_spi->stm_ao_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_stm_ai_data));
			((struct neuronspi_stm_ao_data*)iio_priv(n_spi->stm_ao_driver))->parent = spi;
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
			n_spi->sec_ai_driver = kzalloc(sizeof(struct neuronspi_sec_ai_data*) * n_spi->features->sec_ai_count, GFP_KERNEL);
			for (i = 0; i < n_spi->features->sec_ai_count; i++) {
				n_spi->sec_ai_driver[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_sec_ai_data));
				((struct neuronspi_sec_ai_data*)iio_priv(n_spi->sec_ai_driver[i]))->parent = spi;
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
			n_spi->sec_ao_driver = kzalloc(sizeof(struct neuronspi_sec_ao_data*) * n_spi->features->sec_ao_count, GFP_KERNEL);
			for (i = 0; i < n_spi->features->sec_ao_count; i++) {
				n_spi->sec_ao_driver[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_sec_ao_data));
				((struct neuronspi_sec_ao_data*)iio_priv(n_spi->sec_ao_driver[i]))->parent = spi;
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
		n_spi->uart_buf = kzalloc(NEURONSPI_FIFO_SIZE, GFP_KERNEL);
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

static s32 neuronspi_spi_remove(struct spi_device *spi)
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

static s32 char_register_driver(void)
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

static s32 char_unregister_driver(void)
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
	neuronspi_spi_w_spinlock = kzalloc(sizeof(struct spinlock), GFP_KERNEL);
	spin_lock_init(neuronspi_spi_w_spinlock);
	mutex_init(&neuronspi_master_mutex);
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
MODULE_DESCRIPTION("UniPi Neuron driver");
