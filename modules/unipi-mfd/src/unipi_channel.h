/*
 * UniPi Multi Function Device Driver - Copyright (C) 2021 UniPi Technology
 *
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_CHANNEL_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_CHANNEL_H_

#include <linux/version.h>
#include "unipi_common.h"

struct unipi_channel;

typedef void (*OperationCallback)(void*, int);

typedef void (*RxCharCallback)(void*, u8, u8, int);
typedef void (*IntStatusCallback)(void*, u8);
typedef int  (*PingAsync)(void*, void*, OperationCallback);
typedef int  (*RegsAsync)(void *, unsigned int, unsigned int, u8*, void*,  OperationCallback);
typedef int  (*BitsAsync)(void *, unsigned int, unsigned int, u8*, void*,  OperationCallback);
typedef int  (*StrAsync)(void *, unsigned int, u8*, unsigned int, void*,  OperationCallback);
typedef void (*Populated)(void *);
typedef int  (*FirmwareOp)(void*, u8*, u8*, s32, int);



struct unipi_protocol_op 
{
	PingAsync  ping_async;
	RegsAsync  read_regs_async;
	RegsAsync  write_regs_async;
	BitsAsync  read_bits_async;
	BitsAsync  write_bits_async;
	FirmwareOp firmware_sync;
	int(*lock_op)(void*);
	void(*unlock_op)(void*);
	StrAsync       write_str_async;
	StrAsync       read_str_async;
	void(*populated)(void*);
};

struct unipi_channel
{
	struct regmap*			registers;
	struct regmap*			coils;
	void					*rx_self;
	RxCharCallback			rx_char_callback;
	void					*interrupt_self;
	IntStatusCallback		interrupt_status_callback;
	struct unipi_protocol_op  	*op;
	void		*proto_self;
	struct device	*dev;
	struct device *chrdev;
};


int unipi_channel_init(struct unipi_channel * channel, struct device* dev);
int unipi_channel_exit(struct unipi_channel * channel);
//struct unipi_channel* unipi_spi_get_channel(struct spi_device* spi_dev);


/* sync ops */
int unipi_ping_sync(struct unipi_channel *channel);
int unipi_write_bits_sync(struct unipi_channel *channel, unsigned int reg, unsigned int count, u8* data);
int unipi_read_bits_sync( struct unipi_channel *channel, unsigned int reg, unsigned int count, u8* data);
int unipi_write_regs_sync(struct unipi_channel *channel, unsigned int reg, unsigned int count, u16* data);
int unipi_read_regs_sync( struct unipi_channel *channel, unsigned int reg, unsigned int count, u16* data);
/**/

#define unipi_mfd_rx_char(channel, port, ch, remain) {if ((channel)->rx_char_callback) \
                         (channel)->rx_char_callback((channel)->rx_self, port,ch,remain);}

#define unipi_mfd_int_status(channel, int_status) {if ((channel)->interrupt_status_callback) \
                         (channel)->interrupt_status_callback((channel)->interrupt_self, int_status);}

#define unipi_read_str_async(channel, port, data, count, cb_data, cb_function) \
                         (channel)->op->read_str_async(channel->proto_self, port, data, count, cb_data, cb_function)

#define unipi_write_str_async(channel, port, data, count, cb_data, cb_function)  \
                         (channel)->op->write_str_async(channel->proto_self, port, data, count, cb_data, cb_function)

#define unipi_populated(channel) (channel)->op->populated(channel->proto_self)

#define unipi_read_regs_async(channel, reg, count, data, cb_data, cb_function) \
                         (channel)->op->read_regs_async((channel)->proto_self,  reg, count, data, cb_data, cb_function)

#define unipi_write_regs_async(channel, reg, count, data, cb_data, cb_function) \
                         (channel)->op->write_regs_async((channel)->proto_self,  reg, count, data, cb_data, cb_function)

#define unipi_read_bits_async(channel, reg, count, data, cb_data, cb_function) \
                         (channel)->op->read_bits_async((channel)->proto_self,  reg, count, data, cb_data, cb_function)

#define unipi_write_bits_async(channel, reg, count, data, cb_data, cb_function) \
                         (channel)->op->write_bits_async((channel)->proto_self,  reg, count, data, cb_data, cb_function)

#define unipi_ping_async(channel, cb_data, cb_function) \
                         (channel)->op->ping_async((channel)->proto_self, cb_data, cb_function)




static inline void unipi_channel_put(struct unipi_channel *channel)
{
	if (channel)
		put_device(channel->dev);
}

static inline struct unipi_channel* unipi_channel_get(struct unipi_channel *channel)
{
	if (!channel || !get_device(channel->dev))
		return NULL;
	return channel;
}


#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_CHANNEL_H_ */
