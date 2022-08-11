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

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_MODBUS_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_MODBUS_H_

#include <linux/version.h>
#include "unipi_common.h"
#include "unipi_channel.h"

#define UNIPI_MODBUS_DEVICE_NAME		"unipichannel"
#define UNIPI_MODBUS_DEVICE_NAME_T		"unipichannel%d"
#define UNIPI_MODBUS_DEVICE_CLASS		"unipi_modbus"
#define UNIPI_MODBUS_BUFFER_MAX			1152
#define UNIPI_MODBUS_HEADER_SIZE		4

enum UNIPI_MODBUS_OP
{
	UNIPI_MODBUS_OP_READBIT   = 0x01,
	UNIPI_MODBUS_OP_READREG   = 0x04,
	UNIPI_MODBUS_OP_WRITEBIT  = 0x05,
	UNIPI_MODBUS_OP_WRITEREG  = 0x06,
	UNIPI_MODBUS_OP_WRITEBITS = 0x0F,
};

struct device* unipi_modbus_classdev_register(struct unipi_channel *channel, u8 address);
void unipi_modbus_classdev_unregister(struct device *dev);

struct unipi_channel * unipi_modbus_dev_by_address(u8 modbus_address);

int __init unipi_modbus_init(void);
void __exit unipi_modbus_exit(void);

#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_MODBUS_H_ */
