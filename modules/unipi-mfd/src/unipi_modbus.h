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

#define UNIPI_MODBUS_DEVICE_NAME		"unipichannel"
#define UNIPI_MODBUS_DEVICE_NAME_T		"unipichannel%d"
#define UNIPI_MODBUS_DEVICE_CLASS		"unipi_modbus"
#define UNIPI_MODBUS_BUFFER_MAX			1152

struct device* unipi_modbus_classdev_register(struct spi_device *spi_dev, u8 address);
void unipi_modbus_classdev_unregister(struct device *dev);

struct spi_device * unipi_modbus_dev_by_address(u8 modbus_address);

int __init unipi_modbus_init(void);
void __exit unipi_modbus_exit(void);

#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_MODBUS_H_ */
