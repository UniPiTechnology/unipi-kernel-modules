/*
 * UniPi PLC device driver - Copyright (C) 2018 UniPi Technology
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

#ifndef MODULES_SRC_UNIPI_COMMON_H_
#define MODULES_SRC_UNIPI_COMMON_H_

/************
 * Includes *
 ************/

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <uapi/linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/spi/spi.h>
#include <linux/leds.h>
#include <linux/uaccess.h>
#include <asm/termbits.h>
//#include <asm/gpio.h>

/***************
 * Definitions *
 ***************/

#define UNIPI_MODULE_MAJOR_VERSIONSTRING "2.00:2022:07:01"

#define UNIPI_SPI_DETAILED_DEBUG    2
#define UNIPI_MODBUS_DETAILED_DEBUG 0
#define UNIPI_TTY_DETAILED_DEBUG    2


#define lo(x) (x & 0xff)
#define hi(x) (x >> 8)


#endif /* MODULES_NEURON_SPI_SRC_UNIPI_COMMON_H_ */
