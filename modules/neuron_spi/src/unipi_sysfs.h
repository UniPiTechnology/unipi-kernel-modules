/*
 * UniPi Neuron tty serial driver - Copyright (C) 2018 UniPi Technologies
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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_SYSFS_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_SYSFS_H_

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
#include <asm/gpio.h>

#include "unipi_common.h"
#include "unipi_platform.h"

extern const struct attribute_group neuron_stm_ai_group;
extern const struct attribute_group neuron_stm_ao_group;
extern const struct attribute_group neuron_sec_ai_group;
extern const struct attribute_group neuron_sec_ao_group;

extern const struct attribute_group *neuron_plc_attr_groups[];
extern const struct attribute_group *neuron_board_attr_groups[];
extern const struct attribute_group *neuron_gpio_di_attr_groups[];
extern const struct attribute_group *neuron_gpio_do_attr_groups[];
extern const struct attribute_group *neuron_gpio_ro_attr_groups[];

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_SYSFS_H_ */
