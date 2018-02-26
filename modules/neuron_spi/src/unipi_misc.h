/*
 * unipi_misc.h
 *
 *  Created on: 26 Feb 2018
 *      Author: Tom Knot <knot@faster.cz>
 */

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_

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


void neuronspi_led_proc(struct kthread_work *ws);
void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness);

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_ */
