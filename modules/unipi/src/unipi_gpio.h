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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_GPIO_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_GPIO_H_

/************
 * Includes *
 ************/

#include "unipi_common.h"


struct neuronspi_gpio_port {
	struct spi_device* spi;
	struct gpio_chip gpio_c;
	struct platform_device *plat_dev;
	u8 io_index;
};

struct neuronspi_gpio_driver {
    int count;
    struct neuronspi_gpio_port ports[1];
};


/*************************
 * Function Declarations *
 *************************/

struct neuronspi_gpio_driver * neuronspi_di_probe(int di_count, int neuron_index, struct platform_device *board_device);
struct neuronspi_gpio_driver * neuronspi_ro_probe(int ro_count, int neuron_index, struct platform_device *board_device);
struct neuronspi_gpio_driver * neuronspi_do_probe(int do_count, int neuron_index, struct platform_device *board_device);
void neuronspi_gpio_remove(struct neuronspi_gpio_driver * gpio_driver);

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_GPIO_H_ */
