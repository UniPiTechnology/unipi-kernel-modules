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

/************
 * Includes *
 ************/

#include "unipi_gpio.h"
#include "unipi_spi.h"

/************************
 * Non-static Functions *
 ************************/

int neuronspi_gpio_di_direction_input(struct gpio_chip *chip, unsigned offset) {
	return 0;
}

int neuronspi_gpio_di_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return -EINVAL;
}

int neuronspi_gpio_di_get(struct gpio_chip *chip, unsigned offset) {
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

