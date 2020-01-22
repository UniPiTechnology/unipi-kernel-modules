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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_

/************
 * Includes *
 ************/

#include "unipi_common.h"

// Instantiated once per LED
struct neuronspi_led_driver
{
	struct led_classdev	ldev;
	struct spi_device	*spi;
	struct kthread_work	led_work;
    int                 id;
    u16                 coil;
	int					brightness;
	char				name[sizeof("unipi:green:uled-x1")];
	spinlock_t			lock;
};


/*************************
 * Function Declarations *
 *************************/

void neuronspi_led_proc(struct kthread_work *ws);
void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness);
struct neuronspi_led_driver * neuronspi_led_probe(int uled_count, int sysled_count, int neuron_index, struct platform_device *board_device);

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_ */
