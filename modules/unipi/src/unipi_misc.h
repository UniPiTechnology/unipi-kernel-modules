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

/*************************
 * Function Declarations *
 *************************/

void neuronspi_led_proc(struct kthread_work *ws);
void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness);

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_MISC_H_ */
