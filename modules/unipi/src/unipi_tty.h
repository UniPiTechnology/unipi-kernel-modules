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

#ifndef MODULES_UNIPI_SRC_UNIPI_TTY_H_
#define MODULES_UNIPI_SRC_UNIPI_TTY_H_

#include "unipi_common.h"

int neuronspi_tty_init(void);

#endif /* MODULES_UNIPI_SRC_UNIPI_TTY_H_ */
