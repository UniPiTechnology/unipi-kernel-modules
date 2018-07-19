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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_IIO_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_IIO_H_

/************
 * Includes *
 ************/

#include "unipi_common.h"

/*************************
 * Function Declarations *
 *************************/

int neuronspi_iio_stm_ai_read_raw(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
int neuronspi_iio_stm_ao_read_raw(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
int neuronspi_iio_stm_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask);
int neuronspi_iio_sec_ai_read_raw(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
int neuronspi_iio_sec_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask);

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_IIO_H_ */
