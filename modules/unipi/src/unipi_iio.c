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

/************
 * Includes *
 ************/

#include "unipi_iio.h"
#include "unipi_spi.h"

/************************
 * Non-static Functions *
 ************************/

int neuronspi_iio_stm_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask) {
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg, &ai_data->mode);
	switch(ai_data->mode) {
	case 0: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_stm_ai_read_voltage(indio_dev, ch, val, val2, mask);
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 1: {
		if (ch->type == IIO_CURRENT) {
			neuronspi_spi_iio_stm_ai_read_current(indio_dev, ch, val, val2, mask);
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_stm_ao_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, &ao_data->mode);
	switch(ao_data->mode) {
	case 3: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_stm_ao_read_resistance(indio_dev, ch, val, val2, mask);
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_stm_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, &ao_data->mode);
	switch(ao_data->mode) {
	case 0: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_stm_ao_set_voltage(indio_dev, ch, val, val2, mask);
			return 0;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 1: {
		if (ch->type == IIO_CURRENT) {
			neuronspi_spi_iio_stm_ao_set_current(indio_dev, ch, val, val2, mask);
			return 0;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_sec_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg, &ai_data->mode);
	switch(ai_data->mode) {
	case 0: {
		return -EINVAL;
		break;
	}
	case 1: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_sec_ai_read_voltage(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 2: {
		if (ch->type == IIO_VOLTAGE) {
			neuronspi_spi_iio_sec_ai_read_voltage(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 3: {
		if (ch->type == IIO_CURRENT) {
			neuronspi_spi_iio_sec_ai_read_current(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 4: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_sec_ai_read_resistance(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	case 5: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_sec_ai_read_resistance(indio_dev, ch, val, val2, mask);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
		break;
	}
	default: {
		return -EINVAL;
		break;
	}
	}
}

int neuronspi_iio_sec_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	if (ch->type == IIO_VOLTAGE) {
		neuronspi_spi_iio_stm_ao_set_voltage(indio_dev, ch, val, val2, mask);
		return 0;
	} else {
		return -EINVAL;
	}
}
