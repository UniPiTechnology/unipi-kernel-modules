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

#include "unipi_iio.h"
#include "unipi_spi.h"

/************************
 * Non-static Functions *
 ************************/
/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */
void neuronspi_spi_iio_sec_ai_read_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_l);
	sec_ai_val_m = ((((u32)sec_ai_val_h) << 25) | (((u32)sec_ai_val_l) << 9)) >> 16;
	sec_ai_exp = (sec_ai_val_h & 0x7F80) >> 7;

	*val = sec_ai_val_m | 0x00010000;
	if (142 - ((int)sec_ai_exp) <= 0) {
		*val = (*val << (((int)sec_ai_exp) - 142)) * 1000;
		*val2 = 1;
	} else {
		*val = *val * 1000;
		*val2 = 2 << (142 - sec_ai_exp);
	}

}

void neuronspi_spi_iio_sec_ai_read_current(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;

	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_l);
	sec_ai_val_m = ((((u32)sec_ai_val_h) << 25) | (((u32)sec_ai_val_l) << 9)) >> 16;
	sec_ai_exp = (sec_ai_val_h & 0x7F80) >> 7;
	*val = sec_ai_val_m | 0x00010000;
	if (142 - ((int)sec_ai_exp) <= 0) {
		*val2 = 1;
		*val = *val << (((int)sec_ai_exp) - 142);
	} else {
		*val2 = 2 << (142 - sec_ai_exp);
	}
}

void neuronspi_spi_iio_sec_ai_read_resistance(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_ai_val_l = 0;
	u32 sec_ai_val_h = 0;
	u32 sec_ai_val_m = 0;
	u8 sec_ai_exp = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + 1 + (2 * ai_data->index), &sec_ai_val_h);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_val_reg + (2 * ai_data->index), &sec_ai_val_l);
	sec_ai_val_m = ((((u32)sec_ai_val_h) << 25) | (((u32)sec_ai_val_l) << 9)) >> 16;
	sec_ai_exp = (sec_ai_val_h & 0x7F80) >> 7;
	*val = sec_ai_val_m | 0x00010000;
	if (142 - ((int)sec_ai_exp) <= 0) {
		*val2 = 1;
		*val = *val << (((int)sec_ai_exp) - 142);
	} else {
		*val2 = 2 << (142 - sec_ai_exp);
	}
}

void neuronspi_spi_iio_sec_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 sec_true_val;
	if (val > 10000) val = 10000;
	sec_true_val = (val * 2) / 5;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ao_val_reg + ao_data->index, sec_true_val);
}

/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */
void neuronspi_spi_iio_stm_ai_read_voltage(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(iio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_ai_val = 0;
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_v_err = 0;
	u32 stm_v_off = 0;
	u64 stm_true_val = 0;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_val_reg, &stm_ai_val);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_vol_err, &stm_v_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_vol_off, &stm_v_off);
	stm_true_ref = ((u64)stm_v_int_ref) * 99000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = stm_true_ref * ((u64)(stm_ai_val * 1000));
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 4096);
	stm_true_val *= (10000 + stm_v_err);
	stm_true_val += stm_v_off;
	do_div(stm_true_val, 10000);
	*val = stm_true_val;
}

void neuronspi_spi_iio_stm_ai_read_current(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_ai_val = 0;
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_i_err = 0;
	u32 stm_i_off = 0;
	u64 stm_true_val = 0;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_val_reg, &stm_ai_val);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_curr_err, &stm_i_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_curr_off, &stm_i_off);
	stm_true_ref = ((u64)stm_v_int_ref) * 330000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = stm_true_ref * ((u64)(stm_ai_val * 1000));
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 4096);
	stm_true_val *= (10000 + stm_i_err);
	stm_true_val += stm_i_off;
	do_div(stm_true_val, 10000);
	*val = stm_true_val;
	*val2 = 1000;
}

void neuronspi_spi_iio_stm_ao_read_resistance(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_aio_val = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_aio_val_reg, &stm_aio_val);
	*val = stm_aio_val;
	*val2 = 10;
}


void neuronspi_spi_iio_stm_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_v_err = 0;
	u32 stm_v_off = 0;
	u64 stm_true_val = val;
	u64 stm_true_val_fraction = val2 / 100;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_err, &stm_v_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_off, &stm_v_off);
	stm_true_ref = ((u64)stm_v_int_ref) * (99000 + stm_v_err) * 1000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	stm_true_val = ((stm_true_val * 10000) + (stm_true_val_fraction) - stm_v_off) * 4095;
	do_div(stm_true_ref, stm_v_inp_ref);
	stm_v_inp_ref = stm_true_ref;
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 10000);
	if (stm_true_val > 4095) stm_true_val = 4095;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg, (unsigned int) stm_true_val);
}

void neuronspi_spi_iio_stm_ao_set_current(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_i_err = 0;
	u32 stm_i_off = 0;
	u64 stm_true_val = val;
	u64 stm_true_val_fraction = val2 / 100;
	u64 stm_true_ref = 0;
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_curr_err, &stm_i_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_curr_off, &stm_i_off);
	stm_true_ref = ((u64)stm_v_int_ref) * (330000 + stm_i_err) * 100;
	stm_v_inp_ref = stm_v_inp_ref * 1000;
	stm_true_val = (((stm_true_val * 10000) + (stm_true_val_fraction)) - stm_i_off) * 4095;
	do_div(stm_true_ref, stm_v_inp_ref);
	stm_v_inp_ref = stm_true_ref;
	do_div(stm_true_val, stm_v_inp_ref);
	do_div(stm_true_val, 10);
	if (stm_true_val > 4095) stm_true_val = 4095;
	regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_val_reg, (unsigned int)stm_true_val);
}

int neuronspi_iio_stm_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask) {
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg + ai_data->index, &ai_data->mode);
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

int neuronspi_iio_stm_ao_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg + ao_data->index, &ao_data->mode);
	switch(ao_data->mode) {
	case 3: {
		if (ch->type == IIO_RESISTANCE) {
			neuronspi_spi_iio_stm_ao_read_resistance(indio_dev, ch, val, val2, mask);
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

int neuronspi_iio_stm_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg + ao_data->index, &ao_data->mode);
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
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, &ai_data->mode);
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
		neuronspi_spi_iio_sec_ao_set_voltage(indio_dev, ch, val, val2, mask);
		return 0;
	} else {
		return -EINVAL;
	}
}


struct iio_dev* neuronspi_stm_ai_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
    struct spi_device *spi = neuronspi_s_dev[neuron_index];
    struct iio_dev *iio_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));

	((struct neuronspi_analog_data*)iio_priv(iio_driver))->parent = spi;
	((struct neuronspi_analog_data*)iio_priv(iio_driver))->index = 0;
	iio_driver->modes = INDIO_DIRECT_MODE;
	iio_driver->currentmode = INDIO_DIRECT_MODE;
	iio_driver->name = "ai_type_a";
	iio_driver->dev.parent = &(board_device->dev);
	dev_set_name(&iio_driver->dev, "ai_%d_1", neuron_index + 1);
	iio_driver->num_channels = 2;
	iio_driver->channels = neuronspi_stm_ai_chan_spec;
	iio_driver->info = &neuronspi_stm_ai_info;
	iio_device_register(iio_driver);

    return iio_driver;
}

struct iio_dev* neuronspi_stm_ao_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
    struct spi_device *spi = neuronspi_s_dev[neuron_index];
    struct iio_dev *iio_driver = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));

	((struct neuronspi_analog_data*)iio_priv(iio_driver))->parent = spi;
	((struct neuronspi_analog_data*)iio_priv(iio_driver))->index = 0;
	iio_driver->modes = INDIO_DIRECT_MODE;
	iio_driver->currentmode = INDIO_DIRECT_MODE;
	iio_driver->name = "ao_type_a";
	iio_driver->dev.parent = &(board_device->dev);
	dev_set_name(&iio_driver->dev, "ao_%d_1", neuron_index + 1);
	iio_driver->num_channels = 3;
	iio_driver->channels = neuronspi_stm_ao_chan_spec;
	iio_driver->info = &neuronspi_stm_ao_info;
	iio_device_register(iio_driver);

    return iio_driver;
}

struct iio_dev** neuronspi_sec_ai_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
    struct iio_dev **iio_driver_arr = kzalloc(sizeof(struct neuronspi_analog_data*) * io_count, GFP_ATOMIC);
    struct spi_device *spi = neuronspi_s_dev[neuron_index];
    int i;

	for (i = 0; i < io_count; i++) {
        iio_driver_arr[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
        ((struct neuronspi_analog_data*)iio_priv(iio_driver_arr[i]))->parent = spi;
		((struct neuronspi_analog_data*)iio_priv(iio_driver_arr[i]))->index = i;
		iio_driver_arr[i]->modes = INDIO_DIRECT_MODE;
        iio_driver_arr[i]->currentmode = INDIO_DIRECT_MODE;
		iio_driver_arr[i]->name = "ai_type_b";
		iio_driver_arr[i]->dev.parent = &(board_device->dev);
		dev_set_name(&iio_driver_arr[i]->dev, "ai_%d_%d", neuron_index + 1,  i + 1);
		iio_driver_arr[i]->num_channels = 3;
		iio_driver_arr[i]->channels = neuronspi_sec_ai_chan_spec;
		iio_driver_arr[i]->info = &neuronspi_sec_ai_info;
		iio_device_register(iio_driver_arr[i]);
    }
    return iio_driver_arr;
}

struct iio_dev** neuronspi_sec_ao_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
    struct iio_dev **iio_driver_arr = kzalloc(sizeof(struct neuronspi_analog_data*) * io_count, GFP_ATOMIC);
    struct spi_device *spi = neuronspi_s_dev[neuron_index];
    int i;

	for (i = 0; i < io_count; i++) {
        iio_driver_arr[i] = devm_iio_device_alloc(&(spi->dev), sizeof(struct neuronspi_analog_data));
        ((struct neuronspi_analog_data*)iio_priv(iio_driver_arr[i]))->parent = spi;
		((struct neuronspi_analog_data*)iio_priv(iio_driver_arr[i]))->index = i;
		iio_driver_arr[i]->modes = INDIO_DIRECT_MODE;
        iio_driver_arr[i]->currentmode = INDIO_DIRECT_MODE;
		iio_driver_arr[i]->name = "ao_type_b";
		iio_driver_arr[i]->dev.parent = &(board_device->dev);
		dev_set_name(&iio_driver_arr[i]->dev, "ao_%d_%d", neuron_index + 1,  i + 1);
		iio_driver_arr[i]->num_channels = 1;
		iio_driver_arr[i]->channels = neuronspi_sec_ao_chan_spec;
		iio_driver_arr[i]->info = &neuronspi_sec_ao_info;
		iio_device_register(iio_driver_arr[i]);
    }
    return iio_driver_arr;
}
