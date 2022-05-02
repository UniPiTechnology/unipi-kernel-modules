/*
 * UniPi PLC device driver - Copyright (C) 2018 UniPi Technology
 * Author: Tomas Knot <tomasknot@gmail.com>
 *
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

//#include "unipi_iio.h"
#include "unipi_mfd.h"
#include "unipi_mfd_iogroup.h"

static const struct iio_chan_spec unipi_iio_stm_ai_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	},
	{
			.type = IIO_CURRENT,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	}
};

static const struct iio_chan_spec unipi_iio_stm_ao_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 1
	},
	{
			.type = IIO_CURRENT,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 1
	},
	{
			.type = IIO_RESISTANCE,
			.channel = 2,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	}
};

struct unipi_iio_stm_device
{
	struct regmap* map;
	int valreg;
	int modereg;
	u32 mode;
	u32 kvolt;
	int fvolt;
	u32 kamp;
	int famp;
};


/************************
 * Non-static Functions *
 ************************/

/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */

/* load constants to transform raw analog values to miliVolt
 * 		result is mul16 
 * 		transforation from raw is   V = ( raw * k14 + f14)  >>14
 * 		                       or   A = ( raw * k13 + f13 ) >> 13
 */
void unipi_iio_stm_read_vref(struct unipi_iio_stm_device *n_iio)
{
	u32 vref_int = 0;
	u32 vref_inp = 0;
	int err1=0, err2=0, offs1=0, offs2=0;

	regmap_read(n_iio->map, UNIPI_MFD_REG_VREFINT, &vref_int);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_VREFINP, &vref_inp);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AIV_DEV, &err1);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AIV_OFS, &offs1);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AIA_DEV, &err2);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AIA_OFS, &offs2);
	// safety check or set defaults
	vref_inp = vref_inp ? : 11328;
	vref_int = vref_inp ? : 12208;

	/* ToDo: use do_div */
	n_iio->kvolt = (((u64)9900 << 2)*vref_int*(10000+err1))/10000/vref_inp;
	n_iio->fvolt = ((u64)offs1 << 14) / 10;
	n_iio->kamp = (((u64)33000 << 1)*vref_int*(10000+err2))/10000/vref_inp;
	n_iio->famp = ((u64)offs2 << 13) / 10;
}

void unipi_iio_stm_read_vref_rev(struct unipi_iio_stm_device *n_iio)
{
	u32 vref_int = 0;
	u32 vref_inp = 0;
	int err1=0, err2=0, offs1=0, offs2=0;

	regmap_read(n_iio->map, UNIPI_MFD_REG_VREFINT, &vref_int);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_VREFINP, &vref_inp);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AOV_DEV, &err1);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AOV_OFS, &offs1);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AOA_DEV, &err2);
	regmap_read(n_iio->map, UNIPI_MFD_REG_BRAIN_AOA_OFS, &offs2);
	// safety check or set defaults
	vref_inp = vref_inp ? : 11328;
	vref_int = vref_inp ? : 12208;

	/* ToDo: use do_div */
	n_iio->kvolt = (((u64)vref_inp << 16) * 4095*10000) / vref_int / (9900 * (10000+err1));
	n_iio->fvolt = (((u64)vref_inp << 16) * 4095 * offs1) / vref_int / 99000;
	n_iio->kamp = (((u64)vref_inp << 16) * 4095*10000) / vref_int / (33000 * (10000+err1));
	n_iio->famp = (((u64)vref_inp << 16) * 4095 * offs1) / vref_int / 33000;
}

/*void unipi_iio_stm_ai_read_voltage(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *ai_data = iio_priv(iio_dev);
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
*/
/*
void unipi_iio_stm_ai_read_current(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *ai_data = iio_priv(indio_dev);
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
*/
/*
void unipi_iio_stm_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct unipi_iio_stm_device *n_iio = iio_priv(indio_dev);
	u32 stm_v_int_ref = 0;
	u32 stm_v_inp_ref = 0;
	u32 stm_v_err = 0;
	u32 stm_v_off = 0;
	u64 stm_true_ref = 0;
	u64 val64 = val;
	u64 val64_fraction = val2 / 100;

	val64 = ((val64 * 10000) + (val64_fraction) - n_iio->v_offset) * 4095;

	stm_true_ref = ((u64)stm_v_int_ref) * (99000 + stm_v_err) * 1000;
	stm_v_inp_ref = stm_v_inp_ref * 10000;
	do_div(stm_true_ref, stm_v_inp_ref);

	stm_v_inp_ref = stm_true_ref;
	do_div(val64, stm_v_inp_ref);
	do_div(val64, 10000);
	if (val64 > 4095) val64 = 4095;
	regmap_write(n_iio->map, n_iio->valreg, (unsigned int) val64);

	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_int, &stm_v_int_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->vref_inp, &stm_v_inp_ref);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_err, &stm_v_err);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_vol_off, &stm_v_off);

}

void unipi_iio_stm_ao_set_current(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct unipi_iio_device *ao_data = iio_priv(indio_dev);
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
*/
int unipi_iio_stm_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask) {

	struct unipi_iio_stm_device *n_iio = iio_priv(indio_dev);
	u32 stm_val = 0;

	regmap_read(n_iio->map, n_iio->modereg, &n_iio->mode);
	regmap_read(n_iio->map, n_iio->valreg, &stm_val);
	switch(n_iio->mode) {
	case 0: {
		if (ch->type == IIO_VOLTAGE) {
			*val = (stm_val * n_iio->kvolt + n_iio->fvolt) >> 14;
			return IIO_VAL_INT;
		}
		break;
	}
	case 1: {
		if (ch->type == IIO_CURRENT) {
			*val = (stm_val * n_iio->kamp + n_iio->famp) >> 13;
			*val2 = 1000;
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	}
	return -EINVAL;
}


int unipi_iio_stm_ao_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_stm_device *n_iio = iio_priv(indio_dev);
	u32 stm_val = 0;
	
	regmap_read(n_iio->map, n_iio->modereg, &n_iio->mode);
	switch(n_iio->mode) {
	case 3: {
		if (ch->type == IIO_RESISTANCE) {
			regmap_read(n_iio->map, n_iio->valreg, &stm_val);
			*val = stm_val;
			*val2 = 10;
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	}
	return -EINVAL;
}

int unipi_iio_stm_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	struct unipi_iio_stm_device *n_iio = iio_priv(indio_dev);
	u32 stm_val = 0;

	regmap_read(n_iio->map, n_iio->modereg, &n_iio->mode);
	switch(n_iio->mode) {
	case 0: {
		if (ch->type == IIO_VOLTAGE) {
			stm_val = (val * n_iio->kvolt + n_iio->fvolt) >> 16;
			if (stm_val > 4095) stm_val=4095;
			regmap_write(n_iio->map, n_iio->valreg, stm_val);
			return 0;
		}
		break;
	}
	case 1: {
		if (ch->type == IIO_CURRENT) {
			stm_val = (val * n_iio->kamp + n_iio->famp) >> 17;
			if (stm_val > 4095) stm_val=4095;
			regmap_write(n_iio->map, n_iio->valreg, stm_val);
			return 0;
		}
		break;
	}
	}
	return -EINVAL;
}

/*************************************
 *  sysfs 
 *
 ************************************/ 
static ssize_t mode_voltage_current_resistance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct unipi_iio_stm_device *n_iio = iio_priv(iio_dev);
	unsigned int val = 0;
	regmap_read(n_iio->map, n_iio->modereg, &val);
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t mode_voltage_current_resistance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct unipi_iio_stm_device *n_iio = iio_priv(iio_dev);
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		regmap_write(n_iio->map, n_iio->modereg, val);
	}
	return count;
}

DEVICE_ATTR_RW(mode_voltage_current_resistance);
static struct attribute *unipi_iio_stm_ao_attrs[] = {
		&dev_attr_mode_voltage_current_resistance.attr,
		NULL,
};
const struct attribute_group unipi_iio_stm_ao_group = {
	.attrs = unipi_iio_stm_ao_attrs,
};

static DEVICE_ATTR(mode_voltage_current, 0660, mode_voltage_current_resistance_show, mode_voltage_current_resistance_store);
static struct attribute *unipi_iio_stm_ai_attrs[] = {
		&dev_attr_mode_voltage_current.attr,
		NULL,
};
const struct attribute_group unipi_iio_stm_ai_group = {
	.attrs = unipi_iio_stm_ai_attrs,
};

/*****************************************************************
 * Probe data and functions 
 * 
 *****************************************************************/ 
static const struct iio_info unipi_iio_stm_ai_info = {
	.read_raw = unipi_iio_stm_ai_read_raw,
	.attrs = &unipi_iio_stm_ai_group,
};

static const struct iio_info unipi_iio_stm_ao_info = {
	.read_raw = unipi_iio_stm_ao_read_raw,
	.write_raw = unipi_iio_stm_ao_write_raw,
	.attrs = &unipi_iio_stm_ao_group,
};

int unipi_iio_stm_ai_register(struct device* dev, struct regmap *map, int group_index, int valreg, int modereg)
{
	struct iio_dev *iio_dev;
	struct unipi_iio_stm_device *n_iio;

	iio_dev = devm_iio_device_alloc(dev, sizeof(struct unipi_iio_stm_device));
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->currentmode = INDIO_DIRECT_MODE;
	iio_dev->name = "ai_type_legacy";
	iio_dev->dev.parent = dev;
	dev_set_name(&iio_dev->dev, "ai_%d_1", group_index);
	iio_dev->num_channels = ARRAY_SIZE(unipi_iio_stm_ai_chan_spec);
	iio_dev->channels = unipi_iio_stm_ai_chan_spec;
	iio_dev->info = &unipi_iio_stm_ai_info;
	n_iio = (struct unipi_iio_stm_device*) iio_priv(iio_dev);
	n_iio->map = map;
	n_iio->valreg = valreg;
	n_iio->modereg = modereg;
	unipi_iio_stm_read_vref(n_iio);
	devm_iio_device_register(dev, iio_dev);
	return 0;
}

int unipi_iio_stm_ao_register(struct device* dev, struct regmap *map, int group_index, int valreg, int modereg)
{
	struct iio_dev *iio_dev;
	struct unipi_iio_stm_device *n_iio;

	iio_dev = devm_iio_device_alloc(dev, sizeof(struct unipi_iio_stm_device));
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->currentmode = INDIO_DIRECT_MODE;
	iio_dev->name = "ao_type_legacy";
	iio_dev->dev.parent = dev;
	dev_set_name(&iio_dev->dev, "ao_%d_1", group_index);
	iio_dev->num_channels = ARRAY_SIZE(unipi_iio_stm_ao_chan_spec);
	iio_dev->channels = unipi_iio_stm_ao_chan_spec;
	iio_dev->info = &unipi_iio_stm_ao_info;
	n_iio = (struct unipi_iio_stm_device*) iio_priv(iio_dev);
	n_iio->map = map;
	n_iio->valreg = valreg;
	n_iio->modereg = modereg;
	unipi_iio_stm_read_vref_rev(n_iio);
	devm_iio_device_register(dev, iio_dev);
	return 0;
}

static int unipi_iio_stm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regmap* map;
	int ao_valreg = UNIPI_MFD_REG_BRAIN_AO_VAL;
	int ai_valreg = UNIPI_MFD_REG_BRAIN_AI_VAL;
	int ao_modereg = UNIPI_MFD_REG_BRAIN_AO_MODE;
	int ai_modereg = UNIPI_MFD_REG_BRAIN_AI_MODE;
	int ret;

	//map  = dev_get_regmap(parent, "registers");
	map = unipi_mfd_get_regmap(dev->parent, "registers");

	if (IS_ERR(map) || map == NULL) {
		dev_err(dev, "No regmap for Unipi device\n");
		return PTR_ERR(map);
	}

	ret = of_property_read_u32(np, "ai-value-reg", &ai_valreg);
	ret = of_property_read_u32(np, "ai-mode-reg", &ai_modereg);
	ret = of_property_read_u32(np, "ao-value-reg", &ao_valreg);
	ret = of_property_read_u32(np, "ao-mode-reg", &ao_modereg);

	unipi_iio_stm_ai_register(dev, map, 1, ai_valreg, ai_modereg);
	unipi_iio_stm_ao_register(dev, map, 1, ao_valreg, ao_modereg);
	return 0;
}

static const struct of_device_id of_unipi_iio_legacy_match[] = {
	{ .compatible = "unipi,aio_legacy"},
	{},
};
MODULE_DEVICE_TABLE(of, of_unipi_iio_legacy_match);

static struct platform_driver unipi_iio_legacy_driver = {
	.probe		= unipi_iio_stm_probe,
	.driver		= {
		.name	= "unipi-iio-legacy",
		.of_match_table = of_unipi_iio_legacy_match,
	},
};

module_platform_driver(unipi_iio_legacy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Legacy IIO Chip Driver");
