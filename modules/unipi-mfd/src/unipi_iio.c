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

#define AI_VAL_REG_COUNT  2
#define AI_MODE_REG_COUNT 1

#define AO_VAL_REG_COUNT  1
#define AO_MODE_REG_COUNT 1

static const struct iio_chan_spec unipi_iio_ai_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 0,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), 
			                 /* | BIT(IIO_CHAN_INFO_PROCESSED), */
			.output = 0
	},
	{
			.type = IIO_CURRENT,
			.indexed = 0,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	},
	{
			.type = IIO_RESISTANCE,
			.indexed = 0,
			.channel = 2,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	}
};

static const struct iio_chan_spec unipi_iio_ao_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 0,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 1
	}
};

struct unipi_iio_device
{
	struct regmap* map;
	int valreg;
	int modereg;
	u32 mode;
};


/************************
 * Non-static Functions *
 ************************/

#define lo16(x)  (x & 0xffff)
#define hi16(x)  ((x>>16) & 0xffff)
/*
 * NOTE: This function uses 64-bit fixed-point arithmetic,
 * which necessitates using the do_div macro to avoid unnecessary long/long division.
 */

/*
 * Convert float32 (low_val:hight_val) to integer value / divider. 
 *   optionaly multiply value by factor
*/
void float2int_with_divider(u16 low_val, u16 high_val, int factor, int* value, int* divider)
{
        int exponent;
        //int i, mask;
        int preshift = 0;
        s64 tmp;
        // exponent can be [-127 .. 128], exp=0 means value in (1 .. 1.999999)
        // mantisa has 24 bit
        
        // calc log2(factor)
        if (factor > 1) {
            for (preshift = 30; preshift > 0; preshift--) {
                if (factor & (1 << preshift)) {
                    if (factor != (1 << preshift)) preshift++;
                    break;
                }
            }
        }
        exponent = (int)((high_val >> 7) & 0xff) - 127 + preshift;
        if (exponent > 23) { 
            // value > 16M
            *value = 1 << 24;
            *divider = 1;
        } else if (exponent < -23) {
            // value is almost zero, set value to 0
            *value = 0;
            *divider = 1;
        } else {
            *value = (1 << 23) | ((high_val & 0x7f) << 16) | low_val;
            if ((23 - exponent) < 30) {
                // shift divider into right position
                *divider = 1 << (23 - exponent);
            } else {
                // set max divider and shift value
                *divider = 1 << 30;
                *value = *value >> ((23 - exponent) - 30);
            }
            if (preshift) {
                tmp = *value;
                *value = (tmp * factor) >> preshift;
            }
        }
        // check and set sign of value
        if (high_val & 0x8000) *value = -(*value);
}


int unipi_iio_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	u32 float_as_u32;

	regmap_read(n_iio->map, n_iio->modereg, &n_iio->mode);
	regmap_bulk_read(n_iio->map, n_iio->valreg, &float_as_u32, AI_VAL_REG_COUNT);

	switch(n_iio->mode) {
	case 1:
	case 2: {
		if (ch->type == IIO_VOLTAGE) {
			float2int_with_divider(lo16(float_as_u32), hi16(float_as_u32), 1000, val, val2);
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	case 3: {
		if (ch->type == IIO_CURRENT) {
			float2int_with_divider(lo16(float_as_u32), hi16(float_as_u32), 1, val, val2);
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	case 4:
	case 5: {
		if (ch->type == IIO_RESISTANCE) {
			float2int_with_divider(lo16(float_as_u32), hi16(float_as_u32), 1, val, val2);
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	}
	return -EINVAL;
}

int unipi_iio_ao_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	u32 sec_true_val;

	if (ch->type == IIO_VOLTAGE) {
		regmap_read(n_iio->map, n_iio->valreg, &sec_true_val);
		*val = (sec_true_val * 5) / 2;
		if (*val > 10000) *val = 10000;
		return 0;
	}
	return -EINVAL;
}

int unipi_iio_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	u32 sec_true_val;

	if (ch->type == IIO_VOLTAGE) {
		if (val > 10000) val = 10000;
		sec_true_val = (val * 2) / 5;
		regmap_write(n_iio->map, n_iio->valreg, sec_true_val);
		return 0;
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
	struct unipi_iio_device *n_iio = iio_priv(iio_dev);
	unsigned int val = 0;
	regmap_read(n_iio->map, n_iio->modereg, &val);
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t mode_voltage_current_resistance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct unipi_iio_device *n_iio = iio_priv(iio_dev);
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		regmap_write(n_iio->map, n_iio->modereg, val);
	}
	return count;
}

DEVICE_ATTR_RW(mode_voltage_current_resistance);
static struct attribute *unipi_iio_ai_attrs[] = {
		&dev_attr_mode_voltage_current_resistance.attr,
		NULL,
};

const struct attribute_group unipi_iio_ai_group = {
	.attrs = unipi_iio_ai_attrs,
};


/*****************************************************************
 * Probe data and functions 
 * 
 *****************************************************************/ 
static const struct iio_info unipi_iio_ai_info = {
	.read_raw = unipi_iio_ai_read_raw,
	.attrs = &unipi_iio_ai_group,
};

static const struct iio_info unipi_iio_ao_info = {
	.read_raw = unipi_iio_ao_read_raw,
	.write_raw = unipi_iio_ao_write_raw,
	//.attrs = &neuron_sec_ao_group,
};

int unipi_iio_ai_register(struct device* dev, struct regmap *map, int group_index, int ai_count, int valreg, int modereg)
{
	struct iio_dev *iio_dev;
	struct unipi_iio_device *n_iio;
	int i;

	for (i = 0; i < ai_count; i++) {
		iio_dev = devm_iio_device_alloc(dev, sizeof(struct unipi_iio_device));
		iio_dev->modes = INDIO_DIRECT_MODE;
		iio_dev->currentmode = INDIO_DIRECT_MODE;
		iio_dev->name = "ai_type_unipi";
		iio_dev->dev.parent = dev;
		dev_set_name(&iio_dev->dev, "ai_%d_%d", group_index,  i + 1);
		iio_dev->num_channels = ARRAY_SIZE(unipi_iio_ai_chan_spec);
		iio_dev->channels = unipi_iio_ai_chan_spec;
		iio_dev->info = &unipi_iio_ai_info;
		n_iio = (struct unipi_iio_device*) iio_priv(iio_dev);
		n_iio->map = map;
		n_iio->valreg = valreg + (i * AI_VAL_REG_COUNT);
		n_iio->modereg = modereg + (i * AI_MODE_REG_COUNT);
		devm_iio_device_register(dev, iio_dev);
	}
	return 0;
}

int unipi_iio_ao_register(struct device* dev, struct regmap *map, int group_index, int ao_count, int valreg, int modereg)
{
	struct iio_dev *iio_dev;
	struct unipi_iio_device *n_iio;
	int i;

	for (i = 0; i < ao_count; i++) {
		iio_dev = devm_iio_device_alloc(dev, sizeof(struct unipi_iio_device));
		iio_dev->modes = INDIO_DIRECT_MODE;
		iio_dev->currentmode = INDIO_DIRECT_MODE;
		iio_dev->name = "ao_type_unipi";
		iio_dev->dev.parent = dev;
		dev_set_name(&iio_dev->dev, "ao_%d_%d", group_index,  i + 1);
		iio_dev->num_channels = ARRAY_SIZE(unipi_iio_ao_chan_spec);
		iio_dev->channels = unipi_iio_ao_chan_spec;
		iio_dev->info = &unipi_iio_ao_info;
		n_iio = (struct unipi_iio_device*) iio_priv(iio_dev);
		n_iio->map = map;
		n_iio->valreg = valreg + (i * AO_VAL_REG_COUNT);
		n_iio->modereg = modereg + (i * AO_MODE_REG_COUNT);
		devm_iio_device_register(dev, iio_dev);
	}
	return 0;
}


static struct dev_mfd_attribute dev_attr_ai_mode = {
	__ATTR(mode, 0664, unipi_mfd_show_reg, unipi_mfd_store_reg),
	0
};

static struct dev_mfd_attribute dev_attr_ai_value = {
	__ATTR(value, 0444, unipi_mfd_show_int, NULL),
	0
};


static struct attribute *unipi_mfd_ai_attrs[] = {
	&dev_attr_ai_mode.attr.attr,
	&dev_attr_ai_value.attr.attr,
};


static int unipi_iio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regmap* map;
	int ai_count = 0;
	int ao_count = 0;
	int ao_modereg, ai_modereg, ai_valreg, ao_valreg;
	char name[30];
	int ret, i;

	map = unipi_mfd_get_regmap(dev->parent, "registers");
	//map  = dev_get_regmap(parent, "registers");
	if (IS_ERR(map) || map == NULL) {
		dev_err(dev, "No regmap for Unipi device\n");
		return PTR_ERR(map);
	}

	of_property_read_u32(np, "ai-count", &ai_count);
	if (ai_count > 0) {
		ret = of_property_read_u32(np, "ai-value-reg", &ai_valreg);
		if (ret) {
			dev_err(dev, "Invalid ai-value-reg property in devicetree\n");
			return -EINVAL;
		}
		ret = of_property_read_u32(np, "ai-mode-reg", &ai_modereg);
		if (ret) {
			dev_err(dev, "Invalid ai-mode-reg property in devicetree\n");
			return -EINVAL;
		}
	}
	of_property_read_u32(np, "ao-count", &ao_count);
	if (ao_count > 0) {
		ret = of_property_read_u32(np, "ao-value-reg", &ao_valreg);
		if (ret) {
			dev_err(dev, "Invalid ao-value-reg property in devicetree\n");
			return -EINVAL;
		}
		ret = of_property_read_u32(np, "ao-mode-reg", &ao_modereg);
		if (ret) {
			dev_err(dev, "Invalid ao-mode-reg property in devicetree\n");
			return -EINVAL;
		}
	}

	if (ai_count+ao_count == 0) {
		dev_err(dev, "Unspecified ai-count nor ao-count\n");
		return -EINVAL;
	}
	unipi_iio_ai_register(dev, map, 1, ai_count, ai_valreg, ai_modereg);
	unipi_iio_ao_register(dev, map, 1, ao_count, ao_valreg, ao_modereg);
	for (i=0; i<ai_count; i++) {
		snprintf(name, sizeof(name), "AI1.%d", i+1);
		unipi_mfd_add_group(dev->parent, name, unipi_mfd_ai_attrs, 2,
			            (u32) ai_modereg+i, (u32) ai_valreg + 2*i);
	}

	return 0;
}

static const struct of_device_id of_unipi_iio_match[] = {
	{ .compatible = "unipi,aio" },
/*	{ .compatible = "unipi,aio_type_b", .data = &unipi_gpio_data_di }, */
	{},
};
MODULE_DEVICE_TABLE(of, of_unipi_iio_match);

static struct platform_driver unipi_iio_driver = {
	.probe		= unipi_iio_probe,
	.driver		= {
		.name	= "unipi-iio",
		.of_match_table = of_unipi_iio_match,
	},
};

module_platform_driver(unipi_iio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi IIO Chip Driver");
