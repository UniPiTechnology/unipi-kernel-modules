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
#include <linux/string.h>
#include <linux/string_helpers.h>

#include "unipi_common.h"
#include "unipi_iogroup_bus.h"
#include "unipi_mfd.h"

#define AI_VAL_REG_COUNT  2
#define AI_MODE_REG_COUNT 1

#define AO_VAL_REG_COUNT  1
#define AO_MODE_REG_COUNT 1

const char* unipi_iio_mode_name[] = {
	"Disabled",
	"Voltage 10V",
	"Voltage 2.5V",
	"Current 20mA",
	"Resistance 3Wire 2kOhm",
	"Resistance 2Wire auto",
	"Temperature PT100",
	"Temperature PT1000",
};


static const struct iio_chan_spec unipi_iio_ai_chan_univ[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 0,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
			                    | BIT(IIO_CHAN_INFO_PROCESSED),
			.output = 0
	},
	{
			.type = IIO_CURRENT,
			.indexed = 0,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
			                    | BIT(IIO_CHAN_INFO_PROCESSED),
			.output = 0
	},
	{
			.type = IIO_RESISTANCE,
			.indexed = 0,
			.channel = 2,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
			                    | BIT(IIO_CHAN_INFO_PROCESSED),
			.output = 0
	}
};

static const struct iio_chan_spec unipi_iio_ai_chan_ui32[] = {
	{
			.type = IIO_VOLTAGE,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	},
	{
			.type = IIO_CURRENT,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct iio_chan_spec unipi_iio_r18_chan[] = {
	{
			.type = IIO_RESISTANCE,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct iio_chan_spec unipi_iio_ai_chan_resistance[] = {
	{
			.type = IIO_RESISTANCE,
			.indexed = 0,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	},
	{
			.type = IIO_TEMP,
			.indexed = 0,
			.channel = 1,
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


struct unipi_iio_descriptor
{
	int num_channels;
	int valsize;
	const struct iio_chan_spec *channels;
	const struct iio_info *info;
	const char* fname;
	int (*map_mode) (struct iio_dev *indio_dev);
};

struct unipi_iio_platform
{
	int io_count;
	struct regmap* map;
	const struct unipi_iio_descriptor *descriptor;
};

struct unipi_iio_device
{
	struct unipi_iio_platform *unipi_iio_platform;
//	struct regmap* map;
	int valreg;
	int modereg;
	u32 mode;
//	const struct unipi_iio_descriptor *descriptor;
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

int unipi_iio_map_mode_univ(struct iio_dev *indio_dev)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	switch(n_iio->mode) {
		case 1:
		case 2: return IIO_VOLTAGE;
		case 3: return IIO_CURRENT;
		case 4:
		case 5: return IIO_RESISTANCE;
	}
	return -1;
}

int unipi_iio_map_mode_uioff(struct iio_dev *indio_dev)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	switch(n_iio->mode) {
		case 1: return IIO_VOLTAGE;
		case 2: return IIO_CURRENT;
	}
	return -1;
}

int unipi_iio_map_mode_rtd(struct iio_dev *indio_dev)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	switch(n_iio->mode) {
		case 0:
		case 1: return IIO_TEMP;
	}
	return -1;
}

int unipi_iio_map_mode_ui(struct iio_dev *indio_dev)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	switch(n_iio->mode) {
		case 0: return IIO_VOLTAGE;
		case 1: return IIO_CURRENT;
	}
	return -1;
}

int unipi_iio_map_mode_r18(struct iio_dev *indio_dev)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	switch(n_iio->mode) {
		case 0: return IIO_RESISTANCE;
	}
	return -1;
}

int unipi_iio_map_mode_resistance(struct iio_dev *indio_dev)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	switch(n_iio->mode) {
		case 4:
		case 5: return IIO_RESISTANCE;
		case 6: return IIO_TEMP;
	}
	return -1;
}


int unipi_iio_ai_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	struct unipi_iio_platform *iio_platform = n_iio->unipi_iio_platform;
	u32 float_as_u32;
	int mode;

	regmap_read(iio_platform->map, n_iio->modereg, &n_iio->mode);
	mode = iio_platform->descriptor->map_mode(indio_dev);
	regmap_bulk_read(iio_platform->map, n_iio->valreg, &float_as_u32, AI_VAL_REG_COUNT);

	switch(mode) {
	case IIO_VOLTAGE: {
		if (ch->type == IIO_VOLTAGE) {
			float2int_with_divider(lo16(float_as_u32), hi16(float_as_u32), 1000, val, val2);
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	case IIO_CURRENT: {
		if (ch->type == IIO_CURRENT) {
			float2int_with_divider(lo16(float_as_u32), hi16(float_as_u32), 1, val, val2);
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	case IIO_RESISTANCE: {
		if (ch->type == IIO_RESISTANCE) {
			float2int_with_divider(lo16(float_as_u32), hi16(float_as_u32), 1, val, val2);
			return IIO_VAL_FRACTIONAL;
		}
		break;
	}
	}
	return -EINVAL;
}

int unipi_iio_ai_read_raw_u32(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	struct unipi_iio_platform *iio_platform = n_iio->unipi_iio_platform;
	u32 raw_value;
	int mode;

	regmap_read(iio_platform->map, n_iio->modereg, &n_iio->mode);
	mode = iio_platform->descriptor->map_mode(indio_dev);
	if (mode != ch->type)
		return -EINVAL;

	if (mask == IIO_CHAN_INFO_RAW) {
		regmap_bulk_read(iio_platform->map, n_iio->valreg, &raw_value, AI_VAL_REG_COUNT);
		*val = raw_value;
		return IIO_VAL_INT;
	}
	if (mask == IIO_CHAN_INFO_SCALE) {
		switch(mode) {
			case IIO_VOLTAGE: 
				*val = 10000; *val2 = 18;
				return  IIO_VAL_FRACTIONAL_LOG2;

			case IIO_CURRENT: 
				*val = 20000; *val2 = 18;
				return  IIO_VAL_FRACTIONAL_LOG2;

			case IIO_RESISTANCE: 
				*val = 1; *val2 = 1000;
				return  IIO_VAL_FRACTIONAL;
		}
	}
	return -EINVAL;
}

int unipi_iio_ao_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	struct unipi_iio_platform *iio_platform = n_iio->unipi_iio_platform;
	u32 sec_true_val;

	if (ch->type == IIO_VOLTAGE) {
		regmap_read(iio_platform->map, n_iio->valreg, &sec_true_val);
		*val = (sec_true_val * 5) / 2;
		if (*val > 10000) *val = 10000;
		return 0;
	}
	return -EINVAL;
}

int unipi_iio_ao_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int val, int val2, long mask)
{
	struct unipi_iio_device *n_iio = iio_priv(indio_dev);
	struct unipi_iio_platform *iio_platform = n_iio->unipi_iio_platform;
	u32 sec_true_val;

	if (ch->type == IIO_VOLTAGE) {
		if (val > 10000) val = 10000;
		sec_true_val = (val * 2) / 5;
		regmap_write(iio_platform->map, n_iio->valreg, sec_true_val);
		return 0;
	}
	return -EINVAL;
}

/*************************************
 *  sysfs 
 *
 ************************************/ 
static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct unipi_iio_device *n_iio = iio_priv(iio_dev);
	struct unipi_iio_platform *iio_platform = n_iio->unipi_iio_platform;
	unsigned int val = 0;
	regmap_read(iio_platform->map, n_iio->modereg, &val);
	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct unipi_iio_device *n_iio = iio_priv(iio_dev);
	struct unipi_iio_platform *iio_platform = n_iio->unipi_iio_platform;
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		regmap_write(iio_platform->map, n_iio->modereg, val);
	}
	return count;
}

DEVICE_ATTR_RW(mode);
static struct attribute *unipi_iio_ai_attrs[] = {
		&dev_attr_mode.attr,
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

static const struct iio_info unipi_iio_ai_info32 = {
	.read_raw = unipi_iio_ai_read_raw_u32,
	.attrs = &unipi_iio_ai_group,
};

static const struct iio_info unipi_iio_r18_info = {
	.read_raw = unipi_iio_ai_read_raw_u32,
};

static const struct iio_info unipi_iio_ao_info = {
	.read_raw = unipi_iio_ao_read_raw,
	.write_raw = unipi_iio_ao_write_raw,
	//.attrs = &neuron_sec_ao_group,
};

/*
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
*/

static int unipi_iio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(dev->parent);
	struct unipi_iio_platform *iio_platform;
	//struct regmap* map;
	//int io_count = 0;
	int io_modereg = -1, io_valreg;
	//char name[30];
	//const struct unipi_iio_descriptor *descriptor;
	struct iio_dev *iio_dev;
	struct unipi_iio_device *n_iio;
	int ret, i;

	iio_platform = devm_kzalloc(dev, sizeof(struct unipi_iio_platform), GFP_KERNEL);
	if (!iio_platform)
		return -ENOMEM;

	iio_platform->descriptor = of_device_get_match_data(dev);
	iio_platform->map = unipi_mfd_get_regmap(dev->parent, "registers");
	if (IS_ERR(iio_platform->map) || iio_platform->map == NULL) {
		devm_kfree(dev, iio_platform);
		dev_err(dev, "No regmap for Unipi device\n");
		return PTR_ERR(iio_platform->map);
	}

	of_property_read_u32(np, "io-count", &iio_platform->io_count);
	if (iio_platform->io_count <= 0) {
		devm_kfree(dev, iio_platform);
		dev_err(dev, "Unspecified io-count\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(np, "io-value-reg", &io_valreg);
	if (ret != 0) {
		devm_kfree(dev, iio_platform);
		dev_err(dev, "Invalid io-value-reg property in devicetree\n");
		return -EINVAL;
	}
	of_property_read_u32(np, "io-mode-reg", &io_modereg);
	platform_set_drvdata(pdev, iio_platform);

	for (i = 0; i < iio_platform->io_count; i++) {
		iio_dev = devm_iio_device_alloc(dev, sizeof(struct unipi_iio_device));
		iio_dev->modes = INDIO_DIRECT_MODE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,19,0)
		iio_dev->currentmode = INDIO_DIRECT_MODE;
#endif
		iio_dev->name = "aio_type_unipi";
		iio_dev->dev.parent = dev;
		dev_set_name(&iio_dev->dev, iio_platform->descriptor->fname, iogroup->address,  i + 1);
		iio_dev->num_channels = iio_platform->descriptor->num_channels;
		iio_dev->channels = iio_platform->descriptor->channels;
		iio_dev->info = iio_platform->descriptor->info;
		n_iio = (struct unipi_iio_device*) iio_priv(iio_dev);
		n_iio->unipi_iio_platform = iio_platform;
		n_iio->valreg = io_valreg + (i * iio_platform->descriptor->valsize);
		n_iio->modereg = (io_modereg == -1) ? -1 : io_modereg + i;
		devm_iio_device_register(dev, iio_dev);
	}
/*
	for (i=0; i < iio_platform->io_count; i++) {
		snprintf(name, sizeof(name), iio_platform->descriptor->fname, iogroup->address, i+1);
		string_upper(name, name);
		unipi_mfd_add_group(dev->parent, name, unipi_mfd_ai_attrs, 2,
		                (u32) (io_modereg == -1) ? -1 : io_modereg + i,
		                (u32) io_valreg + (i * iio_platform->descriptor->valsize));
	}
*/
	return 0;
}

int unipi_iio_remove(struct platform_device *pdev)
{
/*
	int i;
	struct unipi_iio_platform *iio_platform = (struct unipi_iio_platform*) platform_get_drvdata(pdev);
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(pdev->dev.parent);
	char name[30];

	if (!iio_platform || !iio_platform->descriptor)
		return 0;

	for (i=0; i < iio_platform->io_count; i++) {
		snprintf(name, sizeof(name), iio_platform->descriptor->fname, iogroup->address, i+1);
		string_upper(name, name);
		unipi_mfd_remove_group(pdev->dev.parent, name);
	}
*/
	return 0;
}

static const struct unipi_iio_descriptor unipi_iio_descriptor_univ =
{ .num_channels = 3,
  .valsize = 2,
  .channels = unipi_iio_ai_chan_univ,
  .info = &unipi_iio_ai_info,
  .map_mode = unipi_iio_map_mode_univ,
  .fname = "AIi%d.%d",
};

static const struct unipi_iio_descriptor unipi_iio_descriptor_ui12 =
{ .num_channels = 2,
  .valsize = 2,
  .channels = unipi_iio_ai_chan_univ,
  .info = &unipi_iio_ai_info,
  .map_mode = unipi_iio_map_mode_uioff,
  .fname = "AI%d.%d",
};

static const struct unipi_iio_descriptor unipi_iio_descriptor_rtd =
{ .num_channels = 2,
  .valsize = 2,
  .channels = unipi_iio_ai_chan_univ,
  .info = &unipi_iio_ai_info,
  .map_mode = unipi_iio_map_mode_rtd,
  .fname = "AI%d.%d",
};

static const struct unipi_iio_descriptor unipi_iio_descriptor_ui18 =
{ .num_channels = 2,
  .valsize = 2,
  .channels = unipi_iio_ai_chan_ui32,
  .info = &unipi_iio_ai_info32,
  .map_mode = unipi_iio_map_mode_ui,
  .fname = "AI%d.%d",
};

static const struct unipi_iio_descriptor unipi_iio_descriptor_r18 =
{ .num_channels = 1,
  .valsize = 2,
  .channels = unipi_iio_r18_chan,
  .info = &unipi_iio_r18_info,
  .map_mode = unipi_iio_map_mode_r18,
  .fname = "AI%d.%d",
};

static const struct unipi_iio_descriptor unipi_iio_descriptor_ao =
{ .num_channels = 1,
  .valsize = 1,
  .channels = unipi_iio_ao_chan_spec,
  .info = &unipi_iio_ao_info,
  .map_mode = unipi_iio_map_mode_ui,
  .fname = "AO%d.%d",
};
/*
static const struct unipi_iio_descriptor unipi_iio_descriptor_ui =
{ .num_channels = 2,
  .channels = unipi_iio_ai_chan_spec,
  .map_mode = unipi_iio_map_mode_ui,
};
static const struct unipi_iio_descriptor unipi_iio_descriptor_resistance =
{ .num_channels = 2,
  .channels = unipi_iio_ai_chan_spec_resistance,
  .map_mode = unipi_iio_map_mode_resistance,
};
*/

/*
 - unipi,ai12:  U/I, float+int32 value, ai12 on x51 (used id-aiuc8-1, ic-ai2ao1)
                mode: 0(off), 1(10V), 2(20mA)
 - unipi,airtd: Temp/R, float(+/) int16 value, (used id_airtd8)
                mode: 0(PT100), 1(PT1000)
 - unipi,air18: only R, int32 value, MAX11410(used ac-heating)
 - unipi,ai18:  U/I, int32, ADS8698(id-aiuc8-2)
 - unipi,ao:    ao on x51 or max5715(ic-ai2ao1-1, ac-heating)
                mode: 0(10V), 1(20mA)
*/

static const struct of_device_id of_unipi_iio_match[] = {
	{ .compatible = "unipi,ai",   .data = &unipi_iio_descriptor_univ},
	{ .compatible = "unipi,ai12", .data = &unipi_iio_descriptor_ui12},
	{ .compatible = "unipi,airtd",.data = &unipi_iio_descriptor_rtd},
	{ .compatible = "unipi,ai18", .data = &unipi_iio_descriptor_ui18},
	{ .compatible = "unipi,air18",.data = &unipi_iio_descriptor_r18},
	{ .compatible = "unipi,ao",   .data = &unipi_iio_descriptor_ao},
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
