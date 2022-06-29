// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Unipi GPIO Driver
 *
 * Copyright (c) 2021, Unipi Technology
 * Author: Miroslav Ondra <ondra@faster.cz>
 */

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "unipi_common.h"
#include "unipi_iogroup_bus.h"
#include "unipi_mfd.h"

#define UNIPI_GPIO_IN	BIT(0)
#define UNIPI_GPIO_OUT	BIT(1)


struct unipi_gpio_data {
	char*			regmap_name;
	int				is_coil_map;
	char*			dt_value_reg_name;
	unsigned int	flags;  /* UNIPI_GPIO_IN | UNIPI_GPIO_OUT */
	int				to_reg_shift;
	int				to_bit_mask;
	char			*fname;
};

struct unipi_gpio_device {
	struct gpio_chip		chip;
	struct regmap*			map;
	const struct unipi_gpio_data	*data;
	u32						ngpio;
	u32						value_reg;
	int						debounce_reg;
	int						counter_reg;
};

static int unipi_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct unipi_gpio_device *priv = gpiochip_get_data(chip);
	unsigned int val, offs;
	int ret;

	offs = priv->value_reg + (offset >> priv->data->to_reg_shift);
	//printk("gpio get %d, %d", offset, offs);

	ret = regmap_read(priv->map, offs, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset & priv->data->to_bit_mask));
}

static void unipi_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	/* using coils to update */
	struct unipi_gpio_device *priv = gpiochip_get_data(chip);
	//printk("gpio set %d, %d", offset, val);

	regmap_write(priv->map, priv->value_reg + offset, !!val);
}

/* ToDo
static void unipi_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits)
{

}

static int unipi_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bit)
{
	return 0; // OK
}
*/

static int unipi_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	/* 0=out, 1=in */
	struct unipi_gpio_device *priv = gpiochip_get_data(chip);
	return (priv->data->flags & UNIPI_GPIO_OUT)? 0 : 1;
}

static const struct unipi_gpio_data unipi_gpio_data_di = {
	.regmap_name = "registers",
	.is_coil_map = 0,
	.dt_value_reg_name = "value-reg",
	.flags		= UNIPI_GPIO_IN,
	.to_reg_shift = 4,
	.to_bit_mask = 0xf,
	.fname = "DI%d.%d",
};

static const struct unipi_gpio_data unipi_gpio_data_do = {
	.regmap_name = "coils",
	.is_coil_map = 1,
	.dt_value_reg_name = "value-coil",
	.flags		= UNIPI_GPIO_OUT,
	.to_reg_shift = 0,
	.to_bit_mask = 0,
	.fname = "DO%d.%d",
};

static const struct unipi_gpio_data unipi_gpio_data_ro = {
	.regmap_name = "coils",
	.is_coil_map = 1,
	.dt_value_reg_name = "value-coil",
	.flags		= UNIPI_GPIO_OUT,
	.to_reg_shift = 0,
	.to_bit_mask = 0,
	.fname = "RO%d.%d",
};


static struct dev_mfd_attribute dev_attr_di_debounce = {
	__ATTR(debounce, 0664, unipi_mfd_show_reg, unipi_mfd_store_reg),
	0
};

static struct dev_mfd_attribute dev_attr_di_counter = {
	__ATTR(counter, 0444, unipi_mfd_show_int, NULL),
	0
};

static struct dev_mfd_attribute dev_attr_di_value = {
	__ATTR(value, 0444, unipi_mfd_show_regbool, NULL),
	0
};

static struct attribute *unipi_mfd_di_attrs[] = {
	&dev_attr_di_debounce.attr.attr,
	&dev_attr_di_counter.attr.attr,
	&dev_attr_di_value.attr.attr,
};

static struct dev_mfd_attribute dev_attr_do_value = {
	__ATTR(value, 0664, unipi_mfd_show_bool, unipi_mfd_store_bool),
	0
};

static struct attribute *unipi_mfd_do_attrs[] = {
	&dev_attr_do_value.attr.attr,
};


static int unipi_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct unipi_gpio_device *priv;
	struct device_node *np = dev->of_node;
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(dev->parent);
	char name[30];
	int ret, i, regaddr;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->data = of_device_get_match_data(dev);
	priv->debounce_reg = priv->counter_reg = -1;
	priv->map = unipi_mfd_get_regmap(dev->parent, priv->data->regmap_name);
	if (IS_ERR(priv->map) || priv->map == NULL) {
		devm_kfree(dev, priv);
		dev_err(dev, "No %s regmap for Unipi device\n", priv->data->regmap_name);
		return PTR_ERR(priv->map);
	}

	ret = of_property_read_u32(np, "ngpio", &priv->ngpio);
	if (ret) {
		devm_kfree(dev, priv);
		dev_err(dev, "Invalid ngpio property in devicetree\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(np, priv->data->dt_value_reg_name, &priv->value_reg);
	if (ret) {
		devm_kfree(dev, priv);
		dev_err(dev, "Invalid %s property in devicetree\n", priv->data->dt_value_reg_name);
		return -EINVAL;
	}

	priv->chip.parent = dev;
	priv->chip.owner = THIS_MODULE;
	priv->chip.label = dev_name(dev);
	priv->chip.base = -1;
	priv->chip.ngpio = priv->ngpio;
	priv->chip.get = unipi_gpio_get;
	priv->chip.get_direction = unipi_gpio_get_direction;
	/* ToDo
	priv->chip.get_multiple = unipi_gpio_get_multiple;
	priv->chip.set_multiple = unipi_gpio_set_multiple;
	*/
	if (priv->data->flags & UNIPI_GPIO_OUT) {
		priv->chip.set = unipi_gpio_set;
	}

	platform_set_drvdata(pdev, priv);
	
	ret = of_property_read_u32(np, "debounce-reg", &priv->debounce_reg);
	ret = of_property_read_u32(np, "counter-reg", &priv->counter_reg);

	for (i=0; i<priv->ngpio; i++) {
		snprintf(name, sizeof(name), priv->data->fname, iogroup->address, i+1);
		if (priv->data->flags & UNIPI_GPIO_IN) {
			//if (priv->counter_reg>=0)
			regaddr = priv->value_reg+(i/16);
			regaddr |= (i%16) << 16;
			unipi_mfd_add_group(dev->parent, name, unipi_mfd_di_attrs, 3,
			            (u32) priv->debounce_reg+i,
			            (u32) priv->counter_reg+2*i,
			            (u32) regaddr);
		} else {
			unipi_mfd_add_group(dev->parent, name, unipi_mfd_do_attrs, 1,
			            (u32) priv->value_reg+i);
		}
	}

	return devm_gpiochip_add_data(&pdev->dev, &priv->chip, priv);
}

int unipi_gpio_remove(struct platform_device *pdev)
{
	int i;
	struct unipi_gpio_device *priv = (struct unipi_gpio_device*) platform_get_drvdata(pdev);
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(pdev->dev.parent);
	char name[30];

	if (!priv || !priv->data)
		return 0;
	for (i=0; i<priv->ngpio; i++) {
		snprintf(name, sizeof(name), priv->data->fname, iogroup->address, i+1);
		unipi_mfd_remove_group(pdev->dev.parent, name);
	}

	return 0;
}

static const struct of_device_id of_unipi_gpio_match[] = {
	{ .compatible = "unipi,gpio-di", .data = &unipi_gpio_data_di },
	{ .compatible = "unipi,gpio-do", .data = &unipi_gpio_data_do },
	{ .compatible = "unipi,gpio-ro", .data = &unipi_gpio_data_ro },
	{},
};
MODULE_DEVICE_TABLE(of, of_unipi_gpio_match);

static struct platform_driver unipi_gpio_driver = {
	.probe		= unipi_gpio_probe,
	.remove		= unipi_gpio_remove,
	.driver		= {
		.name	= "unipi-gpio",
		.of_match_table = of_unipi_gpio_match,
	},
};

module_platform_driver(unipi_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi GPIO Driver");
