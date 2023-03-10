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


struct unipi_gpiochip_data {
	char*			regmap_name;
	int				is_coil_map;
	char*			dt_value_reg_name;
	unsigned int	flags;  /* UNIPI_GPIO_IN | UNIPI_GPIO_OUT */
	int				to_reg_shift;
	int				to_bit_mask;
	char			*fname;
};

struct unipi_gpiochip_device {
	struct gpio_chip		chip;
	struct regmap*			map;
	const struct unipi_gpiochip_data	*data;
	u32						ngpio;
	u32						value_reg;
	int						debounce_reg;
	int						counter_reg;
	int						ds_count;
	int						ds_enable_coil;
	int						ds_toggle_coil;
	int						ds_inv_coil;
	struct unipi_gpio_sysfs_kobj *kobj;
};

static int unipi_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct unipi_gpiochip_device *unipi_gpiochip = gpiochip_get_data(chip);
	unsigned int val, offs;
	int ret;

	offs = unipi_gpiochip->value_reg + (offset >> unipi_gpiochip->data->to_reg_shift);
	//printk("gpio get %d, %d", offset, offs);

	ret = regmap_read(unipi_gpiochip->map, offs, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset & unipi_gpiochip->data->to_bit_mask));
}

static void unipi_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	/* using coils to update */
	struct unipi_gpiochip_device *unipi_gpiochip = gpiochip_get_data(chip);
	//printk("gpio set %d, %d", offset, val);

	regmap_write(unipi_gpiochip->map, unipi_gpiochip->value_reg + offset, !!val);
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
	struct unipi_gpiochip_device *unipi_gpiochip = gpiochip_get_data(chip);
	return (unipi_gpiochip->data->flags & UNIPI_GPIO_OUT)? 0 : 1;
}

static const struct unipi_gpiochip_data unipi_gpiochip_data_di = {
	.regmap_name = "registers",
	.is_coil_map = 0,
	.dt_value_reg_name = "value-reg",
	.flags		= UNIPI_GPIO_IN,
	.to_reg_shift = 4,
	.to_bit_mask = 0xf,
	.fname = "DI%d.%d",
};

static const struct unipi_gpiochip_data unipi_gpiochip_data_do = {
	.regmap_name = "coils",
	.is_coil_map = 1,
	.dt_value_reg_name = "value-coil",
	.flags		= UNIPI_GPIO_OUT,
	.to_reg_shift = 0,
	.to_bit_mask = 0,
	.fname = "DO%d.%d",
};

static const struct unipi_gpiochip_data unipi_gpiochip_data_ro = {
	.regmap_name = "coils",
	.is_coil_map = 1,
	.dt_value_reg_name = "value-coil",
	.flags		= UNIPI_GPIO_OUT,
	.to_reg_shift = 0,
	.to_bit_mask = 0,
	.fname = "RO%d.%d",
};

struct unipi_gpio_sysfs_kobj {
	struct kobject kobject;
	struct unipi_gpiochip_device *gpio_dev;
	int    index;
	struct unipi_gpio_sysfs_kobj *next;
};
#define to_unipi_gpio_sysfs_obj(x) container_of(x, struct unipi_gpio_sysfs_kobj, kobject)


static void unipi_gpio_release(struct kobject *kobj)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj;

	gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	kfree(gpio_kobj);
}

static ssize_t di_debounce_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->debounce_reg + gpio_kobj->index;
	ret = regmap_read(unipi_gpiochip->map, reg, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", val);
}

static ssize_t di_debounce_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->debounce_reg + gpio_kobj->index;
	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;
	regmap_write(unipi_gpiochip->map, reg, val);
	return count;
}

static ssize_t di_counter_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->counter_reg + 2*gpio_kobj->index;

	ret = regmap_bulk_read(unipi_gpiochip->map, reg, &val, 2);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", val);
}

static ssize_t di_counter_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->counter_reg + 2*gpio_kobj->index;
	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;
    regmap_bulk_write(unipi_gpiochip->map, reg, &val, 2);
	return count;
}

static ssize_t di_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, offs;
	int ret;

	offs = unipi_gpiochip->value_reg + (gpio_kobj->index >> unipi_gpiochip->data->to_reg_shift);
	ret = regmap_read(unipi_gpiochip->map, offs, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", !!(val & BIT(gpio_kobj->index & unipi_gpiochip->data->to_bit_mask)));
}

static ssize_t do_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->value_reg + gpio_kobj->index;
	ret = regmap_read(unipi_gpiochip->map, reg, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", !!(val & 1));
}

static ssize_t do_value_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;
	reg = unipi_gpiochip->value_reg + gpio_kobj->index;
	regmap_write(unipi_gpiochip->map, reg, !!val);
	return count;
}

static ssize_t ds_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->ds_enable_coil + gpio_kobj->index;
	ret = regmap_read(unipi_gpiochip->map, reg, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", !!(val & 1));
}

static ssize_t ds_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;
	reg = unipi_gpiochip->ds_enable_coil + gpio_kobj->index;
	regmap_write(unipi_gpiochip->map, reg, !!val);
	return count;
}

static ssize_t ds_toggle_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->ds_toggle_coil + gpio_kobj->index;
	ret = regmap_read(unipi_gpiochip->map, reg, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", !!(val & 1));
}

static ssize_t ds_toggle_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;
	reg = unipi_gpiochip->ds_toggle_coil + gpio_kobj->index;
	regmap_write(unipi_gpiochip->map, reg, !!val);
	return count;
}

static ssize_t ds_inv_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	reg = unipi_gpiochip->ds_inv_coil + gpio_kobj->index;
	ret = regmap_read(unipi_gpiochip->map, reg, &val);
	if (ret)
		return ret;
	return sprintf(buf, "%d\n", !!(val & 1));
}

static ssize_t ds_inv_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	struct unipi_gpio_sysfs_kobj *gpio_kobj = to_unipi_gpio_sysfs_obj(kobj);
	struct unipi_gpiochip_device *unipi_gpiochip = gpio_kobj->gpio_dev;
	unsigned int val, reg;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;
	reg = unipi_gpiochip->ds_inv_coil + gpio_kobj->index;
	regmap_write(unipi_gpiochip->map, reg, !!val);
	return count;
}

static struct kobj_attribute gpio_attr_di_debounce = __ATTR(debounce, 0664, di_debounce_show, di_debounce_store);
static struct kobj_attribute gpio_attr_di_counter = __ATTR(counter, 0664, di_counter_show, di_counter_store);
static struct kobj_attribute gpio_attr_di_value = __ATTR(value, 0444, di_value_show, NULL);
static struct kobj_attribute gpio_attr_do_value = __ATTR(value, 0664, do_value_show, do_value_store);
static struct kobj_attribute gpio_attr_ds_enable = __ATTR(ds-enable, 0664, ds_enable_show, ds_enable_store);
static struct kobj_attribute gpio_attr_ds_toggle = __ATTR(ds-toggle, 0664, ds_toggle_show, ds_toggle_store);
static struct kobj_attribute gpio_attr_ds_inv = __ATTR(ds-inv, 0664, ds_inv_show, ds_inv_store);

static struct attribute *unipi_gpio_di_attrs[] = {
	&gpio_attr_di_debounce.attr,
	&gpio_attr_di_counter.attr,
	&gpio_attr_di_value.attr,
	NULL,
};
ATTRIBUTE_GROUPS(unipi_gpio_di);

static struct kobj_type unipi_gpio_di_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.release = unipi_gpio_release,
	.default_groups = unipi_gpio_di_groups,
};

static struct attribute *unipi_gpio_dods_attrs[] = {
	&gpio_attr_do_value.attr,
	&gpio_attr_ds_enable.attr,
	&gpio_attr_ds_toggle.attr,
	&gpio_attr_ds_inv.attr,
	NULL,
};
ATTRIBUTE_GROUPS(unipi_gpio_dods);

static struct kobj_type unipi_gpio_dods_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.release = unipi_gpio_release,
	.default_groups = unipi_gpio_dods_groups,
};

static struct attribute *unipi_gpio_do_attrs[] = {
	&gpio_attr_do_value.attr,
	NULL,
};
ATTRIBUTE_GROUPS(unipi_gpio_do);

static struct kobj_type unipi_gpio_do_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.release = unipi_gpio_release,
	.default_groups = unipi_gpio_do_groups,
};


static struct kset *unipi_gpio_kset;

static int unipi_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct unipi_gpiochip_device *unipi_gpiochip;
	struct device_node *np = dev->of_node;
	struct unipi_iogroup_device *iogroup = to_unipi_iogroup_device(dev->parent);
//	char name[30];
	int ret, i;

	unipi_gpiochip = devm_kzalloc(dev, sizeof(*unipi_gpiochip), GFP_KERNEL);
	if (!unipi_gpiochip)
		return -ENOMEM;

	unipi_gpiochip->data = of_device_get_match_data(dev);
	unipi_gpiochip->debounce_reg = unipi_gpiochip->counter_reg = -1;
	unipi_gpiochip->ds_count = 0;
	unipi_gpiochip->ds_enable_coil = unipi_gpiochip->ds_toggle_coil = unipi_gpiochip->ds_inv_coil = -1;
	unipi_gpiochip->map = unipi_mfd_get_regmap(dev->parent, unipi_gpiochip->data->regmap_name);
	if (IS_ERR(unipi_gpiochip->map) || unipi_gpiochip->map == NULL) {
		devm_kfree(dev, unipi_gpiochip);
		dev_err(dev, "No %s regmap for Unipi device\n", unipi_gpiochip->data->regmap_name);
		return PTR_ERR(unipi_gpiochip->map);
	}

	ret = of_property_read_u32(np, "ngpio", &unipi_gpiochip->ngpio);
	if (ret) {
		devm_kfree(dev, unipi_gpiochip);
		dev_err(dev, "Invalid ngpio property in devicetree\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(np, unipi_gpiochip->data->dt_value_reg_name, &unipi_gpiochip->value_reg);
	if (ret) {
		devm_kfree(dev, unipi_gpiochip);
		dev_err(dev, "Invalid %s property in devicetree\n", unipi_gpiochip->data->dt_value_reg_name);
		return -EINVAL;
	}

	unipi_gpiochip->chip.parent = dev;
	unipi_gpiochip->chip.owner = THIS_MODULE;
	unipi_gpiochip->chip.label = dev_name(dev);
	unipi_gpiochip->chip.base = -1;
	unipi_gpiochip->chip.ngpio = unipi_gpiochip->ngpio;
	unipi_gpiochip->chip.get = unipi_gpio_get;
	unipi_gpiochip->chip.get_direction = unipi_gpio_get_direction;
	unipi_gpiochip->chip.can_sleep = true;
	/* ToDo
	unipi_gpiochip->chip.get_multiple = unipi_gpio_get_multiple;
	unipi_gpiochip->chip.set_multiple = unipi_gpio_set_multiple;
	*/
	if (unipi_gpiochip->data->flags & UNIPI_GPIO_OUT) {
		unipi_gpiochip->chip.set = unipi_gpio_set;
	}

	platform_set_drvdata(pdev, unipi_gpiochip);

	ret = of_property_read_u32(np, "debounce-reg", &unipi_gpiochip->debounce_reg);
	ret = of_property_read_u32(np, "counter-reg", &unipi_gpiochip->counter_reg);
	ret = of_property_read_u32(np, "ds-count", &unipi_gpiochip->ds_count);
	ret = of_property_read_u32(np, "ds-enable-coil", &unipi_gpiochip->ds_enable_coil);
	ret = of_property_read_u32(np, "ds-toggle-coil", &unipi_gpiochip->ds_toggle_coil);
	ret = of_property_read_u32(np, "ds-inv-coil", &unipi_gpiochip->ds_inv_coil);

	/* create DI/DO kobjs */
	for (i=0; i<unipi_gpiochip->ngpio; i++) {
		struct unipi_gpio_sysfs_kobj *gpio_kobj;
		gpio_kobj = kzalloc(sizeof(struct unipi_gpio_sysfs_kobj), GFP_KERNEL);
		if (gpio_kobj == NULL) break;
		gpio_kobj->gpio_dev = unipi_gpiochip;
		gpio_kobj->index = i;
		gpio_kobj->kobject.kset = unipi_gpio_kset;

		if (unipi_gpiochip->data->flags & UNIPI_GPIO_IN) {
			ret = kobject_init_and_add(&gpio_kobj->kobject, &unipi_gpio_di_ktype, &dev->kobj,
			                           unipi_gpiochip->data->fname, iogroup->address, i+1);
		} else if (i<unipi_gpiochip->ds_count) {
			ret = kobject_init_and_add(&gpio_kobj->kobject, &unipi_gpio_dods_ktype, &dev->kobj,
			                           unipi_gpiochip->data->fname, iogroup->address, i+1);
		} else {
			ret = kobject_init_and_add(&gpio_kobj->kobject, &unipi_gpio_do_ktype, &dev->kobj,
			                           unipi_gpiochip->data->fname, iogroup->address, i+1);
		}
		if (ret < 0) {
			kfree(gpio_kobj);
			break;
		}
		kobject_uevent(&gpio_kobj->kobject, KOBJ_ADD);
		gpio_kobj->next = unipi_gpiochip->kobj;
		unipi_gpiochip->kobj = gpio_kobj;
	}

	return devm_gpiochip_add_data(&pdev->dev, &unipi_gpiochip->chip, unipi_gpiochip);
}

int unipi_gpio_remove(struct platform_device *pdev)
{
	struct unipi_gpiochip_device *unipi_gpiochip = (struct unipi_gpiochip_device*) platform_get_drvdata(pdev);
	struct unipi_gpio_sysfs_kobj *kobj, *kobj_d;

	if (!unipi_gpiochip || !unipi_gpiochip->data)
		return 0;
	kobj = unipi_gpiochip->kobj;
	while (kobj) {
		kobj_d = kobj;
		kobj = kobj->next;
		kobject_del(&kobj_d->kobject);
		kobject_put(&kobj_d->kobject);
	}
	return 0;
}

static const struct of_device_id of_unipi_gpio_match[] = {
	{ .compatible = "unipi,gpio-di", .data = &unipi_gpiochip_data_di },
	{ .compatible = "unipi,gpio-do", .data = &unipi_gpiochip_data_do },
	{ .compatible = "unipi,gpio-ro", .data = &unipi_gpiochip_data_ro },
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

static int __init unipi_gpio_init(void)
{
	/* kset must be defined for DI/DO kobjs - to allow uevent sending */
	unipi_gpio_kset = kset_create_and_add("unipi_gpio", NULL, kernel_kobj);
	if (!unipi_gpio_kset)
		return -ENOMEM;
	return platform_driver_register(&unipi_gpio_driver);
}

static void __exit unipi_gpio_exit(void)
{
	kset_unregister(unipi_gpio_kset);
	return platform_driver_unregister(&unipi_gpio_driver);
}

//module_platform_driver(unipi_gpio_driver);
module_init(unipi_gpio_init);
module_exit(unipi_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi GPIO Driver");
