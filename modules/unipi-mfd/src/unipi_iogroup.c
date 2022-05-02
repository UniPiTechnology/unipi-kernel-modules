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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/nvmem-consumer.h>

#include "unipi_common.h"
#include "unipi_mfd.h"
#include "unipi_mfd_iogroup.h"
#include "unipi_plc.h"

// DOCASNE!!!
#define unipi_mfd_iogroup_device unipi_mfd_device

// #define to_ext_attr(x) container_of(x, struct dev_mfd_attribute, attr)
#define to_mfd_attr(x) container_of(x, struct dev_mfd_attribute, attr)

ssize_t unipi_mfd_show_version(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int val;
	regmap_read(mfd->registers, ea->reg, &val);
	return sysfs_emit(buf, "%d.%d\n", hi(val), lo(val));
}

ssize_t unipi_mfd_show_variant(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int val;
	regmap_read(mfd->registers, ea->reg, &val);
	return sysfs_emit(buf, "%d-%d%s\n", hi(val), minor(val), is_cal(val)?" CAL":"");
}

ssize_t unipi_mfd_store_ulong(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int ret;
	unsigned long new;

	ret = kstrtoul(buf, 0, &new);
	if (ret)
		return ret;
	regmap_bulk_write(mfd->registers, ea->reg, &new, 4);
	/* Always return full write size even if we didn't consume all */
	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_ulong);

ssize_t unipi_mfd_show_ulong(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	unsigned long val;
	regmap_bulk_read(mfd->registers, ea->reg, &val, 4);
	return sysfs_emit(buf, "%ld\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_ulong);

ssize_t unipi_mfd_store_int(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int ret;
	long new;

	printk("store_int r=%d v=%s", ea->reg, buf);
	ret = kstrtol(buf, 0, &new);
	if (ret)
		return ret;

	if (new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	regmap_bulk_write(mfd->registers, ea->reg, &new, 2);
	/* Always return full write size even if we didn't consume all */
	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_int);

ssize_t unipi_mfd_show_int(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int val;
	printk("show_int r=%d\n", ea->reg);
	regmap_bulk_read(mfd->registers, ea->reg, &val, 2);
	return sysfs_emit(buf, "%d\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_int);

ssize_t unipi_mfd_store_reg(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int ret;
	long new;

	ret = kstrtol(buf, 0, &new);
	if (ret)
		return ret;

	if (new > USHRT_MAX || new < 0)
		return -EINVAL;
	regmap_write(mfd->registers, ea->reg, new);
	/* Always return full write size even if we didn't consume all */
	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_reg);

ssize_t unipi_mfd_show_reg(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int val;
	regmap_read(mfd->registers, ea->reg, &val);
	return sysfs_emit(buf, "%d\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_reg);

ssize_t unipi_mfd_showhex_reg(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int val;
	regmap_read(mfd->registers, ea->reg, &val);
	return sysfs_emit(buf, "0x%04x\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_showhex_reg);

ssize_t unipi_mfd_store_bool(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	bool val;

	if (strtobool(buf, &val) < 0)
		return -EINVAL;
	regmap_write(mfd->coils, ea->reg, val);

	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_bool);

ssize_t unipi_mfd_show_bool(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_mfd_iogroup_device *mfd = dev_get_drvdata(dev);
	int val;

	regmap_read(mfd->coils, ea->reg, &val);
	return sysfs_emit(buf, "%d\n", !!val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_bool);

static struct dev_mfd_attribute dev_attr_was_watchdog = {
	__ATTR(was_watchdog, 0664, unipi_mfd_show_bool, unipi_mfd_store_bool),
	UNIPI_MFD_COIL_WAS_WATCHDOG
};

static struct dev_mfd_attribute dev_attr_ow_power_off = {
	__ATTR(ow_power_off, 0664, unipi_mfd_show_bool, unipi_mfd_store_bool),
	UNIPI_MFD_COIL_OW_POWER_OFF
};

static struct dev_mfd_attribute dev_attr_reboot = {
	__ATTR(reboot, 0220, NULL, unipi_mfd_store_bool),
	UNIPI_MFD_COIL_REBOOT
};

static struct dev_mfd_attribute dev_attr_save_nvram = {
	__ATTR(save_nvram, 0220, NULL, unipi_mfd_store_bool),
	UNIPI_MFD_COIL_NVRAM_SAVE
};

static struct dev_mfd_attribute dev_attr_firmware_version = {
	__ATTR(firmware_version, 0444, unipi_mfd_show_version, NULL),
	UNIPI_MFD_REG_FW_VERSION
};

static struct dev_mfd_attribute dev_attr_firmware_variant = {
	__ATTR(firmware_variant, 0444, unipi_mfd_show_variant, NULL),
	UNIPI_MFD_REG_FW_VARIANT
};

static struct dev_mfd_attribute dev_attr_board_id = {
	__ATTR(board_id, 0444, unipi_mfd_show_variant, NULL),
	UNIPI_MFD_REG_BOARD_ID
};

static struct dev_mfd_attribute dev_attr_board_serial = {
	__ATTR(board_serial, 0444, unipi_mfd_show_int, NULL),
	UNIPI_MFD_REG_BOARD_SERIAL
};

static struct dev_mfd_attribute dev_attr_interrupt_mask = {
	__ATTR(interrupt_mask, 0444, unipi_mfd_showhex_reg, NULL),
	UNIPI_MFD_REG_INTERRUPT_MASK
};

static struct dev_mfd_attribute dev_attr_master_watchdog_timeout = {
	__ATTR(master_watchdog_timeout, 0444, unipi_mfd_show_reg, NULL),
	UNIPI_MFD_REG_MWD_TIMEOUT
};

static struct dev_mfd_attribute dev_attr_sysled_mode = {
	__ATTR(sysled_mode, 0664, unipi_mfd_show_reg, unipi_mfd_store_reg),
	UNIPI_MFD_REG_SYSLED_MODE
};

static struct dev_mfd_attribute dev_attr_master_watchdog_enable = {
	__ATTR(master_watchdog_enable, 0444, unipi_mfd_show_reg, NULL),
	0
};

static struct dev_mfd_attribute dev_attr_cycle_counter = {
	__ATTR(cycle_counter, 0444, unipi_mfd_show_ulong, NULL),
	0
};

static struct attribute *unipi_mfd_device_attrs[] = {
	&dev_attr_reboot.attr.attr,
	&dev_attr_was_watchdog.attr.attr,
	&dev_attr_ow_power_off.attr.attr,
	&dev_attr_save_nvram.attr.attr,
	&dev_attr_firmware_version.attr.attr,
	&dev_attr_firmware_variant.attr.attr,
	&dev_attr_board_id.attr.attr,
	&dev_attr_board_serial.attr.attr,
	&dev_attr_interrupt_mask.attr.attr,
	&dev_attr_master_watchdog_timeout.attr.attr,
	&dev_attr_sysled_mode.attr.attr,
	NULL
};


static struct attribute *unipi_mfd_optional_attrs[] = {
	&dev_attr_cycle_counter.attr.attr,
	&dev_attr_master_watchdog_enable.attr.attr,
};

static const struct attribute_group unipi_mfd_group_def = {
	.name  = "core",
	.attrs  = unipi_mfd_device_attrs,
};

int unipi_mfd_iogroup_add_core_group(struct device *dev, struct device_node *mfd_np)
{
	//struct dev_mfd_attribute *mfda;
	struct attribute_group * unipi_mfd_group ;
	struct device_node *core_np = NULL;
	struct dev_mfd_attribute *dev_attr;
	struct attribute **attr_array;
	int val, i, count = 0;

	/* Find DT group "core" and count additional properties */
	/*   of_find_node_by_name makes of_node_put(from), 
	 *   so we must do get before calling it */
	if (of_node_get(mfd_np)) {
		core_np = of_find_node_by_name(mfd_np, "core");
		if (core_np) {
			for (i=0; i<ARRAY_SIZE(unipi_mfd_optional_attrs); i++) {
				if (of_property_read_u32(core_np, unipi_mfd_optional_attrs[i]->name, &val)==0)
					count ++;
			}
		}
	}
	if (count == 0) {
		if (core_np) of_node_put(core_np);
		return devm_device_add_group(dev, &unipi_mfd_group_def);
	}
	/* There are additional attributes */
	unipi_mfd_group = devm_kzalloc(dev, sizeof(struct attribute_group)
	                            + count * sizeof(struct dev_mfd_attribute)
								+ (ARRAY_SIZE(unipi_mfd_device_attrs) + count) * sizeof(void*),
								GFP_KERNEL);
	if (unipi_mfd_group == NULL) {
		of_node_put(core_np);
		return -ENOMEM;
	}
	dev_attr   = (struct dev_mfd_attribute*)((u8*)unipi_mfd_group + sizeof(struct attribute_group));
	attr_array = (struct attribute**)((u8*)dev_attr + count * sizeof(struct dev_mfd_attribute));
	unipi_mfd_group->name = unipi_mfd_group_def.name;
	unipi_mfd_group->attrs = attr_array;

	for (i=0; i<ARRAY_SIZE(unipi_mfd_optional_attrs); i++) {
		if (of_property_read_u32(core_np, unipi_mfd_optional_attrs[i]->name, &val)==0) {
			*attr_array++ = &dev_attr->attr.attr;
			memmove(dev_attr, unipi_mfd_optional_attrs[i], sizeof(struct dev_mfd_attribute));
			dev_attr->reg = val;
			dev_attr++;
		}
	}
	memmove(attr_array, unipi_mfd_device_attrs, sizeof(unipi_mfd_device_attrs));
	of_node_put(core_np);
	return devm_device_add_group(dev, unipi_mfd_group);
}


int unipi_mfd_iogroup_probe_registers(struct device* dev, struct unipi_mfd_device *mfd)
{
	struct device_node *core_np;
	int fw_variant = -1;
	int ret;
	u16 id_registers[6];
	const char *name1 = NULL;
	const char *name2 = NULL;
	int board_id, dt_variant;
	
	if (of_node_get(dev->of_node)) {
		core_np = of_find_node_by_name(dev->of_node, "core");
		if (core_np) {
			of_property_read_u32(core_np, "fw_variant", &dt_variant);
			name1 = of_get_property(core_np, "fw_name", NULL);
			name2 = of_get_property(core_np, "board_name", NULL);
			of_node_put(core_np);
		}
	}
	/* read identification registers */
	ret = regmap_bulk_read(mfd->registers, UNIPI_MFD_REG_FW_VERSION, id_registers, ARRAY_SIZE(id_registers));
	if (ret==0) {
		/* check device tree mismatch */
		if (dt_variant != id_registers[3]) {
			ret = 1;
			name1 = name2 = NULL;
		}
		fw_variant = id_registers[3];
		name1 = name1? : "";//unipi_mfd_find_firmware_name(hi(fw_variant));
		board_id = id_registers[4];
		name2 = name2? : "";//unipi_mfd_find_firmware_name(hi(board_id));
		dev_info(dev, "Found board %s (id=%d-%d). Firmware variant %s (%d-%d%s) version=%d.%d\n",
					name2, hi(board_id), minor(board_id),
					name1, hi(fw_variant), minor(fw_variant), is_cal(fw_variant)?"C":"",
					hi(id_registers[0]), lo(id_registers[0]));
		/*if (id_registers[0] == 0x0600) {
			degraded = 1;
			dev_warn(dev, "Device is in degraded (bootloader) mode. Load operational firmware!\n");
		} else */
		if (ret) {
			dev_warn(dev, "Device found doesn't comply with provided device tree (%d-%d%s).\n"\
			              "\t\tNot fully operational.\n",
			              hi(dt_variant), minor(dt_variant), is_cal(dt_variant)?"C":"");
		}
	}
	return ret;
}

int unipi_mfd_add_group(struct device *dev, const char *groupname, struct attribute ** templ, int count, ...)
{
	//struct dev_mfd_attribute *mfda;
	struct attribute_group * unipi_mfd_group ;
	struct dev_mfd_attribute *dev_attr;
	struct attribute **attr_array;
	char * name;
	int  i, attr_name_size;
	
	va_list vargs;

	attr_name_size = strlen(groupname) + 1;
	unipi_mfd_group = devm_kzalloc(dev, sizeof(struct attribute_group)\
	                            + count * sizeof(struct dev_mfd_attribute)\
								+ (count+1) * sizeof(void*)\
								+ attr_name_size,
								GFP_KERNEL);
	if (unipi_mfd_group == NULL) {
		return -ENOMEM;
	}

	dev_attr   = (struct dev_mfd_attribute*)((u8*)unipi_mfd_group + sizeof(struct attribute_group));
	attr_array = (struct attribute**)((u8*)dev_attr + count * sizeof(struct dev_mfd_attribute));
	name = (char*) ((u8*)attr_array + (count+1) * sizeof(void*));
	strcpy(name, groupname);
	unipi_mfd_group->name = name;
	unipi_mfd_group->attrs = attr_array;

	va_start(vargs, count);
	for (i=0; i<count; i++) {
		*attr_array++ = &dev_attr->attr.attr;
		memmove(dev_attr, templ[i], sizeof(struct dev_mfd_attribute));
		dev_attr->reg = va_arg(vargs, u32);
		dev_attr++;
	}
	va_end(vargs);
	/* sentinel */ 
	*attr_array = NULL;
	return devm_device_add_group(dev, unipi_mfd_group);
}
EXPORT_SYMBOL_GPL(unipi_mfd_add_group);

//static struct of_dev_auxdata xx_auxdata_lookup[] = {
//	OF_DEV_AUXDATA("unipi,gpio-di", 0, "iogroupXdi", NULL),
//	{ /* sentinel */ },
//};


int unipi_mfd_iogroup_probe(struct unipi_iogroup_device *iogroup)
{
	struct device *dev = &iogroup->dev;
	struct device_node *np = dev->of_node;
	struct device_node *spi_np;
	//struct unipi_mfd_iogroup_device *mfd_iogroup;
	struct unipi_mfd_device *mfd_iogroup  = NULL;
	struct spi_device * spi_dev;
	//u16 id_registers[6];
	int ret = 0;
	//unsigned long flags;

	spi_np = of_parse_phandle(np, "unipispi", 0);
	if (!spi_np) 
		goto deferred;
		
	spi_dev = of_find_spi_device_by_node(spi_np); /* must call device_put() after using */
	of_node_put(spi_np);
	if (!spi_dev) 
		goto deferred;

	mfd_iogroup = unipi_spi_get_mfd(spi_dev);
	if (!mfd_iogroup) 
		goto deferred;

	if (device_link_add(dev, &spi_dev->dev, DL_FLAG_AUTOREMOVE_CONSUMER) == NULL) {
		dev_err(dev, "Error create link\n");
	}

	unipi_iogroup_set_drvdata(iogroup, mfd_iogroup);
	
	ret = unipi_mfd_iogroup_probe_registers(dev, mfd_iogroup);
	//regmap_bulk_read(mfd_iogroup->registers, 1000, id_registers, ARRAY_SIZE(id_registers));
	//dev_info(dev, "regs ret=%d, regs= %04x %04x\n", ret, id_registers[0], id_registers[5]);

	if (ret == 0) {
		unipi_mfd_iogroup_add_core_group(dev, np);
		devm_of_platform_populate(dev);
		unipi_mfd_populated(mfd_iogroup);
		ret = 0;
	} else {
		ret = devm_device_add_group(dev, &unipi_mfd_group_def);
	}
/*
	ret = of_property_read_u32(np, "debounce-reg", &priv->debounce_reg);
	ret = of_property_read_u32(np, "counter-reg", &priv->counter_reg);

	if (priv->counter_reg>=0)
		unipi_gpio_add_counter_group(pdev->dev.parent, priv->ngpio, "counter", priv->counter_reg, 2);
*/
	return ret;

deferred:
	//dev_err(dev, "No working MFD with regmap\n");
	return -EPROBE_DEFER;
}

int unipi_mfd_iogroup_remove(struct unipi_iogroup_device *iogroup)
{
	//struct unipi_mfd_iogroup_device *mfd_iogroup = unipi_iogroup_get_drvdata(iogroup);
/*	if (mfd_iogroup) {
		if (mfd_iogroup->spi_dev) put_device(&mfd_iogroup->spi_dev->dev);
	}*/
//	struct device* dev = &iogroup->dev;

//	device_for_each_child_reverse(dev, NULL, mfd_iogroup_remove_devices_fn);
	return 0;
}


/*********************
 * Final definitions *
 *********************/

static const struct of_device_id unipi_mfd_iogroup_id_match[] = {
		{.compatible = "unipi,regmap-group"},
		{}
};

MODULE_DEVICE_TABLE(of, unipi_mfd_iogroup_id_match);
//MODULE_ALIAS("iogroup:regmap-group");
//MODULE_ALIAS("regmap-group");

struct unipi_iogroup_driver unipi_mfd_iogroup_driver =
{
	.driver =
	{
		.name			= "regmap-group",
		.of_match_table	= of_match_ptr(unipi_mfd_iogroup_id_match)
	},
	.probe				= unipi_mfd_iogroup_probe,
	.remove				= unipi_mfd_iogroup_remove,
};

struct regmap* unipi_mfd_get_regmap(struct device* dev, const char* name)
{
	struct unipi_iogroup_device *iogroup;
	struct unipi_mfd_iogroup_device *mfd_iogroup;

	/* test if device is driven by this driver */
	if ((dev==NULL) || (dev->driver==NULL) || (strcmp(unipi_mfd_iogroup_driver.driver.name, dev->driver->name) !=0 ))
		return NULL;

	iogroup = to_unipi_iogroup_device(dev);
	mfd_iogroup = (struct unipi_mfd_iogroup_device *)unipi_iogroup_get_drvdata(iogroup);
	if (strcmp(name, "coils") == 0)
		return mfd_iogroup->coils;
	if (strcmp(name, "registers") == 0)
		return mfd_iogroup->registers;
	return NULL;
}
EXPORT_SYMBOL_GPL(unipi_mfd_get_regmap);

static int __init unipi_mfd_iogroup_init(void)
{
	int ret = 0;

	ret = unipi_iogroup_register_driver(&unipi_mfd_iogroup_driver);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to register driver --> %d\n", unipi_mfd_iogroup_driver.driver.name, ret);
		return ret;
	}
	return ret;
}

static void __exit unipi_mfd_iogroup_exit(void)
{
	unipi_iogroup_unregister_driver(&unipi_mfd_iogroup_driver);
}

module_init(unipi_mfd_iogroup_init);
module_exit(unipi_mfd_iogroup_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi MFD Iogroup Driver");
