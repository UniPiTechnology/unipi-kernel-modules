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
#include "unipi_channel.h"
#include "unipi_mfd.h"
#include "unipi_iogroup_bus.h"

/* macros to parse fw_version, fw_variant, board_id */
#define minor(x) ((x & 0xf0)>>4)
#define is_cal(x) ((x & 0x8) != 0)

// #define to_ext_attr(x) container_of(x, struct dev_mfd_attribute, attr)
#define to_mfd_attr(x) container_of(x, struct dev_mfd_attribute, attr)
#define to_mfd_of_attr(x) container_of(x, struct dev_mfd_of_attribute, attr)


ssize_t unipi_mfd_show_version(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int val, ret;
	ret = regmap_read(channel->registers, ea->reg, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d.%d\n", hi(val), lo(val));
}

ssize_t unipi_mfd_show_variant(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int val, ret;
	ret = regmap_read(channel->registers, ea->reg, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d-%d%s\n", hi(val), minor(val), is_cal(val)?" CAL":"");
}

ssize_t unipi_mfd_store_ulong(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int ret;
	unsigned long new;

	ret = kstrtoul(buf, 0, &new);
	if (ret)
		return ret;
	ret = regmap_bulk_write(channel->registers, ea->reg, &new, 4);
	if (ret<0) return ret;
	/* Always return full write size even if we didn't consume all */
	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_ulong);

ssize_t unipi_mfd_show_ulong(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int ret;
	unsigned long val;
	ret = regmap_bulk_read(channel->registers, ea->reg, &val, 4);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%ld\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_ulong);

ssize_t unipi_mfd_store_int(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int ret;
	long new;

	printk("store_int r=%d v=%s", ea->reg, buf);
	ret = kstrtol(buf, 0, &new);
	if (ret)
		return ret;

	if (new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	ret = regmap_bulk_write(channel->registers, ea->reg, &new, 2);
	if (ret<0) return ret;
	/* Always return full write size even if we didn't consume all */
	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_int);

ssize_t unipi_mfd_show_int(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int val, ret;
	//printk("show_int r=%d\n", ea->reg);
	ret = regmap_bulk_read(channel->registers, ea->reg, &val, 2);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_int);

ssize_t unipi_mfd_store_reg(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int ret;
	long new;

	ret = kstrtol(buf, 0, &new);
	if (ret)
		return ret;

	if (new > USHRT_MAX || new < 0)
		return -EINVAL;
	ret = regmap_write(channel->registers, ea->reg, new);
	if (ret<0) return ret;
	/* Always return full write size even if we didn't consume all */
	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_reg);

ssize_t unipi_mfd_show_reg(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int val, ret;
	ret = regmap_read(channel->registers, ea->reg, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_reg);

ssize_t unipi_mfd_showhex_reg(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int val, ret;
	ret = regmap_read(channel->registers, ea->reg, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "0x%04x\n", val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_showhex_reg);

ssize_t unipi_mfd_store_bool(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	bool val;
	int ret;

	if (strtobool(buf, &val) < 0)
		return -EINVAL;
	ret = regmap_write(channel->coils, ea->reg, val);
	if (ret<0) return ret;

	return size;
}
EXPORT_SYMBOL_GPL(unipi_mfd_store_bool);

ssize_t unipi_mfd_show_bool(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int val, ret;

	ret = regmap_read(channel->coils, ea->reg, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d\n", !!val);
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_bool);

ssize_t unipi_mfd_show_regbool(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	/* low 16bit is register address, high 16 bits is shift to get bit in register value */
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int regaddr =  (ea->reg) & 0xffff;
	int shift =  (ea->reg) >> 16;
	int val, ret;

	ret = regmap_read(channel->registers, regaddr, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d\n", !!((val>>shift)&1));
}
EXPORT_SYMBOL_GPL(unipi_mfd_show_regbool);

static ssize_t sys_board_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct device_node *core_np;
	const char *sys_board_name;
	struct dev_mfd_of_attribute *ea = to_mfd_of_attr(attr);
	int ret = 0;

	if (of_node_get(dev->of_node)) {
		core_np = of_find_node_by_name(dev->of_node, "core");
		if (core_np) {
			sys_board_name = of_get_property(core_np, ea->ofname, NULL);
			of_node_put(core_np);
			if (sys_board_name) {
				ret = scnprintf(buf, 255, "%s\n", sys_board_name);
			}
		}
		// of_node_put(dev->of_node); !! must not be here - is done in of_find 
	}
	return ret;
}

static struct dev_mfd_of_attribute dev_attr_sys_board_name = {
	__ATTR(sys_board_name, 0444, sys_board_name_show, NULL),
	"sys_board_name"
};
static struct dev_mfd_of_attribute dev_attr_firmware_name = {
	__ATTR(firmware_name, 0444, sys_board_name_show, NULL),
	"fw_name"
};

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

static struct dev_mfd_attribute dev_attr_sys_board_serial = {
	__ATTR(sys_board_serial, 0444, unipi_mfd_show_int, NULL),
	UNIPI_MFD_REG_BOARD_SERIAL
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
	__ATTR(master_watchdog_timeout, 0664, unipi_mfd_show_reg, unipi_mfd_store_reg),
	UNIPI_MFD_REG_MWD_TIMEOUT
};

static struct dev_mfd_attribute dev_attr_sysled_mode = {
	__ATTR(sysled_mode, 0664, unipi_mfd_show_reg, unipi_mfd_store_reg),
	UNIPI_MFD_REG_SYSLED_MODE
};

static struct dev_mfd_attribute dev_attr_master_watchdog_enable = {
	__ATTR(master_watchdog_enable, 0664, unipi_mfd_show_reg, unipi_mfd_store_reg),
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
	&dev_attr_firmware_name.attr.attr,
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

static struct attribute *unipi_plc_device_attrs[] = {
	&dev_attr_sys_board_name.attr.attr,
	&dev_attr_sys_board_serial.attr.attr,
	NULL
};

static const struct attribute_group unipi_plc_group_def = {
	.attrs  = unipi_plc_device_attrs,
};


int devm_unipi_add_group(struct device *dev, struct attribute_group *grp);

int unipi_mfd_add_core_group(struct device *dev, struct device_node *mfd_np)
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
	unipi_mfd_group = kzalloc( sizeof(struct attribute_group)
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
	//return devm_unipi_add_group(dev, unipi_mfd_group);
	return devm_unipi_add_group(dev, unipi_mfd_group);
}


static int devm_unipi_group_match(struct device *dev, void *res, void *data)
{
	struct attribute_group **pgroup = res;
	const char *name = (*pgroup)->name;
	if (name && data) {
		return !strcmp(name, data);
	}
	return 0;
}

static void devm_unipi_group_remove(struct device *dev, void *res)
{
	struct attribute_group **pgroup = res;

	dev_dbg(dev, "%s: removing group %p\n", __func__, *pgroup);
	sysfs_remove_group(&dev->kobj, *pgroup);
	kfree(*pgroup);
}

int devm_unipi_add_group(struct device *dev, struct attribute_group *grp)
{
	struct attribute_group **devres;
	int error;

	devres = devres_alloc(devm_unipi_group_remove,
			      sizeof(*devres), GFP_KERNEL);
	if (!devres)
		return -ENOMEM;

	error = sysfs_create_group(&dev->kobj, grp);
	if (error) {
		devres_free(devres);
		return error;
	}

	*devres = grp;
	devres_add(dev, devres);
	return 0;
}


int unipi_mfd_add_group(struct device *dev, const char *groupname, struct attribute ** templ, int count, ...)
{
	//struct dev_mfd_attribute *mfda;
	struct attribute_group * unipi_mfd_group ;
	struct dev_mfd_attribute *dev_attr;
	struct attribute **attr_array;
	char * name;
	int  i, attr_name_size, ret;
	
	va_list vargs;

	attr_name_size = strlen(groupname) + 1;
	unipi_mfd_group = kzalloc( sizeof(struct attribute_group)\
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
	ret = devm_unipi_add_group(dev, unipi_mfd_group);
	return ret;
}
EXPORT_SYMBOL_GPL(unipi_mfd_add_group);

void unipi_mfd_remove_group(struct device *dev, const char *groupname)
{
	devres_release(dev, devm_unipi_group_remove,
			       devm_unipi_group_match,
			       /* cast away const */ (void *)groupname);
}
EXPORT_SYMBOL_GPL(unipi_mfd_remove_group);

//static struct of_dev_auxdata xx_auxdata_lookup[] = {
//	OF_DEV_AUXDATA("unipi,gpio-di", 0, "iogroupXdi", NULL),
//	{ /* sentinel */ },
//};

static const char *unipi_plc_linkname = "io_group%d";

int device_match_unipi_plc(struct device *dev, const void *unused)
{
	if (!dev->driver && dev_name(dev) && (strcmp(dev_name(dev),"unipi_plc")==0))
		return 1;
	return 0;
}

static void unipi_mfd_link_plc(struct unipi_iogroup_device *iogroup)
{
	struct device *plc_dev;
	struct platform_device *pdev;
	char *name;

	plc_dev = bus_find_device(&platform_bus_type, NULL, NULL, device_match_unipi_plc);
	if (!plc_dev) {
		pdev = platform_device_alloc("unipi_plc", -1);
		if (!pdev) 
			return;
		//plc_dev->dev.groups = neuron_plc_attr_groups;
		if (platform_device_add(pdev) != 0) 
			return;
		plc_dev = &pdev->dev;
	}
	if (devm_device_add_group(&iogroup->dev, &unipi_plc_group_def) < 0) return;
	name = devm_kzalloc(plc_dev, strlen(unipi_plc_linkname) + 10, GFP_KERNEL);
	if (name) {
		sprintf(name, unipi_plc_linkname, iogroup->address);
		if (sysfs_create_link(&plc_dev->kobj, &iogroup->dev.kobj, name) < 0)
			devm_kfree(plc_dev, name);
	}
}

static void unipi_mfd_remove_plc(struct unipi_iogroup_device *iogroup)
{
	//void sysfs_remove_link(struct kobject *kobj, const char *name)
	struct device *plc_dev;
	char *name;

	plc_dev = bus_find_device(&platform_bus_type, NULL, NULL, device_match_unipi_plc);
	if (!plc_dev)
		return;
	name = kzalloc(strlen(unipi_plc_linkname) + 10, GFP_KERNEL);
	if (name) {
		sprintf(name, unipi_plc_linkname, iogroup->address);
		sysfs_remove_link(&plc_dev->kobj, name);
		kfree(name);
	}
}

/* callback of poll_timer - for devices without or disfunctional interrupt */
static enum hrtimer_restart unipi_mfd_poll_timer_func(struct hrtimer *timer)
{
	struct unipi_iogroup_device *iogroup = ((container_of((timer), struct unipi_iogroup_device, poll_timer)));
	unipi_ping_async(iogroup->channel, NULL, NULL);
	//unipi_spi_trace((channel->dev), "Pseudo IRQ\n");
	return HRTIMER_RESTART;
}


int unipi_mfd_enable_interrupt(struct unipi_iogroup_device *iogroup, u16 mask)
{
	if (iogroup->irq == 0) {
		iogroup->poll_enabled = (mask != 0);
		if (iogroup->poll_enabled) {
			if ((iogroup->poll_timer.function == NULL)) {
				hrtimer_init(&iogroup->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
				iogroup->poll_timer.function = unipi_mfd_poll_timer_func;
			}
			hrtimer_start_range_ns(&iogroup->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
		} else {
			if (iogroup->poll_timer.function != NULL) {
				hrtimer_try_to_cancel(&iogroup->poll_timer);
			}
		}
	}
	regmap_write_async(iogroup->channel->registers, UNIPI_MFD_REG_INTERRUPT_MASK, mask);
	return 0;
}
EXPORT_SYMBOL_GPL(unipi_mfd_enable_interrupt);

/* real irq handler - emit idle_op asynchronously */
irqreturn_t unipi_mfd_irq(s32 irq, void *dev_id)
{
	struct unipi_iogroup_device *iogroup = (struct unipi_iogroup_device *)dev_id;
	unipi_ping_async(iogroup->channel, NULL, NULL);
	//unipi_spi_trace((iogroup->dev), "IRQ\n");
	return IRQ_HANDLED;
}

void unipi_mfd_int_status_callback(void* iogroup, u8 int_status)
{
	/* int_status & UNIPI_MFD_INT_RX_NOT_EMPTY */
}


/*
	unipi_dev = bus_find_device(&platform_bus_type, NULL, NULL, device_match_unipi_id);
	if (unipi_dev) {
		struct unipi_id_data *unipi_id = dev_get_platdata(unipi_dev);
		dev_info(dev, "Found PLC %s Model: %.6s\n",  unipi_id_get_family_name(unipi_id), unipi_id->descriptor.product_info.model_str);
		put_device(unipi_dev);
	} else {
		return  -EPROBE_DEFER;
	}

	struct kernfs_node *value_kn;

	data->value_kn = sysfs_get_dirent(unipi_dev->kobj.sd, "unipi_id");
	sysfs_notify_dirent(data->value_kn);

	sysfs_put(data->value_kn);

****
		nfit_kernfs = sysfs_get_dirent(nvdimm_kobj(nvdimm)->sd, "nfit");
		if (nfit_kernfs)
			nfit_mem->flags_attr = sysfs_get_dirent(nfit_kernfs,
					"flags");
		sysfs_put(nfit_kernfs);

*/

int unipi_mfd_probe(struct unipi_iogroup_device *iogroup)
{
	struct device *dev = &iogroup->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	struct device_node *core_np;
	int fw_variant = -1;
	u16 id_registers[5];
	const char *name1 = NULL;
	const char *name2 = NULL;
	const char *sys_board_name = NULL;

	int board_id, dt_variant = -1, degraded = 0;
	
	if (of_node_get(dev->of_node)) {
		core_np = of_find_node_by_name(dev->of_node, "core");
		if (core_np) {
			of_property_read_u32(core_np, "fw_variant", &dt_variant);
			name1 = of_get_property(core_np, dev_attr_firmware_name.ofname, NULL);
			name2 = of_get_property(core_np, "board_name", NULL);
			sys_board_name = of_get_property(core_np, dev_attr_sys_board_name.ofname, NULL);
			of_node_put(core_np);
		}
		// of_node_put(dev->of_node); !! must not be here - is done in of_find 
	}
	/* read identification registers */
	ret = regmap_bulk_read(iogroup->channel->registers, UNIPI_MFD_REG_FW_VERSION, id_registers, ARRAY_SIZE(id_registers));
	if (ret==0) {
		/* check device tree mismatch */
		if (dt_variant != id_registers[3]) {
			name1 = name2 = NULL;
		}
		fw_variant = id_registers[3];
		name1 = name1? : "";
		board_id = id_registers[4];
		name2 = name2? : name1;
		dev_info(dev, "Found board %s (id=0x%x-%d).\n\t\t\tFirmware variant %s (0x%x-%d%s) version=%d.%d\n",
					name2, hi(board_id), minor(board_id),
					name1, hi(fw_variant), minor(fw_variant), is_cal(fw_variant)?"C":"",
					hi(id_registers[0]), lo(id_registers[0]));
		if (id_registers[0] == 0x0600) {
			degraded = 1;
			dev_warn(dev, "Device is in degraded (bootloader) mode. Load operational firmware!\n");
			return 0;

		} else if (dt_variant == -1) {
			dev_warn(dev, "Device found doesn't comply with provided device tree (%d-%d%s).\n"\
			              "\t\tNot fully operational.\n",
			              hi(dt_variant), minor(dt_variant), is_cal(dt_variant)?"C":"");
		}
	}

	if (dt_variant != -1) {
		unipi_mfd_add_core_group(dev, np);
		devm_of_platform_populate(dev);
		unipi_populated(iogroup->channel);
		ret = 0;
	} else {
		ret = devm_device_add_group(dev, &unipi_mfd_group_def);
	}
	/* Fixup for Mervis old licensing process */
	if ((iogroup->address < 114) && (sys_board_name)) {
		unipi_mfd_link_plc(iogroup);
	}

	if (iogroup->irq) {
		ret = devm_request_irq(dev, iogroup->irq, unipi_mfd_irq, 0, dev_name(dev), iogroup);
		dev_info(dev, "IRQ %d registration: ret=%d\n", iogroup->irq, ret);
		if (ret != 0) iogroup->irq = 0;
	}
	iogroup->channel->interrupt_status_callback = unipi_mfd_int_status_callback;
	iogroup->channel->interrupt_self = iogroup;

	return ret;
}

int unipi_mfd_remove(struct unipi_iogroup_device *iogroup)
{
	//struct unipi_channel *mfd_iogroup = unipi_iogroup_get_drvdata(iogroup);
/*	if (mfd_iogroup) {
		if (mfd_iogroup->spi_dev) put_device(&mfd_iogroup->spi_dev->dev);
	}*/
//	struct device* dev = &iogroup->dev;

//	device_for_each_child_reverse(dev, NULL, mfd_iogroup_remove_devices_fn);
	if (iogroup->poll_timer.function != NULL) {
		hrtimer_try_to_cancel(&iogroup->poll_timer);
	}
	unipi_mfd_remove_plc(iogroup);
	return 0;
}


/*********************
 * Final definitions *
 *********************/

static const struct of_device_id unipi_mfd_id_match[] = {
		{.compatible = "unipi,unipi-mfd"},
		{}
};

MODULE_DEVICE_TABLE(of, unipi_mfd_id_match);
//MODULE_ALIAS("iogroup:regmap-group");
//MODULE_ALIAS("regmap-group");

struct unipi_iogroup_driver unipi_mfd_driver =
{
	.driver =
	{
		.name			= "unipi-mfd",
		.of_match_table	= of_match_ptr(unipi_mfd_id_match)
	},
	.probe				= unipi_mfd_probe,
	.remove				= unipi_mfd_remove,
};

struct regmap* unipi_mfd_get_regmap(struct device* dev, const char* name)
{
	struct unipi_iogroup_device *iogroup;
	struct unipi_channel *mfd_iogroup;

	/* test if device is driven by this driver */
	if ((dev==NULL) || (dev->driver==NULL) || (strcmp(unipi_mfd_driver.driver.name, dev->driver->name) !=0 ))
		return NULL;

	iogroup = to_unipi_iogroup_device(dev);
	mfd_iogroup = to_unipi_iogroup_device(dev)->channel;
	//mfd_iogroup = (struct unipi_channel *)unipi_iogroup_get_drvdata(iogroup);
	if (strcmp(name, "coils") == 0)
		return mfd_iogroup->coils;
	if (strcmp(name, "registers") == 0)
		return mfd_iogroup->registers;
	return NULL;
}
EXPORT_SYMBOL_GPL(unipi_mfd_get_regmap);

static int __init unipi_mfd_init(void)
{
	int ret = 0;

	ret = unipi_iogroup_register_driver(&unipi_mfd_driver);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to register driver --> %d\n", unipi_mfd_driver.driver.name, ret);
		return ret;
	}
	return ret;
}

static void __exit unipi_mfd_exit(void)
{
	unipi_iogroup_unregister_driver(&unipi_mfd_driver);
}

module_init(unipi_mfd_init);
module_exit(unipi_mfd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi MFD Iogroup Driver");
