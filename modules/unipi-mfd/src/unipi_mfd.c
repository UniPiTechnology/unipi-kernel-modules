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


struct dev_mfd_attribute {
	struct device_attribute attr;
	u32 reg;
};

struct dev_mfd_of_attribute {
	struct device_attribute attr;
	const char* ofname;
};


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
	return sysfs_emit(buf, "0x%02x-%d%s\n", hi(val), minor(val), is_cal(val)?" CAL":"");
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

ssize_t unipi_mfd_store_int(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct dev_mfd_attribute *ea = to_mfd_attr(attr);
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int ret;
	long new;

	//printk("store_int r=%d v=%s", ea->reg, buf);
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


static int unipi_mfd_of_get_regnum(struct device *dev, char* attrname)
{
	struct device_node *np = to_unipi_iogroup_device(dev)->variant_node;
	int ret, regaddr;

	if (!of_node_get(np)) return -EINVAL;
	ret = of_property_read_u32(np, attrname, &regaddr);
	of_node_put(np);
	if (ret < 0) return ret;
	if (regaddr > 3000) return -EINVAL;
	return regaddr;
}

ssize_t unipi_mfd_show_wd_enable(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int regaddr, val, ret;

	regaddr = unipi_mfd_of_get_regnum(dev, "master_watchdog_enable");
	if (regaddr < 0) return  regaddr;
	ret = regmap_read(channel->registers, regaddr, &val);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%d\n", !!((val)&1));
}

ssize_t unipi_mfd_store_wd_enable(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t size)
{
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	bool val;
	int ret, regaddr;

	regaddr = unipi_mfd_of_get_regnum(dev, "master_watchdog_enable");
	if (strtobool(buf, &val) < 0)
		return -EINVAL;

	if (strtobool(buf, &val) < 0)
		return -EINVAL;
	ret = regmap_write(channel->registers, regaddr, val);
	if (ret<0) return ret;
	return size;
}

ssize_t unipi_mfd_show_cycle(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct unipi_channel *channel = to_unipi_iogroup_device(dev)->channel;
	int regaddr, ret;
	unsigned long val;
	regaddr = unipi_mfd_of_get_regnum(dev, "cycle_counter");
	if (regaddr < 0) return  regaddr;

	ret = regmap_bulk_read(channel->registers, regaddr, &val, 4);
	if (ret<0) return ret;
	return sysfs_emit(buf, "%ld\n", val);
}


static ssize_t sys_board_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct device_node *np = to_unipi_iogroup_device(dev)->variant_node;
	const char *sys_board_name;
	struct dev_mfd_of_attribute *ea = to_mfd_of_attr(attr);
	int ret = 0;

	if (of_node_get(np)) {
		sys_board_name = of_get_property(np, ea->ofname, NULL);
		of_node_put(np);
		if (sys_board_name) {
			ret = scnprintf(buf, 255, "%s\n", sys_board_name);
		}
	}
	return ret;
}

static ssize_t firmware_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct device_node *np = to_unipi_iogroup_device(dev)->variant_node;
	const char *firmware_name = NULL;
	int ret = 0;
	int index = to_unipi_iogroup_device(dev)->variant_index;
	if (index < 1)
		return 0;

	if (of_node_get(np)) {
		of_property_read_string_index(np, "fw_name", index-1, &firmware_name);
		of_node_put(np);
		if (firmware_name) {
			ret = scnprintf(buf, 255, "%s\n", firmware_name);
		}
	}
	return ret;
}

DEVICE_ATTR_RO(firmware_name);

static struct dev_mfd_of_attribute dev_attr_sys_board_name = {
	__ATTR(sys_board_name, 0444, sys_board_name_show, NULL),
	"sys_board_name"
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
	__ATTR(master_watchdog_enable, 0664, unipi_mfd_show_wd_enable, unipi_mfd_store_wd_enable),
	0
};

static struct dev_mfd_attribute dev_attr_cycle_counter = {
	__ATTR(cycle_counter, 0444, unipi_mfd_show_cycle, NULL),
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
	&dev_attr_firmware_name.attr,
	&dev_attr_cycle_counter.attr.attr,
	&dev_attr_master_watchdog_enable.attr.attr,
	NULL
};


static const struct attribute_group unipi_mfd_group_def = {
//	.name  = "core",
	.attrs  = unipi_mfd_device_attrs,
};

#ifdef CONFIG_OF
static struct dev_mfd_attribute dev_attr_sys_board_serial = {
	__ATTR(sys_board_serial, 0444, unipi_mfd_show_int, NULL),
	UNIPI_MFD_REG_BOARD_SERIAL
};

static struct attribute *unipi_plc_device_attrs[] = {
	&dev_attr_sys_board_name.attr.attr,
	&dev_attr_sys_board_serial.attr.attr,
	NULL
};

static const struct attribute_group unipi_plc_group_def = {
	.attrs  = unipi_plc_device_attrs,
};


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
#endif

void unipi_mfd_poll_callback(void* cb_data, int result)
{
	struct unipi_iogroup_device *iogroup = (struct unipi_iogroup_device *) cb_data;
	hrtimer_start_range_ns(&iogroup->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
}


/* callback of poll_timer - for devices without or disfunctional interrupt */
static enum hrtimer_restart unipi_mfd_poll_timer_func(struct hrtimer *timer)
{
	struct unipi_iogroup_device *iogroup = ((container_of((timer), struct unipi_iogroup_device, poll_timer)));
	if (unipi_ping_async(iogroup->channel, iogroup, unipi_mfd_poll_callback) != 0) {
		// ?? ToDo: return HRTIMER_RESTART;
	}
	//unipi_spi_trace((channel->dev), "Pseudo IRQ\n");
	return HRTIMER_NORESTART;
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

void unipi_mfd_int_status_callback(void* self, u8 int_status)
{
	struct unipi_iogroup_device *iogroup = (struct unipi_iogroup_device *)self;
	/* int_status & UNIPI_MFD_INT_RX_NOT_EMPTY */

	if (int_status & (UNIPI_MFD_INT_RX_NOT_EMPTY | UNIPI_MFD_INT_RX_MODBUS)) {
		if (iogroup->uart_rx_callback) {
			iogroup->uart_rx_callback(iogroup->uart_rx_self, 0);
		}
	}
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

#define MAX_ALLOWED_FW 32
int unipi_mfd_find_variant(struct device *dev, struct device_node *node, u16 fw_variant)
{
	u32 allowed_fw_variants[MAX_ALLOWED_FW];
	int i;
	int allowed_count = -1;

	allowed_count = of_property_read_variable_u32_array(node, "fw_variant", allowed_fw_variants, 1, MAX_ALLOWED_FW);
	/* skip lowest nibble of fwvariant in compare */
	for (i=0; i< allowed_count; i++) {
		if ((allowed_fw_variants[i] >> 4) == (fw_variant >> 4)) {
			return i+1;
		}
	}
	return 0;
}

int unipi_mfd_probe(struct unipi_iogroup_device *iogroup)
{
	struct device *dev = &iogroup->dev;
	//struct device_node *np = dev->of_node;
	int ret = 0;

	struct device_node *child;
	u16 id_registers[5];
	const char *name1 = NULL;
	const char *name2 = NULL;
	const char *sys_board_name = NULL;
	u16 fw_variant;
	int found = 0;
	int board_id;

	/* read identification registers */
	ret = regmap_bulk_read(iogroup->channel->registers, UNIPI_MFD_REG_FW_VERSION, id_registers, ARRAY_SIZE(id_registers));
	if (ret != 0)
		return ret;
	board_id = id_registers[4];
	fw_variant = id_registers[3];

	found = unipi_mfd_find_variant(dev, dev->of_node, fw_variant);
	if (!found) {
		for_each_child_of_node(dev->of_node, child) {
			found = unipi_mfd_find_variant(dev, child, fw_variant);
			if (found) break;
		}
	} else {
		child = dev->of_node;
	}
	if (found && of_node_get(child)) {
		//name1 = of_get_property(core_np, dev_attr_firmware_name.ofname, NULL);
		of_property_read_string_index(child, "fw_name", found-1, &name1);
		name2 = of_get_property(child, "board_name", NULL);
		sys_board_name = of_get_property(child, dev_attr_sys_board_name.ofname, NULL);
		of_node_put(child);
	}
	name1 = (name1 && found)? name1: "";
	name2 = (name2 && found)? name2: name1;
	dev_info(dev, "Found board %s (id=0x%x-%d).\n\t\t\tFirmware variant %s (0x%x-%d%s) version=%d.%d\n",
					name2, hi(board_id), minor(board_id),
					name1, hi(fw_variant), minor(fw_variant), is_cal(fw_variant)?"C":"",
					hi(id_registers[0]), lo(id_registers[0]));

	if (id_registers[0] == 0x0600) {
		dev_warn(dev, "Device is in degraded (bootloader) mode. Load operational firmware!\n");
		return 0;
	}
	ret = devm_device_add_group(dev, &unipi_mfd_group_def);
	iogroup->variant_index = found;
	if (!found) {
		dev_warn(dev, "Device found doesn't comply with provided device tree. Not fully operational.\n");
	} else {
		iogroup->variant_node = child;
		if (of_platform_populate(child, NULL, NULL, dev)==0) {
			unipi_populated(iogroup->channel);
		}
		ret = 0;
	}
#ifdef CONFIG_OF
	/* Fixup for Mervis old licensing process */
	if ((iogroup->address >= 1) && (iogroup->address <= 3) && (sys_board_name)) {
		unipi_mfd_link_plc(iogroup);
	}
#endif
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
	if (iogroup->poll_timer.function != NULL) {
		hrtimer_try_to_cancel(&iogroup->poll_timer);
	}
#ifdef CONFIG_OF
	unipi_mfd_remove_plc(iogroup);
#endif
	/* depopulate platform devices (of_platform_depopulate) */
	if (iogroup->variant_node && of_node_check_flag(iogroup->variant_node, OF_POPULATED_BUS)) {
		device_for_each_child_reverse(&iogroup->dev, NULL, of_platform_device_destroy);
		of_node_clear_flag(iogroup->variant_node, OF_POPULATED_BUS);
	}

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
	if (strcmp(name, "coils") == 0)
		return mfd_iogroup->coils;
	if (strcmp(name, "registers") == 0)
		return mfd_iogroup->registers;
	return NULL;
}
EXPORT_SYMBOL_GPL(unipi_mfd_get_regmap);

static int __init unipi_mfd_init(void)
{
	int ret;
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
