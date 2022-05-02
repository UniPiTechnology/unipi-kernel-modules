/*
 * Unipi PLC spi device driver - Copyright (C) 2018 Unipi Technology
 * 
 * Author: Tomas Knot <tomasknot@gmail.com>
 * Author: Miroslav Ondra <ondra@faster.cz>
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
#include <linux/completion.h>
#include <linux/cpufreq.h>
#include <linux/version.h>
#include <uapi/linux/sched/types.h>

#include "unipi_common.h"
#include "unipi_modbus.h"
#include "unipi_spi.h"
#include "unipi_spi_crc.h"


static ssize_t messages_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.messages);
	return len;
}
static ssize_t bytes_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%llu", n_spi->stat.bytes);
	return len;
}
static ssize_t errors_tx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_tx);
	return len;
}
static ssize_t messages_prio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	//unsigned long flags;
	//spin_lock_irqsave(&n_spi->lock, flags);
	len = sprintf(buf, "%lu", n_spi->stat.messages_prio);
	//spin_unlock_irqrestore(&n_spi->lock, flags);
	return len;
}
static ssize_t errors_crc1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_crc1);
	return len;
}
static ssize_t errors_crc2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_crc2);
	return len;
}
static ssize_t errors_opcode1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%lu", n_spi->stat.errors_opcode1);
	return len;
}

static ssize_t firmware_in_progress_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%d", n_spi->firmware_in_progress);
	return len;
}

static ssize_t frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%u Hz", n_spi->frequency);
	return len;
}
static ssize_t frequency_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		n_spi->frequency = val;
	}
	return count;
}

static ssize_t hmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	ssize_t len;
	len = sprintf(buf, "%d", n_spi->hmode);
	return len;
}
static ssize_t hmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);
	unsigned int val = 0;
	if (kstrtouint(buf, 0, &val) >= 0) {
		if (val == 0 || val == 1)
			n_spi->hmode = val;
	}
	return count;
}

DEVICE_ATTR_RO(messages);
DEVICE_ATTR_RO(errors_tx);
DEVICE_ATTR_RO(bytes);
DEVICE_ATTR_RO(messages_prio);
DEVICE_ATTR_RO(errors_crc1);
DEVICE_ATTR_RO(errors_crc2);
DEVICE_ATTR_RO(errors_opcode1);

DEVICE_ATTR_RW(frequency);
DEVICE_ATTR_RW(hmode);
DEVICE_ATTR_RO(firmware_in_progress);

static struct attribute *unipi_spi2_attrs[] = {
    &dev_attr_messages.attr,
    &dev_attr_errors_tx.attr,
    &dev_attr_bytes.attr,
    &dev_attr_messages_prio.attr,
    &dev_attr_errors_crc1.attr,
    &dev_attr_errors_crc2.attr,
    &dev_attr_errors_opcode1.attr,
	NULL,
};
static const struct attribute_group unipi_spi2_group = {
	.name  = "statistics2",
	.attrs  = unipi_spi2_attrs,
};

static struct attribute *unipi_spi1_attrs[] = {
    &dev_attr_frequency.attr,
    &dev_attr_firmware_in_progress.attr,
    &dev_attr_hmode.attr,
	NULL,
};
static const struct attribute_group unipi_spi1_group = {
	.attrs  = unipi_spi1_attrs,
};

static const struct attribute_group *unipi_spi_groups[] = {
	&unipi_spi1_group,
	&unipi_spi2_group,
	NULL,
};


u16 unipi_spi_crc_set(u8* data, int len, u16 start)
{
	start = unipi_spi_crc(data, len, start);
	memcpy(data+len, &start, sizeof(start));
	return start;
}

/*  Async op for READREG */
int unipi_spi_read_regs_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	if (n_spi->hmode) {
		return unipi_spi2_read_simple(spi_dev, UNIPI_MODBUS_OP_READREG, reg, count, data, cb_data, cb_function);
	} else {
		return unipi_spi1_read_regs_async(spi_dev, reg, count, data, cb_data, cb_function);
	}
}

void unipi_spi_populated(void * self)
{
	struct spi_device *spi_dev = (struct spi_device*) self;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);
	if (spi_dev->max_speed_hz) n_spi->frequency = spi_dev->max_speed_hz;
}

struct unipi_mfd_op mfd_op_spi_v1 = {
	.async_idle = unipi_spi_idle_op_async,
	.reg_read_async = unipi_reg_read_async,
	.read_str_async = unipi_spi_read_str_async,
	.write_str_async = unipi_spi_write_str_async,
	.populated = unipi_spi_populated,
};

int unipi_spi_probe(struct spi_device *spi)
{
	struct unipi_spi_device *n_spi;
	u16  first_probe[8 /*UNIPISPI_PROBE_MESSAGE_LEN+*/];
	int ret, i, degraded=0; //, no_irq = 0;
	u32 probe_always_succeeds = 0;

	//unsigned long flags;

	n_spi = kzalloc(sizeof *n_spi, GFP_ATOMIC);
	if (!n_spi)
		return -ENOMEM;

	unipi_spi_trace(spi, "channel device probe start\n");

	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	spi->mode		    = spi->mode ? spi->mode : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? spi->max_speed_hz : 12000000;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,9,0)
	spi->rt = 1;
#endif
	ret = spi_setup(spi);
	if (ret) {
        kfree(n_spi);
		return ret;
    }

	//n_spi->spi = spi; // back link used by hrtimer
	spin_lock_init(&n_spi->firmware_lock);
	n_spi->firmware_in_progress = 0;
	n_spi->frequency = UNIPI_SPI_PROBE_FREQ;

	INIT_LIST_HEAD(&n_spi->queue);
	spin_lock_init(&n_spi->busy_lock);
	n_spi->busy = 0;

	hrtimer_init(&n_spi->frame_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	n_spi->frame_timer.function = unipi_spi_timer_func;
	n_spi->spi_dev = spi; // required by timer

	unipi_spi_trace(spi, "Max Hz controller=%d device=%d\n", spi->master->max_speed_hz, spi->max_speed_hz);
	if (spi->dev.of_node) {
		of_property_read_u32(spi->dev.of_node, "probe-always-succeeds", &(probe_always_succeeds));
		of_property_read_u32(spi->dev.of_node, "modbus-address", &(n_spi->mfd.modbus_address));
		if (n_spi->mfd.modbus_address > 254) {
			dev_warn(&spi->dev, "DT property modbus-address value (%d) invalid.\n", n_spi->mfd.modbus_address);
			n_spi->mfd.modbus_address = 0;
		} 
	}

	n_spi->mfd.self = spi;
	n_spi->mfd.op = &mfd_op_spi_v1;

	spi_set_drvdata(spi, n_spi);
	// We perform an initial probe of registers 1000-1004 to identify the device, using a premade message
	// Throw away the first message - the associated SPI Master is sometimes not properly initialised at this point
	unipi_spi_idle_op(spi/*, UNIPISPI_IDLE_MESSAGE, NULL, 0*/);
	first_probe[0] = 0;
	for (i=0; i< 5; i++) {
		int x= unipi_spi_regs_op(spi, UNIPI_MODBUS_OP_READREG, UNIPI_MFD_REG_FW_VERSION, 5, (u8*)first_probe);
		unipi_spi_trace_1(spi, "Probe %10ph ret=%d \n", first_probe, x);
		if (x==5)
			break;//, first_probe, UNIPISPI_PROBE_MESSAGE_LEN);
		//if (first_probe[0] != 0) break;
	}

	if (first_probe[0] == 0x0600) {
		degraded = 1;
		dev_warn(&spi->dev, "spi device is in degraded (bootloader) mode. Load operational firmware!\n");
	} else if (first_probe[0] != 0) {
		dev_info(&spi->dev, "spi channel is alive\n");
	} else if (probe_always_succeeds) {
		// dummy channel
		dev_info(&spi->dev, "spi channel is forced up\n");
	} else {
		ret = -ENODEV;
		spi_set_drvdata(spi, NULL);
		kfree(n_spi);
		dev_err(&spi->dev, "spi channel looks empty\n");
		return ret;
	}
	n_spi->frequency = UNIPI_SPI_SLOWER_FREQ;
	if (spi->max_speed_hz && (spi->max_speed_hz < UNIPI_SPI_SLOWER_FREQ)) 
		n_spi->frequency = spi->max_speed_hz;

	if (n_spi->mfd.modbus_address) {
		n_spi->chrdev = unipi_modbus_classdev_register(spi, n_spi->mfd.modbus_address);
	} else {
		dev_warn(&spi->dev, "Undefined modbus-address. Channel inaccessible via modbus chrdev.\n");
	}

	n_spi->mfd.irq = spi->irq;
/*
	n_spi->mfd.op.async_idle = unipi_spi_idle_op_async;
	n_spi->mfd.op.reg_read_async = unipi_reg_read_async;
	n_spi->mfd.op.read_str_async = unipi_spi_read_str_async;
	n_spi->mfd.op.write_str_async = unipi_spi_write_str_async;
	n_spi->mfd.op.populated = unipi_spi_populated;
*/
	n_spi->mfd.registers = devm_regmap_init_unipi_regs(spi, NULL);
	if (IS_ERR(n_spi->mfd.registers)) {
		return PTR_ERR(n_spi->mfd.registers);
	}
	n_spi->mfd.coils = devm_regmap_init_unipi_coils(spi, NULL);
	if (IS_ERR(n_spi->mfd.coils)) {
		return PTR_ERR(n_spi->mfd.coils);
	}
	ret = unipi_mfd_init(&n_spi->mfd, &spi->dev);

	return ret;
}

int unipi_spi_remove(struct spi_device *spi)
{
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);

	if (! n_spi) return 0;

	unipi_mfd_exit(&n_spi->mfd);
	if (n_spi->chrdev) {
		unipi_modbus_classdev_unregister(n_spi->chrdev);
	}

	kfree(n_spi);
	dev_info(&spi->dev, "spi channel removed\n");
	return 0;
}


/*********************
 * Final definitions *
 *********************/
static const struct spi_device_id unipi_spi_ids[] = {
	{ .name = "unipi-spi" },
	{ .name = "neuron" },
	{ .name = "axon" },
	{ .name = "unipispi" },
	{},
};
MODULE_DEVICE_TABLE(spi, unipi_spi_ids);

static const struct of_device_id unipi_spi_id_match[] = {
		{.compatible = "unipi,unipi-spi"},
		{.compatible = "unipi,neuron"},
		{.compatible = "unipi,axon"},
		{.compatible = "unipispi"},
		{}
};

MODULE_DEVICE_TABLE(of, unipi_spi_id_match);

struct spi_driver unipi_spi_driver =
{
	.driver =
	{
		.name			= "unipi-spi",
		.of_match_table	= of_match_ptr(unipi_spi_id_match),
		.dev_groups = unipi_spi_groups,
	},
	.probe				= unipi_spi_probe,
	.remove				= unipi_spi_remove,
	.id_table			= unipi_spi_ids,
};

MODULE_ALIAS("spi:unipispi");
MODULE_ALIAS("spi:unipi-spi");

struct unipi_mfd_device* unipi_spi_get_mfd(struct spi_device* spi_dev)
{
	struct unipi_spi_device *n_spi;
	//struct unipi_mfd_device *mfd;

	/* test if device is driven by this driver */
	if ((spi_dev==NULL) || (spi_dev->dev.driver==NULL) || (strcmp(unipi_spi_driver.driver.name, spi_dev->dev.driver->name) !=0 ))
		return NULL;

	n_spi = spi_get_drvdata(spi_dev);
	if (!n_spi) 
		return NULL;
	return &n_spi->mfd;
}
EXPORT_SYMBOL_GPL(unipi_spi_get_mfd);

static int __init unipi_spi_init(void)
{
	int ret = 0;

	ret = unipi_modbus_init();
	if (ret) return (ret);

	//spin_lock_init(&unipi_spi_master_lock);

	ret = spi_register_driver(&unipi_spi_driver);

	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to register spi driver --> %d\n", unipi_spi_driver.driver.name, ret);
		return ret;
	}
	printk(KERN_INFO "%s: Driver registered. Major Version: %s\n", unipi_spi_driver.driver.name, UNIPI_MODULE_MAJOR_VERSIONSTRING);
	return ret;
}

static void __exit unipi_spi_exit(void)
{
	spi_unregister_driver(&unipi_spi_driver);
	unipi_modbus_exit();
}

module_init(unipi_spi_init);
module_exit(unipi_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomas Knot <knot@faster.cz>");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Spi Channel Driver");
