/*
 * UniPi Neuron tty serial driver - Copyright (C) 2018 UniPi Technologies
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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_SYSFS_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_SYSFS_H_

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <uapi/linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/spi/spi.h>
#include <linux/leds.h>
#include <linux/uaccess.h>
#include <asm/termbits.h>
#include <asm/gpio.h>

#include "unipi_common.h"
#include "unipi_platform.h"

ssize_t neuronspi_show_model(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_show_eeprom(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_show_regmap(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_store_regmap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_show_serial(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_board(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_lboard_id(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_uboard_id(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_hw_version(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_hw_flash_version(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_fw_version(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_uart_queue_length(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_show_uart_config(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_store_uart_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_show_watchdog_status(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_store_watchdog_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_show_watchdog_timeout(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_store_watchdog_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_show_pwm_freq(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_store_pwm_freq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_di_show_counter(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_di_store_counter(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_di_show_debounce(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_di_store_debounce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_show_pwm_cycle(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_store_pwm_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_show_pwm_presc(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_store_pwm_presc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_store_ds_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_store_ds_toggle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_store_ds_polarity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_show_ds_enable(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_ds_toggle(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_ds_polarity(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_do_prefix(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_do_base(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_do_count(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_di_prefix(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_di_base(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_di_count(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_ro_prefix(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_ro_base(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_show_ro_count(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_di_show_value(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_do_show_value(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_do_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_spi_gpio_ro_show_value(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_spi_gpio_ro_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_iio_show_primary_ai_mode(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_iio_store_primary_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_iio_show_primary_ao_mode(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_iio_store_primary_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_iio_show_secondary_ai_mode(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_iio_store_secondary_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t neuronspi_iio_show_secondary_ao_mode(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t neuronspi_iio_store_secondary_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(model_name, 0440, neuronspi_show_model, NULL);
static DEVICE_ATTR(sys_eeprom_name, 0440, neuronspi_show_eeprom, NULL);
static DEVICE_ATTR(register_read, 0660, neuronspi_show_regmap, neuronspi_store_regmap);
static DEVICE_ATTR(sys_board_serial, 0440, neuronspi_spi_show_serial, NULL);
static DEVICE_ATTR(sys_board_name, 0440, neuronspi_spi_show_board, NULL);
static DEVICE_ATTR(sys_primary_major_id, 0440, neuronspi_spi_show_lboard_id, NULL);
static DEVICE_ATTR(sys_secondary_major_id, 0440, neuronspi_spi_show_uboard_id, NULL);
static DEVICE_ATTR(sys_primary_minor_id, 0440, neuronspi_spi_show_hw_version, NULL);
static DEVICE_ATTR(sys_secondary_minor_id, 0440, neuronspi_spi_show_hw_flash_version, NULL);
static DEVICE_ATTR(firmware_version, 0440, neuronspi_spi_show_fw_version, NULL);
static DEVICE_ATTR(watchdog_status, 0660, neuronspi_spi_show_watchdog_status, neuronspi_spi_store_watchdog_status);
static DEVICE_ATTR(watchdog_timeout, 0660, neuronspi_spi_show_watchdog_timeout, neuronspi_spi_store_watchdog_timeout);
static DEVICE_ATTR(sys_gpio_do_count, 0440, neuronspi_spi_gpio_show_do_count, NULL);
static DEVICE_ATTR(sys_gpio_do_prefix, 0440, neuronspi_spi_gpio_show_do_prefix, NULL);
static DEVICE_ATTR(sys_gpio_do_base, 0440, neuronspi_spi_gpio_show_do_base, NULL);
static DEVICE_ATTR(sys_gpio_di_count, 0440, neuronspi_spi_gpio_show_di_count, NULL);
static DEVICE_ATTR(sys_gpio_di_prefix, 0440, neuronspi_spi_gpio_show_di_prefix, NULL);
static DEVICE_ATTR(ro_value, 0660, neuronspi_spi_gpio_ro_show_value, neuronspi_spi_gpio_ro_store_value);
static DEVICE_ATTR(do_value, 0660, neuronspi_spi_gpio_do_show_value, neuronspi_spi_gpio_do_store_value);
static DEVICE_ATTR(counter, 0660, neuronspi_spi_gpio_di_show_counter, neuronspi_spi_gpio_di_store_counter);
static DEVICE_ATTR(debounce, 0660, neuronspi_spi_gpio_di_show_debounce, neuronspi_spi_gpio_di_store_debounce);
static DEVICE_ATTR(di_value, 0440, neuronspi_spi_gpio_di_show_value, NULL);
static DEVICE_ATTR(direct_switch_enable, 0660, neuronspi_spi_gpio_show_ds_enable, neuronspi_spi_gpio_store_ds_enable);
static DEVICE_ATTR(direct_switch_toggle, 0660, neuronspi_spi_gpio_show_ds_toggle, neuronspi_spi_gpio_store_ds_toggle);
static DEVICE_ATTR(direct_switch_polarity, 0660, neuronspi_spi_gpio_show_ds_polarity, neuronspi_spi_gpio_store_ds_polarity);
static DEVICE_ATTR(pwm_frequency_cycle, 0660, neuronspi_spi_gpio_show_pwm_freq, neuronspi_spi_gpio_store_pwm_freq);
static DEVICE_ATTR(pwm_prescale, 0660, neuronspi_spi_gpio_show_pwm_presc, neuronspi_spi_gpio_store_pwm_presc);
static DEVICE_ATTR(pwm_duty_cycle, 0660, neuronspi_spi_gpio_show_pwm_cycle, neuronspi_spi_gpio_store_pwm_cycle);
static DEVICE_ATTR(uart_queue_length, 0440, neuronspi_spi_show_uart_queue_length, NULL);
static DEVICE_ATTR(uart_config, 0660, neuronspi_spi_show_uart_config, neuronspi_spi_store_uart_config);
static DEVICE_ATTR(sys_gpio_di_base, 0440, neuronspi_spi_gpio_show_di_base, NULL);
static DEVICE_ATTR(sys_gpio_ro_count, 0440, neuronspi_spi_gpio_show_ro_count, NULL);
static DEVICE_ATTR(sys_gpio_ro_prefix, 0440, neuronspi_spi_gpio_show_ro_prefix, NULL);
static DEVICE_ATTR(sys_gpio_ro_base, 0440, neuronspi_spi_gpio_show_ro_base, NULL);
static DEVICE_ATTR(mode_ai_type_a, 0660, neuronspi_iio_show_primary_ai_mode, neuronspi_iio_store_primary_ai_mode);
static DEVICE_ATTR(mode_ao_type_a, 0660, neuronspi_iio_show_primary_ao_mode, neuronspi_iio_store_primary_ao_mode);
static DEVICE_ATTR(mode_ai_type_b, 0660, neuronspi_iio_show_secondary_ai_mode, neuronspi_iio_store_secondary_ai_mode);
static DEVICE_ATTR(mode_ao_type_b, 0660, neuronspi_iio_show_secondary_ao_mode, neuronspi_iio_store_secondary_ao_mode);

static struct attribute *neuron_plc_attrs[] = {
		&dev_attr_model_name.attr,
		&dev_attr_sys_eeprom_name.attr,
		NULL,
};

static struct attribute *neuron_board_attrs[] = {
		&dev_attr_sys_board_name.attr,
		&dev_attr_sys_primary_major_id.attr,
		&dev_attr_sys_secondary_major_id.attr,
		&dev_attr_sys_primary_minor_id.attr,
		&dev_attr_sys_secondary_minor_id.attr,
		&dev_attr_firmware_version.attr,
		&dev_attr_watchdog_status.attr,
		&dev_attr_watchdog_timeout.attr,
		&dev_attr_sys_board_serial.attr,
		&dev_attr_uart_queue_length.attr,
		&dev_attr_uart_config.attr,
		&dev_attr_register_read.attr,
		NULL,
};

static struct attribute *neuron_gpio_di_attrs[] = {
		&dev_attr_sys_gpio_di_count.attr,
		&dev_attr_sys_gpio_di_prefix.attr,
		&dev_attr_sys_gpio_di_base.attr,
		&dev_attr_direct_switch_enable.attr,
		&dev_attr_direct_switch_toggle.attr,
		&dev_attr_direct_switch_polarity.attr,
		&dev_attr_di_value.attr,
		&dev_attr_counter.attr,
		&dev_attr_debounce.attr,
		NULL,
};

static struct attribute *neuron_gpio_do_attrs[] = {
		&dev_attr_sys_gpio_do_count.attr,
		&dev_attr_sys_gpio_do_prefix.attr,
		&dev_attr_sys_gpio_do_base.attr,
		&dev_attr_pwm_frequency_cycle.attr,
		&dev_attr_pwm_prescale.attr,
		&dev_attr_pwm_duty_cycle.attr,
		&dev_attr_do_value.attr,
		NULL,
};

static struct attribute *neuron_gpio_ro_attrs[] = {
		&dev_attr_sys_gpio_ro_count.attr,
		&dev_attr_sys_gpio_ro_prefix.attr,
		&dev_attr_sys_gpio_ro_base.attr,
		&dev_attr_ro_value.attr,
		NULL,
};

static struct attribute *neuron_stm_ai_attrs[] = {
		&dev_attr_mode_ai_type_a.attr,
		NULL,
};

static struct attribute *neuron_stm_ao_attrs[] = {
		&dev_attr_mode_ao_type_a.attr,
		NULL,
};

static struct attribute *neuron_sec_ai_attrs[] = {
		&dev_attr_mode_ai_type_b.attr,
		NULL,
};

static struct attribute *neuron_sec_ao_attrs[] = {
		&dev_attr_mode_ao_type_b.attr,
		NULL,
};

static struct attribute_group neuron_plc_attr_group = {
	.attrs = neuron_plc_attrs,
};

static struct attribute_group neuron_board_attr_group = {
	.attrs = neuron_board_attrs,
};

static struct attribute_group neuron_gpio_di_attr_group = {
	.attrs = neuron_gpio_di_attrs,
};

static struct attribute_group neuron_gpio_do_attr_group = {
	.attrs = neuron_gpio_do_attrs,
};

static struct attribute_group neuron_gpio_ro_attr_group = {
	.attrs = neuron_gpio_ro_attrs,
};

static struct attribute_group neuron_stm_ai_group = {
	.attrs = neuron_stm_ai_attrs,
};

static struct attribute_group neuron_stm_ao_group = {
	.attrs = neuron_stm_ao_attrs,
};

static struct attribute_group neuron_sec_ai_group = {
	.attrs = neuron_sec_ai_attrs,
};

static struct attribute_group neuron_sec_ao_group = {
	.attrs = neuron_sec_ao_attrs,
};

static const struct attribute_group *neuron_plc_attr_groups[] = {
	&neuron_plc_attr_group,
	NULL,
};

static const struct attribute_group *neuron_board_attr_groups[] = {
	&neuron_board_attr_group,
	NULL,
};

static const struct attribute_group *neuron_gpio_di_attr_groups[] = {
	&neuron_gpio_di_attr_group,
	NULL,
};

static const struct attribute_group *neuron_gpio_do_attr_groups[] = {
	&neuron_gpio_do_attr_group,
	NULL,
};

static const struct attribute_group *neuron_gpio_ro_attr_groups[] = {
	&neuron_gpio_ro_attr_group,
	NULL,
};

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_SYSFS_H_ */
