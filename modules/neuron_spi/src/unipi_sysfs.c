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

#include "unipi_sysfs.h"


ssize_t neuronspi_show_model(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	if (neuronspi_model_id != -1) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].model_name);
	}
	return ret;
}

ssize_t neuronspi_show_eeprom(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	if (neuronspi_model_id != -1) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].eeprom_name);
	}
	return ret;
}

ssize_t neuronspi_spi_show_serial(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val[2] = {0, 0};
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_serial_num, val);
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_serial_num + 1, &(val[1]));
		ret = scnprintf(buf, 255, "%d\n", val[0]);
	}
	return ret;
}

ssize_t neuronspi_spi_show_hw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_hw_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x\n", (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

ssize_t neuronspi_spi_show_hw_flash_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_hw_flash_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x\n", (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

ssize_t neuronspi_spi_show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_sw_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x%x\n", (val & 0xF00) >> 8, (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

ssize_t neuronspi_spi_show_uart_queue_length(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->regstart_table->uart_queue_reg) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->uart_queue_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_show_uart_config(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->uart_conf_reg, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_store_uart_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->uart_conf_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_show_watchdog_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->wd_val_reg, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_store_watchdog_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->wd_val_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_show_watchdog_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->wd_timeout_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_store_watchdog_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->wd_timeout_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_show_pwm_presc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_ps_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_store_pwm_presc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_ps_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_show_pwm_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_c_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_store_pwm_freq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_c_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_show_pwm_cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_reg + n_do->do_index, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_store_pwm_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_reg + n_do->do_index, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_di_show_counter(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + (2 * n_di->di_index), &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_di_store_counter(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + (2 * n_di->di_index), val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_di_show_debounce(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_deboun_reg + n_di->di_index, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_di_store_debounce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_deboun_reg + n_di->di_index, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_di_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_val_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_do_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->do_count > n_do->do_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), &val);
		val &= 0x1 << (n_do->do_index % 15);
		val = val >> (n_do->do_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_do_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->do_count > n_do->do_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), &val);
		val &= ~(0x1 << (n_do->do_index % 15));
		val |= inp << (n_do->do_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_ro_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ro_count > n_ro->ro_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), &val);
		val &= 0x1 << (n_ro->ro_index % 15);
		val = val >> (n_ro->ro_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_ro_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ro_count > n_ro->ro_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), &val);
		val &= ~(0x1 << (n_ro->ro_index % 15));
		val |= inp << (n_ro->ro_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), val);
	}
err_end:
	return count;
}


ssize_t neuronspi_spi_gpio_show_ds_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_ds_toggle(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_ds_polarity(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_store_ds_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_store_ds_toggle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_gpio_store_ds_polarity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	int inp = 0;
	unsigned int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	err = kstrtoint(buf, 0, &inp);
	if (err < 0) goto err_end;
	if (inp > 1 || inp < 0) {
		goto err_end;
	}
	if (n_spi && n_spi->combination_id != -1 && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

ssize_t neuronspi_show_regmap(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->reg_map) {
		spin_lock(&n_spi->sysfs_regmap_lock);
		regmap_read(n_spi->reg_map, n_spi->sysfs_regmap_target, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
		spin_unlock(&n_spi->sysfs_regmap_lock);
	}
	return ret;
}

ssize_t neuronspi_store_regmap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->reg_map && val < 65536) {
		spin_lock(&n_spi->sysfs_regmap_lock);
		n_spi->sysfs_regmap_target = val;
		spin_unlock(&n_spi->sysfs_regmap_lock);
	}
err_end:
	return count;
}

ssize_t neuronspi_spi_show_board(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != -1 && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name);
	}
	return ret;
}

ssize_t neuronspi_spi_show_lboard_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != -1 && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%d\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->lower_board_id);
	}
	return ret;
}

ssize_t neuronspi_spi_show_uboard_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != -1 && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%d\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->upper_board_id);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_do_prefix(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi->features && n_spi->features->do_count > 0 && n_spi->do_driver) {
		ret = scnprintf(buf, 255, "%s_%d\n", n_spi->do_driver[n_do->do_index]->gpio_c.label, n_spi->neuron_index + 1);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_di_prefix(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi->features && n_spi->features->di_count > 0 && n_spi->di_driver) {
		ret = scnprintf(buf, 255, "%s_%d\n", n_spi->di_driver[n_di->di_index]->gpio_c.label, n_spi->neuron_index + 1);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_ro_prefix(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi->features && n_spi->features->ro_count > 0 && n_spi->ro_driver) {
		ret = scnprintf(buf, 255, "%s_%d\n", n_spi->ro_driver[n_ro->ro_index]->gpio_c.label, n_spi->neuron_index + 1);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_do_base(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi->features && n_spi->features->do_count > 0 && n_spi->do_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->do_driver[n_do->do_index]->gpio_c.base);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_di_base(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi->features && n_spi->features->di_count > 0 && n_spi->di_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->di_driver[n_di->di_index]->gpio_c.base);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_ro_base(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi->features && n_spi->features->ro_count > 0 && n_spi->ro_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->ro_driver[n_ro->ro_index]->gpio_c.base);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_do_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi->features && n_spi->features->do_count > 0 && n_spi->do_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->do_driver[n_do->do_index]->gpio_c.ngpio);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_di_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi->features && n_spi->features->di_count > 0 && n_spi->di_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->di_driver[n_di->di_index]->gpio_c.ngpio);
	}
	return ret;
}

ssize_t neuronspi_spi_gpio_show_ro_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi->features && n_spi->features->ro_count > 0 && n_spi->ro_driver) {
		ret = scnprintf(buf, 255, "%d\n", n_spi->ro_driver[n_ro->ro_index]->gpio_c.ngpio);
	}
	return ret;
}


ssize_t neuronspi_iio_show_primary_ai_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}

ssize_t neuronspi_iio_store_primary_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_iio_show_primary_ao_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}

ssize_t neuronspi_iio_store_primary_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_stm_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg, val);
	}
err_end:
	return count;
}

ssize_t neuronspi_iio_show_secondary_ai_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}
ssize_t neuronspi_iio_store_secondary_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ai_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, val);
	}
err_end:
	return count;
}
ssize_t neuronspi_iio_show_secondary_ao_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ao_mode_reg + ao_data->index, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}
ssize_t neuronspi_iio_store_secondary_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_sec_ao_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ao_mode_reg + ao_data->index, val);
	}
err_end:
	return count;
}
