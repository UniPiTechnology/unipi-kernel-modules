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

/************
 * Includes *
 ************/

#include "unipi_sysfs.h"
#include "unipi_spi.h"

/*********************
 * Data Declarations *
 *********************/

int unipi_use_custom_speed = 0;
u32 unipi_custom_speed_value = NEURONSPI_DEFAULT_SYSFS_SPEED;

/************************
 * Static Functions *
 ************************/

static ssize_t neuronspi_show_model(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	if (neuronspi_model_id != -1) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].model_name);
	}
	return ret;
}

static ssize_t neuronspi_show_sysfs_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	mutex_lock(&unipi_inv_speed_mutex);
	ret = scnprintf(buf, 255, "%d Hz\n", unipi_custom_speed_value);
	mutex_unlock(&unipi_inv_speed_mutex);
	return ret;
}

static ssize_t neuronspi_store_sysfs_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (val == NEURONSPI_DEFAULT_SYSFS_SPEED) {
		unipi_use_custom_speed = 0;
		mutex_lock(&unipi_inv_speed_mutex);
		unipi_custom_speed_value = 50;
		mutex_unlock(&unipi_inv_speed_mutex);
	} else if (val > 0) {
		unipi_use_custom_speed = 1;
		mutex_lock(&unipi_inv_speed_mutex);
		unipi_custom_speed_value = val > 1000 ? 1000 : val;
		mutex_unlock(&unipi_inv_speed_mutex);
	}
err_end:
	return count;
}

static ssize_t neuronspi_show_driver_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
#ifdef NEURONSPI_MAJOR_VERSIONSTRING
	ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MAJOR_VERSIONSTRING);
#else
	ret = scnprintf(buf, 255, "%s\n", "Unspecified Driver Version");
#endif
	return ret;
}

static ssize_t neuronspi_show_eeprom(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	if (neuronspi_model_id != -1) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_MODELTABLE[neuronspi_model_id].eeprom_name);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_serial(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val[2] = {0, 0};
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_serial_num, val);
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_serial_num + 1, &(val[1]));
		ret = scnprintf(buf, 255, "%d\n", val[0]);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_hw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_hw_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x\n", (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_hw_flash_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_hw_flash_ver, &val);
		ret = scnprintf(buf, 255, "%x.%x\n", (val & 0xF0) >> 4, val & 0xF);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->sys_sw_ver, &val);
		ret = scnprintf(buf, 255, "%x.%d\n", (val & 0xF00) >> 8, (int)(val & 0xFF));
	}
	return ret;
}

static ssize_t neuronspi_spi_show_uart_queue_length(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_queue_reg) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->uart_queue_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_uart_config(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->uart_conf_reg, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_uart_config(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->uart_conf_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_uart_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	u8 *inp_buf, *outp_buf;
	int read_length;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	printk(KERN_INFO "NEURONSPI: Index %d\n", n_spi->neuron_index);
	spi = neuronspi_s_dev[n_spi->neuron_index];
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		read_length = neuronspi_spi_compose_single_register_read(504, &inp_buf, &outp_buf);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, read_length, n_spi->ideal_frequency, 25, 1, 0);
		val = outp_buf[11];
		memcpy(&val, &outp_buf[NEURONSPI_HEADER_LENGTH], sizeof(u16));
		kfree(inp_buf);
		kfree(outp_buf);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_uart_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	u8 *inp_buf, *outp_buf;
	int write_length;
	struct spi_device *spi;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	printk(KERN_INFO "NEURONSPI: Index %d\n", n_spi->neuron_index);
	spi = neuronspi_s_dev[n_spi->neuron_index];
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->regstart_table->uart_conf_reg) {
		write_length = neuronspi_spi_compose_single_register_write(504, &inp_buf, &outp_buf, val);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 25, 1, 0);
		kfree(inp_buf);
		kfree(outp_buf);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_watchdog_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->wd_val_reg, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_watchdog_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->wd_val_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_watchdog_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->wd_timeout_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_watchdog_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->wd_timeout_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_show_pwm_presc(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_ps_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_pwm_presc(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_ps_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_show_pwm_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_c_reg, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_pwm_freq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_c_reg, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_show_pwm_cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_pwm_reg + n_do->do_index, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_store_pwm_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_pwm_reg + n_do->do_index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_di_show_counter(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	u32 val_upper = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + (2 * n_di->di_index), &val);
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + 1 + (2 * n_di->di_index), &val_upper);
		val |= val_upper << 16;
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_di_store_counter(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + (2 * n_di->di_index), val & 0xFFFF);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_counter_reg + 1 + (2 * n_di->di_index), val >> 16);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_di_show_debounce(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	u32 val = 0;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_deboun_reg + n_di->di_index, &val);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_di_store_debounce(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_deboun_reg + n_di->di_index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_di_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->di_count > n_di->di_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_val_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_do_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_do_driver *n_do;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_do = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_do->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->do_count > n_do->do_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), &val);
		val &= 0x1 << (n_do->do_index % 15);
		val = val >> (n_do->do_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_do_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->do_count > n_do->do_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), &val);
		val &= ~(0x1 << (n_do->do_index % 15));
		val |= inp << (n_do->do_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->do_val_reg + (n_do->do_index / 16), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_ro_show_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_ro_driver *n_ro;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_ro = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_ro->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->ro_count > n_ro->ro_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), &val);
		val &= 0x1 << (n_ro->ro_index % 15);
		val = val >> (n_ro->ro_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_ro_store_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->ro_count > n_ro->ro_index) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), &val);
		val &= ~(0x1 << (n_ro->ro_index % 15));
		val |= inp << (n_ro->ro_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->ro_val_reg + (n_ro->ro_index / 16), val);
	}
err_end:
	return count;
}


static ssize_t neuronspi_spi_gpio_show_ds_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_direct_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ds_toggle(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int val;
	struct neuronspi_di_driver *n_di;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_di = platform_get_drvdata(plat);
	n_spi = spi_get_drvdata(n_di->spi);
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), &val);
		val &= 0x1 << (n_di->di_index % 15);
		val = val >> (n_di->di_index % 15);
		ret = scnprintf(buf, 255, "%d\n", val);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_ds_polarity(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_store_ds_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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

static ssize_t neuronspi_spi_gpio_store_ds_toggle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_toggle_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_gpio_store_ds_polarity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->features && n_spi->features->ds_count) {
		regmap_read(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), &val);
		val &= ~(0x1 << (n_di->di_index % 15));
		val |= inp << (n_di->di_index % 15);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->di_polar_reg + (n_di->di_index / 16), val);
	}
err_end:
	return count;
}


static ssize_t neuronspi_spi_show_register(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 *inp_buf, *outp_buf;
	int read_length;
	ssize_t ret = 0;
	u16 val = 0;
	unsigned long flags;
	struct neuronspi_driver_data *n_spi;
	struct spi_device *spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	spi = neuronspi_s_dev[n_spi->neuron_index];
	if (n_spi && n_spi->reg_map) {
		spin_lock_irqsave(&n_spi->sysfs_regmap_lock, flags);
		read_length = neuronspi_spi_compose_single_register_read(n_spi->sysfs_register_target, &inp_buf, &outp_buf);
		spin_unlock_irqrestore(&n_spi->sysfs_regmap_lock, flags);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, read_length, n_spi->ideal_frequency, 125, 1, 0);
		memcpy(&val, &outp_buf[NEURONSPI_HEADER_LENGTH], sizeof(u16));
		ret = scnprintf(buf, 255, "%x\n", (u32)val);
	}
	return ret;
}

static ssize_t neuronspi_spi_store_register(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	unsigned long flags;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->reg_map && val < 65536) {
		spin_lock_irqsave(&n_spi->sysfs_regmap_lock, flags);
		n_spi->sysfs_register_target = val;
		spin_unlock_irqrestore(&n_spi->sysfs_regmap_lock, flags);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_store_register_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 *inp_buf, *outp_buf;
	int write_length;
	ssize_t err = 0;
	unsigned int val = 0;
	unsigned long flags;
	struct neuronspi_driver_data *n_spi;
	struct spi_device *spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	spi = neuronspi_s_dev[n_spi->neuron_index];
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->reg_map && val < 65536) {
		spin_lock_irqsave(&n_spi->sysfs_regmap_lock, flags);
		write_length = neuronspi_spi_compose_single_register_read(n_spi->sysfs_register_target, &inp_buf, &outp_buf);
		spin_unlock_irqrestore(&n_spi->sysfs_regmap_lock, flags);
		neuronspi_spi_send_message(spi, inp_buf, outp_buf, write_length, n_spi->ideal_frequency, 125, 1, 0);
	}
err_end:
	return count;
}



static ssize_t neuronspi_show_regmap(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	unsigned int target = 0;
	u32 val = 0;
	unsigned long flags;
	struct neuronspi_driver_data *n_spi;
	struct spi_device *spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	spi = neuronspi_s_dev[n_spi->neuron_index];
	if (n_spi && n_spi->reg_map) {
		spin_lock_irqsave(&n_spi->sysfs_regmap_lock, flags);
		target = n_spi->sysfs_regmap_target;
		spin_unlock_irqrestore(&n_spi->sysfs_regmap_lock, flags);
		regmap_read(n_spi->reg_map, target, &val);
		ret = scnprintf(buf, 255, "%x\n", val);
	}
	return ret;
}

static ssize_t neuronspi_store_regmap(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	unsigned long flags;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->reg_map && val < 65536) {
		spin_lock_irqsave(&n_spi->sysfs_regmap_lock, flags);
		n_spi->sysfs_regmap_target = val;
		spin_unlock_irqrestore(&n_spi->sysfs_regmap_lock, flags);
	}
err_end:
	return count;
}

static ssize_t neuronspi_store_regmap_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	unsigned int target = 0;
	unsigned long flags;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->reg_map && val < 65536) {
		spin_lock_irqsave(&n_spi->sysfs_regmap_lock, flags);
		target = n_spi->sysfs_regmap_target;
		spin_unlock_irqrestore(&n_spi->sysfs_regmap_lock, flags);
		regmap_write(n_spi->reg_map, n_spi->sysfs_regmap_target, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_spi_show_board(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != 0xFF && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%s\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->combination_name);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_lboard_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != 0xFF && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%d\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->lower_board_id);
	}
	return ret;
}

static ssize_t neuronspi_spi_show_uboard_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct neuronspi_driver_data *n_spi;
	struct platform_device *plat = to_platform_device(dev);
	n_spi = platform_get_drvdata(plat);
	if (n_spi->combination_id != 0xFF && n_spi->combination_id < NEURONSPI_BOARDTABLE_LEN) {
		ret = scnprintf(buf, 255, "%d\n", NEURONSPI_BOARDTABLE[n_spi->combination_id].definition->upper_board_id);
	}
	return ret;
}

static ssize_t neuronspi_spi_gpio_show_do_prefix(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_di_prefix(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_ro_prefix(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_do_base(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_di_base(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_ro_base(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_do_count(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_di_count(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t neuronspi_spi_gpio_show_ro_count(struct device *dev, struct device_attribute *attr, char *buf)
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


static ssize_t neuronspi_iio_show_stm_ai_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg + ai_data->index, &val);

	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}

static ssize_t neuronspi_iio_store_stm_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ai_mode_reg + ai_data->index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_iio_show_stm_ao_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);

	regmap_read(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg + ao_data->index, &val);
	printk(KERN_INFO "NEURONSPI: Mode register %d set to %x", n_spi->regstart_table->stm_ao_mode_reg + ao_data->index, val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}

static ssize_t neuronspi_iio_store_stm_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != -1 && n_spi->reg_map) {
		printk(KERN_INFO "NEURONSPI: Mode register %d set to %x", n_spi->regstart_table->stm_ao_mode_reg + ao_data->index, val);
		regmap_write(n_spi->reg_map, n_spi->regstart_table->stm_ao_mode_reg + ao_data->index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_iio_show_external_ai_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}

static ssize_t neuronspi_iio_store_external_ai_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ai_data = iio_priv(indio_dev);
	struct spi_device *spi = ai_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ai_mode_reg + ai_data->index, val);
	}
err_end:
	return count;
}

static ssize_t neuronspi_iio_show_external_ao_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	regmap_read(n_spi->reg_map, n_spi->regstart_table->sec_ao_mode_reg + ao_data->index, &val);
	ret = scnprintf(buf, 255, "%d\n", val);
	return ret;
}

static ssize_t neuronspi_iio_store_external_ao_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t err = 0;
	unsigned int val = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct neuronspi_analog_data *ao_data = iio_priv(indio_dev);
	struct spi_device *spi = ao_data->parent;
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	err = kstrtouint(buf, 0, &val);
	if (err < 0) goto err_end;
	if (n_spi && n_spi->combination_id != 0xFF && n_spi->reg_map) {
		regmap_write(n_spi->reg_map, n_spi->regstart_table->sec_ao_mode_reg + ao_data->index, val);
	}
err_end:
	return count;
}

/**********************************
 * Function Structure Definitions *
 **********************************/

static DEVICE_ATTR(model_name, 0444, neuronspi_show_model, NULL);
static DEVICE_ATTR(sys_reading_freq, 0660, neuronspi_show_sysfs_speed, neuronspi_store_sysfs_speed);
static DEVICE_ATTR(sys_eeprom_name, 0444, neuronspi_show_eeprom, NULL);
static DEVICE_ATTR(driver_version, 0444, neuronspi_show_driver_version, NULL);
static DEVICE_ATTR(register_read, 0660, neuronspi_spi_show_register, neuronspi_spi_store_register);
static DEVICE_ATTR(register_set, 0220, NULL, neuronspi_spi_store_register_value);
static DEVICE_ATTR(regmap_read, 0660, neuronspi_show_regmap, neuronspi_store_regmap);
static DEVICE_ATTR(regmap_set, 0220, NULL, neuronspi_store_regmap_value);
static DEVICE_ATTR(sys_board_serial, 0440, neuronspi_spi_show_serial, NULL);
static DEVICE_ATTR(sys_board_name, 0444, neuronspi_spi_show_board, NULL);
static DEVICE_ATTR(sys_primary_major_id, 0444, neuronspi_spi_show_lboard_id, NULL);
static DEVICE_ATTR(sys_secondary_major_id, 0444, neuronspi_spi_show_uboard_id, NULL);
static DEVICE_ATTR(sys_primary_minor_id, 0444, neuronspi_spi_show_hw_version, NULL);
static DEVICE_ATTR(sys_secondary_minor_id, 0444, neuronspi_spi_show_hw_flash_version, NULL);
static DEVICE_ATTR(firmware_version, 0444, neuronspi_spi_show_fw_version, NULL);
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
static DEVICE_ATTR(uart_config, 0664, neuronspi_spi_show_uart_config, neuronspi_spi_store_uart_config);
static DEVICE_ATTR(uart_timeout, 0664, neuronspi_spi_show_uart_timeout, neuronspi_spi_store_uart_timeout);
static DEVICE_ATTR(sys_gpio_di_base, 0440, neuronspi_spi_gpio_show_di_base, NULL);
static DEVICE_ATTR(sys_gpio_ro_count, 0440, neuronspi_spi_gpio_show_ro_count, NULL);
static DEVICE_ATTR(sys_gpio_ro_prefix, 0440, neuronspi_spi_gpio_show_ro_prefix, NULL);
static DEVICE_ATTR(sys_gpio_ro_base, 0440, neuronspi_spi_gpio_show_ro_base, NULL);
static DEVICE_ATTR(mode_ai_voltage_current, 0660, neuronspi_iio_show_stm_ai_mode, neuronspi_iio_store_stm_ai_mode);
static DEVICE_ATTR(mode_ao_voltage_current_resistance, 0660, neuronspi_iio_show_stm_ao_mode, neuronspi_iio_store_stm_ao_mode);
static DEVICE_ATTR(mode_ai_voltage_current_resistance, 0660, neuronspi_iio_show_external_ai_mode, neuronspi_iio_store_external_ai_mode);
//static DEVICE_ATTR(mode_ao_voltage, 0660, neuronspi_iio_show_external_ao_mode, neuronspi_iio_store_external_ao_mode);

static struct attribute *neuron_plc_attrs[] = {
		&dev_attr_model_name.attr,
		&dev_attr_sys_reading_freq.attr,
		&dev_attr_sys_eeprom_name.attr,
		&dev_attr_driver_version.attr,
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
		&dev_attr_uart_timeout.attr,
		&dev_attr_register_read.attr,
		&dev_attr_register_set.attr,
		&dev_attr_regmap_read.attr,
		&dev_attr_regmap_set.attr,
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
		&dev_attr_mode_ai_voltage_current.attr,
		NULL,
};

static struct attribute *neuron_stm_ao_attrs[] = {
		&dev_attr_mode_ao_voltage_current_resistance.attr,
		NULL,
};

static struct attribute *neuron_sec_ai_attrs[] = {
		&dev_attr_mode_ai_voltage_current_resistance.attr,
		NULL,
};

/*
 * static struct attribute *neuron_sec_ao_attrs[] = {
		&dev_attr_mode_ao_voltage.attr,
		NULL,
};
*/

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

const struct attribute_group neuron_stm_ai_group = {
	.attrs = neuron_stm_ai_attrs,
};

const struct attribute_group neuron_stm_ao_group = {
	.attrs = neuron_stm_ao_attrs,
};

const struct attribute_group neuron_sec_ai_group = {
	.attrs = neuron_sec_ai_attrs,
};

/*
const struct attribute_group neuron_sec_ao_group = {
	.attrs = neuron_sec_ao_attrs,
};
*/

const struct attribute_group *neuron_plc_attr_groups[] = {
	&neuron_plc_attr_group,
	NULL,
};

const struct attribute_group *neuron_board_attr_groups[] = {
	&neuron_board_attr_group,
	NULL,
};

const struct attribute_group *neuron_gpio_di_attr_groups[] = {
	&neuron_gpio_di_attr_group,
	NULL,
};

const struct attribute_group *neuron_gpio_do_attr_groups[] = {
	&neuron_gpio_do_attr_group,
	NULL,
};

const struct attribute_group *neuron_gpio_ro_attr_groups[] = {
	&neuron_gpio_ro_attr_group,
	NULL,
};
