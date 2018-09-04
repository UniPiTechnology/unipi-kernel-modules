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

#include "unipi_gpio.h"
#include "unipi_spi.h"

/************************
 * Non-static Functions *
 ************************/

int neuronspi_spi_gpio_di_get(struct spi_device* spi_dev, u32 id)
{
	u8 *recv_buf;
	bool ret = 0;
	u32 offset = id / 16;
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	recv_buf = kzalloc(4, GFP_ATOMIC);
	regmap_read(d_data->reg_map, d_data->regstart_table->di_val_reg + offset, (void*)recv_buf);
	if (*recv_buf & (0x1 << offset)) {
		ret = 1;
	}
	kfree(recv_buf);
	return ret;
}

int neuronspi_spi_gpio_do_set(struct spi_device* spi_dev, u32 id, int value)
{
	u32 current_value = 0;
	bool ret = 0;
	u32 offset = id / 16;
	u16 off_val = value << (id % 16);
	u16 mask = ~(1 << (id % 16));
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	regmap_read(d_data->reg_map, d_data->regstart_table->do_val_reg + offset, &current_value);
	current_value&= mask;
	current_value|= off_val;
	regmap_write(d_data->reg_map, d_data->regstart_table->do_val_reg + offset, current_value);
	return ret;
}

int neuronspi_spi_gpio_ro_set(struct spi_device* spi_dev, u32 id, int value)
{
	u32 current_value = 0;
	bool ret = 0;
	u32 offset = id / 16;
	u16 off_val = value << (id % 16);
	u16 mask = ~(1 << (id % 16));
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
	regmap_read(d_data->reg_map, d_data->regstart_table->ro_val_reg + offset, &current_value);
	current_value&= mask;
	current_value|= off_val;
	regmap_write(d_data->reg_map, d_data->regstart_table->ro_val_reg + offset, current_value);
	return ret;
}


int neuronspi_gpio_di_direction_input(struct gpio_chip *chip, unsigned offset) {
	return 0;
}

int neuronspi_gpio_di_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return -EINVAL;
}

int neuronspi_gpio_di_get(struct gpio_chip *chip, unsigned offset) {
	struct neuronspi_gpio_driver *n_di = gpiochip_get_data(chip);
	struct spi_device *spi = n_di->spi;
	return neuronspi_spi_gpio_di_get(spi, n_di->io_index);
}

int neuronspi_gpio_do_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return 0;
}

void neuronspi_gpio_do_set(struct gpio_chip *chip, unsigned offset, int value) {
	struct neuronspi_gpio_driver *n_do = gpiochip_get_data(chip);
	struct spi_device *spi = n_do->spi;
	neuronspi_spi_gpio_do_set(spi, n_do->io_index, value);
}

int neuronspi_gpio_ro_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return 0;
}

void neuronspi_gpio_ro_set(struct gpio_chip *chip, unsigned offset, int value) {
	struct neuronspi_gpio_driver *n_ro = gpiochip_get_data(chip);
	struct spi_device *spi = n_ro->spi;
	neuronspi_spi_gpio_ro_set(spi, n_ro->io_index, value);
}


struct neuronspi_gpio_driver * neuronspi_di_probe(int di_count, int neuron_index, struct platform_device *board_device)
{
	struct neuronspi_gpio_driver* di_driver_arr;
	struct neuronspi_gpio_driver* pdi_driver;
	int i;
    char buf[20];

	if (di_count <= 0) return NULL;

	di_driver_arr = kzalloc(sizeof(struct neuronspi_gpio_driver) * di_count, GFP_ATOMIC);
	for (i = 0; i < di_count; i++) {
		pdi_driver = di_driver_arr + i;
        scnprintf(buf, 20, "di_%d_%02d", neuron_index+1, i+1);
		/*strcpy(pdi_driver->name, "di_0_00");
		pdi_driver->name[3] = neuron_index + '1';
		pdi_driver->name[5] = ((i + 1) / 10) + '0';
		pdi_driver->name[6] = ((i + 1) % 10) + '0';*/
		pdi_driver->io_index = i;
		pdi_driver->spi = neuronspi_s_dev[neuron_index];
				
		pdi_driver->plat_dev = platform_device_alloc(buf, -1);
		pdi_driver->plat_dev->dev.parent = &(board_device->dev);
		pdi_driver->plat_dev->dev.groups = neuron_gpio_di_attr_groups;
		pdi_driver->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
		platform_device_add(pdi_driver->plat_dev);

		platform_set_drvdata(pdi_driver->plat_dev, pdi_driver);
		pdi_driver->gpio_c.owner = THIS_MODULE;
		pdi_driver->gpio_c.parent = &(pdi_driver->plat_dev->dev);
		pdi_driver->gpio_c.label = "neuron_di";
		pdi_driver->gpio_c.can_sleep = 1;
		pdi_driver->gpio_c.ngpio = 1;
		pdi_driver->gpio_c.base = -1;
		pdi_driver->gpio_c.direction_input = neuronspi_gpio_di_direction_input;
		pdi_driver->gpio_c.get = neuronspi_gpio_di_get;
		gpiochip_add_data(&pdi_driver->gpio_c, pdi_driver);
	}  
	return di_driver_arr;
}

struct neuronspi_gpio_driver * neuronspi_ro_probe(int ro_count, int neuron_index, struct platform_device *board_device)
{
	struct neuronspi_gpio_driver* ro_driver_arr;
	struct neuronspi_gpio_driver* pro_driver;
	int i;
    char buf[20];

	if (ro_count <= 0) return NULL;

	ro_driver_arr = kzalloc(sizeof(struct neuronspi_gpio_driver) * ro_count, GFP_ATOMIC);
	for (i = 0; i < ro_count; i++) {
		pro_driver = ro_driver_arr + i;

        scnprintf(buf, 20, "ro_%d_%02d", neuron_index+1, i+1);
		/*strcpy(pro_driver->name, "ro_0_00");
		pro_driver->name[3] = neuron_index + '1';
		pro_driver->name[5] = ((i + 1) / 10) + '0';
		pro_driver->name[6] = ((i + 1) % 10) + '0';*/
		pro_driver->io_index = i;
		pro_driver->spi = neuronspi_s_dev[neuron_index];
				
		pro_driver->plat_dev = platform_device_alloc(buf, -1);
		pro_driver->plat_dev->dev.parent = &(board_device->dev);
		pro_driver->plat_dev->dev.groups = neuron_gpio_ro_attr_groups;
		pro_driver->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
		platform_device_add(pro_driver->plat_dev);

		platform_set_drvdata(pro_driver->plat_dev, pro_driver);
		pro_driver->gpio_c.owner = THIS_MODULE;
		pro_driver->gpio_c.parent = &(pro_driver->plat_dev->dev);
		pro_driver->gpio_c.label = "neuron_ro";
		pro_driver->gpio_c.can_sleep = 1;
		pro_driver->gpio_c.ngpio = 1;
		pro_driver->gpio_c.base = -1;
		pro_driver->gpio_c.direction_output = neuronspi_gpio_ro_direction_output;
		pro_driver->gpio_c.set = neuronspi_gpio_ro_set;
		gpiochip_add_data(&pro_driver->gpio_c, pro_driver);

	}
	return ro_driver_arr;
}

struct neuronspi_gpio_driver * neuronspi_do_probe(int do_count, int neuron_index, struct platform_device *board_device)
{
	struct neuronspi_gpio_driver* do_driver_arr;
	struct neuronspi_gpio_driver* pdo_driver;
	int i;
    char buf[20];

	if (do_count <= 0) return NULL;

	do_driver_arr = kzalloc(sizeof(struct neuronspi_gpio_driver) * do_count, GFP_ATOMIC);
	for (i = 0; i < do_count; i++) {
		pdo_driver = do_driver_arr + i;

        scnprintf(buf, 20, "do_%d_%02d", neuron_index+1, i+1);
		/*strcpy(pdo_driver->name, "do_0_00");
		pdo_driver->name[3] = neuron_index + '1';
		pdo_driver->name[5] = ((i + 1) / 10) + '0';
		pdo_driver->name[6] = ((i + 1) % 10) + '0';*/
		pdo_driver->io_index = i;
		pdo_driver->spi = neuronspi_s_dev[neuron_index];
				
		pdo_driver->plat_dev = platform_device_alloc(buf, -1);
		pdo_driver->plat_dev->dev.parent = &(board_device->dev);
		pdo_driver->plat_dev->dev.groups = neuron_gpio_do_attr_groups;
		pdo_driver->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
		platform_device_add(pdo_driver->plat_dev);

		platform_set_drvdata(pdo_driver->plat_dev, pdo_driver);
		pdo_driver->gpio_c.owner = THIS_MODULE;
		pdo_driver->gpio_c.parent = &(pdo_driver->plat_dev->dev);
		pdo_driver->gpio_c.label = "neuron_do";
		pdo_driver->gpio_c.can_sleep = 1;
		pdo_driver->gpio_c.ngpio = 1;
		pdo_driver->gpio_c.base = -1;
		pdo_driver->gpio_c.direction_output = neuronspi_gpio_do_direction_output;
		pdo_driver->gpio_c.set = neuronspi_gpio_do_set;
		gpiochip_add_data(&pdo_driver->gpio_c, pdo_driver);

	}
	return do_driver_arr;
}

