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
	struct neuronspi_gpio_port *n_di = gpiochip_get_data(chip);
	struct spi_device *spi = n_di->spi;
	return neuronspi_spi_gpio_di_get(spi, n_di->io_index);
}

int neuronspi_gpio_do_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return 0;
}

void neuronspi_gpio_do_set(struct gpio_chip *chip, unsigned offset, int value) {
	struct neuronspi_gpio_port *n_do = gpiochip_get_data(chip);
	struct spi_device *spi = n_do->spi;
	neuronspi_spi_gpio_do_set(spi, n_do->io_index, value);
}

int neuronspi_gpio_ro_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
	return 0;
}

void neuronspi_gpio_ro_set(struct gpio_chip *chip, unsigned offset, int value) {
	struct neuronspi_gpio_port *n_ro = gpiochip_get_data(chip);
	struct spi_device *spi = n_ro->spi;
	neuronspi_spi_gpio_ro_set(spi, n_ro->io_index, value);
}


struct neuronspi_gpio_driver * neuronspi_di_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
	struct neuronspi_gpio_driver* di_driver;
	struct neuronspi_gpio_port* pdi_driver;
	int i;
    char buf[20];

	if (io_count <= 0) return NULL;

	di_driver = kzalloc(sizeof(struct neuronspi_gpio_driver) + (sizeof(struct neuronspi_gpio_port)) * (io_count-1), GFP_ATOMIC);
	for (i = 0; i < io_count; i++) {
		pdi_driver = di_driver->ports + i;
        scnprintf(buf, 20, "di_%d_%02d", neuron_index+1, i+1);
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
    di_driver->count = io_count;
	return di_driver;
}

struct neuronspi_gpio_driver * neuronspi_ro_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
	struct neuronspi_gpio_driver* ro_driver;
	struct neuronspi_gpio_port* gpio_port;
	int i;
    char buf[20];

	if (io_count <= 0) return NULL;

	ro_driver = kzalloc(sizeof(struct neuronspi_gpio_driver) + (sizeof(struct neuronspi_gpio_port)) * (io_count-1), GFP_ATOMIC);
	for (i = 0; i < io_count; i++) {
		gpio_port = ro_driver->ports + i;

        scnprintf(buf, 20, "ro_%d_%02d", neuron_index+1, i+1);
		gpio_port->io_index = i;
		gpio_port->spi = neuronspi_s_dev[neuron_index];
				
		gpio_port->plat_dev = platform_device_alloc(buf, -1);
		gpio_port->plat_dev->dev.parent = &(board_device->dev);
		gpio_port->plat_dev->dev.groups = neuron_gpio_ro_attr_groups;
		gpio_port->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
		platform_device_add(gpio_port->plat_dev);

		platform_set_drvdata(gpio_port->plat_dev, gpio_port);
		gpio_port->gpio_c.owner = THIS_MODULE;
		gpio_port->gpio_c.parent = &(gpio_port->plat_dev->dev);
		gpio_port->gpio_c.label = "neuron_ro";
		gpio_port->gpio_c.can_sleep = 1;
		gpio_port->gpio_c.ngpio = 1;
		gpio_port->gpio_c.base = -1;
		gpio_port->gpio_c.direction_output = neuronspi_gpio_ro_direction_output;
		gpio_port->gpio_c.set = neuronspi_gpio_ro_set;
		gpiochip_add_data(&gpio_port->gpio_c, gpio_port);

	}
    ro_driver->count = io_count;
	return ro_driver;
}

struct neuronspi_gpio_driver * neuronspi_do_probe(int io_count, int neuron_index, struct platform_device *board_device)
{
	struct neuronspi_gpio_driver* do_driver;
	struct neuronspi_gpio_port* gpio_port;
	int i;
    char buf[20];

	if (io_count <= 0) return NULL;

	do_driver = kzalloc(sizeof(struct neuronspi_gpio_driver) + (sizeof(struct neuronspi_gpio_port)) * (io_count-1), GFP_ATOMIC);
	for (i = 0; i < io_count; i++) {
		gpio_port = do_driver->ports + i;

        scnprintf(buf, 20, "do_%d_%02d", neuron_index+1, i+1);
		gpio_port->io_index = i;
		gpio_port->spi = neuronspi_s_dev[neuron_index];
				
		gpio_port->plat_dev = platform_device_alloc(buf, -1);
		gpio_port->plat_dev->dev.parent = &(board_device->dev);
		gpio_port->plat_dev->dev.groups = neuron_gpio_do_attr_groups;
		gpio_port->plat_dev->dev.driver = &neuronspi_spi_driver.driver;
		platform_device_add(gpio_port->plat_dev);

		platform_set_drvdata(gpio_port->plat_dev, gpio_port);
		gpio_port->gpio_c.owner = THIS_MODULE;
		gpio_port->gpio_c.parent = &(gpio_port->plat_dev->dev);
		gpio_port->gpio_c.label = "neuron_do";
		gpio_port->gpio_c.can_sleep = 1;
		gpio_port->gpio_c.ngpio = 1;
		gpio_port->gpio_c.base = -1;
		gpio_port->gpio_c.direction_output = neuronspi_gpio_do_direction_output;
		gpio_port->gpio_c.set = neuronspi_gpio_do_set;
		gpiochip_add_data(&gpio_port->gpio_c, gpio_port);

	}
    do_driver->count = io_count;
	return do_driver;
}

void neuronspi_gpio_remove(struct neuronspi_gpio_driver * gpio_driver)
{
    int i;
	for (i = 0; i < gpio_driver->count; i++) {
        gpiochip_remove(&gpio_driver->ports[i].gpio_c);
		platform_set_drvdata(gpio_driver->ports[i].plat_dev, 0);
		platform_device_unregister(gpio_driver->ports[i].plat_dev);
    }
	kfree(gpio_driver);
    
}
