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

#include "unipi_misc.h"
#include "unipi_spi.h"

/************************
 * Non-static Functions *
 ************************/

void neuronspi_spi_led_set_brightness(struct spi_device* spi_dev, int brightness, u16 coil)
{
    unipispi_modbus_write_coil(spi_dev, coil, brightness);
}


void neuronspi_led_proc(struct kthread_work *ws)
{
	struct neuronspi_led_driver *led = to_led_driver(ws, led_work);
	neuronspi_spi_led_set_brightness(led->spi, led->brightness > 0, led->coil);
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "UNIPISPI: SPI LED Set, Dev-CS:%d, led id:%d\n", led->spi->chip_select, led->id);
#endif
}

void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness)
{
	struct neuronspi_led_driver *led = container_of(ldev, struct neuronspi_led_driver, ldev);
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(led->spi);
	unsigned long flags;
	spin_lock_irqsave(&led->lock, flags);
	led->brightness = brightness;
	kthread_queue_work(n_spi->primary_worker, &led->led_work);
	spin_unlock_irqrestore(&led->lock, flags);
}



struct neuronspi_led_driver * neuronspi_led_probe(int led_count, int neuron_index, struct platform_device *board_device)
{
    struct spi_device* spi = neuronspi_s_dev[neuron_index];
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(spi);
	struct neuronspi_led_driver * led_driver = kzalloc(sizeof(struct neuronspi_led_driver) * led_count, GFP_ATOMIC);
	int i, coil;

	if (n_spi->features != NULL) {
		coil = n_spi->features->di_count + n_spi->features->do_count + n_spi->features->ro_count;
	} else {
		coil = 8;
	}
	
	for (i = 0; i < led_count; i++) {
		scnprintf(led_driver[i].name, sizeof(led_driver[i].name), "unipi:green:uled-x%x", i);
		// Initialise the rest of the structure
		led_driver[i].id = i;
		led_driver[i].coil = coil+i;
		led_driver[i].brightness = LED_OFF;
		led_driver[i].spi = spi;

		spin_lock_init(&led_driver[i].lock);
		led_driver[i].ldev.name = led_driver[i].name;
		led_driver[i].ldev.brightness = led_driver[i].brightness;
		led_driver[i].ldev.brightness_set = neuronspi_led_set_brightness;
		led_classdev_register(&(board_device->dev), &(led_driver[i].ldev));
		kthread_init_work(&(led_driver[i].led_work), neuronspi_led_proc);
	}
	return led_driver;
}