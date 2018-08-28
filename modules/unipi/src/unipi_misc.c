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

void neuronspi_spi_led_set_brightness(struct spi_device* spi_dev, enum led_brightness brightness, int id)
{
	struct neuronspi_driver_data *d_data = spi_get_drvdata(spi_dev);
        u16 coil;
#if NEURONSPI_DETAILED_DEBUG > 0
	printk(KERN_INFO "UNIPISPI: SPI LED Set, Dev-CS:%d, led id:%d\n", spi_dev->chip_select, id);
#endif

	if (d_data->features != NULL) {
		coil = d_data->features->di_count + d_data->features->do_count + d_data->features->ro_count + id;
	} else {
		coil = 8 + id;
	}
    unipispi_modbus_write_coil(spi_dev, coil, brightness > 0);
}


void neuronspi_led_proc(struct kthread_work *ws)
{
	struct neuronspi_led_driver *led = to_led_driver(ws, led_work);
	neuronspi_spi_led_set_brightness(led->spi, led->brightness, led->id);
}

void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness)
{
	struct neuronspi_led_driver *led = container_of(ldev, struct neuronspi_led_driver, ldev);
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(led->spi);
	unsigned long flags;
	spin_lock_irqsave(&led->lock, flags);
	led->brightness = brightness;
	kthread_queue_work(&n_spi->primary_worker, &led->led_work);
	spin_unlock_irqrestore(&led->lock, flags);
}
