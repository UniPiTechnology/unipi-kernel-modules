/*
 * unipi_misc.c
 *
 *  Created on: 26 Feb 2018
 *      Author: Tom Knot <knot@faster.cz>
 */

#include "unipi_misc.h"
#include "unipi_spi.h"

void neuronspi_led_proc(struct kthread_work *ws)
{
	struct neuronspi_led_driver *led = to_led_driver(ws, led_work);
	printk("NEURONSPI: BRIGHT id:%d\n", led->id);
	neuronspi_spi_led_set_brightness(led->spi, led->brightness, led->id);
}

void neuronspi_led_set_brightness(struct led_classdev *ldev, enum led_brightness brightness)
{
	struct neuronspi_led_driver *led = container_of(ldev, struct neuronspi_led_driver, ldev);
	struct neuronspi_driver_data *n_spi = spi_get_drvdata(led->spi);
	spin_lock(&led->lock);
	led->brightness = brightness;
	kthread_queue_work(&n_spi->primary_worker, &led->led_work);
	spin_unlock(&led->lock);
}
