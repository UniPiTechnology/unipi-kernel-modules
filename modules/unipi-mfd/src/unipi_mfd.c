/*
 * UniPi PLC device driver - Copyright (C) 2018 UniPi Technology
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
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
#include <linux/platform_device.h>

#include "unipi_common.h"
#include "unipi_mfd.h"


static char* unipi_mfd_firmware_names[] = {
	"B-1000",		// 0
	"E-8Di8Ro",		// 1
	"E-14Ro",	 	// 2
	"E-16Di",		// 3
	"+P-11DiR485",	// 4
	"+P-11DiR485",  // 5
	"+P-11DiR485",  // 6
	"+U-14Ro",  	// 7
	"+U-14Ro",  	// 8
	"+U-14Di",  	// 9
	"+U-14Di",  	// 10  0x0a
	"E-4Ai44o",		// 11  0x0b
	"+P-6Di5Ro",	// 12  0x0c
	"B-485",		// 13  0x0d
	"E-4Light",		// 14  0x0e
	"+U-6Di5Ro",	// 15  0x0f
	"X_1Ir",		// 16  0x10
	"MM_8OW",		// 17  0x11
	"+P-4Di6Ro",	// 18  0x12
	"+U-4Di6Ro",	// 19  0x13
	"+P-4Di4Ro",	// 20  0x14
};

char* unipi_mfd_find_firmware_name(u8 id)
{
	if (id < ARRAY_SIZE(unipi_mfd_firmware_names)) return unipi_mfd_firmware_names[id];
	return "";
}

/* callback of poll_timer - for devices without or disfunctional interrupt */
static enum hrtimer_restart unipi_mfd_poll_timer_func(struct hrtimer *timer)
{
	struct unipi_mfd_device *mfd = ((container_of((timer), struct unipi_mfd_device, poll_timer)));
	mfd->op->async_idle(mfd->self, NULL, NULL);
	//unipi_spi_trace((mfd->dev), "Pseudo IRQ\n");
	return HRTIMER_RESTART;
}


int unipi_mfd_enable_interrupt(struct unipi_mfd_device *mfd, u16 mask)
{
	//struct unipi_mfd_device *mfd = spi_get_drvdata(spi);
	if (mfd->irq == 0) {
		mfd->poll_enabled = (mask != 0);
		if (mfd->poll_enabled) {
			if ((mfd->poll_timer.function == NULL)) {
				hrtimer_init(&mfd->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
				mfd->poll_timer.function = unipi_mfd_poll_timer_func;
			}
			hrtimer_start_range_ns(&mfd->poll_timer, 2000000, 4000000, HRTIMER_MODE_REL);
		} else {
			if (mfd->poll_timer.function != NULL) {
				hrtimer_try_to_cancel(&mfd->poll_timer);
			}
		}
	}

	regmap_write_async(mfd->registers, UNIPI_MFD_REG_INTERRUPT_MASK, mask);
	return 0;
}
EXPORT_SYMBOL_GPL(unipi_mfd_enable_interrupt);

/* real irq handler - emit idle_op asynchronously */
irqreturn_t unipi_mfd_irq(s32 irq, void *dev_id)
{
	struct unipi_mfd_device *mfd = (struct unipi_mfd_device *)dev_id;
	mfd->op->async_idle(mfd->self, NULL, NULL);
	//unipi_spi_trace((mfd->dev), "IRQ\n");
	return IRQ_HANDLED;
}

void unipi_mfd_int_status_callback(struct unipi_mfd_device* mfd, u8 int_status)
{
	/* int_status & UNIPI_MFD_INT_RX_NOT_EMPTY */
}

int unipi_mfd_init(struct unipi_mfd_device * mfd, struct device *dev)
{
	int ret = 0;
	//unsigned long flags;

	if (mfd->irq) {
		ret = devm_request_irq(dev, mfd->irq, unipi_mfd_irq, 0, dev_name(dev), mfd);
		dev_info(dev, "IRQ %d registration: ret=%d\n", mfd->irq, ret);
		if (ret != 0) mfd->irq = 0;
	}
	mfd->interrupt_status_callback = unipi_mfd_int_status_callback;

/*	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, subdevs, n_subdevs, NULL, 0, NULL);*/

	return 0;
}

int unipi_mfd_exit(struct unipi_mfd_device * mfd)
{
	mfd->interrupt_status_callback = NULL;
	if (mfd->poll_timer.function != NULL) {
		hrtimer_try_to_cancel(&mfd->poll_timer);
	}
	return 0;
}


EXPORT_SYMBOL_GPL(unipi_mfd_init);
EXPORT_SYMBOL_GPL(unipi_mfd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Multifunction Device Driver");
