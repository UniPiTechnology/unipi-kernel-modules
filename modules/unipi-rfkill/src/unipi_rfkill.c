// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2011, NVIDIA Corporation.
 * Copyright (c) 2022, Miroslav Ondra, Faster CZ, Unipi Technology
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>


//#define XXX_SHOW 1

enum {
    UNIPI_RFKILL_ALGO_LEVEL,
    UNIPI_RFKILL_ALGO_PULSE,
};

enum unipi_rfkill_state {
	UNIPI_RFKILL_STATE_UNKNOWN = 0,
	UNIPI_RFKILL_STATE_ON,
	UNIPI_RFKILL_STATE_INOP_BLOCK,
	UNIPI_RFKILL_STATE_WAIT_OFF,
	UNIPI_RFKILL_STATE_OFF,
	UNIPI_RFKILL_STATE_INOP_UNBLOCK,
	UNIPI_RFKILL_STATE_WAIT_ON,
	UNIPI_RFKILL_STATE_INOP_RESET,
	UNIPI_RFKILL_STATE_WAIT_RESET,
	UNIPI_RFKILL_STATE_INOP_FORCE_RESET,
	UNIPI_RFKILL_STATE_INOP_FORCE_OFF
};

struct unipi_rfkill_data {
	const char		*name;
	enum rfkill_type	type;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*rfkill_gpio;

	struct rfkill		*rfkill_dev;
	struct rfkill		*reset_dev;
	unsigned int		algo;
	unsigned int		start_pulse_ms;
	unsigned int		stop_pulse_ms;
	unsigned int		wait_after_ms;
	unsigned int		reset_pulse_ms;
#ifdef XXX_SHOW
	struct device *dev;
#endif
	spinlock_t		lock;
	enum unipi_rfkill_state state;
	int counter;
};


void unipi_quectel_poll(struct rfkill *rfkill_dev, void *data)
{
	struct unipi_rfkill_data *rfkill = data;
	int status, op;
	unsigned long flags;

#ifdef XXX_SHOW
	printk("poll %d\n", rfkill->state);
#endif

	switch (rfkill->state) {
		case UNIPI_RFKILL_STATE_INOP_BLOCK:
		case UNIPI_RFKILL_STATE_INOP_UNBLOCK:
		case UNIPI_RFKILL_STATE_INOP_RESET:
		case UNIPI_RFKILL_STATE_INOP_FORCE_RESET:
		case UNIPI_RFKILL_STATE_INOP_FORCE_OFF:
			return;
			break;
		default:
			break;
	}

	status = ! gpiod_get_value_cansleep(rfkill->reset_gpio);
	op = 0;


	spin_lock_irqsave(&rfkill->lock, flags);
	rfkill->counter++;
	switch (rfkill->state) {
		case UNIPI_RFKILL_STATE_ON:
				if (!status) {
					rfkill->state = UNIPI_RFKILL_STATE_INOP_UNBLOCK;
					op = 1;
				}
				break;
		case UNIPI_RFKILL_STATE_OFF:
				if (status) rfkill->state = UNIPI_RFKILL_STATE_ON;
				break;

		case UNIPI_RFKILL_STATE_WAIT_ON:
				if (status)
					rfkill->state = UNIPI_RFKILL_STATE_ON;
				else if (rfkill->counter > 3) {
					rfkill->state = UNIPI_RFKILL_STATE_INOP_UNBLOCK;
					op = 1;
				}
				break;

		case UNIPI_RFKILL_STATE_WAIT_OFF:
				if (!status)
					rfkill->state = UNIPI_RFKILL_STATE_OFF;
				else if (rfkill->counter > 3) {
					rfkill->state = UNIPI_RFKILL_STATE_INOP_FORCE_OFF;
					op = 2;
				}
				break;
				
		case UNIPI_RFKILL_STATE_WAIT_RESET:
				if (!status) {
					rfkill->state = UNIPI_RFKILL_STATE_INOP_UNBLOCK;
					op = 1;
				} else if (rfkill->counter > 3) {
					rfkill->state = UNIPI_RFKILL_STATE_INOP_FORCE_RESET;
					op = 2;
				}
				break;

		case UNIPI_RFKILL_STATE_UNKNOWN:
				rfkill->state = status ? UNIPI_RFKILL_STATE_ON : UNIPI_RFKILL_STATE_OFF;
				break;
		default:
			break;
	}
	spin_unlock_irqrestore(&rfkill->lock, flags);
#ifdef XXX_SHOW
	printk("poll pin=%d %d\n", status, rfkill->state);
#endif
	if (!op) 
		return;
	if (op == 2) {
		/* generate pulse on reset_gpio */
		gpiod_set_value_cansleep(rfkill->reset_gpio, 1);
		mdelay(rfkill->reset_pulse_ms);
		gpiod_set_value_cansleep(rfkill->reset_gpio, 0);
	} else {
		/* generate pulse on rfkill_gpio */
		gpiod_set_value_cansleep(rfkill->rfkill_gpio, 1);
		mdelay(rfkill->start_pulse_ms);
		gpiod_set_value_cansleep(rfkill->rfkill_gpio, 0);
	}

	spin_lock_irqsave(&rfkill->lock, flags);
	if (rfkill->state == UNIPI_RFKILL_STATE_INOP_BLOCK) {
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_OFF;
	} else if (rfkill->state == UNIPI_RFKILL_STATE_INOP_UNBLOCK) {
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_ON;
	} else if (rfkill->state == UNIPI_RFKILL_STATE_INOP_RESET) {
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_RESET;
	} else if (rfkill->state == UNIPI_RFKILL_STATE_INOP_FORCE_OFF) {
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_OFF;
	}
	else if (rfkill->state == UNIPI_RFKILL_STATE_INOP_FORCE_RESET) {
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_RESET;
	}
	rfkill->counter = 0;
	spin_unlock_irqrestore(&rfkill->lock, flags);
}

int unipi_quectel_block_or_reset(struct unipi_rfkill_data *rfkill, int blockop)
{
	unsigned long flags;
	int op;

#ifdef XXX_SHOW
	dev_info(rfkill->dev,"Set %d state=%d\n", blockop, rfkill->state);
#endif

	if (rfkill->state == UNIPI_RFKILL_STATE_UNKNOWN) 
		unipi_quectel_poll(NULL, rfkill);

	op = 0;
	spin_lock_irqsave(&rfkill->lock, flags);
	switch (rfkill->state) {
		case UNIPI_RFKILL_STATE_ON:
			if (blockop==1) {
				op = 1;
				rfkill->state = UNIPI_RFKILL_STATE_INOP_BLOCK;
			} else if (blockop==2) {
				op = 1;
				rfkill->state = UNIPI_RFKILL_STATE_INOP_RESET;
			}
			break;
		case UNIPI_RFKILL_STATE_OFF:
			if (blockop==0) {
				op = 1;
				rfkill->state = UNIPI_RFKILL_STATE_INOP_UNBLOCK;
			}
			break;
		case UNIPI_RFKILL_STATE_INOP_BLOCK:
			if (blockop==0) rfkill->state = UNIPI_RFKILL_STATE_INOP_UNBLOCK;
			break;
		case UNIPI_RFKILL_STATE_INOP_RESET:
			if (blockop==1) rfkill->state = UNIPI_RFKILL_STATE_INOP_BLOCK;
			break;
		case UNIPI_RFKILL_STATE_INOP_UNBLOCK:
			if (blockop==1) rfkill->state = UNIPI_RFKILL_STATE_INOP_BLOCK;
			break;
		case UNIPI_RFKILL_STATE_INOP_FORCE_RESET:
			if (blockop==1) rfkill->state = UNIPI_RFKILL_STATE_INOP_FORCE_OFF;
			break;
		case UNIPI_RFKILL_STATE_INOP_FORCE_OFF:
			if (blockop==1) rfkill->state = UNIPI_RFKILL_STATE_INOP_FORCE_RESET;
			break;

		case UNIPI_RFKILL_STATE_WAIT_OFF:
			if (blockop==0) rfkill->state = UNIPI_RFKILL_STATE_ON;
			break;
		case UNIPI_RFKILL_STATE_WAIT_RESET:
			if (blockop==1) rfkill->state = UNIPI_RFKILL_STATE_WAIT_OFF;
			break;
		case UNIPI_RFKILL_STATE_WAIT_ON:
			if (blockop==1) rfkill->state = UNIPI_RFKILL_STATE_OFF;
			break;
		default:
			break;
	}
	spin_unlock_irqrestore(&rfkill->lock, flags);

	if (!op) 
		return 0;

	/* generate pulse on rfkill_gpio */
	gpiod_set_value_cansleep(rfkill->rfkill_gpio, 1);
	mdelay(blockop==0?rfkill->start_pulse_ms:rfkill->stop_pulse_ms);
	gpiod_set_value_cansleep(rfkill->rfkill_gpio, 0);
	mdelay(rfkill->wait_after_ms);

	spin_lock_irqsave(&rfkill->lock, flags);
	if (rfkill->state == UNIPI_RFKILL_STATE_INOP_BLOCK)
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_OFF;
	else if (rfkill->state == UNIPI_RFKILL_STATE_INOP_UNBLOCK)
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_ON;
	else if (rfkill->state == UNIPI_RFKILL_STATE_INOP_RESET)
		rfkill->state = UNIPI_RFKILL_STATE_WAIT_RESET;
	rfkill->counter = 0;
	spin_unlock_irqrestore(&rfkill->lock, flags);
	/* check real state */
	unipi_quectel_poll(NULL, rfkill);
	return 0;
}

static int unipi_quectel_set_power(void *data, bool blocked)
{
	struct unipi_rfkill_data *rfkill = data;

	unipi_quectel_block_or_reset(rfkill, blocked ? 1 : 0);
	return 0;
}

static int unipi_quectel_set_reset(void *data, bool blocked)
{
	struct unipi_rfkill_data *rfkill = data;

	unipi_quectel_block_or_reset(rfkill, blocked ? 2 : 0);
	return blocked;
}

static const struct rfkill_ops unipi_quectel_ops = {
	.set_block = unipi_quectel_set_power,
	.poll = unipi_quectel_poll,
};

static const struct rfkill_ops unipi_quectel_reset_ops = {
	.set_block = unipi_quectel_set_reset,
};

struct unipi_rfkill_data unipi_rfkill_data_simple = {

}; 

struct unipi_rfkill_data unipi_rfkill_data_quectel = {
	.start_pulse_ms = 500, .stop_pulse_ms = 600, .reset_pulse_ms = 300,
	.wait_after_ms = 20
}; 

static const struct of_device_id unipi_rfkill_ids[] = {
    { .compatible = "unipi,unipi-rfkill", .data = &unipi_rfkill_data_simple },
    { .compatible = "unipi,rfkill-quectel912", .data = &unipi_rfkill_data_quectel },
    { /*sentinel */}
};
MODULE_DEVICE_TABLE(of, unipi_rfkill_ids);


static int unipi_rfkill_probe(struct platform_device *pdev)
{
	struct unipi_rfkill_data *rfkill;
	const struct unipi_rfkill_data *cdata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_desc *gpio;
	const char *type_name;
	enum gpiod_flags gflags;
	const char *algo;
	char rname[128];

	int ret;

	rfkill = devm_kzalloc(dev, sizeof(*rfkill), GFP_KERNEL);
	if (!rfkill)
		return -ENOMEM;

	if (np && of_match_device(unipi_rfkill_ids, dev)) {
		cdata = of_device_get_match_data(dev);
		rfkill->start_pulse_ms = cdata->start_pulse_ms;
		rfkill->stop_pulse_ms = cdata->stop_pulse_ms;
		rfkill->reset_pulse_ms = cdata->reset_pulse_ms;
		rfkill->wait_after_ms = cdata->wait_after_ms;
	}

#ifdef XXX_SHOW
	rfkill->dev = dev;
#endif
	of_property_read_string(np, "name", &rfkill->name);
	of_property_read_string(np, "type", &type_name);

	if (!rfkill->name)
		rfkill->name = dev_name(dev);

	rfkill->type = rfkill_find_type(type_name);

	ret = of_property_read_string(np, "algo", &algo);
	if (ret)
		return ret;
	if (!strcmp(algo, "pulse")) {
		rfkill->algo = UNIPI_RFKILL_ALGO_PULSE;
		gflags = GPIOD_IN;
		rfkill->start_pulse_ms = 500;
		rfkill->stop_pulse_ms = 600;
		rfkill->reset_pulse_ms = 300;
		rfkill->wait_after_ms = 20;
	} else if (!strcmp(algo, "level")) {
		rfkill->algo = UNIPI_RFKILL_ALGO_LEVEL;
		gflags = GPIOD_OUT_LOW;
	} else {
		return -EINVAL;
	}
	gpio = devm_gpiod_get_optional(&pdev->dev, "reset", gflags);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	rfkill->reset_gpio = gpio;

	gpio = devm_gpiod_get(&pdev->dev, "rfkill", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	rfkill->rfkill_gpio = gpio;


	/* Make sure at-least one GPIO is defined for this instance */
	if (!rfkill->reset_gpio && !rfkill->rfkill_gpio) {
		dev_err(&pdev->dev, "invalid platform data\n");
		return -EINVAL;
	}

	spin_lock_init(&rfkill->lock);

	rfkill->rfkill_dev = rfkill_alloc(rfkill->name, &pdev->dev,
					  rfkill->type, &unipi_quectel_ops,
					  rfkill);
	if (!rfkill->rfkill_dev)
		return -ENOMEM;

	ret = rfkill_register(rfkill->rfkill_dev);
	if (ret < 0)
		goto err_destroy;

	snprintf(rname, 128, "%.120s-reset",rfkill->name);

	rfkill->reset_dev = rfkill_alloc(rname, &pdev->dev,
					  rfkill->type, &unipi_quectel_reset_ops,
					  rfkill);
	if (!rfkill->reset_dev)
		return -ENOMEM;

	ret = rfkill_register(rfkill->reset_dev);
	if (ret < 0)
		goto err_destroy1;

	platform_set_drvdata(pdev, rfkill);

	dev_info(&pdev->dev, "%s device registered.\n", rfkill->name);

	return 0;

err_destroy1:
	rfkill_destroy(rfkill->reset_dev);
	rfkill_unregister(rfkill->rfkill_dev);
err_destroy:
	rfkill_destroy(rfkill->rfkill_dev);

	return ret;
}

static int unipi_rfkill_remove(struct platform_device *pdev)
{
	struct unipi_rfkill_data *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill->reset_dev);
	rfkill_destroy(rfkill->reset_dev);

	rfkill_unregister(rfkill->rfkill_dev);
	rfkill_destroy(rfkill->rfkill_dev);

	return 0;
}


static struct platform_driver unipi_rfkill_driver = {
	.probe = unipi_rfkill_probe,
	.remove = unipi_rfkill_remove,
	.driver = {
		.name = "unipi_rfkill",
		.of_match_table = of_match_ptr(unipi_rfkill_ids),
		.owner = THIS_MODULE,
	},
};

module_platform_driver(unipi_rfkill_driver);

MODULE_DESCRIPTION("rfkill for usb/spi modems");
MODULE_AUTHOR("Miroslav Ondra");
MODULE_LICENSE("GPL");
