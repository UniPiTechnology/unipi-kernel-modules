// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Unipi LEDs Driver
 *
 * Copyright (c) 2021, Unipi Technology
 * Author: Miroslav Ondra <ondra@faster.cz>
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/leds.h>

#include "unipi_common.h"
#include "unipi_mfd_iogroup.h"

struct unipi_led {
	struct led_classdev cdev;
	struct regmap *map;
	u32    reg;
	bool   state;
	char   name[32];
};

static void unipi_led_set(struct led_classdev *led_cdev,
                          enum led_brightness value)
{
	struct unipi_led *uled =
		container_of(led_cdev, struct unipi_led, cdev);
	u32 val;
	int ret;

	val = (value != LED_OFF);
	uled->state = val ? true : false;
	ret = regmap_write_async(uled->map, uled->reg, val);
	if (ret < 0)
		dev_err(uled->cdev.dev, "error updating LED status\n");
}

static int unipi_prepare_led(struct unipi_led *uled, struct device_node *parent_node, u32 reg)
{
	struct device_node *np = NULL;
	int ret;
	const char  *state;

	for_each_child_of_node(parent_node, np) {
		u32 child_reg = 0;

		of_property_read_u32(np, "reg", &child_reg);
		if (reg == child_reg)
			break;
	}

	if (np) {
		uled->cdev.name =
			of_get_property(np, "label", NULL) ? : np->name;
		uled->cdev.default_trigger =
			of_get_property(np, "linux,default-trigger", NULL);

		state = of_get_property(np, "default-state", NULL);
		if (state) {
			if (!strcmp(state, "keep")) {
				u32 val;
				ret = regmap_read(uled->map, uled->reg, &val);
				uled->state = !!(val);
			} else if (!strcmp(state, "on")) {
				uled->state = true;
				ret = regmap_write(uled->map, uled->reg, 1);
			} else {
				uled->state = false;
				ret = regmap_write(uled->map, uled->reg, 0);
			}
		}
		of_node_put(np);
	}
	return 0;
}

static int unipi_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regmap *map;
	struct unipi_led *uled;
	//const char *state;
	u32    coil_range[2];
	const char   *label_prefix; 
	int    nleds, i;

	//map = dev_get_regmap(parent,"coils");
	map = unipi_mfd_get_regmap(dev->parent, "coils");

	if (IS_ERR(map) || map == NULL) {
		dev_err(dev, "no coils regmap for Unipi device\n");
		return PTR_ERR(map);
	}
	if (of_property_read_u32_array(np, "coil-range", coil_range, 2))
		return -EINVAL;
	if (coil_range[1] == 0) {
		dev_err(dev, "bad coil-range for Unipi device (count must be non-zero)\n");
		return -EINVAL;
	}
	/*if (device_link_add(dev, dev->parent, DL_FLAG_AUTOREMOVE_CONSUMER) == NULL) {
		dev_err(dev, "Error create link\n");
	}*/

	label_prefix = of_get_property(np, "label-prefix", NULL) ? :"unipi:green:uled-x";
	nleds = 0;
	for (i=0; i<coil_range[1]; i++) {
		uled = devm_kzalloc(dev, sizeof(*uled), GFP_KERNEL);
		if (!uled)
			return -ENOMEM;
		uled->map = map;
		uled->reg = coil_range[0] + i;
		scnprintf(uled->name, sizeof(uled->name), "%s%x", label_prefix, i+1);
		//dev_info(dev, "led %s\n", uled->name);
		uled->cdev.name = uled->name;
		unipi_prepare_led(uled, np, i);
		uled->cdev.brightness_set = unipi_led_set;
		if (devm_led_classdev_register(dev, &uled->cdev) == 0) 
			nleds++;
	}

	//platform_set_drvdata(pdev, uled);
	dev_info(dev, "registered %d LEDs\n", nleds);
	return 0;
}

static const struct of_device_id of_unipi_leds_match[] = {
	{ .compatible = "unipi,leds", },
	{},
};
MODULE_DEVICE_TABLE(of, of_unipi_leds_match);

static struct platform_driver unipi_led_driver = {
	.probe		= unipi_led_probe,
	.driver		= {
		.name	= "unipi-leds",
		.of_match_table = of_match_ptr(of_unipi_leds_match),
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(unipi_led_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
MODULE_DESCRIPTION("Unipi Led Device Driver");
