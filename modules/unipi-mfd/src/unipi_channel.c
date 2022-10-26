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
#include "unipi_channel.h"
#include "unipi_modbus.h"
#include "unipi_iogroup_bus.h"

/* from unipi_regmap.c */
struct regmap *devm_regmap_init_unipi_regs(struct unipi_channel *channel,
					const struct regmap_config *config);

struct regmap *devm_regmap_init_unipi_coils(struct unipi_channel *channel,
					const struct regmap_config *config);
/***/

/**************************************************************
 * 
 * Synchronous operations
 * 
 * ***********************************************************/
struct unipi_channel_sync_cb_data
{
	struct completion* done;
	int		result;
};

void unipi_channel_sync_op_callback(void* cb_data, int result)
{
	struct unipi_channel_sync_cb_data* unipi_cb_data = (struct unipi_channel_sync_cb_data*) cb_data;
	if (cb_data) {
		unipi_cb_data->result = result;
		complete(unipi_cb_data->done);
	}
}


int unipi_write_bits_sync(struct unipi_channel* channel, unsigned int reg,
                          unsigned int count, u8* data)
{
	int ret = 0;
	struct unipi_channel_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

	init_completion(&done);
	cb_data.done = &done;
	ret = unipi_write_bits_async(channel, reg, count, data, &cb_data, unipi_channel_sync_op_callback);
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}

int unipi_read_bits_sync(struct unipi_channel* channel, unsigned int reg,
                         unsigned int count, u8* data)
{
	int ret = 0;
	struct unipi_channel_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

	init_completion(&done);
	cb_data.done = &done;
	ret = unipi_read_bits_async(channel, reg, count, data, &cb_data, unipi_channel_sync_op_callback);
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}

int unipi_write_regs_sync(struct unipi_channel* channel, unsigned int reg,
                          unsigned int count, u16* data)
{
	int ret = 0;
	struct unipi_channel_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

	init_completion(&done);
	cb_data.done = &done;
	ret = unipi_write_regs_async(channel, reg, count, (u8*)data, &cb_data, unipi_channel_sync_op_callback);
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}

int unipi_read_regs_sync(struct unipi_channel* channel, unsigned int reg,
                         unsigned int count, u16* data)
{
	int ret = 0;
	struct unipi_channel_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done); //struct completion done;

	init_completion(&done);
	cb_data.done = &done;
	ret = unipi_read_regs_async(channel, reg, count, (u8*)data, &cb_data, unipi_channel_sync_op_callback);
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}

int unipi_ping_sync(struct unipi_channel* channel)
{
	int ret = 0;
	struct unipi_channel_sync_cb_data cb_data;
	DECLARE_COMPLETION_ONSTACK(done);

	init_completion(&done);
	cb_data.done = &done;
	ret = unipi_ping_async(channel, &cb_data, unipi_channel_sync_op_callback);
	if (ret != 0)
		return ret;
	wait_for_completion(&done);
	return cb_data.result;
}



int unipi_channel_init(struct unipi_channel * channel, struct device *dev)
{
	struct unipi_iogroup_device *iogroup;
	struct device_node *nc;
	unsigned int modbus_address = 0;

	if (dev->of_node) {
		of_property_read_u32(dev->of_node, "modbus-address", &modbus_address);
		if (modbus_address > 254) {
			dev_warn(dev, "DT property modbus-address value (%d) invalid.\n",modbus_address);
			modbus_address = 0;
		}
	} else {
		for (modbus_address=254; modbus_address>0; modbus_address--) {
			if (unipi_modbus_dev_by_address(modbus_address) != NULL) break;
		}
	}
	/* create MODBUS character device channel */
	if (modbus_address) {
		channel->chrdev = unipi_modbus_classdev_register(channel, modbus_address);
	} else {
		dev_warn(dev, "Undefined modbus-address. Channel inaccessible via modbus chrdev.\n");
	}

	/* create register maps */
	channel->registers = devm_regmap_init_unipi_regs(channel, NULL);
	if (IS_ERR(channel->registers)) {
		return PTR_ERR(channel->registers);
	}
	channel->coils = devm_regmap_init_unipi_coils(channel, NULL);
	if (IS_ERR(channel->coils)) {
		return PTR_ERR(channel->coils);
	}

	/* ToDo: find the best iogroup definition based on firmware */
	nc = of_get_compatible_child(dev->of_node, "iogroup");
	if (nc) {
		if (!of_node_test_and_set_flag(nc, OF_POPULATED)) {
			iogroup = of_register_iogroup_device(channel, nc);
			if (!IS_ERR(iogroup)) {
				return 0;
			}
			of_node_clear_flag(nc, OF_POPULATED);
		}
		of_node_put(nc);
	} else if (!dev->of_node) {
		/* fallback to empty iogroup */
		iogroup = register_iogroup_device(channel, modbus_address, "unipi-mfd");
		if (IS_ERR(iogroup)) {
		}
	}
	return 0;
}

int unipi_channel_exit(struct unipi_channel *channel)
{
	channel->interrupt_status_callback = NULL;
/*	if (channel->poll_timer.function != NULL) {
		hrtimer_try_to_cancel(&channel->poll_timer);
	}
*/
	iogroup_unregister_by_channel(channel);
	if (channel->chrdev) {
		unipi_modbus_classdev_unregister(channel->chrdev);
	}
	return 0;
}


EXPORT_SYMBOL_GPL(unipi_channel_init);
EXPORT_SYMBOL_GPL(unipi_channel_exit);

//MODULE_LICENSE("GPL");
//MODULE_AUTHOR("Miroslav Ondra <ondra@faster.cz>");
//MODULE_DESCRIPTION("Unipi Multifunction Device Driver");
