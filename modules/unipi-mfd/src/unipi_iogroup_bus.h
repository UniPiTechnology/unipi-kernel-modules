/*
 * UniPi Multi Function Device Driver - Copyright (C) 2021 UniPi Technology
 *
 * Author: Miroslav Ondra <ondra@faster.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_IOGROUP_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_IOGROUP_H_

#include <linux/version.h>
#include <linux/nvmem-consumer.h>

#include "unipi_common.h"

struct unipi_iogroup_device;


/**
 * struct unipi_iogroup_device - Plc side proxy for an SPI slave device
 * @dev: Driver model representation of the device.
 * @channel: Channel used to communication with the device.
 * @address: Unique id of iogroup device (usually slot number in plc)
 * @irq:  Interrupt used by iogroup
 * @poll_timer, poll_enabled: Alternative to interrupt
 * @modalias: Name of the driver to use with this device, or an alias
 *	for that name.  This appears in the sysfs "modalias" attribute
 *	for driver coldplugging, and in uevents used for hotplugging
 *
 * In @dev, the platform_data is used to hold information about this
 * device that's meaningful to the device's protocol driver, but not
 * to its controller.  One example might be an identifier for a chip
 * variant with slightly different functionality; another might be
 * information about how this particular board wires the chip's pins.
 */
struct unipi_iogroup_device {
	struct device		dev;
	struct unipi_channel *channel;
	u8					address;
	int					irq;
	struct hrtimer		poll_timer;
	int					poll_enabled;
	char			modalias[SPI_NAME_SIZE];
	void			*uart_rx_self;
	void			(*uart_rx_callback)(void*, int port);
	/* the statistics */
	//struct spi_statistics	statistics;
};

static inline struct unipi_iogroup_device *to_unipi_iogroup_device(struct device *dev)
{
	return dev ? container_of(dev, struct unipi_iogroup_device, dev) : NULL;
}

/* most drivers won't need to care about device refcounting */
static inline struct unipi_iogroup_device *unipi_iogroup_dev_get(struct unipi_iogroup_device *iogroup)
{
	return (iogroup && get_device(&iogroup->dev)) ? iogroup : NULL;
}

static inline void unipi_iogroup_dev_put(struct unipi_iogroup_device *iogroup)
{
	if (iogroup)
		put_device(&iogroup->dev);
}

/**
 * struct unipi_iogroup_driver - Host side "protocol" driver
 * @probe: Binds this driver to the iogroup device.  Drivers can verify
 *	that the device is actually present, and may need to configure
 *	characteristics which weren't needed for
 *	the initial configuration done during system setup.
 * @remove: Unbinds this driver from the iogroup device
 * @shutdown: Standard shutdown callback used during system state
 *	transitions such as powerdown/halt and kexec
 * @driver: iogroup device drivers should initialize the name and owner
 *	field of this structure.
 */
struct unipi_iogroup_driver {
	int			(*probe)(struct unipi_iogroup_device *iogroup);
	int			(*remove)(struct unipi_iogroup_device *iogroup);
	void		(*shutdown)(struct unipi_iogroup_device *iogroup);
	struct device_driver	driver;
};

/* device driver data */

/*
static inline void unipi_iogroup_set_drvdata(struct unipi_iogroup_device *iogroup, void *data)
{
	dev_set_drvdata(&iogroup->dev, data);
}

static inline void *unipi_iogroup_get_drvdata(struct unipi_iogroup_device *iogroup)
{
	return dev_get_drvdata(&iogroup->dev);
}
*/
static inline struct unipi_iogroup_driver *to_unipi_iogroup_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct unipi_iogroup_driver, driver) : NULL;
}

extern int __unipi_iogroup_register_driver(struct module *owner, struct unipi_iogroup_driver *sdrv);

/**
 * unipi_iogroup_unregister_driver - reverse effect of unipi_iogroup_register_driver
 * @sdrv: the driver to unregister
 * Context: can sleep
 */
static inline void unipi_iogroup_unregister_driver(struct unipi_iogroup_driver *sdrv)
{
	if (sdrv)
		driver_unregister(&sdrv->driver);
}

/* use a define to avoid include chaining to get THIS_MODULE */
#define unipi_iogroup_register_driver(driver) \
	__unipi_iogroup_register_driver(THIS_MODULE, driver)

struct unipi_iogroup_device *
       of_register_iogroup_device(struct unipi_channel *channel, struct device_node *nc);
struct unipi_iogroup_device *
       register_iogroup_device(struct unipi_channel *channel, int reg, const char* modalias);
void iogroup_unregister_by_channel(struct unipi_channel *channel);
void iogroup_unregister_device(struct unipi_iogroup_device *iogroup);


#endif
