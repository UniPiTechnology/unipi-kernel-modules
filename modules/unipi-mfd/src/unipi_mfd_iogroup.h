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

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_MFD_IOGROUP_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_MFD_IOGROUP_H_

#include <linux/version.h>
#include "unipi_common.h"

/*struct unipi_mfd_iogroup_device {
	//int slot_index;
	//struct spi_device * spi_dev;
	struct regmap* registers;
	struct regmap* coils;
};
*/
struct dev_mfd_attribute {
	struct device_attribute attr;
	int reg;
};


ssize_t unipi_mfd_store_ulong(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size);
ssize_t unipi_mfd_show_ulong(struct device *dev,
					struct device_attribute *attr,
					char *buf);
ssize_t unipi_mfd_store_int(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size);
ssize_t unipi_mfd_show_int(struct device *dev,
					struct device_attribute *attr,
					char *buf);
ssize_t unipi_mfd_store_reg(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size);
ssize_t unipi_mfd_show_reg(struct device *dev,
					struct device_attribute *attr,
					char *buf);
ssize_t unipi_mfd_showhex_reg(struct device *dev,
					struct device_attribute *attr,
					char *buf);

struct regmap* unipi_mfd_get_regmap(struct device* dev, const char* name);
int unipi_mfd_add_group(struct device *dev, const char *groupname, struct attribute ** templ, int count, ...);


#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_MFD_IOGROUP_H_ */
