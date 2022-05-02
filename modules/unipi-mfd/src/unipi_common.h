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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_COMMON_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_COMMON_H_

#define DISABLE_PLATFORM 1

/************
 * Includes *
 ************/

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <uapi/linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/spi/spi.h>
#include <linux/leds.h>
#include <linux/uaccess.h>
#include <asm/termbits.h>
//#include <asm/gpio.h>

/***************
 * Definitions *
 ***************/
//#define UNIPISPI_USE_RX_THREAD


#define UNIPI_MODULE_MAJOR_VERSIONSTRING "1.78:2021:10:20"

#define UNIPI_SPI_DETAILED_DEBUG 2


//#define NEURONSPI_MAX_DEVS				7	//Maximal SPI card count
//#define NEURONSPI_MAX_UART				16
//#define NEURONSPI_BUFFER_MAX			1152
//#define NEURONSPI_HEADER_LENGTH 		10
//#define NEURONSPI_FIRST_MESSAGE_LENGTH	6
//#define NEURONSPI_FIRST_MESSAGE_ALLOC	8
//#define NEURONSPI_EDGE_DELAY			10
//#define NEURONSPI_B_PER_WORD 			8
//#define NEURONSPI_DEFAULT_FREQ			600000
//#define NEURONSPI_COMMON_FREQ			12000000

//#define NEURONSPI_SYSLED_REGISTER 	509


//#define NEURONSPI_MAX_TX				256
#define NEURONSPI_MAX_BAUD				115200
//#define NEURONSPI_FIFO_SIZE				256
#define NEURONSPI_FIFO_MIN_CONTINUOUS	50
#define NEURONSPI_LAST_TRANSFER_DELAY	40
#define MAX_RX_QUEUE_LEN                16

#define UNIPI_SPI_INTER_FRAME_NS	(NEURONSPI_LAST_TRANSFER_DELAY*1000)


#define STRICT_RESERVING
#define NEURONSPI_ALWAYS_EXPORT

#define NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(X)	((((X) + 15) >> 4) << 1)

#define UNIPI_MODBUS_DETAILED_DEBUG		2


/********************
 * Module Constants *
 ********************/

#define NEURONSPI_NO_INTERRUPT_MODELS_LEN 				4
static const u16 NEURONSPI_NO_INTERRUPT_MODELS[NEURONSPI_NO_INTERRUPT_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10, 0xb20
};

/* macros to parse fw_version, fw_variant, board_id */
#define lo(x) (x & 0xff)
#define hi(x) (x >> 8)
#define minor(x) ((x & 0xf0)>>4)
#define is_cal(x) ((x & 0x8) != 0)

/*******************
 * Data Structures *
 *******************/

#define CB_WRITESTRING 1
#define CB_GETTXFIFO  2
#define START_TX       3


#endif /* MODULES_NEURON_SPI_SRC_UNIPI_COMMON_H_ */
