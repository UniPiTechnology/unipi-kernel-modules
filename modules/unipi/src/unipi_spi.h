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

#ifndef MODULES_NEURON_SPI_SRC_UNIPI_SPI_H_
#define MODULES_NEURON_SPI_SRC_UNIPI_SPI_H_

/************
 * Includes *
 ************/
#include <linux/version.h>

#include "unipi_common.h"
#include "unipi_platform.h"
#include "unipi_sysfs.h"
#include "unipi_iio.h"

/********************
 * Module Constants *
 ********************/
/*
#define NEURONSPI_SLOWER_MODELS_LEN 					3
static const u16 NEURONSPI_SLOWER_MODELS[NEURONSPI_SLOWER_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10, 0xb20
};
*/

struct neuronspi_frequecy_map
{
    u16  model;
    u16  mask;
    int  frequency;
};

#define NEURONSPI_FREQUENCY_LEN      2
static const struct neuronspi_frequecy_map NEURONSPI_FREQUENCY_MAP[NEURONSPI_FREQUENCY_LEN] = {
	{model:0x0b00, mask:0xff00, frequency:NEURONSPI_SLOWER_FREQ },
	{model:0x0000, mask:0x0000, frequency:NEURONSPI_COMMON_FREQ}
};


#define NEURONSPI_CRC16TABLE_LEN						256
static const u16 NEURONSPI_CRC16TABLE[NEURONSPI_CRC16TABLE_LEN] = {
    0,  1408,  3968,  2560,  7040,  7680,  5120,  4480, 13184, 13824, 15360,
14720, 10240, 11648, 10112,  8704, 25472, 26112, 27648, 27008, 30720, 32128,
30592, 29184, 20480, 21888, 24448, 23040, 19328, 19968, 17408, 16768, 50048,
50688, 52224, 51584, 55296, 56704, 55168, 53760, 61440, 62848, 65408, 64000,
60288, 60928, 58368, 57728, 40960, 42368, 44928, 43520, 48000, 48640, 46080,
45440, 37760, 38400, 39936, 39296, 34816, 36224, 34688, 33280, 33665, 34305,
35841, 35201, 38913, 40321, 38785, 37377, 45057, 46465, 49025, 47617, 43905,
44545, 41985, 41345, 57345, 58753, 61313, 59905, 64385, 65025, 62465, 61825,
54145, 54785, 56321, 55681, 51201, 52609, 51073, 49665, 16385, 17793, 20353,
18945, 23425, 24065, 21505, 20865, 29569, 30209, 31745, 31105, 26625, 28033,
26497, 25089,  9089,  9729, 11265, 10625, 14337, 15745, 14209, 12801,  4097,
 5505,  8065,  6657,  2945,  3585,  1025,   385,   899,  1539,  3075,  2435,
 6147,  7555,  6019,  4611, 12291, 13699, 16259, 14851, 11139, 11779,  9219,
 8579, 24579, 25987, 28547, 27139, 31619, 32259, 29699, 29059, 21379, 22019,
23555, 22915, 18435, 19843, 18307, 16899, 49155, 50563, 53123, 51715, 56195,
56835, 54275, 53635, 62339, 62979, 64515, 63875, 59395, 60803, 59267, 57859,
41859, 42499, 44035, 43395, 47107, 48515, 46979, 45571, 36867, 38275, 40835,
39427, 35715, 36355, 33795, 33155, 32770, 34178, 36738, 35330, 39810, 40450,
37890, 37250, 45954, 46594, 48130, 47490, 43010, 44418, 42882, 41474, 58242,
58882, 60418, 59778, 63490, 64898, 63362, 61954, 53250, 54658, 57218, 55810,
52098, 52738, 50178, 49538, 17282, 17922, 19458, 18818, 22530, 23938, 22402,
20994, 28674, 30082, 32642, 31234, 27522, 28162, 25602, 24962,  8194,  9602,
12162, 10754, 15234, 15874, 13314, 12674,  4994,  5634,  7170,  6530,  2050,
 3458,  1922,   514
};

/***************
 * Definitions *
 ***************/

#define NEURON_INT_RX_NOT_EMPTY 			0x1
#define NEURON_INT_TX_FINISHED  			0x2
#define NEURON_INT_RX_MODBUS    			0x4
#define NEURON_INT_DI_CHANGED   			0x8
#define NEURON_INT_ID_MASK      			0x0f
#define NEURON_INT_NO_INT_BIT   			0x0f

#define NEURONSPI_RECONF_MD					(1 << 0)
#define NEURONSPI_RECONF_IER				(1 << 1)
#define NEURONSPI_RECONF_RS485				(1 << 2)

//#define MODBUS_FIRST_DATA_BYTE				10

#define MODBUS_MAX_READ_BITS                2000
#define MODBUS_MAX_WRITE_BITS               1968
#define MODBUS_MAX_READ_REGISTERS           125
#define MODBUS_MAX_WRITE_REGISTERS          123
#define MODBUS_MAX_WR_WRITE_REGISTERS       121
#define MODBUS_MAX_WR_READ_REGISTERS        125

#define UNIPISPI_OP_MODE_SEND_HEADER        0x1
#define UNIPISPI_OP_MODE_DO_CRC             0x2
#define UNIPISPI_OP_MODE_HAVE_CRC_SPACE     0x4

/*************************
 * Function Declarations *
 *************************/

s32 neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, int length, u8 uart_index);
void neuronspi_spi_uart_read(struct spi_device* spi_dev, u8 *recv_buf, s32 len, u8 uart_index);
int unipispi_modbus_read_register(struct spi_device* spi_dev, u16 reg, u16* value);
int unipispi_modbus_read_u32(struct spi_device* spi_dev, u16 reg, u32* value);
int unipispi_modbus_read_many(struct spi_device* spi_dev, u16 reg, u16* value, int register_count);
int unipispi_modbus_write_register(struct spi_device* spi_dev, u16 reg, u16 value);
int unipispi_modbus_write_u32(struct spi_device* spi_dev, u16 reg, u32 value);
int unipispi_modbus_write_many(struct spi_device* spi_dev, u16 reg, u16* value, int register_count);
int unipispi_modbus_write_coil(struct spi_device* spi_dev, u16 coil, int value);
int unipi_spi_write_str(struct spi_device* spi, struct neuronspi_port* port, int length);
int unipi_spi_get_tx_fifo(struct spi_device* spi, struct neuronspi_port* port);

void neuronspi_enable_uart_interrupt(struct neuronspi_port* n_port);
void neuronspi_uart_flush_proc(struct kthread_work *ws);

/***********************
 * Function structures *
 ***********************/

// Host driver struct
extern struct spi_driver neuronspi_spi_driver;

extern struct mutex unipi_inv_speed_mutex;

static const struct regmap_bus neuronspi_regmap_bus =
{
	.fast_io 					= 0,
	.write 						= neuronspi_regmap_hw_write,
	.gather_write 				= neuronspi_regmap_hw_gather_write,
	.reg_write					= neuronspi_regmap_hw_reg_write,
	.read						= neuronspi_regmap_hw_read,
	.reg_read					= neuronspi_regmap_hw_reg_read,
	.reg_format_endian_default  = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default  = REGMAP_ENDIAN_NATIVE,
	.max_raw_read				= 200,								// CRC and other overhead not included
	.max_raw_write				= 200,								// CRC and other overhead not included
};

static const struct regmap_config neuronspi_regmap_config_default =
{
		.name 					= "UniPiSPI Regmap",
		.reg_bits				= 16,
		.reg_stride				= 0,
		.pad_bits				= 0,
		.val_bits				= 16,
		.max_register			= 65535,
		.cache_type				= REGCACHE_RBTREE,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,6)
		.use_single_read			= 0,
		.use_single_write			= 0,
#else
		.use_single_rw			= 0,
#endif
		.can_multi_write		= 1,
};


// These defines need to be at the end
#define to_neuronspi_uart_data(p,e)  ((container_of((p), struct neuronspi_uart_data, e)))
#define to_neuronspi_port(p,e)	((container_of((p), struct neuronspi_port, e)))
#define to_led_driver(p,e)	((container_of((p), struct neuronspi_led_driver, e)))
#define to_uart_port(p,e)	((container_of((p), struct uart_port, e)))


/*********************
 * In-line Functions *
 *********************/

static __always_inline u16 neuronspi_spi_crc(u8* inputstring, s32 length, u16 initval)
{
    s32 i;
    u16 result = initval;
    for (i=0; i<length; i++) {
        result = (result >> 8) ^ NEURONSPI_CRC16TABLE[(result ^ inputstring[i]) & 0xff];
    }
    return result;
}

static __always_inline int neuronspi_frequency_by_model(u16 model)
{
    int i;
    for (i = 0; i < NEURONSPI_FREQUENCY_LEN; i++) {
        if ((NEURONSPI_FREQUENCY_MAP[i].mask & model) == NEURONSPI_FREQUENCY_MAP[i].model) 
			return NEURONSPI_FREQUENCY_MAP[i].frequency;
    }
	return NEURONSPI_COMMON_FREQ;
/*
    for (i = 0; i < NEURONSPI_SLOWER_MODELS_LEN; i++) {
        if (NEURONSPI_SLOWER_MODELS[i] == model) return 1;
    }
    return 0;
*/
}

static __always_inline int neuronspi_is_noirq_model(u16 model)
{
    int i;
    for (i = 0; i < NEURONSPI_NO_INTERRUPT_MODELS_LEN; i++) {
        if (NEURONSPI_NO_INTERRUPT_MODELS[i] == model) return 1;
    }
    return 0;
}

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_SPI_H_ */
