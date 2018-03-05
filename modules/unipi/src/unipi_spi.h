/*
 * UniPi Neuron tty serial driver - Copyright (C) 2017 UniPi Technologies
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

#include "unipi_common.h"
#include "unipi_platform.h"
#include "unipi_sysfs.h"
#include "unipi_iio.h"

/********************
 * Module Constants *
 ********************/

#define NEURONSPI_SLOWER_MODELS_LEN 					3
static const u16 NEURONSPI_SLOWER_MODELS[NEURONSPI_SLOWER_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10
};

#define NEURONSPI_PROBE_MESSAGE_LEN						22
static const u8 NEURONSPI_PROBE_MESSAGE[NEURONSPI_PROBE_MESSAGE_LEN] = {
		0x04, 0x0e, 0xe8, 0x03, 0xa0, 0xdd,
		0x04, 0x00, 0xe8, 0x03,	0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00,	0x12, 0x16
};

#define NEURONSPI_UART_PROBE_MESSAGE_LEN				6
static const u8 NEURONSPI_UART_PROBE_MESSAGE[NEURONSPI_UART_PROBE_MESSAGE_LEN] = {
		0xfa, 0x00, 0x55, 0x0e, 0xb6, 0x0a
};

#define NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN			6
static const u8 NEURONSPI_SPI_UART_SHORT_MESSAGE[NEURONSPI_SPI_UART_SHORT_MESSAGE_LEN] = {
		0x41, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_UART_LONG_MESSAGE_LEN				8
static const u8 NEURONSPI_SPI_UART_LONG_MESSAGE[NEURONSPI_SPI_UART_LONG_MESSAGE_LEN] = {
		0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00
};

#define NEURONSPI_SPI_UART_READ_MESSAGE_LEN				14
static const u8 NEURONSPI_SPI_UART_READ_MESSAGE[NEURONSPI_SPI_UART_READ_MESSAGE_LEN] = {
		0x65, 0x06, 0x00, 0x00, 0x44, 0x69,
		0x65, 0x03, 0x00, 0x00, 0x00, 0x05,
		0x6a, 0x0c
};

#define NEURONSPI_SPI_IRQ_SET_MESSAGE_LEN				14
static const u8 NEURONSPI_SPI_IRQ_SET_MESSAGE[NEURONSPI_SPI_UART_READ_MESSAGE_LEN] = {
		0x06, 0x06, 0xef, 0x03, 0x00, 0x00,
		0x06, 0x01, 0xef, 0x03, 0x05, 0x00,
		0x00, 0x00
};

#define NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN 		16
static const u8 NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE[NEURONSPI_SPI_UART_GET_CFLAG_MESSAGE_LEN] = {
		0x04, 0x08, 0xf4, 0x01, 0x00, 0x00,
		0x04, 0x02, 0xf4, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_UART_GET_LDISC_MESSAGE_LEN 		16
static const u8 NEURONSPI_SPI_UART_GET_LDISC_MESSAGE[NEURONSPI_SPI_UART_GET_LDISC_MESSAGE_LEN] = {
		0x04, 0x08, 0xf6, 0x01, 0x00, 0x00,
		0x04, 0x02, 0xf6, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN		16
static const u8 NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE[NEURONSPI_SPI_UART_SET_CFLAG_MESSAGE_LEN] = {
		0x06, 0x08, 0xf4, 0x01, 0x00, 0x00,
		0x06, 0x02, 0xf4, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
};

#define NEURONSPI_SPI_LED_SET_MESSAGE_LEN				6
static const u8 NEURONSPI_SPI_LED_SET_MESSAGE[NEURONSPI_SPI_LED_SET_MESSAGE_LEN] = {
		0x05, 0x00, 0x08, 0x00, 0x00, 0x00
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

static const struct of_device_id neuronspi_id_match[] = {
		{.compatible = "unipi,neuron"},
		{.compatible = NEURON_DEVICE_NAME},
		{}
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

#define MODBUS_FIRST_DATA_BYTE				10

#define MODBUS_MAX_READ_BITS                2000
#define MODBUS_MAX_WRITE_BITS               1968
#define MODBUS_MAX_READ_REGISTERS           125
#define MODBUS_MAX_WRITE_REGISTERS          123
#define MODBUS_MAX_WR_WRITE_REGISTERS       121
#define MODBUS_MAX_WR_READ_REGISTERS        125

/*************************
 * Function Declarations *
 *************************/

int neuronspi_open (struct inode *, struct file *);
int neuronspi_release (struct inode *, struct file *);
ssize_t neuronspi_read (struct file *, char *, size_t, loff_t *);
ssize_t neuronspi_write (struct file *, const char *, size_t, loff_t *);
s32 char_register_driver(void);
s32 char_unregister_driver(void);
irqreturn_t neuronspi_spi_irq(s32 irq, void *dev_id);
s32 neuronspi_spi_probe(struct spi_device *spi);
s32 neuronspi_spi_remove(struct spi_device *spi);
void neuronspi_spi_send_message(struct spi_device *spi_dev, u8 *send_buf, u8 *recv_buf, s32 len, s32 freq, s32 delay, s32 send_header);
s32 neuronspi_spi_uart_write(struct spi_device *spi, u8 *send_buf, u8 length, u8 uart_index);
void neuronspi_spi_uart_read(struct spi_device* spi_dev, u8 *send_buf, u8 *recv_buf, s32 len, u8 uart_index);
void neuronspi_spi_set_irqs(struct spi_device* spi_dev, u16 to);
void neuronspi_spi_led_set_brightness(struct spi_device* spi_dev, enum led_brightness brightness, int id);
void neuronspi_spi_iio_stm_ai_read_voltage(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
void neuronspi_spi_iio_stm_ai_read_current(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
void neuronspi_spi_iio_stm_ao_read_resistance(struct iio_dev *indio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
void neuronspi_spi_iio_stm_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask);
void neuronspi_spi_iio_stm_ao_set_current(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask);
void neuronspi_spi_iio_sec_ai_read_voltage(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
void neuronspi_spi_iio_sec_ai_read_current(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
void neuronspi_spi_iio_sec_ai_read_resistance(struct iio_dev *iio_dev, struct iio_chan_spec const *ch, int *val, int *val2, long mask);
void neuronspi_spi_iio_sec_ao_set_voltage(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask);
int neuronspi_spi_gpio_do_set(struct spi_device* spi_dev, u32 id, int value);
int neuronspi_spi_gpio_ro_set(struct spi_device* spi_dev, u32 id, int value);
int neuronspi_spi_gpio_di_get(struct spi_device* spi_dev, u32 id);
int neuronspi_spi_gpio_di_get(struct spi_device* spi_dev, u32 id);

/***********************
 * Function structures *
 ***********************/

// Host driver struct
extern struct spi_driver neuronspi_spi_driver;
extern struct file_operations file_ops;

static const struct regmap_bus neuronspi_regmap_bus =
{
	.fast_io 					= 1,
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
		.name 					= "Neuronspi Regmap",
		.reg_bits				= 16,
		.reg_stride				= 0,
		.pad_bits				= 0,
		.val_bits				= 16,
		.max_register			= 65535,
		.cache_type				= REGCACHE_RBTREE,
		.use_single_rw			= 0,
		.can_multi_write		= 1,
};

static const struct iio_chan_spec neuronspi_stm_ai_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	},
	{
			.type = IIO_CURRENT,
			.indexed = 1,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	}
};

static const struct iio_chan_spec neuronspi_stm_ao_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 1
	},
	{
			.type = IIO_CURRENT,
			.indexed = 1,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 1
	},
	{
			.type = IIO_RESISTANCE,
			.indexed = 1,
			.channel = 2,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	}
};

static const struct iio_chan_spec neuronspi_sec_ai_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	},
	{
			.type = IIO_CURRENT,
			.indexed = 1,
			.channel = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	},
	{
			.type = IIO_RESISTANCE,
			.indexed = 1,
			.channel = 2,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 0
	}
};

static const struct iio_chan_spec neuronspi_sec_ao_chan_spec[] = {
	{
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
			.output = 1
	}
};

static const struct iio_info neuronspi_stm_ai_info = {
	.read_raw = neuronspi_iio_stm_ai_read_raw,
	.driver_module = THIS_MODULE,
	.attrs = &neuron_stm_ai_group,
};

static const struct iio_info neuronspi_stm_ao_info = {
	.read_raw = neuronspi_iio_stm_ao_read_raw,
	.write_raw = neuronspi_iio_stm_ao_write_raw,
	.driver_module = THIS_MODULE,
	.attrs = &neuron_stm_ao_group,
};

static const struct iio_info neuronspi_sec_ai_info = {
	.read_raw = neuronspi_iio_sec_ai_read_raw,
	.driver_module = THIS_MODULE,
	.attrs = &neuron_sec_ai_group,
};

static const struct iio_info neuronspi_sec_ao_info = {
	.write_raw = neuronspi_iio_sec_ao_write_raw,
	.driver_module = THIS_MODULE,
	.attrs = &neuron_sec_ao_group,
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

static __always_inline size_t neuronspi_spi_compose_single_coil_write(u16 start, u8 **buf_inp, u8 **buf_outp, u8 data)
{
	u16 crc1;
	*buf_outp = kzalloc(6, GFP_KERNEL);
	*buf_inp = kzalloc(6, GFP_KERNEL);
	(*buf_inp)[0] = 0x05;
	(*buf_inp)[1] = data;
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	return 6;
}

static __always_inline size_t neuronspi_spi_compose_single_coil_read(u16 start, u8 **buf_inp, u8 **buf_outp)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(14, GFP_KERNEL);
	*buf_inp = kzalloc(14, GFP_KERNEL);
	(*buf_inp)[0] = 0x01;
	(*buf_inp)[1] = 0x06;
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 6, crc1);
	memcpy(&(*buf_inp)[12], &crc2, 2);
	return 14;
}

static __always_inline size_t neuronspi_spi_compose_multiple_coil_write(u8 number, u16 start, u8 **buf_inp, u8 **buf_outp, u8 *data)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	*buf_inp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	(*buf_inp)[0] = 0x0F;
	(*buf_inp)[1] = 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = number;
	memcpy(&(*buf_inp)[10], data, NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number));
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), crc1);
	memcpy(&(*buf_inp)[10 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number)], &crc2, 2);
	return 12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
}

static __always_inline size_t neuronspi_spi_compose_multiple_coil_read(u8 number, u16 start, u8 **buf_inp, u8 **buf_outp)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	*buf_inp = kzalloc(12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), GFP_KERNEL);
	(*buf_inp)[0] = 0x01;
	(*buf_inp)[1] = 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number), crc1);
	memcpy(&(*buf_inp)[10 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number)], &crc2, 2);
	return 12 + NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(number);
}

static __always_inline size_t neuronspi_spi_compose_single_register_write(u16 start, u8 **buf_inp, u8 **buf_outp, u16 data)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(14, GFP_KERNEL);
	*buf_inp = kzalloc(14, GFP_KERNEL);
	(*buf_inp)[0] = 0x06;
	(*buf_inp)[1] = 0x06;
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	printk(KERN_INFO "NEURONSPI: COMPOSE SINGLE WRITE DATA: %x\n", data);
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = 0x01;
	memcpy(&(*buf_inp)[10], &data, 2);
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 6, crc1);
	memcpy(&(*buf_inp)[12], &crc2, 2);
	return 14;
}

static __always_inline size_t neuronspi_spi_compose_single_register_read(u16 start, u8 **buf_inp, u8 **buf_outp)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(14, GFP_KERNEL);
	*buf_inp = kzalloc(14, GFP_KERNEL);
	(*buf_inp)[0] = 0x03;
	(*buf_inp)[1] = 0x06;
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = 0x01;
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 6, crc1);
	memcpy(&(*buf_inp)[12], &crc2, 2);
	return 14;
}

static __always_inline size_t neuronspi_spi_compose_multiple_register_write(u8 number, u16 start, u8 **buf_inp, u8 **buf_outp, u8 *data)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(12 + (number * 2), GFP_KERNEL);
	*buf_inp = kzalloc(12 + (number * 2), GFP_KERNEL);
	(*buf_inp)[0] = 0x10;
	(*buf_inp)[1] = 4 + (number * 2);
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = number;
	memcpy(&(*buf_inp)[10], data, number * 2);
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + (number * 2), crc1);
	memcpy(&(*buf_inp)[10 + (number * 2)], &crc2, 2);
	return 12 + (number * 2);
}

static __always_inline size_t neuronspi_spi_compose_multiple_register_read(u8 number, u16 start, u8 **buf_inp, u8 **buf_outp)
{
	u16 crc1, crc2;
	*buf_outp = kzalloc(12 + (number * 2), GFP_KERNEL);
	*buf_inp = kzalloc(12 + (number * 2), GFP_KERNEL);
	(*buf_inp)[0] = 0x03;
	(*buf_inp)[1] = 4 + (number * 2);
	(*buf_inp)[2] = start & 0xFF;
	(*buf_inp)[3] = start >> 8;
	crc1 = neuronspi_spi_crc(*buf_inp, 4, 0);
	memcpy(&(*buf_inp)[4], &crc1, 2);
	memcpy(&(*buf_inp)[6], *buf_inp, 4);
	(*buf_inp)[7] = number;
	crc2 = neuronspi_spi_crc(&(*buf_inp)[6], 4 + (number * 2), crc1);
	memcpy(&(*buf_inp)[10 + (number * 2)], &crc2, 2);
	return 12 + (number * 2);
}

#endif /* MODULES_NEURON_SPI_SRC_UNIPI_SPI_H_ */
