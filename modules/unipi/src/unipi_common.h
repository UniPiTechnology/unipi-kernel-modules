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

#define NEURONSPI_SCHED_REQUIRED 1 // Older kernels do not require sched/types to be specifically imported
#if NEURONSPI_SCHED_REQUIRED > 0
	#include <uapi/linux/sched/types.h>
#endif
#define NEURONSPI_MAJOR_VERSIONSTRING "Version 1.24:2019:08:13"

#define NEURONSPI_MAX_DEVS				3
#define NEURONSPI_MAX_UART				16
#define NEURONSPI_BUFFER_MAX			1152
#define NEURONSPI_HEADER_LENGTH 		10
#define NEURONSPI_FIRST_MESSAGE_LENGTH	6
#define NEURONSPI_FIRST_MESSAGE_ALLOC	8
#define NEURONSPI_EDGE_DELAY			10
#define NEURONSPI_B_PER_WORD 			8
#define NEURONSPI_DEFAULT_FREQ			600000
#define NEURONSPI_COMMON_FREQ			12000000

#if defined(CONFIG_ARM64)
/* on NanoPi there are only 12MHz and 6MHz available, not in between */
   #define NEURONSPI_SLOWER_FREQ			6000000
#else
   #define NEURONSPI_SLOWER_FREQ			7500000
#endif

#define NEURONSPI_MAX_TX				60
//#define NEURONSPI_MAX_TX				256
#define NEURONSPI_MAX_BAUD				115200
#define NEURONSPI_FIFO_SIZE				256
#define NEURONSPI_FIFO_MIN_CONTINUOUS	50
#define NEURONSPI_DETAILED_DEBUG		0
#define NEURONSPI_LAST_TRANSFER_DELAY	40
#define MAX_RX_QUEUE_LEN                16

#define NEURON_DEVICE_NAME 				"unipispi"
#define NEURON_DEVICE_CLASS 			"modbus_spi"
#define NEURON_DRIVER_NAME				"UNIPISPI"
#define PORT_NEURONSPI					184

#define STRICT_RESERVING
#define NEURONSPI_ALWAYS_EXPORT

#define NEURONSPI_GET_COIL_READ_PHASE2_BYTE_LENGTH(X)	((((X) + 15) >> 4) << 1)

/********************
 * Module Constants *
 ********************/

#define NEURONSPI_NO_INTERRUPT_MODELS_LEN 				3
static const u16 NEURONSPI_NO_INTERRUPT_MODELS[NEURONSPI_NO_INTERRUPT_MODELS_LEN] = {
		0xb10, 0xc10, 0xf10
};

/*******************
 * Data Structures *
 *******************/

struct neuronspi_op_buffer
{
    u8  first_message[8];
    u8  *second_message;
};


#define CB_WRITESTRING 1
#define CB_GETTXFIFO  2
#define START_TX       3

struct neuronspi_port
{
	struct uart_port			port;
	u8							dev_index;  // index into global array neuronspi_s_dev 
	u8							dev_port;   // index of port on neuronspi device
    struct neuronspi_driver_data  *n_spi;     // shorthand to n_spi 
    
    u8                          rx_remain;
    
	struct kthread_work			tx_work;
    u16                         tx_fifo_reg;  // register in neuronspi device modbus map to read internal tx fifo length
                                              // 0 if undefined
    u16                         tx_fifo_len;  // estimates char count in neuron internal tx fifo
	struct hrtimer				tx_timer;

	s32							baud;
    s64                         one_char_nsec;

    spinlock_t                  rx_in_progress_lock;
    int                         rx_in_progress;
    struct neuronspi_op_buffer  rx_send_buf;
    struct neuronspi_op_buffer  rx_recv_buf;
    u8                          rx_send_msg[NEURONSPI_FIFO_SIZE+4];
    u8                          rx_recv_msg[NEURONSPI_FIFO_SIZE+4];

    spinlock_t                  txop_lock;
    int                         pending_txop;
    struct neuronspi_op_buffer  tx_send_buf;
    struct neuronspi_op_buffer  tx_recv_buf;
    u8                          tx_send_msg[NEURONSPI_FIFO_SIZE+4];
    u8                          tx_recv_msg[NEURONSPI_FIFO_SIZE+4];
};

struct neuronspi_uart_data
{
	struct neuronspi_port			*p;             // array p[p_count]
	u8								p_count;
};

// Instantiated once per SPI device
struct neuronspi_driver_data
{
	s32 neuron_index;
	u8 reserved_device;

	struct kthread_worker   *primary_worker;
	struct kthread_work		irq_work;
    struct hrtimer			poll_timer;
    int                     poll_enabled;

	struct platform_device *board_device;

	struct regmap *reg_map;
	struct mutex device_lock;
	struct neuronspi_board_features *features;
	struct neuronspi_board_regstart_table *regstart_table;
	struct spinlock sysfs_regmap_lock;

	u8  combination_id;
	u16 firmware_version;
	u8  no_irq;
	u32 ideal_frequency;
	u8 uart_count_to_probe;
	int uart_count;
	int uart_pindex;

	u16 sysfs_regmap_target;
	u16 sysfs_register_target;
	u16 sysfs_counter_target;
    
    //struct neuronspi_op_buffer  idle_recv_buf;
    cycles_t last_cs_cycles;
};


/*
struct neuronspi_direct_acc
{
	void __iomem		*vaddr;
	u32					size;
};
*/

/*********************
 * Data Declarations *
 *********************/

extern struct spi_device    *neuronspi_s_dev[NEURONSPI_MAX_DEVS];
extern struct task_struct   *neuronspi_invalidate_thread;

extern int                  neuronspi_model_id;


#endif /* MODULES_NEURON_SPI_SRC_UNIPI_COMMON_H_ */
