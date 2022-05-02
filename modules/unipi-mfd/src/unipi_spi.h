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

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_SPI_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_SPI_H_

/************
 * Includes *
 ************/
#include <linux/version.h>

#include "unipi_common.h"
#include "unipi_mfd.h"

#define UNIPI_SPI_MESSAGE_HEADER_LENGTH	4
#define UNIPI_SPI_FIRST_MESSAGE_LENGTH	6   /*including CRC */
//#define UNIPI_SPI_FIRST_MESSAGE_ALLOC	8
#define UNIPI_SPI_FIRST_MESSAGE_ALLOC	20
#define UNIPI_SPI_CRC_LENGTH			2

#define UNIPI_SPI_EDGE_DELAY			10
#define UNIPI_SPI_B_PER_WORD 			8

#define UNIPI_SPI_PROBE_FREQ			1000000
#define UNIPI_SPI_SLOWER_FREQ			6000000

//#if defined(CONFIG_ARM64)
/* on NanoPi there are only 12MHz and 6MHz available, not in between */
//   #define UNIPI_SPI_SLOWER_FREQ			6000000
//#else
//   #define UNIPI_SPI_SLOWER_FREQ			7500000
//#endif

/* On some spi controller (BCM) don't work correctly long transfers 
 * this number defines maximum length of one chunk */
#if defined(CONFIG_ARM64)
#define NEURONSPI_MAX_TX				2048
#else
#define NEURONSPI_MAX_TX				60
#endif

enum UNIPI_MODBUS_OP 
{
	UNIPI_MODBUS_OP_READBIT   = 0x01,
	UNIPI_MODBUS_OP_READREG   = 0x04,
	UNIPI_MODBUS_OP_WRITEBIT  = 0x05,
	UNIPI_MODBUS_OP_WRITEREG  = 0x06,
	UNIPI_MODBUS_OP_WRITEBITS = 0x0F,

	UNIPI_MODBUS_OP_WRITECHAR = 0x41,
	UNIPI_MODBUS_OP_WRITESTR  = 0x64,
	UNIPI_MODBUS_OP_READSTR   = 0x65,
	UNIPI_MODBUS_OP_IDLE      = 0xFA
};


#define UNIPI_MODBUS_HEADER_SIZE   4
#define UNIPI_MODBUS_REGISTER_SIZE 2
#define UNIPI_MODBUS_TAIL_SIZE     4

/**
 * struct unipi_spi_statistics - statistics for transfers
 * @lock:          lock protecting this structure
 *
 * @messages:      number of spi-messages handled
 * @errors:        number of errors during spi sending
 * @bytes:         number of bytes transferred to/from device
 * @messages_prio  number of priority rx messages
 * @errors_crc1    number of crc errors in const part of message
 * @errors_crc2    number of crc errors in data part of message
 * @errors_opcode1 number of unknown priority opcode
 */

struct unipi_spi_statistics {
	spinlock_t			lock; /* lock for the whole structure */

	unsigned long		messages;
	unsigned long		errors_tx;
	unsigned long long	bytes;

	unsigned long		messages_prio;
	unsigned long		errors_crc1;
	unsigned long		errors_crc2;
	unsigned long		errors_opcode1;
};

struct unipi_spi_device 
{
	struct unipi_mfd_device mfd; // must by the first member!
	int frequency;
	cycles_t last_cs_cycles;
	struct device *chrdev;
	spinlock_t firmware_lock;   // protect access to firmware_in_progress
	int firmware_in_progress;
	struct unipi_spi_statistics stat;
	spinlock_t busy_lock;       // protect access to busy and queue
	int busy;
	struct list_head queue;     // queue contains messages to be transferred
	                            // after sending current message and inactive_delay (not busy)
	struct hrtimer frame_timer;
	struct spi_device *spi_dev; // required by timer
	int hmode;
};


struct regmap *devm_regmap_init_unipi_regs(struct spi_device *spi,
					const struct regmap_config *config);

struct regmap *devm_regmap_init_unipi_coils(struct spi_device *spi,
					const struct regmap_config *config);

int unipi_reg_read_async(void *context, unsigned int reg,
					void* cb_data, OperationCallback callback);

u16 unipi_spi_crc_set(u8* data, int len, u16 start);


int unipi_spi_read_str_async(void* spi_dev, unsigned int port, u8* data, unsigned int count, 
                             void* cb_data, OperationCallback cb_function);

int unipi_spi_write_str_async(void* spi_dev, unsigned int port, u8* data, unsigned int count, 
                              void* cb_data, OperationCallback cb_function);

int unipi_spi_write_regs_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function);

int unipi_spi_read_regs_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function);

int unipi_spi1_read_regs_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function);

int unipi_spi2_read_simple(struct spi_device* spi_dev, enum UNIPI_MODBUS_OP op,
                           unsigned int reg, unsigned int count, u8 *data,
                           void* cb_data, OperationCallback cb_function);

int unipi_spi_write_bits_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function);

int unipi_spi_read_bits_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function);

int unipi_spi_read_bits_async(struct spi_device* spi_dev, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function);

int unipi_spi_idle_op_async(void *self, void* cb_data, OperationCallback cb_function);


int unipi_spi_regs_op(struct spi_device* spi_dev, enum UNIPI_MODBUS_OP op, unsigned int reg, unsigned int count, u8* data);
int unipi_spi_idle_op(struct spi_device* spi_dev);
enum hrtimer_restart unipi_spi_timer_func(struct hrtimer *timer);


/***************
 * Definitions *
 ***************/


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
/*
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
*/
/***********************
 * Function structures *
 ***********************/

// Host driver struct
/*extern struct spi_driver neuronspi_spi_driver;

// These defines need to be at the end
#define to_neuronspi_uart_data(p,e)  ((container_of((p), struct neuronspi_uart_data, e)))
#define to_led_driver(p,e)	((container_of((p), struct neuronspi_led_driver, e)))
#define to_uart_port(p,e)	((container_of((p), struct uart_port, e)))
*/

int unipi_spi_send_op(struct spi_device* spi_dev, u8* send_buf, u8* recv_buf, s32 len);
int unipi_spi_async_op(struct spi_device* spi_dev, u8* send_buf, u8* recv_buf, s32 len, 
                       void* cb_data, OperationCallback cb_function);
int unipi_spi_firmware_op(struct spi_device* spi_dev, u8* send_buf, u8* recv_buf, s32 len, int firmware_cookie);

void unipi_spi_unlock(struct spi_device* spi_dev);
int unipi_spi_lock(struct spi_device* spi_dev);

/*********************
 * In-line Functions *
 *********************/
/*
static __always_inline int neuronspi_frequency_by_model(u16 model)
{
    int i;
    for (i = 0; i < NEURONSPI_FREQUENCY_LEN; i++) {
        if ((NEURONSPI_FREQUENCY_MAP[i].mask & model) == NEURONSPI_FREQUENCY_MAP[i].model) 
			return NEURONSPI_FREQUENCY_MAP[i].frequency;
    }
	return NEURONSPI_COMMON_FREQ;

//    for (i = 0; i < NEURONSPI_SLOWER_MODELS_LEN; i++) {
//        if (NEURONSPI_SLOWER_MODELS[i] == model) return 1;
//    }
    return 0;

}

static __always_inline int neuronspi_is_noirq_model(u16 model)
{
    int i;
    for (i = 0; i < NEURONSPI_NO_INTERRUPT_MODELS_LEN; i++) {
        if (NEURONSPI_NO_INTERRUPT_MODELS[i] == model) return 1;
    }
    return 0;
}
*/

/* using trace_printk or printk ???*/
#if UNIPI_SPI_DETAILED_DEBUG > 1
# define unipi_spi_trace_1(spi, f, args...)	dev_info(&spi->dev, f, ##args)
#else
# define unipi_spi_trace_1(spi, f, args...) {}
#endif

#if UNIPI_SPI_DETAILED_DEBUG > 0
# define unipi_spi_trace(spi, f, args...)	dev_info(&spi->dev, f, ##args)
#else
# define unipi_spi_trace(spi, f, args...) {}
#endif

#define unipi_spi_error(spi, f, args...)	dev_err(&spi->dev, f, ##args)

#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_SPI_H_ */
