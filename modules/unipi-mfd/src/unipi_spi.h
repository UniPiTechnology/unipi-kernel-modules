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
#include "unipi_channel.h"

enum UNIPI_SPI_OP
{
	UNIPI_SPI_OP_READBIT   = 0x01,
	UNIPI_SPI_OP_READREG   = 0x04,
	UNIPI_SPI_OP_WRITEBIT  = 0x05,
	UNIPI_SPI_OP_WRITEREG  = 0x06,
	UNIPI_SPI_OP_WRITEBITS = 0x0F,

	UNIPI_SPI_OP_WRITECHAR = 0x41,
	UNIPI_SPI_OP_WRITESTR  = 0x64,
	UNIPI_SPI_OP_READSTR   = 0x65,
	UNIPI_SPI_OP_IDLE      = 0xFA,

	UNIPI_SPI_OP2_READSTR  = 0x60,
	UNIPI_SPI_OP2_WRITESTR = 0x70,
	UNIPI_SPI_OP2_IDLE     = 0xAF,
};


#define UNIPI_SPI_MESSAGE_HEADER_LENGTH	4
#define UNIPI_SPI_FIRST_MESSAGE_LENGTH	6   /*including CRC */
#define UNIPI_SPI_FIRST_MESSAGE_ALLOC	8
#define UNIPI_SPI_CRC_LENGTH			2

#define UNIPI_SPI2_MESSAGE_LEN 16
#define UNIPI_SPI2_MESSAGE_ALLOC UNIPI_SPI2_MESSAGE_LEN + 4


#define UNIPI_SPI_EDGE_DELAY			10
#define UNIPI_SPI_B_PER_WORD 			8
#define UNIPI_SPI_INTER_FRAME_US		40
#define UNIPI_SPI_INTER_FRAME_NS		(UNIPI_SPI_INTER_FRAME_US*1000)

#define UNIPI_SPI_PROBE_FREQ			1000000
#define UNIPI_SPI_SLOWER_FREQ			6000000


/* On some spi controller (BCM) don't work correctly long transfers 
 * this number defines maximum length of one chunk */
#if defined(CONFIG_ARM64)
#define NEURONSPI_MAX_TX				2048
#else
#define NEURONSPI_MAX_TX				60
#endif


//#define UNIPI_MODBUS_HEADER_SIZE   4
//#define UNIPI_MODBUS_REGISTER_SIZE 2
//#define UNIPI_MODBUS_TAIL_SIZE     4

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
	struct unipi_channel channel; // must by the first member!
	int frequency;
	cycles_t last_cs_cycles;
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
	int enable_v2;
	int allow_v2;
	int probe_mode;
};

struct unipi_spi_devstatus {
	enum UNIPI_SPI_OP opcode;
	unsigned int interrupt;
	uint8_t port;
	int remain;
	uint8_t ch;
};

struct unipi_spi_reply {
	enum UNIPI_SPI_OP opcode;
	int ret;
//	unsigned int reg;
//	uint8_t *data;
};

struct unipi_spi_context
{
	struct spi_message message;
	void * operation_callback_data;
	OperationCallback operation_callback;
	int (*parse_frame)(struct unipi_spi_context*, struct unipi_spi_devstatus*, struct unipi_spi_reply*);
	int interframe_nsec;
	int simple_read;
	int simple_len;
	u8* data;
	int len;
	u16 packet_crc;
	union {
		u8 tx_header[UNIPI_SPI2_MESSAGE_ALLOC];
		struct {
			u8 tx_header1[UNIPI_SPI_FIRST_MESSAGE_ALLOC];
			u8 tx_header2[UNIPI_SPI_MESSAGE_HEADER_LENGTH+2*2+4];
		};
	};
	union {
		u8 rx_header[UNIPI_SPI2_MESSAGE_ALLOC];
		struct {
			u8 rx_header1[UNIPI_SPI_FIRST_MESSAGE_ALLOC];
			u8 rx_header2[UNIPI_SPI_MESSAGE_HEADER_LENGTH+2*2+4];
		};
	};
};

/* from unipi_spi.c */
u16 unipi_spi_crc_set(u8* data, int len, u16 start);
int unipi_spi_exec_context(struct spi_device* spi_dev, struct unipi_spi_context *context,
                           void* cb_data, OperationCallback cb_function);
void unipi_spi_populated(void * self);
int unipi_spi_set_v1(struct spi_device* spi_dev);
int unipi_spi_try_v2(struct spi_device* spi_dev);


/* from unipi_firmware.c */
int  unipi_spi_firmware_op(void *proto_self, u8* send_buf, u8* recv_buf, s32 len, int firmware_cookie);
void unipi_spi_unlock(void *proto_self);
int  unipi_spi_lock(void *proto_self);




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
