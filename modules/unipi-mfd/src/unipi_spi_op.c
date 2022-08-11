/*
 * Unipi PLC spi device driver - Copyright (C) 2018 Unipi Technology
 * 
 * Author: Tomas Knot <tomasknot@gmail.com>
 * Author: Miroslav Ondra <ondra@faster.cz>
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
#include <linux/completion.h>
#include <linux/cpufreq.h>
#include <linux/version.h>
#include <uapi/linux/sched/types.h>

#include "unipi_common.h"
#include "unipi_modbus.h"
#include "unipi_spi.h"
#include "unipi_spi_crc.h"


/****************************************************************/

#define roundup_block(count, blocksize) (1+((count)-1)/(blocksize))

/* bottom part of all async op */
int unipi_spi_check_message(struct spi_device* spi, struct unipi_spi_context* context)
{
	u16 recv_crc;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi);

	//unipi_spi_trace_1(spi, "Read (op1) %8ph\n", context->rx_header);
	unipi_spi_trace_1(spi, "Read (op1) %16ph\n", context->rx_header);
	// check crc on header
	recv_crc = unipi_spi_crc(context->rx_header, UNIPI_SPI_MESSAGE_HEADER_LENGTH, 0);
	context->packet_crc = *((u16*)(context->rx_header+UNIPI_SPI_MESSAGE_HEADER_LENGTH));
	if (recv_crc != context->packet_crc) {
		unipi_spi_error(spi, "Incorrect CRC1 (Received: %04x Calculated: %04x)\n", context->packet_crc, recv_crc);
		unipi_spi_error(spi, "  part1 %4phC\n", context->rx_header);
		n_spi->stat.errors_crc1++;
		context->rx_header2[0] = 0;
		context->rx_header[0] = 0;
		return -EIO;
	}

	if (context->len > 0) {
		if (context->simple_read) {
			// Check second message crc
			recv_crc = unipi_spi_crc(context->rx_header2, context->len, recv_crc);
			context->packet_crc = context->rx_header2[context->len] | (context->rx_header2[context->len+1] << 8);
			unipi_spi_trace_1(spi, "Read - %d: %10ph", context->len, context->rx_header2);
		} else if (context->data) {
			recv_crc = unipi_spi_crc(context->rx_header2, UNIPI_SPI_MESSAGE_HEADER_LENGTH, recv_crc);
			recv_crc = unipi_spi_crc(context->data, context->len, recv_crc);
			context->packet_crc = context->data[context->len] | (context->data[context->len+1] << 8);
			unipi_spi_trace_1(spi, "Read - %d: %4ph", context->len, context->rx_header2);
		} else {
			return 0;
		}
		if (recv_crc != context->packet_crc) {
			unipi_spi_error(spi, "Incorrect CRC2: %04x COMPUTED: %04x\n", context->packet_crc, recv_crc);
			unipi_spi_error(spi, "  len=%d, tx=%4phC rx=%4phC\n", context->len, context->tx_header,context->rx_header2);
			n_spi->stat.errors_crc2++;
			//!!! Do Not return error, set only error on recv_buf
			context->rx_header2[0] = 0;
		}
	}
	return 0;
}

static int unipi_spi_parse_frame_common(struct unipi_spi_context* context, struct unipi_spi_devstatus *devstatus, struct unipi_spi_reply *reply)
{
	u16 recv_crc;
	//struct unipi_spi_device *n_spi;
	//struct spi_device* spi = context->message.spi;

	unipi_spi_trace_1(context->message.spi, "RESP(rqop=%02x)   %6phC\n", context->tx_header[0], context->rx_header);
	recv_crc = unipi_spi_crc(context->rx_header, UNIPI_SPI_MESSAGE_HEADER_LENGTH, 0);
	context->packet_crc = *((u16*)(context->rx_header+UNIPI_SPI_MESSAGE_HEADER_LENGTH));
	if (recv_crc != context->packet_crc) {
		return -EIO;
	}

	devstatus->opcode = context->rx_header[0];
	devstatus->interrupt = context->rx_header[1] & (~0x80);  // mask out v2_enabled flag
	if (devstatus->opcode >= UNIPI_SPI_OP_WRITECHAR && devstatus->opcode < (UNIPI_SPI_OP_WRITECHAR+4)) {
		devstatus->remain = (context->rx_header[2]==0  ? 256 : context->rx_header[2]) - 1;
		devstatus->port = devstatus->opcode - UNIPI_SPI_OP_WRITECHAR;
		devstatus->opcode = UNIPI_SPI_OP_WRITECHAR;
		devstatus->ch = context->rx_header[3];
	}

	if ((devstatus->opcode==UNIPI_SPI_OP_IDLE) && (context->rx_header[1] & 0x80))
		unipi_spi_try_v2(context->message.spi);

	reply->opcode = context->tx_header[0];
	reply->ret = 0;
	return 0;
}

static int unipi_spi_parse_frame_simple(struct unipi_spi_context* context, struct unipi_spi_devstatus *devstatus, struct unipi_spi_reply *reply)
{
	u16 recv_crc;
	int ret = unipi_spi_parse_frame_common(context, devstatus, reply);
	if (ret != 0) return ret;

	// Check second message crc
	recv_crc = unipi_spi_crc(context->rx_header2, context->len, context->packet_crc);
	context->packet_crc = context->rx_header2[context->len] | (context->rx_header2[context->len+1] << 8);
	//unipi_spi_trace_1(spi, "Read - %d: %10ph", context->len, context->rx_header2);

	if (recv_crc != context->packet_crc) {
		reply->ret = -ENXIO;
		return 0;
	}
	if (reply->opcode != context->rx_header2[0]) {
		reply->ret = -EINVAL;
		return 0;
	}
	reply->ret = context->rx_header2[1];
	if ((reply->ret > 0) && (context->simple_read) && (context->data))
		memmove(context->data, context->rx_header2+UNIPI_SPI_MESSAGE_HEADER_LENGTH, context->simple_len);
	return 0;
}

static int unipi_spi_parse_frame_read(struct unipi_spi_context* context, struct unipi_spi_devstatus *devstatus, struct unipi_spi_reply *reply)
{
	u16 recv_crc;
	int ret = unipi_spi_parse_frame_common(context, devstatus, reply);
	if (ret != 0) return ret;

	// Check second message crc
	recv_crc = unipi_spi_crc(context->rx_header2, UNIPI_SPI_MESSAGE_HEADER_LENGTH, context->packet_crc);
	recv_crc = unipi_spi_crc(context->data, context->len, recv_crc);
	context->packet_crc = context->data[context->len] | (context->data[context->len+1] << 8);
	//printk("Read - %d: %10ph ccrc=%04x rcrc=%04x\n", context->len, context->rx_header2, recv_crc, context->packet_crc);

	if (recv_crc != context->packet_crc) {
		reply->ret = -ENXIO;
		return 0;
	}
	if (reply->opcode != context->rx_header2[0]) {
		reply->ret = -EINVAL;
		return 0;
	}
	if (reply->opcode == UNIPI_SPI_OP_READSTR) {
		/* combine len + remain */
		reply->ret = context->rx_header2[1] | (context->rx_header2[3] << 8);
	} else {
		reply->ret = context->rx_header2[1];
	}
	return 0;
}

static int unipi_spi_parse_frame_write(struct unipi_spi_context* context, struct unipi_spi_devstatus *devstatus, struct unipi_spi_reply *reply)
{
	int ret = unipi_spi_parse_frame_common(context, devstatus, reply);
	if (ret != 0) return ret;
	if (reply->opcode != context->rx_header2[0]) {
		reply->ret = -EINVAL;
		return 0;
	}
	reply->ret = context->rx_header2[1];
	return 0;
}

/* upper part of all async opertations
 * - create context with space for 2+add_trans transactions
 * - setup data of first part */
struct unipi_spi_context *unipi_spi_alloc_context(struct spi_device* spi_dev, enum UNIPI_SPI_OP op, 
                                                   unsigned int len, unsigned int reg, int add_trans)
{
	int trans_count, freq;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;
	struct unipi_spi_device *n_spi = spi_get_drvdata(spi_dev);

	trans_count = 2 + add_trans;
	context = kzalloc(sizeof(struct unipi_spi_context) + trans_count * sizeof(struct spi_transfer), GFP_ATOMIC);
	if (! context) {
		return NULL;
	}
	freq = n_spi->frequency;
	context->parse_frame = unipi_spi_parse_frame_common;
	context->tx_header[0] = op;
	context->tx_header[1] = len;
	context->tx_header[2] = lo(reg);
	context->tx_header[3] = hi(reg);
	context->packet_crc = unipi_spi_crc_set(context->tx_header, UNIPI_SPI_MESSAGE_HEADER_LENGTH, 0);

	s_trans = (struct spi_transfer *)(context + 1);
	spi_message_init_with_transfers(&context->message, s_trans, trans_count);

	s_trans[0].delay.value = UNIPI_SPI_EDGE_DELAY;
	s_trans[0].delay.unit = SPI_DELAY_UNIT_USECS;
	s_trans[0].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[0].speed_hz = freq;
	s_trans[1].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[1].speed_hz = freq;
	s_trans[1].len = UNIPI_SPI_FIRST_MESSAGE_LENGTH;
	s_trans[1].tx_buf = context->tx_header;
	s_trans[1].rx_buf = context->rx_header;
	unipi_spi_trace_1(spi_dev, "Write(op1) %6phC\n", context->tx_header);
 
	context->message.context = context;
	context->message.spi = spi_dev;
	context->interframe_nsec = UNIPI_SPI_INTER_FRAME_NS;
	return context;
}

/**************************************************************
 * 
 * Asynchronous operations
 * 
 * ***********************************************************/

/*  Async op for WRITEREGS, WRITEBITS using own buffer 
 * 		data buffer can be unstable (on-stack)
 * 		regcount = 1..2   pro WRITEREGS
 *		         = 1..32  pro WRITEBITS
 */ 
int unipi_spi_write_simple(struct spi_device* spi_dev, enum UNIPI_SPI_OP op,
                           unsigned int reg, unsigned int count, u8* data,
                           void* cb_data, OperationCallback cb_function)
{
	int len;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;

	len  = ((op==UNIPI_SPI_OP_WRITEREG)? count : roundup_block(count, 16)) * 2;
	len += UNIPI_MODBUS_HEADER_SIZE;
	context = unipi_spi_alloc_context(spi_dev, op, len, reg, 1);
	if (context == NULL) return -ENOMEM;

	//unipi_spi_trace_1(spi_dev, "A %6phC len=%d\n", context->tx_header, len);

	context->tx_header2[0] = op;
	context->tx_header2[1] = count;
	context->tx_header2[2] = lo(reg);
	context->tx_header2[3] = hi(reg);
	memmove(context->tx_header2+UNIPI_MODBUS_HEADER_SIZE, data,\
	        (op==UNIPI_SPI_OP_WRITEREG)? count*2 : roundup_block(count, 8));
	context->packet_crc = unipi_spi_crc_set(context->tx_header2, len, context->packet_crc);

	unipi_spi_trace_1(spi_dev, "Write(op2) %6phC\n", context->tx_header2);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	s_trans[2].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[2].speed_hz = s_trans[1].speed_hz;
	s_trans[2].len = len+UNIPI_SPI_CRC_LENGTH;
	s_trans[2].tx_buf = context->tx_header2;
	s_trans[2].rx_buf = context->rx_header2;

	context->len = len;
	context->parse_frame = unipi_spi_parse_frame_write;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

/*  Async op for READREG, READBITS using own data buffer 
 * 		regcount = 1..2   pro READREG
 *		         = 1..32  pro READBIT
 */ 
int unipi_spi_read_simple(struct spi_device* spi_dev, enum UNIPI_SPI_OP op,
                           unsigned int reg, unsigned int count, u8 *data,
                           void* cb_data, OperationCallback cb_function)
{
	int len;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;

	len  = ((op==UNIPI_SPI_OP_READREG)? count : roundup_block(count, 16)) * 2;
	len += UNIPI_MODBUS_HEADER_SIZE;
	context = unipi_spi_alloc_context(spi_dev, op, len, reg, 1);
	if (context == NULL) return -ENOMEM;

	/* crc must be calculated due to firmware behavior */
	context->packet_crc = unipi_spi_crc_set(context->tx_header2, len, context->packet_crc);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	s_trans[2].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[2].speed_hz = s_trans[1].speed_hz;
	s_trans[2].len = len+UNIPI_SPI_CRC_LENGTH;
	s_trans[2].rx_buf = context->rx_header2;
	s_trans[2].tx_buf = context->tx_header2; /* firmware misbehave else can be NULL*/

	context->len = len;
	context->simple_read = 1;
	context->simple_len = (op==UNIPI_SPI_OP_READREG) ? count*2 : roundup_block(count, 8);
	context->data = data;
	context->parse_frame = unipi_spi_parse_frame_simple;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}


/*  Async op for READREG */
int unipi_spi_read_regs_async(void *proto_self, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	int len, data_trans, remain, i;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;

	if (count<=2) {
		return unipi_spi_read_simple(spi_dev, UNIPI_SPI_OP_READREG, reg, count, data,
		                             cb_data, cb_function);
	}
	len = (count * 2);
	data_trans = roundup_block(len+UNIPI_SPI_CRC_LENGTH, NEURONSPI_MAX_TX);
	context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_READREG, len+UNIPI_MODBUS_HEADER_SIZE, reg, 1+data_trans);
	if (context == NULL) return -ENOMEM;

	/* crc must be calculated due to firmware behavior */
	context->packet_crc = unipi_spi_crc(context->tx_header2, UNIPI_MODBUS_HEADER_SIZE, context->packet_crc);
	context->packet_crc = unipi_spi_crc_set(data, len, context->packet_crc);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25 + ((count>27)?(count-27):0);
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	s_trans[2].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[2].speed_hz = s_trans[1].speed_hz;
	s_trans[2].len = UNIPI_MODBUS_HEADER_SIZE;
	s_trans[2].rx_buf = context->rx_header2;
	s_trans[2].tx_buf = context->tx_header2; /* firmware misbehave else can be NULL*/

	remain = len + UNIPI_SPI_CRC_LENGTH;
	for ( i=0; i < data_trans; i++) {
		s_trans[3+i].bits_per_word = UNIPI_SPI_B_PER_WORD;
		s_trans[3+i].speed_hz = s_trans[1].speed_hz;
		s_trans[3+i].rx_buf = data + (NEURONSPI_MAX_TX * i);
		s_trans[3+i].tx_buf = data + (NEURONSPI_MAX_TX * i);  /* firmware misbehave else can be NULL*/
		s_trans[3+i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
		remain -= NEURONSPI_MAX_TX;
	}

	context->len = len;
	context->data = data;
	context->parse_frame = unipi_spi_parse_frame_read;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

/*  Async op for WRITEREG */
int unipi_spi_write_regs_async(void *proto_self, unsigned int reg, unsigned int count, u8* data,
                               void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	int len, remain, i, data_trans;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;

	if (count<=2) {
		return unipi_spi_write_simple(spi_dev, UNIPI_SPI_OP_WRITEREG, reg, count, data,
		                             cb_data, cb_function);
	}
	len = (count * 2);
	data_trans = roundup_block(len+UNIPI_SPI_CRC_LENGTH, NEURONSPI_MAX_TX);
	context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_WRITEREG, len+UNIPI_MODBUS_HEADER_SIZE, reg, 1+data_trans);
	if (context == NULL) return -ENOMEM;

	context->tx_header2[0] = UNIPI_SPI_OP_WRITEREG;
	context->tx_header2[1] = count;
	context->tx_header2[2] = lo(reg);
	context->tx_header2[3] = hi(reg);
	context->packet_crc = unipi_spi_crc(context->tx_header2, UNIPI_MODBUS_HEADER_SIZE, context->packet_crc);
	context->packet_crc = unipi_spi_crc_set(data, len, context->packet_crc);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	s_trans[2].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[2].speed_hz = s_trans[1].speed_hz;
	s_trans[2].len = UNIPI_MODBUS_HEADER_SIZE;
	s_trans[2].tx_buf = context->tx_header2;
	s_trans[2].rx_buf = context->rx_header2;

	remain = len + UNIPI_SPI_CRC_LENGTH;
	for ( i=0; i < data_trans; i++) {
		s_trans[3+i].bits_per_word = UNIPI_SPI_B_PER_WORD;
		s_trans[3+i].speed_hz = s_trans[1].speed_hz;
		s_trans[3+i].tx_buf = data + (NEURONSPI_MAX_TX * i);
		s_trans[3+i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
		remain -= NEURONSPI_MAX_TX;
	}

	context->len = len;
	context->parse_frame = unipi_spi_parse_frame_write;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

int unipi_spi_write_str_async(void* spi, unsigned int port, u8* data, unsigned int count, 
                              void* cb_data, OperationCallback cb_function)
{
	int len, data_trans, i, remain;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;
	struct spi_device* spi_dev = (struct spi_device*) spi;

	if (count == 1) {
		context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_WRITECHAR, data[0], port << 8, 0);
		if (context==NULL)
			return -ENOMEM;
		return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
	}

	len = (count & 1) ? count+1 : count;
	data_trans = roundup_block(len+UNIPI_SPI_CRC_LENGTH, NEURONSPI_MAX_TX);
	context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_WRITESTR, (count & 0xff), port << 8, data_trans);
	if (context == NULL) return -ENOMEM;

	context->packet_crc = unipi_spi_crc_set(data, len, context->packet_crc);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	remain = len + UNIPI_SPI_CRC_LENGTH;
	for ( i=0; i < data_trans; i++) {
		s_trans[2+i].bits_per_word = UNIPI_SPI_B_PER_WORD;
		s_trans[2+i].speed_hz = s_trans[1].speed_hz;
		s_trans[2+i].tx_buf = data + (NEURONSPI_MAX_TX * i );
		s_trans[2+i].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
		remain -= NEURONSPI_MAX_TX;
	}

	context->len = len;
	context->parse_frame = unipi_spi_parse_frame_write;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}

int unipi_spi_read_str_async(void* spi, unsigned int port, u8* data, unsigned int count, 
                             void* cb_data, OperationCallback cb_function)
{
	int len, data_trans, i, remain;
	struct unipi_spi_context *context;
	struct spi_transfer * s_trans;
	struct spi_device* spi_dev = (struct spi_device*) spi;

	len = count;
	if (len < 246) {
		len += 4;
		len = (len & 1 ? len+1 : len);  // transmit_length must be even
	} else {
		len = 250;
	}

	//unipi_spi_trace_1(spi_dev, "Read str async port=%d, len=%d\n", port, len);

	data_trans = roundup_block(len+UNIPI_SPI_CRC_LENGTH, NEURONSPI_MAX_TX);
	context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_READSTR, len+UNIPI_MODBUS_HEADER_SIZE, port <<8, 1+data_trans);
	if (context == NULL) return -ENOMEM;

	/* crc must be calculated due to firmware behavior */
	context->packet_crc = unipi_spi_crc(context->tx_header2, UNIPI_MODBUS_HEADER_SIZE, context->packet_crc);
	context->packet_crc = unipi_spi_crc_set(data, len, context->packet_crc);

	s_trans = (struct spi_transfer *)(context + 1);
	s_trans[1].delay.value = 25;
	s_trans[1].delay.unit = SPI_DELAY_UNIT_USECS;

	s_trans[2].bits_per_word = UNIPI_SPI_B_PER_WORD;
	s_trans[2].speed_hz = s_trans[1].speed_hz;
	s_trans[2].len = UNIPI_MODBUS_HEADER_SIZE;
	s_trans[2].tx_buf = context->tx_header2;
	s_trans[2].rx_buf = context->rx_header2;

	remain = len + UNIPI_SPI_CRC_LENGTH;
	for ( i=0; i < data_trans; i++) {
		s_trans[i+3].bits_per_word = UNIPI_SPI_B_PER_WORD;
		s_trans[i+3].speed_hz = s_trans[1].speed_hz;
		s_trans[3+i].tx_buf = data + (NEURONSPI_MAX_TX * i);  /* firmware misbehave else can be NULL*/
		s_trans[i+3].rx_buf = data + (NEURONSPI_MAX_TX * i);
		s_trans[i+3].len = (remain > NEURONSPI_MAX_TX) ? NEURONSPI_MAX_TX : remain;
		remain -= NEURONSPI_MAX_TX;
	}

	context->len = len;
	context->data = data;
	context->parse_frame = unipi_spi_parse_frame_read;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
}


/*  Async op for WRITEBIT */
int unipi_spi_write_bits_async(void *proto_self, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	struct unipi_spi_context *context;
	if (count==0 || count > 32) return -EINVAL;

	if (count == 1) {
		context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_WRITEBIT, data[0], reg, 0);
		if (context==NULL)
			return -ENOMEM;
		return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
	}
	return unipi_spi_write_simple(spi_dev, UNIPI_SPI_OP_WRITEBITS, reg, count, data,
	                              cb_data, cb_function);
}

/*  Async op for READBIT */
int unipi_spi_read_bits_async(void *proto_self, unsigned int reg, unsigned int count, u8* data,
                              void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) proto_self;
	if (count==0 || count > 32) return -EINVAL;

	return unipi_spi_read_simple(spi_dev, UNIPI_SPI_OP_READBIT, reg, count, data,
	                             cb_data, cb_function);
}


int unipi_spi_idle_op_async(void* self, void* cb_data, OperationCallback cb_function)
{
	struct spi_device* spi_dev = (struct spi_device*) self;
	struct unipi_spi_context *context;
	context = unipi_spi_alloc_context(spi_dev, UNIPI_SPI_OP_IDLE, 0, 0x0e55, 0);
	if (context==NULL)
		return -ENOMEM;
	return unipi_spi_exec_context(spi_dev, context, cb_data, cb_function);
} 





struct unipi_protocol_op spi_op_v1 = {
	.ping_async = unipi_spi_idle_op_async,
	.read_regs_async = unipi_spi_read_regs_async,
	.write_regs_async = unipi_spi_write_regs_async,
	.read_bits_async = unipi_spi_read_bits_async,
	.write_bits_async = unipi_spi_write_bits_async,
	.read_str_async = unipi_spi_read_str_async,
	.write_str_async = unipi_spi_write_str_async,
	.populated = unipi_spi_populated,
	.firmware_sync = unipi_spi_firmware_op,
	.lock_op = unipi_spi_lock,
	.unlock_op = unipi_spi_unlock,
	.max_write_str_len = (128),
};

